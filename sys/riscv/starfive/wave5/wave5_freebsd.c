/*-
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * Copyright (c) 2026 Tej <itej89@github.com>
 *
 * StarFive JH7110 video decoder (Chips&Media Wave511) -- FreeBSD attachment.
 *
 * This file replaces the vendor driver's wave5-vpu.c (platform driver) and
 * wave5-vpu-dec.c (V4L2 m2m glue). The hardware half of the vendor driver --
 * wave5-hw.c, wave5-vpuapi.c and wave5-vdi.c -- is ported nearly verbatim
 * alongside; see wave5_osal.h for why it is shimmed rather than run on
 * LinuxKPI.
 *
 * What the hardware is: a Wave511 decoding core at 0x130a0000, IRQ 13, with
 * its own VCPU that runs a firmware blob we upload at attach. Measured on the
 * Debian board with the vendor driver, it decodes 1080p30 H.264 at 178 fps,
 * so the ceiling here is the driver, not the silicon.
 *
 * Bring-up order matters and is not obvious from the DT alone:
 *   1. power domain PD_VDEC on   (nothing responds before this)
 *   2. all six clocks enabled
 *   3. all five resets deasserted
 *   4. only then does the product-code register read as anything but zero
 * Getting 0x00000000 from VPU_PRODUCT_CODE_REGISTER means a step above was
 * skipped, not that the core is broken.
 */

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/bus.h>
#include <sys/kernel.h>
#include <sys/module.h>
#include <sys/malloc.h>
#include <sys/rman.h>
#include <sys/linker.h>
#include <sys/firmware.h>
#include <sys/sysctl.h>

#include <machine/bus.h>

#include <vm/vm.h>
#include <vm/vm_param.h>
#include <vm/pmap.h>

#include <dev/ofw/openfirm.h>
#include <dev/ofw/ofw_bus.h>
#include <dev/ofw/ofw_bus_subr.h>

#include <dev/clk/clk.h>
#include <dev/hwreset/hwreset.h>
#include <dev/pwrdom/pwrdom.h>

#include "wave5_osal.h"
#include "wave5-vpu.h"
#include "wave5-vpuapi.h"
#include "wave5-regdefine.h"
#include "wave5.h"	/* wave5_vpu_get_product_id, wave5_vpu_clear_interrupt */

MALLOC_DEFINE(M_WAVE5, "wave5", "StarFive Wave5 VPU");

int wave5_debug = 0;
SYSCTL_NODE(_hw, OID_AUTO, wave5, CTLFLAG_RD | CTLFLAG_MPSAFE, 0,
    "StarFive Wave5 video codec");
SYSCTL_INT(_hw_wave5, OID_AUTO, debug, CTLFLAG_RWTUN, &wave5_debug, 0,
    "enable verbose decoder logging");

/*
 * The decoder firmware. firmware(9) resolves this name to
 * /boot/firmware/wave511_dec_fw.bin (same mechanism the GPU DDK uses). The
 * blob ships with StarFive's Debian image; it is 952768 bytes there.
 */
#define	WAVE5_DEC_FW_NAME	"wave511_dec_fw.bin"

#define	WAVE5_MAX_CLKS		8
#define	WAVE5_MAX_RESETS	8

struct wave5_softc {
	/*
	 * vpu_device must be reachable from struct device's drvdata, because
	 * the ported files do dev_get_drvdata() to find it. Keeping both here
	 * and pointing them at each other in attach avoids a second
	 * allocation.
	 */
	struct vpu_device	vdev;
	struct device		dev;

	device_t		bsddev;
	struct resource		*mem_res;
	struct resource		*irq_res;
	void			*irq_cookie;
	int			mem_rid;
	int			irq_rid;

	clk_t			clks[WAVE5_MAX_CLKS];
	int			nclks;
	hwreset_t		resets[WAVE5_MAX_RESETS];
	int			nresets;
	pwrdom_t		pwrdom;

	const struct firmware	*fw;
	uint32_t		fw_revision;
	uint32_t		product_id;
	bool			fw_loaded;
};

static struct ofw_compat_data compat_data[] = {
	{ "starfive,vdec",	1 },
	{ NULL,			0 }
};

/* ------------------------------------------------------------ interrupt */

/*
 * Mirrors the vendor driver's wave5_vpu_handle_irq(). The firmware reports
 * which instances completed through two registers rather than through the
 * interrupt reason alone, so each instance's bit is tested and cleared by
 * writing it back.
 *
 * Runs as a threaded (filter-free) handler: wave5_vpu_clear_interrupt() and
 * the instance callbacks take the sleepable hw_lock.
 */
static void
wave5_intr(void *arg)
{
	struct wave5_softc *sc = arg;
	struct vpu_device *dev = &sc->vdev;
	struct vpu_instance *inst;
	uint32_t irq_reason, seq_done, cmd_done;

	if (wave5_vdi_read_register(dev, W5_VPU_VPU_INT_STS) == 0)
		return;

	irq_reason = wave5_vdi_read_register(dev, W5_VPU_VINT_REASON);
	wave5_vdi_write_register(dev, W5_VPU_VINT_REASON_CLR, irq_reason);
	wave5_vdi_write_register(dev, W5_VPU_VINT_CLEAR, 0x1);

	list_for_each_entry(inst, &dev->instances, list) {
		seq_done = wave5_vdi_read_register(dev,
		    W5_RET_SEQ_DONE_INSTANCE_INFO);
		cmd_done = wave5_vdi_read_register(dev,
		    W5_RET_QUEUE_CMD_DONE_INST);

		if ((irq_reason & BIT(INT_WAVE5_INIT_SEQ)) ||
		    (irq_reason & BIT(INT_WAVE5_ENC_SET_PARAM))) {
			if (seq_done & BIT(inst->id)) {
				seq_done &= ~BIT(inst->id);
				wave5_vdi_write_register(dev,
				    W5_RET_SEQ_DONE_INSTANCE_INFO, seq_done);
				complete(&inst->irq_done);
			}
		}

		if ((irq_reason & BIT(INT_WAVE5_DEC_PIC)) ||
		    (irq_reason & BIT(INT_WAVE5_ENC_PIC))) {
			if (cmd_done & BIT(inst->id)) {
				cmd_done &= ~BIT(inst->id);
				wave5_vdi_write_register(dev,
				    W5_RET_QUEUE_CMD_DONE_INST, cmd_done);
				if (inst->ops != NULL &&
				    inst->ops->finish_process != NULL)
					inst->ops->finish_process(inst);
			}
		}

		wave5_vpu_clear_interrupt(inst, irq_reason);
	}
}

int
wave5_vpu_wait_interrupt(struct vpu_instance *inst, unsigned int timeout)
{
	int ret;

	ret = wait_for_completion_timeout(&inst->irq_done,
	    msecs_to_jiffies(timeout));
	if (ret == 0)
		return (-ETIMEDOUT);

	reinit_completion(&inst->irq_done);

	return (0);
}

/* --------------------------------------------------------- bring-up bits */

static int
wave5_enable_clocks(struct wave5_softc *sc)
{
	clk_t clk;
	int i, err;

	for (i = 0; i < WAVE5_MAX_CLKS; i++) {
		if (clk_get_by_ofw_index(sc->bsddev, 0, i, &clk) != 0)
			break;
		err = clk_enable(clk);
		if (err != 0) {
			device_printf(sc->bsddev,
			    "could not enable clock %d: %d\n", i, err);
			return (err);
		}
		sc->clks[i] = clk;
	}
	sc->nclks = i;

	if (sc->nclks == 0) {
		device_printf(sc->bsddev, "no clocks in the device tree\n");
		return (ENXIO);
	}

	return (0);
}

static int
wave5_deassert_resets(struct wave5_softc *sc)
{
	hwreset_t rst;
	int i, err;

	for (i = 0; i < WAVE5_MAX_RESETS; i++) {
		if (hwreset_get_by_ofw_idx(sc->bsddev, 0, i, &rst) != 0)
			break;
		err = hwreset_deassert(rst);
		if (err != 0) {
			device_printf(sc->bsddev,
			    "could not deassert reset %d: %d\n", i, err);
			return (err);
		}
		sc->resets[i] = rst;
	}
	sc->nresets = i;

	return (0);
}

/*
 * Upload the firmware and ask the VCPU what it is. A successful version read
 * is the real proof that power, clocks, resets, the register window, the DMA
 * mapping and the cache flushing all work -- it requires the core to have
 * fetched our upload out of DRAM, started executing, and answered a command.
 */
/*
 * Why the VCPU did not start.
 *
 * The core boots by fetching our uploaded image out of DRAM, so a failure
 * here is almost always one of two things: the image never reached DRAM, or
 * the core was never really running. Guessing between them wastes a
 * build-deploy-reboot cycle each time, so this answers it directly.
 *
 * The cache check is the interesting one. The CPU wrote the firmware through
 * a normal cached mapping and wave5_flush_l2_cache() pushed it out; if the
 * bytes read back correctly through the cached mapping but NOT through the
 * SiFive L2 bypass alias, then the flush did not take and the core is reading
 * stale DRAM -- which looks exactly like a dead core from the register side.
 */
static void
wave5_dump_fw_state(struct wave5_softc *sc, const uint8_t *fw)
{
	struct vpu_device *vdev = &sc->vdev;
	const uint32_t *cached;
	int i, bad = 0, bad_alias = 0;

	/*
	 * ONLY these four registers.
	 *
	 * An earlier version of this also read W5_RET_SUCCESS,
	 * W5_RET_FAIL_REASON and W5_VPU_VINT_REASON here. Reading those while
	 * the VCPU is mid-command and BUSY is still set hangs the APB bus hard
	 * enough that the board stops responding entirely and only the
	 * watchdog recovers it. The four below were read safely in that same
	 * state. Do not add to this list without expecting a hang.
	 */
	/*
	 * Check DRAM FIRST, before reading any VPU register: a register read
	 * in this state can hang the bus, and if it does we still want the
	 * memory answer on the console.
	 *
	 * The comparison that matters is through the L2 bypass alias. Reading
	 * back through the ordinary cached mapping would be satisfied by the
	 * very cache we are trying to prove was flushed, and would report
	 * success even if DRAM still held stale data.
	 */
	if (vdev->common_mem.vaddr != NULL) {
		volatile uint32_t *alias;
		uint64_t off = sifive_ccache_uncached_offset();

		cached = (const uint32_t *)vdev->common_mem.vaddr;
		for (i = 0; i < 16; i++)
			if (cached[i] != ((const uint32_t *)fw)[i])
				bad++;

		if (off != 0) {
			alias = pmap_mapdev(
			    (vm_paddr_t)vdev->common_mem.daddr + off, 64);
			if (alias != NULL) {
				for (i = 0; i < 16; i++)
					if (alias[i] !=
					    ((const uint32_t *)fw)[i])
						bad_alias++;
				pmap_unmapdev(__DEVOLATILE(void *, alias), 64);
			}
		}
		device_printf(sc->bsddev,
		    "  firmware at %#jx: cached %d/16 wrong, DRAM %d/16 wrong\n",
		    (uintmax_t)vdev->common_mem.daddr, bad, bad_alias);
	}

	device_printf(sc->bsddev,
	    "  PO_CONF=%#x CUR_PC=%#x BUSY=%#x REMAP_START=%#x\n",
	    wave5_vdi_read_register(vdev, W5_PO_CONF),
	    wave5_vdi_read_register(vdev, W5_VCPU_CUR_PC),
	    wave5_vdi_read_register(vdev, W5_VPU_BUSY_STATUS),
	    wave5_vdi_read_register(vdev, W5_VPU_REMAP_CORE_START));

}

static int
wave5_load_firmware(struct wave5_softc *sc)
{
	int ret;

	sc->fw = firmware_get(WAVE5_DEC_FW_NAME);
	if (sc->fw == NULL) {
		device_printf(sc->bsddev,
		    "firmware \"%s\" not found; place it in /boot/firmware\n",
		    WAVE5_DEC_FW_NAME);
		return (ENOENT);
	}

	device_printf(sc->bsddev, "firmware %s: %zu bytes\n",
	    WAVE5_DEC_FW_NAME, sc->fw->datasize);

	ret = wave5_vpu_init_with_bitcode(&sc->dev,
	    __DECONST(u8 *, sc->fw->data), sc->fw->datasize);
	if (ret != 0) {
		device_printf(sc->bsddev,
		    "firmware init failed: %d\n", ret);
		wave5_dump_fw_state(sc, sc->fw->data);
		return (-ret);
	}

	ret = wave5_vpu_get_version_info(&sc->dev, &sc->fw_revision,
	    &sc->product_id);
	if (ret != 0) {
		device_printf(sc->bsddev,
		    "could not read version info: %d\n", ret);
		return (-ret);
	}

	sc->fw_loaded = true;
	return (0);
}

/* ------------------------------------------------------------- newbus */

static int	wave5_detach(device_t dev);

static int
wave5_probe(device_t dev)
{

	if (!ofw_bus_status_okay(dev))
		return (ENXIO);
	if (ofw_bus_search_compatible(dev, compat_data)->ocd_data == 0)
		return (ENXIO);

	device_set_desc(dev, "StarFive JH7110 Wave511 video decoder");
	return (BUS_PROBE_DEFAULT);
}

static int
wave5_attach(device_t dev)
{
	struct wave5_softc *sc;
	int err, ret;

	sc = device_get_softc(dev);
	sc->bsddev = dev;

	sc->dev.bsddev = dev;
	sc->dev.drvdata = &sc->vdev;
	sc->vdev.dev = &sc->dev;

	sc->mem_rid = 0;
	sc->mem_res = bus_alloc_resource_any(dev, SYS_RES_MEMORY, &sc->mem_rid,
	    RF_ACTIVE);
	if (sc->mem_res == NULL) {
		device_printf(dev, "could not map registers\n");
		return (ENXIO);
	}
	sc->vdev.vdb_register = (void *)rman_get_bushandle(sc->mem_res);

	/*
	 * Buffers must be reachable by 32-bit addresses. This is the parent
	 * tag; wave5_vdi_allocate_dma_memory() creates a correctly sized child
	 * of it per buffer.
	 */
	err = bus_dma_tag_create(bus_get_dma_tag(dev), WAVE5_DMA_ALIGN, 0,
	    WAVE5_DMA_HIGHADDR, BUS_SPACE_MAXADDR, NULL, NULL,
	    BUS_SPACE_MAXSIZE_32BIT, 1, BUS_SPACE_MAXSIZE_32BIT, 0,
	    NULL, NULL, &sc->dev.dmat);
	if (err != 0) {
		device_printf(dev, "could not create DMA tag: %d\n", err);
		goto fail;
	}

	/*
	 * Power first. The register window reads back zeroes while PD_VDEC is
	 * gated, which looks exactly like broken hardware.
	 */
	if (pwrdom_get_by_ofw_idx(dev, 0, 0, &sc->pwrdom) == 0) {
		err = pwrdom_enable(sc->pwrdom);
		if (err != 0) {
			device_printf(dev, "could not power on VDEC: %d\n",
			    err);
			goto fail;
		}
	} else {
		device_printf(dev, "no power domain in the device tree\n");
	}

	err = wave5_enable_clocks(sc);
	if (err != 0)
		goto fail;

	err = wave5_deassert_resets(sc);
	if (err != 0)
		goto fail;

	mutex_init(&sc->vdev.dev_lock);
	mutex_init(&sc->vdev.hw_lock);
	INIT_LIST_HEAD(&sc->vdev.instances);
	ida_init(&sc->vdev.inst_ida);

	sc->vdev.product_code = wave5_vdi_read_register(&sc->vdev,
	    VPU_PRODUCT_CODE_REGISTER);
	if (sc->vdev.product_code == 0 || sc->vdev.product_code == 0xffffffff) {
		device_printf(dev,
		    "product code reads %#x -- core is not responding\n",
		    sc->vdev.product_code);
		err = ENXIO;
		goto fail;
	}

	ret = wave5_vdi_init(&sc->dev);
	if (ret != 0) {
		device_printf(dev, "vdi init failed: %d\n", ret);
		err = -ret;
		goto fail;
	}
	sc->vdev.product = wave5_vpu_get_product_id(&sc->vdev);

	sc->irq_rid = 0;
	sc->irq_res = bus_alloc_resource_any(dev, SYS_RES_IRQ, &sc->irq_rid,
	    RF_ACTIVE);
	if (sc->irq_res == NULL) {
		device_printf(dev, "could not allocate interrupt\n");
		err = ENXIO;
		goto fail;
	}
	err = bus_setup_intr(dev, sc->irq_res, INTR_TYPE_MISC | INTR_MPSAFE,
	    NULL, wave5_intr, sc, &sc->irq_cookie);
	if (err != 0) {
		device_printf(dev, "could not set up interrupt: %d\n", err);
		goto fail;
	}

	err = wave5_load_firmware(sc);
	if (err != 0)
		goto fail;

	device_printf(dev,
	    "Wave511 product %#x id %u firmware revision %u\n",
	    sc->vdev.product_code, sc->product_id, sc->fw_revision);

	return (0);

fail:
	/*
	 * newbus does NOT call detach when attach fails, so the unwind has to
	 * happen here or the register mapping leaks and the next load fails
	 * with "could not map registers" -- which looks like a hardware
	 * problem and is not one. wave5_detach() is written to tolerate a
	 * partially initialised softc.
	 */
	wave5_detach(dev);
	return (err);
}

static int
wave5_detach(device_t dev)
{
	struct wave5_softc *sc;
	int i;

	sc = device_get_softc(dev);

	if (sc->irq_cookie != NULL) {
		bus_teardown_intr(dev, sc->irq_res, sc->irq_cookie);
		sc->irq_cookie = NULL;
	}
	if (sc->irq_res != NULL) {
		bus_release_resource(dev, SYS_RES_IRQ, sc->irq_rid,
		    sc->irq_res);
		sc->irq_res = NULL;
	}

	if (sc->fw_loaded) {
		wave5_vdi_release(&sc->dev);
		sc->fw_loaded = false;
	}
	if (sc->fw != NULL) {
		firmware_put(sc->fw, FIRMWARE_UNLOAD);
		sc->fw = NULL;
	}

	for (i = sc->nresets - 1; i >= 0; i--)
		hwreset_assert(sc->resets[i]);
	sc->nresets = 0;
	for (i = sc->nclks - 1; i >= 0; i--)
		clk_disable(sc->clks[i]);
	sc->nclks = 0;

	if (sc->dev.dmat != NULL) {
		bus_dma_tag_destroy(sc->dev.dmat);
		sc->dev.dmat = NULL;
	}
	if (sc->mem_res != NULL) {
		bus_release_resource(dev, SYS_RES_MEMORY, sc->mem_rid,
		    sc->mem_res);
		sc->mem_res = NULL;
	}

	return (0);
}

static device_method_t wave5_methods[] = {
	DEVMETHOD(device_probe,		wave5_probe),
	DEVMETHOD(device_attach,	wave5_attach),
	DEVMETHOD(device_detach,	wave5_detach),
	DEVMETHOD_END
};

DEFINE_CLASS_0(wave5, wave5_driver, wave5_methods,
    sizeof(struct wave5_softc));
DRIVER_MODULE(wave5, simplebus, wave5_driver, 0, 0);
MODULE_VERSION(wave5, 1);
