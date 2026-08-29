/*-
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * Copyright (c) 2024 Jari Sihvola <jsihv@gmx.com>
 * Copyright (c) 2024 The FreeBSD Foundation
 *
 * Portions of this software were developed by Mitchell Horne
 * <mhorne@FreeBSD.org> under sponsorship from the FreeBSD Foundation.
 */

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/sysctl.h>
#include <sys/bus.h>
#include <sys/kernel.h>
#include <sys/module.h>
#include <sys/mutex.h>

#include <machine/bus.h>

#include <dev/fdt/simplebus.h>
#include <dev/ofw/ofw_bus.h>
#include <dev/ofw/ofw_bus_subr.h>

#include <dev/clk/clk.h>
#include <dev/regulator/regulator.h>
#include <dev/clk/starfive/jh7110_clk.h>
#include <dev/clk/starfive/jh7110_clk_pll.h>
#include <dev/syscon/syscon.h>

#include <dt-bindings/clock/starfive,jh7110-crg.h>

#include "clkdev_if.h"
#include "syscon_if.h"

#define JH7110_SYS_SYSCON_SYSCFG24		0x18
#define JH7110_SYS_SYSCON_SYSCFG28		0x1c
#define JH7110_SYS_SYSCON_SYSCFG32		0x20
#define JH7110_SYS_SYSCON_SYSCFG36		0x24
#define JH7110_SYS_SYSCON_SYSCFG40		0x28
#define JH7110_SYS_SYSCON_SYSCFG44		0x2c
#define JH7110_SYS_SYSCON_SYSCFG48		0x30
#define JH7110_SYS_SYSCON_SYSCFG52		0x34

#define	DEVICE_LOCK(_clk)				\
	CLKDEV_DEVICE_LOCK(clknode_get_device(_clk))
#define	DEVICE_UNLOCK(_clk)				\
	CLKDEV_DEVICE_UNLOCK(clknode_get_device(_clk))

#define PLL_MASK_FILL(sc, id)					\
do {								\
	sc->dacpd_mask = PLL## id ##_DACPD_MASK;		\
	sc->dsmpd_mask = PLL## id ##_DSMPD_MASK;		\
	sc->fbdiv_mask = PLL## id ##_FBDIV_MASK;		\
	sc->frac_mask = PLL## id ##_FRAC_MASK;			\
	sc->prediv_mask = PLL## id ##_PREDIV_MASK;		\
	sc->postdiv1_mask = PLL## id ##_POSTDIV1_MASK;		\
} while (0)

#define PLL_SHIFT_FILL(sc, id)					\
do {								\
	sc->dacpd_shift = PLL## id ##_DACPD_SHIFT;		\
	sc->dsmpd_shift = PLL## id ##_DSMPD_SHIFT;		\
	sc->fbdiv_shift = PLL## id ##_FBDIV_SHIFT;		\
	sc->frac_shift = PLL## id ##_FRAC_SHIFT;		\
	sc->prediv_shift = PLL## id ##_PREDIV_SHIFT;		\
	sc->postdiv1_shift = PLL## id ##_POSTDIV1_SHIFT;	\
} while (0)

struct jh7110_clk_pll_softc {
	struct mtx		mtx;
	struct clkdom		*clkdom;
	struct syscon		*syscon;
};

struct jh7110_pll_clknode_softc {
	uint32_t	dacpd_offset;
	uint32_t	dsmpd_offset;
	uint32_t	fbdiv_offset;
	uint32_t	frac_offset;
	uint32_t	prediv_offset;
	uint32_t	postdiv1_offset;

	uint32_t	dacpd_mask;
	uint32_t	dsmpd_mask;
	uint32_t	fbdiv_mask;
	uint32_t	frac_mask;
	uint32_t	prediv_mask;
	uint32_t	postdiv1_mask;

	uint32_t	dacpd_shift;
	uint32_t	dsmpd_shift;
	uint32_t	fbdiv_shift;
	uint32_t	frac_shift;
	uint32_t	prediv_shift;
	uint32_t	postdiv1_shift;

	const struct jh7110_pll_syscon_value *syscon_arr;
	int		syscon_nitems;
};

static const char *pll_parents[] = { "osc" };

static struct jh7110_clk_def pll_out_clks[] = {
	{
		.clkdef.id = JH7110_PLLCLK_PLL0_OUT,
		.clkdef.name = "pll0_out",
		.clkdef.parent_names = pll_parents,
		.clkdef.parent_cnt = nitems(pll_parents),
		.clkdef.flags = CLK_NODE_STATIC_STRINGS,
	},
	{
		.clkdef.id = JH7110_PLLCLK_PLL1_OUT,
		.clkdef.name = "pll1_out",
		.clkdef.parent_names = pll_parents,
		.clkdef.parent_cnt = nitems(pll_parents),
		.clkdef.flags = CLK_NODE_STATIC_STRINGS,
	},
	{
		.clkdef.id = JH7110_PLLCLK_PLL2_OUT,
		.clkdef.name = "pll2_out",
		.clkdef.parent_names = pll_parents,
		.clkdef.parent_cnt = nitems(pll_parents),
		.clkdef.flags = CLK_NODE_STATIC_STRINGS,
	},
};

static int jh7110_clk_pll_register(struct clkdom *clkdom,
    struct jh7110_clk_def *clkdef);

static int
jh7110_clk_pll_recalc_freq(struct clknode *clk, uint64_t *freq)
{
	struct jh7110_clk_pll_softc *sc;
	struct jh7110_pll_clknode_softc *clk_sc;
	uint32_t dacpd, dsmpd, fbdiv, prediv, postdiv1;
	uint64_t frac, fcal = 0;

	sc = device_get_softc(clknode_get_device(clk));
	clk_sc = clknode_get_softc(clk);

	DEVICE_LOCK(clk);

	dacpd = (SYSCON_READ_4(sc->syscon, clk_sc->dacpd_offset) & clk_sc->dacpd_mask) >>
	    clk_sc->dacpd_shift;
	dsmpd = (SYSCON_READ_4(sc->syscon, clk_sc->dsmpd_offset) & clk_sc->dsmpd_mask) >>
	    clk_sc->dsmpd_shift;
	fbdiv = (SYSCON_READ_4(sc->syscon, clk_sc->fbdiv_offset) & clk_sc->fbdiv_mask) >>
	    clk_sc->fbdiv_shift;
	prediv = (SYSCON_READ_4(sc->syscon, clk_sc->prediv_offset) & clk_sc->prediv_mask) >>
	    clk_sc->prediv_shift;
	postdiv1 = (SYSCON_READ_4(sc->syscon, clk_sc->postdiv1_offset) &
	    clk_sc->postdiv1_mask) >> clk_sc->postdiv1_shift;
	frac = (SYSCON_READ_4(sc->syscon, clk_sc->frac_offset) & clk_sc->frac_mask) >>
	    clk_sc->frac_shift;

	DEVICE_UNLOCK(clk);

	/* dacpd and dsmpd both being 0 entails Fraction Multiple Mode */
	if (dacpd == 0 && dsmpd == 0)
		fcal = frac * FRAC_PATR_SIZE / (1 << 24);

	*freq = *freq / FRAC_PATR_SIZE * (fbdiv * FRAC_PATR_SIZE + fcal) /
	    prediv / (1 << postdiv1);

	return (0);
}

static int
jh7110_clk_pll_set_freq(struct clknode *clk, uint64_t fin, uint64_t *fout,
    int flags, int *done)
{
	struct jh7110_clk_pll_softc *sc;
	struct jh7110_pll_clknode_softc *clk_sc;
	const struct jh7110_pll_syscon_value *syscon_val = NULL;

	sc = device_get_softc(clknode_get_device(clk));
	clk_sc = clknode_get_softc(clk);

	for (int i = 0; i != clk_sc->syscon_nitems; i++) {
		if (*fout == clk_sc->syscon_arr[i].freq) {
			syscon_val = &clk_sc->syscon_arr[i];
		}
	}

	if (syscon_val == NULL) {
		printf("%s: tried to set an unknown frequency %ju for %s\n",
		       __func__, *fout, clknode_get_name(clk));
		return (EINVAL);
	}

	if ((flags & CLK_SET_DRYRUN) != 0) {
		*done = 1;
		return (0);
	}

	DEVICE_LOCK(clk);

	SYSCON_MODIFY_4(sc->syscon, clk_sc->dacpd_offset, clk_sc->dacpd_mask,
	    syscon_val->dacpd << clk_sc->dacpd_shift & clk_sc->dacpd_mask);
	SYSCON_MODIFY_4(sc->syscon, clk_sc->dsmpd_offset, clk_sc->dsmpd_mask,
	    syscon_val->dsmpd << clk_sc->dsmpd_shift & clk_sc->dsmpd_mask);
	SYSCON_MODIFY_4(sc->syscon, clk_sc->prediv_offset, clk_sc->prediv_mask,
	    syscon_val->prediv << clk_sc->prediv_shift & clk_sc->prediv_mask);
	SYSCON_MODIFY_4(sc->syscon, clk_sc->fbdiv_offset, clk_sc->fbdiv_mask,
	    syscon_val->fbdiv << clk_sc->fbdiv_shift & clk_sc->fbdiv_mask);
	SYSCON_MODIFY_4(sc->syscon, clk_sc->postdiv1_offset,
	    clk_sc->postdiv1_mask, (syscon_val->postdiv1 >> 1) <<
	    clk_sc->postdiv1_shift & clk_sc->postdiv1_mask);

	if (!syscon_val->dacpd && !syscon_val->dsmpd) {
		SYSCON_MODIFY_4(sc->syscon, clk_sc->frac_offset, clk_sc->frac_mask,
		    syscon_val->frac << clk_sc->frac_shift & clk_sc->frac_mask);
	}

	DEVICE_UNLOCK(clk);

	*done = 1;
	return (0);
}

static int
jh7110_clk_pll_init(struct clknode *clk, device_t dev)
{
	clknode_init_parent_idx(clk, 0);

	return (0);
}

static int
jh7110_clk_pll_probe(device_t dev)
{
	if (!ofw_bus_status_okay(dev))
		return (ENXIO);

	if (!ofw_bus_is_compatible(dev, "starfive,jh7110-pll"))
		return (ENXIO);

	device_set_desc(dev, "StarFive JH7110 PLL clock generator");

	return (BUS_PROBE_DEFAULT);
}

static int
jh7110_clk_pll_attach(device_t dev)
{
	struct jh7110_clk_pll_softc *sc;
	int error;

	sc = device_get_softc(dev);

	mtx_init(&sc->mtx, device_get_nameunit(dev), NULL, MTX_DEF);

	sc->clkdom = clkdom_create(dev);
	if (sc->clkdom == NULL) {
		device_printf(dev, "Couldn't create clkdom\n");
		return (ENXIO);
	}

	error = syscon_get_by_ofw_node(dev, OF_parent(ofw_bus_get_node(dev)),
	    &sc->syscon);
	if (error != 0) {
		device_printf(dev, "Couldn't get syscon handle of parent\n");
		return (error);
	}

	for (int i = 0; i < nitems(pll_out_clks); i++) {
		error = jh7110_clk_pll_register(sc->clkdom, &pll_out_clks[i]);
		if (error != 0)
			device_printf(dev, "Couldn't register clock %s: %d\n",
			    pll_out_clks[i].clkdef.name, error);
	}

	error = clkdom_finit(sc->clkdom);
	if (error != 0) {
		device_printf(dev, "clkdom_finit() returned %d\n", error);
	}

	if (bootverbose)
		clkdom_dump(sc->clkdom);

	return (0);
}

static void
jh7110_clk_pll_device_lock(device_t dev)
{
	struct jh7110_clk_pll_softc *sc;

	sc = device_get_softc(dev);
	mtx_lock(&sc->mtx);
}

static void
jh7110_clk_pll_device_unlock(device_t dev)
{
	struct jh7110_clk_pll_softc *sc;

	sc = device_get_softc(dev);
	mtx_unlock(&sc->mtx);
}

static clknode_method_t jh7110_pllnode_methods[] = {
	/* Device interface */
	CLKNODEMETHOD(clknode_init,		jh7110_clk_pll_init),
	CLKNODEMETHOD(clknode_recalc_freq,	jh7110_clk_pll_recalc_freq),
	CLKNODEMETHOD(clknode_set_freq,		jh7110_clk_pll_set_freq),

	CLKNODEMETHOD_END
};

static device_method_t jh7110_clk_pll_methods[] = {
	/* Device interface */
	DEVMETHOD(device_probe,			jh7110_clk_pll_probe),
	DEVMETHOD(device_attach,		jh7110_clk_pll_attach),

	/* clkdev interface */
	DEVMETHOD(clkdev_device_lock,		jh7110_clk_pll_device_lock),
	DEVMETHOD(clkdev_device_unlock,		jh7110_clk_pll_device_unlock),

	DEVMETHOD_END
};

DEFINE_CLASS_1(jh7110_pllnode, jh7110_pllnode_class, jh7110_pllnode_methods,
    sizeof(struct jh7110_pll_clknode_softc), clknode_class);
DEFINE_CLASS_0(jh7110_clk_pll, jh7110_clk_pll_driver, jh7110_clk_pll_methods,
    sizeof(struct jh7110_clk_pll_softc));
EARLY_DRIVER_MODULE(jh7110_clk_pll, simplebus, jh7110_clk_pll_driver, 0, 0,
    BUS_PASS_BUS + BUS_PASS_ORDER_EARLY);
MODULE_VERSION(jh7110_clk_pll, 1);

int
jh7110_clk_pll_register(struct clkdom *clkdom, struct jh7110_clk_def *clkdef)
{
	struct clknode *clk = NULL;
	struct jh7110_pll_clknode_softc *sc;

	clk = clknode_create(clkdom, &jh7110_pllnode_class, &clkdef->clkdef);
	if (clk == NULL)
		return (1);

	sc = clknode_get_softc(clk);

	switch (clkdef->clkdef.id) {
	case JH7110_PLLCLK_PLL0_OUT:
		sc->syscon_arr = jh7110_pll0_syscon_freq;
		sc->syscon_nitems = nitems(jh7110_pll0_syscon_freq);
		PLL_MASK_FILL(sc, 0);
		PLL_SHIFT_FILL(sc, 0);
		sc->dacpd_offset = JH7110_SYS_SYSCON_SYSCFG24;
		sc->dsmpd_offset = JH7110_SYS_SYSCON_SYSCFG24;
		sc->fbdiv_offset = JH7110_SYS_SYSCON_SYSCFG28;
		sc->frac_offset = JH7110_SYS_SYSCON_SYSCFG32;
		sc->prediv_offset = JH7110_SYS_SYSCON_SYSCFG36;
		sc->postdiv1_offset = JH7110_SYS_SYSCON_SYSCFG32;
		break;
	case JH7110_PLLCLK_PLL1_OUT:
		sc->syscon_arr = jh7110_pll1_syscon_freq;
		sc->syscon_nitems = nitems(jh7110_pll1_syscon_freq);
		PLL_MASK_FILL(sc, 1);
		PLL_SHIFT_FILL(sc, 1);
		sc->dacpd_offset = JH7110_SYS_SYSCON_SYSCFG36;
		sc->dsmpd_offset = JH7110_SYS_SYSCON_SYSCFG36;
		sc->fbdiv_offset = JH7110_SYS_SYSCON_SYSCFG36;
		sc->frac_offset = JH7110_SYS_SYSCON_SYSCFG40;
		sc->prediv_offset = JH7110_SYS_SYSCON_SYSCFG44;
		sc->postdiv1_offset = JH7110_SYS_SYSCON_SYSCFG40;
		break;
	case JH7110_PLLCLK_PLL2_OUT:
		sc->syscon_arr = jh7110_pll2_syscon_freq;
		sc->syscon_nitems = nitems(jh7110_pll2_syscon_freq);
		PLL_MASK_FILL(sc, 2);
		PLL_SHIFT_FILL(sc, 2);
		sc->dacpd_offset = JH7110_SYS_SYSCON_SYSCFG44;
		sc->dsmpd_offset = JH7110_SYS_SYSCON_SYSCFG44;
		sc->fbdiv_offset = JH7110_SYS_SYSCON_SYSCFG44;
		sc->frac_offset = JH7110_SYS_SYSCON_SYSCFG48;
		sc->prediv_offset = JH7110_SYS_SYSCON_SYSCFG52;
		sc->postdiv1_offset = JH7110_SYS_SYSCON_SYSCFG48;
		break;
	default:
		return (EINVAL);
	}

	clknode_register(clkdom, clk);

	return (0);
}


/*
 * PLL0 rate, applied late.
 *
 * We leave PLL0 at U-Boot's 1000 MHz; the reference platform runs it at
 * 1500 MHz, and that single difference cascades into dozens of downstream
 * clock mismatches (see docs/study/soc/01-soc-glue-parity.md).
 *
 * cpu_core divides straight off cpu_root/pll0_out, so raising PLL0 raises the
 * CPU clock, and the reference OPP table requires more voltage to run there:
 *
 *     375 / 500 / 750 MHz -> 900 000 uV
 *     1500 MHz            -> 1 040 000 uV
 *
 * vdd-cpu sits at 900 000 uV, so the rail must come up FIRST. That regulator
 * lives on the AXP15060, which is behind i2c and attaches long after this
 * clock driver, so this runs from a late SYSINIT rather than from attach.
 *
 *   hw.jh7110.pll0_hz=1500000000   opt in
 *   hw.jh7110.vdd_cpu_uv=<uV>      override the target voltage
 */
static void
jh7110_pll0_apply_tunable(void *dummy __unused)
{
	struct clknode *pll0;
	regulator_t vdd_cpu;
	char *ev;
	uint64_t want, got = 0;
	int uv_target = 1040000, uv_now = 0, error;

	ev = kern_getenv("hw.jh7110.pll0_hz");
	if (ev == NULL)
		return;
	want = strtoul(ev, NULL, 0);
	freeenv(ev);
	if (want == 0)
		return;

	ev = kern_getenv("hw.jh7110.vdd_cpu_uv");
	if (ev != NULL) {
		uv_target = (int)strtoul(ev, NULL, 0);
		freeenv(ev);
	}

	pll0 = clknode_find_by_name("pll0_out");
	if (pll0 == NULL) {
		printf("jh7110_pll0: pll0_out clknode not found\n");
		return;
	}

	/* Voltage first, and only ever upwards from here. */
	error = regulator_get_by_name(root_bus, "vdd-cpu", &vdd_cpu);
	if (error != 0) {
		printf("jh7110_pll0: vdd-cpu regulator not found (%d), "
		    "refusing to raise PLL0\n", error);
		return;
	}
	regulator_get_voltage(vdd_cpu, &uv_now);
	if (uv_now < uv_target) {
		error = regulator_set_voltage(vdd_cpu, uv_target, uv_target);
		if (error != 0) {
			printf("jh7110_pll0: vdd-cpu %d -> %d uV FAILED (%d), "
			    "refusing to raise PLL0\n", uv_now, uv_target, error);
			return;
		}
		regulator_get_voltage(vdd_cpu, &uv_now);
		printf("jh7110_pll0: vdd-cpu now %d uV\n", uv_now);
	}
	if (uv_now < uv_target) {
		printf("jh7110_pll0: vdd-cpu still %d uV (< %d), "
		    "refusing to raise PLL0\n", uv_now, uv_target);
		return;
	}

	error = clknode_set_freq(pll0, want, 0, 0);
	clknode_get_freq(pll0, &got);
	printf("jh7110_pll0: requested %ju Hz, got %ju Hz (err=%d)\n",
	    (uintmax_t)want, (uintmax_t)got, error);

	/*
	 * Restore the clocks that must hold an exact rate regardless of the
	 * parent.
	 *
	 * usb_125m is a plain divider off pll0_out sized for a 1000 MHz parent
	 * (1000/8). Raising PLL0 drags it to 187.5 MHz, and USB 3.0 needs
	 * exactly 125 MHz - on this board that is also the WiFi dongle. The
	 * reference platform reaches the same 125 MHz as 1500/12, so ask for
	 * the rate and let the divider follow.
	 */
	{
		struct clknode *usb = clknode_find_by_name("usb_125m");
		uint64_t urate = 0;

		if (usb != NULL) {
			int uerr = clknode_set_freq(usb, 125000000, 0, 0);

			clknode_get_freq(usb, &urate);
			printf("jh7110_pll0: usb_125m now %ju Hz (err=%d)\n",
			    (uintmax_t)urate, uerr);
		}
	}
}

/*
 * perh_root: wrong parent, same bug as gpu_root and isp_2x.
 *
 * perh_root is a mux over {pll0_out, pll2_out}. We select pll0_out; the
 * reference platform selects pll2_out and divides by 2 for 594 MHz, which is
 * unreachable from PLL0 by an integer divider. uart4_core, uart5_core and the
 * CAN clocks all hang off it, which is why they are the residue that raising
 * PLL0 alone cannot fix:
 *
 *     perh_root   ours 500 MHz (pll0/2)   reference 594 MHz (pll2/2)
 *     uart4/5     ours 195312 Hz          reference 59 400 000 Hz
 *
 * Not the console UART - that is uart0 off the 24 MHz oscillator - so this is
 * safe to change on a live system.
 */
static void
jh7110_perh_root_fix(void)
{
	struct clknode *perh;
	uint64_t got = 0;
	int error;

	perh = clknode_find_by_name("perh_root");
	if (perh == NULL) {
		printf("jh7110_perh: perh_root clknode not found\n");
		return;
	}

	error = clknode_set_parent_by_name(perh, "pll2_out");
	if (error != 0) {
		printf("jh7110_perh: reparent to pll2_out failed (%d)\n", error);
		return;
	}

	error = clknode_set_freq(perh, 594000000, 0, 0);
	clknode_get_freq(perh, &got);
	printf("jh7110_perh: perh_root now %ju Hz (err=%d)\n",
	    (uintmax_t)got, error);
}
/*
 * Deliberately NOT a SYSINIT.
 *
 * This ran at SI_SUB_LAST, immediately after "Trying to mount root" and
 * before init produces any output. PLL0 feeds qspi_ref, the UART clocks and
 * the peripheral bus, so raising it there took out console and storage
 * together and left the board unbootable - every subsequent boot re-applied
 * the tunable from loader.conf, so the only way back in was the serial
 * loader prompt or pulling the SD card.
 *
 * Apply it from userland instead (sysctl hw.jh7110.pll0_apply=1), where a bad
 * outcome costs one reboot rather than bricking every boot.
 */

/*
 * Same routine, triggerable at runtime:  sysctl hw.jh7110.pll0_apply=1
 *
 * The PMIC is behind i2c and the boot-time SYSINIT may run before transfers
 * to it are reliable. Being able to re-run the sequence from userland tells
 * the two failure modes apart.
 */
static int
jh7110_pll0_apply_sysctl(SYSCTL_HANDLER_ARGS)
{
	int val = 0, error;

	error = sysctl_handle_int(oidp, &val, 0, req);
	if (error != 0 || req->newptr == NULL)
		return (error);
	if (val != 0) {
		jh7110_pll0_apply_tunable(NULL);
		jh7110_perh_root_fix();
	}
	return (0);
}

static SYSCTL_NODE(_hw, OID_AUTO, jh7110, CTLFLAG_RD | CTLFLAG_MPSAFE, 0,
    "StarFive JH7110");
SYSCTL_PROC(_hw_jh7110, OID_AUTO, pll0_apply,
    CTLTYPE_INT | CTLFLAG_WR | CTLFLAG_MPSAFE, NULL, 0,
    jh7110_pll0_apply_sysctl, "I",
    "Write 1 to apply hw.jh7110.pll0_hz (raises vdd-cpu first)");
