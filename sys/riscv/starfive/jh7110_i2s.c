/*-
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * Copyright (c) 2026 Tej <itej89@github.com>
 *
 * Synopsys DesignWare I2S transmitter (I2S TX0) on the StarFive JH7110.
 *
 * This is the CPU side of the HDMI sound card: it clocks samples out to the
 * Innosilicon transmitter, which is configured by jh7110_hdmi_audio.c. It is
 * fed by the AXI DMAC (jh7110_axidma.c) on hardware handshake 47.
 *
 * Register meanings follow Linux's sound/soc/dwc/dwc-i2s.c, which is the only
 * documentation for this block, including one JH7110-specific quirk that is
 * not obvious and not optional: the DMA handshake is driven by the *interrupt*
 * outputs, not by the block's own DMACR. Leave the FIFO interrupts masked and
 * the DMAC is never asked for data and nothing plays.
 *
 * The refill structure is the one arrived at on the PWMDAC: the ring is
 * written through the L2 bypass alias and each period evicted from the CCache
 * afterwards (the DMAC's reads are served from it), the refill chases the
 * play position rather than assuming one period per interrupt, and it writes
 * the period furthest from the play point. See the audio tracker for why each
 * of those is necessary.
 */

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/bus.h>
#include <sys/kernel.h>
#include <sys/lock.h>
#include <sys/module.h>
#include <sys/mutex.h>
#include <sys/rman.h>
#include <sys/sysctl.h>

#include <machine/bus.h>

#include <vm/vm.h>
#include <vm/pmap.h>

#include <dev/ofw/ofw_bus.h>
#include <dev/ofw/ofw_bus_subr.h>

#include <dev/clk/clk.h>
#include <dev/hwreset/hwreset.h>

#include <riscv/sifive/sifive_ccache.h>

#include "opt_snd.h"
#include <dev/sound/pcm/sound.h>
#include <dev/sound/fdt/audio_dai.h>
#include "audio_dai_if.h"

#include "jh7110_axidma.h"
#include "jh7110_i2s.h"

/* Common registers. */
#define	I2S_IER			0x000
#define	I2S_IRER		0x004
#define	I2S_ITER		0x008
#define	I2S_CER			0x00c
#define	I2S_CCR			0x010
#define	I2S_TXFFR		0x018

#define	IER_IEN			(1u << 0)

/* Per-channel block, 0x40 apart. */
#define	I2S_TER(x)		(0x40 * (x) + 0x02c)
#define	I2S_TCR(x)		(0x40 * (x) + 0x034)
#define	I2S_ISR(x)		(0x40 * (x) + 0x038)
#define	I2S_IMR(x)		(0x40 * (x) + 0x03c)
#define	I2S_TFF(x)		(0x40 * (x) + 0x054)
#define	I2S_TFCR(x)		(0x40 * (x) + 0x04c)

#define	TER_TXCHEN		(1u << 0)

/* Transmit FIFO interrupt bits in IMR: overrun and empty. */
#define	IMR_TX_MASK		0x30u

/* The DMA window. Writes here alternate left and right automatically. */
#define	I2S_TXDMA		0x1c8

#define	I2S_DMACR		0x200
#define	I2S_COMP_PARAM_1	0x1f4
#define	COMP1_FIFO_DEPTH(r)	((((r) >> 2) & 0x3))

/* Transfer resolutions for TCR. */
#define	XFER_RES_16BIT		0x02

/*
 * Ring geometry. 1 KiB is 256 stereo frames, 5.33 ms at 48 kHz, so 187
 * interrupts a second; 16 of them is 85 ms of buffering.
 */
#define	I2S_PERIOD_BYTES	1024
#define	I2S_NPERIODS		16
#define	I2S_RING_BYTES		(I2S_PERIOD_BYTES * I2S_NPERIODS)

static uint32_t jh7110_i2s_fmts[] = {
	SND_FORMAT(AFMT_S16_LE, 2, 0),
	0,
};

static struct pcmchan_caps jh7110_i2s_caps = {
	32000, 48000, jh7110_i2s_fmts, 0,
};

struct jh7110_i2s_softc {
	device_t		dev;
	struct resource		*res;
	struct mtx		mtx;

	clk_t			clk_i2s;	/* bit clock, master */
	clk_t			clk_apb;
	clk_t			clk_mclk;
	clk_t			clk_mclk_inner;
	clk_t			clk_mclk_ext;
	/*
	 * This instance is a slave (COMP_PARAM_1 bit 4 is clear): it shifts
	 * data only while these run, and they come from the clock controller
	 * rather than from the block. The frame clock in particular is not in
	 * the upstream binding and was simply never enabled.
	 */
	clk_t			clk_bclk;
	clk_t			clk_lrck;

	uint32_t		speed;
	uint32_t		fifo_th;
	int			running;
	int			clk_running;

	driver_intr_t		*intr_handler;
	void			*intr_arg;

	struct jh7110_dma_chan	*dma;
	bus_dma_tag_t		ring_tag;
	bus_dmamap_t		ring_map;
	void			*ring_cached;	/* mapping we must not use */
	volatile uint8_t	*ring;		/* L2 bypass alias */
	bus_addr_t		ring_pa;

	uint32_t		play_ptr;
	u_int			fill_idx;
};

#define	I2S_LOCK(sc)		mtx_lock(&(sc)->mtx)
#define	I2S_UNLOCK(sc)		mtx_unlock(&(sc)->mtx)
#define	I2S_RD(sc, o)		bus_read_4((sc)->res, (o))
#define	I2S_WR(sc, o, v)	bus_write_4((sc)->res, (o), (v))

/*
 * Attenuation in percent, 100 being untouched. HDMI is a full-scale digital
 * link and the sink owns the real volume control; this is a convenience, and
 * it costs resolution, so it defaults to off.
 */
static int i2s_volume = 100;

static uint32_t i2s_periods;
static uint32_t i2s_underruns;

static struct ofw_compat_data compat_data[] = {
	{ "starfive,jh7110-i2stx0",	1 },
	{ NULL,				0 }
};

/*
 * Register dump. ISR bit 4 is the transmit FIFO empty flag -- the thing that
 * is supposed to raise the DMA request on this SoC -- and IMR bit 4 is
 * whether it is unmasked. CER/ITER/IER say whether the block is actually
 * transmitting.
 */
static int
jh7110_i2s_regs(SYSCTL_HANDLER_ARGS)
{
	struct jh7110_i2s_softc *sc = arg1;
	char buf[512];
	int off = 0;

	off += snprintf(buf + off, sizeof(buf) - off,
	    "IER %#x IRER %#x ITER %#x CER %#x CCR %#x",
	    I2S_RD(sc, I2S_IER), I2S_RD(sc, I2S_IRER),
	    I2S_RD(sc, I2S_ITER), I2S_RD(sc, I2S_CER),
	    I2S_RD(sc, I2S_CCR));
	off += snprintf(buf + off, sizeof(buf) - off,
	    " | TER0 %#x TCR0 %#x TFCR0 %#x",
	    I2S_RD(sc, I2S_TER(0)), I2S_RD(sc, I2S_TCR(0)),
	    I2S_RD(sc, I2S_TFCR(0)));
	off += snprintf(buf + off, sizeof(buf) - off,
	    " | IMR0 %#x ISR0 %#x TFF0 %#x",
	    I2S_RD(sc, I2S_IMR(0)), I2S_RD(sc, I2S_ISR(0)),
	    I2S_RD(sc, I2S_TFF(0)));
	off += snprintf(buf + off, sizeof(buf) - off,
	    " | COMP1 %#x DMACR %#x",
	    I2S_RD(sc, I2S_COMP_PARAM_1), I2S_RD(sc, I2S_DMACR));

	return (sysctl_handle_string(oidp, buf, sizeof(buf), req));
}

static int jh7110_i2s_ring_alloc(struct jh7110_i2s_softc *sc);

/*
 * Bring the block up for a stream. Shared by the sound layer's trigger and by
 * the diagnostic tone, so that the tone exercises exactly the same path.
 */
static void
jh7110_i2s_hw_start(struct jh7110_i2s_softc *sc)
{
	uint32_t i;

	if (!sc->clk_running) {
		clk_enable(sc->clk_i2s);
		if (sc->clk_bclk != NULL)
			clk_enable(sc->clk_bclk);
		if (sc->clk_lrck != NULL)
			clk_enable(sc->clk_lrck);
		sc->clk_running = 1;
	}

	I2S_WR(sc, I2S_TXFFR, 1);
	for (i = 0; i < 4; i++)
		I2S_WR(sc, I2S_TER(i), 0);
	I2S_WR(sc, I2S_CCR, 0x00);
	I2S_WR(sc, I2S_TCR(0), XFER_RES_16BIT);
	I2S_WR(sc, I2S_TFCR(0), sc->fifo_th - 1);
	I2S_WR(sc, I2S_TER(0), TER_TXCHEN);

	I2S_WR(sc, I2S_IER, IER_IEN);
	I2S_WR(sc, I2S_ITER, 1);

	/*
	 * Unmask the transmit FIFO interrupts: on this SoC that is what
	 * drives the DMA request. CER is not written -- this instance is a
	 * slave (COMP_PARAM_1 bit 4 clear) and does not generate clocks.
	 */
	I2S_WR(sc, I2S_IMR(0), I2S_RD(sc, I2S_IMR(0)) & ~IMR_TX_MASK);
	I2S_WR(sc, I2S_CER, 1);
}

static void
jh7110_i2s_hw_stop(struct jh7110_i2s_softc *sc)
{

	I2S_WR(sc, I2S_IMR(0), I2S_RD(sc, I2S_IMR(0)) | IMR_TX_MASK);
	I2S_WR(sc, I2S_ITER, 0);
	I2S_WR(sc, I2S_CER, 0);
	I2S_WR(sc, I2S_IER, 0);

	if (sc->clk_running) {
		if (sc->clk_lrck != NULL)
			clk_disable(sc->clk_lrck);
		if (sc->clk_bclk != NULL)
			clk_disable(sc->clk_bclk);
		clk_disable(sc->clk_i2s);
		sc->clk_running = 0;
	}
}

/*
 * DIAGNOSTIC: a tone that lives in the DMA ring, with no refill and no sound
 * layer. See add-i2s-tone.py -- every measurement before this was taken of a
 * system already tearing down, because the player exits the moment a write
 * returns EAGAIN.
 */
static int
jh7110_i2s_testtone(SYSCTL_HANDLER_ARGS)
{
	struct jh7110_i2s_softc *sc = arg1;
	struct jh7110_dma_slave_config cfg;
	volatile int16_t *w;
	int secs = 0, err, i, n;

	err = sysctl_handle_int(oidp, &secs, 0, req);
	if (err != 0 || req->newptr == NULL)
		return (err);
	if (secs <= 0)
		return (0);
	if (secs > 30)
		secs = 30;

	if (sc->dma == NULL) {
		sc->dma = jh7110_axidma_get(sc->dev, "tx");
		if (sc->dma == NULL)
			return (ENXIO);
	}
	if (jh7110_i2s_ring_alloc(sc) != 0)
		return (ENOMEM);

	/*
	 * 1 kHz at 48 kHz is 48 frames per cycle and the ring holds 4096
	 * frames, so it closes exactly and any discontinuity heard is the
	 * hardware's rather than the waveform's.
	 */
	w = (volatile int16_t *)sc->ring;
	n = I2S_RING_BYTES / 4;
	for (i = 0; i < n; i++) {
		static const int16_t sine48[48] = {
			0, 4276, 8480, 12539, 16383, 19947, 23169, 25995,
			28377, 30272, 31650, 32486, 32767, 32486, 31650,
			30272, 28377, 25995, 23169, 19947, 16383, 12539,
			8480, 4276, 0, -4276, -8480, -12539, -16383, -19947,
			-23169, -25995, -28377, -30272, -31650, -32486,
			-32767, -32486, -31650, -30272, -28377, -25995,
			-23169, -19947, -16383, -12539, -8480, -4276
		};
		int16_t v = sine48[i % 48];

		w[i * 2] = v;
		w[i * 2 + 1] = v;
	}
	sifive_ccache_flush_range(sc->ring_pa, I2S_RING_BYTES);

	device_printf(sc->dev, "testtone: %d s, ring-resident, no refill\n",
	    secs);

	I2S_LOCK(sc);
	sc->running = 1;
	jh7110_i2s_hw_start(sc);
	I2S_UNLOCK(sc);

	cfg.dev_addr = rman_get_start(sc->res) + I2S_TXDMA;
	cfg.dev_width = 2;
	cfg.maxburst = 16;
	cfg.direction = JH7110_DMA_MEM_TO_DEV;

	err = jh7110_axidma_cyclic(sc->dma, &cfg, sc->ring_pa,
	    I2S_RING_BYTES, I2S_PERIOD_BYTES, NULL, NULL);
	if (err != 0) {
		device_printf(sc->dev, "testtone: start failed %d\n", err);
		I2S_LOCK(sc);
		sc->running = 0;
		jh7110_i2s_hw_stop(sc);
		I2S_UNLOCK(sc);
		return (err);
	}

	pause("i2stone", secs * hz);

	jh7110_axidma_stop(sc->dma);
	I2S_LOCK(sc);
	sc->running = 0;
	jh7110_i2s_hw_stop(sc);
	I2S_UNLOCK(sc);

	device_printf(sc->dev, "testtone: done\n");

	return (0);
}

/*
 * Set the playback attenuation, 0-100. Called from the HDMI codec's mixer --
 * audio_soc(4) registers mixers against the codec, but the gain has to be
 * applied where the samples are copied, which is here.
 */
void
jh7110_i2s_set_volume(int pct)
{

	if (pct < 0)
		pct = 0;
	else if (pct > 100)
		pct = 100;
	i2s_volume = pct;
}

int
jh7110_i2s_get_volume(void)
{

	return (i2s_volume);
}

static int
jh7110_i2s_probe(device_t dev)
{

	if (!ofw_bus_status_okay(dev))
		return (ENXIO);
	if (!ofw_bus_search_compatible(dev, compat_data)->ocd_data)
		return (ENXIO);

	device_set_desc(dev, "StarFive JH7110 I2S TX0");
	return (BUS_PROBE_DEFAULT);
}

static void
jh7110_i2s_ring_cb(void *arg, bus_dma_segment_t *segs, int nseg, int error)
{

	if (error != 0)
		return;
	*(bus_addr_t *)arg = segs[0].ds_addr;
}

/*
 * The DMAC is not coherent with the caches and this SoC has no Svpbmt, so an
 * "uncached" mapping request is silently ignored. The ring is therefore
 * accessed through the SiFive L2 bypass alias, and each period is evicted
 * from the CCache after it is written -- the engine's reads are served from
 * that cache, and a write through the alias does not snoop it.
 */
static int
jh7110_i2s_ring_alloc(struct jh7110_i2s_softc *sc)
{
	uint64_t off;
	int error;

	if (sc->ring != NULL)
		return (0);

	error = bus_dma_tag_create(bus_get_dma_tag(sc->dev), 4, 0,
	    BUS_SPACE_MAXADDR, BUS_SPACE_MAXADDR, NULL, NULL,
	    I2S_RING_BYTES, 1, I2S_RING_BYTES, 0, NULL, NULL, &sc->ring_tag);
	if (error != 0)
		return (error);

	error = bus_dmamem_alloc(sc->ring_tag, &sc->ring_cached,
	    BUS_DMA_WAITOK | BUS_DMA_ZERO | BUS_DMA_COHERENT, &sc->ring_map);
	if (error != 0)
		return (error);

	error = bus_dmamap_load(sc->ring_tag, sc->ring_map, sc->ring_cached,
	    I2S_RING_BYTES, jh7110_i2s_ring_cb, &sc->ring_pa, BUS_DMA_NOWAIT);
	if (error != 0 || sc->ring_pa == 0)
		return (error != 0 ? error : ENOMEM);

	sifive_ccache_flush_range(sc->ring_pa, I2S_RING_BYTES);

	off = sifive_ccache_uncached_offset();
	if (off == 0) {
		device_printf(sc->dev, "no L2 bypass alias available\n");
		return (ENXIO);
	}
	sc->ring = pmap_mapdev((vm_paddr_t)sc->ring_pa + off, I2S_RING_BYTES);
	if (sc->ring == NULL)
		return (ENOMEM);

	return (0);
}

static void
jh7110_i2s_ring_free(struct jh7110_i2s_softc *sc)
{

	if (sc->ring != NULL) {
		pmap_unmapdev(__DEVOLATILE(void *, sc->ring), I2S_RING_BYTES);
		sc->ring = NULL;
	}
	if (sc->ring_pa != 0) {
		bus_dmamap_unload(sc->ring_tag, sc->ring_map);
		sc->ring_pa = 0;
	}
	if (sc->ring_cached != NULL) {
		bus_dmamem_free(sc->ring_tag, sc->ring_cached, sc->ring_map);
		sc->ring_cached = NULL;
	}
	if (sc->ring_tag != NULL) {
		bus_dma_tag_destroy(sc->ring_tag);
		sc->ring_tag = NULL;
	}
}

static void
jh7110_i2s_period_cb(void *arg)
{
	struct jh7110_i2s_softc *sc = arg;

	if (sc->intr_handler != NULL)
		sc->intr_handler(sc->intr_arg);
}

/*
 * Clock bring-up, in the order StarFive's driver uses.
 *
 * The parent of mclk is switched to the inner source first and only moved to
 * the external 12.288 MHz oscillator after the resets are released, because
 * the external pin is not driven until its pinmux is configured and clocking
 * the block from a dead pin wedges it.
 */
static int
jh7110_i2s_clk_init(struct jh7110_i2s_softc *sc)
{
	hwreset_t rst;
	int i, error;

	error = clk_enable(sc->clk_apb);
	if (error != 0)
		return (error);

	if (sc->clk_mclk != NULL && sc->clk_mclk_inner != NULL)
		clk_set_parent_by_clk(sc->clk_mclk, sc->clk_mclk_inner);

	clk_enable(sc->clk_i2s);

	/* Both resets: APB and BCLK. */
	for (i = 0; i < 2; i++) {
		if (hwreset_get_by_ofw_idx(sc->dev, 0, i, &rst) == 0)
			hwreset_deassert(rst);
	}

	if (sc->clk_mclk != NULL && sc->clk_mclk_ext != NULL)
		clk_set_parent_by_clk(sc->clk_mclk, sc->clk_mclk_ext);

	/* Re-enabled per stream with the right rate. */
	clk_disable(sc->clk_i2s);

	return (0);
}

static int
jh7110_i2s_attach(device_t dev)
{
	struct jh7110_i2s_softc *sc = device_get_softc(dev);
	phandle_t node;
	uint32_t comp1;
	int rid, error;

	sc->dev = dev;
	sc->speed = 48000;
	mtx_init(&sc->mtx, device_get_nameunit(dev), NULL, MTX_DEF);

	rid = 0;
	sc->res = bus_alloc_resource_any(dev, SYS_RES_MEMORY, &rid, RF_ACTIVE);
	if (sc->res == NULL) {
		device_printf(dev, "cannot allocate registers\n");
		return (ENXIO);
	}

	if (clk_get_by_ofw_name(dev, 0, "i2sclk", &sc->clk_i2s) != 0 ||
	    clk_get_by_ofw_name(dev, 0, "apb", &sc->clk_apb) != 0) {
		device_printf(dev, "cannot get i2sclk/apb clocks\n");
		return (ENXIO);
	}
	/* Optional: present on tx0, absent on some variants. */
	clk_get_by_ofw_name(dev, 0, "mclk", &sc->clk_mclk);
	clk_get_by_ofw_name(dev, 0, "mclk_inner", &sc->clk_mclk_inner);
	clk_get_by_ofw_name(dev, 0, "mclk_ext", &sc->clk_mclk_ext);
	clk_get_by_ofw_name(dev, 0, "bclk", &sc->clk_bclk);
	clk_get_by_ofw_name(dev, 0, "lrck", &sc->clk_lrck);

	error = jh7110_i2s_clk_init(sc);
	if (error != 0) {
		device_printf(dev, "clock init failed: %d\n", error);
		return (error);
	}

	/*
	 * FIFO depth is 2^(1 + field). The threshold is half of it, which is
	 * what the DMA burst is sized against.
	 */
	comp1 = I2S_RD(sc, I2S_COMP_PARAM_1);
	sc->fifo_th = (1u << (1 + COMP1_FIFO_DEPTH(comp1))) / 2;
	if (sc->fifo_th == 0)
		sc->fifo_th = 8;

	node = ofw_bus_get_node(dev);
	OF_device_register_xref(OF_xref_from_node(node), dev);

	SYSCTL_ADD_PROC(device_get_sysctl_ctx(dev),
	    SYSCTL_CHILDREN(device_get_sysctl_tree(dev)), OID_AUTO,
	    "testtone", CTLTYPE_INT | CTLFLAG_WR | CTLFLAG_MPSAFE, sc, 0,
	    jh7110_i2s_testtone, "I",
	    "play a ring-resident tone with no refill (diagnostic)");

	SYSCTL_ADD_PROC(device_get_sysctl_ctx(dev),
	    SYSCTL_CHILDREN(device_get_sysctl_tree(dev)), OID_AUTO,
	    "regs", CTLTYPE_STRING | CTLFLAG_RD | CTLFLAG_MPSAFE, sc, 0,
	    jh7110_i2s_regs, "A", "I2S register state");

	SYSCTL_ADD_INT(device_get_sysctl_ctx(dev),
	    SYSCTL_CHILDREN(device_get_sysctl_tree(dev)), OID_AUTO,
	    "volume", CTLFLAG_RW, &i2s_volume, 0,
	    "attenuation in percent; 100 is untouched, sink owns the real one");

	SYSCTL_ADD_UINT(device_get_sysctl_ctx(dev),
	    SYSCTL_CHILDREN(device_get_sysctl_tree(dev)), OID_AUTO,
	    "periods", CTLFLAG_RD, &i2s_periods, 0,
	    "periods refilled from the sound layer");
	SYSCTL_ADD_UINT(device_get_sysctl_ctx(dev),
	    SYSCTL_CHILDREN(device_get_sysctl_tree(dev)), OID_AUTO,
	    "underruns", CTLFLAG_RD, &i2s_underruns, 0,
	    "periods that could not be filled completely");

	device_printf(dev, "I2S TX0 at 0x%lx, FIFO threshold %u\n",
	    rman_get_start(sc->res), sc->fifo_th);

	return (0);
}

static int
jh7110_i2s_detach(device_t dev)
{
	struct jh7110_i2s_softc *sc = device_get_softc(dev);

	if (sc->dma != NULL) {
		jh7110_axidma_put(sc->dma);
		sc->dma = NULL;
	}
	jh7110_i2s_ring_free(sc);

	return (0);
}

static int
jh7110_i2s_dai_init(device_t dev, uint32_t format)
{
	struct jh7110_i2s_softc *sc = device_get_softc(dev);

	I2S_LOCK(sc);
	I2S_WR(sc, I2S_IER, 0);
	I2S_WR(sc, I2S_ITER, 0);
	I2S_WR(sc, I2S_CER, 0);
	I2S_UNLOCK(sc);

	return (0);
}

static uint32_t
jh7110_i2s_dai_set_chanspeed(device_t dev, uint32_t speed)
{
	struct jh7110_i2s_softc *sc = device_get_softc(dev);

	if (speed != 32000 && speed != 48000)
		speed = 48000;
	sc->speed = speed;

	/*
	 * 64 bit clocks per frame: two 32-bit slots, which is what the
	 * transmitter expects at 256fs MCLK.
	 */
	clk_set_freq(sc->clk_i2s, (uint64_t)speed * 64, CLK_SET_ROUND_DOWN);

	return (speed);
}

static uint32_t
jh7110_i2s_dai_set_chanformat(device_t dev, uint32_t format)
{

	return (0);
}

static int
jh7110_i2s_dai_set_sysclk(device_t dev, unsigned int rate, int dai_dir)
{

	return (0);
}

static struct pcmchan_caps *
jh7110_i2s_dai_get_caps(device_t dev)
{

	return (&jh7110_i2s_caps);
}

static int
jh7110_i2s_dai_setup_intr(device_t dev, driver_intr_t intr_handler,
    void *intr_arg)
{
	struct jh7110_i2s_softc *sc = device_get_softc(dev);

	sc->intr_handler = intr_handler;
	sc->intr_arg = intr_arg;

	return (0);
}

static uint32_t
jh7110_i2s_fill_slot(struct jh7110_i2s_softc *sc, struct snd_dbuf *play_buf,
    u_int idx, uint32_t readyptr, uint32_t ready)
{
	volatile uint32_t *slot;
	const uint8_t *samples;
	uint32_t avail, size, i;
	int vol = i2s_volume;

	if (vol < 0)
		vol = 0;
	else if (vol > 100)
		vol = 100;

	slot = (volatile uint32_t *)(sc->ring +
	    (size_t)idx * I2S_PERIOD_BYTES);
	size = play_buf->bufsize;
	samples = (const uint8_t *)play_buf->buf;

	avail = ready;
	if (avail > I2S_PERIOD_BYTES)
		avail = I2S_PERIOD_BYTES;
	avail &= ~3u;

	/*
	 * Straight copy: unlike the PWMDAC there is no requantisation to do,
	 * the samples reach the sink bit-exact. The destination is the
	 * uncached alias, so this goes a frame at a time rather than a byte
	 * at a time.
	 */
	for (i = 0; i < avail; i += 4) {
		uint32_t off = readyptr + i;
		uint32_t w;

		w = (uint32_t)samples[off % size] |
		    ((uint32_t)samples[(off + 1) % size] << 8) |
		    ((uint32_t)samples[(off + 2) % size] << 16) |
		    ((uint32_t)samples[(off + 3) % size] << 24);

		if (vol < 100) {
			int32_t l = (int16_t)(w & 0xffff);
			int32_t r = (int16_t)(w >> 16);

			l = (l * vol) / 100;
			r = (r * vol) / 100;
			w = (uint32_t)(uint16_t)(int16_t)l |
			    ((uint32_t)(uint16_t)(int16_t)r << 16);
		}

		slot[i / 4] = w;
	}

	/* Underrun is genuine silence here; zero is silence on I2S. */
	for (i = avail; i < I2S_PERIOD_BYTES; i += 4)
		slot[i / 4] = 0;

	sifive_ccache_flush_range(sc->ring_pa +
	    (size_t)idx * I2S_PERIOD_BYTES, I2S_PERIOD_BYTES);

	i2s_periods++;
	if (avail < I2S_PERIOD_BYTES)
		i2s_underruns++;

	sc->play_ptr += avail;
	sc->play_ptr %= size;

	return (avail);
}

static int
jh7110_i2s_dai_intr(device_t dev, struct snd_dbuf *play_buf,
    struct snd_dbuf *rec_buf)
{
	struct jh7110_i2s_softc *sc = device_get_softc(dev);
	uint32_t pos, filled = 0, readyptr, ready, size;
	u_int target, n, k;

	if (play_buf == NULL || !sc->running || sc->ring == NULL)
		return (0);

	/*
	 * Write the period diametrically opposite the one being played: the
	 * engine does not stop, and this handler can be slow enough that
	 * CH_LLP has moved on by the time it is read.
	 */
	pos = jh7110_axidma_position(sc->dma);
	target = (pos / I2S_PERIOD_BYTES + I2S_NPERIODS / 2) % I2S_NPERIODS;

	I2S_LOCK(sc);

	size = play_buf->bufsize;
	readyptr = sndbuf_getreadyptr(play_buf);
	ready = sndbuf_getready(play_buf);

	if (sc->fill_idx >= I2S_NPERIODS) {
		sc->fill_idx = target;
		n = 1;
	} else {
		/*
		 * Chase the target. The DMAC's interrupt status is a level,
		 * not a count, so two periods finishing before this runs
		 * raise one interrupt -- writing only one period for them
		 * makes the ring drift.
		 */
		n = (target - sc->fill_idx + I2S_NPERIODS) % I2S_NPERIODS + 1;
		if (n > I2S_NPERIODS / 2)
			n = I2S_NPERIODS / 2;
	}

	for (k = 0; k < n; k++) {
		uint32_t took;

		took = jh7110_i2s_fill_slot(sc, play_buf, sc->fill_idx,
		    (readyptr + filled) % size, ready - filled);
		sc->fill_idx = (sc->fill_idx + 1) % I2S_NPERIODS;
		filled += took;
		if (took < I2S_PERIOD_BYTES)
			break;
	}

	I2S_UNLOCK(sc);

	return (filled > 0 ? AUDIO_DAI_PLAY_INTR : 0);
}

static uint32_t
jh7110_i2s_dai_get_ptr(device_t dev, int pcm_dir)
{
	struct jh7110_i2s_softc *sc = device_get_softc(dev);
	uint32_t ptr;

	I2S_LOCK(sc);
	ptr = sc->play_ptr;
	I2S_UNLOCK(sc);

	return (ptr);
}

static int
jh7110_i2s_dai_trigger(device_t dev, int go, int pcm_dir)
{
	struct jh7110_i2s_softc *sc = device_get_softc(dev);
	struct jh7110_dma_slave_config cfg;
	uint32_t i;
	int error;

	if (pcm_dir != PCMDIR_PLAY)
		return (EINVAL);

	switch (go) {
	case PCMTRIG_START:
		if (sc->dma == NULL) {
			sc->dma = jh7110_axidma_get(dev, "tx");
			if (sc->dma == NULL) {
				device_printf(dev, "no DMA channel\n");
				return (ENXIO);
			}
		}
		if (jh7110_i2s_ring_alloc(sc) != 0) {
			device_printf(dev, "cannot allocate DMA ring\n");
			return (ENOMEM);
		}

		for (i = 0; i < I2S_RING_BYTES / 4; i++)
			((volatile uint32_t *)sc->ring)[i] = 0;
		sifive_ccache_flush_range(sc->ring_pa, I2S_RING_BYTES);

		if (!sc->clk_running) {
			clk_enable(sc->clk_i2s);
			/*
			 * The bit and frame clocks the slave shifts against.
			 * Enabling the mux propagates to i2stx0_lrck_mst,
			 * which is where the gate actually is.
			 */
			if (sc->clk_bclk != NULL)
				clk_enable(sc->clk_bclk);
			if (sc->clk_lrck != NULL)
				clk_enable(sc->clk_lrck);
			sc->clk_running = 1;
		}

		I2S_LOCK(sc);
		sc->play_ptr = 0;
		sc->fill_idx = I2S_NPERIODS;	/* not anchored yet */
		sc->running = 1;
		i2s_periods = 0;
		i2s_underruns = 0;
		jh7110_i2s_hw_start(sc);
		I2S_UNLOCK(sc);

		cfg.dev_addr = rman_get_start(sc->res) + I2S_TXDMA;
		cfg.dev_width = 2;		/* one 16-bit sample */
		cfg.maxburst = 16;
		cfg.direction = JH7110_DMA_MEM_TO_DEV;

		error = jh7110_axidma_cyclic(sc->dma, &cfg, sc->ring_pa,
		    I2S_RING_BYTES, I2S_PERIOD_BYTES,
		    jh7110_i2s_period_cb, sc);
		if (error != 0) {
			device_printf(dev, "cannot start DMA: %d\n", error);
			I2S_LOCK(sc);
			sc->running = 0;
			I2S_WR(sc, I2S_CER, 0);
			I2S_WR(sc, I2S_ITER, 0);
			I2S_WR(sc, I2S_IER, 0);
			I2S_UNLOCK(sc);
			return (error);
		}
		break;

	case PCMTRIG_STOP:
	case PCMTRIG_ABORT:
		I2S_LOCK(sc);
		sc->running = 0;
		I2S_UNLOCK(sc);

		if (sc->dma != NULL)
			jh7110_axidma_stop(sc->dma);

		I2S_LOCK(sc);
		jh7110_i2s_hw_stop(sc);
		I2S_UNLOCK(sc);

		break;
	}

	return (0);
}

static device_method_t jh7110_i2s_methods[] = {
	DEVMETHOD(device_probe,			jh7110_i2s_probe),
	DEVMETHOD(device_attach,		jh7110_i2s_attach),
	DEVMETHOD(device_detach,		jh7110_i2s_detach),

	DEVMETHOD(audio_dai_init,		jh7110_i2s_dai_init),
	DEVMETHOD(audio_dai_setup_intr,		jh7110_i2s_dai_setup_intr),
	DEVMETHOD(audio_dai_set_sysclk,		jh7110_i2s_dai_set_sysclk),
	DEVMETHOD(audio_dai_set_chanspeed,	jh7110_i2s_dai_set_chanspeed),
	DEVMETHOD(audio_dai_set_chanformat,	jh7110_i2s_dai_set_chanformat),
	DEVMETHOD(audio_dai_intr,		jh7110_i2s_dai_intr),
	DEVMETHOD(audio_dai_get_caps,		jh7110_i2s_dai_get_caps),
	DEVMETHOD(audio_dai_get_ptr,		jh7110_i2s_dai_get_ptr),
	DEVMETHOD(audio_dai_trigger,		jh7110_i2s_dai_trigger),

	DEVMETHOD_END
};

static driver_t jh7110_i2s_driver = {
	"jh7110_i2s",
	jh7110_i2s_methods,
	sizeof(struct jh7110_i2s_softc),
};

DRIVER_MODULE(jh7110_i2s, simplebus, jh7110_i2s_driver, 0, 0);
SIMPLEBUS_PNP_INFO(compat_data);
MODULE_DEPEND(jh7110_i2s, sound, SOUND_MINVER, SOUND_PREFVER, SOUND_MAXVER);
