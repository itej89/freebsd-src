/*-
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * Copyright (c) 2026 Tej <itej89@github.com>
 *
 * Synopsys DesignWare AXI DMA controller, as instantiated on the StarFive
 * JH7110 ("starfive,jh7110-axi-dma", the dma1p block at 0x16050000).
 *
 * This exists because FreeBSD has no driver for it and the JH7110 audio
 * blocks cannot work without one. The PWMDAC has a two-register interface --
 * a data word and a control word, with no status register and no FIFO level
 * to poll -- so the only way to feed it at a stable sample rate is to let the
 * DMAC do it against the hardware handshake. Feeding it from a callout was
 * measured at 24.2 us per frame against the 20.83 us the rate demands: 16%
 * slow before jitter, which is audible as continuous white noise under the
 * programme material.
 *
 * Scope is deliberately narrow: cyclic slave transfers only. That covers the
 * PWMDAC and, later, I2S. There is no memory-to-memory path, no single-shot
 * scatter-gather, and no xdma(4) integration -- xdma's request/sglist model
 * is built around queued one-shot transfers and fits an audio ring badly.
 *
 * The register programming follows Linux's drivers/dma/dw-axi-dmac
 * deliberately closely, including the choices that look odd, because that is
 * the only configuration of this block known to work on this SoC. Where this
 * driver diverges the comment says why.
 *
 * Cache coherency: the DMAC is not coherent with the CPU caches and this SoC
 * has no Svpbmt, so an ordinary "uncached" mapping is silently cached. The
 * descriptor ring is therefore accessed through the SiFive L2 bypass alias
 * (sifive_ccache_uncached_offset()), which is the same technique the GPU and
 * VPU ports use. Consumers must do the same for the sample buffer itself.
 */

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/bus.h>
#include <sys/kernel.h>
#include <sys/lock.h>
#include <sys/malloc.h>
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

#include "jh7110_axidma.h"

/* ------------------------------------------------------------------ regs */

#define	COMMON_REG_LEN		0x100
#define	CHAN_REG_LEN		0x100

#define	DMAC_ID			0x000
#define	DMAC_COMPVER		0x008
#define	DMAC_CFG		0x010
#define	DMAC_CHEN		0x018
#define	DMAC_INTSTATUS		0x030
#define	DMAC_COMMON_INTCLEAR	0x038
#define	DMAC_CHABORTREG		0x028
#define	DMAC_RESET		0x058

#define	DMAC_EN_MASK		(1u << 0)
#define	DMAC_INT_EN_MASK	(1u << 1)

#define	DMAC_CHAN_EN_SHIFT	0
#define	DMAC_CHAN_EN_WE_SHIFT	8
#define	DMAC_CHAN_SUSP_SHIFT	16
#define	DMAC_CHAN_SUSP_WE_SHIFT	24

#define	CH_SAR			0x000
#define	CH_DAR			0x008
#define	CH_BLOCK_TS		0x010
#define	CH_CTL_L		0x018
#define	CH_CTL_H		0x01c
#define	CH_CFG_L		0x020
#define	CH_CFG_H		0x024
#define	CH_LLP			0x028
#define	CH_STATUS		0x030
#define	CH_INTSTATUS_ENA	0x080
#define	CH_INTSTATUS		0x088
#define	CH_INTSIGNAL_ENA	0x090
#define	CH_INTCLEAR		0x098

/*
 * CHx_CTL bit 58: raise BLOCK_TRF when this block finishes. Without it a
 * descriptor that is not LLI_LAST completes silently, which is why dropping
 * LLI_LAST left us with no period interrupt at all.
 */
#define	CH_CTL_H_IOC_BLKTFR	(1u << 26)
#define	CH_CTL_H_LLI_LAST	(1u << 30)
#define	CH_CTL_H_LLI_VALID	(1u << 31)
#define	CH_CTL_H_ARLEN_EN	(1u << 6)
#define	CH_CTL_H_ARLEN_POS	7
#define	CH_CTL_H_AWLEN_EN	(1u << 15)
#define	CH_CTL_H_AWLEN_POS	16

#define	CH_CTL_L_DST_MSIZE_POS	18
#define	CH_CTL_L_SRC_MSIZE_POS	14
#define	CH_CTL_L_DST_WIDTH_POS	11
#define	CH_CTL_L_SRC_WIDTH_POS	8
#define	CH_CTL_L_DST_INC_POS	6
#define	CH_CTL_L_SRC_INC_POS	4
#define	CH_CTL_L_DST_MAST	(1u << 2)
#define	CH_CTL_L_SRC_MAST	(1u << 0)

#define	DWAXIDMAC_CH_CTL_L_INC		0
#define	DWAXIDMAC_CH_CTL_L_NOINC	1

/* Burst length encoding: 0 => 1 item, 1 => 4, 2 => 8, 3 => 16, ... */
#define	DWAXIDMAC_BURST_TRANS_LEN_4	1

/*
 * This instantiation has four channels, so it uses the "<= 8 channels"
 * register map, but the DT match carries AXI_DMA_FLAG_USE_CFG2 which forces
 * the newer CFG2 field layout. Both facts matter and they are independent.
 */
#define	CH_CFG_L_DST_MULTBLK_TYPE_POS	2
#define	CH_CFG_L_SRC_MULTBLK_TYPE_POS	0
#define	CH_CFG2_L_SRC_PER_POS		4
#define	CH_CFG2_L_DST_PER_POS		11
#define	CH_CFG2_H_TT_FC_POS		0
#define	CH_CFG2_H_HS_SEL_SRC_POS	3
#define	CH_CFG2_H_HS_SEL_DST_POS	4
#define	CH_CFG2_H_PRIORITY_POS		20

#define	DWAXIDMAC_MBLK_TYPE_LL		3
#define	DWAXIDMAC_HS_SEL_HW		0

#define	DWAXIDMAC_TT_FC_MEM_TO_PER_DMAC	1
#define	DWAXIDMAC_TT_FC_PER_TO_MEM_DMAC	2

#define	DWAXIDMAC_IRQ_BLOCK_TRF		(1u << 0)
#define	DWAXIDMAC_IRQ_DMA_TRF		(1u << 1)
#define	DWAXIDMAC_IRQ_SUSPENDED		(1u << 29)
#define	DWAXIDMAC_IRQ_ALL		0xffffffffu
/* GENMASK(21,16) | GENMASK(14,5) */
#define	DWAXIDMAC_IRQ_ALL_ERR		(0x003f0000u | 0x00007fe0u)

#define	DWAXIDMAC_TRANS_WIDTH_32	2

#define	AXIDMA_MAX_CHANNELS	8
#define	AXIDMA_MAX_PERIODS	64

/* ------------------------------------------------------------------ desc */

/*
 * Linked list item. The layout is fixed by the hardware; it is 64 bytes and
 * every field is little-endian. RISC-V is little-endian so no swapping is
 * needed, but the packing must not change.
 */
struct axidma_lli {
	uint64_t	sar;
	uint64_t	dar;
	uint32_t	block_ts_lo;
	uint32_t	block_ts_hi;
	uint64_t	llp;
	uint32_t	ctl_lo;
	uint32_t	ctl_hi;
	uint32_t	sstat;
	uint32_t	dstat;
	uint32_t	status_lo;
	uint32_t	status_hi;
	uint32_t	reserved_lo;
	uint32_t	reserved_hi;
} __packed;
CTASSERT(sizeof(struct axidma_lli) == 64);

struct jh7110_axidma_softc;

struct jh7110_dma_chan {
	struct jh7110_axidma_softc *sc;
	int			id;
	bool			in_use;
	bool			running;

	/* Set from the consumer's "dmas" cell: the hardware handshake line. */
	u_int			handshake;

	bus_dma_tag_t		lli_tag;
	bus_dmamap_t		lli_map;
	void			*lli_cached;	/* mapping we must not use */
	volatile struct axidma_lli *lli;	/* L2 bypass alias */
	bus_addr_t		lli_pa;
	size_t			lli_sz;

	u_int			nperiods;
	uint32_t		period_len;
	int			previdx;

	void			(*cb)(void *);
	void			*cbarg;
};

struct jh7110_axidma_softc {
	device_t		dev;
	struct resource		*res;
	struct resource		*irq_res;
	void			*irq_cookie;
	struct mtx		mtx;
	clk_t			clk_core;
	clk_t			clk_cfgr;

	u_int			nchannels;
	u_int			nmasters;
	u_int			data_width;	/* log2 bytes */
	uint32_t		block_size[AXIDMA_MAX_CHANNELS];
	uint32_t		priority[AXIDMA_MAX_CHANNELS];
	bool			restrict_burst;
	uint32_t		burst_len;

	struct jh7110_dma_chan	chan[AXIDMA_MAX_CHANNELS];
};

/*
 * There is exactly one of these blocks and consumers reach it by phandle, so
 * a single global is enough and saves inventing a device-tree walk in every
 * consumer. Published at the end of attach; NULL means "not ready yet".
 */
static struct jh7110_axidma_softc *jh7110_axidma_sc;

/*
 * DIAGNOSTIC. The cyclic scheme restarts the channel at every period, so the
 * one thing worth being able to read back is the sequence of descriptor
 * indices the engine actually visits. A clean 0,1,2,...,n-1,0,... walk means
 * the ring is being traversed; anything else explains a stutter directly.
 */
SYSCTL_NODE(_hw, OID_AUTO, axidma, CTLFLAG_RD | CTLFLAG_MPSAFE, 0,
    "JH7110 AXI DMA controller");

/*
 * Walk the descriptor ring continuously instead of halting at every period
 * and restarting from the interrupt handler. See fix-refill.py: the restart
 * put the interrupt handler in the audio timing path, and the handler also
 * carries the refill, so any overrun of the 5.33 ms period became an audible
 * gap. Default on; 0 restores the old scheme for comparison.
 */
static int axidma_continuous = 1;
SYSCTL_INT(_hw_axidma, OID_AUTO, continuous, CTLFLAG_RWTUN,
    &axidma_continuous, 0,
    "walk the cyclic ring without halting at each period");

/*
 * Re-arm CH_CTL_H_LLI_VALID behind the engine. Only needed if this
 * instantiation clears the bit when it fetches a descriptor; if it does,
 * continuous mode stops after one lap and this is the fix. Off until there is
 * evidence it is wanted, because writing a descriptor the engine may be
 * prefetching is not free of risk.
 */
static int axidma_revalidate = 0;
SYSCTL_INT(_hw_axidma, OID_AUTO, revalidate, CTLFLAG_RWTUN,
    &axidma_revalidate, 0,
    "re-set LLI_VALID behind the engine in continuous mode");

static uint32_t axidma_blocks;
static uint32_t axidma_jumps;
static uint32_t axidma_irqs;
static uint32_t axidma_errors;
static uint32_t axidma_lasterr;
static uint8_t axidma_idxtrace[64];
static uint32_t axidma_ntrace;

SYSCTL_UINT(_hw_axidma, OID_AUTO, irqs, CTLFLAG_RD, &axidma_irqs, 0,
    "period interrupts taken");
SYSCTL_UINT(_hw_axidma, OID_AUTO, blocks, CTLFLAG_RD, &axidma_blocks, 0,
    "blocks the engine finished (exceeds irqs when they coalesce)");
SYSCTL_UINT(_hw_axidma, OID_AUTO, jumps, CTLFLAG_RD, &axidma_jumps, 0,
    "interrupts that covered more than one block");
SYSCTL_UINT(_hw_axidma, OID_AUTO, errors, CTLFLAG_RD, &axidma_errors, 0,
    "error interrupts taken");
SYSCTL_UINT(_hw_axidma, OID_AUTO, lasterr, CTLFLAG_RD, &axidma_lasterr, 0,
    "most recent error status word");

static uint64_t ch_read64(struct jh7110_dma_chan *ch, uint32_t off);

/*
 * Live descriptor index per channel. Reading it twice a second apart says
 * whether the engine is walking the ring, without needing anyone to listen.
 */
static int
axidma_llp_sysctl(SYSCTL_HANDLER_ARGS)
{
	struct jh7110_axidma_softc *sc = jh7110_axidma_sc;
	char buf[128];
	int off = 0;
	u_int i;

	buf[0] = 0;
	if (sc == NULL)
		return (sysctl_handle_string(oidp, buf, sizeof(buf), req));

	for (i = 0; i < sc->nchannels && off < (int)sizeof(buf) - 24; i++) {
		struct jh7110_dma_chan *ch = &sc->chan[i];
		uint64_t llp;
		int idx = -1;

		if (ch->running && ch->lli_pa != 0) {
			llp = ch_read64(ch, CH_LLP);
			if (llp >= ch->lli_pa)
				idx = (int)((llp - ch->lli_pa) /
				    sizeof(struct axidma_lli));
		}
		off += snprintf(buf + off, sizeof(buf) - off, "ch%u=%d ", i,
		    idx);
	}

	return (sysctl_handle_string(oidp, buf, sizeof(buf), req));
}

SYSCTL_PROC(_hw_axidma, OID_AUTO, llp,
    CTLTYPE_STRING | CTLFLAG_RD | CTLFLAG_MPSAFE, NULL, 0,
    axidma_llp_sysctl, "A",
    "live descriptor index per channel (-1 = idle)");

static int
axidma_idxtrace_sysctl(SYSCTL_HANDLER_ARGS)
{
	char buf[64 * 4 + 1];
	int i, n, off = 0;

	n = axidma_ntrace < nitems(axidma_idxtrace) ? axidma_ntrace :
	    nitems(axidma_idxtrace);
	for (i = 0; i < n && off < (int)sizeof(buf) - 4; i++)
		off += snprintf(buf + off, sizeof(buf) - off, "%u ",
		    axidma_idxtrace[i]);
	buf[off] = 0;

	return (sysctl_handle_string(oidp, buf, sizeof(buf), req));
}

SYSCTL_PROC(_hw_axidma, OID_AUTO, idxtrace,
    CTLTYPE_STRING | CTLFLAG_RD | CTLFLAG_MPSAFE, NULL, 0,
    axidma_idxtrace_sysctl, "A",
    "first descriptor indices the engine visited");

#define	DMA_LOCK(sc)		mtx_lock(&(sc)->mtx)
#define	DMA_UNLOCK(sc)		mtx_unlock(&(sc)->mtx)
#define	RD4(sc, off)		bus_read_4((sc)->res, (off))
#define	WR4(sc, off, v)		bus_write_4((sc)->res, (off), (v))
#define	CH_RD4(ch, off)							\
	bus_read_4((ch)->sc->res, COMMON_REG_LEN +			\
	    (ch)->id * CHAN_REG_LEN + (off))
#define	CH_WR4(ch, off, v)						\
	bus_write_4((ch)->sc->res, COMMON_REG_LEN +			\
	    (ch)->id * CHAN_REG_LEN + (off), (v))

static struct ofw_compat_data compat_data[] = {
	{ "starfive,jh7110-axi-dma",	1 },
	{ NULL,				0 }
};

/* ---------------------------------------------------------------- helpers */

/*
 * Two 32-bit accesses, low half first: some instantiations of this block do
 * not accept a 64-bit access, and writing the low half first is what arms the
 * register pair.
 */
static void
ch_write64(struct jh7110_dma_chan *ch, uint32_t off, uint64_t val)
{

	CH_WR4(ch, off, (uint32_t)val);
	CH_WR4(ch, off + 4, (uint32_t)(val >> 32));
}

static uint64_t
ch_read64(struct jh7110_dma_chan *ch, uint32_t off)
{
	uint32_t lo, hi;

	lo = CH_RD4(ch, off);
	hi = CH_RD4(ch, off + 4);
	return (((uint64_t)hi << 32) | lo);
}

static void
axidma_enable(struct jh7110_axidma_softc *sc)
{

	WR4(sc, DMAC_CFG, RD4(sc, DMAC_CFG) | DMAC_EN_MASK);
}

static void
axidma_irq_enable(struct jh7110_axidma_softc *sc)
{

	WR4(sc, DMAC_CFG, RD4(sc, DMAC_CFG) | DMAC_INT_EN_MASK);
}

static void
axidma_irq_disable(struct jh7110_axidma_softc *sc)
{

	WR4(sc, DMAC_CFG, RD4(sc, DMAC_CFG) & ~DMAC_INT_EN_MASK);
}

/*
 * DMAC_CHEN is write-enable protected: a bit only takes effect if the
 * matching "write enable" bit is set in the same store. With four channels
 * the write-enable bits live at +8 (the <= 8 channel map).
 */
static void
axidma_chan_enable(struct jh7110_dma_chan *ch)
{
	struct jh7110_axidma_softc *sc = ch->sc;
	uint32_t val;

	val = RD4(sc, DMAC_CHEN);
	val |= (1u << ch->id) << DMAC_CHAN_EN_SHIFT;
	val |= (1u << ch->id) << DMAC_CHAN_EN_WE_SHIFT;
	WR4(sc, DMAC_CHEN, val);
}

static void
axidma_chan_disable(struct jh7110_dma_chan *ch)
{
	struct jh7110_axidma_softc *sc = ch->sc;
	uint32_t val;

	val = RD4(sc, DMAC_CHEN);
	val &= ~((1u << ch->id) << DMAC_CHAN_EN_SHIFT);
	val |= (1u << ch->id) << DMAC_CHAN_EN_WE_SHIFT;
	WR4(sc, DMAC_CHEN, val);
}

static bool
axidma_chan_is_enabled(struct jh7110_dma_chan *ch)
{

	return ((RD4(ch->sc, DMAC_CHEN) &
	    ((1u << ch->id) << DMAC_CHAN_EN_SHIFT)) != 0);
}

/* ------------------------------------------------------------ descriptors */

static void
axidma_dmamap_cb(void *arg, bus_dma_segment_t *segs, int nseg, int error)
{

	if (error != 0)
		return;
	KASSERT(nseg == 1, ("axidma: descriptor ring not contiguous"));
	*(bus_addr_t *)arg = segs[0].ds_addr;
}

static int
axidma_alloc_lli(struct jh7110_dma_chan *ch)
{
	struct jh7110_axidma_softc *sc = ch->sc;
	uint64_t off;
	int error;

	ch->lli_sz = AXIDMA_MAX_PERIODS * sizeof(struct axidma_lli);

	error = bus_dma_tag_create(bus_get_dma_tag(sc->dev),
	    64, 0,				/* alignment, boundary */
	    BUS_SPACE_MAXADDR, BUS_SPACE_MAXADDR,
	    NULL, NULL,
	    ch->lli_sz, 1, ch->lli_sz,
	    0, NULL, NULL, &ch->lli_tag);
	if (error != 0) {
		device_printf(sc->dev, "ch%d: cannot create LLI tag\n", ch->id);
		return (error);
	}

	error = bus_dmamem_alloc(ch->lli_tag, &ch->lli_cached,
	    BUS_DMA_WAITOK | BUS_DMA_ZERO | BUS_DMA_COHERENT, &ch->lli_map);
	if (error != 0) {
		device_printf(sc->dev, "ch%d: cannot alloc LLI ring\n", ch->id);
		return (error);
	}

	error = bus_dmamap_load(ch->lli_tag, ch->lli_map, ch->lli_cached,
	    ch->lli_sz, axidma_dmamap_cb, &ch->lli_pa, BUS_DMA_NOWAIT);
	if (error != 0 || ch->lli_pa == 0) {
		device_printf(sc->dev, "ch%d: cannot map LLI ring\n", ch->id);
		return (error != 0 ? error : ENOMEM);
	}

	/*
	 * bus_dmamem_alloc() zeroed the ring through the cached mapping, so
	 * push that out before anyone looks at DRAM through the alias.
	 */
	sifive_ccache_flush_range(ch->lli_pa, ch->lli_sz);

	/*
	 * Everything after this point touches the ring through the L2 bypass
	 * alias. Writing descriptors through the cached mapping would leave
	 * them sitting in L2 where the DMAC cannot see them, and the ISR
	 * rewrites a descriptor on every period, so a flush-per-update would
	 * be both slow and easy to get wrong.
	 */
	off = sifive_ccache_uncached_offset();
	if (off == 0) {
		device_printf(sc->dev,
		    "ch%d: no L2 bypass alias; refusing to run uncoherently\n",
		    ch->id);
		return (ENXIO);
	}
	ch->lli = pmap_mapdev((vm_paddr_t)ch->lli_pa + off, ch->lli_sz);
	if (ch->lli == NULL) {
		device_printf(sc->dev, "ch%d: cannot map LLI alias\n", ch->id);
		return (ENOMEM);
	}

	return (0);
}

static void
axidma_free_lli(struct jh7110_dma_chan *ch)
{

	if (ch->lli != NULL) {
		pmap_unmapdev(__DEVOLATILE(void *, ch->lli), ch->lli_sz);
		ch->lli = NULL;
	}
	if (ch->lli_pa != 0) {
		bus_dmamap_unload(ch->lli_tag, ch->lli_map);
		ch->lli_pa = 0;
	}
	if (ch->lli_cached != NULL) {
		bus_dmamem_free(ch->lli_tag, ch->lli_cached, ch->lli_map);
		ch->lli_cached = NULL;
	}
	if (ch->lli_tag != NULL) {
		bus_dma_tag_destroy(ch->lli_tag);
		ch->lli_tag = NULL;
	}
}

/* ------------------------------------------------------------------- init */

static void
axidma_hw_init(struct jh7110_axidma_softc *sc)
{
	u_int i;

	WR4(sc, DMAC_RESET, 1);
	while ((RD4(sc, DMAC_RESET) & 1) != 0)
		DELAY(10);

	for (i = 0; i < sc->nchannels; i++) {
		struct jh7110_dma_chan *ch = &sc->chan[i];

		CH_WR4(ch, CH_INTSTATUS_ENA, 0);
		CH_WR4(ch, CH_INTCLEAR, DWAXIDMAC_IRQ_ALL);
		axidma_chan_disable(ch);
	}
}

/* -------------------------------------------------------------- interrupt */

static void
axidma_intr(void *arg)
{
	struct jh7110_axidma_softc *sc = arg;
	void (*cbs[AXIDMA_MAX_CHANNELS])(void *);
	void *cbargs[AXIDMA_MAX_CHANNELS];
	u_int i, ncb = 0;

	DMA_LOCK(sc);

	/*
	 * Mask at the controller for the duration, exactly as Linux does: the
	 * per-channel status bits are cleared one at a time and a new edge
	 * arriving mid-sweep would otherwise be lost.
	 */
	axidma_irq_disable(sc);

	for (i = 0; i < sc->nchannels; i++) {
		struct jh7110_dma_chan *ch = &sc->chan[i];
		uint32_t status;
		uint64_t llp;
		u_int idx;

		status = CH_RD4(ch, CH_INTSTATUS);
		if (status == 0)
			continue;
		CH_WR4(ch, CH_INTCLEAR, status);

		if ((status & DWAXIDMAC_IRQ_ALL_ERR) != 0) {
			axidma_errors++;
			axidma_lasterr = status;
			if (axidma_errors < 8)
				device_printf(sc->dev, "ch%d: error %#x\n", i,
				    status);
			axidma_chan_disable(ch);
			ch->running = false;
			continue;
		}

		if ((status & (DWAXIDMAC_IRQ_DMA_TRF |
		    DWAXIDMAC_IRQ_BLOCK_TRF)) == 0 || !ch->running)
			continue;

		/*
		 * CH_LLP names the descriptor the engine has moved on to.
		 * Record the walk so it can be read back, and in the legacy
		 * halt-per-period mode re-arm that descriptor and restart the
		 * channel. In continuous mode there is nothing to restart --
		 * the engine never stopped -- so a late interrupt costs
		 * freshness, not silence.
		 */
		llp = ch_read64(ch, CH_LLP);
		if (llp >= ch->lli_pa &&
		    llp < ch->lli_pa + ch->nperiods * sizeof(struct axidma_lli)) {
			idx = (u_int)((llp - ch->lli_pa) /
			    sizeof(struct axidma_lli));
			if (!axidma_continuous || axidma_revalidate) {
				u_int v = axidma_continuous ?
				    (idx + ch->nperiods - 2) % ch->nperiods :
				    idx;

				ch->lli[v].ctl_hi |= CH_CTL_H_LLI_VALID;
			}
			/*
			 * CH_INTSTATUS is a level, so one interrupt can cover
			 * several blocks. The distance CH_LLP moved is the
			 * real count.
			 */
			if (ch->previdx >= 0) {
				u_int d = (idx - ch->previdx +
				    ch->nperiods) % ch->nperiods;

				axidma_blocks += d;
				if (d != 1)
					axidma_jumps++;
			}
			ch->previdx = (int)idx;

			if (axidma_ntrace < nitems(axidma_idxtrace))
				axidma_idxtrace[axidma_ntrace] = idx;
			axidma_ntrace++;
		}
		axidma_irqs++;

		if (!axidma_continuous)
			axidma_chan_enable(ch);

		if (ch->cb != NULL) {
			cbs[ncb] = ch->cb;
			cbargs[ncb] = ch->cbarg;
			ncb++;
		}
	}

	axidma_irq_enable(sc);
	DMA_UNLOCK(sc);

	/*
	 * Consumer callbacks run without our lock. The sound layer takes its
	 * own locks in chn_intr() and calls back into us for the position, so
	 * holding the controller lock across them would invert the order.
	 */
	for (i = 0; i < ncb; i++)
		cbs[i](cbargs[i]);
}

/* ----------------------------------------------------------- consumer API */

struct jh7110_dma_chan *
jh7110_axidma_get(device_t consumer, const char *name)
{
	struct jh7110_axidma_softc *sc = jh7110_axidma_sc;
	struct jh7110_dma_chan *ch = NULL;
	phandle_t node, xref;
	pcell_t *cells;
	int idx, ncells, error;
	u_int i;

	if (sc == NULL)
		return (NULL);

	node = ofw_bus_get_node(consumer);
	if (node <= 0)
		return (NULL);

	idx = 0;
	if (name != NULL &&
	    ofw_bus_find_string_index(node, "dma-names", name, &idx) != 0)
		return (NULL);

	error = ofw_bus_parse_xref_list_alloc(node, "dmas", "#dma-cells", idx,
	    &xref, &ncells, &cells);
	if (error != 0)
		return (NULL);

	if (OF_node_from_xref(xref) != ofw_bus_get_node(sc->dev) ||
	    ncells < 1) {
		OF_prop_free(cells);
		return (NULL);
	}

	DMA_LOCK(sc);
	for (i = 0; i < sc->nchannels; i++) {
		if (!sc->chan[i].in_use) {
			ch = &sc->chan[i];
			ch->in_use = true;
			ch->handshake = cells[0];
			break;
		}
	}
	DMA_UNLOCK(sc);

	OF_prop_free(cells);

	if (ch == NULL) {
		device_printf(sc->dev, "no free channel for %s\n",
		    device_get_nameunit(consumer));
		return (NULL);
	}

	if (ch->lli == NULL && axidma_alloc_lli(ch) != 0) {
		axidma_free_lli(ch);
		DMA_LOCK(sc);
		ch->in_use = false;
		DMA_UNLOCK(sc);
		return (NULL);
	}

	if (bootverbose)
		device_printf(sc->dev, "ch%d -> %s (handshake %u)\n", ch->id,
		    device_get_nameunit(consumer), ch->handshake);

	return (ch);
}

void
jh7110_axidma_put(struct jh7110_dma_chan *ch)
{

	if (ch == NULL)
		return;
	jh7110_axidma_stop(ch);
	DMA_LOCK(ch->sc);
	ch->in_use = false;
	DMA_UNLOCK(ch->sc);
}

int
jh7110_axidma_cyclic(struct jh7110_dma_chan *ch,
    const struct jh7110_dma_slave_config *cfg, bus_addr_t buf_pa,
    size_t buf_len, size_t period_len, void (*cb)(void *), void *cbarg)
{
	struct jh7110_axidma_softc *sc;
	uint32_t ctl_lo, ctl_hi, cfg_lo, cfg_hi, irq_mask;
	u_int mem_width, reg_width, dst_per, src_per, tt_fc;
	size_t block_ts;
	u_int i, nperiods;

	if (ch == NULL || cfg == NULL)
		return (EINVAL);
	sc = ch->sc;

	if (period_len == 0 || buf_len == 0 || (buf_len % period_len) != 0)
		return (EINVAL);
	nperiods = buf_len / period_len;
	if (nperiods < 2 || nperiods > AXIDMA_MAX_PERIODS)
		return (EINVAL);
	if ((buf_pa & 3) != 0 || (period_len & 3) != 0)
		return (EINVAL);
	if (cfg->dev_width == 0 || !powerof2(cfg->dev_width) ||
	    cfg->dev_width > 8)
		return (EINVAL);

	/*
	 * Transfer width for the memory side is the largest power of two that
	 * divides the bus width, the address and the length -- capped at 32
	 * bits, which is as wide as this block is programmed for here.
	 */
	mem_width = ffs((1u << sc->data_width) | (u_int)buf_pa |
	    (u_int)period_len) - 1;
	if (mem_width > DWAXIDMAC_TRANS_WIDTH_32)
		mem_width = DWAXIDMAC_TRANS_WIDTH_32;
	reg_width = ffs(cfg->dev_width) - 1;

	dst_per = src_per = 0;
	switch (cfg->direction) {
	case JH7110_DMA_MEM_TO_DEV:
		tt_fc = DWAXIDMAC_TT_FC_MEM_TO_PER_DMAC;
		dst_per = ch->handshake;
		ctl_lo = (reg_width << CH_CTL_L_DST_WIDTH_POS) |
		    (mem_width << CH_CTL_L_SRC_WIDTH_POS) |
		    (DWAXIDMAC_CH_CTL_L_NOINC << CH_CTL_L_DST_INC_POS) |
		    (DWAXIDMAC_CH_CTL_L_INC << CH_CTL_L_SRC_INC_POS);
		block_ts = period_len >> mem_width;
		break;
	case JH7110_DMA_DEV_TO_MEM:
		tt_fc = DWAXIDMAC_TT_FC_PER_TO_MEM_DMAC;
		src_per = ch->handshake;
		ctl_lo = (reg_width << CH_CTL_L_SRC_WIDTH_POS) |
		    (mem_width << CH_CTL_L_DST_WIDTH_POS) |
		    (DWAXIDMAC_CH_CTL_L_INC << CH_CTL_L_DST_INC_POS) |
		    (DWAXIDMAC_CH_CTL_L_NOINC << CH_CTL_L_SRC_INC_POS);
		block_ts = period_len >> reg_width;
		break;
	default:
		return (EINVAL);
	}

	if (block_ts == 0 || block_ts > sc->block_size[ch->id])
		return (EINVAL);

	/*
	 * Burst size is hardcoded at 4 items, matching Linux. The consumer's
	 * maxburst is advisory: on a FIFO one word deep (the PWMDAC) a longer
	 * burst only lengthens the handshake, it does not help.
	 */
	ctl_lo |= (DWAXIDMAC_BURST_TRANS_LEN_4 << CH_CTL_L_DST_MSIZE_POS) |
	    (DWAXIDMAC_BURST_TRANS_LEN_4 << CH_CTL_L_SRC_MSIZE_POS);

	/* One AXI master on this SoC, so both sides use master 0. */
	ctl_lo &= ~(CH_CTL_L_SRC_MAST | CH_CTL_L_DST_MAST);

	/*
	 * LLI_LAST halts the channel at the end of every block. That is what
	 * Linux does, and it is what put the interrupt handler in the audio
	 * timing path; without it the engine simply follows the circular list
	 * for ever. See fix-refill.py.
	 */
	ctl_hi = CH_CTL_H_LLI_VALID;
	if (axidma_continuous)
		ctl_hi |= CH_CTL_H_IOC_BLKTFR;
	else
		ctl_hi |= CH_CTL_H_LLI_LAST;
	if (sc->restrict_burst) {
		ctl_hi |= CH_CTL_H_ARLEN_EN | CH_CTL_H_AWLEN_EN |
		    (sc->burst_len << CH_CTL_H_ARLEN_POS) |
		    (sc->burst_len << CH_CTL_H_AWLEN_POS);
	}

	jh7110_axidma_stop(ch);

	DMA_LOCK(sc);

	ch->nperiods = nperiods;
	ch->period_len = period_len;
	ch->previdx = -1;
	axidma_blocks = 0;
	axidma_jumps = 0;
	axidma_irqs = 0;
	axidma_ntrace = 0;
	ch->cb = cb;
	ch->cbarg = cbarg;

	for (i = 0; i < nperiods; i++) {
		volatile struct axidma_lli *lli = &ch->lli[i];
		bus_addr_t mem = buf_pa + (bus_addr_t)i * period_len;

		if (cfg->direction == JH7110_DMA_MEM_TO_DEV) {
			lli->sar = mem;
			lli->dar = cfg->dev_addr;
		} else {
			lli->sar = cfg->dev_addr;
			lli->dar = mem;
		}
		lli->block_ts_lo = block_ts - 1;
		lli->block_ts_hi = 0;
		lli->ctl_lo = ctl_lo;
		lli->ctl_hi = ctl_hi;
		lli->status_lo = 0;
		lli->status_hi = 0;
		/* Close the ring: the last item points back at the first. */
		lli->llp = ch->lli_pa + (bus_addr_t)((i + 1) % nperiods) *
		    sizeof(struct axidma_lli);
	}

	cfg_lo = (DWAXIDMAC_MBLK_TYPE_LL << CH_CFG_L_DST_MULTBLK_TYPE_POS) |
	    (DWAXIDMAC_MBLK_TYPE_LL << CH_CFG_L_SRC_MULTBLK_TYPE_POS) |
	    (src_per << CH_CFG2_L_SRC_PER_POS) |
	    (dst_per << CH_CFG2_L_DST_PER_POS);
	cfg_hi = (tt_fc << CH_CFG2_H_TT_FC_POS) |
	    (DWAXIDMAC_HS_SEL_HW << CH_CFG2_H_HS_SEL_SRC_POS) |
	    (DWAXIDMAC_HS_SEL_HW << CH_CFG2_H_HS_SEL_DST_POS) |
	    (sc->priority[ch->id] << CH_CFG2_H_PRIORITY_POS);

	axidma_enable(sc);

	CH_WR4(ch, CH_CFG_L, cfg_lo);
	CH_WR4(ch, CH_CFG_H, cfg_hi);

	/* Low bit of LLP selects the master used to fetch the list: AXI0. */
	ch_write64(ch, CH_LLP, ch->lli_pa);

	/*
	 * With no LLI_LAST there is no "transfer complete", only block
	 * boundaries, so the period signal has to come from BLOCK_TRF.
	 */
	irq_mask = axidma_continuous ? DWAXIDMAC_IRQ_BLOCK_TRF :
	    DWAXIDMAC_IRQ_DMA_TRF;

	CH_WR4(ch, CH_INTSIGNAL_ENA, irq_mask | DWAXIDMAC_IRQ_ALL_ERR);
	CH_WR4(ch, CH_INTSTATUS_ENA, irq_mask | DWAXIDMAC_IRQ_ALL_ERR |
	    DWAXIDMAC_IRQ_SUSPENDED);

	ch->running = true;
	axidma_irq_enable(sc);
	axidma_chan_enable(ch);

	DMA_UNLOCK(sc);

	return (0);
}

int
jh7110_axidma_stop(struct jh7110_dma_chan *ch)
{
	struct jh7110_axidma_softc *sc;
	int timeout;

	if (ch == NULL)
		return (EINVAL);
	sc = ch->sc;

	DMA_LOCK(sc);
	ch->running = false;
	ch->cb = NULL;
	axidma_chan_disable(ch);
	DMA_UNLOCK(sc);

	for (timeout = 5000; timeout > 0; timeout--) {
		if (!axidma_chan_is_enabled(ch))
			break;
		DELAY(10);
	}
	if (timeout == 0) {
		/*
		 * The engine will not clear CHEN while a transfer is
		 * outstanding, so a channel waiting on a handshake that never
		 * arrives is stuck for good. Abort it: without this the first
		 * failure poisons every later start, which then writes the
		 * channel's registers while it is still live and reports
		 * WRONCHEN.
		 */
		device_printf(sc->dev, "ch%d: will not stop; aborting\n",
		    ch->id);
		WR4(sc, DMAC_CHABORTREG, 1u << ch->id);

		for (timeout = 1000; timeout > 0; timeout--) {
			if (!axidma_chan_is_enabled(ch))
				break;
			DELAY(10);
		}
		if (timeout == 0)
			device_printf(sc->dev, "ch%d: abort failed too\n",
			    ch->id);
	}

	CH_WR4(ch, CH_INTSTATUS_ENA, 0);
	CH_WR4(ch, CH_INTCLEAR, DWAXIDMAC_IRQ_ALL);

	return (0);
}

uint32_t
jh7110_axidma_position(struct jh7110_dma_chan *ch)
{
	uint64_t llp;
	u_int idx;

	if (ch == NULL || !ch->running)
		return (0);

	llp = ch_read64(ch, CH_LLP);
	if (llp < ch->lli_pa ||
	    llp >= ch->lli_pa + ch->nperiods * sizeof(struct axidma_lli))
		return (0);

	idx = (u_int)((llp - ch->lli_pa) / sizeof(struct axidma_lli));

	/*
	 * CH_LLP names the descriptor in flight, so everything before it is
	 * certainly consumed and everything in it might not be. Reporting the
	 * start of the current period is the conservative answer, which is
	 * the one the sound layer wants.
	 */
	return (idx * ch->period_len);
}

/* ------------------------------------------------------------- newbus glue */

static int
jh7110_axidma_probe(device_t dev)
{

	if (!ofw_bus_status_okay(dev))
		return (ENXIO);
	if (!ofw_bus_search_compatible(dev, compat_data)->ocd_data)
		return (ENXIO);

	device_set_desc(dev, "StarFive JH7110 AXI DMA controller");
	return (BUS_PROBE_DEFAULT);
}

static int
jh7110_axidma_parse_fdt(struct jh7110_axidma_softc *sc)
{
	phandle_t node = ofw_bus_get_node(sc->dev);
	pcell_t cells[AXIDMA_MAX_CHANNELS];
	pcell_t val;
	u_int i;

	if (OF_getencprop(node, "dma-channels", &val, sizeof(val)) <= 0)
		return (ENXIO);
	if (val == 0 || val > AXIDMA_MAX_CHANNELS) {
		device_printf(sc->dev, "unsupported channel count %u\n", val);
		return (ENXIO);
	}
	sc->nchannels = val;

	if (OF_getencprop(node, "snps,dma-masters", &val, sizeof(val)) <= 0)
		return (ENXIO);
	sc->nmasters = val;

	if (OF_getencprop(node, "snps,data-width", &val, sizeof(val)) <= 0)
		return (ENXIO);
	sc->data_width = val;

	if (OF_getencprop(node, "snps,block-size", cells,
	    sc->nchannels * sizeof(pcell_t)) <= 0)
		return (ENXIO);
	for (i = 0; i < sc->nchannels; i++)
		sc->block_size[i] = cells[i];

	if (OF_getencprop(node, "snps,priority", cells,
	    sc->nchannels * sizeof(pcell_t)) <= 0)
		return (ENXIO);
	for (i = 0; i < sc->nchannels; i++)
		sc->priority[i] = cells[i];

	/*
	 * Optional. Note the value goes into ARLEN/AWLEN as written, which is
	 * what Linux does even though those fields encode "beats - 1"; keeping
	 * the same value keeps the same proven behaviour.
	 */
	if (OF_getencprop(node, "snps,axi-max-burst-len", &val,
	    sizeof(val)) > 0 && val >= 1 && val <= 256) {
		sc->restrict_burst = true;
		sc->burst_len = val;
	}

	return (0);
}

static int
jh7110_axidma_attach(device_t dev)
{
	struct jh7110_axidma_softc *sc = device_get_softc(dev);
	hwreset_t rst;
	int rid, error;
	u_int i;

	sc->dev = dev;
	mtx_init(&sc->mtx, device_get_nameunit(dev), NULL, MTX_DEF);

	error = jh7110_axidma_parse_fdt(sc);
	if (error != 0) {
		device_printf(dev, "missing or bad DT properties\n");
		goto fail;
	}

	rid = 0;
	sc->res = bus_alloc_resource_any(dev, SYS_RES_MEMORY, &rid, RF_ACTIVE);
	if (sc->res == NULL) {
		device_printf(dev, "cannot allocate registers\n");
		error = ENXIO;
		goto fail;
	}

	if (clk_get_by_ofw_name(dev, 0, "core-clk", &sc->clk_core) == 0)
		clk_enable(sc->clk_core);
	if (clk_get_by_ofw_name(dev, 0, "cfgr-clk", &sc->clk_cfgr) == 0)
		clk_enable(sc->clk_cfgr);

	for (i = 0; i < 2; i++) {
		if (hwreset_get_by_ofw_idx(dev, 0, i, &rst) == 0)
			hwreset_deassert(rst);
	}

	for (i = 0; i < sc->nchannels; i++) {
		sc->chan[i].sc = sc;
		sc->chan[i].id = i;
	}

	axidma_hw_init(sc);

	rid = 0;
	sc->irq_res = bus_alloc_resource_any(dev, SYS_RES_IRQ, &rid, RF_ACTIVE);
	if (sc->irq_res == NULL) {
		device_printf(dev, "cannot allocate interrupt\n");
		error = ENXIO;
		goto fail;
	}
	error = bus_setup_intr(dev, sc->irq_res, INTR_TYPE_AV | INTR_MPSAFE,
	    NULL, axidma_intr, sc, &sc->irq_cookie);
	if (error != 0) {
		device_printf(dev, "cannot set up interrupt\n");
		goto fail;
	}

	if (bootverbose)
		device_printf(dev, "%u channels, id %#x, comp %#x\n",
		    sc->nchannels, RD4(sc, DMAC_ID), RD4(sc, DMAC_COMPVER));

	jh7110_axidma_sc = sc;
	return (0);

fail:
	if (sc->irq_cookie != NULL)
		bus_teardown_intr(dev, sc->irq_res, sc->irq_cookie);
	if (sc->irq_res != NULL)
		bus_release_resource(dev, SYS_RES_IRQ, 0, sc->irq_res);
	if (sc->res != NULL)
		bus_release_resource(dev, SYS_RES_MEMORY, 0, sc->res);
	mtx_destroy(&sc->mtx);
	return (error);
}

static int
jh7110_axidma_detach(device_t dev)
{
	struct jh7110_axidma_softc *sc = device_get_softc(dev);
	u_int i;

	for (i = 0; i < sc->nchannels; i++) {
		if (sc->chan[i].in_use)
			return (EBUSY);
	}

	jh7110_axidma_sc = NULL;

	for (i = 0; i < sc->nchannels; i++)
		axidma_free_lli(&sc->chan[i]);

	if (sc->irq_cookie != NULL)
		bus_teardown_intr(dev, sc->irq_res, sc->irq_cookie);
	if (sc->irq_res != NULL)
		bus_release_resource(dev, SYS_RES_IRQ, 0, sc->irq_res);
	if (sc->res != NULL)
		bus_release_resource(dev, SYS_RES_MEMORY, 0, sc->res);
	mtx_destroy(&sc->mtx);

	return (0);
}

static device_method_t jh7110_axidma_methods[] = {
	DEVMETHOD(device_probe,		jh7110_axidma_probe),
	DEVMETHOD(device_attach,	jh7110_axidma_attach),
	DEVMETHOD(device_detach,	jh7110_axidma_detach),
	DEVMETHOD_END
};

DEFINE_CLASS_0(jh7110_axidma, jh7110_axidma_driver, jh7110_axidma_methods,
    sizeof(struct jh7110_axidma_softc));
EARLY_DRIVER_MODULE(jh7110_axidma, simplebus, jh7110_axidma_driver, 0, 0,
    BUS_PASS_INTERRUPT + BUS_PASS_ORDER_LATE);
