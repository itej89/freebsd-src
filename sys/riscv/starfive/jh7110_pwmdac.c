/*-
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * Copyright (c) 2026 Tej <itej89@github.com>
 *
 * PWMDAC audio driver for StarFive JH7110 SoC.
 * Outputs audio via PWM on GPIO33 (left) and GPIO34 (right)
 * through external RC filters to a 3.5mm jack.
 *
 * Based on Linux sound/soc/starfive/starfive_pwmdac.c
 */

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/bus.h>
#include <sys/callout.h>
#include <sys/sysctl.h>
#include <sys/firmware.h>
#include <sys/kernel.h>
#include <sys/lock.h>
#include <sys/module.h>
#include <sys/mutex.h>
#include <sys/rman.h>

#include <machine/bus.h>

#include <vm/vm.h>
#include <vm/pmap.h>

#include <dev/ofw/ofw_bus.h>
#include <dev/ofw/ofw_bus_subr.h>

#include <dev/clk/clk.h>
#include <dev/hwreset/hwreset.h>

#include <riscv/sifive/sifive_ccache.h>

#include "jh7110_axidma.h"

#include "opt_snd.h"
#include <dev/sound/pcm/sound.h>
#include <dev/sound/fdt/audio_dai.h>
#include "audio_dai_if.h"

/* PWMDAC registers */
#define	PWMDAC_WDATA		0x00
#define	PWMDAC_CTRL		0x04
#define	PWMDAC_SATAE		0x08

/* CTRL register bits */
#define	CTRL_ENABLE		(1 << 0)
#define	CTRL_SHIFT_8BIT		(0 << 1)
#define	CTRL_SHIFT_10BIT	(1 << 1)
#define	CTRL_DUTY_LEFT		(0 << 2)
#define	CTRL_DUTY_RIGHT		(1 << 2)
#define	CTRL_DUTY_CENTER	(2 << 2)
#define	CTRL_CNT_N_SHIFT	4
#define	CTRL_CNT_N_MASK		(0x1ff << 4)
#define	CTRL_DATA_CHANGE	(1 << 13)
#define	CTRL_DATA_MODE_INV	(1 << 14)
#define	CTRL_DATA_SHIFT_SHIFT	15

static uint32_t jh7110_pwmdac_fmts[] = {
	SND_FORMAT(AFMT_S16_LE, 2, 0),
	0,
};

static struct pcmchan_caps jh7110_pwmdac_caps = {
	8000, 48000, jh7110_pwmdac_fmts, 0,
};

/*
 * The DMA ring. Eight periods of 1 KiB is 256 stereo frames each: 5.33 ms per
 * period at 48 kHz, so 187 interrupts a second and 42 ms of buffering. Small
 * enough that a late interrupt is recoverable, large enough that the
 * interrupt rate is not itself a cost.
 */
#define	PWMDAC_STREAM_PERIOD	1024	/* stream bytes per period */
#define	PWMDAC_MAX_OSR		8
#define	PWMDAC_PERIOD_BYTES	(PWMDAC_STREAM_PERIOD * PWMDAC_MAX_OSR)
#define	PWMDAC_NPERIODS		16
#define	PWMDAC_RING_BYTES	(PWMDAC_PERIOD_BYTES * PWMDAC_NPERIODS)

struct jh7110_pwmdac_softc {
	device_t		dev;
	struct resource		*res;
	struct mtx		mtx;
	clk_t			clk_apb;
	clk_t			clk_core;
	uint32_t		play_ptr;
	uint32_t		speed;
	driver_intr_t		*intr_handler;
	void			*intr_arg;
	int			running;

	/*
	 * Acquired lazily on the first PCMTRIG_START, never in attach: the
	 * DMAC is an ordinary simplebus child and may attach after us.
	 */
	struct jh7110_dma_chan	*dma;

	bus_dma_tag_t		ring_tag;
	bus_dmamap_t		ring_map;
	void			*ring_cached;	/* mapping we must not use */
	volatile uint8_t	*ring;		/* L2 bypass alias */
	bus_addr_t		ring_pa;

	/*
	 * Next ring slot the refill path will write. PWMDAC_NPERIODS means
	 * "not anchored yet"; the first interrupt after a start sets it.
	 */
	u_int			fill_idx;

	/*
	 * Oversampling makes the period length a runtime value: one period is
	 * PWMDAC_STREAM_PERIOD bytes of stream either way, but 1x and 4x turn
	 * that into different amounts of ring.
	 */
	u_int			osr;
	uint32_t		period_bytes;
	uint32_t		ring_bytes;

	/* Interpolator state, carried across period boundaries. */
	int32_t			prev_l;
	int32_t			prev_r;

	/*
	 * Fade state. ramp runs 0 (output at 0 V) to PWMDAC_RAMP_ONE (output
	 * as written), stepping ramp_step per output sample toward
	 * ramp_target. It exists to keep the DC level off the ear: the pin
	 * idles at 0 V and rests at mid rail while playing, and stepping
	 * between the two is what popped.
	 */
	int32_t			ramp;
	int32_t			ramp_target;
	int32_t			ramp_step;

	/*
	 * Stopping cannot wait in the trigger path: chn_trigger() holds the
	 * channel mutex and sleeping under it panics. The fade runs on in the
	 * DMA interrupt and this callout finishes the job afterwards.
	 */
	struct callout		fade_co;
	int			fading;
};

#define	PWMDAC_RAMP_ONE		65536

/* Sample value that puts the PWM at 0% duty, i.e. 0 V on the jack. */
#define	PWMDAC_ZERO		(-32768)
#define	PWMDAC_ZERO_FRAME	0x80008000u

#define	PWMDAC_LOCK(sc)		mtx_lock(&(sc)->mtx)
#define	PWMDAC_UNLOCK(sc)	mtx_unlock(&(sc)->mtx)
#define	PWMDAC_RD(sc, off)	bus_read_4((sc)->res, (off))
#define	PWMDAC_WR(sc, off, v)	bus_write_4((sc)->res, (off), (v))

static struct ofw_compat_data compat_data[] = {
	{ "starfive,jh7110-pwmdac",	1 },
	{ NULL,				0 }
};

static int
jh7110_pwmdac_probe(device_t dev)
{

	if (!ofw_bus_status_okay(dev))
		return (ENXIO);
	if (!ofw_bus_search_compatible(dev, compat_data)->ocd_data)
		return (ENXIO);

	device_set_desc(dev, "StarFive JH7110 PWMDAC");
	return (BUS_PROBE_DEFAULT);
}

/*
 * Diagnostic attenuation, as a right shift. The jack is far louder than
 * the sample scale suggests, so this is deliberately quiet by default and
 * adjustable without a rebuild.
 */
static int pwmdac_atten = 0;		/* full scale, as Linux runs it */

/*
 * TPDF dither, on by default.
 *
 * The DAC truncates to 8 bits in hardware. Undithered truncation leaves
 * quantisation error correlated with the signal, which is what makes decaying
 * notes fizz. One LSB at 8 bits is 256 in 16-bit units, so triangular noise of
 * about that amplitude is what is wanted.
 */
static int pwmdac_dither = 0;

/*
 * Noise shaping order around the 8-bit quantiser. 0 is what Linux does.
 */
static int pwmdac_noiseshape = 2;

/*
 * Dither inside the shaping loop, in 1/256ths of the quantiser step. It was
 * a fixed half step, and listening to silence showed that constant floor was
 * the whole of the residual hiss. At this oversampling ratio the shaper
 * decorrelates the error well enough without it, so the default is off.
 */
static int pwmdac_nsdither = 0;

/*
 * Fade duration in milliseconds at start and stop. 0 reverts to switching the
 * DAC on and off abruptly, which steps the output by half of full scale.
 */
static int pwmdac_fade_ms = 60;

static uint32_t pwmdac_rng_state = 0x1234567u;

static __inline int32_t
pwmdac_tpdf(void)
{
	uint32_t x = pwmdac_rng_state;
	int32_t a, b;

	/* xorshift32: cheap, and quality is irrelevant for dither */
	x ^= x << 13; x ^= x >> 17; x ^= x << 5;
	pwmdac_rng_state = x;

	a = (int32_t)((x >> 8) & 0xff) - 128;
	b = (int32_t)((x >> 16) & 0xff) - 128;

	return (a + b);		/* triangular, about +/-1 LSB of 8 bits */
}

static __inline int16_t
pwmdac_shape(int32_t v)
{

	if (pwmdac_dither)
		v += pwmdac_tpdf();
	if (v > 32767)
		v = 32767;
	else if (v < -32768)
		v = -32768;

	return ((int16_t)v);
}

/*
 * Error-feedback noise shaping around the DAC's 8-bit quantiser.
 *
 * The hardware keeps the top eight bits of the 16-bit word, so the step is
 * 256 in these units and the noise floor is fixed at about -49 dBFS whatever
 * the signal does. The power cannot be reduced, but where it sits can be
 * chosen: feeding the quantisation error back through (1 - z^-1)^n pushes it
 * away from the frequencies the ear cares about and piles it up near Nyquist.
 * Second order crosses unity at fs/6, around 7.9 kHz here, so everything
 * below that improves and the cost is paid above 16 kHz.
 *
 * Half an LSB of TPDF dither is added inside the loop, so the shaper carries
 * it out of band as well; that is why this helps where dither on its own only
 * added a flat floor.
 *
 * The value returned is already rounded to a multiple of 256, so the DAC's
 * own truncation is exact.
 */
/*
 * Advance the fade by one output sample and place v between 0 V and itself.
 *
 * Interpolating toward PWMDAC_ZERO rather than multiplying by a gain matters:
 * a gain would fade the signal but leave the DC sitting at mid rail, and the
 * DC step is the pop.
 */
static __inline int32_t
pwmdac_ramp_apply(struct jh7110_pwmdac_softc *sc, int32_t v)
{
	int64_t g;

	if (sc->ramp < sc->ramp_target) {
		sc->ramp += sc->ramp_step;
		if (sc->ramp > sc->ramp_target)
			sc->ramp = sc->ramp_target;
	} else if (sc->ramp > sc->ramp_target) {
		sc->ramp -= sc->ramp_step;
		if (sc->ramp < sc->ramp_target)
			sc->ramp = sc->ramp_target;
	}

	if (sc->ramp >= PWMDAC_RAMP_ONE)
		return (v);
	if (sc->ramp <= 0)
		return (PWMDAC_ZERO);

	/*
	 * Shape the ramp: x^2 * (3 - 2x), the cheap cubic stand-in for a
	 * raised cosine. It leaves 0 and arrives at 1 with zero slope, so
	 * there is no corner at either end -- a straight line has one at both,
	 * and that is what still sounded like a drop.
	 */
	g = ((int64_t)sc->ramp * sc->ramp) >> 16;
	g = (g * (3 * PWMDAC_RAMP_ONE - 2 * (int64_t)sc->ramp)) >> 16;

	return (PWMDAC_ZERO + (int32_t)(((int64_t)(v - PWMDAC_ZERO) * g)
	    >> 16));
}

struct pwmdac_ns {
	int32_t	e1;
	int32_t	e2;
	int32_t	e3;
};

static __inline int16_t
pwmdac_quantise(int32_t x, struct pwmdac_ns *ns)
{
	int32_t v, q, e;

	if (pwmdac_noiseshape <= 0) {
		if (x > 32767)
			x = 32767;
		else if (x < -32768)
			x = -32768;
		return ((int16_t)x);
	}

	v = x;
	switch (pwmdac_noiseshape) {
	case 1:
		v -= ns->e1;
		break;
	case 2:
		v -= 2 * ns->e1 - ns->e2;
		break;
	default:
		v -= 3 * ns->e1 - 3 * ns->e2 + ns->e3;
		break;
	}

	/*
	 * Dither inside the loop. Off by default: it is the only thing that
	 * puts a floor under silence, and at this oversampling ratio the
	 * shaper does not need it to decorrelate the error.
	 */
	if (pwmdac_nsdither > 0)
		v += (pwmdac_tpdf() * pwmdac_nsdither) / 256;

	/*
	 * Round to the step the hardware will keep. Clamping before the error
	 * is taken is what bounds the feedback, so a clipped passage cannot
	 * make the loop run away.
	 */
	q = (v + 128) & ~255;
	if (q > 32512)
		q = 32512;
	else if (q < -32768)
		q = -32768;

	e = q - v;
	if (e > 4096)
		e = 4096;
	else if (e < -4096)
		e = -4096;

	ns->e3 = ns->e2;
	ns->e2 = ns->e1;
	ns->e1 = e;

	return ((int16_t)q);
}

static void
jh7110_pwmdac_ring_cb(void *arg, bus_dma_segment_t *segs, int nseg, int error)
{

	if (error != 0)
		return;
	*(bus_addr_t *)arg = segs[0].ds_addr;
}

/*
 * Allocate the cyclic DMA ring.
 *
 * Everything the CPU writes here has to be visible to the DMAC, which is not
 * coherent with the caches, and this SoC has no Svpbmt so an "uncached"
 * mapping request is silently ignored. The only mapping that really bypasses
 * the L2 is the SiFive alias, so that is what we hand to the copy path.
 */
static int
jh7110_pwmdac_ring_alloc(struct jh7110_pwmdac_softc *sc)
{
	uint64_t off;
	int error;

	if (sc->ring != NULL)
		return (0);

	error = bus_dma_tag_create(bus_get_dma_tag(sc->dev), 4, 0,
	    BUS_SPACE_MAXADDR, BUS_SPACE_MAXADDR, NULL, NULL,
	    PWMDAC_RING_BYTES, 1, PWMDAC_RING_BYTES, 0, NULL, NULL,
	    &sc->ring_tag);
	if (error != 0)
		return (error);

	error = bus_dmamem_alloc(sc->ring_tag, &sc->ring_cached,
	    BUS_DMA_WAITOK | BUS_DMA_ZERO | BUS_DMA_COHERENT, &sc->ring_map);
	if (error != 0)
		return (error);

	error = bus_dmamap_load(sc->ring_tag, sc->ring_map, sc->ring_cached,
	    PWMDAC_RING_BYTES, jh7110_pwmdac_ring_cb, &sc->ring_pa,
	    BUS_DMA_NOWAIT);
	if (error != 0 || sc->ring_pa == 0)
		return (error != 0 ? error : ENOMEM);

	/* Push out the zeroing that bus_dmamem_alloc() did through the cache. */
	sifive_ccache_flush_range(sc->ring_pa, PWMDAC_RING_BYTES);

	off = sifive_ccache_uncached_offset();
	if (off == 0) {
		device_printf(sc->dev, "no L2 bypass alias available\n");
		return (ENXIO);
	}
	sc->ring = pmap_mapdev((vm_paddr_t)sc->ring_pa + off,
	    PWMDAC_RING_BYTES);
	if (sc->ring == NULL)
		return (ENOMEM);

	return (0);
}

static void
jh7110_pwmdac_ring_free(struct jh7110_pwmdac_softc *sc)
{

	if (sc->ring != NULL) {
		pmap_unmapdev(__DEVOLATILE(void *, sc->ring),
		    PWMDAC_RING_BYTES);
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

/*
 * One period boundary has gone by. Hand control to the sound layer, which
 * comes straight back into jh7110_pwmdac_dai_intr() to refill the period that
 * just finished.
 */
/*
 * Fade the audio still queued in the ring, in place.
 *
 * Only the eight periods after the one being played hold current audio -- the
 * refill writes to (play + 8) each interrupt, so that is how far ahead the
 * stream reaches. Everything beyond is a lap old and must not be allowed to
 * play, so it is set to true zero.
 *
 * Read-modify-write through the bypass alias reads DRAM, which is what was
 * written, and the cache eviction afterwards is the same one the refill does
 * for the same reason.
 *
 * Returns the number of periods of audio the fade covers, which is how long
 * the caller must wait before switching the DAC off.
 */
static u_int
pwmdac_fade_tail(struct jh7110_pwmdac_softc *sc)
{
	u_int frames_per = sc->period_bytes / 4;
	u_int span = PWMDAC_NPERIODS / 2;
	u_int total = span * frames_per;
	uint32_t pos;
	u_int play, k, j, n = 0;

	pos = jh7110_axidma_position(sc->dma);
	play = pos / sc->period_bytes;

	for (k = 1; k < PWMDAC_NPERIODS; k++) {
		u_int idx = (play + k) % PWMDAC_NPERIODS;
		volatile uint32_t *slot = (volatile uint32_t *)(sc->ring +
		    (size_t)idx * sc->period_bytes);

		if (k > span) {
			/* Stale by a lap: must not be played. */
			for (j = 0; j < frames_per; j++)
				slot[j] = PWMDAC_ZERO_FRAME;
		} else {
			for (j = 0; j < frames_per; j++, n++) {
				int64_t u, g;
				uint32_t w;
				int32_t l, r;

				/* 1 at the play point, 0 at the end. */
				u = PWMDAC_RAMP_ONE -
				    ((int64_t)n * PWMDAC_RAMP_ONE) / total;
				g = (u * u) >> 16;
				g = (g * (3 * PWMDAC_RAMP_ONE - 2 * u)) >> 16;

				w = slot[j];
				l = (int16_t)(w & 0xffff);
				r = (int16_t)(w >> 16);

				l = PWMDAC_ZERO + (int32_t)
				    ((((int64_t)l - PWMDAC_ZERO) * g) >> 16);
				r = PWMDAC_ZERO + (int32_t)
				    ((((int64_t)r - PWMDAC_ZERO) * g) >> 16);

				slot[j] = (uint32_t)(uint16_t)(int16_t)l |
				    ((uint32_t)(uint16_t)(int16_t)r << 16);
			}
		}

		sifive_ccache_flush_range(sc->ring_pa +
		    (size_t)idx * sc->period_bytes, sc->period_bytes);
	}

	return (span);
}

/*
 * Finish a stop once the fade has run. Callout context, so taking the DMA
 * lock and disabling the DAC here is legal where doing it in trigger was not.
 */
static void
jh7110_pwmdac_fade_done(void *arg)
{
	struct jh7110_pwmdac_softc *sc = arg;
	uint32_t ctrl;

	PWMDAC_LOCK(sc);
	sc->running = 0;
	sc->fading = 0;
	PWMDAC_UNLOCK(sc);

	if (sc->dma != NULL)
		jh7110_axidma_stop(sc->dma);

	PWMDAC_LOCK(sc);
	PWMDAC_WR(sc, PWMDAC_WDATA, PWMDAC_ZERO_FRAME);
	ctrl = PWMDAC_RD(sc, PWMDAC_CTRL);
	PWMDAC_WR(sc, PWMDAC_CTRL, ctrl & ~CTRL_ENABLE);
	PWMDAC_UNLOCK(sc);
}

static void
jh7110_pwmdac_period_cb(void *arg)
{
	struct jh7110_pwmdac_softc *sc = arg;

	if (sc->intr_handler != NULL)
		sc->intr_handler(sc->intr_arg);
}

/*
 * DIAGNOSTIC. See fix-margin.py: writes a phase-continuous sine from the
 * refill path instead of the sound layer's samples, leaving every other part
 * of the path alone, to say whether a stutter comes from the ring or from the
 * data we put in it.
 */
/*
 * PWM resolution: 8 or 10 bits. See add-10bit.py -- 10 costs a slightly
 * different sample rate and buys about 12 dB of quantisation noise. Takes
 * effect at the next stream start.
 */
static int pwmdac_bits = 8;

/*
 * The 10-bit mode hung the board: asking for its 49.5 MHz core clock let the
 * request propagate up through audio_root to pll2_out, which also clocks the
 * buses. Reaching it safely needs a path that cannot propagate. Until then
 * bits=10 is refused unless this is set as well.
 */
static int pwmdac_bits_unsafe = 0;

/*
 * Oversampling ratio: 1 keeps the reference 47,353 Hz, 4 runs the DAC at
 * 193,359 Hz so the noise shaper has somewhere to put the noise. Applies at
 * the next stream start.
 */
/*
 * 8 runs the DAC at 386,718 Hz, where the shaper has somewhere to put the
 * quantisation noise; 1 is the 47,353 Hz the reference platform uses.
 */
static int pwmdac_osr = 8;

static int pwmdac_synth;
static uint32_t pwmdac_phase;
static uint8_t pwmdac_filltrace[64];
static uint32_t pwmdac_nfill;

static struct pwmdac_ns pwmdac_ns_l;
static struct pwmdac_ns pwmdac_ns_r;

/*
 * Envelope of what is actually written to the ring: the peak distance from
 * 0 V over each group of 64 output frames, kept in a circular buffer. See
 * add-tailenv.py -- the ending needs to be looked at rather than inferred.
 */
#define	PWMDAC_ENV_GROUP	64
#define	PWMDAC_ENV_POINTS	256

static uint16_t pwmdac_env[PWMDAC_ENV_POINTS];
static uint8_t pwmdac_env_mark[PWMDAC_ENV_POINTS];
static uint32_t pwmdac_env_pos;
static uint32_t pwmdac_env_acc;
static uint32_t pwmdac_env_n;
static uint8_t pwmdac_env_pending;

static __inline void
pwmdac_env_add(int32_t v)
{
	uint32_t mag;

	mag = (uint32_t)(v - PWMDAC_ZERO);
	if (mag > 65535)
		mag = 65535;
	mag = (mag * 999) / 65535;

	if (mag > pwmdac_env_acc)
		pwmdac_env_acc = mag;

	if (++pwmdac_env_n >= PWMDAC_ENV_GROUP) {
		pwmdac_env[pwmdac_env_pos] = (uint16_t)pwmdac_env_acc;
		pwmdac_env_mark[pwmdac_env_pos] = pwmdac_env_pending;
		pwmdac_env_pending = 0;
		pwmdac_env_pos = (pwmdac_env_pos + 1) % PWMDAC_ENV_POINTS;
		pwmdac_env_acc = 0;
		pwmdac_env_n = 0;
	}
}

static uint32_t pwmdac_periods;
static uint32_t pwmdac_underruns;

/*
 * DIAGNOSTIC: dev.pwmdac.0.dmatone=<seconds>.
 *
 * Fills the ring with a sine that closes exactly over its 2048 frames and
 * hands it to the DMAC with no callback, so nothing refills it and the sound
 * layer is not involved at all. This separates "the DMA engine is wrong" from
 * "the refill is wrong": if this is a clean continuous tone the engine and
 * its restart-per-period scheme are fine.
 */
static int
jh7110_pwmdac_dmatone(SYSCTL_HANDLER_ARGS)
{
	struct jh7110_pwmdac_softc *sc = arg1;
	struct jh7110_dma_slave_config cfg;
	volatile int16_t *w;
	uint32_t ctrl;
	int secs = 0, err, i;

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
	if (jh7110_pwmdac_ring_alloc(sc) != 0)
		return (ENOMEM);

	/*
	 * 2048 frames per ring. 19 cycles over the ring is 445.3 Hz and joins
	 * seamlessly end to end, so any discontinuity heard is the engine's,
	 * not the waveform's.
	 */
	sc->osr = 1;
	sc->period_bytes = PWMDAC_STREAM_PERIOD;
	sc->ring_bytes = sc->period_bytes * PWMDAC_NPERIODS;

	w = (volatile int16_t *)sc->ring;
	for (i = 0; i < (int)(sc->ring_bytes / 4); i++) {
		static const int16_t quarter[] = {
			0, 5126, 10126, 14876, 19260, 23170, 26509, 29196,
			31163, 32364, 32767, 32364, 31163, 29196, 26509,
			23170, 19260, 14876, 10126, 5126, 0, -5126, -10126,
			-14876, -19260, -23170, -26509, -29196, -31163,
			-32364, -32767, -32364, -31163, -29196, -26509,
			-23170, -19260, -14876, -10126, -5126
		};
		int16_t v;

		/* 40-entry table, 19 cycles across 2048 frames. */
		v = quarter[(i * 19 * 40 / (int)(sc->ring_bytes / 4)) % 40];
		v = (int16_t)(v >> pwmdac_atten);
		w[i * 2] = v;
		w[i * 2 + 1] = v;
	}

	/*
	 * Same reason as the refill path: the tone was written through the
	 * bypass alias, so evict the ring before handing it to the engine.
	 */
	sifive_ccache_flush_range(sc->ring_pa, sc->ring_bytes);

	device_printf(sc->dev, "dmatone: %d s, ring-resident, no refill\n",
	    secs);

	PWMDAC_LOCK(sc);
	ctrl = PWMDAC_RD(sc, PWMDAC_CTRL);
	PWMDAC_WR(sc, PWMDAC_CTRL, ctrl | CTRL_ENABLE);
	PWMDAC_UNLOCK(sc);

	cfg.dev_addr = rman_get_start(sc->res) + PWMDAC_WDATA;
	cfg.dev_width = 4;
	cfg.maxburst = 16;
	cfg.direction = JH7110_DMA_MEM_TO_DEV;

	err = jh7110_axidma_cyclic(sc->dma, &cfg, sc->ring_pa,
	    sc->ring_bytes, sc->period_bytes, NULL, NULL);
	if (err != 0) {
		device_printf(sc->dev, "dmatone: start failed %d\n", err);
		return (err);
	}

	pause("dmatone", secs * hz);

	jh7110_axidma_stop(sc->dma);

	PWMDAC_LOCK(sc);
	ctrl = PWMDAC_RD(sc, PWMDAC_CTRL);
	PWMDAC_WR(sc, PWMDAC_CTRL, ctrl & ~CTRL_ENABLE);
	PWMDAC_UNLOCK(sc);

	device_printf(sc->dev, "dmatone: done\n");

	return (0);
}

static int jh7110_pwmdac_testtone(SYSCTL_HANDLER_ARGS);
static int jh7110_pwmdac_playclip(SYSCTL_HANDLER_ARGS);
static int
jh7110_pwmdac_tailenv(SYSCTL_HANDLER_ARGS)
{
	char buf[PWMDAC_ENV_POINTS * 6 + 1];
	int i, off = 0;

	for (i = 0; i < PWMDAC_ENV_POINTS && off < (int)sizeof(buf) - 8; i++) {
		uint32_t k = (pwmdac_env_pos + i) % PWMDAC_ENV_POINTS;

		off += snprintf(buf + off, sizeof(buf) - off, "%s%u ",
		    (pwmdac_env_mark[k] & 2) ? "|" :
		    (pwmdac_env_mark[k] & 1) ? ">" : "",
		    pwmdac_env[k]);
	}
	buf[off] = 0;

	return (sysctl_handle_string(oidp, buf, sizeof(buf), req));
}

static int
jh7110_pwmdac_filltrace(SYSCTL_HANDLER_ARGS)
{
	char buf[64 * 4 + 1];
	int i, n, off = 0;

	n = pwmdac_nfill < nitems(pwmdac_filltrace) ? pwmdac_nfill :
	    nitems(pwmdac_filltrace);
	for (i = 0; i < n && off < (int)sizeof(buf) - 4; i++)
		off += snprintf(buf + off, sizeof(buf) - off, "%u ",
		    pwmdac_filltrace[i]);
	buf[off] = 0;

	return (sysctl_handle_string(oidp, buf, sizeof(buf), req));
}

static int jh7110_pwmdac_datamode(SYSCTL_HANDLER_ARGS);

/*
 * Set pwmdac_core on its own, with the DAC idle. A higher core clock is what
 * oversampling needs, and an earlier attempt at one ended with the board
 * rebooting for reasons never established -- so the clock change is reachable
 * without also starting a stream.
 */
static int
jh7110_pwmdac_coreclk(SYSCTL_HANDLER_ARGS)
{
	struct jh7110_pwmdac_softc *sc = arg1;
	uint64_t freq = 0;
	int err;

	if (clk_get_freq(sc->clk_core, &freq) != 0)
		freq = 0;

	err = sysctl_handle_64(oidp, &freq, 0, req);
	if (err != 0 || req->newptr == NULL)
		return (err);

	if (sc->running)
		return (EBUSY);
	if (freq < 1000000 || freq > 100000000)
		return (EINVAL);

	err = clk_set_freq(sc->clk_core, freq, CLK_SET_ROUND_UP);
	if (err != 0)
		return (err);

	if (clk_get_freq(sc->clk_core, &freq) == 0)
		device_printf(sc->dev, "core clock now %ju Hz\n",
		    (uintmax_t)freq);

	return (0);
}

static int
jh7110_pwmdac_attach(device_t dev)
{
	struct jh7110_pwmdac_softc *sc;
	phandle_t node;
	hwreset_t rst;
	int rid;

	sc = device_get_softc(dev);
	sc->dev = dev;

	mtx_init(&sc->mtx, device_get_nameunit(dev), NULL, MTX_DEF);
	callout_init(&sc->fade_co, 1);

	rid = 0;
	sc->res = bus_alloc_resource_any(dev, SYS_RES_MEMORY, &rid, RF_ACTIVE);
	if (sc->res == NULL) {
		device_printf(dev, "could not allocate registers\n");
		return (ENXIO);
	}

	if (clk_get_by_ofw_name(dev, 0, "apb", &sc->clk_apb) != 0 ||
	    clk_get_by_ofw_name(dev, 0, "core", &sc->clk_core) != 0) {
		device_printf(dev, "could not get clocks\n");
		return (ENXIO);
	}
	clk_enable(sc->clk_apb);
	clk_enable(sc->clk_core);

	if (hwreset_get_by_ofw_idx(dev, 0, 0, &rst) == 0)
		hwreset_deassert(rst);

	node = ofw_bus_get_node(dev);
	OF_device_register_xref(OF_xref_from_node(node), dev);

	device_printf(dev, "PWMDAC audio at 0x%lx\n",
	    rman_get_start(sc->res));

	/*
	 * DIAGNOSTIC: dev.pwmdac.0.testtone=<seconds> plays a paced 440 Hz
	 * tone straight to the DAC, bypassing the pcm layer. The ordinary
	 * playback path delivers samples with no pacing, so nearly all of
	 * them are overwritten before conversion; this establishes whether
	 * that is the whole story and whether the DAC's own configuration is
	 * right. Busy-waits a core -- diagnostic only.
	 */
	SYSCTL_ADD_PROC(device_get_sysctl_ctx(dev),
	    SYSCTL_CHILDREN(device_get_sysctl_tree(dev)), OID_AUTO,
	    "testtone", CTLTYPE_INT | CTLFLAG_WR | CTLFLAG_MPSAFE, sc, 0,
	    jh7110_pwmdac_testtone, "I",
	    "play a paced 440 Hz test tone for N seconds (diagnostic)");

	SYSCTL_ADD_PROC(device_get_sysctl_ctx(dev),
	    SYSCTL_CHILDREN(device_get_sysctl_tree(dev)), OID_AUTO,
	    "playclip", CTLTYPE_INT | CTLFLAG_WR | CTLFLAG_MPSAFE, sc, 0,
	    jh7110_pwmdac_playclip, "I",
	    "play /boot/firmware/pwmdac_clip.raw, paced (diagnostic)");

	SYSCTL_ADD_PROC(device_get_sysctl_ctx(dev),
	    SYSCTL_CHILDREN(device_get_sysctl_tree(dev)), OID_AUTO,
	    "dmatone", CTLTYPE_INT | CTLFLAG_WR | CTLFLAG_MPSAFE, sc, 0,
	    jh7110_pwmdac_dmatone, "I",
	    "play a ring-resident tone over DMA with no refill (diagnostic)");

	SYSCTL_ADD_INT(device_get_sysctl_ctx(dev),
	    SYSCTL_CHILDREN(device_get_sysctl_tree(dev)), OID_AUTO,
	    "bits", CTLFLAG_RW, &pwmdac_bits, 0,
	    "PWM resolution, 8 or 10; 10 also needs bits_unsafe");

	SYSCTL_ADD_INT(device_get_sysctl_ctx(dev),
	    SYSCTL_CHILDREN(device_get_sysctl_tree(dev)), OID_AUTO,
	    "osr", CTLFLAG_RW, &pwmdac_osr, 0,
	    "oversampling: 1 = 47 kHz like Linux, 4 = 193 kHz");

	SYSCTL_ADD_PROC(device_get_sysctl_ctx(dev),
	    SYSCTL_CHILDREN(device_get_sysctl_tree(dev)), OID_AUTO,
	    "coreclk", CTLTYPE_U64 | CTLFLAG_RW | CTLFLAG_MPSAFE, sc, 0,
	    jh7110_pwmdac_coreclk, "QU",
	    "pwmdac_core rate; set while idle to test a rate on its own");

	SYSCTL_ADD_INT(device_get_sysctl_ctx(dev),
	    SYSCTL_CHILDREN(device_get_sysctl_tree(dev)), OID_AUTO,
	    "bits_unsafe", CTLFLAG_RW, &pwmdac_bits_unsafe, 0,
	    "allow bits=10; its clock request hung the board once");

	SYSCTL_ADD_INT(device_get_sysctl_ctx(dev),
	    SYSCTL_CHILDREN(device_get_sysctl_tree(dev)), OID_AUTO,
	    "fade_ms", CTLFLAG_RW, &pwmdac_fade_ms, 0,
	    "fade in/out duration in ms; 0 switches abruptly and pops");

	SYSCTL_ADD_INT(device_get_sysctl_ctx(dev),
	    SYSCTL_CHILDREN(device_get_sysctl_tree(dev)), OID_AUTO,
	    "nsdither", CTLFLAG_RW, &pwmdac_nsdither, 0,
	    "dither inside the shaping loop, in 1/256ths of a step");

	SYSCTL_ADD_INT(device_get_sysctl_ctx(dev),
	    SYSCTL_CHILDREN(device_get_sysctl_tree(dev)), OID_AUTO,
	    "noiseshape", CTLFLAG_RW, &pwmdac_noiseshape, 0,
	    "error-feedback noise shaping order: 0 off, 1, 2, 3");

	SYSCTL_ADD_INT(device_get_sysctl_ctx(dev),
	    SYSCTL_CHILDREN(device_get_sysctl_tree(dev)), OID_AUTO,
	    "synth", CTLFLAG_RW, &pwmdac_synth, 0,
	    "refill with our own sine instead of the stream (diagnostic)");

	SYSCTL_ADD_PROC(device_get_sysctl_ctx(dev),
	    SYSCTL_CHILDREN(device_get_sysctl_tree(dev)), OID_AUTO,
	    "tailenv", CTLTYPE_STRING | CTLFLAG_RD | CTLFLAG_MPSAFE,
	    sc, 0, jh7110_pwmdac_tailenv, "A",
	    "output envelope, oldest first; > stream dry, | stop");

	SYSCTL_ADD_PROC(device_get_sysctl_ctx(dev),
	    SYSCTL_CHILDREN(device_get_sysctl_tree(dev)), OID_AUTO,
	    "filltrace", CTLTYPE_STRING | CTLFLAG_RD | CTLFLAG_MPSAFE,
	    sc, 0, jh7110_pwmdac_filltrace, "A",
	    "first ring slots the refill path wrote");

	SYSCTL_ADD_UINT(device_get_sysctl_ctx(dev),
	    SYSCTL_CHILDREN(device_get_sysctl_tree(dev)), OID_AUTO,
	    "periods", CTLFLAG_RD, &pwmdac_periods, 0,
	    "periods refilled from the sound layer");

	SYSCTL_ADD_UINT(device_get_sysctl_ctx(dev),
	    SYSCTL_CHILDREN(device_get_sysctl_tree(dev)), OID_AUTO,
	    "underruns", CTLFLAG_RD, &pwmdac_underruns, 0,
	    "periods that could not be filled completely");

	SYSCTL_ADD_INT(device_get_sysctl_ctx(dev),
	    SYSCTL_CHILDREN(device_get_sysctl_tree(dev)), OID_AUTO,
	    "atten", CTLFLAG_RW, &pwmdac_atten, 0,
	    "diagnostic attenuation as a right shift (higher = quieter)");

	SYSCTL_ADD_INT(device_get_sysctl_ctx(dev),
	    SYSCTL_CHILDREN(device_get_sysctl_tree(dev)), OID_AUTO,
	    "dither", CTLFLAG_RW, &pwmdac_dither, 0,
	    "TPDF dither before the hardware truncates to 8 bits");

	SYSCTL_ADD_PROC(device_get_sysctl_ctx(dev),
	    SYSCTL_CHILDREN(device_get_sysctl_tree(dev)), OID_AUTO,
	    "datamode_inv", CTLTYPE_INT | CTLFLAG_RW | CTLFLAG_MPSAFE, sc, 0,
	    jh7110_pwmdac_datamode, "I",
	    "MSB inversion: 1 = signed input (correct), 0 = unsigned");

	return (0);
}

static int
jh7110_pwmdac_detach(device_t dev)
{
	struct jh7110_pwmdac_softc *sc = device_get_softc(dev);

	callout_drain(&sc->fade_co);

	if (sc->dma != NULL) {
		jh7110_axidma_put(sc->dma);
		sc->dma = NULL;
	}
	jh7110_pwmdac_ring_free(sc);

	return (0);
}

static int
jh7110_pwmdac_dai_init(device_t dev, uint32_t format)
{
	struct jh7110_pwmdac_softc *sc;
	uint32_t ctrl;

	sc = device_get_softc(dev);

	/*
	 * CTRL_DATA_MODE_INV inverts the MSB, turning signed two's-complement
	 * samples into the offset binary the PWM comparator expects. Linux
	 * sets the same thing (data_mode = INVERTER_DATA_MSB). Without it the
	 * DAC reads signed data as unsigned: a quiet sine sits near zero where
	 * -1 is 0xFFFF, i.e. full scale, so attenuating the signal makes the
	 * output *louder* and turns it into a square wave.
	 */
	ctrl = CTRL_SHIFT_8BIT | CTRL_DUTY_CENTER | CTRL_DATA_MODE_INV |
	    (0 << CTRL_CNT_N_SHIFT);	/* cnt_n 1, i.e. field 0 */
	PWMDAC_WR(sc, PWMDAC_CTRL, ctrl);

	return (0);
}

/*
 * DIAGNOSTIC: paced test tone. See the header of add-testtone.py.
 *
 * Writes a 440 Hz sine directly to WDATA at 48 kHz, one stereo frame every
 * ~20.8 us. The ordinary playback path pushes samples with no pacing at all,
 * so nearly all of them are overwritten before conversion; if that is the
 * whole story, this should sound clean.
 *
 * Busy-waits a core for the requested number of seconds. Diagnostic only.
 */
static int
jh7110_pwmdac_testtone(SYSCTL_HANDLER_ARGS)
{
	struct jh7110_pwmdac_softc *sc = arg1;
	static const int16_t sine[] = {
		0, 1886, 3765, 5631, 7476, 9296, 11083, 12832, 14536, 16191,
		17789, 19327, 20798, 22197, 23520, 24762, 25917, 26983, 27955,
		28830, 29604, 30274, 30838, 31294, 31639, 31873, 31994, 32003,
		31899, 31683, 31356, 30920, 30376, 29727, 28976, 28126, 27181,
		26146, 25025, 23823, 22546, 21199, 19789, 18322, 16805, 15245,
		13648, 12023, 10376, 8716, 7050, 5385, 3730, 2091, 477,
		-1106, -2651, -4152, -5603, -6998, -8331, -9597, -10791,
		-11908, -12943, -13893, -14753, -15521, -16193, -16768,
		-17243, -17617, -17889, -18059, -18127, -18093, -17959,
		-17727, -17398, -16976, -16463, -15864, -15182, -14423,
		-13591, -12692, -11732, -10717, -9654, -8549, -7410, -6244,
		-5058, -3861, -2659, -1461, -274, 894, 2037, 3147, 4219,
		5245, 6220, 7138, 7993, 8780, 9494, 10131, 10687, 11159
	};
	int secs = 0, err, i, frames;
	uint32_t ctrl;

	err = sysctl_handle_int(oidp, &secs, 0, req);
	if (err != 0 || req->newptr == NULL)
		return (err);
	if (secs <= 0)
		return (0);
	if (secs > 10)
		secs = 10;

	device_printf(sc->dev, "test tone: 440 Hz, %d s, paced at 48 kHz\n",
	    secs);

	PWMDAC_LOCK(sc);
	ctrl = PWMDAC_RD(sc, PWMDAC_CTRL);
	PWMDAC_WR(sc, PWMDAC_CTRL, ctrl | CTRL_ENABLE);
	PWMDAC_UNLOCK(sc);

	frames = 48000 * secs;
	for (i = 0; i < frames; i++) {
		int16_t v = pwmdac_shape(sine[i % nitems(sine)] >> pwmdac_atten);
		uint32_t frame = ((uint32_t)(uint16_t)v) |
		    ((uint32_t)(uint16_t)v << 16);

		PWMDAC_WR(sc, PWMDAC_WDATA, frame);
		DELAY(20);		/* ~48 kHz; the DAC needs 20.8 us */
	}

	device_printf(sc->dev, "test tone: done (%d frames)\n", frames);

	return (0);
}

/*
 * DIAGNOSTIC: play a raw clip, paced the way the hardware needs.
 *
 * Same idea as the test tone but with real audio, so the result can be judged
 * on music. The clip is raw 48 kHz S16_LE stereo at
 * /boot/firmware/pwmdac_clip.raw (firmware(9) resolves the name to that path).
 *
 * Attenuated by 18 dB: the full-scale tone was far too loud.
 */
/*
 * Toggle the MSB inversion live, so its effect can be heard directly.
 */
static int
jh7110_pwmdac_datamode(SYSCTL_HANDLER_ARGS)
{
	struct jh7110_pwmdac_softc *sc = arg1;
	uint32_t ctrl;
	int val, err;

	PWMDAC_LOCK(sc);
	ctrl = PWMDAC_RD(sc, PWMDAC_CTRL);
	PWMDAC_UNLOCK(sc);
	val = (ctrl & CTRL_DATA_MODE_INV) ? 1 : 0;

	err = sysctl_handle_int(oidp, &val, 0, req);
	if (err != 0 || req->newptr == NULL)
		return (err);

	PWMDAC_LOCK(sc);
	ctrl = PWMDAC_RD(sc, PWMDAC_CTRL);
	if (val)
		ctrl |= CTRL_DATA_MODE_INV;
	else
		ctrl &= ~CTRL_DATA_MODE_INV;
	PWMDAC_WR(sc, PWMDAC_CTRL, ctrl);
	PWMDAC_UNLOCK(sc);

	return (0);
}

static int
jh7110_pwmdac_playclip(SYSCTL_HANDLER_ARGS)
{
	struct jh7110_pwmdac_softc *sc = arg1;
	const struct firmware *fw;
	const int16_t *pcm;
	uint32_t ctrl;
	size_t frames, i;
	int go = 0, err;

	err = sysctl_handle_int(oidp, &go, 0, req);
	if (err != 0 || req->newptr == NULL)
		return (err);
	if (go == 0)
		return (0);

	fw = firmware_get("pwmdac_clip.raw");
	if (fw == NULL) {
		device_printf(sc->dev,
		    "playclip: /boot/firmware/pwmdac_clip.raw not found\n");
		return (ENOENT);
	}

	pcm = (const int16_t *)fw->data;
	frames = fw->datasize / 4;		/* 2 channels x 16 bit */
	device_printf(sc->dev, "playclip: %zu frames (%zu ms)\n",
	    frames, frames * 1000 / 48000);

	PWMDAC_LOCK(sc);
	ctrl = PWMDAC_RD(sc, PWMDAC_CTRL);
	PWMDAC_WR(sc, PWMDAC_CTRL, ctrl | CTRL_ENABLE);
	PWMDAC_UNLOCK(sc);

	for (i = 0; i < frames; i++) {
		int16_t l = pwmdac_shape(pcm[i * 2] >> pwmdac_atten);
		int16_t r = pwmdac_shape(pcm[i * 2 + 1] >> pwmdac_atten);
		uint32_t frame = ((uint32_t)(uint16_t)l) |
		    ((uint32_t)(uint16_t)r << 16);

		PWMDAC_WR(sc, PWMDAC_WDATA, frame);
		DELAY(19);	/* ~20.8 us including loop overhead */
	}

	device_printf(sc->dev, "playclip: done\n");
	firmware_put(fw, FIRMWARE_UNLOAD);

	return (0);
}


static int
jh7110_pwmdac_dai_trigger(device_t dev, int go, int pcm_dir)
{
	struct jh7110_pwmdac_softc *sc;
	struct jh7110_dma_slave_config cfg;
	uint32_t ctrl, i, speed_hz;
	int error;

	sc = device_get_softc(dev);

	if (pcm_dir != PCMDIR_PLAY)
		return (EINVAL);

	switch (go) {
	case PCMTRIG_START:
		/*
		 * Acquire the DMA channel here rather than in attach: the
		 * DMAC is a sibling on simplebus and the probe order between
		 * the two is not defined.
		 */
		if (sc->dma == NULL) {
			sc->dma = jh7110_axidma_get(dev, "tx");
			if (sc->dma == NULL) {
				device_printf(dev,
				    "no DMA channel; playback unavailable\n");
				return (ENXIO);
			}
		}
		if (jh7110_pwmdac_ring_alloc(sc) != 0) {
			device_printf(dev, "cannot allocate DMA ring\n");
			return (ENOMEM);
		}

		/*
		 * Latch the geometry for this stream. One period is always
		 * PWMDAC_STREAM_PERIOD bytes of stream; oversampling decides
		 * how much ring that becomes.
		 */
		sc->osr = (pwmdac_osr >= 1 && pwmdac_osr <= PWMDAC_MAX_OSR) ?
		    pwmdac_osr : 1;
		sc->period_bytes = PWMDAC_STREAM_PERIOD * sc->osr;
		speed_hz = (sc->speed != 0 ? sc->speed : 48000) * sc->osr;
		sc->ring_bytes = sc->period_bytes * PWMDAC_NPERIODS;
		sc->prev_l = sc->prev_r = 0;

		/*
		 * Prime the ring with true zero -- 0% duty, 0 V -- not sample
		 * zero, which is mid rail. The fade then walks the DC up.
		 */
		for (i = 0; i < sc->ring_bytes / 4; i++)
			((volatile uint32_t *)sc->ring)[i] =
			    PWMDAC_ZERO_FRAME;
		sifive_ccache_flush_range(sc->ring_pa, sc->ring_bytes);

		sc->ramp = 0;
		sc->ramp_target = PWMDAC_RAMP_ONE;
		sc->ramp_step = pwmdac_fade_ms > 0 ?
		    MAX(1, PWMDAC_RAMP_ONE /
		    ((int)(speed_hz / 1000) * pwmdac_fade_ms)) :
		    PWMDAC_RAMP_ONE;

		/* A start during a fade-out must cancel the pending stop. */
		callout_stop(&sc->fade_co);

		PWMDAC_LOCK(sc);
		sc->fading = 0;
		sc->play_ptr = 0;
		sc->running = 1;
		pwmdac_periods = 0;
		pwmdac_underruns = 0;
		pwmdac_nfill = 0;
		pwmdac_phase = 0;
		memset(&pwmdac_ns_l, 0, sizeof(pwmdac_ns_l));
		memset(&pwmdac_ns_r, 0, sizeof(pwmdac_ns_r));
		sc->fill_idx = PWMDAC_NPERIODS;		/* not anchored */
		/*
		 * Park the output at 0 V before switching on, or enabling
		 * would publish whatever was last in the register.
		 */
		PWMDAC_WR(sc, PWMDAC_WDATA, PWMDAC_ZERO_FRAME);
		ctrl = PWMDAC_RD(sc, PWMDAC_CTRL);
		PWMDAC_WR(sc, PWMDAC_CTRL, ctrl | CTRL_ENABLE);
		PWMDAC_UNLOCK(sc);

		cfg.dev_addr = rman_get_start(sc->res) + PWMDAC_WDATA;
		cfg.dev_width = 4;		/* one stereo frame */
		cfg.maxburst = 16;
		cfg.direction = JH7110_DMA_MEM_TO_DEV;

		error = jh7110_axidma_cyclic(sc->dma, &cfg, sc->ring_pa,
		    sc->ring_bytes, sc->period_bytes,
		    jh7110_pwmdac_period_cb, sc);
		if (error != 0) {
			device_printf(dev, "cannot start DMA: %d\n", error);
			PWMDAC_LOCK(sc);
			sc->running = 0;
			ctrl = PWMDAC_RD(sc, PWMDAC_CTRL);
			PWMDAC_WR(sc, PWMDAC_CTRL, ctrl & ~CTRL_ENABLE);
			PWMDAC_UNLOCK(sc);
			return (error);
		}
		break;

	case PCMTRIG_STOP:
	case PCMTRIG_ABORT:
		/*
		 * Aim the fade at 0 V and return. The DMA is still running
		 * and its interrupts still arrive, so the refill walks the
		 * ramp down over real periods; jh7110_pwmdac_fade_done()
		 * switches everything off once that has had time to finish.
		 *
		 * This must not wait here: chn_trigger() holds the channel
		 * mutex and sleeping under it panics.
		 */
		if (pwmdac_fade_ms > 0 && sc->running) {
			u_int periods, ms;

			PWMDAC_LOCK(sc);
			pwmdac_env_pending |= 2;	/* stop arrived */
			sc->fading = 1;
			periods = pwmdac_fade_tail(sc);
			PWMDAC_UNLOCK(sc);

			/*
			 * Wait for the faded tail to actually play before
			 * switching anything off. One period is
			 * period_bytes/4 frames at speed * osr.
			 */
			ms = (periods * (sc->period_bytes / 4) * 1000) /
			    MAX(1, (sc->speed != 0 ? sc->speed : 48000) *
			    sc->osr);

			callout_reset(&sc->fade_co,
			    MAX(2, ((int)ms * hz) / 1000 + hz / 50),
			    jh7110_pwmdac_fade_done, sc);
			break;
		}

		jh7110_pwmdac_fade_done(sc);
		break;
	}

	return (0);
}

/*
 * Write one ring slot from the sound layer's buffer. Returns the number of
 * bytes taken from the stream, which is a whole period unless the stream has
 * run dry; the remainder of the slot is silenced rather than left holding the
 * previous lap's audio.
 *
 * The caller holds the PWMDAC lock.
 */
static uint32_t
pwmdac_fill_slot(struct jh7110_pwmdac_softc *sc, struct snd_dbuf *play_buf,
    u_int idx, uint32_t readyptr, uint32_t ready)
{
	volatile uint32_t *slot;
	const uint8_t *samples;
	uint32_t avail, size, i, out = 0;
	u_int osr = sc->osr;
	int all_zero;

	slot = (volatile uint32_t *)(sc->ring +
	    (size_t)idx * sc->period_bytes);

	size = play_buf->bufsize;
	samples = (const uint8_t *)play_buf->buf;

	/*
	 * readyptr and ready are passed in rather than read here: the sound
	 * layer only advances its read pointer in chn_dmaupdate(), which runs
	 * after we return, so filling two slots in one interrupt would
	 * otherwise copy the same period into both.
	 */
	avail = ready;
	if (avail > PWMDAC_STREAM_PERIOD)
		avail = PWMDAC_STREAM_PERIOD;
	avail &= ~3u;

	/*
	 * The ramp follows the stream. A period that cannot be filled means
	 * the audio has run out -- at the end of a file the sound layer keeps
	 * asking for periods while it drains -- and the rest of this period
	 * must fade rather than step to mid rail. Filling with sample zero
	 * instead would be a full-amplitude step, because zero is the middle
	 * of the swing on this DAC and not silence.
	 *
	 * If data comes back the target returns to full and it fades back in,
	 * which is also the right response to a transient underrun: a short
	 * duck is far less objectionable than a click.
	 */
	/*
	 * Decide where the fade is heading before writing anything, so the
	 * period that first goes silent is itself faded.
	 *
	 * Silence is detected from the samples, not from being told. The
	 * sound layer does not stop or run dry at the end of a file -- it
	 * keeps handing over full periods of zeros -- and zero is mid rail on
	 * this DAC, not 0 V. Without this the output parks at half scale and
	 * whatever eventually disables the DAC produces a step.
	 */
	all_zero = 1;
	for (i = 0; i < avail; i++) {
		if (samples[(readyptr + i) % size] != 0) {
			all_zero = 0;
			break;
		}
	}
	if (avail < PWMDAC_STREAM_PERIOD || all_zero) {
		if (sc->ramp_target != 0)
			pwmdac_env_pending |= 1;	/* going silent */
		sc->ramp_target = 0;
	} else {
		sc->ramp_target = PWMDAC_RAMP_ONE;
	}

	for (i = 0; i < avail; i += 4) {
		uint32_t off = readyptr + i;
		int32_t l, r;
		u_int k;

		if (pwmdac_synth) {
			/*
			 * Bisect: same interrupt, same slot, same
			 * bookkeeping, but the samples are ours and are
			 * phase-continuous across periods. Any stutter heard
			 * here cannot come from the sound layer.
			 */
			static const int16_t quarter[] = {
				0, 5126, 10126, 14876, 19260, 23170, 26509,
				29196, 31163, 32364, 32767, 32364, 31163,
				29196, 26509, 23170, 19260, 14876, 10126,
				5126, 0, -5126, -10126, -14876, -19260,
				-23170, -26509, -29196, -31163, -32364,
				-32767, -32364, -31163, -29196, -26509,
				-23170, -19260, -14876, -10126, -5126
			};

			l = r = quarter[pwmdac_phase % nitems(quarter)];
			pwmdac_phase++;
		} else {
			l = (int16_t)(samples[off % size] |
			    (samples[(off + 1) % size] << 8));
			r = (int16_t)(samples[(off + 2) % size] |
			    (samples[(off + 3) % size] << 8));
		}

		if (pwmdac_atten != 0 || pwmdac_dither) {
			l = pwmdac_shape(l >> pwmdac_atten);
			r = pwmdac_shape(r >> pwmdac_atten);
		}

		/*
		 * Linear interpolation up to the DAC's rate. It is only
		 * -13 dB on the first image, but the images land above
		 * 24 kHz where the jack's RC filter and hearing both help,
		 * and the point of running fast is the shaper, not the
		 * interpolator. prev_* carries across periods so there is no
		 * discontinuity at the seam.
		 */
		for (k = 0; k < osr; k++) {
			int32_t vl, vr;
			int16_t ql, qr;

			if (osr == 1) {
				vl = l;
				vr = r;
			} else {
				vl = sc->prev_l +
				    (int32_t)((l - sc->prev_l) * (int32_t)k) /
				    (int32_t)osr;
				vr = sc->prev_r +
				    (int32_t)((r - sc->prev_r) * (int32_t)k) /
				    (int32_t)osr;
			}

			vl = pwmdac_ramp_apply(sc, vl);
			vr = pwmdac_ramp_apply(sc, vr);

			ql = pwmdac_quantise(vl, &pwmdac_ns_l);
			qr = pwmdac_quantise(vr, &pwmdac_ns_r);
			pwmdac_env_add(ql);

			slot[out / 4] = (uint32_t)(uint16_t)ql |
			    ((uint32_t)(uint16_t)qr << 16);
			out += 4;
		}

		sc->prev_l = l;
		sc->prev_r = r;
	}

	/*
	 * Short: the rest of the period fades toward 0 V. Writing zeroes here
	 * would be mid rail, not silence, and that step is audible.
	 */
	for (i = out; i < sc->period_bytes; i += 4) {
		int16_t q;

		q = pwmdac_quantise(pwmdac_ramp_apply(sc, 0), &pwmdac_ns_l);
		pwmdac_env_add(q);
		slot[i / 4] = (uint32_t)(uint16_t)q |
		    ((uint32_t)(uint16_t)q << 16);
	}

	/*
	 * Drop this period out of the CCache. The stores above went through
	 * the L2 bypass alias, straight to DRAM, and so did not snoop the
	 * cache -- but the DMAC's reads are served *from* it, so without this
	 * the engine keeps replaying whatever the first lap pulled in. The
	 * range is the real DRAM address, since the cache is indexed
	 * physically; nothing here is dirty, so this is an invalidate.
	 */
	sifive_ccache_flush_range(sc->ring_pa +
	    (size_t)idx * sc->period_bytes, sc->period_bytes);

	if (pwmdac_nfill < nitems(pwmdac_filltrace))
		pwmdac_filltrace[pwmdac_nfill] = idx;
	pwmdac_nfill++;
	pwmdac_periods++;
	if (avail < PWMDAC_STREAM_PERIOD)
		pwmdac_underruns++;

	sc->play_ptr += avail;
	sc->play_ptr %= size;

	return (avail);
}

static int
jh7110_pwmdac_dai_intr(device_t dev, struct snd_dbuf *play_buf,
    struct snd_dbuf *rec_buf)
{
	struct jh7110_pwmdac_softc *sc;
	uint32_t pos, filled = 0, readyptr, ready, size;
	u_int target, n, k;

	sc = device_get_softc(dev);

	/*
	 * While the tail is being faded the ring must be left alone: the fade
	 * is applied in place to periods that are already queued, and a
	 * refill would overwrite them.
	 */
	if (play_buf == NULL || !sc->running || sc->fading || sc->ring == NULL)
		return (0);

	/*
	 * Aim at the period diametrically opposite the one being played: the
	 * engine never stops, and this handler is slow enough that CH_LLP can
	 * have moved on by the time we read it, so anything near the play
	 * point risks being overwritten underneath the engine.
	 */
	pos = jh7110_axidma_position(sc->dma);
	target = (pos / sc->period_bytes + PWMDAC_NPERIODS / 2) %
	    PWMDAC_NPERIODS;

	PWMDAC_LOCK(sc);

	size = play_buf->bufsize;
	readyptr = sndbuf_getreadyptr(play_buf);
	ready = sndbuf_getready(play_buf);


	if (sc->fill_idx >= PWMDAC_NPERIODS) {
		/* First interrupt of this stream: just anchor the pointer. */
		sc->fill_idx = target;
		n = 1;
	} else {
		/*
		 * Chase the target rather than assuming one period per
		 * interrupt. CH_INTSTATUS is a level, not a count, so two
		 * periods finishing before we run raise one interrupt -- and
		 * writing only one slot for them is what made the seam drift.
		 */
		n = (target - sc->fill_idx + PWMDAC_NPERIODS) %
		    PWMDAC_NPERIODS + 1;
		if (n > PWMDAC_NPERIODS / 2)
			n = PWMDAC_NPERIODS / 2;
	}

	for (k = 0; k < n; k++) {
		uint32_t took;

		took = pwmdac_fill_slot(sc, play_buf, sc->fill_idx,
		    (readyptr + filled) % size, ready - filled);
		sc->fill_idx = (sc->fill_idx + 1) % PWMDAC_NPERIODS;
		filled += took;
		if (took < PWMDAC_STREAM_PERIOD)
			break;		/* stream is dry; stop here */
	}

	PWMDAC_UNLOCK(sc);

	return (filled > 0 ? AUDIO_DAI_PLAY_INTR : 0);
}

static struct pcmchan_caps *
jh7110_pwmdac_dai_get_caps(device_t dev)
{

	return (&jh7110_pwmdac_caps);
}

static uint32_t
jh7110_pwmdac_dai_get_ptr(device_t dev, int pcm_dir)
{
	struct jh7110_pwmdac_softc *sc;
	uint32_t ptr;

	sc = device_get_softc(dev);

	PWMDAC_LOCK(sc);
	ptr = sc->play_ptr;
	PWMDAC_UNLOCK(sc);

	return (ptr);
}

static int
jh7110_pwmdac_dai_setup_intr(device_t dev, driver_intr_t intr_handler,
    void *intr_arg)
{
	struct jh7110_pwmdac_softc *sc;

	sc = device_get_softc(dev);
	sc->intr_handler = intr_handler;
	sc->intr_arg = intr_arg;

	return (0);
}

static uint32_t
jh7110_pwmdac_dai_set_chanformat(device_t dev, uint32_t format)
{

	return (0);
}

static uint32_t
jh7110_pwmdac_dai_set_chanspeed(device_t dev, uint32_t speed)
{
	struct jh7110_pwmdac_softc *sc;
	uint32_t ctrl, cnt_n;
	uint64_t mclk;

	sc = device_get_softc(dev);
	sc->speed = speed;

	if (pwmdac_osr > 1) {
		/*
		 * fs = core_clk / 2^bits. Four times the rate is four times
		 * the clock: 48000 * 256 * 4 = 49,152,000, which rounds up to
		 * audio_root/12 = 49.5 MHz and an actual 193,359 Hz.
		 */
		cnt_n = 1;
		mclk = (uint64_t)speed * (1 << 8) * pwmdac_osr;
		clk_set_freq(sc->clk_core, mclk, CLK_SET_ROUND_UP);
	} else if (pwmdac_bits == 10 && pwmdac_bits_unsafe) {
		/*
		 * fs = core_clk / 2^bits / cnt_n, so 10 bits needs four times
		 * the clock 8 bits does. Round *up*: 48 kHz wants 49.152 MHz
		 * and rounding down lands on audio_root/13 = 45.7 MHz, a
		 * 44.6 kHz sample rate. Rounding up lands on audio_root/12 =
		 * 49.5 MHz and 48,340 Hz, which is closer to 48 kHz than the
		 * 8-bit mode manages.
		 */
		cnt_n = 1;
		mclk = (uint64_t)speed * 1024;
		clk_set_freq(sc->clk_core, mclk, CLK_SET_ROUND_UP);
	} else {
		switch (speed) {
		case 8000:	cnt_n = 3; mclk = 6144000; break;
		case 11025:	cnt_n = 2; mclk = 5644800; break;
		case 16000:	cnt_n = 3; mclk = 12288000; break;
		case 22050:	cnt_n = 1; mclk = 5644800; break;
		case 32000:	cnt_n = 1; mclk = 8192000; break;
		case 44100:	cnt_n = 1; mclk = 11289600; break;
		case 48000:	cnt_n = 1; mclk = 12288000; break;
		default:	cnt_n = 1; mclk = 12288000; break;
		}

		clk_set_freq(sc->clk_core, mclk + 64, CLK_SET_ROUND_DOWN);
	}

	if (clk_get_freq(sc->clk_core, &mclk) == 0)
		device_printf(sc->dev,
		    "%d-bit PWM, core %ju Hz, actual rate %ju Hz\n",
		    pwmdac_bits, (uintmax_t)mclk,
		    (uintmax_t)(mclk / (1 << pwmdac_bits) / cnt_n));

	PWMDAC_LOCK(sc);
	ctrl = PWMDAC_RD(sc, PWMDAC_CTRL);
	/*
	 * The field holds "PWM periods per sample, minus one" -- Linux writes
	 * (cnt_n - 1) and we were writing cnt_n, so every rate ran with one
	 * period too many. It did not show while samples were pushed from a
	 * callout, because nothing was paced by the DAC; now that the DMAC is
	 * gated on the DAC's handshake this field sets the real sample rate.
	 */
	ctrl &= ~(CTRL_CNT_N_MASK | CTRL_SHIFT_10BIT);
	ctrl |= ((cnt_n - 1) << CTRL_CNT_N_SHIFT);
	if (pwmdac_bits == 10)
		ctrl |= CTRL_SHIFT_10BIT;
	PWMDAC_WR(sc, PWMDAC_CTRL, ctrl);
	PWMDAC_UNLOCK(sc);

	return (speed);
}

static int
jh7110_pwmdac_dai_set_sysclk(device_t dev, unsigned int rate, int dai_dir)
{

	return (0);
}

static device_method_t jh7110_pwmdac_methods[] = {
	DEVMETHOD(device_probe,		jh7110_pwmdac_probe),
	DEVMETHOD(device_attach,	jh7110_pwmdac_attach),
	DEVMETHOD(device_detach,	jh7110_pwmdac_detach),

	DEVMETHOD(audio_dai_init,	jh7110_pwmdac_dai_init),
	DEVMETHOD(audio_dai_setup_intr,	jh7110_pwmdac_dai_setup_intr),
	DEVMETHOD(audio_dai_set_sysclk,	jh7110_pwmdac_dai_set_sysclk),
	DEVMETHOD(audio_dai_set_chanspeed, jh7110_pwmdac_dai_set_chanspeed),
	DEVMETHOD(audio_dai_set_chanformat, jh7110_pwmdac_dai_set_chanformat),
	DEVMETHOD(audio_dai_intr,	jh7110_pwmdac_dai_intr),
	DEVMETHOD(audio_dai_get_caps,	jh7110_pwmdac_dai_get_caps),
	DEVMETHOD(audio_dai_get_ptr,	jh7110_pwmdac_dai_get_ptr),
	DEVMETHOD(audio_dai_trigger,	jh7110_pwmdac_dai_trigger),

	DEVMETHOD_END,
};

static driver_t jh7110_pwmdac_driver = {
	"pwmdac",
	jh7110_pwmdac_methods,
	sizeof(struct jh7110_pwmdac_softc),
};

DRIVER_MODULE(jh7110_pwmdac, simplebus, jh7110_pwmdac_driver, 0, 0);
SIMPLEBUS_PNP_INFO(compat_data);
MODULE_DEPEND(jh7110_pwmdac, sound, SOUND_MINVER, SOUND_PREFVER, SOUND_MAXVER);
