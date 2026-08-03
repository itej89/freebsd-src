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
#include <sys/kernel.h>
#include <sys/kthread.h>
#include <sys/lock.h>
#include <sys/module.h>
#include <sys/mutex.h>
#include <sys/proc.h>
#include <sys/rman.h>
#include <sys/unistd.h>

#include <machine/bus.h>

#include <dev/ofw/ofw_bus.h>
#include <dev/ofw/ofw_bus_subr.h>

#include <dev/clk/clk.h>
#include <dev/hwreset/hwreset.h>

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
	struct proc		*tx_thread;
	volatile int		running;
	volatile int		thread_exit;
};

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

	{
		uint64_t freq;

		if (clk_get_freq(sc->clk_core, &freq) == 0)
			device_printf(dev, "core clk: %lu Hz\n",
			    (unsigned long)freq);
		if (clk_get_freq(sc->clk_apb, &freq) == 0)
			device_printf(dev, "apb clk: %lu Hz\n",
			    (unsigned long)freq);
	}
	device_printf(dev, "PWMDAC audio at 0x%lx\n",
	    rman_get_start(sc->res));

	return (0);
}

static int
jh7110_pwmdac_detach(device_t dev)
{

	return (0);
}

static int
jh7110_pwmdac_dai_init(device_t dev, uint32_t format)
{
	struct jh7110_pwmdac_softc *sc;
	uint32_t ctrl;

	sc = device_get_softc(dev);

	ctrl = CTRL_SHIFT_8BIT | CTRL_DUTY_CENTER |
	    (1 << CTRL_CNT_N_SHIFT);
	PWMDAC_WR(sc, PWMDAC_CTRL, ctrl);

	return (0);
}

static void jh7110_pwmdac_thread(void *arg);

static int
jh7110_pwmdac_dai_trigger(device_t dev, int go, int pcm_dir)
{
	struct jh7110_pwmdac_softc *sc;
	uint32_t ctrl;

	sc = device_get_softc(dev);

	if (pcm_dir != PCMDIR_PLAY)
		return (EINVAL);

	switch (go) {
	case PCMTRIG_START:
		sc->play_ptr = 0;
		sc->thread_exit = 0;
		sc->running = 1;
		PWMDAC_LOCK(sc);
		ctrl = PWMDAC_RD(sc, PWMDAC_CTRL);
		PWMDAC_WR(sc, PWMDAC_CTRL, ctrl | CTRL_ENABLE);
		PWMDAC_UNLOCK(sc);
		kproc_create(jh7110_pwmdac_thread, sc, &sc->tx_thread,
		    0, 0, "pwmdac_tx");
		break;
	case PCMTRIG_STOP:
	case PCMTRIG_ABORT:
		sc->thread_exit = 1;
		sc->running = 0;
		PWMDAC_LOCK(sc);
		ctrl = PWMDAC_RD(sc, PWMDAC_CTRL);
		PWMDAC_WR(sc, PWMDAC_CTRL, ctrl & ~CTRL_ENABLE);
		PWMDAC_UNLOCK(sc);
		break;
	}

	return (0);
}

static int
jh7110_pwmdac_dai_intr(device_t dev, struct snd_dbuf *play_buf,
    struct snd_dbuf *rec_buf)
{
	struct jh7110_pwmdac_softc *sc;
	uint8_t *samples;
	uint32_t count, size, readyptr;
	int ret = 0;

	sc = device_get_softc(dev);

	PWMDAC_LOCK(sc);
	if (play_buf == NULL)
		goto out;

	count = sndbuf_getready(play_buf);
	if (count < 4)
		goto out;

	size = play_buf->bufsize;
	readyptr = sndbuf_getreadyptr(play_buf);
	samples = play_buf->buf;

	{
		uint16_t left, right;
		uint32_t sample;

		left = samples[readyptr % size] |
		    (samples[(readyptr + 1) % size] << 8);
		right = samples[(readyptr + 2) % size] |
		    (samples[(readyptr + 3) % size] << 8);
		sample = (uint32_t)left | ((uint32_t)right << 16);

		PWMDAC_WR(sc, PWMDAC_WDATA, sample);
		sc->play_ptr = (sc->play_ptr + 4) % size;
		ret |= AUDIO_DAI_PLAY_INTR;
	}
out:
	PWMDAC_UNLOCK(sc);

	return (ret);
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

static void
jh7110_pwmdac_thread(void *arg)
{
	struct jh7110_pwmdac_softc *sc = arg;

	while (!sc->thread_exit) {
		if (sc->intr_handler != NULL)
			sc->intr_handler(sc->intr_arg);
		DELAY(21);
	}
	kproc_exit(0);
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

	{
		uint64_t actual;
		if (clk_get_freq(sc->clk_core, &actual) == 0)
			device_printf(sc->dev,
			    "chanspeed %u: cnt_n=%u mclk_req=%lu mclk_actual=%lu\n",
			    speed, cnt_n, (unsigned long)mclk,
			    (unsigned long)actual);
	}

	PWMDAC_LOCK(sc);
	ctrl = PWMDAC_RD(sc, PWMDAC_CTRL);
	ctrl &= ~CTRL_CNT_N_MASK;
	ctrl |= (cnt_n << CTRL_CNT_N_SHIFT);
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
