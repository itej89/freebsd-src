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
	struct callout		intr_callout;
	int			running;
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

/*
 * Diagnostic attenuation, as a right shift. The jack is far louder than
 * the sample scale suggests, so this is deliberately quiet by default and
 * adjustable without a rebuild.
 */
static int pwmdac_atten = 8;		/* >>8, about -48 dB */

static int jh7110_pwmdac_testtone(SYSCTL_HANDLER_ARGS);
static int jh7110_pwmdac_playclip(SYSCTL_HANDLER_ARGS);
static int jh7110_pwmdac_datamode(SYSCTL_HANDLER_ARGS);

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

	SYSCTL_ADD_INT(device_get_sysctl_ctx(dev),
	    SYSCTL_CHILDREN(device_get_sysctl_tree(dev)), OID_AUTO,
	    "atten", CTLFLAG_RW, &pwmdac_atten, 0,
	    "diagnostic attenuation as a right shift (higher = quieter)");

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
	    (1 << CTRL_CNT_N_SHIFT);
	PWMDAC_WR(sc, PWMDAC_CTRL, ctrl);

	return (0);
}

static void jh7110_pwmdac_callout(void *arg);

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
		int16_t v = sine[i % nitems(sine)] >> pwmdac_atten;
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
		int16_t l = pcm[i * 2] >> pwmdac_atten;
		int16_t r = pcm[i * 2 + 1] >> pwmdac_atten;
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
	uint32_t ctrl;

	sc = device_get_softc(dev);

	if (pcm_dir != PCMDIR_PLAY)
		return (EINVAL);

	PWMDAC_LOCK(sc);
	ctrl = PWMDAC_RD(sc, PWMDAC_CTRL);

	switch (go) {
	case PCMTRIG_START:
		sc->play_ptr = 0;
		sc->running = 1;
		PWMDAC_WR(sc, PWMDAC_CTRL, ctrl | CTRL_ENABLE);
		callout_reset(&sc->intr_callout, 1,
		    jh7110_pwmdac_callout, sc);
		break;
	case PCMTRIG_STOP:
	case PCMTRIG_ABORT:
		sc->running = 0;
		PWMDAC_WR(sc, PWMDAC_CTRL, ctrl & ~CTRL_ENABLE);
		callout_stop(&sc->intr_callout);
		break;
	}
	PWMDAC_UNLOCK(sc);

	return (0);
}

static int
jh7110_pwmdac_dai_intr(device_t dev, struct snd_dbuf *play_buf,
    struct snd_dbuf *rec_buf)
{
	struct jh7110_pwmdac_softc *sc;
	uint8_t *samples;
	uint32_t count, size, readyptr, written;
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

	written = 0;
	while (count >= 4 && written < 256) {
		uint32_t sample;

		sample = samples[readyptr % size] |
		    (samples[(readyptr + 1) % size] << 8) |
		    (samples[(readyptr + 2) % size] << 16) |
		    (samples[(readyptr + 3) % size] << 24);
		PWMDAC_WR(sc, PWMDAC_WDATA, sample);
		readyptr += 4;
		count -= 4;
		written += 4;
	}
	sc->play_ptr += written;
	sc->play_ptr %= size;
	ret |= AUDIO_DAI_PLAY_INTR;
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
jh7110_pwmdac_callout(void *arg)
{
	struct jh7110_pwmdac_softc *sc = arg;

	if (sc->intr_handler != NULL)
		sc->intr_handler(sc->intr_arg);

	if (sc->running)
		callout_reset(&sc->intr_callout, 1, jh7110_pwmdac_callout, sc);
}

static int
jh7110_pwmdac_dai_setup_intr(device_t dev, driver_intr_t intr_handler,
    void *intr_arg)
{
	struct jh7110_pwmdac_softc *sc;

	sc = device_get_softc(dev);
	sc->intr_handler = intr_handler;
	sc->intr_arg = intr_arg;

	callout_init(&sc->intr_callout, 1);

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
