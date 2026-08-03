/*-
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * Copyright (c) 2026 Tej <itej89@github.com>
 *
 * HDMI audio driver for StarFive JH7110 SoC.
 * Uses DesignWare I2S TX0 controller to feed audio to HDMI TX.
 *
 * Based on Linux sound/soc/starfive/starfive_i2s.c and inno_hdmi.c
 */

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/bus.h>
#include <sys/callout.h>
#include <sys/kernel.h>
#include <sys/lock.h>
#include <sys/module.h>
#include <sys/mutex.h>
#include <sys/rman.h>

#include <machine/bus.h>

#include <dev/ofw/openfirm.h>
#include <dev/ofw/ofw_bus.h>
#include <dev/ofw/ofw_bus_subr.h>

#include <dev/clk/clk.h>
#include <dev/hwreset/hwreset.h>

#include "opt_snd.h"
#include <dev/sound/pcm/sound.h>
#include <dev/sound/fdt/audio_dai.h>
#include "audio_dai_if.h"

/* DesignWare I2S TX registers */
#define	I2S_IER		0x000
#define	I2S_ITER	0x008
#define	I2S_CER		0x00C
#define	I2S_CCR		0x010
#define	I2S_TXFFR	0x018
#define	I2S_LTHR(ch)	(0x40 * (ch) + 0x020)
#define	I2S_RTHR(ch)	(0x40 * (ch) + 0x024)
#define	I2S_TER(ch)	(0x40 * (ch) + 0x02C)
#define	I2S_TCR(ch)	(0x40 * (ch) + 0x034)
#define	I2S_ISR(ch)	(0x40 * (ch) + 0x038)
#define	I2S_IMR(ch)	(0x40 * (ch) + 0x03C)
#define	I2S_TFCR(ch)	(0x40 * (ch) + 0x04C)

/* HDMI audio registers (byte addresses, accessed via *4) */
#define	HDMI_AV_MUTE		0x05
#define	HDMI_AUDIO_CTRL1	0x35
#define	HDMI_AUDIO_RATE		0x37
#define	HDMI_AUDIO_I2S_MODE	0x38
#define	HDMI_AUDIO_I2S_MAP	0x39
#define	HDMI_AUDIO_N_H		0x3f
#define	HDMI_AUDIO_N_M		0x40
#define	HDMI_AUDIO_N_L		0x41
#define	HDMI_AUDIO_CHST		0x3e
#define	HDMI_PKT_SEND_AUTO	0x9d
#define	HDMI_PKT_BUF_INDEX	0x9f
#define	HDMI_PKT_BUF_ADDR	0xa0

static uint32_t jh7110_hdmi_audio_fmts[] = {
	SND_FORMAT(AFMT_S16_LE, 2, 0),
	0,
};

static struct pcmchan_caps jh7110_hdmi_audio_caps = {
	32000, 48000, jh7110_hdmi_audio_fmts, 0,
};

struct jh7110_hdmi_audio_softc {
	device_t		dev;
	struct resource		*i2s_res;	/* I2S TX0 regs */
	struct resource		*hdmi_res;	/* HDMI TX regs */
	int			hdmi_rid;
	struct mtx		mtx;
	clk_t			clk_apb;
	clk_t			clk_bclk;
	uint32_t		play_ptr;
	driver_intr_t		*intr_handler;
	void			*intr_arg;
	struct callout		intr_callout;
	int			running;
};

#define	I2S_RD(sc, off)		bus_read_4((sc)->i2s_res, (off))
#define	I2S_WR(sc, off, v)	bus_write_4((sc)->i2s_res, (off), (v))
#define	HDMI_RD(sc, off)	bus_read_4((sc)->hdmi_res, (off) * 4)
#define	HDMI_WR(sc, off, v)	bus_write_4((sc)->hdmi_res, (off) * 4, (v))
#define	LOCK(sc)		mtx_lock(&(sc)->mtx)
#define	UNLOCK(sc)		mtx_unlock(&(sc)->mtx)

static struct ofw_compat_data compat_data[] = {
	{ "starfive,jh7110-i2stx0",	1 },
	{ NULL,				0 }
};

static void jh7110_hdmi_audio_callout(void *arg);

static void
jh7110_hdmi_audio_config(struct jh7110_hdmi_audio_softc *sc, uint32_t rate)
{
	uint32_t rate_code, n_value;

	switch (rate) {
	case 32000:	rate_code = 0x3; n_value = 4096; break;
	case 44100:	rate_code = 0x0; n_value = 6272; break;
	case 48000:
	default:	rate_code = 0x2; n_value = 6144; break;
	}

	/* Audio source = I2S, MCLK enable, MCLK ratio = 256fs, CTS internal */
	HDMI_WR(sc, HDMI_AUDIO_CTRL1,
	    (0 << 7) |		/* CTS internal */
	    (0 << 5) |		/* no downsample */
	    (0 << 3) |		/* I2S source */
	    (1 << 2) |		/* MCLK enable */
	    1);			/* MCLK 256fs */

	/* Sample rate */
	HDMI_WR(sc, HDMI_AUDIO_RATE, rate_code);

	/* I2S mode: standard I2S, channels 1-2 */
	HDMI_WR(sc, HDMI_AUDIO_I2S_MODE,
	    (1 << 2) |		/* channel 1-2 enable */
	    0);			/* standard I2S */

	/* Default channel map */
	HDMI_WR(sc, HDMI_AUDIO_I2S_MAP, 0x00);

	/* N value for audio clock recovery */
	HDMI_WR(sc, HDMI_AUDIO_N_H, (n_value >> 16) & 0x0f);
	HDMI_WR(sc, HDMI_AUDIO_N_M, (n_value >> 8) & 0xff);
	HDMI_WR(sc, HDMI_AUDIO_N_L, n_value & 0xff);

	/* Channel status: PCM, no copyright */
	HDMI_WR(sc, HDMI_AUDIO_CHST, 0x00);

	/* Enable audio InfoFrame auto-send */
	HDMI_WR(sc, HDMI_PKT_SEND_AUTO,
	    HDMI_RD(sc, HDMI_PKT_SEND_AUTO) | (1 << 6));
}

static int
jh7110_hdmi_audio_probe(device_t dev)
{

	if (!ofw_bus_status_okay(dev))
		return (ENXIO);
	if (!ofw_bus_search_compatible(dev, compat_data)->ocd_data)
		return (ENXIO);

	device_set_desc(dev, "StarFive JH7110 HDMI Audio (I2S TX0)");
	return (BUS_PROBE_DEFAULT);
}

static int
jh7110_hdmi_audio_attach(device_t dev)
{
	struct jh7110_hdmi_audio_softc *sc;
	phandle_t node;
	hwreset_t rst;
	int rid, i;

	sc = device_get_softc(dev);
	sc->dev = dev;

	mtx_init(&sc->mtx, device_get_nameunit(dev), NULL, MTX_DEF);

	/* Map I2S TX0 registers */
	rid = 0;
	sc->i2s_res = bus_alloc_resource_any(dev, SYS_RES_MEMORY, &rid,
	    RF_ACTIVE);
	if (sc->i2s_res == NULL) {
		device_printf(dev, "could not allocate I2S registers\n");
		return (ENXIO);
	}

	/* Map HDMI TX registers at fixed address */
	sc->hdmi_rid = 1;
	sc->hdmi_res = bus_alloc_resource(dev, SYS_RES_MEMORY,
	    &sc->hdmi_rid, 0x29590000, 0x29593FFF, 0x4000, RF_ACTIVE);
	if (sc->hdmi_res == NULL) {
		device_printf(dev, "could not map HDMI registers\n");
		return (ENXIO);
	}

	/* Enable clocks */
	if (clk_get_by_ofw_name(dev, 0, "apb", &sc->clk_apb) == 0)
		clk_enable(sc->clk_apb);
	if (clk_get_by_ofw_name(dev, 0, "i2sclk", &sc->clk_bclk) == 0)
		clk_enable(sc->clk_bclk);

	/* Deassert resets */
	for (i = 0; hwreset_get_by_ofw_idx(dev, 0, i, &rst) == 0; i++)
		hwreset_deassert(rst);

	/* Register as DAI for audio_soc */
	node = ofw_bus_get_node(dev);
	OF_device_register_xref(OF_xref_from_node(node), dev);

	device_printf(dev, "HDMI audio (I2S TX0) ready\n");

	return (0);
}

static int
jh7110_hdmi_audio_detach(device_t dev)
{

	return (0);
}

static int
jh7110_hdmi_audio_dai_init(device_t dev, uint32_t format)
{
	struct jh7110_hdmi_audio_softc *sc;

	sc = device_get_softc(dev);

	/* Disable I2S first */
	I2S_WR(sc, I2S_IER, 0);
	I2S_WR(sc, I2S_ITER, 0);
	I2S_WR(sc, I2S_TER(0), 0);

	/* Configure for 16-bit resolution */
	I2S_WR(sc, I2S_CCR, 0x00);
	I2S_WR(sc, I2S_TCR(0), 0x02);	/* 16-bit */
	I2S_WR(sc, I2S_TFCR(0), 0x03);	/* TX FIFO trigger level */
	I2S_WR(sc, I2S_IMR(0), 0x30);	/* mask RX interrupts */

	/* Flush TX FIFO */
	I2S_WR(sc, I2S_TXFFR, 1);

	/* Configure HDMI audio for 48kHz default */
	jh7110_hdmi_audio_config(sc, 48000);

	return (0);
}

static int
jh7110_hdmi_audio_dai_trigger(device_t dev, int go, int pcm_dir)
{
	struct jh7110_hdmi_audio_softc *sc;
	uint32_t val;

	sc = device_get_softc(dev);

	if (pcm_dir != PCMDIR_PLAY)
		return (EINVAL);

	LOCK(sc);
	switch (go) {
	case PCMTRIG_START:
		sc->play_ptr = 0;
		sc->running = 1;

		/* Flush FIFO, enable I2S */
		I2S_WR(sc, I2S_TXFFR, 1);
		I2S_WR(sc, I2S_TER(0), 1);
		I2S_WR(sc, I2S_ITER, 1);
		I2S_WR(sc, I2S_IER, 1);

		/* Unmute HDMI audio */
		val = HDMI_RD(sc, HDMI_AV_MUTE);
		HDMI_WR(sc, HDMI_AV_MUTE, val & ~(1 << 1));

		callout_reset(&sc->intr_callout, 1,
		    jh7110_hdmi_audio_callout, sc);
		break;

	case PCMTRIG_STOP:
	case PCMTRIG_ABORT:
		sc->running = 0;
		callout_stop(&sc->intr_callout);

		/* Mute HDMI audio */
		val = HDMI_RD(sc, HDMI_AV_MUTE);
		HDMI_WR(sc, HDMI_AV_MUTE, val | (1 << 1));

		/* Disable I2S */
		I2S_WR(sc, I2S_ITER, 0);
		I2S_WR(sc, I2S_IER, 0);
		break;
	}
	UNLOCK(sc);

	return (0);
}

static int
jh7110_hdmi_audio_dai_intr(device_t dev, struct snd_dbuf *play_buf,
    struct snd_dbuf *rec_buf)
{
	struct jh7110_hdmi_audio_softc *sc;
	uint8_t *samples;
	uint32_t count, size, readyptr, written;
	int ret = 0;

	sc = device_get_softc(dev);

	LOCK(sc);
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
		uint16_t left, right;

		left = samples[readyptr % size] |
		    (samples[(readyptr + 1) % size] << 8);
		right = samples[(readyptr + 2) % size] |
		    (samples[(readyptr + 3) % size] << 8);

		I2S_WR(sc, I2S_LTHR(0), left);
		I2S_WR(sc, I2S_RTHR(0), right);

		readyptr += 4;
		count -= 4;
		written += 4;
	}
	sc->play_ptr += written;
	sc->play_ptr %= size;
	ret |= AUDIO_DAI_PLAY_INTR;
out:
	UNLOCK(sc);

	return (ret);
}

static struct pcmchan_caps *
jh7110_hdmi_audio_dai_get_caps(device_t dev)
{

	return (&jh7110_hdmi_audio_caps);
}

static uint32_t
jh7110_hdmi_audio_dai_get_ptr(device_t dev, int pcm_dir)
{
	struct jh7110_hdmi_audio_softc *sc;
	uint32_t ptr;

	sc = device_get_softc(dev);

	LOCK(sc);
	ptr = sc->play_ptr;
	UNLOCK(sc);

	return (ptr);
}

static void
jh7110_hdmi_audio_callout(void *arg)
{
	struct jh7110_hdmi_audio_softc *sc = arg;

	if (sc->intr_handler != NULL)
		sc->intr_handler(sc->intr_arg);

	if (sc->running)
		callout_reset(&sc->intr_callout, 1,
		    jh7110_hdmi_audio_callout, sc);
}

static int
jh7110_hdmi_audio_dai_setup_intr(device_t dev, driver_intr_t intr_handler,
    void *intr_arg)
{
	struct jh7110_hdmi_audio_softc *sc;

	sc = device_get_softc(dev);
	sc->intr_handler = intr_handler;
	sc->intr_arg = intr_arg;

	callout_init(&sc->intr_callout, 1);

	return (0);
}

static uint32_t
jh7110_hdmi_audio_dai_set_chanformat(device_t dev, uint32_t format)
{

	return (0);
}

static uint32_t
jh7110_hdmi_audio_dai_set_chanspeed(device_t dev, uint32_t speed)
{
	struct jh7110_hdmi_audio_softc *sc;

	sc = device_get_softc(dev);

	if (sc->clk_bclk != NULL) {
		uint64_t bclk_rate;

		switch (speed) {
		case 32000:	bclk_rate = 2048000; break;
		case 44100:	bclk_rate = 2822400; break;
		case 48000:
		default:	bclk_rate = 3072000; break;
		}
		clk_set_freq(sc->clk_bclk, bclk_rate, CLK_SET_ROUND_DOWN);
	}

	jh7110_hdmi_audio_config(sc, speed);

	return (speed);
}

static int
jh7110_hdmi_audio_dai_set_sysclk(device_t dev, unsigned int rate, int dai_dir)
{

	return (0);
}

static device_method_t jh7110_hdmi_audio_methods[] = {
	DEVMETHOD(device_probe,		jh7110_hdmi_audio_probe),
	DEVMETHOD(device_attach,	jh7110_hdmi_audio_attach),
	DEVMETHOD(device_detach,	jh7110_hdmi_audio_detach),

	DEVMETHOD(audio_dai_init,	jh7110_hdmi_audio_dai_init),
	DEVMETHOD(audio_dai_setup_intr,	jh7110_hdmi_audio_dai_setup_intr),
	DEVMETHOD(audio_dai_set_sysclk,	jh7110_hdmi_audio_dai_set_sysclk),
	DEVMETHOD(audio_dai_set_chanspeed, jh7110_hdmi_audio_dai_set_chanspeed),
	DEVMETHOD(audio_dai_set_chanformat, jh7110_hdmi_audio_dai_set_chanformat),
	DEVMETHOD(audio_dai_intr,	jh7110_hdmi_audio_dai_intr),
	DEVMETHOD(audio_dai_get_caps,	jh7110_hdmi_audio_dai_get_caps),
	DEVMETHOD(audio_dai_get_ptr,	jh7110_hdmi_audio_dai_get_ptr),
	DEVMETHOD(audio_dai_trigger,	jh7110_hdmi_audio_dai_trigger),

	DEVMETHOD_END,
};

static driver_t jh7110_hdmi_audio_driver = {
	"hdmiaudio",
	jh7110_hdmi_audio_methods,
	sizeof(struct jh7110_hdmi_audio_softc),
};

DRIVER_MODULE(jh7110_hdmi_audio, simplebus, jh7110_hdmi_audio_driver, 0, 0);
SIMPLEBUS_PNP_INFO(compat_data);
MODULE_DEPEND(jh7110_hdmi_audio, sound, SOUND_MINVER, SOUND_PREFVER,
    SOUND_MAXVER);
