/*-
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * Copyright (c) 2026 Tej <itej89@github.com>
 *
 * HDMI audio codec for the StarFive JH7110's Innosilicon transmitter.
 *
 * This is the codec half of the HDMI sound card: the CPU half is the I2S TX0
 * block (jh7110_i2s.c), which carries the samples, and this configures the
 * transmitter to accept them -- sample rate, channel mapping, clock
 * regeneration and mute.
 *
 * It is a separate driver from jh7110_inno_hdmi.c, which owns the register
 * window, because that file is `standard` and must not pull sound(4) into
 * every kernel. The registers here are reached through the accessors that
 * driver exports. The device tree gives this a node of its own so that
 * audio_soc(4) can resolve it as a codec by phandle.
 *
 * The register meanings follow StarFive's own driver
 * (drivers/gpu/drm/verisilicon/starfive_hdmi_audio.c). That is the only
 * documentation for this block.
 */

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/bus.h>
#include <sys/kernel.h>
#include <sys/module.h>

#include <machine/bus.h>

#include <dev/ofw/ofw_bus.h>
#include <dev/ofw/ofw_bus_subr.h>

#include "opt_snd.h"
#include <dev/sound/pcm/sound.h>
#include <dev/sound/fdt/audio_dai.h>
#include "audio_dai_if.h"

#include "mixer_if.h"

#include "jh7110_inno_hdmi.h"

#include "jh7110_i2s.h"

/* Registers, byte-numbered within the HDMI window. */
#define	HDMI_DEV_MUTE		0x005
#define	HDMI_AUDIO_CFG		0x035
#define	HDMI_SAMPLE_FRE		0x037
#define	HDMI_PINS_ENA		0x038
#define	HDMI_CHANNEL_INPUT	0x039
#define	HDMI_N_VALUE1		0x03f
#define	HDMI_N_VALUE2		0x040
#define	HDMI_N_VALUE3		0x041
#define	HDMI_CTS_VALUE1		0x045
#define	HDMI_CTS_VALUE2		0x046
#define	HDMI_CTS_VALUE3		0x047

/* DEV_MUTE */
#define	AUDIO_MUTE_MASK		(1u << 1)
#define	AUDIO_MUTE		(1u << 1)
#define	AUDIO_NO_MUTE		0u

/* AUDIO_CFG */
#define	MCLK_RATIO_MASK		0x3u
#define	MCLK_256FS		0x1u
#define	AUDIO_TYPE_SEL_MASK	(0x3u << 3)
#define	AUDIO_SEL_I2S		0u
#define	CTS_SOURCE_SEL_MASK	(1u << 7)
#define	CTS_EXTER		(1u << 7)

/* SAMPLE_FRE -- these are the HDMI channel-status frequency codes. */
#define	FREQ_32K		0x3
#define	FREQ_44K		0x0
#define	FREQ_48K		0x2
#define	FREQ_88K		0x8
#define	FREQ_96K		0xa
#define	FREQ_176K		0xc
#define	FREQ_192K		0xe

/* PINS_ENA */
#define	I2S_FORMAT_MASK		0x3u
#define	STANDARD_MODE		0u
#define	I2S_PIN_ENA_MASK	(0xfu << 2)
#define	I2S0_ENA		(1u << 2)
#define	I2S1_ENA		(1u << 3)
#define	I2S2_ENA		(1u << 4)
#define	I2S3_ENA		(1u << 5)

/* CHANNEL_INPUT: each pair of bits picks which I2S lane feeds a channel. */
#define	CHANNEL0_I2S0		(0u << 0)
#define	CHANNEL1_I2S1		(0u << 2)
#define	CHANNEL2_I2S2		(0u << 4)
#define	CHANNEL3_I2S3		(0u << 6)

struct jh7110_hdmi_audio_softc {
	device_t	dev;
	uint32_t	speed;
	int		channels;
};

static struct ofw_compat_data compat_data[] = {
	{ "starfive,jh7110-hdmi-audio",	1 },
	{ NULL,				0 }
};

static int
jh7110_hdmi_audio_probe(device_t dev)
{

	if (!ofw_bus_status_okay(dev))
		return (ENXIO);
	if (!ofw_bus_search_compatible(dev, compat_data)->ocd_data)
		return (ENXIO);

	device_set_desc(dev, "JH7110 HDMI audio codec");
	return (BUS_PROBE_DEFAULT);
}

static int
jh7110_hdmi_audio_attach(device_t dev)
{
	struct jh7110_hdmi_audio_softc *sc = device_get_softc(dev);
	phandle_t node;

	sc->dev = dev;
	sc->speed = 48000;
	sc->channels = 2;

	node = ofw_bus_get_node(dev);
	OF_device_register_xref(OF_xref_from_node(node), dev);

	return (0);
}

static int
jh7110_hdmi_audio_detach(device_t dev)
{

	return (0);
}

/*
 * Static configuration of the audio block. Everything here is a property of
 * how the I2S TX0 block is wired to the transmitter rather than of the
 * stream, so it is set once.
 */
static int
jh7110_hdmi_audio_dai_init(device_t dev, uint32_t format)
{

	/*
	 * Take CTS from the external (measured) source rather than letting
	 * the transmitter generate it internally: the value written by
	 * set_chanspeed is derived from the real TMDS rate.
	 */
	jh7110_hdmi_audio_modb(HDMI_AUDIO_CFG, CTS_SOURCE_SEL_MASK, CTS_EXTER);
	jh7110_hdmi_audio_modb(HDMI_AUDIO_CFG, AUDIO_TYPE_SEL_MASK,
	    AUDIO_SEL_I2S);
	jh7110_hdmi_audio_modb(HDMI_AUDIO_CFG, MCLK_RATIO_MASK, MCLK_256FS);

	/* I2S proper, not left- or right-justified. */
	jh7110_hdmi_audio_modb(HDMI_PINS_ENA, I2S_FORMAT_MASK, STANDARD_MODE);

	/* Straight-through lane mapping. */
	jh7110_hdmi_audio_write(HDMI_CHANNEL_INPUT, CHANNEL0_I2S0 |
	    CHANNEL1_I2S1 | CHANNEL2_I2S2 | CHANNEL3_I2S3);

	/* Start muted; trigger unmutes. */
	jh7110_hdmi_audio_modb(HDMI_DEV_MUTE, AUDIO_MUTE_MASK, AUDIO_MUTE);

	return (0);
}

static uint32_t
jh7110_hdmi_audio_dai_set_chanspeed(device_t dev, uint32_t speed)
{
	struct jh7110_hdmi_audio_softc *sc = device_get_softc(dev);
	uint32_t freq, n, cts, tmds;

	switch (speed) {
	case 32000:	freq = FREQ_32K; break;
	case 44100:	freq = FREQ_44K; break;
	case 48000:	freq = FREQ_48K; break;
	case 88200:	freq = FREQ_88K; break;
	case 96000:	freq = FREQ_96K; break;
	case 176400:	freq = FREQ_176K; break;
	case 192000:	freq = FREQ_192K; break;
	default:
		device_printf(dev, "unsupported rate %u\n", speed);
		return (sc->speed);
	}

	sc->speed = speed;
	jh7110_hdmi_audio_write(HDMI_SAMPLE_FRE, freq);

	/*
	 * Audio clock regeneration. The sink recovers the audio clock from
	 * N and CTS against the TMDS clock:
	 *
	 *	fs = TMDS * N / (128 * CTS)
	 *
	 * Choosing N = 128 * fs / 1000 makes CTS = TMDS / 1000 exactly, which
	 * is what StarFive's driver does and keeps both inside their 20-bit
	 * fields for every rate and mode this board produces.
	 *
	 * TMDS comes from the transmitter, which knows the mode currently
	 * programmed. If no mode has been set there is nothing being sent and
	 * the values do not matter yet; set_chanspeed runs again on the next
	 * stream.
	 */
	tmds = jh7110_hdmi_tmds_rate();
	if (tmds == 0) {
		device_printf(dev,
		    "no video mode set; audio clock regeneration skipped\n");
		return (speed);
	}

	n = 128 * (speed / 1000);
	cts = tmds / 1000;

	jh7110_hdmi_audio_write(HDMI_N_VALUE1, (n >> 16) & 0xf);
	jh7110_hdmi_audio_write(HDMI_N_VALUE2, (n >> 8) & 0xff);
	jh7110_hdmi_audio_write(HDMI_N_VALUE3, n & 0xff);

	jh7110_hdmi_audio_write(HDMI_CTS_VALUE1, (cts >> 16) & 0xf);
	jh7110_hdmi_audio_write(HDMI_CTS_VALUE2, (cts >> 8) & 0xff);
	jh7110_hdmi_audio_write(HDMI_CTS_VALUE3, cts & 0xff);

	if (bootverbose)
		device_printf(dev, "%u Hz, TMDS %u Hz, N %u, CTS %u\n",
		    speed, tmds, n, cts);

	return (speed);
}

static uint32_t
jh7110_hdmi_audio_dai_set_chanformat(device_t dev, uint32_t format)
{
	struct jh7110_hdmi_audio_softc *sc = device_get_softc(dev);
	uint32_t pins;

	/*
	 * One I2S lane carries two channels, so the number of lanes enabled
	 * follows the channel count.
	 */
	sc->channels = AFMT_CHANNEL(format);
	switch (sc->channels) {
	case 2:	pins = I2S0_ENA; break;
	case 4:	pins = I2S0_ENA | I2S1_ENA; break;
	case 6:	pins = I2S0_ENA | I2S1_ENA | I2S2_ENA; break;
	case 8:	pins = I2S0_ENA | I2S1_ENA | I2S2_ENA | I2S3_ENA; break;
	default:
		pins = I2S0_ENA;
		sc->channels = 2;
		break;
	}

	jh7110_hdmi_audio_modb(HDMI_PINS_ENA, I2S_PIN_ENA_MASK, pins);

	return (0);
}

static int
jh7110_hdmi_audio_dai_trigger(device_t dev, int go, int pcm_dir)
{

	if (pcm_dir != PCMDIR_PLAY)
		return (0);

	switch (go) {
	case PCMTRIG_START:
		jh7110_hdmi_audio_modb(HDMI_DEV_MUTE, AUDIO_MUTE_MASK,
		    AUDIO_NO_MUTE);
		break;
	case PCMTRIG_STOP:
	case PCMTRIG_ABORT:
		jh7110_hdmi_audio_modb(HDMI_DEV_MUTE, AUDIO_MUTE_MASK,
		    AUDIO_MUTE);
		break;
	}

	return (0);
}

static int
jh7110_hdmi_audio_dai_set_sysclk(device_t dev, unsigned int rate, int dai_dir)
{

	return (0);
}

/*
 * OSS mixer. The transmitter itself can only mute, so the gain is the I2S
 * refill's; this exists so that mixer(8) and anything built on the SOUND_MIXER
 * ioctls -- the desktop volume keys included -- have something to talk to.
 */
static int
jh7110_hdmi_mixer_init(struct snd_mixer *m)
{

	/*
	 * One gain, reported under both names. Claiming two independent
	 * controls that secretly move together would be worse than this.
	 */
	mix_setdevs(m, SOUND_MASK_VOLUME | SOUND_MASK_PCM);

	return (0);
}

static int
jh7110_hdmi_mixer_set(struct snd_mixer *m, unsigned dev, unsigned left,
    unsigned right)
{

	if (dev != SOUND_MIXER_VOLUME && dev != SOUND_MIXER_PCM)
		return (-1);

	/* Mono gain: take the louder side rather than silently halving. */
	jh7110_i2s_set_volume(left > right ? (int)left : (int)right);

	return (0);
}

static uint32_t
jh7110_hdmi_mixer_setrecsrc(struct snd_mixer *m, uint32_t src)
{

	return (0);
}

static kobj_method_t jh7110_hdmi_mixer_methods[] = {
	KOBJMETHOD(mixer_init,		jh7110_hdmi_mixer_init),
	KOBJMETHOD(mixer_set,		jh7110_hdmi_mixer_set),
	KOBJMETHOD(mixer_setrecsrc,	jh7110_hdmi_mixer_setrecsrc),
	KOBJMETHOD_END
};
MIXER_DECLARE(jh7110_hdmi_mixer);

static int
jh7110_hdmi_audio_dai_setup_mixer(device_t dev, device_t ausocdev)
{
	struct jh7110_hdmi_audio_softc *sc = device_get_softc(dev);

	if (mixer_init(ausocdev, &jh7110_hdmi_mixer_class, sc) != 0) {
		device_printf(dev, "cannot register mixer\n");
		return (ENXIO);
	}

	return (0);
}

static device_method_t jh7110_hdmi_audio_methods[] = {
	DEVMETHOD(device_probe,			jh7110_hdmi_audio_probe),
	DEVMETHOD(device_attach,		jh7110_hdmi_audio_attach),
	DEVMETHOD(device_detach,		jh7110_hdmi_audio_detach),

	DEVMETHOD(audio_dai_init,		jh7110_hdmi_audio_dai_init),
	DEVMETHOD(audio_dai_set_sysclk,		jh7110_hdmi_audio_dai_set_sysclk),
	DEVMETHOD(audio_dai_set_chanspeed,	jh7110_hdmi_audio_dai_set_chanspeed),
	DEVMETHOD(audio_dai_set_chanformat,	jh7110_hdmi_audio_dai_set_chanformat),
	DEVMETHOD(audio_dai_trigger,		jh7110_hdmi_audio_dai_trigger),
	DEVMETHOD(audio_dai_setup_mixer,	jh7110_hdmi_audio_dai_setup_mixer),

	DEVMETHOD_END
};

static driver_t jh7110_hdmi_audio_driver = {
	"jh7110_hdmi_audio",
	jh7110_hdmi_audio_methods,
	sizeof(struct jh7110_hdmi_audio_softc),
};

DRIVER_MODULE(jh7110_hdmi_audio, simplebus, jh7110_hdmi_audio_driver, 0, 0);
SIMPLEBUS_PNP_INFO(compat_data);
MODULE_DEPEND(jh7110_hdmi_audio, sound, SOUND_MINVER, SOUND_PREFVER,
    SOUND_MAXVER);
