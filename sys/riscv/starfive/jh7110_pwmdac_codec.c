/*-
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * Copyright (c) 2026 Tej <itej89@github.com>
 *
 * Codec for the JH7110 PWMDAC sound card.
 *
 * There is no codec chip on this path -- the PWM pins drive the 3.5 mm jack
 * through an RC filter -- but audio_soc(4) requires a codec device, and the
 * codec is the only device it offers a mixer hook on. The card previously
 * used dummy_codec(4), which has no mixer, so the analogue output had no
 * volume control that anything but a sysctl could reach.
 *
 * So this is a real codec device for a path that has no codec: it owns the
 * OSS mixer and forwards the gain to where the samples are actually written,
 * in jh7110_pwmdac.c. It is also the right place for jack detection or an
 * external amplifier should either ever be wired.
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

#include "jh7110_pwmdac.h"

struct jh7110_pwmdac_codec_softc {
	device_t	dev;
};

static struct ofw_compat_data compat_data[] = {
	{ "starfive,jh7110-pwmdac-codec",	1 },
	{ NULL,					0 }
};

static int
jh7110_pwmdac_codec_probe(device_t dev)
{

	if (!ofw_bus_status_okay(dev))
		return (ENXIO);
	if (!ofw_bus_search_compatible(dev, compat_data)->ocd_data)
		return (ENXIO);

	device_set_desc(dev, "JH7110 PWMDAC codec");
	return (BUS_PROBE_DEFAULT);
}

static int
jh7110_pwmdac_codec_attach(device_t dev)
{
	struct jh7110_pwmdac_codec_softc *sc = device_get_softc(dev);
	phandle_t node;

	sc->dev = dev;
	node = ofw_bus_get_node(dev);
	OF_device_register_xref(OF_xref_from_node(node), dev);

	return (0);
}

static int
jh7110_pwmdac_codec_detach(device_t dev)
{

	return (0);
}

static int
jh7110_pwmdac_codec_dai_init(device_t dev, uint32_t format)
{

	return (0);
}

/*
 * The gain is applied in the PWMDAC's refill, before the noise shaper, so
 * that shaping still works against the value actually converted.
 */
static int
jh7110_pwmdac_mixer_init(struct snd_mixer *m)
{

	mix_setdevs(m, SOUND_MASK_VOLUME | SOUND_MASK_PCM);

	return (0);
}

static int
jh7110_pwmdac_mixer_set(struct snd_mixer *m, unsigned dev, unsigned left,
    unsigned right)
{

	if (dev != SOUND_MIXER_VOLUME && dev != SOUND_MIXER_PCM)
		return (-1);

	/* Mono gain: take the louder side rather than silently halving. */
	jh7110_pwmdac_set_volume(left > right ? (int)left : (int)right);

	return (0);
}

static uint32_t
jh7110_pwmdac_mixer_setrecsrc(struct snd_mixer *m, uint32_t src)
{

	return (0);
}

static kobj_method_t jh7110_pwmdac_mixer_methods[] = {
	KOBJMETHOD(mixer_init,		jh7110_pwmdac_mixer_init),
	KOBJMETHOD(mixer_set,		jh7110_pwmdac_mixer_set),
	KOBJMETHOD(mixer_setrecsrc,	jh7110_pwmdac_mixer_setrecsrc),
	KOBJMETHOD_END
};
MIXER_DECLARE(jh7110_pwmdac_mixer);

static int
jh7110_pwmdac_codec_setup_mixer(device_t dev, device_t ausocdev)
{
	struct jh7110_pwmdac_codec_softc *sc = device_get_softc(dev);

	if (mixer_init(ausocdev, &jh7110_pwmdac_mixer_class, sc) != 0) {
		device_printf(dev, "cannot register mixer\n");
		return (ENXIO);
	}

	return (0);
}

static device_method_t jh7110_pwmdac_codec_methods[] = {
	DEVMETHOD(device_probe,		jh7110_pwmdac_codec_probe),
	DEVMETHOD(device_attach,	jh7110_pwmdac_codec_attach),
	DEVMETHOD(device_detach,	jh7110_pwmdac_codec_detach),

	DEVMETHOD(audio_dai_init,	jh7110_pwmdac_codec_dai_init),
	DEVMETHOD(audio_dai_setup_mixer, jh7110_pwmdac_codec_setup_mixer),

	DEVMETHOD_END
};

static driver_t jh7110_pwmdac_codec_driver = {
	"jh7110_pwmdac_codec",
	jh7110_pwmdac_codec_methods,
	sizeof(struct jh7110_pwmdac_codec_softc),
};

DRIVER_MODULE(jh7110_pwmdac_codec, simplebus, jh7110_pwmdac_codec_driver, 0, 0);
SIMPLEBUS_PNP_INFO(compat_data);
MODULE_DEPEND(jh7110_pwmdac_codec, sound, SOUND_MINVER, SOUND_PREFVER,
    SOUND_MAXVER);
