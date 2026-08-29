/*-
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * Copyright (c) 2026 Tej <itej89@github.com>
 *
 * Power Management Unit (PMU) driver for the StarFive JH7110 SoC.
 * Controls power gating for VOUT, GPU, ISP, VDEC, VENC domains.
 *
 * Based on Linux drivers/pmdomain/starfive/jh71xx-pmu.c
 *
 * The PMU uses a 3-write "encourage" sequence to switch power states:
 *   1. Write domain mask to SW_TURN_ON/OFF register
 *   2. Write reset command (0xFF) to ENCOURAGE register
 *   3. Write encourage sequence (low then high) to ENCOURAGE register
 *   4. Poll CURR_POWER_MODE until domain bit reflects new state
 */

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/bus.h>
#include <sys/kernel.h>
#include <sys/module.h>
#include <sys/mutex.h>
#include <sys/rman.h>
#include <sys/sysctl.h>

#include <machine/bus.h>

#include <dev/ofw/openfirm.h>
#include <dev/ofw/ofw_bus.h>
#include <dev/ofw/ofw_bus_subr.h>

#include <dev/pwrdom/pwrdom.h>

#include <dt-bindings/power/starfive,jh7110-pmu.h>

#include "pwrdom_if.h"

/* PMU registers */
#define	PMU_SW_TURN_ON		0x0C
#define	PMU_SW_TURN_OFF		0x10
#define	PMU_SW_ENCOURAGE	0x44
#define	PMU_CURR_POWER_MODE	0x80

/* Encourage sequence values */
#define	PMU_ENCOURAGE_RESET	0xFF
#define	PMU_ENCOURAGE_ON_LO	0x05
#define	PMU_ENCOURAGE_ON_HI	0x50
#define	PMU_ENCOURAGE_OFF_LO	0x0A
#define	PMU_ENCOURAGE_OFF_HI	0xA0

/* Timeout for power state change */
#define	PMU_TIMEOUT_US		100000

#define	RD4(sc, off)	bus_read_4((sc)->res, (off))
#define	WR4(sc, off, v)	bus_write_4((sc)->res, (off), (v))

struct jh7110_pmu_softc {
	struct resource	*res;
	int		rid;
	struct mtx	mtx;
};

static struct ofw_compat_data compat_data[] = {
	{ "starfive,jh7110-pmu",	1 },
	{ NULL,				0 }
};

static int
jh7110_pmu_set_domain(struct jh7110_pmu_softc *sc, uint32_t domain, bool on)
{
	uint32_t mask, mode, lo, hi, val;
	int timeout;

	mask = (1 << domain);

	if (on) {
		mode = PMU_SW_TURN_ON;
		lo = PMU_ENCOURAGE_ON_LO;
		hi = PMU_ENCOURAGE_ON_HI;
	} else {
		mode = PMU_SW_TURN_OFF;
		lo = PMU_ENCOURAGE_OFF_LO;
		hi = PMU_ENCOURAGE_OFF_HI;
	}

	mtx_lock(&sc->mtx);

	WR4(sc, mode, mask);
	WR4(sc, PMU_SW_ENCOURAGE, PMU_ENCOURAGE_RESET);
	WR4(sc, PMU_SW_ENCOURAGE, lo);
	WR4(sc, PMU_SW_ENCOURAGE, hi);

	mtx_unlock(&sc->mtx);

	/* Wait for power state change */
	timeout = PMU_TIMEOUT_US;
	while (--timeout > 0) {
		val = RD4(sc, PMU_CURR_POWER_MODE);
		if (on && (val & mask))
			return (0);
		if (!on && !(val & mask))
			return (0);
		DELAY(1);
	}

	return (ETIMEDOUT);
}

static int
jh7110_pmu_pwrdom_enable(device_t dev, intptr_t id)
{
	struct jh7110_pmu_softc *sc;

	sc = device_get_softc(dev);
	return (jh7110_pmu_set_domain(sc, (uint32_t)id, true));
}

static int
jh7110_pmu_pwrdom_disable(device_t dev, intptr_t id)
{
	struct jh7110_pmu_softc *sc;

	sc = device_get_softc(dev);
	return (jh7110_pmu_set_domain(sc, (uint32_t)id, false));
}

static int
jh7110_pmu_pwrdom_is_enabled(device_t dev, intptr_t id, bool *value)
{
	struct jh7110_pmu_softc *sc;
	uint32_t status;

	sc = device_get_softc(dev);
	status = RD4(sc, PMU_CURR_POWER_MODE);
	*value = (status & (1 << (uint32_t)id)) != 0;
	return (0);
}

static int
jh7110_pmu_probe(device_t dev)
{

	if (!ofw_bus_status_okay(dev))
		return (ENXIO);
	if (ofw_bus_search_compatible(dev, compat_data)->ocd_data == 0)
		return (ENXIO);

	device_set_desc(dev, "StarFive JH7110 Power Management Unit");
	return (BUS_PROBE_DEFAULT);
}

static int
jh7110_pmu_attach(device_t dev)
{
	struct jh7110_pmu_softc *sc;
	uint32_t status;
	int error;

	sc = device_get_softc(dev);

	sc->rid = 0;
	sc->res = bus_alloc_resource_any(dev, SYS_RES_MEMORY, &sc->rid,
	    RF_ACTIVE);
	if (sc->res == NULL) {
		device_printf(dev, "could not allocate registers\n");
		return (ENXIO);
	}

	mtx_init(&sc->mtx, device_get_nameunit(dev), NULL, MTX_DEF);

	/* Read current power state */
	status = RD4(sc, PMU_CURR_POWER_MODE);
	device_printf(dev, "current power mode: 0x%02x\n", status);

	/* Power on VOUT domain for display pipeline */
	if (!(status & (1 << JH7110_PD_VOUT))) {
		error = jh7110_pmu_set_domain(sc, JH7110_PD_VOUT, true);
		if (error != 0)
			device_printf(dev, "failed to power on VOUT domain\n");
		else
			device_printf(dev, "VOUT power domain enabled\n");
	}

	/*
	 * Power on GPU domain.
	 *
	 * Debian does NOT do this: there, the GPU device is the domain's genpd
	 * consumer and the island follows it, so CURR_POWER_MODE cycles with
	 * the GPUA bit clear while the GPU is idle ("GPUA off-0 /
	 * 18000000.gpu suspended"). Forcing it on here leaves us pinned at
	 * 0x17 with no consumer able to release it.
	 *
	 * pvr now acquires this domain itself (pvr_device.c), so the force is
	 * only a fallback. hw.jh7110_pmu.force_gpua=0 hands ownership fully to
	 * the consumer, matching the reference platform.
	 */
	{
		char *ev = kern_getenv("hw.jh7110_pmu.force_gpua");
		int force = 1;

		if (ev != NULL) {
			force = (int)strtoul(ev, NULL, 0);
			freeenv(ev);
		}
		if (!force) {
			device_printf(dev,
			    "GPU power domain left to its consumer "
			    "(hw.jh7110_pmu.force_gpua=0)\n");
			goto skip_gpua;
		}
	}

	if (!(status & (1 << JH7110_PD_GPUA))) {
		error = jh7110_pmu_set_domain(sc, JH7110_PD_GPUA, true);
		if (error != 0)
			device_printf(dev, "failed to power on GPU domain\n");
		else
			device_printf(dev, "GPU power domain enabled\n");
	}

skip_gpua:
	pwrdom_register_ofw_provider(dev);

	return (0);
}

static int
jh7110_pmu_detach(device_t dev)
{
	struct jh7110_pmu_softc *sc;

	sc = device_get_softc(dev);

	mtx_destroy(&sc->mtx);
	if (sc->res != NULL)
		bus_release_resource(dev, SYS_RES_MEMORY, sc->rid, sc->res);

	return (0);
}

static device_method_t jh7110_pmu_methods[] = {
	DEVMETHOD(device_probe,		jh7110_pmu_probe),
	DEVMETHOD(device_attach,	jh7110_pmu_attach),
	DEVMETHOD(device_detach,	jh7110_pmu_detach),

	DEVMETHOD(pwrdom_enable,	jh7110_pmu_pwrdom_enable),
	DEVMETHOD(pwrdom_disable,	jh7110_pmu_pwrdom_disable),
	DEVMETHOD(pwrdom_is_enabled,	jh7110_pmu_pwrdom_is_enabled),

	DEVMETHOD_END,
};

DEFINE_CLASS_0(jh7110_pmu, jh7110_pmu_driver, jh7110_pmu_methods,
    sizeof(struct jh7110_pmu_softc));

EARLY_DRIVER_MODULE(jh7110_pmu, simplebus, jh7110_pmu_driver, 0, 0,
    BUS_PASS_BUS + BUS_PASS_ORDER_LATE);
