/*-
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * Copyright (c) 2026 Tej <itej89@github.com>
 *
 * Temperature sensor driver for the StarFive JH7110 SoC.
 * Based on Linux drivers/hwmon/sfctemp.c by Emil Renner Berthing.
 *
 * The sensor uses a single register with control bits and a 12-bit
 * digital output (DOUT). Temperature is calculated as:
 *   Temp(C) = DOUT * 237.5 / 4094 - 81.1
 */

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/bus.h>
#include <sys/kernel.h>
#include <sys/module.h>
#include <sys/sysctl.h>
#include <sys/rman.h>

#include <machine/bus.h>

#include <dev/ofw/openfirm.h>
#include <dev/ofw/ofw_bus.h>
#include <dev/ofw/ofw_bus_subr.h>

#include <dev/clk/clk.h>
#include <dev/hwreset/hwreset.h>

/* Single register — control bits and data output share one 32-bit reg */
#define	SFCTEMP_RSTN		(1 << 0)
#define	SFCTEMP_PD		(1 << 1)
#define	SFCTEMP_RUN		(1 << 2)
#define	SFCTEMP_DOUT_POS	16
#define	SFCTEMP_DOUT_MSK	0x0fff0000

/* Conversion constants: Temp(mC) = DOUT * Y / Z - K */
#define	SFCTEMP_Y1000		237500L
#define	SFCTEMP_Z		4094L
#define	SFCTEMP_K1000		81100L

#define	RD4(sc, off)	bus_read_4((sc)->res, (off))
#define	WR4(sc, off, v)	bus_write_4((sc)->res, (off), (v))

struct jh7110_temp_softc {
	struct resource	*res;
	int		rid;
	clk_t		clk_sense;
	clk_t		clk_bus;
};

static struct ofw_compat_data compat_data[] = {
	{ "starfive,jh7110-temp",	1 },
	{ NULL,				0 }
};

static void
jh7110_temp_power_up(struct jh7110_temp_softc *sc)
{

	WR4(sc, 0, SFCTEMP_PD);
	DELAY(1);
	WR4(sc, 0, 0);
	DELAY(60);
	WR4(sc, 0, SFCTEMP_RSTN);
	DELAY(1);
}

static void
jh7110_temp_run(struct jh7110_temp_softc *sc)
{

	WR4(sc, 0, SFCTEMP_RSTN | SFCTEMP_RUN);
	DELAY(1);
}

static int
jh7110_temp_read_mC(struct jh7110_temp_softc *sc)
{
	uint32_t dout;

	dout = (RD4(sc, 0) & SFCTEMP_DOUT_MSK) >> SFCTEMP_DOUT_POS;

	return ((int)(dout * SFCTEMP_Y1000 / SFCTEMP_Z - SFCTEMP_K1000));
}

static int
jh7110_temp_sysctl(SYSCTL_HANDLER_ARGS)
{
	struct jh7110_temp_softc *sc = arg1;
	int temp_mC, error;

	temp_mC = jh7110_temp_read_mC(sc);

	error = sysctl_handle_int(oidp, &temp_mC, 0, req);

	return (error);
}

static int
jh7110_temp_probe(device_t dev)
{

	if (!ofw_bus_status_okay(dev))
		return (ENXIO);
	if (ofw_bus_search_compatible(dev, compat_data)->ocd_data == 0)
		return (ENXIO);

	device_set_desc(dev, "StarFive JH7110 Temperature Sensor");
	return (BUS_PROBE_DEFAULT);
}

static int
jh7110_temp_attach(device_t dev)
{
	struct jh7110_temp_softc *sc;
	hwreset_t rst;
	int temp;

	sc = device_get_softc(dev);

	sc->rid = 0;
	sc->res = bus_alloc_resource_any(dev, SYS_RES_MEMORY, &sc->rid,
	    RF_ACTIVE);
	if (sc->res == NULL) {
		device_printf(dev, "could not allocate registers\n");
		return (ENXIO);
	}

	/* Enable clocks: "sense" and "bus" */
	if (clk_get_by_ofw_name(dev, 0, "bus", &sc->clk_bus) == 0)
		clk_enable(sc->clk_bus);
	if (clk_get_by_ofw_name(dev, 0, "sense", &sc->clk_sense) == 0)
		clk_enable(sc->clk_sense);

	/* Deassert resets: bus first, then sense */
	for (int i = 0; hwreset_get_by_ofw_idx(dev, 0, i, &rst) == 0; i++)
		hwreset_deassert(rst);

	/* Power up and start continuous conversion */
	jh7110_temp_power_up(sc);
	jh7110_temp_run(sc);

	/* Wait for conversion to complete (needs ~100us per the datasheet) */
	DELAY(100000);
	temp = jh7110_temp_read_mC(sc);
	device_printf(dev, "raw reg: 0x%08x, temperature: %d.%d C\n",
	    RD4(sc, 0), temp / 1000, (temp % 1000) / 100);

	/* Export via sysctl */
	SYSCTL_ADD_PROC(device_get_sysctl_ctx(dev),
	    SYSCTL_CHILDREN(device_get_sysctl_tree(dev)),
	    OID_AUTO, "temperature",
	    CTLTYPE_INT | CTLFLAG_RD | CTLFLAG_NEEDGIANT,
	    sc, 0, jh7110_temp_sysctl, "I",
	    "CPU temperature in milli-Celsius");

	return (0);
}

static int
jh7110_temp_detach(device_t dev)
{
	struct jh7110_temp_softc *sc;

	sc = device_get_softc(dev);

	/* Power down */
	WR4(sc, 0, SFCTEMP_PD);

	if (sc->res != NULL)
		bus_release_resource(dev, SYS_RES_MEMORY, sc->rid, sc->res);

	return (0);
}

static device_method_t jh7110_temp_methods[] = {
	DEVMETHOD(device_probe,		jh7110_temp_probe),
	DEVMETHOD(device_attach,	jh7110_temp_attach),
	DEVMETHOD(device_detach,	jh7110_temp_detach),

	DEVMETHOD_END,
};

DEFINE_CLASS_0(jh7110_temp, jh7110_temp_driver, jh7110_temp_methods,
    sizeof(struct jh7110_temp_softc));

DRIVER_MODULE(jh7110_temp, simplebus, jh7110_temp_driver, NULL, NULL);
