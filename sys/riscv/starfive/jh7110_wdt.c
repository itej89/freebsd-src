/*-
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * Copyright (c) 2026 Tej <itej89@github.com>
 *
 * Watchdog timer driver for the StarFive JH7110 SoC.
 * Based on Linux starfive-wdt.c.
 *
 * The watchdog counts down from a loaded value. When it reaches zero,
 * it fires an interrupt. On the second timeout (if not cleared), it
 * resets the system.
 */

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/bus.h>
#include <sys/eventhandler.h>
#include <sys/kernel.h>
#include <sys/lock.h>
#include <sys/module.h>
#include <sys/mutex.h>
#include <sys/rman.h>
#include <sys/watchdog.h>

#include <machine/bus.h>

#include <dev/ofw/openfirm.h>
#include <dev/ofw/ofw_bus.h>
#include <dev/ofw/ofw_bus_subr.h>

#include <dev/clk/clk.h>
#include <dev/hwreset/hwreset.h>

/* JH7110 watchdog registers */
#define	JH7110_WDT_LOAD		0x000
#define	JH7110_WDT_VALUE	0x004
#define	JH7110_WDT_CONTROL	0x008
#define	JH7110_WDT_INTCLR	0x00c
#define	JH7110_WDT_IMS		0x014
#define	JH7110_WDT_LOCK		0xc00

#define	JH7110_WDT_UNLOCK_KEY	0x1acce551
#define	JH7110_WDT_CONTROL_EN	(1 << 0)
#define	JH7110_WDT_CONTROL_RSTEN (1 << 1)
#define	JH7110_WDT_INTCLR_BIT	0x1

#define	RD4(sc, off)	bus_read_4((sc)->res, (off))
#define	WR4(sc, off, v)	bus_write_4((sc)->res, (off), (v))

struct jh7110_wdt_softc {
	struct resource		*res;
	int			rid;
	struct mtx		mtx;
	clk_t			clk_apb;
	clk_t			clk_core;
	uint64_t		freq;
	eventhandler_tag	ev_tag;
};

static struct ofw_compat_data compat_data[] = {
	{ "starfive,jh7110-wdt",	1 },
	{ NULL,				0 }
};

static void
jh7110_wdt_unlock(struct jh7110_wdt_softc *sc)
{

	WR4(sc, JH7110_WDT_LOCK, JH7110_WDT_UNLOCK_KEY);
}

static void
jh7110_wdt_lock(struct jh7110_wdt_softc *sc)
{

	WR4(sc, JH7110_WDT_LOCK, ~JH7110_WDT_UNLOCK_KEY);
}

static void
jh7110_wdt_start(struct jh7110_wdt_softc *sc, uint32_t count)
{
	uint32_t val;

	jh7110_wdt_unlock(sc);

	WR4(sc, JH7110_WDT_LOAD, count);
	WR4(sc, JH7110_WDT_INTCLR, JH7110_WDT_INTCLR_BIT);

	val = RD4(sc, JH7110_WDT_CONTROL);
	val |= JH7110_WDT_CONTROL_EN | JH7110_WDT_CONTROL_RSTEN;
	WR4(sc, JH7110_WDT_CONTROL, val);

	jh7110_wdt_lock(sc);
}

static void
jh7110_wdt_stop(struct jh7110_wdt_softc *sc)
{
	uint32_t val;

	jh7110_wdt_unlock(sc);

	val = RD4(sc, JH7110_WDT_CONTROL);
	val &= ~JH7110_WDT_CONTROL_EN;
	WR4(sc, JH7110_WDT_CONTROL, val);

	jh7110_wdt_lock(sc);
}

static void
jh7110_wdt_event(void *arg, unsigned int cmd, int *error)
{
	struct jh7110_wdt_softc *sc = arg;
	uint64_t timeout_ticks;
	int timeout;

	mtx_lock(&sc->mtx);

	timeout = cmd & WD_INTERVAL;

	if (cmd == 0 || timeout == WD_TO_NEVER) {
		jh7110_wdt_stop(sc);
		mtx_unlock(&sc->mtx);
		return;
	}

	/*
	 * Convert FreeBSD watchdog timeout (power-of-2 nanoseconds)
	 * to hardware tick count.
	 * WD_TO_1SEC = 30, meaning 2^30 ns ≈ 1.07 seconds.
	 */
	timeout_ticks = (uint64_t)sc->freq;
	if (timeout >= WD_TO_1SEC)
		timeout_ticks *= (1 << (timeout - WD_TO_1SEC));
	else
		timeout_ticks >>= (WD_TO_1SEC - timeout);

	if (timeout_ticks > 0xffffffff)
		timeout_ticks = 0xffffffff;

	jh7110_wdt_start(sc, (uint32_t)timeout_ticks);
	*error = 0;

	mtx_unlock(&sc->mtx);
}

static int
jh7110_wdt_probe(device_t dev)
{

	if (!ofw_bus_status_okay(dev))
		return (ENXIO);
	if (ofw_bus_search_compatible(dev, compat_data)->ocd_data == 0)
		return (ENXIO);

	device_set_desc(dev, "StarFive JH7110 Watchdog");
	return (BUS_PROBE_DEFAULT);
}

static int
jh7110_wdt_attach(device_t dev)
{
	struct jh7110_wdt_softc *sc;
	hwreset_t rst;

	sc = device_get_softc(dev);

	sc->rid = 0;
	sc->res = bus_alloc_resource_any(dev, SYS_RES_MEMORY, &sc->rid,
	    RF_ACTIVE);
	if (sc->res == NULL) {
		device_printf(dev, "could not allocate registers\n");
		return (ENXIO);
	}

	/* Enable clocks */
	if (clk_get_by_ofw_name(dev, 0, "apb", &sc->clk_apb) == 0)
		clk_enable(sc->clk_apb);
	if (clk_get_by_ofw_name(dev, 0, "core", &sc->clk_core) == 0) {
		clk_enable(sc->clk_core);
		clk_get_freq(sc->clk_core, &sc->freq);
	}

	if (sc->freq == 0)
		sc->freq = 1000000;

	/* Deassert resets */
	for (int i = 0; hwreset_get_by_ofw_idx(dev, 0, i, &rst) == 0; i++)
		hwreset_deassert(rst);

	mtx_init(&sc->mtx, device_get_nameunit(dev), NULL, MTX_DEF);

	/* Start disabled */
	jh7110_wdt_stop(sc);

	sc->ev_tag = EVENTHANDLER_REGISTER(watchdog_list, jh7110_wdt_event,
	    sc, 0);

	device_printf(dev, "watchdog registered (clock %ju Hz)\n",
	    (uintmax_t)sc->freq);

	return (0);
}

static int
jh7110_wdt_detach(device_t dev)
{
	struct jh7110_wdt_softc *sc;

	sc = device_get_softc(dev);

	if (sc->ev_tag != NULL)
		EVENTHANDLER_DEREGISTER(watchdog_list, sc->ev_tag);

	jh7110_wdt_stop(sc);
	mtx_destroy(&sc->mtx);

	if (sc->res != NULL)
		bus_release_resource(dev, SYS_RES_MEMORY, sc->rid, sc->res);

	return (0);
}

static device_method_t jh7110_wdt_methods[] = {
	DEVMETHOD(device_probe,		jh7110_wdt_probe),
	DEVMETHOD(device_attach,	jh7110_wdt_attach),
	DEVMETHOD(device_detach,	jh7110_wdt_detach),

	DEVMETHOD_END,
};

DEFINE_CLASS_0(jh7110_wdt, jh7110_wdt_driver, jh7110_wdt_methods,
    sizeof(struct jh7110_wdt_softc));

DRIVER_MODULE(jh7110_wdt, simplebus, jh7110_wdt_driver, NULL, NULL);
