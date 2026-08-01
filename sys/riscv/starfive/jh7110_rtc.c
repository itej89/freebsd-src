/*-
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * Copyright (c) 2026 Tej <itej89@github.com>
 *
 * RTC driver for the StarFive JH7110 SoC.
 * Based on Linux rtc-starfive.c by Emil Renner Berthing / StarFive.
 * The RTC stores time in BCD format in two 32-bit registers (TIME + DATE).
 */

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/bus.h>
#include <sys/clock.h>
#include <sys/kernel.h>
#include <sys/lock.h>
#include <sys/module.h>
#include <sys/mutex.h>

#include <dev/ofw/ofw_bus.h>
#include <dev/ofw/ofw_bus_subr.h>

#include <dev/clk/clk.h>
#include <dev/hwreset/hwreset.h>

#include <machine/bus.h>
#include <sys/rman.h>

#include "clock_if.h"

/*
 * Register offsets — same as the hardware manual and Linux driver.
 *
 * The RTC has two sets of time/date registers:
 *   CFG_TIME/CFG_DATE: write here to SET the time
 *   TIME/DATE:         read here to GET the current time
 */
#define	SFT_RTC_CFG		0x00
#define	SFT_RTC_IRQ_EN		0x10
#define	SFT_RTC_IRQ_EVENT	0x14
#define	SFT_RTC_IRQ_STATUS	0x18
#define	SFT_RTC_CFG_TIME	0x28
#define	SFT_RTC_CFG_DATE	0x2C
#define	SFT_RTC_TIME		0x3C
#define	SFT_RTC_DATE		0x40

/* RTC_CFG register bits */
#define	RTC_CFG_ENABLE		(1 << 0)
#define	RTC_CFG_HOUR_MODE_24	(1 << 3)

/* IRQ event bits */
#define	RTC_IRQ_UPDATE		(1u << 31)

/* IRQ status bits */
#define	RTC_IRQ_1SEC		(1 << 3)

/*
 * TIME register layout (BCD encoded):
 *   bits  6:0  = seconds (0-59)
 *   bits 13:7  = minutes (0-59)
 *   bits 20:14 = hours   (0-23)
 *
 * DATE register layout (BCD encoded):
 *   bits  5:0  = day   (1-31)
 *   bits 10:6  = month (1-12)
 *   bits 18:11 = year  (0-99, relative to 2000)
 */
#define	TIME_SEC_MASK		0x0000007f
#define	TIME_SEC_SHIFT		0
#define	TIME_MIN_MASK		0x00003f80
#define	TIME_MIN_SHIFT		7
#define	TIME_HOUR_MASK		0x001fc000
#define	TIME_HOUR_SHIFT		14

#define	DATE_DAY_MASK		0x0000003f
#define	DATE_DAY_SHIFT		0
#define	DATE_MON_MASK		0x000007c0
#define	DATE_MON_SHIFT		6
#define	DATE_YEAR_MASK		0x0007f800
#define	DATE_YEAR_SHIFT		11

#define	RD4(sc, off)	bus_read_4((sc)->res, (off))
#define	WR4(sc, off, v)	bus_write_4((sc)->res, (off), (v))

static uint8_t
bcd2bin(uint8_t val)
{
	return ((val >> 4) * 10 + (val & 0x0f));
}

static uint8_t
bin2bcd(uint8_t val)
{
	return (((val / 10) << 4) | (val % 10));
}

struct jh7110_rtc_softc {
	struct resource	*res;
	int		rid;
	struct mtx	mtx;
	clk_t		pclk;
	clk_t		cal_clk;
};

static struct ofw_compat_data compat_data[] = {
	{ "starfive,jh7110-rtc",	1 },
	{ NULL,				0 }
};

static int
jh7110_rtc_probe(device_t dev)
{

	if (!ofw_bus_status_okay(dev))
		return (ENXIO);
	if (ofw_bus_search_compatible(dev, compat_data)->ocd_data == 0)
		return (ENXIO);

	device_set_desc(dev, "StarFive JH7110 RTC");
	return (BUS_PROBE_DEFAULT);
}

static int
jh7110_rtc_attach(device_t dev)
{
	struct jh7110_rtc_softc *sc;
	uint32_t val;
	int error;

	sc = device_get_softc(dev);

	sc->rid = 0;
	sc->res = bus_alloc_resource_any(dev, SYS_RES_MEMORY, &sc->rid,
	    RF_ACTIVE);
	if (sc->res == NULL) {
		device_printf(dev, "could not allocate registers\n");
		return (ENXIO);
	}

	/* Enable clocks */
	if (clk_get_by_ofw_name(dev, 0, "pclk", &sc->pclk) == 0)
		clk_enable(sc->pclk);
	if (clk_get_by_ofw_name(dev, 0, "cal_clk", &sc->cal_clk) == 0)
		clk_enable(sc->cal_clk);

	/* Deassert all resets */
	hwreset_t rst;
	for (int i = 0; hwreset_get_by_ofw_idx(dev, 0, i, &rst) == 0; i++)
		hwreset_deassert(rst);

	mtx_init(&sc->mtx, device_get_nameunit(dev), NULL, MTX_DEF);

	/* Enable RTC in 24-hour mode */
	val = RD4(sc, SFT_RTC_CFG);
	val |= RTC_CFG_ENABLE | RTC_CFG_HOUR_MODE_24;
	WR4(sc, SFT_RTC_CFG, val);

	/* Disable all interrupts */
	WR4(sc, SFT_RTC_IRQ_EN, 0);

	clock_register_flags(dev, 1000000, CLOCKF_SETTIME_NO_ADJ);
	clock_schedule(dev, 1);

	return (0);
}

static int
jh7110_rtc_detach(device_t dev)
{
	struct jh7110_rtc_softc *sc;

	sc = device_get_softc(dev);

	clock_unregister(dev);
	mtx_destroy(&sc->mtx);

	if (sc->res != NULL)
		bus_release_resource(dev, SYS_RES_MEMORY, sc->rid, sc->res);

	return (0);
}

static int
jh7110_rtc_gettime(device_t dev, struct timespec *ts)
{
	struct jh7110_rtc_softc *sc;
	struct clocktime ct;
	uint32_t time_reg, date_reg;
	int irq_start, irq_end;

	sc = device_get_softc(dev);

	if (!(RD4(sc, SFT_RTC_CFG) & RTC_CFG_ENABLE))
		return (EINVAL);

	/*
	 * Read time and date registers. The 1-second IRQ status tells us
	 * if a second boundary was crossed during the read — if so, re-read
	 * to avoid getting inconsistent time/date (e.g., 23:59:59 time
	 * with the next day's date).
	 */
	mtx_lock(&sc->mtx);
	irq_start = RD4(sc, SFT_RTC_IRQ_STATUS) & RTC_IRQ_1SEC;

	time_reg = RD4(sc, SFT_RTC_TIME);
	date_reg = RD4(sc, SFT_RTC_DATE);

	if (irq_start == 0) {
		irq_end = RD4(sc, SFT_RTC_IRQ_STATUS) & RTC_IRQ_1SEC;
		if (irq_end != 0) {
			time_reg = RD4(sc, SFT_RTC_TIME);
			date_reg = RD4(sc, SFT_RTC_DATE);
		}
	}
	mtx_unlock(&sc->mtx);

	ct.nsec = 0;
	ct.sec  = bcd2bin((time_reg & TIME_SEC_MASK)  >> TIME_SEC_SHIFT);
	ct.min  = bcd2bin((time_reg & TIME_MIN_MASK)  >> TIME_MIN_SHIFT);
	ct.hour = bcd2bin((time_reg & TIME_HOUR_MASK) >> TIME_HOUR_SHIFT);
	ct.day  = bcd2bin((date_reg & DATE_DAY_MASK)  >> DATE_DAY_SHIFT);
	ct.mon  = bcd2bin((date_reg & DATE_MON_MASK)  >> DATE_MON_SHIFT);
	ct.year = bcd2bin((date_reg & DATE_YEAR_MASK) >> DATE_YEAR_SHIFT) + 2000;
	ct.dow  = -1;

	return (clock_ct_to_ts(&ct, ts));
}

static int
jh7110_rtc_settime(device_t dev, struct timespec *ts)
{
	struct jh7110_rtc_softc *sc;
	struct clocktime ct;
	uint32_t time_reg, date_reg;

	sc = device_get_softc(dev);

	ts->tv_sec -= utc_offset();
	clock_ts_to_ct(ts, &ct);

	time_reg = (bin2bcd(ct.sec)  << TIME_SEC_SHIFT) |
	    (bin2bcd(ct.min)  << TIME_MIN_SHIFT) |
	    (bin2bcd(ct.hour) << TIME_HOUR_SHIFT);

	date_reg = (bin2bcd(ct.day) << DATE_DAY_SHIFT) |
	    (bin2bcd(ct.mon) << DATE_MON_SHIFT) |
	    (bin2bcd(ct.year - 2000) << DATE_YEAR_SHIFT);

	mtx_lock(&sc->mtx);
	WR4(sc, SFT_RTC_CFG_TIME, time_reg);
	WR4(sc, SFT_RTC_CFG_DATE, date_reg);
	WR4(sc, SFT_RTC_IRQ_EVENT, RTC_IRQ_UPDATE);
	mtx_unlock(&sc->mtx);

	return (0);
}

static device_method_t jh7110_rtc_methods[] = {
	DEVMETHOD(device_probe,		jh7110_rtc_probe),
	DEVMETHOD(device_attach,	jh7110_rtc_attach),
	DEVMETHOD(device_detach,	jh7110_rtc_detach),

	DEVMETHOD(clock_gettime,	jh7110_rtc_gettime),
	DEVMETHOD(clock_settime,	jh7110_rtc_settime),

	DEVMETHOD_END,
};

DEFINE_CLASS_0(jh7110_rtc, jh7110_rtc_driver, jh7110_rtc_methods,
    sizeof(struct jh7110_rtc_softc));

DRIVER_MODULE(jh7110_rtc, simplebus, jh7110_rtc_driver, NULL, NULL);
