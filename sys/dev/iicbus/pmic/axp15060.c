/*-
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * Copyright (c) 2026 Tej <itej89@github.com>
 *
 * X-Powers AXP15060 PMIC driver for FreeBSD.
 * Based on the existing FreeBSD AXP81x driver (sys/arm/allwinner/axp81x.c)
 * and the Linux AXP20x MFD/regulator drivers.
 *
 * The AXP15060 is an I2C-attached power management IC used on the
 * StarFive VisionFive 2 board. It provides 23 voltage regulators
 * and controls system power-off/wakeup.
 *
 * Step 1: Skeleton driver with I2C access, probe/attach, and shutdown.
 * Step 2: Full regulator support (enable/disable/voltage control).
 * Step 3: DVFS, IRQ handling, power key, sensors.
 */

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/bus.h>
#include <sys/eventhandler.h>
#include <sys/kernel.h>
#include <sys/lock.h>
#include <sys/module.h>
#include <sys/mutex.h>
#include <sys/reboot.h>
#include <sys/rman.h>

#include <machine/bus.h>

#include <dev/iicbus/iicbus.h>
#include <dev/iicbus/iiconf.h>

#include <dev/ofw/openfirm.h>
#include <dev/ofw/ofw_bus.h>
#include <dev/ofw/ofw_bus_subr.h>

#include "axp15060reg.h"

#include "iicbus_if.h"

struct axp15060_softc {
	device_t	dev;
	uint16_t	addr;	/* I2C slave address (shifted) */
	struct mtx	mtx;
};

static struct ofw_compat_data compat_data[] = {
	{ "x-powers,axp15060",	1 },
	{ NULL,			0 }
};

/*
 * I2C read: write register address, then read data.
 * Same pattern as axp81x.c — two-message I2C transfer.
 */
static int
axp15060_read(device_t dev, uint8_t reg, uint8_t *val)
{
	struct axp15060_softc *sc;
	struct iic_msg msgs[2];

	sc = device_get_softc(dev);

	msgs[0].slave = sc->addr;
	msgs[0].flags = IIC_M_WR;
	msgs[0].len = 1;
	msgs[0].buf = &reg;

	msgs[1].slave = sc->addr;
	msgs[1].flags = IIC_M_RD;
	msgs[1].len = 1;
	msgs[1].buf = val;

	return (iicbus_transfer(dev, msgs, 2));
}

/*
 * I2C write: write register address + value.
 */
static int
axp15060_write(device_t dev, uint8_t reg, uint8_t val)
{
	struct axp15060_softc *sc;
	struct iic_msg msgs[2];

	sc = device_get_softc(dev);

	msgs[0].slave = sc->addr;
	msgs[0].flags = IIC_M_WR;
	msgs[0].len = 1;
	msgs[0].buf = &reg;

	msgs[1].slave = sc->addr;
	msgs[1].flags = IIC_M_WR;
	msgs[1].len = 1;
	msgs[1].buf = &val;

	return (iicbus_transfer(dev, msgs, 2));
}

/*
 * Read-modify-write helper: read register, clear bits in mask,
 * set bits in val, write back.
 */
static int
axp15060_modify(device_t dev, uint8_t reg, uint8_t mask, uint8_t val)
{
	uint8_t tmp;
	int error;

	error = axp15060_read(dev, reg, &tmp);
	if (error != 0)
		return (error);

	tmp &= ~mask;
	tmp |= (val & mask);

	return (axp15060_write(dev, reg, tmp));
}

/*
 * Shutdown handler — called when the system does "shutdown -p now".
 * Write bit 7 of REG 0x32 to tell the PMIC to cut all power rails.
 */
static void
axp15060_shutdown(void *devp, int howto)
{
	device_t dev;

	if ((howto & RB_POWEROFF) == 0)
		return;

	dev = (device_t)devp;

	if (bootverbose)
		device_printf(dev, "Powering off via AXP15060\n");

	axp15060_write(dev, AXP15060_PWR_DISABLE_DOWN_SEQ,
	    AXP15060_POWEROFF);

	/* Give capacitors time to drain */
	DELAY(500000);
}

static phandle_t
axp15060_get_node(device_t dev, device_t bus)
{

	return (ofw_bus_get_node(dev));
}

static int
axp15060_probe(device_t dev)
{

	if (!ofw_bus_status_okay(dev))
		return (ENXIO);
	if (ofw_bus_search_compatible(dev, compat_data)->ocd_data == 0)
		return (ENXIO);

	device_set_desc(dev, "X-Powers AXP15060 Power Management Unit");
	return (BUS_PROBE_DEFAULT);
}

static int
axp15060_attach(device_t dev)
{
	struct axp15060_softc *sc;
	uint8_t chip_id;

	sc = device_get_softc(dev);
	sc->dev = dev;
	sc->addr = iicbus_get_addr(dev);

	mtx_init(&sc->mtx, device_get_nameunit(dev), NULL, MTX_DEF);

	/* Read chip ID to verify communication */
	if (axp15060_read(dev, AXP15060_STARTUP_SRC, &chip_id) != 0) {
		device_printf(dev, "cannot communicate with PMIC\n");
		mtx_destroy(&sc->mtx);
		return (ENXIO);
	}

	device_printf(dev, "AXP15060 PMIC found (startup src: 0x%02x)\n",
	    chip_id);

	/* Register shutdown handler — enables "shutdown -p now" */
	EVENTHANDLER_REGISTER(shutdown_final, axp15060_shutdown, dev,
	    SHUTDOWN_PRI_LAST);

	return (0);
}

static int
axp15060_detach(device_t dev)
{
	struct axp15060_softc *sc;

	sc = device_get_softc(dev);
	mtx_destroy(&sc->mtx);

	return (0);
}

static device_method_t axp15060_methods[] = {
	/* Device interface */
	DEVMETHOD(device_probe,		axp15060_probe),
	DEVMETHOD(device_attach,	axp15060_attach),
	DEVMETHOD(device_detach,	axp15060_detach),

	/* ofw_bus interface — needed for DT child matching */
	DEVMETHOD(ofw_bus_get_node,	axp15060_get_node),

	DEVMETHOD_END,
};

static driver_t axp15060_driver = {
	"axp15060",
	axp15060_methods,
	sizeof(struct axp15060_softc),
};

EARLY_DRIVER_MODULE(axp15060, iicbus, axp15060_driver, 0, 0,
    BUS_PASS_INTERRUPT + BUS_PASS_ORDER_LATE);
MODULE_VERSION(axp15060, 1);
MODULE_DEPEND(axp15060, iicbus, IICBUS_MINVER, IICBUS_PREFVER, IICBUS_MAXVER);
IICBUS_FDT_PNP_INFO(compat_data);
