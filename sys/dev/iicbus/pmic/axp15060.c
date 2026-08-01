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
 */

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/bus.h>
#include <sys/eventhandler.h>
#include <sys/kernel.h>
#include <sys/lock.h>
#include <sys/malloc.h>
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

#include <dev/regulator/regulator.h>

#include "axp15060reg.h"

#include "iicbus_if.h"
#include "regdev_if.h"

static MALLOC_DEFINE(M_AXP15060_REG, "AXP15060 regulator",
    "AXP15060 regulator allocations");

/* Regulator definition — describes one voltage rail */
struct axp15060_regdef {
	intptr_t	id;
	char		*name;
	uint8_t		enable_reg;
	uint8_t		enable_mask;
	uint8_t		voltage_reg;
	uint8_t		voltage_mask;
	int		voltage_min1;	/* mV, range 1 start */
	int		voltage_max1;	/* mV, range 1 end */
	int		voltage_step1;	/* mV, range 1 step (0 = fixed/switch) */
	int		voltage_nstep1;	/* Number of steps in range 1 */
	int		voltage_min2;	/* mV, range 2 start (0 = single range) */
	int		voltage_step2;	/* mV, range 2 step */
	int		voltage_nstep2;	/* Number of steps in range 2 */
};

/* Per-regulator softc, stored inside each regnode */
struct axp15060_reg_sc {
	struct regnode		*regnode;
	device_t		base_dev;
	struct axp15060_regdef	*def;
	phandle_t		xref;
	struct regnode_std_param *param;
};

/*
 * All 23 regulators on the AXP15060.
 * Voltage ranges from the AXP15060 datasheet and Linux driver.
 *
 * For DCDC2/3/4 (two-step ranges) and DCDC5, we use the simplified
 * single-step range that covers the most common use case.
 * Full two-step support is deferred to Step 3.
 */
/*
 * Regulator table macro helpers for readability.
 * SIMPLE: single linear voltage range
 * TWOSTEP: two linear ranges with different step sizes
 * FIXED: fixed voltage, no control register
 * SWITCH: on/off only, no voltage control
 */
#define	SIMPLE(id, nm, ereg, emask, vreg, vmask, min, max, step) \
	{ (id), (nm), (ereg), (emask), (vreg), (vmask), \
	  (min), (max), (step), (((max)-(min))/(step)), 0, 0, 0 }

#define	TWOSTEP(id, nm, ereg, emask, vreg, vmask, \
	    min1, step1, nstep1, min2, step2, nstep2) \
	{ (id), (nm), (ereg), (emask), (vreg), (vmask), \
	  (min1), (min1) + (nstep1) * (step1), (step1), (nstep1), \
	  (min2), (step2), (nstep2) }

#define	FIXED(id, nm, voltage) \
	{ (id), (nm), 0, 0, 0, 0, (voltage), (voltage), 0, 0, 0, 0, 0 }

#define	SWITCH(id, nm, ereg, emask) \
	{ (id), (nm), (ereg), (emask), 0, 0, 0, 0, 0, 0, 0, 0, 0 }

static struct axp15060_regdef axp15060_regdefs[] = {
	/* DCDC1: 1500-3400mV, 100mV steps */
	SIMPLE(AXP15060_REG_DCDC1, "dcdc1",
	    AXP15060_PWR_OUT_CTRL1, AXP15060_DCDC1_EN,
	    AXP15060_DCDC1_V_CTRL, AXP15060_DCDC1_V_MASK,
	    1500, 3400, 100),
	/* DCDC2: 500-1200mV@10mV, 1220-1540mV@20mV */
	TWOSTEP(AXP15060_REG_DCDC2, "dcdc2",
	    AXP15060_PWR_OUT_CTRL1, AXP15060_DCDC2_EN,
	    AXP15060_DCDC2_V_CTRL, AXP15060_DCDC2_V_MASK,
	    500, 10, 70, 1220, 20, 16),
	/* DCDC3: same ranges as DCDC2 */
	TWOSTEP(AXP15060_REG_DCDC3, "dcdc3",
	    AXP15060_PWR_OUT_CTRL1, AXP15060_DCDC3_EN,
	    AXP15060_DCDC3_V_CTRL, AXP15060_DCDC3_V_MASK,
	    500, 10, 70, 1220, 20, 16),
	/* DCDC4: same ranges as DCDC2 */
	TWOSTEP(AXP15060_REG_DCDC4, "dcdc4",
	    AXP15060_PWR_OUT_CTRL1, AXP15060_DCDC4_EN,
	    AXP15060_DCDC4_V_CTRL, AXP15060_DCDC4_V_MASK,
	    500, 10, 70, 1220, 20, 16),
	/* DCDC5: 800-1120mV@10mV, 1140-1840mV@20mV */
	TWOSTEP(AXP15060_REG_DCDC5, "dcdc5",
	    AXP15060_PWR_OUT_CTRL1, AXP15060_DCDC5_EN,
	    AXP15060_DCDC5_V_CTRL, AXP15060_DCDC5_V_MASK,
	    800, 10, 32, 1140, 20, 35),
	/* DCDC6: 500-3400mV, 100mV steps */
	SIMPLE(AXP15060_REG_DCDC6, "dcdc6",
	    AXP15060_PWR_OUT_CTRL1, AXP15060_DCDC6_EN,
	    AXP15060_DCDC6_V_CTRL, AXP15060_DCDC6_V_MASK,
	    500, 3400, 100),
	/* ALDOs: 700-3300mV, 100mV steps */
	SIMPLE(AXP15060_REG_ALDO1, "aldo1",
	    AXP15060_PWR_OUT_CTRL2, AXP15060_ALDO1_EN,
	    AXP15060_ALDO1_V_CTRL, AXP15060_ALDO_V_MASK,
	    700, 3300, 100),
	SIMPLE(AXP15060_REG_ALDO2, "aldo2",
	    AXP15060_PWR_OUT_CTRL2, AXP15060_ALDO2_EN,
	    AXP15060_ALDO2_V_CTRL, AXP15060_ALDO_V_MASK,
	    700, 3300, 100),
	SIMPLE(AXP15060_REG_ALDO3, "aldo3",
	    AXP15060_PWR_OUT_CTRL2, AXP15060_ALDO3_EN,
	    AXP15060_ALDO3_V_CTRL, AXP15060_ALDO_V_MASK,
	    700, 3300, 100),
	SIMPLE(AXP15060_REG_ALDO4, "aldo4",
	    AXP15060_PWR_OUT_CTRL2, AXP15060_ALDO4_EN,
	    AXP15060_ALDO4_V_CTRL, AXP15060_ALDO_V_MASK,
	    700, 3300, 100),
	SIMPLE(AXP15060_REG_ALDO5, "aldo5",
	    AXP15060_PWR_OUT_CTRL2, AXP15060_ALDO5_EN,
	    AXP15060_ALDO5_V_CTRL, AXP15060_ALDO_V_MASK,
	    700, 3300, 100),
	/* BLDOs: 700-3300mV, 100mV steps */
	SIMPLE(AXP15060_REG_BLDO1, "bldo1",
	    AXP15060_PWR_OUT_CTRL2, AXP15060_BLDO1_EN,
	    AXP15060_BLDO1_V_CTRL, AXP15060_BLDO_V_MASK,
	    700, 3300, 100),
	SIMPLE(AXP15060_REG_BLDO2, "bldo2",
	    AXP15060_PWR_OUT_CTRL2, AXP15060_BLDO2_EN,
	    AXP15060_BLDO2_V_CTRL, AXP15060_BLDO_V_MASK,
	    700, 3300, 100),
	SIMPLE(AXP15060_REG_BLDO3, "bldo3",
	    AXP15060_PWR_OUT_CTRL2, AXP15060_BLDO3_EN,
	    AXP15060_BLDO3_V_CTRL, AXP15060_BLDO_V_MASK,
	    700, 3300, 100),
	SIMPLE(AXP15060_REG_BLDO4, "bldo4",
	    AXP15060_PWR_OUT_CTRL3, AXP15060_BLDO4_EN,
	    AXP15060_BLDO4_V_CTRL, AXP15060_BLDO_V_MASK,
	    700, 3300, 100),
	SIMPLE(AXP15060_REG_BLDO5, "bldo5",
	    AXP15060_PWR_OUT_CTRL3, AXP15060_BLDO5_EN,
	    AXP15060_BLDO5_V_CTRL, AXP15060_BLDO_V_MASK,
	    700, 3300, 100),
	/* CLDOs: 700-3300mV (cldo4: 700-4200mV), 100mV steps */
	SIMPLE(AXP15060_REG_CLDO1, "cldo1",
	    AXP15060_PWR_OUT_CTRL3, AXP15060_CLDO1_EN,
	    AXP15060_CLDO1_V_CTRL, AXP15060_CLDO1_V_MASK,
	    700, 3300, 100),
	SIMPLE(AXP15060_REG_CLDO2, "cldo2",
	    AXP15060_PWR_OUT_CTRL3, AXP15060_CLDO2_EN,
	    AXP15060_CLDO2_V_CTRL, AXP15060_CLDO2_V_MASK,
	    700, 3300, 100),
	SIMPLE(AXP15060_REG_CLDO3, "cldo3",
	    AXP15060_PWR_OUT_CTRL3, AXP15060_CLDO3_EN,
	    AXP15060_CLDO3_V_CTRL, AXP15060_CLDO3_V_MASK,
	    700, 3300, 100),
	SIMPLE(AXP15060_REG_CLDO4, "cldo4",
	    AXP15060_PWR_OUT_CTRL3, (1 << 5),
	    AXP15060_CLDO4_V_CTRL, AXP15060_CLDO4_V_MASK,
	    700, 4200, 100),
	/* CPUSLDO: 700-1400mV, 50mV steps */
	SIMPLE(AXP15060_REG_CPUSLDO, "cpusldo",
	    AXP15060_PWR_OUT_CTRL3, (1 << 6),
	    AXP15060_CPUSLDO_V_CTRL, AXP15060_CPUSLDO_V_MASK,
	    700, 1400, 50),
	/* SW: on/off only */
	SWITCH(AXP15060_REG_SW, "sw",
	    AXP15060_PWR_OUT_CTRL3, (1 << 7)),
	/* RTC_LDO: fixed 1.8V, always on */
	FIXED(AXP15060_REG_RTC_LDO, "rtc-ldo", 1800),
};

#define	NREGS	nitems(axp15060_regdefs)

/* Main driver softc */
struct axp15060_softc {
	device_t		dev;
	uint16_t		addr;
	struct mtx		mtx;
	struct axp15060_reg_sc	**regs;
	int			nregs;
};

static struct ofw_compat_data compat_data[] = {
	{ "x-powers,axp15060",	1 },
	{ NULL,			0 }
};

/* I2C helpers */
static int
axp15060_read(device_t dev, uint8_t reg, uint8_t *val)
{
	struct axp15060_softc *sc = device_get_softc(dev);
	struct iic_msg msgs[2];
	int error;

	mtx_lock(&sc->mtx);

	msgs[0].slave = sc->addr;
	msgs[0].flags = IIC_M_WR;
	msgs[0].len = 1;
	msgs[0].buf = &reg;
	msgs[1].slave = sc->addr;
	msgs[1].flags = IIC_M_RD;
	msgs[1].len = 1;
	msgs[1].buf = val;

	error = iicbus_transfer(dev, msgs, 2);

	mtx_unlock(&sc->mtx);
	return (error);
}

static int
axp15060_write(device_t dev, uint8_t reg, uint8_t val)
{
	struct axp15060_softc *sc = device_get_softc(dev);
	struct iic_msg msgs[2];
	int error;

	mtx_lock(&sc->mtx);

	msgs[0].slave = sc->addr;
	msgs[0].flags = IIC_M_WR;
	msgs[0].len = 1;
	msgs[0].buf = &reg;
	msgs[1].slave = sc->addr;
	msgs[1].flags = IIC_M_WR;
	msgs[1].len = 1;
	msgs[1].buf = &val;

	error = iicbus_transfer(dev, msgs, 2);

	mtx_unlock(&sc->mtx);
	return (error);
}

/* ================================================================
 * Regulator node methods (regnode_if.m)
 * ================================================================ */

static int
axp15060_regnode_init(struct regnode *regnode)
{

	return (0);
}

static int
axp15060_regnode_enable(struct regnode *regnode, bool enable, int *udelay)
{
	struct axp15060_reg_sc *sc = regnode_get_softc(regnode);
	uint8_t val;

	/* No enable register = always on (e.g., rtc-ldo) */
	if (sc->def->enable_mask == 0) {
		*udelay = 0;
		return (enable ? 0 : EINVAL);
	}

	axp15060_read(sc->base_dev, sc->def->enable_reg, &val);
	if (enable)
		val |= sc->def->enable_mask;
	else
		val &= ~sc->def->enable_mask;
	axp15060_write(sc->base_dev, sc->def->enable_reg, val);

	*udelay = 0;
	return (0);
}

static int
axp15060_regnode_status(struct regnode *regnode, int *status)
{
	struct axp15060_reg_sc *sc = regnode_get_softc(regnode);
	uint8_t val;

	/* No enable register = always on */
	if (sc->def->enable_mask == 0) {
		*status = REGULATOR_STATUS_ENABLED;
		return (0);
	}

	*status = 0;
	axp15060_read(sc->base_dev, sc->def->enable_reg, &val);
	if (val & sc->def->enable_mask)
		*status = REGULATOR_STATUS_ENABLED;

	return (0);
}

/*
 * Convert a register selector value to microvolts.
 * Handles both simple (single range) and two-step regulators.
 */
static void
axp15060_sel_to_uvolt(struct axp15060_regdef *def, uint8_t sel, int *uvolt)
{

	if (def->voltage_step1 == 0) {
		*uvolt = def->voltage_min1 * 1000;
		return;
	}

	if (sel <= def->voltage_nstep1)
		*uvolt = (def->voltage_min1 + sel * def->voltage_step1) * 1000;
	else if (def->voltage_step2 > 0)
		*uvolt = (def->voltage_min2 +
		    (sel - def->voltage_nstep1 - 1) * def->voltage_step2) *
		    1000;
	else
		*uvolt = def->voltage_max1 * 1000;
}

/*
 * Convert microvolts to a register selector value.
 * Returns 0 on success, ERANGE if the voltage is out of range.
 */
static int
axp15060_uvolt_to_sel(struct axp15060_regdef *def, int min_uvolt,
    int max_uvolt, uint8_t *sel)
{
	int uvolt;
	uint8_t s;

	if (def->voltage_step1 == 0)
		return (EINVAL);

	/* Search range 1 */
	for (s = 0; s <= def->voltage_nstep1; s++) {
		uvolt = (def->voltage_min1 + s * def->voltage_step1) * 1000;
		if (uvolt >= min_uvolt && uvolt <= max_uvolt) {
			*sel = s;
			return (0);
		}
	}

	/* Search range 2 */
	if (def->voltage_step2 > 0) {
		for (s = 0; s <= def->voltage_nstep2; s++) {
			uvolt = (def->voltage_min2 +
			    s * def->voltage_step2) * 1000;
			if (uvolt >= min_uvolt && uvolt <= max_uvolt) {
				*sel = def->voltage_nstep1 + 1 + s;
				return (0);
			}
		}
	}

	return (ERANGE);
}

static int
axp15060_regnode_get_voltage(struct regnode *regnode, int *uvolt)
{
	struct axp15060_reg_sc *sc = regnode_get_softc(regnode);
	uint8_t val;

	/* Fixed voltage (rtc-ldo) or switch */
	if (sc->def->voltage_step1 == 0) {
		*uvolt = sc->def->voltage_min1 * 1000;
		return (0);
	}

	if (sc->def->voltage_reg == 0)
		return (ENXIO);

	axp15060_read(sc->base_dev, sc->def->voltage_reg, &val);
	val &= sc->def->voltage_mask;
	axp15060_sel_to_uvolt(sc->def, val, uvolt);

	return (0);
}

static int
axp15060_regnode_set_voltage(struct regnode *regnode, int min_uvolt,
    int max_uvolt, int *udelay)
{
	struct axp15060_reg_sc *sc = regnode_get_softc(regnode);
	uint8_t sel, val;
	int error;

	if (sc->def->voltage_step1 == 0 || sc->def->voltage_reg == 0)
		return (EINVAL);

	error = axp15060_uvolt_to_sel(sc->def, min_uvolt, max_uvolt, &sel);
	if (error != 0)
		return (error);

	axp15060_read(sc->base_dev, sc->def->voltage_reg, &val);
	val &= ~sc->def->voltage_mask;
	val |= (sel & sc->def->voltage_mask);
	axp15060_write(sc->base_dev, sc->def->voltage_reg, val);

	*udelay = 0;
	return (0);
}

static regnode_method_t axp15060_regnode_methods[] = {
	REGNODEMETHOD(regnode_init,		axp15060_regnode_init),
	REGNODEMETHOD(regnode_enable,		axp15060_regnode_enable),
	REGNODEMETHOD(regnode_status,		axp15060_regnode_status),
	REGNODEMETHOD(regnode_get_voltage,	axp15060_regnode_get_voltage),
	REGNODEMETHOD(regnode_check_voltage,	regnode_method_check_voltage),
	REGNODEMETHOD_END
};

DEFINE_CLASS_1(axp15060_regnode, axp15060_regnode_class,
    axp15060_regnode_methods, sizeof(struct axp15060_reg_sc), regnode_class);

/* Register one regulator from the DT "regulators" subnode */
static struct axp15060_reg_sc *
axp15060_reg_attach(device_t dev, phandle_t node, struct axp15060_regdef *def)
{
	struct axp15060_reg_sc *reg_sc;
	struct regnode_init_def initdef;
	struct regnode *regnode;

	memset(&initdef, 0, sizeof(initdef));
	if (regulator_parse_ofw_stdparam(dev, node, &initdef) != 0)
		return (NULL);
	if (initdef.std_param.min_uvolt == 0)
		initdef.std_param.min_uvolt = def->voltage_min1 * 1000;
	if (initdef.std_param.max_uvolt == 0)
		initdef.std_param.max_uvolt = def->voltage_max1 * 1000;
	initdef.id = def->id;
	initdef.ofw_node = node;

	regnode = regnode_create(dev, &axp15060_regnode_class, &initdef);
	if (regnode == NULL) {
		device_printf(dev, "cannot create regulator %s\n", def->name);
		return (NULL);
	}

	reg_sc = regnode_get_softc(regnode);
	reg_sc->regnode = regnode;
	reg_sc->base_dev = dev;
	reg_sc->def = def;
	reg_sc->xref = OF_xref_from_node(node);
	reg_sc->param = regnode_get_stdparam(regnode);

	regnode_register(regnode);

	return (reg_sc);
}

/* Map DT phandle references to regulator IDs */
static int
axp15060_regdev_map(device_t dev, phandle_t xref, int ncells, pcell_t *cells,
    intptr_t *num)
{
	struct axp15060_softc *sc = device_get_softc(dev);
	int i;

	for (i = 0; i < sc->nregs; i++) {
		if (sc->regs[i] == NULL)
			continue;
		if (sc->regs[i]->xref == xref) {
			*num = sc->regs[i]->def->id;
			return (0);
		}
	}

	return (ENXIO);
}

/* ================================================================
 * Shutdown handler
 * ================================================================ */

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

	DELAY(500000);
}

/* ================================================================
 * Driver entry points
 * ================================================================ */

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
	struct axp15060_reg_sc *reg;
	uint8_t chip_id;
	phandle_t rnode, child;
	int i;

	sc = device_get_softc(dev);
	sc->dev = dev;
	sc->addr = iicbus_get_addr(dev);
	sc->nregs = NREGS;

	mtx_init(&sc->mtx, device_get_nameunit(dev), NULL, MTX_DEF);

	if (axp15060_read(dev, AXP15060_STARTUP_SRC, &chip_id) != 0) {
		device_printf(dev, "cannot communicate with PMIC\n");
		mtx_destroy(&sc->mtx);
		return (ENXIO);
	}

	device_printf(dev, "AXP15060 PMIC found (startup src: 0x%02x)\n",
	    chip_id);

	/* Attach regulators from DT "regulators" subnode */
	sc->regs = malloc(sizeof(struct axp15060_reg_sc *) * sc->nregs,
	    M_AXP15060_REG, M_WAITOK | M_ZERO);

	rnode = ofw_bus_find_child(ofw_bus_get_node(dev), "regulators");
	for (i = 0; i < sc->nregs; i++) {
		child = 0;
		if (rnode > 0)
			child = ofw_bus_find_child(rnode,
			    axp15060_regdefs[i].name);

		if (child != 0) {
			reg = axp15060_reg_attach(dev, child,
			    &axp15060_regdefs[i]);
		} else {
			/* Register even without DT node for full chip control */
			struct regnode_init_def initdef;
			struct regnode *regnode;
			struct axp15060_reg_sc *reg_sc;

			memset(&initdef, 0, sizeof(initdef));
			initdef.name = axp15060_regdefs[i].name;
			initdef.id = axp15060_regdefs[i].id;
			if (axp15060_regdefs[i].voltage_min1 > 0) {
				initdef.std_param.min_uvolt =
				    axp15060_regdefs[i].voltage_min1 * 1000;
				initdef.std_param.max_uvolt =
				    axp15060_regdefs[i].voltage_max1 * 1000;
			}
			regnode = regnode_create(dev,
			    &axp15060_regnode_class, &initdef);
			if (regnode == NULL) {
				device_printf(dev,
				    "cannot create regulator %s\n",
				    axp15060_regdefs[i].name);
				continue;
			}
			reg_sc = regnode_get_softc(regnode);
			reg_sc->regnode = regnode;
			reg_sc->base_dev = dev;
			reg_sc->def = &axp15060_regdefs[i];
			reg_sc->xref = 0;
			reg_sc->param = regnode_get_stdparam(regnode);
			regnode_register(regnode);
			reg = reg_sc;
		}

		if (reg == NULL) {
			device_printf(dev, "cannot attach regulator %s\n",
			    axp15060_regdefs[i].name);
			continue;
		}
		sc->regs[i] = reg;
	}

	EVENTHANDLER_REGISTER(shutdown_final, axp15060_shutdown, dev,
	    SHUTDOWN_PRI_LAST);

	return (0);
}

static int
axp15060_detach(device_t dev)
{
	struct axp15060_softc *sc = device_get_softc(dev);

	if (sc->regs != NULL)
		free(sc->regs, M_AXP15060_REG);
	mtx_destroy(&sc->mtx);

	return (0);
}

static device_method_t axp15060_methods[] = {
	DEVMETHOD(device_probe,		axp15060_probe),
	DEVMETHOD(device_attach,	axp15060_attach),
	DEVMETHOD(device_detach,	axp15060_detach),

	DEVMETHOD(regdev_map,		axp15060_regdev_map),

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
