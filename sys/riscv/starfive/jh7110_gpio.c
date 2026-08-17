/*
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * Copyright (c) 2023 Jari Sihvola <jsihv@gmx.com>
 */

#include <sys/cdefs.h>

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/bus.h>

#include <sys/gpio.h>
#include <sys/kernel.h>
#include <sys/lock.h>
#include <sys/module.h>
#include <sys/mutex.h>
#include <sys/rman.h>

#include <machine/bus.h>
#include <machine/intr.h>
#include <machine/resource.h>

#include <dev/clk/clk.h>
#include <dev/gpio/gpiobusvar.h>
#include <dev/ofw/ofw_bus.h>
#include <dev/ofw/ofw_bus_subr.h>

#include <dev/fdt/fdt_pinctrl.h>

#include "gpio_if.h"
#include "fdt_pinctrl_if.h"

#define GPIO_PINS		64
#define GPIO_REGS		2

#define GP0_DOEN_CFG		0x0
#define GP0_DOUT_CFG		0x40
#define GPIOEN			0xdc
#define GPIOE_0			0x100
#define GPIOE_1			0x104
#define GPIO_DIN_LOW		0x118
#define GPIO_DIN_HIGH		0x11c
#define IOMUX_SYSCFG_288	0x120

#define PAD_INPUT_EN		(1 << 0)
#define PAD_PULLUP		(1 << 3)
#define PAD_PULLDOWN		(1 << 4)
#define PAD_HYST		(1 << 6)

#define ENABLE_MASK		0x3f
#define DATA_OUT_MASK		0x7f
#define DIROUT_DISABLE		1

struct jh7110_gpio_softc {
	device_t		dev;
	device_t		busdev;
	struct mtx		mtx;
	struct resource		*res;
	clk_t			clk;
};

static struct ofw_compat_data compat_data[] = {
	{"starfive,jh7110-sys-pinctrl", 1},
	{NULL,				0}
};

static struct resource_spec jh7110_gpio_spec[] = {
	{ SYS_RES_MEMORY, 0, RF_ACTIVE },
	{ -1, 0 }
};

#define GPIO_RW_OFFSET(_val)		(_val & ~3)
#define GPIO_SHIFT(_val)		((_val & 3) * 8)
#define PAD_OFFSET(_val)		(_val * 4)

#define JH7110_GPIO_LOCK(_sc)		mtx_lock(&(_sc)->mtx)
#define JH7110_GPIO_UNLOCK(_sc)		mtx_unlock(&(_sc)->mtx)

#define JH7110_GPIO_READ(sc, reg)	bus_read_4((sc)->res, (reg))
#define JH7110_GPIO_WRITE(sc, reg, val) bus_write_4((sc)->res, (reg), (val))

static device_t
jh7110_gpio_get_bus(device_t dev)
{
	struct jh7110_gpio_softc *sc;

	sc = device_get_softc(dev);

	return (sc->busdev);
}

static int
jh7110_gpio_pin_max(device_t dev, int *maxpin)
{
	*maxpin = GPIO_PINS - 1;

	return (0);
}

static int
jh7110_gpio_pin_get(device_t dev, uint32_t pin, unsigned int *val)
{
	struct jh7110_gpio_softc *sc;
	uint32_t reg;

	sc = device_get_softc(dev);

	if (pin >= GPIO_PINS)
		return (EINVAL);

	JH7110_GPIO_LOCK(sc);
	if (pin < GPIO_PINS / GPIO_REGS) {
		reg = JH7110_GPIO_READ(sc, GPIO_DIN_LOW);
		*val = (reg >> pin) & 0x1;
	} else {
		reg = JH7110_GPIO_READ(sc, GPIO_DIN_HIGH);
		*val = (reg >> (pin - GPIO_PINS / GPIO_REGS)) & 0x1;
	}
	JH7110_GPIO_UNLOCK(sc);

	return (0);
}

static int
jh7110_gpio_pin_set(device_t dev, uint32_t pin, unsigned int value)
{
	struct jh7110_gpio_softc *sc;
	uint32_t reg;

	sc = device_get_softc(dev);

	if (pin >= GPIO_PINS)
		return (EINVAL);

	JH7110_GPIO_LOCK(sc);
	reg = JH7110_GPIO_READ(sc, GP0_DOUT_CFG + GPIO_RW_OFFSET(pin));
	reg &= ~(DATA_OUT_MASK << GPIO_SHIFT(pin));
	if (value)
		reg |= 0x1 << GPIO_SHIFT(pin);
	JH7110_GPIO_WRITE(sc, GP0_DOUT_CFG + GPIO_RW_OFFSET(pin), reg);
	JH7110_GPIO_UNLOCK(sc);

	return (0);
}

static int
jh7110_gpio_pin_toggle(device_t dev, uint32_t pin)
{
	struct jh7110_gpio_softc *sc;
	uint32_t reg;

	sc = device_get_softc(dev);

	if (pin >= GPIO_PINS)
		return (EINVAL);

	JH7110_GPIO_LOCK(sc);
	reg = JH7110_GPIO_READ(sc, GP0_DOUT_CFG + GPIO_RW_OFFSET(pin));
	if (reg & 0x1 << GPIO_SHIFT(pin)) {
		reg &= ~(DATA_OUT_MASK << GPIO_SHIFT(pin));
	} else {
		reg &= ~(DATA_OUT_MASK << GPIO_SHIFT(pin));
		reg |= 0x1 << GPIO_SHIFT(pin);
	}
	JH7110_GPIO_WRITE(sc, GP0_DOUT_CFG + GPIO_RW_OFFSET(pin), reg);
	JH7110_GPIO_UNLOCK(sc);

	return (0);
}

static int
jh7110_gpio_pin_getcaps(device_t dev, uint32_t pin, uint32_t *caps)
{
	if (pin >= GPIO_PINS)
		return (EINVAL);

	*caps = (GPIO_PIN_INPUT | GPIO_PIN_OUTPUT);

	return (0);
}

static int
jh7110_gpio_pin_getname(device_t dev, uint32_t pin, char *name)
{
	if (pin >= GPIO_PINS)
		return (EINVAL);

	snprintf(name, GPIOMAXNAME, "GPIO%d", pin);

	return (0);
}

static int
jh7110_gpio_pin_getflags(device_t dev, uint32_t pin, uint32_t *flags)
{
	struct jh7110_gpio_softc *sc;
	uint32_t reg;

	sc = device_get_softc(dev);

	if (pin >= GPIO_PINS)
		return (EINVAL);

	/* Reading the direction */
	JH7110_GPIO_LOCK(sc);
	reg = JH7110_GPIO_READ(sc, GP0_DOEN_CFG + GPIO_RW_OFFSET(pin));
	if ((reg & ENABLE_MASK << GPIO_SHIFT(pin)) == 0)
		*flags |= GPIO_PIN_OUTPUT;
	else
		*flags |= GPIO_PIN_INPUT;
	JH7110_GPIO_UNLOCK(sc);

	return (0);
}

static int
jh7110_gpio_pin_setflags(device_t dev, uint32_t pin, uint32_t flags)
{
	struct jh7110_gpio_softc *sc;
	uint32_t reg;

	sc = device_get_softc(dev);

	if (pin >= GPIO_PINS)
		return (EINVAL);

	/* Setting the direction, enable or disable output, configuring pads */

	JH7110_GPIO_LOCK(sc);

	if (flags & GPIO_PIN_INPUT) {
		reg = JH7110_GPIO_READ(sc, IOMUX_SYSCFG_288 + PAD_OFFSET(pin));
		reg |= (PAD_INPUT_EN | PAD_HYST);
		JH7110_GPIO_WRITE(sc, IOMUX_SYSCFG_288 + PAD_OFFSET(pin), reg);
	}

	reg = JH7110_GPIO_READ(sc, GP0_DOEN_CFG + GPIO_RW_OFFSET(pin));
	reg &= ~(ENABLE_MASK << GPIO_SHIFT(pin));
	if (flags & GPIO_PIN_INPUT) {
		reg |= DIROUT_DISABLE << GPIO_SHIFT(pin);
	}
	JH7110_GPIO_WRITE(sc, GP0_DOEN_CFG + GPIO_RW_OFFSET(pin), reg);

	if (flags & GPIO_PIN_OUTPUT) {
		reg = JH7110_GPIO_READ(sc, GP0_DOUT_CFG + GPIO_RW_OFFSET(pin));
		reg &= ~(ENABLE_MASK << GPIO_SHIFT(pin));
		reg |= 0x1 << GPIO_SHIFT(pin);
		JH7110_GPIO_WRITE(sc, GP0_DOUT_CFG + GPIO_RW_OFFSET(pin), reg);

		reg = JH7110_GPIO_READ(sc, IOMUX_SYSCFG_288 + PAD_OFFSET(pin));
		reg &= ~(PAD_INPUT_EN | PAD_PULLUP | PAD_PULLDOWN | PAD_HYST);
		JH7110_GPIO_WRITE(sc, IOMUX_SYSCFG_288 + PAD_OFFSET(pin), reg);
	}

	JH7110_GPIO_UNLOCK(sc);

	return (0);
}

/* Pin mux helpers */
#define	SYS_GPI_BASE		0x080
#define	SYS_GPI_MASK		0x7f
#define	DOUT_MASK_MUX		0x7f
#define	DOEN_MASK_MUX		0x3f
#define	GPI_NONE_VAL		255

#define	PADCFG_IE		(1 << 0)
#define	PADCFG_SMT		(1 << 6)

/*
 * Extract fields from GPIOMUX packed value.
 *
 * GPIOMUX(n, dout, doen, din) packs as
 *     (din << 24) | (dout << 16) | (doen << 10) | n
 * while a dedicated pin uses
 *     PINMUX(n, func) = (1 << 10) | (func << 8) | n
 *
 * Bit 10 is therefore both the PINMUX marker AND the low bit of GPIOMUX's
 * doen field, so testing bit 10 alone misclassifies every GPIOMUX entry
 * whose doen is odd, and its mux is silently skipped. That went unnoticed
 * because every pin group in the tree so far (uart, spi, mmc, i2c) happens
 * to use even doen values; the HDMI group does not -- DDC SCL has doen 3
 * and HPD has doen 1, so both were dropped while SDA (4) and CEC (2) were
 * programmed, leaving EDID reads permanently timing out.
 *
 * A real PINMUX() value carries no din/dout, so its upper half is always
 * zero. Treat a non-zero upper half as proof of GPIOMUX and only fall back
 * to the bit-10 marker otherwise.
 */
#define	PINMUX_IS_GPIO(v)	(((v) >> 16) != 0 || (((v) & (1 << 10)) == 0))
#define	PINMUX_GPIO(v)		((v) & 0x3f)
#define	PINMUX_DOUT(v)		(((v) >> 16) & 0xff)
#define	PINMUX_DOEN(v)		(((v) >> 10) & 0x3f)
#define	PINMUX_DIN(v)		(((v) >> 24) & 0xff)

static void
jh7110_set_pin_mux(struct jh7110_gpio_softc *sc, uint32_t pin,
    uint32_t dout, uint32_t doen, uint32_t din)
{
	uint32_t offset, shift, val;

	offset = 4 * (pin / 4);
	shift = 8 * (pin % 4);

	val = JH7110_GPIO_READ(sc, GP0_DOUT_CFG + offset);
	val &= ~(DOUT_MASK_MUX << shift);
	val |= (dout & DOUT_MASK_MUX) << shift;
	JH7110_GPIO_WRITE(sc, GP0_DOUT_CFG + offset, val);

	val = JH7110_GPIO_READ(sc, GP0_DOEN_CFG + offset);
	val &= ~(DOEN_MASK_MUX << shift);
	val |= (doen & DOEN_MASK_MUX) << shift;
	JH7110_GPIO_WRITE(sc, GP0_DOEN_CFG + offset, val);

	if (din != GPI_NONE_VAL) {
		uint32_t ioffset = 4 * (din / 4);
		uint32_t ishift = 8 * (din % 4);

		val = JH7110_GPIO_READ(sc, SYS_GPI_BASE + ioffset);
		val &= ~(SYS_GPI_MASK << ishift);
		val |= ((pin + 2) & SYS_GPI_MASK) << ishift;
		JH7110_GPIO_WRITE(sc, SYS_GPI_BASE + ioffset, val);
	}
}

static int
jh7110_pinctrl_configure(device_t dev, phandle_t cfgxref)
{
	struct jh7110_gpio_softc *sc = device_get_softc(dev);
	phandle_t node, child;
	uint32_t *pinmux;
	uint32_t padcfg, drive;
	int npins, i;

	node = OF_node_from_xref(cfgxref);

	for (child = OF_child(node); child != 0; child = OF_peer(child)) {
		npins = OF_getencprop_alloc_multi(child, "pinmux",
		    sizeof(uint32_t), (void **)&pinmux);
		if (npins <= 0)
			continue;

		padcfg = 0;
		if (OF_hasprop(child, "input-enable"))
			padcfg |= PADCFG_IE;
		if (OF_hasprop(child, "input-schmitt-enable"))
			padcfg |= PADCFG_SMT;
		if (OF_hasprop(child, "bias-pull-up"))
			padcfg |= PAD_PULLUP;
		else if (OF_hasprop(child, "bias-pull-down"))
			padcfg |= PAD_PULLDOWN;
		if (OF_getencprop(child, "drive-strength", &drive,
		    sizeof(drive)) > 0) {
			if (drive <= 2)
				padcfg |= (0 << 1);
			else if (drive <= 4)
				padcfg |= (1 << 1);
			else if (drive <= 8)
				padcfg |= (2 << 1);
			else
				padcfg |= (3 << 1);
		}

		JH7110_GPIO_LOCK(sc);

		for (i = 0; i < npins; i++) {
			uint32_t v = pinmux[i];
			uint32_t pin;

			if (!PINMUX_IS_GPIO(v)) {
				/* PINMUX() — dedicated pin (64+), pad config only */
				pin = v & 0xff;
				if (pin < 64)
					JH7110_GPIO_WRITE(sc,
					    IOMUX_SYSCFG_288 + PAD_OFFSET(pin),
					    padcfg);
				continue;
			}

			/* GPIOMUX() — GPIO pin 0-63, set mux + pad */
			pin = PINMUX_GPIO(v);
			if (pin >= 64)
				continue;

			jh7110_set_pin_mux(sc, pin,
			    PINMUX_DOUT(v), PINMUX_DOEN(v), PINMUX_DIN(v));
			JH7110_GPIO_WRITE(sc,
			    IOMUX_SYSCFG_288 + PAD_OFFSET(pin), padcfg);
		}

		JH7110_GPIO_UNLOCK(sc);
		OF_prop_free(pinmux);
	}

	return (0);
}

static int
jh7110_gpio_probe(device_t dev)
{
	if (!ofw_bus_status_okay(dev))
		return (ENXIO);

	if (ofw_bus_search_compatible(dev, compat_data)->ocd_data == 0)
		return (ENXIO);

	device_set_desc(dev, "StarFive JH7110 GPIO controller");

	return (BUS_PROBE_DEFAULT);
}

static int
jh7110_gpio_detach(device_t dev)
{
	struct jh7110_gpio_softc *sc;

	sc = device_get_softc(dev);

	bus_release_resources(dev, jh7110_gpio_spec, &sc->res);
	if (sc->busdev != NULL)
		gpiobus_detach_bus(dev);
	if (sc->clk != NULL)
		clk_release(sc->clk);
	mtx_destroy(&sc->mtx);

	return (0);
}

static int
jh7110_gpio_attach(device_t dev)
{
	struct jh7110_gpio_softc *sc;

	sc = device_get_softc(dev);
	sc->dev = dev;

	mtx_init(&sc->mtx, device_get_nameunit(sc->dev), NULL, MTX_DEF);

	if (bus_alloc_resources(dev, jh7110_gpio_spec, &sc->res) != 0) {
		device_printf(dev, "Could not allocate resources\n");
		bus_release_resources(dev, jh7110_gpio_spec, &sc->res);
		mtx_destroy(&sc->mtx);
		return (ENXIO);
	}

	if (clk_get_by_ofw_index(dev, 0, 0, &sc->clk) != 0) {
		device_printf(dev, "Cannot get clock\n");
		jh7110_gpio_detach(dev);
		return (ENXIO);
	}

	if (clk_enable(sc->clk) != 0) {
		device_printf(dev, "Could not enable clock %s\n",
		    clk_get_name(sc->clk));
		jh7110_gpio_detach(dev);
		return (ENXIO);
	}

	/* Reseting GPIO interrupts */
	JH7110_GPIO_WRITE(sc, GPIOE_0, 0);
	JH7110_GPIO_WRITE(sc, GPIOE_1, 0);
	JH7110_GPIO_WRITE(sc, GPIOEN, 1);

	sc->busdev = gpiobus_add_bus(dev);
	if (sc->busdev == NULL) {
		device_printf(dev, "Cannot attach gpiobus\n");
		jh7110_gpio_detach(dev);
		return (ENXIO);
	}

	fdt_pinctrl_register(dev, NULL);
	fdt_pinctrl_configure_tree(dev);

	bus_attach_children(dev);
	return (0);
}

static phandle_t
jh7110_gpio_get_node(device_t bus, device_t dev)
{
	return (ofw_bus_get_node(bus));
}

static device_method_t jh7110_gpio_methods[] = {
	/* Device interface */
	DEVMETHOD(device_probe,		jh7110_gpio_probe),
	DEVMETHOD(device_attach,	jh7110_gpio_attach),
	DEVMETHOD(device_detach,	jh7110_gpio_detach),

	/* GPIO protocol */
	DEVMETHOD(gpio_get_bus,		jh7110_gpio_get_bus),
	DEVMETHOD(gpio_pin_max,		jh7110_gpio_pin_max),
	DEVMETHOD(gpio_pin_get,		jh7110_gpio_pin_get),
	DEVMETHOD(gpio_pin_set,		jh7110_gpio_pin_set),
	DEVMETHOD(gpio_pin_toggle,	jh7110_gpio_pin_toggle),
	DEVMETHOD(gpio_pin_getflags,	jh7110_gpio_pin_getflags),
	DEVMETHOD(gpio_pin_setflags,	jh7110_gpio_pin_setflags),
	DEVMETHOD(gpio_pin_getcaps,	jh7110_gpio_pin_getcaps),
	DEVMETHOD(gpio_pin_getname,	jh7110_gpio_pin_getname),

	/* ofw_bus interface */
	DEVMETHOD(ofw_bus_get_node,	jh7110_gpio_get_node),

	/* fdt_pinctrl interface */
	DEVMETHOD(fdt_pinctrl_configure, jh7110_pinctrl_configure),

	DEVMETHOD_END
};

DEFINE_CLASS_0(gpio, jh7110_gpio_driver, jh7110_gpio_methods,
    sizeof(struct jh7110_gpio_softc));
EARLY_DRIVER_MODULE(jh7110_gpio, simplebus, jh7110_gpio_driver, 0, 0,
    BUS_PASS_INTERRUPT + BUS_PASS_ORDER_MIDDLE);
MODULE_DEPEND(jh7110_gpio, gpiobus, 1, 1, 1);
