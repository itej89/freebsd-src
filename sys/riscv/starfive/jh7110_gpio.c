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
#include <sys/proc.h>
#include <sys/callout.h>
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
#include "pic_if.h"
#include "fdt_pinctrl_if.h"

#define GPIO_PINS		64
#define GPIO_REGS		2

#define GP0_DOEN_CFG		0x0
#define GP0_DOUT_CFG		0x40
#define GPIOEN			0xdc

/*
 * Interrupt block. The layout follows the PL061 model: a sense register
 * selecting edge or level, a both-edges register, an event register giving
 * the polarity, a mask, and raw/masked status plus a write-1-to-clear.
 * Each is a pair, one register per bank of 32 pins.
 *
 * GPIOE_0/GPIOE_1 are the interrupt mask (Linux calls them GPIOIE0/1); the
 * driver already touched them to disable interrupts at attach.
 */
#define GPIOIS_0		0xe0	/* 0 = edge, 1 = level */
#define GPIOIS_1		0xe4
#define GPIOIC_0		0xe8	/* write 1 to clear */
#define GPIOIC_1		0xec
#define GPIOIBE_0		0xf0	/* 1 = both edges */
#define GPIOIBE_1		0xf4
#define GPIOIEV_0		0xf8	/* 1 = rising/high, 0 = falling/low */
#define GPIOIEV_1		0xfc
#define GPIOE_0			0x100	/* interrupt mask */
#define GPIOE_1			0x104
#define GPIORIS_0		0x108	/* raw status */
#define GPIORIS_1		0x10c
#define GPIOMIS_0		0x110	/* masked status */
#define GPIOMIS_1		0x114
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

struct jh7110_gpio_irqsrc {
	struct intr_irqsrc	isrc;
	uint32_t		pin;
	uint32_t		mode;
	uint32_t		count;	/* interrupts in the current window */
	int			window;	/* tick the window opened */
	int			muzzled_at;
	bool			muzzled;
};

/*
 * A pin that re-asserts faster than it can be serviced will wedge the
 * machine with no panic and no console output. Cap it: report the state
 * that caused it, mask the pin for good, and let the system carry on.
 */
/*
 * Storm threshold, per second. Set well above what real signalling produces
 * - a hot-plug line throws tens of edges as a connector seats and as the
 * transmitter powers up - but far below a stuck source, which re-asserts as
 * fast as it can be acknowledged and reaches this in milliseconds.
 */
#define	JH7110_GPIO_PIC_FULL	5
#define	JH7110_GPIO_STORM_LIMIT	2000

struct jh7110_gpio_softc {
	device_t		dev;
	device_t		busdev;
	struct mtx		mtx;
	struct resource		*res;
	clk_t			clk;

	/*
	 * Interrupt state. imtx is a spin lock and is separate from mtx
	 * deliberately: the PIC methods below run in interrupt context, where
	 * the sleepable mtx used by the pin and pinctrl paths cannot be taken.
	 */
	struct mtx		imtx;
	struct callout		unmuzzle;
	struct resource		*irq_res;
	void			*irq_hdlr;
	struct jh7110_gpio_irqsrc irqsrcs[GPIO_PINS];
};

#define	JH7110_GPIO_ILOCK(_sc)		mtx_lock_spin(&(_sc)->imtx)
#define	JH7110_GPIO_IUNLOCK(_sc)	mtx_unlock_spin(&(_sc)->imtx)

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

/*
 * Interrupt controller.
 *
 * The DTS has always described this block as an interrupt-controller with
 * "interrupts = <86>", but the driver never implemented one, so every GPIO
 * consumer had to poll. HDMI hot-plug detect is the case that made this
 * necessary: polling at DRM's 10 s output-poll period misses a display swap
 * completed inside one interval, leaving the CRTC programmed for the mode of
 * the display that was removed.
 *
 * Interrupts are addressed as GPIO map data rather than through FDT
 * interrupt cells, so consumers reach a pin with gpio_alloc_intr_resource()
 * on a gpio_pin_t they already hold.
 */

#define	PIC_INTR_ISRC(sc, pin)	(&(sc)->irqsrcs[(pin)].isrc)

/*
 * Set or clear one pin's bit in a paired interrupt register. base is the
 * bank-0 offset; bank 1 is the next register. Caller holds imtx.
 */
/*
 * Acknowledge one pin's interrupt.
 *
 * GPIOIC is neither write-1-to-clear nor self-clearing: it reads back what
 * was last written, and the clear is taken on the low-to-high transition.
 * Drive the bit low, then high. StarFive's driver acks the same way and
 * likewise leaves it high afterwards.
 *
 * Caller holds imtx.
 */
static void
jh7110_gpio_iclear(struct jh7110_gpio_softc *sc, uint32_t pin)
{
	bus_size_t reg;
	uint32_t mask, val;

	reg = GPIOIC_0 + (pin / 32) * 4;
	mask = 1u << (pin % 32);

	val = JH7110_GPIO_READ(sc, reg) & ~mask;
	JH7110_GPIO_WRITE(sc, reg, val);
	JH7110_GPIO_WRITE(sc, reg, val | mask);
}

static void
jh7110_gpio_imodify(struct jh7110_gpio_softc *sc, bus_size_t base,
    uint32_t pin, bool set)
{
	bus_size_t reg;
	uint32_t bit, val;

	reg = base + (pin / 32) * 4;
	bit = 1u << (pin % 32);

	val = JH7110_GPIO_READ(sc, reg);
	if (set)
		val |= bit;
	else
		val &= ~bit;
	JH7110_GPIO_WRITE(sc, reg, val);
}

/*
 * Re-enable pins that were masked for interrupting too fast.
 *
 * Masking has to be temporary. A burst that trips the threshold is usually
 * transient - a connector seating, a transmitter powering up - and leaving
 * the pin masked for the rest of the boot turns a momentary problem into a
 * permanent loss of function, which is the outage the guard exists to
 * prevent. A source that is genuinely stuck simply trips again.
 */
static void
jh7110_gpio_unmuzzle(void *arg)
{
	struct jh7110_gpio_softc *sc = arg;
	int i;

	JH7110_GPIO_ILOCK(sc);
	for (i = 0; i < GPIO_PINS; i++) {
		if (!sc->irqsrcs[i].muzzled)
			continue;
		if ((ticks - sc->irqsrcs[i].muzzled_at) < hz)
			continue;

		sc->irqsrcs[i].muzzled = false;
		sc->irqsrcs[i].count = 0;
		jh7110_gpio_iclear(sc, i);
		if (sc->irqsrcs[i].isrc.isrc_handlers != 0)
			jh7110_gpio_imodify(sc, GPIOE_0, i, true);
	}
	JH7110_GPIO_IUNLOCK(sc);

	callout_reset(&sc->unmuzzle, hz, jh7110_gpio_unmuzzle, sc);
}

static void
jh7110_gpio_pic_enable_intr(device_t dev, struct intr_irqsrc *isrc)
{
	struct jh7110_gpio_softc *sc;
	uint32_t pin;

	sc = device_get_softc(dev);
	pin = ((struct jh7110_gpio_irqsrc *)isrc)->pin;

	if (((struct jh7110_gpio_irqsrc *)isrc)->muzzled)
		return;

	JH7110_GPIO_ILOCK(sc);
	jh7110_gpio_imodify(sc, GPIOE_0, pin, true);
	JH7110_GPIO_IUNLOCK(sc);
}

static void
jh7110_gpio_pic_disable_intr(device_t dev, struct intr_irqsrc *isrc)
{
	struct jh7110_gpio_softc *sc;
	uint32_t pin;

	sc = device_get_softc(dev);
	pin = ((struct jh7110_gpio_irqsrc *)isrc)->pin;

	JH7110_GPIO_ILOCK(sc);
	jh7110_gpio_imodify(sc, GPIOE_0, pin, false);
	JH7110_GPIO_IUNLOCK(sc);
}

static int
jh7110_gpio_pic_map_intr(device_t dev, struct intr_map_data *data,
    struct intr_irqsrc **isrcp)
{
	struct jh7110_gpio_softc *sc;
	struct intr_map_data_gpio *gdata;
	uint32_t pin;

	if (data->type != INTR_MAP_DATA_GPIO)
		return (ENOTSUP);

	sc = device_get_softc(dev);
	gdata = (struct intr_map_data_gpio *)data;
	pin = gdata->gpio_pin_num;
	if (pin >= GPIO_PINS) {
		device_printf(dev, "invalid interrupt pin %u\n", pin);
		return (EINVAL);
	}

	*isrcp = PIC_INTR_ISRC(sc, pin);

	return (0);
}

static int
jh7110_gpio_pic_setup_intr(device_t dev, struct intr_irqsrc *isrc,
    struct resource *res, struct intr_map_data *data)
{
	struct jh7110_gpio_softc *sc;
	struct intr_map_data_gpio *gdata;
	struct jh7110_gpio_irqsrc *girq;
	uint32_t mode, pin;

	if (data == NULL)
		return (ENOTSUP);

	sc = device_get_softc(dev);
	gdata = (struct intr_map_data_gpio *)data;
	girq = (struct jh7110_gpio_irqsrc *)isrc;

	mode = gdata->gpio_intr_mode;
	pin = gdata->gpio_pin_num;

	if (girq->pin != pin)
		return (EINVAL);

	/* Already set up: only agree if the mode matches. */
	if (isrc->isrc_handlers != 0)
		return (girq->mode == mode ? 0 : EINVAL);

	girq->mode = mode;


	JH7110_GPIO_ILOCK(sc);

	/*
	 * GPIOIS selects edge or level, and this block is the opposite way
	 * round from the PL061 this driver otherwise resembles: the bit SET
	 * means edge triggered, CLEAR means level triggered. Getting that
	 * backwards puts every pin in level mode, where a line resting at the
	 * active level interrupts forever and no acknowledge can stop it.
	 *
	 * GPIOIEV is the polarity: for edges, set selects rising and clear
	 * selects falling; for levels, clear selects high and set selects low.
	 * GPIOIBE takes both edges and overrides GPIOIEV.
	 */
	if (mode & GPIO_INTR_EDGE_BOTH) {
		jh7110_gpio_imodify(sc, GPIOIS_0, pin, true);
		jh7110_gpio_imodify(sc, GPIOIBE_0, pin, true);
		jh7110_gpio_imodify(sc, GPIOIEV_0, pin, false);
	} else if (mode & GPIO_INTR_EDGE_RISING) {
		jh7110_gpio_imodify(sc, GPIOIS_0, pin, true);
		jh7110_gpio_imodify(sc, GPIOIBE_0, pin, false);
		jh7110_gpio_imodify(sc, GPIOIEV_0, pin, true);
	} else if (mode & GPIO_INTR_EDGE_FALLING) {
		jh7110_gpio_imodify(sc, GPIOIS_0, pin, true);
		jh7110_gpio_imodify(sc, GPIOIBE_0, pin, false);
		jh7110_gpio_imodify(sc, GPIOIEV_0, pin, false);
	} else if (mode & GPIO_INTR_LEVEL_HIGH) {
		jh7110_gpio_imodify(sc, GPIOIS_0, pin, false);
		jh7110_gpio_imodify(sc, GPIOIBE_0, pin, false);
		jh7110_gpio_imodify(sc, GPIOIEV_0, pin, false);
	} else if (mode & GPIO_INTR_LEVEL_LOW) {
		jh7110_gpio_imodify(sc, GPIOIS_0, pin, false);
		jh7110_gpio_imodify(sc, GPIOIBE_0, pin, false);
		jh7110_gpio_imodify(sc, GPIOIEV_0, pin, true);
	} else {
		JH7110_GPIO_IUNLOCK(sc);
		return (EINVAL);
	}

	/* Discard anything latched while the trigger was being changed. */
	jh7110_gpio_iclear(sc, pin);

	JH7110_GPIO_IUNLOCK(sc);

	return (0);
}

static int
jh7110_gpio_pic_teardown_intr(device_t dev, struct intr_irqsrc *isrc,
    struct resource *res, struct intr_map_data *data)
{
	struct jh7110_gpio_softc *sc;
	struct jh7110_gpio_irqsrc *girq;

	sc = device_get_softc(dev);
	girq = (struct jh7110_gpio_irqsrc *)isrc;

	if (isrc->isrc_handlers == 0) {
		girq->mode = GPIO_INTR_CONFORM;
		JH7110_GPIO_ILOCK(sc);
		jh7110_gpio_imodify(sc, GPIOE_0, girq->pin, false);
		JH7110_GPIO_IUNLOCK(sc);
	}

	return (0);
}

static void
jh7110_gpio_pic_post_filter(device_t dev, struct intr_irqsrc *isrc)
{
	struct jh7110_gpio_softc *sc;

	sc = device_get_softc(dev);

	JH7110_GPIO_ILOCK(sc);
	jh7110_gpio_iclear(sc, ((struct jh7110_gpio_irqsrc *)isrc)->pin);
	JH7110_GPIO_IUNLOCK(sc);
}

static void
jh7110_gpio_pic_pre_ithread(device_t dev, struct intr_irqsrc *isrc)
{
	jh7110_gpio_pic_disable_intr(dev, isrc);
}

static void
jh7110_gpio_pic_post_ithread(device_t dev, struct intr_irqsrc *isrc)
{
	jh7110_gpio_pic_post_filter(dev, isrc);
	jh7110_gpio_pic_enable_intr(dev, isrc);
}

static int
jh7110_gpio_intr(void *arg)
{
	struct jh7110_gpio_softc *sc;
	struct trapframe *tf;
	uint32_t status;
	struct jh7110_gpio_irqsrc *girq;
	int bank, bit, pin, handled;

	sc = (struct jh7110_gpio_softc *)arg;
	tf = curthread->td_intr_frame;

	handled = 0;

	for (bank = 0; bank < GPIO_REGS; bank++) {
		status = JH7110_GPIO_READ(sc, GPIOMIS_0 + bank * 4);

		while (status != 0) {
			handled++;
			bit = ffs(status) - 1;
			status &= ~(1u << bit);
			pin = bank * 32 + bit;
			girq = &sc->irqsrcs[pin];

			if (girq->muzzled) {
				JH7110_GPIO_ILOCK(sc);
				jh7110_gpio_imodify(sc, GPIOE_0, pin, false);
				jh7110_gpio_iclear(sc, pin);
				JH7110_GPIO_IUNLOCK(sc);
				continue;
			}

			/*
			 * Storm control is a rate, not a lifetime total: a
			 * burst of contact bounce is normal and must not
			 * permanently disable a pin. Only a source that
			 * stays hot for a whole window is muzzled.
			 */
			if (girq->count == 0 ||
			    (ticks - girq->window) > hz) {
				girq->window = ticks;
				girq->count = 0;
			}
			girq->count++;


			if (girq->count > JH7110_GPIO_STORM_LIMIT) {
				girq->muzzled = true;
				girq->muzzled_at = ticks;
				JH7110_GPIO_ILOCK(sc);
				jh7110_gpio_imodify(sc, GPIOE_0, pin, false);
				jh7110_gpio_iclear(sc, pin);
				JH7110_GPIO_IUNLOCK(sc);
				device_printf(sc->dev,
				    "pin %d storming (%u/sec); masked briefly\n",
				    pin, girq->count);
				continue;
			}

			if (intr_isrc_dispatch(PIC_INTR_ISRC(sc, pin), tf) != 0)
				device_printf(sc->dev,
				    "spurious interrupt on pin %d\n", pin);
		}
	}

	/*
	 * Claiming an interrupt we did not service would let a source that
	 * asserts without a matching GPIOMIS bit re-fire forever, with
	 * nothing ever clearing it. Reporting it as stray instead lets the
	 * framework's storm detection mask the line rather than livelock.
	 */
	if (handled == 0)
		return (FILTER_STRAY);

	return (FILTER_HANDLED);
}

/*
 * Bring up the interrupt controller. Failure here is not fatal: the GPIO
 * side of the driver is perfectly usable without interrupts, and returning
 * an error would take the whole pin controller down with it.
 */
static void
jh7110_gpio_pic_attach(struct jh7110_gpio_softc *sc)
{
	device_t dev;
	const char *name;
	phandle_t xref;
	int i, level, rid;

	dev = sc->dev;
	rid = 0;

	/*
	 * Staged, and off unless asked for.
	 *
	 * An earlier version of this hung the boot a few dozen device
	 * attachments later, with no panic and the console dying mid-printf.
	 * Measuring the block from userspace afterwards showed GPIORIS and
	 * GPIOMIS both zero with every source masked, so it was not an
	 * interrupt storm - which means the fault is somewhere in the
	 * bring-up sequence below.
	 *
	 * Interrupts are on by default now. hw.jh7110_gpio.pic remains so the
	 * bring-up can be cut short from the loader without rebuilding, which
	 * is how the original fault was isolated; setting it to 0 restores the
	 * pre-interrupt behaviour if a board ever misbehaves:
	 *   0  no interrupt support at all
	 *   1  allocate the parent interrupt only
	 *   2  ... and register the per-pin sources
	 *   3  ... and install the parent handler
	 *   4  ... and register as a PIC
	 *   5  ... and let consumers request pin interrupts (full, default)
	 *
	 * Recovery from a bad level is a power cycle without setting the
	 * tunable, rather than pulling the card.
	 */
	level = JH7110_GPIO_PIC_FULL;
	TUNABLE_INT_FETCH("hw.jh7110_gpio.pic", &level);
	if (level <= 0)
		return;

	sc->irq_res = bus_alloc_resource_any(dev, SYS_RES_IRQ, &rid,
	    RF_ACTIVE);
	if (sc->irq_res == NULL) {
		device_printf(dev, "no interrupt; GPIO interrupts disabled\n");
		return;
	}
	if (level < 2) {
		return;
	}

	name = device_get_nameunit(dev);
	for (i = 0; i < GPIO_PINS; i++) {
		sc->irqsrcs[i].pin = i;
		sc->irqsrcs[i].mode = GPIO_INTR_CONFORM;
		if (intr_isrc_register(PIC_INTR_ISRC(sc, i), dev, 0, "%s",
		    name) != 0) {
			device_printf(dev, "cannot register isrc for pin %d\n",
			    i);
			goto fail;
		}
	}
	if (level < 3) {
		return;
	}

	/*
	 * Install the parent handler before registering as a PIC, the order
	 * pl061 uses. Registering first advertises an interrupt controller
	 * whose upstream handler is not hooked up yet.
	 */
	if (bus_setup_intr(dev, sc->irq_res, INTR_TYPE_MISC | INTR_MPSAFE,
	    jh7110_gpio_intr, NULL, sc, &sc->irq_hdlr) != 0) {
		device_printf(dev, "cannot set up interrupt\n");
		goto fail;
	}
	if (level < 4) {
		return;
	}

	/*
	 * A node with no phandle yields xref 0, which is not a valid PIC
	 * identity; pl061 guards this and the first version here did not.
	 */
	xref = OF_xref_from_node(ofw_bus_get_node(dev));
	if (xref == 0) {
		device_printf(dev, "no xref for node; not registering PIC\n");
		goto fail;
	}
	if (!intr_pic_register(dev, xref)) {
		device_printf(dev, "cannot register PIC\n");
		goto fail;
	}

	callout_init_mtx(&sc->unmuzzle, &sc->mtx, 0);
	callout_reset(&sc->unmuzzle, hz, jh7110_gpio_unmuzzle, sc);

	return;

fail:
	bus_release_resource(dev, SYS_RES_IRQ, rid, sc->irq_res);
	sc->irq_res = NULL;
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

	if (sc->irq_hdlr != NULL)
		bus_teardown_intr(dev, sc->irq_res, sc->irq_hdlr);
	if (sc->irq_res != NULL)
		bus_release_resource(dev, SYS_RES_IRQ, 0, sc->irq_res);

	bus_release_resources(dev, jh7110_gpio_spec, &sc->res);
	if (sc->busdev != NULL)
		gpiobus_detach_bus(dev);
	if (sc->clk != NULL)
		clk_release(sc->clk);
	mtx_destroy(&sc->mtx);
	mtx_destroy(&sc->imtx);

	return (0);
}

static int
jh7110_gpio_attach(device_t dev)
{
	struct jh7110_gpio_softc *sc;

	sc = device_get_softc(dev);
	sc->dev = dev;

	mtx_init(&sc->mtx, device_get_nameunit(sc->dev), NULL, MTX_DEF);
	mtx_init(&sc->imtx, device_get_nameunit(sc->dev), "jh7110gpio",
	    MTX_SPIN);

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

	/*
	 * Discard anything latched before we masked everything off. The
	 * clear is a pulse, so these have to be driven back to zero -
	 * leaving them set holds the clear line asserted for every pin.
	 */
	JH7110_GPIO_WRITE(sc, GPIOIC_0, 0x00000000);
	JH7110_GPIO_WRITE(sc, GPIOIC_0, 0xffffffff);
	JH7110_GPIO_WRITE(sc, GPIOIC_0, 0x00000000);
	JH7110_GPIO_WRITE(sc, GPIOIC_1, 0x00000000);
	JH7110_GPIO_WRITE(sc, GPIOIC_1, 0xffffffff);
	JH7110_GPIO_WRITE(sc, GPIOIC_1, 0x00000000);

	/*
	 * Register as an interrupt controller before gpiobus attaches, so a
	 * child asking for a pin interrupt at attach time finds us ready.
	 */
	jh7110_gpio_pic_attach(sc);

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

	/* Interrupt controller interface */
	DEVMETHOD(pic_map_intr,		jh7110_gpio_pic_map_intr),
	DEVMETHOD(pic_setup_intr,	jh7110_gpio_pic_setup_intr),
	DEVMETHOD(pic_teardown_intr,	jh7110_gpio_pic_teardown_intr),
	DEVMETHOD(pic_enable_intr,	jh7110_gpio_pic_enable_intr),
	DEVMETHOD(pic_disable_intr,	jh7110_gpio_pic_disable_intr),
	DEVMETHOD(pic_pre_ithread,	jh7110_gpio_pic_pre_ithread),
	DEVMETHOD(pic_post_ithread,	jh7110_gpio_pic_post_ithread),
	DEVMETHOD(pic_post_filter,	jh7110_gpio_pic_post_filter),

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
