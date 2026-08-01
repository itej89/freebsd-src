/*-
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * Copyright (c) 2026 Tej <itej89@github.com>
 *
 * FDT (Flattened Device Tree) attachment for the Synopsys DesignWare I2C
 * controller. Enables the ig4 driver on SoCs like the StarFive JH7110 where
 * the I2C controller is described in the device tree rather than PCI or ACPI.
 */

#include "opt_platform.h"

#include <sys/param.h>
#include <sys/bus.h>
#include <sys/kernel.h>
#include <sys/module.h>
#include <sys/rman.h>

#include <machine/bus.h>
#include <machine/resource.h>

#include <dev/iicbus/iicbus.h>
#include <dev/iicbus/iiconf.h>

/*
 * OFW = Open Firmware. This is FreeBSD's device tree interface.
 * These headers let us read properties from the DTB like "compatible",
 * "reg", "clocks", "resets", etc.
 */
#include <dev/ofw/openfirm.h>
#include <dev/ofw/ofw_bus.h>
#include <dev/ofw/ofw_bus_subr.h>

/*
 * Clock and reset frameworks. On a SoC, peripherals start powered off.
 * We must enable the clock and release the reset before touching registers.
 * (Arduino doesn't need this — ATmega peripherals are always clocked.)
 */
#include <dev/clk/clk.h>
#include <dev/hwreset/hwreset.h>

/*
 * The ig4 core driver headers. ig4_reg.h has register definitions,
 * ig4_var.h has the softc structure and shared function declarations.
 */
#include <dev/ichiic/ig4_reg.h>
#include <dev/ichiic/ig4_var.h>

/*
 * Our softc wraps ig4iic_softc and adds clock/reset handles.
 * The ig4 core driver doesn't know about clocks/resets (Intel doesn't
 * need them), so we manage them in this FDT-specific wrapper.
 *
 * Think of it like: ig4iic_softc is the engine, this struct adds
 * the key to start it (clock) and the parking brake (reset).
 */
struct ig4iic_fdt_softc {
	struct ig4iic_softc	base;
	clk_t			core_clk;
	hwreset_t		reset;
};

/*
 * Compatible string table. The kernel walks every DTB node and checks:
 * "does any driver match this compatible string?"
 *
 * This is like Arduino's board selection — it tells FreeBSD
 * "I know how to drive hardware described as snps,designware-i2c."
 */
static int ig4iic_fdt_detach(device_t dev);

static struct ofw_compat_data compat_data[] = {
	{ "snps,designware-i2c",	1 },
	{ NULL,				0 }
};

/*
 * PROBE: "Is this device mine?"
 *
 * Called by the kernel for every DTB node on simplebus.
 * We check: does this node's "compatible" match our table?
 * And is "status" set to "okay"?
 *
 * Returns BUS_PROBE_DEFAULT (0) if yes — "I can drive this."
 * Returns ENXIO if no — "Not my device, try another driver."
 */
static int
ig4iic_fdt_probe(device_t dev)
{

	if (!ofw_bus_status_okay(dev))
		return (ENXIO);
	if (ofw_bus_search_compatible(dev, compat_data)->ocd_data == 0)
		return (ENXIO);

	device_set_desc(dev, "Synopsys DesignWare I2C Controller");
	return (BUS_PROBE_DEFAULT);
}

/*
 * ATTACH: "Set up this device for use."
 *
 * Called after probe() succeeds. This is where we:
 * 1. Enable the clock (power on the peripheral)
 * 2. Deassert the reset (release the peripheral from reset)
 * 3. Allocate register memory and IRQ
 * 4. Call the core ig4 driver to do the real I2C setup
 *
 * This is like Arduino's setup() — runs once, gets hardware ready.
 */
static int
ig4iic_fdt_attach(device_t dev)
{
	struct ig4iic_fdt_softc *fsc;
	ig4iic_softc_t *sc;
	int error;

	fsc = device_get_softc(dev);
	sc = &fsc->base;
	sc->dev = dev;

	/*
	 * Set the version to IG4_DESIGNWARE so the core driver skips
	 * all Intel-specific register accesses (LPSS, DEVIDLE, etc.)
	 */
	sc->version = IG4_DESIGNWARE;

	/*
	 * Step 1: Enable the clock.
	 * The DTS says: clocks = <&syscrg JH7110_SYSCLK_I2C0_APB>;
	 * Without this, the peripheral has no clock signal and all
	 * register reads return garbage (usually 0xFFFFFFFF).
	 */
	error = clk_get_by_ofw_index(dev, 0, 0, &fsc->core_clk);
	if (error == 0) {
		error = clk_enable(fsc->core_clk);
		if (error != 0) {
			device_printf(dev, "could not enable clock: %d\n",
			    error);
			return (error);
		}
	}

	/*
	 * Step 2: Deassert the reset.
	 * The DTS says: resets = <&syscrg JH7110_SYSRST_I2C0_APB>;
	 * The peripheral starts in reset (held in reset state).
	 * We must release it before it will respond to register writes.
	 * Like releasing the reset button on an Arduino.
	 */
	error = hwreset_get_by_ofw_idx(dev, 0, 0, &fsc->reset);
	if (error == 0) {
		error = hwreset_deassert(fsc->reset);
		if (error != 0) {
			device_printf(dev, "could not deassert reset: %d\n",
			    error);
			goto fail;
		}
	}

	/*
	 * Step 3: Allocate register memory.
	 * The DTS says: reg = <0x0 0x10030000 0x0 0x10000>;
	 * This maps the physical address into the kernel's virtual address
	 * space so we can read/write registers with bus_read_4/bus_write_4.
	 * Like Arduino's TWCR register, but we have to ask the OS to map it.
	 */
	sc->regs_rid = 0;
	sc->regs_res = bus_alloc_resource_any(dev, SYS_RES_MEMORY,
	    &sc->regs_rid, RF_ACTIVE);
	if (sc->regs_res == NULL) {
		device_printf(dev, "could not allocate register memory\n");
		error = ENXIO;
		goto fail;
	}

	/*
	 * Step 4: Allocate the interrupt.
	 * The DTS says: interrupts = <35>;
	 * When an I2C transfer completes, the hardware fires this IRQ.
	 * The core driver sets up a handler in ig4iic_attach().
	 */
	sc->intr_rid = 0;
	sc->intr_res = bus_alloc_resource_any(dev, SYS_RES_IRQ,
	    &sc->intr_rid, RF_SHAREABLE | RF_ACTIVE);
	if (sc->intr_res == NULL) {
		device_printf(dev, "could not allocate interrupt\n");
		error = ENXIO;
		goto fail;
	}

	sc->platform_attached = true;

	/*
	 * Step 5: Call the core ig4 driver.
	 * This does all the real I2C work: configure timing registers,
	 * set up FIFO, create the iicbus child device,
	 * install the interrupt handler.
	 */
	error = ig4iic_attach(sc);
	if (error != 0)
		goto fail;

	return (0);

fail:
	ig4iic_fdt_detach(dev);
	return (error);
}

/*
 * DETACH: "Shut down this device."
 *
 * Called when the driver is unloaded or the device is removed.
 * Reverse of attach: release IRQ, unmap registers, assert reset,
 * disable clock. Order matters — disable in reverse order of enable.
 */
static int
ig4iic_fdt_detach(device_t dev)
{
	struct ig4iic_fdt_softc *fsc;
	ig4iic_softc_t *sc;
	int error;

	fsc = device_get_softc(dev);
	sc = &fsc->base;

	if (sc->platform_attached) {
		error = ig4iic_detach(sc);
		if (error != 0)
			return (error);
		sc->platform_attached = false;
	}

	if (sc->intr_res != NULL) {
		bus_release_resource(dev, SYS_RES_IRQ,
		    sc->intr_rid, sc->intr_res);
		sc->intr_res = NULL;
	}
	if (sc->regs_res != NULL) {
		bus_release_resource(dev, SYS_RES_MEMORY,
		    sc->regs_rid, sc->regs_res);
		sc->regs_res = NULL;
	}
	if (fsc->reset != NULL)
		hwreset_assert(fsc->reset);
	if (fsc->core_clk != NULL)
		clk_disable(fsc->core_clk);

	return (0);
}

/*
 * Device method table. This tells the kernel:
 * - How to probe/attach/detach this driver
 * - How to forward bus operations to children (I2C devices)
 * - How to do I2C transfers (delegated to ig4 core driver)
 *
 * Think of it as a vtable in C++ — function pointers that the
 * kernel calls at the right time.
 */
static phandle_t
ig4iic_fdt_get_node(device_t bus, device_t dev)
{

	return (ofw_bus_get_node(bus));
}

static device_method_t ig4iic_fdt_methods[] = {
	/* Device interface */
	DEVMETHOD(device_probe,		ig4iic_fdt_probe),
	DEVMETHOD(device_attach,	ig4iic_fdt_attach),
	DEVMETHOD(device_detach,	ig4iic_fdt_detach),
	DEVMETHOD(device_suspend,	bus_generic_suspend),
	DEVMETHOD(device_resume,	bus_generic_resume),

	/* Bus interface — forward to generic implementations */
	DEVMETHOD(bus_setup_intr,		bus_generic_setup_intr),
	DEVMETHOD(bus_teardown_intr,		bus_generic_teardown_intr),
	DEVMETHOD(bus_alloc_resource,		bus_generic_alloc_resource),
	DEVMETHOD(bus_release_resource,		bus_generic_release_resource),
	DEVMETHOD(bus_activate_resource,	bus_generic_activate_resource),
	DEVMETHOD(bus_deactivate_resource,	bus_generic_deactivate_resource),
	DEVMETHOD(bus_adjust_resource,		bus_generic_adjust_resource),

	/* ofw_bus interface — lets ofw_iicbus find our DT node */
	DEVMETHOD(ofw_bus_get_node,	ig4iic_fdt_get_node),

	/* iicbus interface — delegated to ig4 core driver */
	DEVMETHOD(iicbus_transfer,	ig4iic_transfer),
	DEVMETHOD(iicbus_reset,		ig4iic_reset),
	DEVMETHOD(iicbus_callback,	ig4iic_callback),

	DEVMETHOD_END
};

/*
 * Driver declaration. This ties everything together:
 * - Name: "ig4iic" (same as PCI/ACPI versions — it's the same driver)
 * - Methods: our FDT-specific probe/attach + shared iicbus methods
 * - Softc size: our wrapper struct (which contains ig4iic_softc inside)
 */
static driver_t ig4iic_fdt_driver = {
	"ig4iic",
	ig4iic_fdt_methods,
	sizeof(struct ig4iic_fdt_softc),
};

/*
 * DRIVER_MODULE: Register this driver with the kernel.
 * "ig4iic on simplebus" — because FDT devices appear under simplebus.
 * (PCI version says "ig4iic on pci", ACPI version says "ig4iic on acpi")
 */
EARLY_DRIVER_MODULE(ig4iic, simplebus, ig4iic_fdt_driver, 0, 0,
    BUS_PASS_BUS);
EARLY_DRIVER_MODULE(ofw_iicbus, ig4iic, ofw_iicbus_driver, 0, 0,
    BUS_PASS_BUS);
MODULE_DEPEND(ig4iic, iicbus, 1, 1, 1);
SIMPLEBUS_PNP_INFO(compat_data);
