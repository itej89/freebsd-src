/*-
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * Copyright (c) 2026 Tej <itej89@github.com>
 *
 * Display driver for StarFive JH7110 SoC: DC8200 + Innosilicon HDMI.
 * Provides a vt(4) framebuffer console over HDMI.
 *
 * Based on Linux drivers/gpu/drm/verisilicon/ (vs_dc_hw.c, inno_hdmi.c)
 */

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/bus.h>
#include <sys/fbio.h>
#include <sys/kernel.h>
#include <sys/module.h>
#include <sys/rman.h>

#include <vm/vm.h>
#include <vm/pmap.h>

#include <machine/bus.h>

#include <dev/ofw/openfirm.h>
#include <dev/ofw/ofw_bus.h>
#include <dev/ofw/ofw_bus_subr.h>

#include <dev/clk/clk.h>
#include <dev/hwreset/hwreset.h>

#include <dev/vt/vt.h>
#include <dev/vt/hw/fb/vt_fb.h>
#include <dev/vt/colors/vt_termcolors.h>

/* DC8200 registers (offset from base 0x29400800) */
#define	DC_HW_REVISION		0x0024
#define	DC_HW_CHIP_CID		0x0030

#define	DC_DISPLAY_H		0x1430
#define	DC_DISPLAY_H_SYNC	0x1438
#define	DC_DISPLAY_V		0x1440
#define	DC_DISPLAY_V_SYNC	0x1448
#define	DC_DISPLAY_PANEL_CONFIG	0x1418
#define	DC_DISPLAY_PANEL_START	0x1CCC

#define	DC_FRAMEBUFFER_CONFIG	0x1518
#define	DC_FRAMEBUFFER_CONFIG_EX 0x1CC0
#define	DC_FRAMEBUFFER_ADDRESS	0x1400
#define	DC_FRAMEBUFFER_STRIDE	0x1408
#define	DC_FRAMEBUFFER_SIZE	0x1810
#define	DC_FRAMEBUFFER_TOP_LEFT		0x24D8
#define	DC_FRAMEBUFFER_BOTTOM_RIGHT	0x24E0
#define	DC_FRAMEBUFFER_BG_COLOR	0x1528

/* DC8200 pixel formats */
#define	FORMAT_X8R8G8B8		5

/* HDMI register base offset from DC8200 */
#define	HDMI_BASE		0x29590000

/* Innosilicon HDMI registers */
#define	HDMI_STATUS		0x00
#define	HDMI_TIMING_H		0x90
#define	HDMI_TIMING_HS		0x94
#define	HDMI_TIMING_HACT	0x98
#define	HDMI_TIMING_V		0xA0
#define	HDMI_TIMING_VS		0xA4
#define	HDMI_TIMING_VACT	0xA8

/* 720p@60Hz timing */
#define	MODE_720P_HACTIVE	1280
#define	MODE_720P_HTOTAL	1650
#define	MODE_720P_HSYNC_START	1390
#define	MODE_720P_HSYNC_END	1430
#define	MODE_720P_VACTIVE	720
#define	MODE_720P_VTOTAL	750
#define	MODE_720P_VSYNC_START	725
#define	MODE_720P_VSYNC_END	730
#define	MODE_720P_PIXCLK	74250000

struct jh7110_display_softc {
	device_t		dev;
	struct resource		*hi_res;	/* DC8200 "hi" regs (rev, IRQ) */
	int			hi_rid;
	struct resource		*dc_res;	/* DC8200 "dc" regs (fb, timing) */
	int			dc_rid;
	struct resource		*hdmi_res;	/* HDMI registers */
	int			hdmi_rid;
	struct fb_info		fb_info;
	vm_paddr_t		fb_paddr;
	vm_offset_t		fb_vaddr;
	uint32_t		fb_size;
};

static struct ofw_compat_data compat_data[] = {
	{ "verisilicon,dc8200",	1 },
	{ NULL,			0 }
};

#define	HI_RD4(sc, off)		bus_read_4((sc)->hi_res, (off))
#define	DC_RD4(sc, off)		bus_read_4((sc)->dc_res, (off))
#define	DC_WR4(sc, off, v)	bus_write_4((sc)->dc_res, (off), (v))

/* ================================================================
 * Stage A: Clock and Reset initialization
 * ================================================================ */

static int
jh7110_display_init_clocks(device_t dev)
{
	clk_t clk;
	hwreset_t rst;
	int i;

	/* Enable all clocks from DTS */
	for (i = 0; clk_get_by_ofw_index(dev, 0, i, &clk) == 0; i++) {
		if (clk_enable(clk) != 0)
			device_printf(dev, "failed to enable clock %d\n", i);
	}
	device_printf(dev, "enabled %d clocks\n", i);

	/* Deassert all resets from DTS */
	for (i = 0; hwreset_get_by_ofw_idx(dev, 0, i, &rst) == 0; i++) {
		if (hwreset_deassert(rst) != 0)
			device_printf(dev, "failed to deassert reset %d\n", i);
	}
	device_printf(dev, "deasserted %d resets\n", i);

	DELAY(100000);

	return (0);
}

/* ================================================================
 * Driver entry points
 * ================================================================ */

static int
jh7110_display_probe(device_t dev)
{

	if (!ofw_bus_status_okay(dev))
		return (ENXIO);
	if (ofw_bus_search_compatible(dev, compat_data)->ocd_data == 0)
		return (ENXIO);

	device_set_desc(dev, "StarFive JH7110 Display (DC8200 + HDMI)");
	return (BUS_PROBE_DEFAULT);
}

static int
jh7110_display_attach(device_t dev)
{
	struct jh7110_display_softc *sc;
	uint32_t rev, cid;

	sc = device_get_softc(dev);
	sc->dev = dev;

	/* Map DC8200 "hi" registers (reg[0] = 0x29400000, revision/IRQ) */
	sc->hi_rid = 0;
	sc->hi_res = bus_alloc_resource_any(dev, SYS_RES_MEMORY,
	    &sc->hi_rid, RF_ACTIVE);
	if (sc->hi_res == NULL) {
		device_printf(dev, "could not allocate HI registers\n");
		return (ENXIO);
	}

	/* Map DC8200 "dc" registers (reg[1] = 0x29400800, fb/timing) */
	sc->dc_rid = 1;
	sc->dc_res = bus_alloc_resource_any(dev, SYS_RES_MEMORY,
	    &sc->dc_rid, RF_ACTIVE);
	if (sc->dc_res == NULL) {
		device_printf(dev, "could not allocate DC registers\n");
		return (ENXIO);
	}

	/* Stage A: Enable clocks and deassert resets */
	jh7110_display_init_clocks(dev);

	/* Read hardware revision from "hi" register space */
	rev = HI_RD4(sc, DC_HW_REVISION);
	cid = HI_RD4(sc, DC_HW_CHIP_CID);
	device_printf(dev, "DC8200 revision 0x%04x, chip ID 0x%03x\n",
	    rev, cid);

	if (rev != 0x5720 && rev != 0x5721) {
		device_printf(dev, "unsupported DC8200 revision\n");
		return (ENXIO);
	}

	/* TODO: Stage B - Display timing */
	/* TODO: Stage C - HDMI TX init */
	/* TODO: Stage D - vt framebuffer registration */

	return (0);
}

static int
jh7110_display_detach(device_t dev)
{
	struct jh7110_display_softc *sc;

	sc = device_get_softc(dev);

	if (sc->hi_res != NULL)
		bus_release_resource(dev, SYS_RES_MEMORY, sc->hi_rid,
		    sc->hi_res);
	if (sc->dc_res != NULL)
		bus_release_resource(dev, SYS_RES_MEMORY, sc->dc_rid,
		    sc->dc_res);

	return (0);
}

static device_method_t jh7110_display_methods[] = {
	DEVMETHOD(device_probe,		jh7110_display_probe),
	DEVMETHOD(device_attach,	jh7110_display_attach),
	DEVMETHOD(device_detach,	jh7110_display_detach),

	DEVMETHOD_END,
};

static driver_t jh7110_display_driver = {
	"jh7110_display",
	jh7110_display_methods,
	sizeof(struct jh7110_display_softc),
};

DRIVER_MODULE(jh7110_display, simplebus, jh7110_display_driver, 0, 0);
