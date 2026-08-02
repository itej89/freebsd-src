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

#include <sys/malloc.h>

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
#define	DC_DISPLAY_DP_CONFIG	0x1CD0
#define	DC_DISPLAY_DPI_CONFIG	0x14B8
#define	DC_DISPLAY_DITHER_CONFIG 0x1410
#define	DC_DISPLAY_PANEL_CONFIG_EX 0x2518

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
	struct resource		*dss_res;	/* dssctrl syscon */
	int			dss_rid;
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
	clk_t clk, pix_clk, hdmi_pix_clk;
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

	/* Set pixel clock mux to use HDMI TX pixel clock */
	if (clk_get_by_ofw_name(dev, 0, "pix_clk", &pix_clk) == 0 &&
	    clk_get_by_ofw_name(dev, 0, "hdmitx0_pixelclk", &hdmi_pix_clk) == 0) {
		if (clk_set_parent_by_clk(pix_clk, hdmi_pix_clk) == 0)
			device_printf(dev, "pixel clock mux set to hdmitx0_pixelclk\n");
		else
			device_printf(dev, "failed to set pixel clock parent\n");
	}

	DELAY(100000);

	return (0);
}

/* ================================================================
 * Stage C: Innosilicon HDMI TX
 * ================================================================ */

/* HDMI registers are at offset * 4 (each logical register is 32-bit spaced) */
#define	HDMI_WR(sc, off, v)	bus_write_4((sc)->hdmi_res, (off) * 4, (v))
#define	HDMI_RD(sc, off)	bus_read_4((sc)->hdmi_res, (off) * 4)

/* Pre-PLL config for 74.25 MHz (720p@60Hz) from Linux table */
static const struct {
	uint8_t prediv, fbdiv_hi, fbdiv_lo;
	uint8_t tmds_div_a, tmds_div_b, tmds_div_c;
	uint8_t pclk_div_a, pclk_div_b, pclk_div_c, pclk_div_d;
	uint32_t fracdiv;
} hdmi_pre_pll_74250 = {
	.prediv = 1, .fbdiv_hi = 0, .fbdiv_lo = 99,
	.tmds_div_a = 1, .tmds_div_b = 2, .tmds_div_c = 2,
	.pclk_div_a = 1, .pclk_div_b = 2, .pclk_div_c = 3, .pclk_div_d = 4,
	.fracdiv = 0,
};

/* Post-PLL config for 74.25 MHz */
static const struct {
	uint8_t prediv, fbdiv, postdiv, post_div_en;
} hdmi_post_pll_74250 = {
	.prediv = 1, .fbdiv = 20, .postdiv = 1, .post_div_en = 3,
};

static int
jh7110_hdmi_init(struct jh7110_display_softc *sc)
{
	phandle_t node;
	hwreset_t rst;
	clk_t clk;
	int i, timeout;

	/* Map HDMI registers at fixed address */
	sc->hdmi_rid = 2;
	sc->hdmi_res = bus_alloc_resource(sc->dev, SYS_RES_MEMORY,
	    &sc->hdmi_rid, 0x29590000, 0x29593FFF, 0x4000, RF_ACTIVE);
	if (sc->hdmi_res == NULL) {
		device_printf(sc->dev, "could not map HDMI registers\n");
		return (ENXIO);
	}

	device_printf(sc->dev, "HDMI registers mapped\n");

	/* Enable HDMI clocks and deassert reset from the HDMI DTS node */
	node = OF_finddevice("/soc/hdmi@29590000");
	if (node > 0) {
		for (i = 0; clk_get_by_ofw_index(sc->dev, node, i, &clk) == 0; i++)
			clk_enable(clk);
		device_printf(sc->dev, "HDMI: enabled %d clocks\n", i);

		if (hwreset_get_by_ofw_idx(sc->dev, node, 0, &rst) == 0)
			hwreset_deassert(rst);
	}

	DELAY(50000);

	/* Pre-init: set bit 2 of 0x1b0, configure 0x1cc */
	HDMI_WR(sc, 0x1b0, HDMI_RD(sc, 0x1b0) | 0x04);
	HDMI_WR(sc, 0x1cc, 0x0f);

	/* PHY power down */
	HDMI_WR(sc, 0x00, 0x63);

	/* Configure PLL for 74.25 MHz (720p) */
	HDMI_WR(sc, 0x1a0, 0x01);
	HDMI_WR(sc, 0x1aa, 0x0f);
	HDMI_WR(sc, 0x1a1, hdmi_pre_pll_74250.prediv);
	HDMI_WR(sc, 0x1a2, 0xf0 | (hdmi_pre_pll_74250.fbdiv_lo >> 8));
	HDMI_WR(sc, 0x1a3, hdmi_pre_pll_74250.fbdiv_lo & 0xff);
	HDMI_WR(sc, 0x1a4,
	    (hdmi_pre_pll_74250.tmds_div_a << 4) |
	    (hdmi_pre_pll_74250.tmds_div_b << 2) |
	    hdmi_pre_pll_74250.tmds_div_c);
	HDMI_WR(sc, 0x1a5,
	    (hdmi_pre_pll_74250.pclk_div_b << 5) |
	    hdmi_pre_pll_74250.pclk_div_a);
	HDMI_WR(sc, 0x1a6,
	    (hdmi_pre_pll_74250.pclk_div_c << 5) |
	    hdmi_pre_pll_74250.pclk_div_d);
	HDMI_WR(sc, 0x1ab, hdmi_post_pll_74250.prediv);
	HDMI_WR(sc, 0x1ac, hdmi_post_pll_74250.fbdiv);
	HDMI_WR(sc, 0x1ad, hdmi_post_pll_74250.postdiv);
	HDMI_WR(sc, 0x1aa, 0x0e);
	HDMI_WR(sc, 0x1a0, 0x00);

	/* Wait for PLL lock (non-fatal on timeout) */
	device_printf(sc->dev, "HDMI: waiting for pre-PLL lock...\n");
	timeout = 500000;
	while (!(HDMI_RD(sc, 0x1a9) & 0x1) && --timeout > 0)
		DELAY(1);
	if (timeout == 0)
		device_printf(sc->dev, "HDMI: pre-PLL lock timeout (continuing)\n");
	else
		device_printf(sc->dev, "HDMI: pre-PLL locked\n");

	device_printf(sc->dev, "HDMI: waiting for post-PLL lock...\n");
	timeout = 500000;
	while (!(HDMI_RD(sc, 0x1af) & 0x1) && --timeout > 0)
		DELAY(1);
	if (timeout == 0)
		device_printf(sc->dev, "HDMI: post-PLL lock timeout (continuing)\n");
	else
		device_printf(sc->dev, "HDMI: post-PLL locked\n");

	/* Turn on LDO */
	HDMI_WR(sc, 0x1b4, 0x07);
	/* Turn on serializer */
	HDMI_WR(sc, 0x1be, 0x71);

	/* Eye diagram improvement for 720p (VIC 4) */
	HDMI_WR(sc, 0x1bf, 0x00);
	HDMI_WR(sc, 0x1c0, 0x00);

	/* PHY power down before timing config */
	HDMI_WR(sc, 0x00, 0x63);

	/* Configure video timing for 720p */
	HDMI_WR(sc, 0x09, MODE_720P_HTOTAL & 0xff);
	HDMI_WR(sc, 0x0a, (MODE_720P_HTOTAL >> 8) & 0xff);
	HDMI_WR(sc, 0x0b, (MODE_720P_HTOTAL - MODE_720P_HACTIVE) & 0xff);
	HDMI_WR(sc, 0x0c, ((MODE_720P_HTOTAL - MODE_720P_HACTIVE) >> 8) & 0xff);
	HDMI_WR(sc, 0x0d, (MODE_720P_HTOTAL - MODE_720P_HSYNC_START) & 0xff);
	HDMI_WR(sc, 0x0e, ((MODE_720P_HTOTAL - MODE_720P_HSYNC_START) >> 8) & 0xff);
	HDMI_WR(sc, 0x0f, (MODE_720P_HSYNC_END - MODE_720P_HSYNC_START) & 0xff);
	HDMI_WR(sc, 0x10, ((MODE_720P_HSYNC_END - MODE_720P_HSYNC_START) >> 8) & 0xff);
	HDMI_WR(sc, 0x11, MODE_720P_VTOTAL & 0xff);
	HDMI_WR(sc, 0x12, (MODE_720P_VTOTAL >> 8) & 0xff);
	HDMI_WR(sc, 0x13, MODE_720P_VTOTAL - MODE_720P_VACTIVE);
	HDMI_WR(sc, 0x14, MODE_720P_VTOTAL - MODE_720P_VSYNC_START);
	HDMI_WR(sc, 0x15, MODE_720P_VSYNC_END - MODE_720P_VSYNC_START);

	/* External video timing, hsync+vsync positive */
	HDMI_WR(sc, 0x08, (1 << 0) | (1 << 2) | (1 << 3));

	/* PHY power on */
	HDMI_WR(sc, 0x00, 0x61);

	/* TMDS driver on */
	HDMI_WR(sc, 0x1b2, 0x8f);

	/* Toggle HDMI output */
	HDMI_WR(sc, 0xce, 0x00);
	HDMI_WR(sc, 0xce, 0x01);

	device_printf(sc->dev, "HDMI TX initialized for 720p@60Hz\n");
	device_printf(sc->dev, "  HDMI reg00=0x%02x reg08=0x%02x regce=0x%02x\n",
	    HDMI_RD(sc, 0x00), HDMI_RD(sc, 0x08), HDMI_RD(sc, 0xce));
	device_printf(sc->dev, "  HDMI reg1b2=0x%02x reg1b4=0x%02x reg1be=0x%02x\n",
	    HDMI_RD(sc, 0x1b2), HDMI_RD(sc, 0x1b4), HDMI_RD(sc, 0x1be));

	return (0);
}

/* ================================================================
 * Stage B: DC8200 display timing + framebuffer
 * ================================================================ */

/*
 * Helper: read-modify-write for DC registers.
 * dc_set_clear(sc, reg, set_bits, clear_bits)
 */
static void
dc_set_clear(struct jh7110_display_softc *sc, uint32_t reg,
    uint32_t set, uint32_t clr)
{
	uint32_t val;

	val = DC_RD4(sc, reg);
	val &= ~clr;
	val |= set;
	DC_WR4(sc, reg, val);
}

static int
jh7110_display_setup_dc(struct jh7110_display_softc *sc)
{
	uint32_t width, height, stride;

	width = MODE_720P_HACTIVE;
	height = MODE_720P_VACTIVE;
	stride = width * 4;
	sc->fb_size = stride * height;

	/* Allocate framebuffer memory (physically contiguous) */
	sc->fb_vaddr = (vm_offset_t)contigmalloc(sc->fb_size, M_DEVBUF,
	    M_NOWAIT | M_ZERO, 0, ~0UL, PAGE_SIZE, 0);
	if (sc->fb_vaddr == 0) {
		device_printf(sc->dev, "failed to allocate framebuffer\n");
		return (ENOMEM);
	}
	sc->fb_paddr = vtophys(sc->fb_vaddr);

	device_printf(sc->dev, "framebuffer %dx%d at phys 0x%lx\n",
	    width, height, (unsigned long)sc->fb_paddr);

	/* dc_hw_init: set panel config to 0x111 (bits 0,4,8) */
	DC_WR4(sc, DC_DISPLAY_PANEL_CONFIG, 0x111);

	/* Disable dither */
	DC_WR4(sc, DC_DISPLAY_DITHER_CONFIG, 0);

	/* Stop panel 0 before configuring */
	dc_set_clear(sc, DC_DISPLAY_PANEL_START, 0, (1 << 0) | (1 << 2));

	/* Set display timing: 720p @ 60Hz */
	DC_WR4(sc, DC_DISPLAY_H,
	    MODE_720P_HACTIVE | (MODE_720P_HTOTAL << 16));
	DC_WR4(sc, DC_DISPLAY_H_SYNC,
	    MODE_720P_HSYNC_START |
	    (MODE_720P_HSYNC_END << 15) |
	    (1 << 30));	/* positive hsync: bit 31 clear, bit 30 set */
	DC_WR4(sc, DC_DISPLAY_V,
	    MODE_720P_VACTIVE | (MODE_720P_VTOTAL << 16));
	DC_WR4(sc, DC_DISPLAY_V_SYNC,
	    MODE_720P_VSYNC_START |
	    (MODE_720P_VSYNC_END << 15) |
	    (1 << 30));	/* positive vsync: bit 31 clear, bit 30 set */

	/* Set background color to blue (visible = working) */
	DC_WR4(sc, DC_FRAMEBUFFER_BG_COLOR, 0x000040FF);

	/* DPI config: RGB888 = 5 */
	DC_WR4(sc, DC_DISPLAY_DPI_CONFIG, 5);

	/* DP config: RGB888(2) + DP select(BIT3) for HDMI output */
	DC_WR4(sc, DC_DISPLAY_DP_CONFIG, 2 | (1 << 3));

	/* Clear YUV mode in panel config (bit 16) */
	dc_set_clear(sc, DC_DISPLAY_PANEL_CONFIG, 0, (1 << 16));

	/* Disable shadow registers so plane writes take effect immediately */
	dc_set_clear(sc, DC_FRAMEBUFFER_CONFIG_EX, 0, (1 << 12));
	dc_set_clear(sc, DC_DISPLAY_PANEL_CONFIG_EX, (1 << 0), 0);

	/* Configure primary plane (plane 0) */
	DC_WR4(sc, DC_FRAMEBUFFER_ADDRESS, (uint32_t)sc->fb_paddr);
	DC_WR4(sc, DC_FRAMEBUFFER_STRIDE, stride);
	DC_WR4(sc, DC_FRAMEBUFFER_SIZE,
	    width | (height << 15));
	DC_WR4(sc, DC_FRAMEBUFFER_TOP_LEFT, 0);
	DC_WR4(sc, DC_FRAMEBUFFER_BOTTOM_RIGHT,
	    width | (height << 15));

	/* Enable primary plane: format=XRGB8888(5), enable, display_id=0 */
	dc_set_clear(sc, DC_FRAMEBUFFER_CONFIG,
	    (FORMAT_X8R8G8B8 << 26),
	    (0x1f << 26));
	dc_set_clear(sc, DC_FRAMEBUFFER_CONFIG_EX,
	    (1 << 13),		/* enable */
	    (1 << 13) | (7 << 16) | (1 << 19));

	/* Re-enable shadow registers */
	dc_set_clear(sc, DC_FRAMEBUFFER_CONFIG_EX, (1 << 12), 0);
	dc_set_clear(sc, DC_DISPLAY_PANEL_CONFIG_EX, 0, (1 << 0));

	/* Panel config: enable output (bit 12) */
	dc_set_clear(sc, DC_DISPLAY_PANEL_CONFIG, (1 << 12), 0);

	/* Start panel 0 */
	dc_set_clear(sc, DC_DISPLAY_PANEL_START, (1 << 0), (1 << 3));

	device_printf(sc->dev, "display timing set: 720p@60Hz\n");

	/* Register readback for debugging */
	device_printf(sc->dev, "  PANEL_CONFIG=0x%08x PANEL_START=0x%08x\n",
	    DC_RD4(sc, DC_DISPLAY_PANEL_CONFIG),
	    DC_RD4(sc, DC_DISPLAY_PANEL_START));
	device_printf(sc->dev, "  PANEL_CONFIG_EX=0x%08x\n",
	    DC_RD4(sc, DC_DISPLAY_PANEL_CONFIG_EX));
	device_printf(sc->dev, "  FB_CONFIG=0x%08x FB_CONFIG_EX=0x%08x\n",
	    DC_RD4(sc, DC_FRAMEBUFFER_CONFIG),
	    DC_RD4(sc, DC_FRAMEBUFFER_CONFIG_EX));
	device_printf(sc->dev, "  FB_ADDR=0x%08x FB_STRIDE=0x%08x\n",
	    DC_RD4(sc, DC_FRAMEBUFFER_ADDRESS),
	    DC_RD4(sc, DC_FRAMEBUFFER_STRIDE));
	device_printf(sc->dev, "  DISP_H=0x%08x DISP_V=0x%08x\n",
	    DC_RD4(sc, DC_DISPLAY_H),
	    DC_RD4(sc, DC_DISPLAY_V));
	device_printf(sc->dev, "  DP_CONFIG=0x%08x DPI_CONFIG=0x%08x\n",
	    DC_RD4(sc, DC_DISPLAY_DP_CONFIG),
	    DC_RD4(sc, DC_DISPLAY_DPI_CONFIG));

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

	/* Map dssctrl syscon (output mux) at fixed address */
	sc->dss_rid = 3;
	sc->dss_res = bus_alloc_resource(dev, SYS_RES_MEMORY,
	    &sc->dss_rid, 0x295b0000, 0x295b008f, 0x90, RF_ACTIVE);
	if (sc->dss_res != NULL) {
		uint32_t val;

		/* Route DC8200 to HDMI: set bit 20 at offset 0x04 */
		val = bus_read_4(sc->dss_res, 0x04);
		val |= (1 << 20);
		bus_write_4(sc->dss_res, 0x04, val);

		/* Enable display path: set bit 3 at offset 0x08 */
		val = bus_read_4(sc->dss_res, 0x08);
		val |= (1 << 3);
		bus_write_4(sc->dss_res, 0x08, val);

		device_printf(dev, "dssctrl mux: reg4=0x%08x reg8=0x%08x\n",
		    bus_read_4(sc->dss_res, 0x04),
		    bus_read_4(sc->dss_res, 0x08));
	} else {
		device_printf(dev, "warning: could not map dssctrl\n");
	}

	/* Read hardware revision from "hi" register space */
	rev = HI_RD4(sc, DC_HW_REVISION);
	cid = HI_RD4(sc, DC_HW_CHIP_CID);
	device_printf(dev, "DC8200 revision 0x%04x, chip ID 0x%03x\n",
	    rev, cid);

	if (rev != 0x5720 && rev != 0x5721) {
		device_printf(dev, "unsupported DC8200 revision\n");
		return (ENXIO);
	}

	/* Stage B: Program display timing and framebuffer */
	if (jh7110_display_setup_dc(sc) != 0) {
		device_printf(dev, "failed to setup display\n");
		return (ENXIO);
	}

	/* Stage C: Initialize HDMI TX */
	if (jh7110_hdmi_init(sc) != 0)
		device_printf(dev, "HDMI init failed (display may not work)\n");

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
