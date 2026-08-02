/*-
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * Copyright (c) 2026 Tej <itej89@github.com>
 *
 * VOUT (Video Output) clock domain driver for the StarFive JH7110 SoC.
 * Provides clocks for DC8200 display controller, HDMI TX, DSI TX, MIPI.
 * Based on Linux drivers/clk/starfive/clk-starfive-jh7110-vout.c
 */

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/bus.h>
#include <sys/kernel.h>
#include <sys/module.h>
#include <sys/mutex.h>
#include <sys/rman.h>

#include <machine/bus.h>

#include <dev/fdt/simplebus.h>
#include <dev/ofw/ofw_bus.h>
#include <dev/ofw/ofw_bus_subr.h>

#include <dev/clk/clk.h>
#include <dev/clk/starfive/jh7110_clk.h>
#include <dev/hwreset/hwreset.h>

#include <dt-bindings/clock/starfive,jh7110-crg.h>

#include "clkdev_if.h"
#include "hwreset_if.h"

#define	VOUTCRG_RESET_SELECTOR	0x48
#define	VOUTCRG_RESET_STATUS	0x4C

static struct ofw_compat_data compat_data[] = {
	{ "starfive,jh7110-voutcrg",	1 },
	{ NULL,				0 }
};

static struct resource_spec res_spec[] = {
	{ SYS_RES_MEMORY, 0, RF_ACTIVE | RF_SHAREABLE },
	RESOURCE_SPEC_END
};

/* Parent clocks from SYS domain (provided via DTS clock-names) */
static const char *vout_src_p[] = { "vout_src" };
static const char *vout_top_ahb_p[] = { "vout_top_ahb" };
static const char *vout_top_axi_p[] = { "vout_top_axi" };
static const char *vout_top_hdmitx0_mclk_p[] = { "vout_top_hdmitx0_mclk" };
static const char *i2stx0_bclk_p[] = { "i2stx0_bclk" };
/* Internal parent references */
static const char *vout_apb_p[] = { "vout_apb" };
static const char *dsi_sys_p[] = { "vout_dsi_sys" };
static const char *tx_esc_p[] = { "vout_tx_esc" };

/* MUX parents */
static const char *pix0_mux_p[] = { "vout_dc8200_pix", "hdmitx0_pixelclk" };
static const char *pix1_mux_p[] = { "vout_dc8200_pix", "hdmitx0_pixelclk" };
static const char *lcd_mux_p[] = { "vout_dc8200_pix0", "vout_dc8200_pix1" };
static const char *dpi_mux_p[] = { "vout_dc8200_pix", "hdmitx0_pixelclk" };

static const struct jh7110_clk_def vout_clks[] = {
	/* Dividers from SYS clock parents */
	JH7110_DIV(JH7110_VOUTCLK_APB, "vout_apb", vout_top_ahb_p, 8),
	JH7110_DIV(JH7110_VOUTCLK_DC8200_PIX, "vout_dc8200_pix",
	    vout_src_p, 63),
	JH7110_DIV(JH7110_VOUTCLK_DSI_SYS, "vout_dsi_sys",
	    vout_src_p, 31),
	JH7110_DIV(JH7110_VOUTCLK_TX_ESC, "vout_tx_esc",
	    vout_top_ahb_p, 31),

	/* Gates */
	JH7110_GATE(JH7110_VOUTCLK_DC8200_AXI, "vout_dc8200_axi",
	    vout_top_axi_p),
	JH7110_GATE(JH7110_VOUTCLK_DC8200_CORE, "vout_dc8200_core",
	    vout_top_axi_p),
	JH7110_GATE(JH7110_VOUTCLK_DC8200_AHB, "vout_dc8200_ahb",
	    vout_top_ahb_p),

	/* Pixel clock muxes (choose internal divider or HDMI pixel clock) */
	JH7110_GATEMUX(JH7110_VOUTCLK_DC8200_PIX0, "vout_dc8200_pix0",
	    pix0_mux_p),
	JH7110_GATEMUX(JH7110_VOUTCLK_DC8200_PIX1, "vout_dc8200_pix1",
	    pix1_mux_p),
	JH7110_GATEMUX(JH7110_VOUTCLK_DOM_VOUT_TOP_LCD,
	    "vout_dom_vout_top_lcd", lcd_mux_p),

	/* DSI clocks */
	JH7110_GATE(JH7110_VOUTCLK_DSITX_APB, "vout_dsiTx_apb",
	    dsi_sys_p),
	JH7110_GATE(JH7110_VOUTCLK_DSITX_SYS, "vout_dsiTx_sys",
	    dsi_sys_p),
	JH7110_GATEMUX(JH7110_VOUTCLK_DSITX_DPI, "vout_dsiTx_dpi",
	    dpi_mux_p),
	JH7110_GATE(JH7110_VOUTCLK_DSITX_TXESC, "vout_dsiTx_txesc",
	    tx_esc_p),

	/* MIPI + HDMI */
	JH7110_GATE(JH7110_VOUTCLK_MIPITX_DPHY_TXESC,
	    "vout_mipitx_dphy_txesc", tx_esc_p),
	JH7110_GATE(JH7110_VOUTCLK_HDMI_TX_MCLK, "vout_hdmi_tx_mclk",
	    vout_top_hdmitx0_mclk_p),
	JH7110_GATE(JH7110_VOUTCLK_HDMI_TX_BCLK, "vout_hdmi_tx_bclk",
	    i2stx0_bclk_p),
	JH7110_GATE(JH7110_VOUTCLK_HDMI_TX_SYS, "vout_hdmi_tx_sys",
	    vout_apb_p),
};

static int
jh7110_clk_vout_probe(device_t dev)
{

	if (!ofw_bus_status_okay(dev))
		return (ENXIO);
	if (ofw_bus_search_compatible(dev, compat_data)->ocd_data == 0)
		return (ENXIO);

	device_set_desc(dev, "StarFive JH7110 VOUT clock generator");
	return (BUS_PROBE_DEFAULT);
}

static int
jh7110_clk_vout_attach(device_t dev)
{
	struct jh7110_clkgen_softc *sc;
	int i, error;

	sc = device_get_softc(dev);

	sc->reset_status_offset = VOUTCRG_RESET_STATUS;
	sc->reset_selector_offset = VOUTCRG_RESET_SELECTOR;

	mtx_init(&sc->mtx, device_get_nameunit(dev), NULL, MTX_DEF);

	error = bus_alloc_resources(dev, res_spec, &sc->mem_res);
	if (error != 0) {
		device_printf(dev, "couldn't allocate resources: %d\n", error);
		return (ENXIO);
	}

	/* Deassert top reset before registering clocks */
	{
		hwreset_t rst;

		if (hwreset_get_by_ofw_idx(dev, 0, 0, &rst) == 0)
			hwreset_deassert(rst);
	}

	sc->clkdom = clkdom_create(dev);
	if (sc->clkdom == NULL) {
		device_printf(dev, "couldn't create clkdom\n");
		return (ENXIO);
	}

	for (i = 0; i < nitems(vout_clks); i++) {
		error = jh7110_clk_register(sc->clkdom, &vout_clks[i]);
		if (error != 0) {
			device_printf(dev, "couldn't register clock %s: %d\n",
			    vout_clks[i].clkdef.name, error);
			return (ENXIO);
		}
	}

	if (clkdom_finit(sc->clkdom) != 0)
		panic("cannot finalize VOUT clkdom initialization\n");

	if (bootverbose)
		clkdom_dump(sc->clkdom);

	hwreset_register_ofw_provider(dev);

	return (0);
}

static int
jh7110_clk_vout_detach(device_t dev)
{

	return (EBUSY);
}

static void
jh7110_clk_vout_device_lock(device_t dev)
{
	struct jh7110_clkgen_softc *sc;

	sc = device_get_softc(dev);
	mtx_lock(&sc->mtx);
}

static void
jh7110_clk_vout_device_unlock(device_t dev)
{
	struct jh7110_clkgen_softc *sc;

	sc = device_get_softc(dev);
	mtx_unlock(&sc->mtx);
}

static device_method_t jh7110_clk_vout_methods[] = {
	DEVMETHOD(device_probe,		jh7110_clk_vout_probe),
	DEVMETHOD(device_attach,	jh7110_clk_vout_attach),
	DEVMETHOD(device_detach,	jh7110_clk_vout_detach),

	DEVMETHOD(clkdev_device_lock,	jh7110_clk_vout_device_lock),
	DEVMETHOD(clkdev_device_unlock,	jh7110_clk_vout_device_unlock),

	DEVMETHOD(hwreset_assert,	jh7110_reset_assert),
	DEVMETHOD(hwreset_is_asserted,	jh7110_reset_is_asserted),

	DEVMETHOD_END
};

DEFINE_CLASS_0(jh7110_clk_vout, jh7110_clk_vout_driver,
    jh7110_clk_vout_methods, sizeof(struct jh7110_clkgen_softc));
EARLY_DRIVER_MODULE(jh7110_clk_vout, simplebus, jh7110_clk_vout_driver,
    0, 0, BUS_PASS_BUS + BUS_PASS_ORDER_LATE);
MODULE_VERSION(jh7110_clk_vout, 1);
