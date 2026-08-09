/*-
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * Copyright (c) 2026 Tej <itej89@github.com>
 *
 * Innosilicon HDMI TX PHY driver for StarFive JH7110.
 * Attaches to "inno,hdmi" DTS node, handles PHY init and dssctrl mux.
 * Exports jh7110_hdmi_enable/disable for DRM driver consumption.
 *
 * PHY init sequence lifted from jh7110_display.c (proven on VisionFive 2).
 */

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/bus.h>
#include <sys/kernel.h>
#include <sys/module.h>
#include <sys/rman.h>

#include <machine/bus.h>

#include <dev/ofw/ofw_bus.h>
#include <dev/ofw/ofw_bus_subr.h>
#include <dev/clk/clk.h>
#include <dev/hwreset/hwreset.h>

#include "jh7110_inno_hdmi.h"

#define	HDMI_WR(off, v)	bus_write_4(hdmi_res, (off) * 4, (v))
#define	HDMI_RD(off)	bus_read_4(hdmi_res, (off) * 4)

static struct resource *hdmi_res;
static struct resource *dss_res;
static device_t hdmi_dev;
static clk_t hdmi_clks[8];
static int hdmi_nclks;
static hwreset_t hdmi_rst;
static bool hdmi_probed = false;

bool
jh7110_hdmi_is_available(void)
{
	return (hdmi_probed);
}

void
jh7110_hdmi_enable(void)
{
	int timeout;
	uint32_t val;

	int ci;

	if (!hdmi_probed || hdmi_res == NULL) {
		printf("jh7110_hdmi_enable: not ready (probed=%d res=%p), skip\n",
		    hdmi_probed, hdmi_res);
		return;
	}

	/* Re-enable clocks and deassert reset before register access */
	printf("jh7110_hdmi_enable: re-enabling %d clocks\n", hdmi_nclks);
	for (ci = 0; ci < hdmi_nclks; ci++) {
		int err = clk_enable(hdmi_clks[ci]);
		uint64_t freq = 0;
		clk_get_freq(hdmi_clks[ci], &freq);
		printf("jh7110_hdmi_enable: clk[%d] enable=%d freq=%lu\n",
		    ci, err, (unsigned long)freq);
	}
	if (hdmi_rst != NULL) {
		printf("jh7110_hdmi_enable: deassert reset\n");
		hwreset_deassert(hdmi_rst);
		printf("jh7110_hdmi_enable: reset deasserted\n");
	}
	printf("jh7110_hdmi_enable: delay 10ms\n");
	DELAY(10000);

	printf("jh7110_hdmi_enable: about to read reg[0x00]\n");
	{
		uint32_t test = bus_read_4(hdmi_res, 0);
		printf("jh7110_hdmi_enable: reg[0x00]=0x%x OK\n", test);
	}

	printf("jh7110_hdmi_enable: about to read reg[0x1b0]\n");
	{
		uint32_t v = HDMI_RD(0x1b0);
		printf("jh7110_hdmi_enable: reg[0x1b0]=0x%x, writing 0x%x\n",
		    v, v | 0x04);
		HDMI_WR(0x1b0, v | 0x04);
		printf("jh7110_hdmi_enable: reg[0x1b0] write done\n");
	}
	HDMI_WR(0x1cc, 0x0f);
	printf("jh7110_hdmi_enable: reg[0x1cc] write done\n");

	HDMI_WR(0x00, 0x63);
	printf("jh7110_hdmi_enable: bandgap+phy_down done\n");

	/* Pre-PLL config for 74.25 MHz (720p@60Hz) */
	HDMI_WR(0x1a0, 0x01);
	HDMI_WR(0x1aa, 0x0f);
	HDMI_WR(0x1a1, 1);
	HDMI_WR(0x1a2, 0xf0);
	HDMI_WR(0x1a3, 99);
	HDMI_WR(0x1a4, (1 << 4) | (2 << 2) | 2);
	HDMI_WR(0x1a5, (2 << 5) | 1);
	HDMI_WR(0x1a6, (3 << 5) | 4);

	/* Post-PLL */
	HDMI_WR(0x1ab, 1);
	HDMI_WR(0x1ac, 20);
	HDMI_WR(0x1ad, 1);
	HDMI_WR(0x1aa, 0x0e);

	/* Enable pre-PLL */
	HDMI_WR(0x1a0, 0x00);
	printf("jh7110_hdmi_enable: PLLs configured, waiting lock\n");

	/* Wait for pre-PLL lock */
	timeout = 500000;
	while (!(HDMI_RD(0x1a9) & 0x1) && --timeout > 0)
		DELAY(1);
	printf("jh7110_hdmi_enable: pre-PLL lock %s (timeout=%d, reg=0x%x)\n",
	    timeout > 0 ? "OK" : "TIMEOUT", timeout, HDMI_RD(0x1a9));

	/* Wait for post-PLL lock */
	timeout = 500000;
	while (!(HDMI_RD(0x1af) & 0x1) && --timeout > 0)
		DELAY(1);
	printf("jh7110_hdmi_enable: post-PLL lock %s (timeout=%d, reg=0x%x)\n",
	    timeout > 0 ? "OK" : "TIMEOUT", timeout, HDMI_RD(0x1af));

	/* LDO + serializer */
	HDMI_WR(0x1b4, 0x07);
	HDMI_WR(0x1be, 0x71);
	HDMI_WR(0x1bf, 0x00);
	HDMI_WR(0x1c0, 0x00);
	printf("jh7110_hdmi_enable: LDO+serializer done\n");

	/* PHY power down before timing config */
	HDMI_WR(0x00, 0x63);

	/* 720p@60Hz video timing */
	HDMI_WR(0x09, 1650 & 0xff);
	HDMI_WR(0x0a, (1650 >> 8) & 0xff);
	HDMI_WR(0x0b, (1650 - 1280) & 0xff);
	HDMI_WR(0x0c, ((1650 - 1280) >> 8) & 0xff);
	HDMI_WR(0x0d, (1650 - 1390) & 0xff);
	HDMI_WR(0x0e, ((1650 - 1390) >> 8) & 0xff);
	HDMI_WR(0x0f, (1430 - 1390) & 0xff);
	HDMI_WR(0x10, ((1430 - 1390) >> 8) & 0xff);
	HDMI_WR(0x11, 750 & 0xff);
	HDMI_WR(0x12, (750 >> 8) & 0xff);
	HDMI_WR(0x13, 750 - 720);
	HDMI_WR(0x14, 750 - 725);
	HDMI_WR(0x15, 730 - 725);
	HDMI_WR(0x08, (1 << 0) | (1 << 2) | (1 << 3));
	printf("jh7110_hdmi_enable: timing done\n");

	/* PHY power on */
	HDMI_WR(0x00, 0x61);
	/* TMDS driver on */
	HDMI_WR(0x1b2, 0x8f);
	/* Toggle output */
	HDMI_WR(0xce, 0x00);
	HDMI_WR(0xce, 0x01);
	printf("jh7110_hdmi_enable: PHY on, TMDS on, output toggled\n");

	/* dssctrl mux: route DC8200 pipe 0 to HDMI */
	if (dss_res != NULL) {
		val = bus_read_4(dss_res, 0x04);
		val |= (1 << 20);
		bus_write_4(dss_res, 0x04, val);

		val = bus_read_4(dss_res, 0x08);
		val |= (1 << 3);
		bus_write_4(dss_res, 0x08, val);
		printf("jh7110_hdmi_enable: dssctrl mux done\n");
	} else {
		printf("jh7110_hdmi_enable: WARNING dss_res is NULL\n");
	}
	printf("jh7110_hdmi_enable: complete\n");
}

void
jh7110_hdmi_disable(void)
{
	if (!hdmi_probed)
		return;

	HDMI_WR(0xce, 0x00);
	HDMI_WR(0x00, 0x63);
}

static struct ofw_compat_data compat_data[] = {
	{ "inno,hdmi",	1 },
	{ NULL,		0 }
};

static int
jh7110_inno_hdmi_probe(device_t dev)
{
	if (!ofw_bus_status_okay(dev))
		return (ENXIO);
	if (ofw_bus_search_compatible(dev, compat_data)->ocd_data == 0)
		return (ENXIO);

	device_set_desc(dev, "Innosilicon HDMI TX (JH7110)");
	return (BUS_PROBE_DEFAULT);
}

static int
jh7110_inno_hdmi_attach(device_t dev)
{
	clk_t clk;
	hwreset_t rst;
	int rid, i;

	/* Map HDMI registers from DTS reg property */
	rid = 0;
	hdmi_res = bus_alloc_resource_any(dev, SYS_RES_MEMORY, &rid,
	    RF_ACTIVE);
	if (hdmi_res == NULL) {
		device_printf(dev, "could not map HDMI registers\n");
		return (ENXIO);
	}

	/* Map dssctrl syscon at fixed address */
	rid = 1;
	dss_res = bus_alloc_resource(dev, SYS_RES_MEMORY, &rid,
	    0x295b0000, 0x295b008f, 0x90, RF_ACTIVE);
	if (dss_res == NULL)
		device_printf(dev, "warning: could not map dssctrl\n");

	hdmi_dev = dev;

	/*
	 * Get clock and reset references but DON'T enable them yet.
	 * DC8200 kmod will enable system clocks first, then
	 * jh7110_hdmi_enable() enables HDMI clocks on demand.
	 */
	for (i = 0; i < 8 && clk_get_by_ofw_index(dev, 0, i, &clk) == 0; i++)
		hdmi_clks[i] = clk;
	hdmi_nclks = i;

	if (hwreset_get_by_ofw_idx(dev, 0, 0, &rst) == 0)
		hdmi_rst = rst;

	DELAY(50000);

	/* Verify register access */
	{
		uint32_t test = bus_read_4(hdmi_res, 0);
		device_printf(dev, "reg[0x00]=0x%x (read test)\n", test);
	}

	hdmi_probed = true;
	device_printf(dev, "HDMI TX ready (res=%p dss=%p)\n",
	    hdmi_res, dss_res);

	return (0);
}

static int
jh7110_inno_hdmi_detach(device_t dev)
{
	return (EBUSY);
}

static device_method_t jh7110_inno_hdmi_methods[] = {
	DEVMETHOD(device_probe,		jh7110_inno_hdmi_probe),
	DEVMETHOD(device_attach,	jh7110_inno_hdmi_attach),
	DEVMETHOD(device_detach,	jh7110_inno_hdmi_detach),
	DEVMETHOD_END
};

DEFINE_CLASS_0(jh7110_inno_hdmi, jh7110_inno_hdmi_driver,
    jh7110_inno_hdmi_methods, 0);
DRIVER_MODULE(jh7110_inno_hdmi, simplebus, jh7110_inno_hdmi_driver, 0, 0);
MODULE_VERSION(jh7110_inno_hdmi, 1);
