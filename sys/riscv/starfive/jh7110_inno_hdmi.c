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
#include <sys/gpio.h>
#include <sys/taskqueue.h>

#include <machine/bus.h>

#include <dev/ofw/ofw_bus.h>
#include <dev/ofw/ofw_bus_subr.h>
#include <dev/clk/clk.h>
#include <dev/hwreset/hwreset.h>
#include <dev/regulator/regulator.h>
#include <dev/fdt/fdt_pinctrl.h>
#include <dev/gpio/gpiobusvar.h>

#include "jh7110_inno_hdmi.h"

#define	HDMI_WR(off, v)	bus_write_4(hdmi_res, (off) * 4, (v))
#define	HDMI_RD(off)	bus_read_4(hdmi_res, (off) * 4)

/*
 * DDC / EDID block.
 *
 * Register indices and the sequence below follow StarFive's own inno_hdmi
 * driver (drivers/gpu/drm/verisilicon/inno_hdmi.c in starfive-tech/linux,
 * JH7110_VisionFive2_devel), which is what Debian runs and is known to read
 * EDID correctly on this board.
 *
 * Notes that cost several failed attempts:
 *  - MASK1 is 0xc0 and STATUS1 is 0xc1 here, the opposite order from
 *    Rockchip's inno_hdmi. Do not cross-reference that driver.
 *  - Rockchip has an inno_hdmi_reset() that programs HDMI_SYS_CTRL before
 *    I2C. StarFive's driver has no such function and never touches
 *    SYS_CTRL for EDID -- only clocks and reset. Writing SYS_CTRL from the
 *    probe-time get_modes path breaks the subsequent modeset.
 *  - The DDC divider is computed from tmds_rate, which StarFive's bind sets
 *    to 51200000 before any mode is known. That yields 128, not the 185 a
 *    74.25 MHz pixel clock would suggest.
 */
#define	DDC_BUS_FREQ_L			0x4b
#define	DDC_BUS_FREQ_H			0x4c
#define	HDMI_EDID_SEGMENT_POINTER	0x4d
#define	HDMI_EDID_WORD_ADDR		0x4e
#define	HDMI_EDID_FIFO_OFFSET		0x4f
#define	HDMI_EDID_FIFO_ADDR		0x50
#define	HDMI_INTERRUPT_MASK1		0xc0
#define	HDMI_INTERRUPT_STATUS1		0xc1
#define	M_INT_EDID_READY		(1u << 2)
#define	HDMI_SYS_CTRL			0x00
#define	 V_NOT_RST_ANALOG		(1 << 6)
#define	 V_NOT_RST_DIGITAL		(1 << 5)
#define	 V_REG_CLK_SOURCE_SYS		(1 << 2)
#define	 V_PWR_ON			(0 << 1)
#define	 V_INT_POL_HIGH			(1 << 0)

#define	HDMI_STATUS			0xc8
#define	M_HOTPLUG			(1u << 7)

#define	HDMI_SCL_RATE			100000		/* 100 kHz */
#define	HDMI_DDC_REF_RATE		51200000	/* per StarFive bind */
#define	EDID_BLOCK_LEN			128
#define	EDID_POLL_ITERS			1000		/* x 100us = 100ms */

static struct resource *hdmi_res;
static struct resource *dss_res;
static device_t hdmi_dev;
static clk_t hdmi_clks[8];
static int hdmi_nclks;
static clk_t dc_pix_clk;
static clk_t dc_hdmitx_pixclk;
static hwreset_t hdmi_rst;
static bool hdmi_probed = false;
static gpio_pin_t hdmi_hpd_pin = NULL;
static struct resource *hdmi_hpd_irq = NULL;
static void *hdmi_hpd_cookie = NULL;
static struct task hdmi_hpd_task;

/* Time the line must be quiet before we believe it. */
#define	HDMI_HPD_SETTLE_TICKS	(hz / 5)	/* 200 ms */
static void (*hdmi_hpd_cb)(void *) = NULL;
static void *hdmi_hpd_cb_arg = NULL;

bool
jh7110_hdmi_is_available(void)
{
	return (hdmi_probed);
}

/*
 * Read raw EDID over DDC.
 *
 * Mirrors inno_hdmi_i2c_xfer()'s single write-then-read transaction: program
 * the segment/word address, wait for the core to fill its FIFO, then read the
 * whole block out of HDMI_EDID_FIFO_ADDR. Returns 0 on success.
 */
/*
 * True once a valid EDID has been read. Used by the connector's detect()
 * as a backstop: a sink that answered DDC is certainly present, even if it
 * drives HPD weakly.
 */
static bool hdmi_edid_seen = false;
/*
 * Hot-plug notification.
 *
 * The DRM bridge registers a callback here and we invoke it from a task
 * whenever the HPD line changes, so a display swap is acted on immediately
 * instead of waiting for DRM's 10 s output poll - which is long enough to
 * miss a swap completed inside one interval, leaving the CRTC programmed for
 * the mode of the display that was removed.
 *
 * The callback runs from a taskqueue rather than the interrupt handler
 * because it ends in drm_kms_helper_hotplug_event(), which re-probes the
 * connector and reads EDID over DDC - both of which sleep.
 */
void
jh7110_hdmi_set_hotplug_cb(void (*cb)(void *), void *arg)
{
	hdmi_hpd_cb = cb;
	hdmi_hpd_cb_arg = arg;
}

static void
jh7110_hdmi_hpd_work(void *ctx, int pending)
{
	void (*cb)(void *);

	cb = hdmi_hpd_cb;
	if (cb != NULL)
		cb(hdmi_hpd_cb_arg);
}

/*
 * HPD edge.
 *
 * No debouncing, deliberately - StarFive's driver has none either. The line
 * is not clean while a display is being driven: it carries activity at the
 * video refresh rate, roughly 30 edges a second at 4K30. Suppressing that
 * is the wrong layer to fix it at. The consumer calls
 * drm_helper_hpd_irq_event(), which re-probes and emits an event only when
 * the connector status actually changed, so spurious edges cost a re-probe
 * and nothing more.
 *
 * FreeBSD's ithread model already gives what IRQF_ONESHOT gives Linux: the
 * source is masked by PIC_PRE_ITHREAD until the handler returns, so edges
 * cannot pile up.
 */
static void
jh7110_hdmi_hpd_intr(void *arg)
{

	taskqueue_enqueue(taskqueue_thread, &hdmi_hpd_task);
}


/*
 * Tri-state hot-plug read: 1 connected, 0 disconnected, -1 unknown.
 *
 * Deliberately touches only the GPIO and never an HDMI register. The
 * connector's detect() can run at any time, including before anything has
 * clocked the HDMI core, and reading that block while it is unclocked stalls
 * the bus and wedges the SoC with no panic and no console output.
 */
/*
 * Authoritative "is a sink attached" test: ask DDC.
 *
 * Hot-plug detect cannot be trusted on its own here. An MPI7002 panel drives
 * the HPD pin high as expected, but an LG 4K on the same board and cable
 * leaves it low the entire time it is attached and displaying - measured, not
 * assumed. Reporting disconnected on that evidence would blank a working
 * display, so a low HPD is treated as "ask DDC" rather than "absent".
 *
 * A sink that answers DDC with a valid EDID header is present no matter what
 * HPD says. The read is bounded by EDID_POLL_ITERS and returns ETIMEDOUT when
 * nothing is attached, so this still reports a genuine unplug.
 */
bool
jh7110_hdmi_sink_present(void)
{
	static const uint8_t hdr[8] = {
	    0x00, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x00
	};
	uint8_t blk[EDID_BLOCK_LEN];

	if (!hdmi_probed)
		return (false);
	if (jh7110_hdmi_read_edid(blk, sizeof(blk)) != 0)
		return (false);

	return (memcmp(blk, hdr, sizeof(hdr)) == 0);
}

int
jh7110_hdmi_hpd_state(void)
{
	bool active;

	if (!hdmi_probed || hdmi_hpd_pin == NULL)
		return (-1);

	if (gpio_pin_is_active(hdmi_hpd_pin, &active) != 0)
		return (-1);

	return (active ? 1 : 0);
}


bool
jh7110_hdmi_is_connected(void)
{
	int ci;

	if (!hdmi_probed || hdmi_res == NULL)
		return (false);

	/*
	 * The connector's detect() runs before get_modes(), and unlike the
	 * EDID path nothing has necessarily clocked the HDMI core by then.
	 * Reading its registers while it is unclocked stalls the bus and
	 * wedges the SoC with no panic and no console output, so make sure
	 * the clocks are running and the block is out of reset first.
	 */
	for (ci = 0; ci < hdmi_nclks; ci++)
		clk_enable(hdmi_clks[ci]);
	if (hdmi_rst != NULL)
		hwreset_deassert(hdmi_rst);
	DELAY(1000);

	/*
	 * The controller's own hot-plug bit is the state source, as it is in
	 * StarFive's driver: the GPIO supplies the interrupt, this register
	 * supplies the answer. It is a debounced view maintained by the HDMI
	 * block, where the raw pin can be sampled mid-bounce.
	 *
	 * This only works because the pad keeps its pin-group routing into
	 * the controller (din 8). Claiming the pin as a plain GPIO rewrites
	 * that and silently disconnects this bit.
	 */
	if ((HDMI_RD(HDMI_STATUS) & M_HOTPLUG) != 0)
		return (true);

	return (hdmi_edid_seen);
}

/*
 * Read raw EDID over DDC, one transaction per 128-byte block.
 *
 * Mirrors StarFive's inno_hdmi_i2c_xfer(): program the segment pointer and
 * word address, wait for the core to fill its FIFO, then read the block out.
 * A sink advertising extension blocks must have them read too, because
 * drm_edid_is_valid() checksums every block the base one claims exists.
 */
int
jh7110_hdmi_read_edid(uint8_t *buf, size_t len)
{
	uint32_t freq;
	size_t block, i;
	int ci, timeout;

	if (!hdmi_probed || hdmi_res == NULL || buf == NULL)
		return (ENXIO);
	if (len == 0)
		return (EINVAL);

	/* Same as StarFive's pm_runtime resume: clocks and reset only. */
	for (ci = 0; ci < hdmi_nclks; ci++)
		clk_enable(hdmi_clks[ci]);
	if (hdmi_rst != NULL)
		hwreset_deassert(hdmi_rst);
	DELAY(10000);

	/* inno_hdmi_i2c_init(): divider, then clear and mask the flag. */
	freq = (HDMI_DDC_REF_RATE >> 2) / HDMI_SCL_RATE;
	HDMI_WR(DDC_BUS_FREQ_L, freq & 0xff);
	HDMI_WR(DDC_BUS_FREQ_H, (freq >> 8) & 0xff);
	HDMI_WR(HDMI_INTERRUPT_MASK1, 0);
	HDMI_WR(HDMI_INTERRUPT_STATUS1, M_INT_EDID_READY);

	HDMI_WR(HDMI_INTERRUPT_MASK1, M_INT_EDID_READY);

	for (block = 0; block < len; block += EDID_BLOCK_LEN) {
		HDMI_WR(HDMI_INTERRUPT_STATUS1, M_INT_EDID_READY);

		/* Write phase: address this block. */
		HDMI_WR(HDMI_EDID_FIFO_OFFSET, 0);
		HDMI_WR(HDMI_EDID_WORD_ADDR, block & 0xff);
		HDMI_WR(HDMI_EDID_SEGMENT_POINTER, block >> 8);

		/* Read phase: wait for the FIFO to be filled. */
		for (timeout = 0; timeout < EDID_POLL_ITERS; timeout++) {
			if (HDMI_RD(HDMI_INTERRUPT_STATUS1) & M_INT_EDID_READY)
				break;
			DELAY(100);
		}
		if (timeout == EDID_POLL_ITERS) {
			device_printf(hdmi_dev,
			    "EDID read timed out at block %zu "
			    "(status1=0x%02x hpd=0x%02x)\n",
			    block / EDID_BLOCK_LEN,
			    HDMI_RD(HDMI_INTERRUPT_STATUS1) & 0xff,
			    HDMI_RD(HDMI_STATUS) & 0xff);
			HDMI_WR(HDMI_INTERRUPT_MASK1, 0);
			return (ETIMEDOUT);
		}

		HDMI_WR(HDMI_INTERRUPT_STATUS1, M_INT_EDID_READY);

		for (i = 0; i < EDID_BLOCK_LEN && block + i < len; i++)
			buf[block + i] = HDMI_RD(HDMI_EDID_FIFO_ADDR) & 0xff;
	}

	HDMI_WR(HDMI_INTERRUPT_MASK1, 0);
	hdmi_edid_seen = true;

	return (0);
}


/*
 * PHY PLL configuration, ported from StarFive's inno_hdmi driver
 * (drivers/gpu/drm/verisilicon/inno_hdmi.c, JH7110_VisionFive2_devel).
 *
 * The PHY generates the pixel clock, so the PLL has to be programmed for the
 * mode being set. Previously this driver hardcoded the 74.25 MHz (720p60)
 * entry, which meant any other mode produced no signal at the sink.
 */
struct jh7110_pre_pll_config {
	uint32_t	pixclock;
	uint32_t	tmdsclock;
	uint8_t		prediv;
	uint16_t	fbdiv;
	uint8_t		tmds_div_a;
	uint8_t		tmds_div_b;
	uint8_t		tmds_div_c;
	uint8_t		pclk_div_a;
	uint8_t		pclk_div_b;
	uint8_t		pclk_div_c;
	uint8_t		pclk_div_d;
	uint8_t		vco_div_5_en;
	uint32_t	fracdiv;
};

struct jh7110_post_pll_config {
	uint32_t	tmdsclock;
	uint8_t		prediv;
	uint16_t	fbdiv;
	uint8_t		postdiv;
	uint8_t		post_div_en;
	uint8_t		version;
};

static const struct jh7110_pre_pll_config jh7110_pre_pll_cfg[] = {
	{  25175000,  25175000, 1, 100, 2, 3, 3, 12, 3, 3, 4, 0, 0xF55555 },
	{  25200000,  25200000, 1, 100, 2, 3, 3, 12, 3, 3, 4, 0, 0        },
	{  27000000,  27000000, 1,  90, 3, 2, 2, 10, 3, 3, 4, 0, 0        },
	{  27027000,  27027000, 1,  90, 3, 2, 2, 10, 3, 3, 4, 0, 0x170A3D },
	{  28320000,  28320000, 1,  28, 2, 1, 1,  3, 0, 3, 4, 0, 0x51EB85 },
	{  30240000,  30240000, 1,  30, 2, 1, 1,  3, 0, 3, 4, 0, 0x3D70A3 },
	{  31500000,  31500000, 1,  31, 2, 1, 1,  3, 0, 3, 4, 0, 0x7FFFFF },
	{  33750000,  33750000, 1,  33, 2, 1, 1,  3, 0, 3, 4, 0, 0xCFFFFF },
	{  36000000,  36000000, 1,  36, 2, 1, 1,  3, 0, 3, 4, 0, 0        },
	{  40000000,  40000000, 1,  80, 2, 2, 2, 12, 2, 2, 2, 0, 0        },
	{  46970000,  46970000, 1,  46, 2, 1, 1,  3, 0, 3, 4, 0, 0xF851EB },
	{  49000000,  49000000, 1,  49, 2, 1, 1,  3, 0, 3, 4, 0, 0        },
	{  49500000,  49500000, 1,  49, 2, 1, 1,  3, 0, 3, 4, 0, 0x7FFFFF },
	{  50000000,  50000000, 1,  50, 2, 1, 1,  3, 0, 3, 4, 0, 0        },
	{  54000000,  54000000, 1,  54, 2, 1, 1,  3, 0, 3, 4, 0, 0        },
	{  54054000,  54054000, 1,  54, 2, 1, 1,  3, 0, 3, 4, 0, 0x0DD2F1 },
	{  57284000,  57284000, 1,  57, 2, 1, 1,  3, 0, 3, 4, 0, 0x48B439 },
	{  58230000,  58230000, 1,  58, 2, 1, 1,  3, 0, 3, 4, 0, 0x3AE147 },
	{  59341000,  59341000, 1,  59, 2, 1, 1,  3, 0, 3, 4, 0, 0x574BC6 },
	{  59400000,  59400000, 1,  99, 3, 1, 1,  1, 3, 3, 4, 0, 0        },
	{  65000000,  65000000, 1, 130, 2, 2, 2, 12, 0, 2, 2, 0, 0        },
	{  68250000,  68250000, 1,  68, 2, 1, 1,  3, 0, 3, 4, 0, 0x3FFFFF },
	{  71000000,  71000000, 1,  71, 2, 1, 1,  3, 0, 3, 4, 0, 0        },
	{  74176000,  74176000, 1,  98, 1, 2, 2,  1, 2, 3, 4, 0, 0xE6AE6B },
	{  74250000,  74250000, 1,  99, 1, 2, 2,  1, 2, 3, 4, 0, 0        },
	{  75000000,  75000000, 1,  75, 2, 1, 1,  3, 0, 3, 4, 0, 0        },
	{  78750000,  78750000, 1,  78, 2, 1, 1,  3, 0, 3, 4, 0, 0xCFFFFF },
	{  79500000,  79500000, 1,  79, 2, 1, 1,  3, 0, 3, 4, 0, 0x7FFFFF },
	{  83500000,  83500000, 2, 167, 2, 1, 1,  1, 0, 0, 6, 0, 0        },
	{  83500000, 104375000, 1, 104, 2, 1, 1,  1, 1, 0, 5, 0, 0x600000 },
	{  85500000,  85500000, 1,  85, 2, 1, 1,  3, 0, 3, 4, 0, 0x7FFFFF },
	{  85750000,  85750000, 1,  85, 2, 1, 1,  3, 0, 3, 4, 0, 0xCFFFFF },
	{  85800000,  85800000, 1,  85, 2, 1, 1,  3, 0, 3, 4, 0, 0xCCCCCC },
	{  88750000,  88750000, 1,  88, 2, 1, 1,  3, 0, 3, 4, 0, 0xCFFFFF },
	{  89000000,  89000000, 1,  89, 2, 1, 1,  3, 0, 3, 4, 0, 0        },
	{  89910000,  89910000, 1,  89, 2, 1, 1,  3, 0, 3, 4, 0, 0xE8F5C1 },
	{  90000000,  90000000, 1,  90, 2, 1, 1,  3, 0, 3, 4, 0, 0        },
	{  99000000,  99000000, 1,  99, 2, 1, 1,  3, 0, 3, 4, 0, 0        },
	{ 101000000, 101000000, 1, 101, 2, 1, 1,  3, 0, 3, 4, 0, 0        },
	{ 102250000, 102250000, 1, 102, 2, 1, 1,  3, 0, 3, 4, 0, 0x3FFFFF },
	{ 106500000, 106500000, 1, 106, 2, 1, 1,  3, 0, 3, 4, 0, 0x7FFFFF },
	{ 108000000, 108000000, 1,  90, 3, 0, 0,  5, 0, 2, 2, 0, 0        },
	{ 118800000, 118800000, 1, 118, 2, 1, 1,  3, 0, 3, 4, 0, 0xCCCCCC },
	{ 119000000, 119000000, 1, 119, 2, 1, 1,  3, 0, 3, 4, 0, 0        },
	{ 131481000, 131481000, 1, 131, 2, 1, 1,  3, 0, 3, 4, 0, 0x7B22D1 },
	{ 135000000, 135000000, 1, 135, 2, 1, 1,  3, 0, 3, 4, 0, 0        },
	{ 136750000, 136750000, 1, 136, 2, 1, 1,  3, 0, 3, 4, 0, 0xCFFFFF },
	{ 147180000, 147180000, 1, 147, 2, 1, 1,  3, 0, 3, 4, 0, 0x2E147A },
	{ 148352000, 148352000, 1,  98, 1, 1, 1,  1, 2, 2, 2, 0, 0xE6AE6B },
	{ 148500000, 148500000, 1,  99, 1, 1, 1,  1, 2, 2, 2, 0, 0        },
	{ 154000000, 154000000, 1, 154, 2, 1, 1,  3, 0, 3, 4, 0, 0        },
	{ 156000000, 156000000, 1, 156, 2, 1, 1,  3, 0, 3, 4, 0, 0        },
	{ 157000000, 157000000, 1, 157, 2, 1, 1,  3, 0, 3, 4, 0, 0        },
	{ 162000000, 162000000, 1, 162, 2, 1, 1,  3, 0, 3, 4, 0, 0        },
	{ 174250000, 174250000, 1, 145, 3, 0, 0,  5, 0, 2, 2, 0, 0x355555 },
	{ 174500000, 174500000, 1, 174, 2, 1, 1,  3, 0, 3, 4, 0, 0x7FFFFF },
	{ 174570000, 174570000, 1, 174, 2, 1, 1,  3, 0, 3, 4, 0, 0x91EB84 },
	{ 175500000, 175500000, 1, 175, 2, 1, 1,  3, 0, 3, 4, 0, 0x7FFFFF },
	{ 185590000, 185590000, 1, 185, 2, 1, 1,  3, 0, 3, 4, 0, 0x970A3C },
	{ 185625000, 185625000, 1, 185, 2, 1, 1,  3, 0, 3, 4, 0, 0xA00000 },
	{ 187000000, 187000000, 1, 187, 2, 1, 1,  3, 0, 3, 4, 0, 0        },
	{ 198000000, 198000000, 1, 198, 2, 1, 1,  3, 0, 3, 4, 0, 0        },
	{ 241500000, 241500000, 1, 161, 1, 1, 1,  4, 0, 2, 2, 0, 0        },
	{ 241700000, 241700000, 1, 241, 2, 1, 1,  3, 0, 3, 4, 0, 0xB33332 },
	{ 262750000, 262750000, 1, 262, 2, 1, 1,  3, 0, 3, 4, 0, 0xCFFFFF },
	{ 296500000, 296500000, 1, 296, 2, 1, 1,  3, 0, 3, 4, 0, 0x7FFFFF },
	{ 296703000, 296703000, 1,  98, 0, 1, 1,  1, 0, 2, 2, 0, 0xE6AE6B },
	{ 297000000, 297000000, 1,  99, 0, 1, 1,  1, 0, 2, 2, 0, 0        },
	{ 594000000, 594000000, 1,  99, 0, 2, 0,  1, 0, 1, 1, 0, 0        },
	{ 0 }
};

static const struct jh7110_post_pll_config jh7110_post_pll_cfg[] = {
	{  25200000, 1, 80, 13, 3, 1 },
	{  27000000, 1, 40, 11, 3, 1 },
	{  27027000, 1, 40, 11, 3, 1 },
	{  33750000, 1, 40, 11, 3, 1 },
	{  49000000, 1, 20,  1, 3, 3 },
	{  65000000, 1, 20,  1, 3, 3 },
	{  74250000, 1, 20,  1, 3, 3 },
	{  88750000, 1, 20,  1, 3, 3 },
	{ 108000000, 1, 20,  1, 3, 3 },
	{ 148500000, 1, 20,  1, 3, 3 },
	{ 162000000, 1, 20,  1, 3, 3 },
	{ 174250000, 1, 20,  1, 3, 3 },
	{ 187000000, 1, 20,  1, 3, 3 },
	{ 241700000, 1, 20,  1, 3, 3 },
	{ 297000000, 4, 20,  0, 0, 3 },
	{ 594000000, 4, 20,  0, 0, 0 },
	{ 0 }
};

/*
 * True if the PLL tables can express this pixel clock. The connector uses
 * this to filter its mode list, so a mode is never selected that the PHY
 * cannot generate.
 */
bool
jh7110_hdmi_pixclock_supported(uint32_t pixclock)
{
	const struct jh7110_pre_pll_config *pre;
	const struct jh7110_post_pll_config *post;

	for (pre = jh7110_pre_pll_cfg; pre->pixclock != 0; pre++)
		if (pre->pixclock == pixclock && pre->tmdsclock == pixclock)
			break;
	if (pre->pixclock == 0)
		return (false);

	for (post = jh7110_post_pll_cfg; post->tmdsclock != 0; post++)
		if (pixclock <= post->tmdsclock)
			return (true);

	return (false);
}

/*
 * Program the pre- and post-PLL for a pixel clock. Mirrors
 * inno_hdmi_config_pll(); 8bpc means tmdsclock == pixclock.
 */
static int
jh7110_hdmi_config_pll(uint32_t pixclock)
{
	const struct jh7110_pre_pll_config *pre;
	const struct jh7110_post_pll_config *post;
	uint8_t reg_1ad, reg_1aa;

	for (pre = jh7110_pre_pll_cfg; pre->pixclock != 0; pre++)
		if (pre->pixclock == pixclock && pre->tmdsclock == pixclock)
			break;
	if (pre->pixclock == 0) {
		device_printf(hdmi_dev,
		    "no pre-PLL entry for %u Hz\n", pixclock);
		return (EINVAL);
	}

	for (post = jh7110_post_pll_cfg; post->tmdsclock != 0; post++)
		if (pixclock <= post->tmdsclock)
			break;
	if (post->tmdsclock == 0) {
		device_printf(hdmi_dev,
		    "no post-PLL entry for %u Hz\n", pixclock);
		return (EINVAL);
	}

	reg_1ad = post->post_div_en ? post->postdiv : 0x00;
	reg_1aa = post->post_div_en ? 0x0e : 0x02;


	HDMI_WR(0x1a0, 0x01);
	HDMI_WR(0x1aa, 0x0f);
	HDMI_WR(0x1a1, pre->prediv);
	HDMI_WR(0x1a2, 0xf0 | (pre->fbdiv >> 8));
	HDMI_WR(0x1a3, pre->fbdiv & 0xff);
	HDMI_WR(0x1a4, (pre->tmds_div_a << 4) | (pre->tmds_div_b << 2) |
	    pre->tmds_div_c);
	HDMI_WR(0x1a5, (pre->pclk_div_b << 5) | pre->pclk_div_a);
	HDMI_WR(0x1a6, (pre->pclk_div_c << 5) | pre->pclk_div_d);
	HDMI_WR(0x1ab, post->prediv);
	HDMI_WR(0x1ac, post->fbdiv & 0xff);
	HDMI_WR(0x1ad, reg_1ad);
	HDMI_WR(0x1aa, reg_1aa);

	if (pre->fracdiv != 0) {
		HDMI_WR(0x1a2, 0xc0 | (pre->fbdiv >> 8));
		HDMI_WR(0x1d3, pre->fracdiv & 0xff);
		HDMI_WR(0x1d2, (pre->fracdiv >> 8) & 0xff);
		HDMI_WR(0x1d1, (pre->fracdiv >> 16) & 0xff);
	}

	HDMI_WR(0x1a0, 0x00);

	return (0);
}

void
jh7110_hdmi_enable(const struct jh7110_hdmi_mode *mode)
{
	int timeout;
	uint32_t val;
	uint8_t timing_ctl;
	int ci;

	if (!hdmi_probed || hdmi_res == NULL || mode == NULL)
		return;

	/* Re-enable clocks and deassert reset before register access */
	for (ci = 0; ci < hdmi_nclks; ci++)
		clk_enable(hdmi_clks[ci]);

	if (dc_pix_clk != NULL) {
		if (dc_hdmitx_pixclk != NULL)
			clk_set_parent_by_clk(dc_pix_clk, dc_hdmitx_pixclk);
		clk_enable(dc_pix_clk);
	}

	if (hdmi_rst != NULL)
		hwreset_deassert(hdmi_rst);
	DELAY(10000);

	/* Bandgap + PHY config */
	HDMI_WR(0x1b0, HDMI_RD(0x1b0) | 0x04);
	HDMI_WR(0x1cc, 0x0f);
	HDMI_WR(HDMI_SYS_CTRL, 0x63);

	if (jh7110_hdmi_config_pll(mode->pixclock) != 0)
		return;

	/* Wait for pre- and post-PLL lock */
	timeout = 500000;
	while (!(HDMI_RD(0x1a9) & 0x1) && --timeout > 0)
		DELAY(1);
	if (timeout == 0)
		device_printf(hdmi_dev, "pre-PLL did not lock\n");

	timeout = 500000;
	while (!(HDMI_RD(0x1af) & 0x1) && --timeout > 0)
		DELAY(1);
	if (timeout == 0)
		device_printf(hdmi_dev, "post-PLL did not lock\n");

	/* LDO + serializer */
	HDMI_WR(0x1b4, 0x07);
	HDMI_WR(0x1be, 0x71);
	HDMI_WR(0x1bf, 0x00);
	HDMI_WR(0x1c0, 0x00);


	/* PHY power down before timing config */
	HDMI_WR(HDMI_SYS_CTRL, 0x63);

	/* Video timing, per inno_hdmi_config_video_timing() */
	HDMI_WR(0x09, mode->htotal & 0xff);
	HDMI_WR(0x0a, (mode->htotal >> 8) & 0xff);

	val = mode->htotal - mode->hdisplay;
	HDMI_WR(0x0b, val & 0xff);
	HDMI_WR(0x0c, (val >> 8) & 0xff);

	val = mode->htotal - mode->hsync_start;
	HDMI_WR(0x0d, val & 0xff);
	HDMI_WR(0x0e, (val >> 8) & 0xff);

	val = mode->hsync_end - mode->hsync_start;
	HDMI_WR(0x0f, val & 0xff);
	HDMI_WR(0x10, (val >> 8) & 0xff);

	HDMI_WR(0x11, mode->vtotal & 0xff);
	HDMI_WR(0x12, (mode->vtotal >> 8) & 0xff);
	HDMI_WR(0x13, (mode->vtotal - mode->vdisplay) & 0xff);
	HDMI_WR(0x14, (mode->vtotal - mode->vsync_start) & 0xff);
	HDMI_WR(0x15, (mode->vsync_end - mode->vsync_start) & 0xff);


	/* v_EXTERANL_VIDEO(1) | polarity | interlace */
	timing_ctl = (1 << 0);
	if (mode->hsync_positive)
		timing_ctl |= (1 << 2);
	if (mode->vsync_positive)
		timing_ctl |= (1 << 3);
	if (mode->interlace)
		timing_ctl |= (1 << 1);
	HDMI_WR(0x08, timing_ctl);


	/* PHY power on */
	HDMI_WR(HDMI_SYS_CTRL, 0x61);
	HDMI_WR(0x1b2, 0x8f);
	HDMI_WR(0xce, 0x00);
	HDMI_WR(0xce, 0x01);


	/* dssctrl mux: route DC8200 pipe 0 to HDMI */
	if (dss_res != NULL) {
		val = bus_read_4(dss_res, 0x04);
		val |= (1 << 20);
		bus_write_4(dss_res, 0x04, val);

		val = bus_read_4(dss_res, 0x08);
		val |= (1 << 3);
		bus_write_4(dss_res, 0x08, val);
	} else {
		device_printf(hdmi_dev, "dssctrl not mapped, no output\n");
	}

	device_printf(hdmi_dev, "%ux%u @ %u Hz pixel clock\n",
	    mode->hdisplay, mode->vdisplay, mode->pixclock);
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

	/* Get DC8200 pixel clock and HDMI TX pixel clock for mux setup */
	{
		phandle_t dc_node;

		dc_node = OF_finddevice("/soc/dc8200@29400000");
		if (dc_node > 0) {
			if (clk_get_by_ofw_name(dev, dc_node, "pix0",
			    &dc_pix_clk) != 0)
				dc_pix_clk = NULL;
		}

		/* Get hdmitx0_pixelclk from HDMI node (clock-names "pclk") */
		if (clk_get_by_ofw_name(dev, 0, "pclk",
		    &dc_hdmitx_pixclk) != 0)
			dc_hdmitx_pixclk = NULL;

	}

	/*
	 * Apply the DDC/CEC/HPD pin muxing. Without it the controller's I2C
	 * master is wired to nothing: SCL and SDA never leave their default
	 * function, so EDID reads return an empty FIFO and the hot-plug bit
	 * never asserts, even though every register in the block programs
	 * correctly. The group lives in jh7110-common.dtsi as &hdmi_pins.
	 *
	 * fdt_pinctrl_configure_tree() in the pinctrl driver may already have
	 * covered this, but attach ordering is not guaranteed, so ask for it
	 * explicitly; a second application is harmless.
	 */
	/*
	 * Claim the HDMI I/O rails, as StarFive's bind does. They are on
	 * because U-Boot left them on, but nothing held a reference, so
	 * nothing stopped them being shut down as unused.
	 */
	{
		static const char *rails[2] = { "hdmi_1p8", "hdmi_0p9" };
		regulator_t r;
		int k;

		for (k = 0; k < 2; k++) {
			r = NULL;
			if (regulator_get_by_name(dev, rails[k], &r) == 0 &&
			    r != NULL)
				regulator_enable(r);
		}
	}

	{
		int perr;

		perr = fdt_pinctrl_configure_by_name(dev, "default");
		if (perr != 0)
			device_printf(dev,
			    "pinctrl 'default' not applied (%d); DDC/EDID "
			    "will not work\n", perr);
	}

	/*
	 * Grab the hot-plug detect GPIO. This has to happen after the pin
	 * group is applied, because claiming the pin re-muxes it to plain
	 * GPIO input - which is what we want, since the controller's own
	 * hot-plug bit never asserts on this board.
	 *
	 * Failing to get it is not fatal: jh7110_hdmi_hpd_state() reports
	 * "unknown" and the connector falls back to always reporting
	 * connected, which is the behaviour before this pin was wired up.
	 */
	if (gpio_pin_get_by_ofw_property(dev, ofw_bus_get_node(dev),
	    "hpd-gpios", &hdmi_hpd_pin) != 0) {
		hdmi_hpd_pin = NULL;
		device_printf(dev, "no hpd-gpios; hot-plug detect disabled\n");
	} else {
		int irqrid = 0;
		int piclevel;

		/*
		 * Deliberately not re-muxed to a plain GPIO input.
		 *
		 * The pin group routes this pad into the HDMI controller
		 * (din 8), which is what feeds the controller HDMI_STATUS
		 * hot-plug bit. Claiming the pin as GPIO rewrites that
		 * routing and silently disconnects it - the reason the bit
		 * always read clear here. Linux leaves the mux alone and
		 * takes the pad only as an interrupt source, which is what
		 * we do now: the pinctrl configuration stands, and the pin
		 * is used for its interrupt.
		 */

		/*
		 * Requesting the interrupt is what first drives the GPIO
		 * driver's PIC methods - mapping, configuring and unmasking
		 * the pin. Keep that behind the same staging tunable so
		 * "a PIC exists" can be tested separately from "something
		 * uses it".
		 */
		piclevel = 5;
		TUNABLE_INT_FETCH("hw.jh7110_gpio.pic", &piclevel);
		if (piclevel < 5) {
			goto no_hpd_irq;
		}

		/*
		 * Take an interrupt on both edges: a plug and an unplug are
		 * equally interesting, and which edge each produces depends
		 * on the sink.
		 */
		TASK_INIT(&hdmi_hpd_task, 0, jh7110_hdmi_hpd_work, NULL);

		hdmi_hpd_irq = gpio_alloc_intr_resource(dev, &irqrid,
		    RF_ACTIVE, hdmi_hpd_pin, GPIO_INTR_EDGE_BOTH);
		if (hdmi_hpd_irq == NULL) {
			device_printf(dev,
			    "no HPD interrupt; falling back to polling\n");
		} else if (bus_setup_intr(dev, hdmi_hpd_irq,
		    INTR_TYPE_MISC | INTR_MPSAFE, NULL, jh7110_hdmi_hpd_intr,
		    NULL, &hdmi_hpd_cookie) != 0) {
			device_printf(dev,
			    "cannot set up HPD interrupt; polling\n");
			bus_release_resource(dev, SYS_RES_IRQ, irqrid,
			    hdmi_hpd_irq);
			hdmi_hpd_irq = NULL;
		} else {
			device_printf(dev, "HPD interrupt enabled\n");
		}
no_hpd_irq:
		;
	}

	DELAY(50000);

	hdmi_probed = true;
	device_printf(dev, "HDMI TX ready\n");

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
