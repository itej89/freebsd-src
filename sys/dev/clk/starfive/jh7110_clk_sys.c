/*-
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * Copyright 2016 Michal Meloun <mmel@FreeBSD.org>
 * Copyright (c) 2020 Oskar Holmlund <oskar.holmlund@ohdata.se>
 * Copyright (c) 2022 Mitchell Horne <mhorne@FreeBSD.org>
 * Copyright (c) 2024 Jari Sihvola <jsihv@gmx.com>
 */

/* Clocks for JH7110 SYS group. PLL driver must be attached before this. */

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/bus.h>
#include <sys/kernel.h>
#include <sys/module.h>
#include <sys/mutex.h>
#include <sys/resource.h>
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

static struct ofw_compat_data compat_data[] = {
	{ "starfive,jh7110-syscrg",	1 },
	{ NULL,				0 }
};

static struct resource_spec res_spec[] = {
	{ SYS_RES_MEMORY, 0, RF_ACTIVE | RF_SHAREABLE },
	RESOURCE_SPEC_END
};

/* parents for non-pll SYS clocks */
static const char *cpu_root_p[] = { "osc", "pll0_out" };
static const char *cpu_core_p[] = { "cpu_root" };
static const char *cpu_bus_p[] = { "cpu_core" };
static const char *perh_root_p[] = { "pll0_out", "pll2_out" };
static const char *bus_root_p[] = { "osc", "pll2_out" };

static const char *apb_bus_p[] = { "stg_axiahb" };
static const char *apb0_p[] = { "apb_bus" };
static const char *u0_sys_iomux_apb_p[] = { "apb_bus" };
static const char *stg_axiahb_p[] = { "axi_cfg0" };
static const char *ahb0_p[] = { "stg_axiahb" };
static const char *axi_cfg0_p[] = { "bus_root" };
static const char *nocstg_bus_p[] = { "bus_root" };
static const char *noc_bus_stg_axi_p[] = { "nocstg_bus" };

static const char *u0_dw_uart_clk_apb_p[] = { "apb0" };
static const char *u0_dw_uart_clk_core_p[] = { "osc" };
static const char *u0_dw_sdio_clk_ahb_p[] = { "ahb0" };
static const char *u0_dw_sdio_clk_sdcard_p[] = { "axi_cfg0" };
static const char *u1_dw_uart_clk_apb_p[] = { "apb0" };
static const char *u1_dw_uart_clk_core_p[] = { "osc" };
static const char *u1_dw_sdio_clk_ahb_p[] = { "ahb0" };
static const char *u1_dw_sdio_clk_sdcard_p[] = { "axi_cfg0" };
static const char *usb_125m_p[] = { "pll0_out" };
static const char *u2_dw_uart_clk_apb_p[] = { "apb0" };
static const char *u2_dw_uart_clk_core_p[] = { "osc" };
static const char *u3_dw_uart_clk_apb_p[] = { "apb0" };
static const char *u3_dw_uart_clk_core_p[] = { "perh_root" };

static const char *gmac_src_p[] = { "pll0_out" };
static const char *gmac_phy_p[] = { "gmac_src" };
static const char *gmac0_gtxclk_p[] = { "pll0_out" };
static const char *gmac0_ptp_p[] = { "gmac_src" };
static const char *gmac0_gtxc_p[] = { "gmac0_gtxclk" };
static const char *gmac1_gtxclk_p[] = { "pll0_out" };
static const char *gmac1_gtxc_p[] = { "gmac1_gtxclk" };
static const char *gmac1_rmii_rtx_p[] = { "gmac1_rmii_refin" };
static const char *gmac1_axi_p[] = { "stg_axiahb" };
static const char *gmac1_ahb_p[] = { "ahb0" };
static const char *gmac1_ptp_p[] = { "gmac_src" };
static const char *gmac1_tx_inv_p[] = { "gmac1_tx" };
static const char *gmac1_tx_p[] = { "gmac1_gtxclk", "gmac1_rmii_rtx" };
static const char *gmac1_rx_p[] = { "gmac1_rgmii_rxin", "gmac1_rmii_rtx" };
static const char *gmac1_rx_inv_p[] = { "gmac1_rx" };

/* parents for additional SYS clocks (IDs 102+) */
static const char *ahb1_p[] = { "stg_axiahb" };
static const char *qspi_ahb_p[] = { "ahb1" };
static const char *qspi_apb_p[] = { "apb_bus" };
static const char *qspi_ref_src_p[] = { "pll0_out" };
static const char *qspi_ref_p[] = { "osc", "qspi_ref_src" };
static const char *can_apb_p[] = { "apb_bus" };
static const char *can_timer_p[] = { "osc" };
static const char *can_can_p[] = { "perh_root" };
static const char *pwm_apb_p[] = { "apb_bus" };
static const char *wdt_apb_p[] = { "apb_bus" };
static const char *wdt_core_p[] = { "osc" };
static const char *timer_apb_p[] = { "apb_bus" };
static const char *timer_p[] = { "osc" };
static const char *temp_apb_p[] = { "apb_bus" };
static const char *temp_core_p[] = { "osc" };
static const char *spi_apb0_p[] = { "apb0" };
static const char *spi_apb_p[] = { "apb_bus" };
static const char *i2c_apb0_p[] = { "apb0" };
static const char *i2c_apb_p[] = { "apb_bus" };
static const char *uart45_apb_p[] = { "apb0" };
static const char *uart45_core_p[] = { "perh_root" };
static const char *pwmdac_apb_p[] = { "apb_bus" };
static const char *pwmdac_core_p[] = { "apb_bus" };
static const char *spdif_apb_p[] = { "apb_bus" };
static const char *spdif_core_p[] = { "apb_bus" };
static const char *tdm_ahb_p[] = { "ahb0" };
static const char *tdm_apb_p[] = { "apb_bus" };
static const char *tdm_internal_p[] = { "apb_bus" };
static const char *pdm_apb_p[] = { "apb_bus" };
static const char *jtag_trng_p[] = { "osc" };

/* Foundation/intermediate clocks */
static const char *pll0_div2_p[] = { "pll0_out" };
static const char *pll1_div2_p[] = { "pll1_out" };
static const char *pll2_div2_p[] = { "pll2_out" };
static const char *audio_root_p[] = { "pll2_out" };
static const char *mclk_inner_p[] = { "audio_root" };
static const char *mclk_p[] = { "mclk_inner", "mclk_ext" };
static const char *mclk_out_p[] = { "mclk_inner" };
static const char *osc_div2_p[] = { "osc" };
static const char *pll1_div4_p[] = { "pll1_div2" };
static const char *pll1_div8_p[] = { "pll1_div4" };
static const char *ddr_bus_p[] = { "osc_div2", "pll1_div2", "pll1_div4", "pll1_div8" };
static const char *rtc_toggle_p[] = { "osc" };
static const char *gpu_root_p[] = { "pll0_out", "pll2_out" };
static const char *gpu_core_p[] = { "gpu_root" };
static const char *gpu_core_clk_p[] = { "gpu_core" };
static const char *gpu_sys_clk_p[] = { "isp_axi" };
static const char *gpu_apb_p[] = { "apb_bus" };
static const char *gpu_rtc_p[] = { "osc" };
static const char *noc_gpu_p[] = { "gpu_core" };
static const char *isp_2x_p[] = { "pll0_out" };
static const char *isp_axi_p[] = { "isp_2x" };
static const char *isp_top_core_p[] = { "isp_2x" };
static const char *isp_top_axi_p[] = { "isp_axi" };
static const char *noc_isp_p[] = { "isp_axi" };
static const char *hifi4_core_p[] = { "bus_root" };
static const char *hifi4_axi_p[] = { "hifi4_core" };
static const char *vout_src_p[] = { "pll2_out" };
static const char *vout_axi_p[] = { "pll2_out" };
static const char *noc_disp_p[] = { "vout_axi" };
static const char *vout_ahb_p[] = { "ahb1" };
static const char *vout_top_axi_p[] = { "vout_axi" };
static const char *vout_mclk_p[] = { "mclk" };
static const char *vout_mipi_p[] = { "osc" };
static const char *jpegc_axi_p[] = { "pll2_out" };
static const char *codaj12_axi_p[] = { "jpegc_axi" };
static const char *codaj12_core_p[] = { "pll2_out" };
static const char *codaj12_apb_p[] = { "apb_bus" };
static const char *vdec_axi_p[] = { "bus_root" };
static const char *wave511_axi_p[] = { "vdec_axi" };
static const char *wave511_bpu_p[] = { "bus_root" };
static const char *wave511_vce_p[] = { "pll0_out" };
static const char *wave511_apb_p[] = { "apb_bus" };
static const char *vdec_jpg_p[] = { "jpegc_axi" };
static const char *vdec_main_p[] = { "vdec_axi" };
static const char *noc_vdec_p[] = { "vdec_axi" };
static const char *venc_axi_p[] = { "pll2_out" };
static const char *wave420l_axi_p[] = { "venc_axi" };
static const char *wave420l_bpu_p[] = { "pll2_out" };
static const char *wave420l_vce_p[] = { "pll2_out" };
static const char *wave420l_apb_p[] = { "apb_bus" };
static const char *noc_venc_p[] = { "venc_axi" };

/* CPU/debug/trace */
static const char *core_p[] = { "cpu_core" };
static const char *debug_p[] = { "cpu_bus" };
static const char *trace_com_p[] = { "cpu_bus" };
static const char *ddr_axi_p[] = { "ddr_bus" };
static const char *axi_cfg0_main_p[] = { "axi_cfg0" };
static const char *axi_cfg1_p[] = { "stg_axiahb" };
static const char *aximem2_p[] = { "axi_cfg0" };

/* I2S clocks (non-MDIV ones only) */
static const char *i2stx0_apb_p[] = { "apb0" };
static const char *i2stx0_bclk_mst_p[] = { "mclk" };
static const char *i2stx1_apb_p[] = { "apb0" };
static const char *i2stx1_bclk_mst_p[] = { "mclk" };
static const char *i2srx_apb_p[] = { "apb0" };
static const char *i2srx_bclk_mst_p[] = { "mclk" };
static const char *pdm_dmic_p[] = { "mclk" };
static const char *gclk0_p[] = { "pll0_div2" };
static const char *gclk1_p[] = { "pll1_div2" };
static const char *gclk2_p[] = { "pll2_div2" };

/* I2S/TDM MUX parents (internal + external from DTS fixed-clock nodes) */
static const char *i2stx0_bclk_p[] = { "i2stx0_bclk_mst", "i2stx_bclk_ext" };
static const char *i2stx0_lrck_mst_p[] = { "i2stx0_bclk_mst_inv" };
static const char *i2stx0_lrck_p[] = { "i2stx0_lrck_mst", "i2stx_lrck_ext" };
static const char *i2stx0_bclk_inv_p[] = { "i2stx0_bclk" };
static const char *i2stx1_bclk_p[] = { "i2stx1_bclk_mst", "i2stx_bclk_ext" };
static const char *i2stx1_lrck_mst_p[] = { "i2stx1_bclk_mst_inv" };
static const char *i2stx1_lrck_p[] = { "i2stx1_lrck_mst", "i2stx_lrck_ext" };
static const char *i2stx1_bclk_inv_p[] = { "i2stx1_bclk" };
static const char *i2srx_bclk_p[] = { "i2srx_bclk_mst", "i2srx_bclk_ext" };
static const char *i2srx_lrck_mst_p[] = { "i2srx_bclk_mst_inv" };
static const char *i2srx_lrck_p[] = { "i2srx_lrck_mst", "i2srx_lrck_ext" };
static const char *i2srx_bclk_inv_p[] = { "i2srx_bclk" };
static const char *tdm_tdm_p[] = { "tdm_internal", "tdm_ext" };
static const char *tdm_tdm_inv_p[] = { "tdm_tdm" };

/* non-pll SYS clocks */
static const struct jh7110_clk_def sys_clks[] = {
	JH7110_MUX(JH7110_SYSCLK_CPU_ROOT, "cpu_root", cpu_root_p),
	JH7110_DIV(JH7110_SYSCLK_CPU_CORE, "cpu_core", cpu_core_p, 7),
	JH7110_DIV(JH7110_SYSCLK_CPU_BUS, "cpu_bus", cpu_bus_p, 2),
	JH7110_GATEDIV(JH7110_SYSCLK_PERH_ROOT, "perh_root", perh_root_p, 2),
	JH7110_MUX(JH7110_SYSCLK_BUS_ROOT, "bus_root", bus_root_p),

	JH7110_GATE(JH7110_SYSCLK_APB0, "apb0", apb0_p),
	JH7110_GATE(JH7110_SYSCLK_IOMUX_APB, "u0_sys_iomux_apb",
	    u0_sys_iomux_apb_p),
	JH7110_GATE(JH7110_SYSCLK_UART0_APB, "u0_dw_uart_clk_apb",
	    u0_dw_uart_clk_apb_p),
	JH7110_GATE(JH7110_SYSCLK_UART0_CORE, "u0_dw_uart_clk_core",
	    u0_dw_uart_clk_core_p),
	JH7110_GATE(JH7110_SYSCLK_UART1_APB, "u1_dw_uart_clk_apb",
	    u1_dw_uart_clk_apb_p),
	JH7110_GATE(JH7110_SYSCLK_UART1_CORE, "u1_dw_uart_clk_core",
	    u1_dw_uart_clk_core_p),
	JH7110_GATE(JH7110_SYSCLK_UART2_APB, "u2_dw_uart_clk_apb",
	    u2_dw_uart_clk_apb_p),
	JH7110_GATE(JH7110_SYSCLK_UART2_CORE, "u2_dw_uart_clk_core",
	    u2_dw_uart_clk_core_p),
	JH7110_GATE(JH7110_SYSCLK_UART3_APB, "u3_dw_uart_clk_apb",
	    u3_dw_uart_clk_apb_p),
	JH7110_GATE(JH7110_SYSCLK_UART3_CORE, "u3_dw_uart_clk_core",
	    u3_dw_uart_clk_core_p),

	JH7110_DIV(JH7110_SYSCLK_AXI_CFG0, "axi_cfg0", axi_cfg0_p, 3),
	JH7110_DIV(JH7110_SYSCLK_STG_AXIAHB, "stg_axiahb", stg_axiahb_p, 2),
	JH7110_DIV(JH7110_SYSCLK_NOCSTG_BUS, "nocstg_bus", nocstg_bus_p, 3),
	JH7110_GATE(JH7110_SYSCLK_NOC_BUS_STG_AXI, "noc_bus_stg_axi",
	    noc_bus_stg_axi_p),
	JH7110_GATE(JH7110_SYSCLK_AHB0, "ahb0", ahb0_p),
	JH7110_DIV(JH7110_SYSCLK_APB_BUS, "apb_bus", apb_bus_p, 8),

	JH7110_GATE(JH7110_SYSCLK_SDIO0_AHB, "u0_dw_sdio_clk_ahb",
	    u0_dw_sdio_clk_ahb_p),
	JH7110_GATE(JH7110_SYSCLK_SDIO1_AHB, "u1_dw_sdio_clk_ahb",
	    u1_dw_sdio_clk_ahb_p),
	JH7110_GATEDIV(JH7110_SYSCLK_SDIO0_SDCARD, "u0_dw_sdio_clk_sdcard",
	    u0_dw_sdio_clk_sdcard_p, 15),
	JH7110_GATEDIV(JH7110_SYSCLK_SDIO1_SDCARD, "u1_dw_sdio_clk_sdcard",
	    u1_dw_sdio_clk_sdcard_p, 15),
	JH7110_DIV(JH7110_SYSCLK_USB_125M, "usb_125m", usb_125m_p, 15),

	JH7110_DIV(JH7110_SYSCLK_GMAC_SRC, "gmac_src", gmac_src_p, 7),
	JH7110_GATEDIV(JH7110_SYSCLK_GMAC0_GTXCLK, "gmac0_gtxclk",
	    gmac0_gtxclk_p, 15),
	JH7110_GATEDIV(JH7110_SYSCLK_GMAC0_PTP, "gmac0_ptp", gmac0_ptp_p, 31),
	JH7110_GATEDIV(JH7110_SYSCLK_GMAC_PHY, "gmac_phy", gmac_phy_p, 31),
	JH7110_GATE(JH7110_SYSCLK_GMAC0_GTXC, "gmac0_gtxc", gmac0_gtxc_p),

	JH7110_MUX(JH7110_SYSCLK_GMAC1_RX, "gmac1_rx", gmac1_rx_p),
	JH7110_INV(JH7110_SYSCLK_GMAC1_RX_INV, "gmac1_rx_inv", gmac1_rx_inv_p),
	JH7110_GATE(JH7110_SYSCLK_GMAC1_AHB, "gmac1_ahb", gmac1_ahb_p),
	JH7110_DIV(JH7110_SYSCLK_GMAC1_GTXCLK, "gmac1_gtxclk",
	    gmac1_gtxclk_p, 15),
	JH7110_GATEMUX(JH7110_SYSCLK_GMAC1_TX, "gmac1_tx", gmac1_tx_p),
	JH7110_INV(JH7110_SYSCLK_GMAC1_TX_INV, "gmac1_tx_inv", gmac1_tx_inv_p),
	JH7110_GATEDIV(JH7110_SYSCLK_GMAC1_PTP, "gmac1_ptp", gmac1_ptp_p, 31),
	JH7110_GATE(JH7110_SYSCLK_GMAC1_AXI, "gmac1_axi", gmac1_axi_p),
	JH7110_GATE(JH7110_SYSCLK_GMAC1_GTXC, "gmac1_gtxc", gmac1_gtxc_p),
	JH7110_DIV(JH7110_SYSCLK_GMAC1_RMII_RTX, "gmac1_rmii_rtx",
	    gmac1_rmii_rtx_p, 30),

	/* AHB1 bus */
	JH7110_GATE(JH7110_SYSCLK_AHB1, "ahb1", ahb1_p),

	/* QSPI (102-106) */
	JH7110_GATE(JH7110_SYSCLK_QSPI_AHB, "qspi_ahb", qspi_ahb_p),
	JH7110_GATE(JH7110_SYSCLK_QSPI_APB, "qspi_apb", qspi_apb_p),
	JH7110_DIV(JH7110_SYSCLK_QSPI_REF_SRC, "qspi_ref_src",
	    qspi_ref_src_p, 16),
	JH7110_GATEMUX(JH7110_SYSCLK_QSPI_REF, "qspi_ref", qspi_ref_p),

	/* Misc (113-114) */
	JH7110_GATE(JH7110_SYSCLK_MAILBOX_APB, "mailbox_apb", apb_bus_p),
	JH7110_GATE(JH7110_SYSCLK_INT_CTRL_APB, "int_ctrl_apb", apb_bus_p),

	/* CAN (115-120) */
	JH7110_GATE(JH7110_SYSCLK_CAN0_APB, "can0_apb", can_apb_p),
	JH7110_GATEDIV(JH7110_SYSCLK_CAN0_TIMER, "can0_timer", can_timer_p, 24),
	JH7110_GATEDIV(JH7110_SYSCLK_CAN0_CAN, "can0_can", can_can_p, 63),
	JH7110_GATE(JH7110_SYSCLK_CAN1_APB, "can1_apb", can_apb_p),
	JH7110_GATEDIV(JH7110_SYSCLK_CAN1_TIMER, "can1_timer", can_timer_p, 24),
	JH7110_GATEDIV(JH7110_SYSCLK_CAN1_CAN, "can1_can", can_can_p, 63),

	/* PWM, WDT, Timer (121-128) */
	JH7110_GATE(JH7110_SYSCLK_PWM_APB, "pwm_apb", pwm_apb_p),
	JH7110_GATE(JH7110_SYSCLK_WDT_APB, "wdt_apb", wdt_apb_p),
	JH7110_GATE(JH7110_SYSCLK_WDT_CORE, "wdt_core", wdt_core_p),
	JH7110_GATE(JH7110_SYSCLK_TIMER_APB, "timer_apb", timer_apb_p),
	JH7110_GATE(JH7110_SYSCLK_TIMER0, "timer0", timer_p),
	JH7110_GATE(JH7110_SYSCLK_TIMER1, "timer1", timer_p),
	JH7110_GATE(JH7110_SYSCLK_TIMER2, "timer2", timer_p),
	JH7110_GATE(JH7110_SYSCLK_TIMER3, "timer3", timer_p),

	/* Temperature sensor (129-130) */
	JH7110_GATE(JH7110_SYSCLK_TEMP_APB, "temp_apb", temp_apb_p),
	JH7110_GATEDIV(JH7110_SYSCLK_TEMP_CORE, "temp_core", temp_core_p, 24),

	/* SPI (131-137) */
	JH7110_GATE(JH7110_SYSCLK_SPI0_APB, "spi0_apb", spi_apb0_p),
	JH7110_GATE(JH7110_SYSCLK_SPI1_APB, "spi1_apb", spi_apb0_p),
	JH7110_GATE(JH7110_SYSCLK_SPI2_APB, "spi2_apb", spi_apb0_p),
	JH7110_GATE(JH7110_SYSCLK_SPI3_APB, "spi3_apb", spi_apb_p),
	JH7110_GATE(JH7110_SYSCLK_SPI4_APB, "spi4_apb", spi_apb_p),
	JH7110_GATE(JH7110_SYSCLK_SPI5_APB, "spi5_apb", spi_apb_p),
	JH7110_GATE(JH7110_SYSCLK_SPI6_APB, "spi6_apb", spi_apb_p),

	/* I2C (138-144) */
	JH7110_GATE(JH7110_SYSCLK_I2C0_APB, "i2c0_apb", i2c_apb0_p),
	JH7110_GATE(JH7110_SYSCLK_I2C1_APB, "i2c1_apb", i2c_apb0_p),
	JH7110_GATE(JH7110_SYSCLK_I2C2_APB, "i2c2_apb", i2c_apb0_p),
	JH7110_GATE(JH7110_SYSCLK_I2C3_APB, "i2c3_apb", i2c_apb_p),
	JH7110_GATE(JH7110_SYSCLK_I2C4_APB, "i2c4_apb", i2c_apb_p),
	JH7110_GATE(JH7110_SYSCLK_I2C5_APB, "i2c5_apb", i2c_apb_p),
	JH7110_GATE(JH7110_SYSCLK_I2C6_APB, "i2c6_apb", i2c_apb_p),

	/* UART 4-5 (153-156) — UART 0-3 already registered above */
	JH7110_GATE(JH7110_SYSCLK_UART4_APB, "uart4_apb", uart45_apb_p),
	JH7110_GATEDIV(JH7110_SYSCLK_UART4_CORE, "uart4_core", uart45_core_p, 10),
	JH7110_GATE(JH7110_SYSCLK_UART5_APB, "uart5_apb", uart45_apb_p),
	JH7110_GATEDIV(JH7110_SYSCLK_UART5_CORE, "uart5_core", uart45_core_p, 10),

	/* Audio: PWMDAC, SPDIF (157-160) */
	JH7110_GATE(JH7110_SYSCLK_PWMDAC_APB, "pwmdac_apb", pwmdac_apb_p),
	JH7110_GATE(JH7110_SYSCLK_PWMDAC_CORE, "pwmdac_core", pwmdac_core_p),
	JH7110_GATE(JH7110_SYSCLK_SPDIF_APB, "spdif_apb", spdif_apb_p),
	JH7110_GATE(JH7110_SYSCLK_SPDIF_CORE, "spdif_core", spdif_core_p),

	/* Audio: TDM, PDM (184-188) */
	JH7110_GATE(JH7110_SYSCLK_TDM_AHB, "tdm_ahb", tdm_ahb_p),
	JH7110_GATE(JH7110_SYSCLK_TDM_APB, "tdm_apb", tdm_apb_p),
	JH7110_GATE(JH7110_SYSCLK_TDM_INTERNAL, "tdm_internal", tdm_internal_p),
	JH7110_GATE(JH7110_SYSCLK_PDM_APB, "pdm_apb", pdm_apb_p),

	/* JTAG/TRNG (189) */
	JH7110_GATE(JH7110_SYSCLK_JTAG_CERTIFICATION_TRNG, "jtag_trng",
	    jtag_trng_p),

	/* Foundation: PLL dividers */
	JH7110_DIV(JH7110_SYSCLK_PLL0_DIV2, "pll0_div2", pll0_div2_p, 2),
	JH7110_DIV(JH7110_SYSCLK_PLL1_DIV2, "pll1_div2", pll1_div2_p, 2),
	JH7110_DIV(JH7110_SYSCLK_PLL2_DIV2, "pll2_div2", pll2_div2_p, 2),
	JH7110_DIV(JH7110_SYSCLK_OSC_DIV2, "osc_div2", osc_div2_p, 2),
	JH7110_DIV(JH7110_SYSCLK_PLL1_DIV4, "pll1_div4", pll1_div4_p, 2),
	JH7110_DIV(JH7110_SYSCLK_PLL1_DIV8, "pll1_div8", pll1_div8_p, 2),
	JH7110_DIV(JH7110_SYSCLK_RTC_TOGGLE, "rtc_toggle", rtc_toggle_p, 6),

	/* Audio root chain */
	JH7110_DIV(JH7110_SYSCLK_AUDIO_ROOT, "audio_root", audio_root_p, 8),
	JH7110_DIV(JH7110_SYSCLK_MCLK_INNER, "mclk_inner", mclk_inner_p, 64),
	JH7110_MUX(JH7110_SYSCLK_MCLK, "mclk", mclk_p),
	JH7110_GATE(JH7110_SYSCLK_MCLK_OUT, "mclk_out", mclk_out_p),

	/* GCLKs */
	JH7110_GATEDIV(JH7110_SYSCLK_GCLK0, "gclk0", gclk0_p, 62),
	JH7110_GATEDIV(JH7110_SYSCLK_GCLK1, "gclk1", gclk1_p, 62),
	JH7110_GATEDIV(JH7110_SYSCLK_GCLK2, "gclk2", gclk2_p, 62),

	/* DDR */
	JH7110_MUX(JH7110_SYSCLK_DDR_BUS, "ddr_bus", ddr_bus_p),
	JH7110_GATE(JH7110_SYSCLK_DDR_AXI, "ddr_axi", ddr_axi_p),

	/* CPU cores + debug */
	JH7110_GATE(JH7110_SYSCLK_CORE, "core", core_p),
	JH7110_GATE(JH7110_SYSCLK_CORE1, "core1", core_p),
	JH7110_GATE(JH7110_SYSCLK_CORE2, "core2", core_p),
	JH7110_GATE(JH7110_SYSCLK_CORE3, "core3", core_p),
	JH7110_GATE(JH7110_SYSCLK_CORE4, "core4", core_p),
	JH7110_GATE(JH7110_SYSCLK_DEBUG, "debug", debug_p),
	JH7110_GATE(JH7110_SYSCLK_TRACE0, "trace0", core_p),
	JH7110_GATE(JH7110_SYSCLK_TRACE1, "trace1", core_p),
	JH7110_GATE(JH7110_SYSCLK_TRACE2, "trace2", core_p),
	JH7110_GATE(JH7110_SYSCLK_TRACE3, "trace3", core_p),
	JH7110_GATE(JH7110_SYSCLK_TRACE4, "trace4", core_p),
	JH7110_GATE(JH7110_SYSCLK_TRACE_COM, "trace_com", trace_com_p),

	/* Bus NOC */
	JH7110_GATE(JH7110_SYSCLK_NOC_BUS_CPU_AXI, "noc_bus_cpu_axi",
	    cpu_bus_p),
	JH7110_GATE(JH7110_SYSCLK_NOC_BUS_AXICFG0_AXI, "noc_bus_axicfg0_axi",
	    axi_cfg0_main_p),
	JH7110_GATE(JH7110_SYSCLK_AXI_CFG1_MAIN, "axi_cfg1_main",
	    axi_cfg1_p),
	JH7110_GATE(JH7110_SYSCLK_AXI_CFG1_AHB, "axi_cfg1_ahb",
	    axi_cfg1_p),
	JH7110_GATE(JH7110_SYSCLK_AXI_CFG0_MAIN_DIV, "axi_cfg0_main_div",
	    axi_cfg0_main_p),
	JH7110_GATE(JH7110_SYSCLK_AXI_CFG0_MAIN, "axi_cfg0_main",
	    axi_cfg0_main_p),
	JH7110_GATE(JH7110_SYSCLK_AXI_CFG0_HIFI4, "axi_cfg0_hifi4",
	    axi_cfg0_main_p),
	JH7110_GATE(JH7110_SYSCLK_AXIMEM2_AXI, "aximem2_axi", aximem2_p),

	/* GPU */
	JH7110_MUX(JH7110_SYSCLK_GPU_ROOT, "gpu_root", gpu_root_p),
	JH7110_DIV(JH7110_SYSCLK_GPU_CORE, "gpu_core", gpu_core_p, 7),
	JH7110_GATE(JH7110_SYSCLK_GPU_CORE_CLK, "gpu_core_clk",
	    gpu_core_clk_p),
	JH7110_GATE(JH7110_SYSCLK_GPU_SYS_CLK, "gpu_sys_clk",
	    gpu_sys_clk_p),
	JH7110_GATE(JH7110_SYSCLK_GPU_APB, "gpu_apb", gpu_apb_p),
	JH7110_GATEDIV(JH7110_SYSCLK_GPU_RTC_TOGGLE, "gpu_rtc_toggle",
	    gpu_rtc_p, 12),
	JH7110_GATE(JH7110_SYSCLK_NOC_BUS_GPU_AXI, "noc_bus_gpu_axi",
	    noc_gpu_p),

	/* ISP (use GATEDIV for isp_2x instead of MDIV) */
	JH7110_GATEDIV(JH7110_SYSCLK_ISP_2X, "isp_2x", isp_2x_p, 8),
	JH7110_DIV(JH7110_SYSCLK_ISP_AXI, "isp_axi", isp_axi_p, 4),
	JH7110_GATE(JH7110_SYSCLK_ISP_TOP_CORE, "isp_top_core",
	    isp_top_core_p),
	JH7110_GATE(JH7110_SYSCLK_ISP_TOP_AXI, "isp_top_axi",
	    isp_top_axi_p),
	JH7110_GATE(JH7110_SYSCLK_NOC_BUS_ISP_AXI, "noc_bus_isp_axi",
	    noc_isp_p),

	/* HiFi4 DSP */
	JH7110_DIV(JH7110_SYSCLK_HIFI4_CORE, "hifi4_core", hifi4_core_p, 15),
	JH7110_DIV(JH7110_SYSCLK_HIFI4_AXI, "hifi4_axi", hifi4_axi_p, 2),

	/* VOUT / Display pipeline */
	JH7110_GATE(JH7110_SYSCLK_VOUT_SRC, "vout_src", vout_src_p),
	JH7110_DIV(JH7110_SYSCLK_VOUT_AXI, "vout_axi", vout_axi_p, 7),
	JH7110_GATE(JH7110_SYSCLK_NOC_BUS_DISP_AXI, "noc_bus_disp_axi",
	    noc_disp_p),
	JH7110_GATE(JH7110_SYSCLK_VOUT_TOP_AHB, "vout_top_ahb", vout_ahb_p),
	JH7110_GATE(JH7110_SYSCLK_VOUT_TOP_AXI, "vout_top_axi",
	    vout_top_axi_p),
	JH7110_GATE(JH7110_SYSCLK_VOUT_TOP_HDMITX0_MCLK,
	    "vout_top_hdmitx0_mclk", vout_mclk_p),
	JH7110_DIV(JH7110_SYSCLK_VOUT_TOP_MIPIPHY_REF,
	    "vout_top_mipiphy_ref", vout_mipi_p, 2),

	/* JPEG codec */
	JH7110_DIV(JH7110_SYSCLK_JPEGC_AXI, "jpegc_axi", jpegc_axi_p, 16),
	JH7110_GATE(JH7110_SYSCLK_CODAJ12_AXI, "codaj12_axi",
	    codaj12_axi_p),
	JH7110_GATEDIV(JH7110_SYSCLK_CODAJ12_CORE, "codaj12_core",
	    codaj12_core_p, 16),
	JH7110_GATE(JH7110_SYSCLK_CODAJ12_APB, "codaj12_apb",
	    codaj12_apb_p),

	/* Video decoder */
	JH7110_DIV(JH7110_SYSCLK_VDEC_AXI, "vdec_axi", vdec_axi_p, 7),
	JH7110_GATE(JH7110_SYSCLK_WAVE511_AXI, "wave511_axi",
	    wave511_axi_p),
	JH7110_GATEDIV(JH7110_SYSCLK_WAVE511_BPU, "wave511_bpu",
	    wave511_bpu_p, 7),
	JH7110_GATEDIV(JH7110_SYSCLK_WAVE511_VCE, "wave511_vce",
	    wave511_vce_p, 7),
	JH7110_GATE(JH7110_SYSCLK_WAVE511_APB, "wave511_apb",
	    wave511_apb_p),
	JH7110_GATE(JH7110_SYSCLK_VDEC_JPG, "vdec_jpg", vdec_jpg_p),
	JH7110_GATE(JH7110_SYSCLK_VDEC_MAIN, "vdec_main", vdec_main_p),
	JH7110_GATE(JH7110_SYSCLK_NOC_BUS_VDEC_AXI, "noc_bus_vdec_axi",
	    noc_vdec_p),

	/* Video encoder */
	JH7110_DIV(JH7110_SYSCLK_VENC_AXI, "venc_axi", venc_axi_p, 15),
	JH7110_GATE(JH7110_SYSCLK_WAVE420L_AXI, "wave420l_axi",
	    wave420l_axi_p),
	JH7110_GATEDIV(JH7110_SYSCLK_WAVE420L_BPU, "wave420l_bpu",
	    wave420l_bpu_p, 15),
	JH7110_GATEDIV(JH7110_SYSCLK_WAVE420L_VCE, "wave420l_vce",
	    wave420l_vce_p, 15),
	JH7110_GATE(JH7110_SYSCLK_WAVE420L_APB, "wave420l_apb",
	    wave420l_apb_p),
	JH7110_GATE(JH7110_SYSCLK_NOC_BUS_VENC_AXI, "noc_bus_venc_axi",
	    noc_venc_p),

	/* I2S TX0 (skip MDIV for LRCK) */
	JH7110_GATE(JH7110_SYSCLK_I2STX0_APB, "i2stx0_apb", i2stx0_apb_p),
	JH7110_GATEDIV(JH7110_SYSCLK_I2STX0_BCLK_MST, "i2stx0_bclk_mst",
	    i2stx0_bclk_mst_p, 32),
	JH7110_INV(JH7110_SYSCLK_I2STX0_BCLK_MST_INV,
	    "i2stx0_bclk_mst_inv", i2stx0_bclk_mst_p),

	/* I2S TX1 (skip MDIV for LRCK) */
	JH7110_GATE(JH7110_SYSCLK_I2STX1_APB, "i2stx1_apb", i2stx1_apb_p),
	JH7110_GATEDIV(JH7110_SYSCLK_I2STX1_BCLK_MST, "i2stx1_bclk_mst",
	    i2stx1_bclk_mst_p, 32),
	JH7110_INV(JH7110_SYSCLK_I2STX1_BCLK_MST_INV,
	    "i2stx1_bclk_mst_inv", i2stx1_bclk_mst_p),

	/* I2S RX (skip MDIV for LRCK) */
	JH7110_GATE(JH7110_SYSCLK_I2SRX_APB, "i2srx_apb", i2srx_apb_p),
	JH7110_GATEDIV(JH7110_SYSCLK_I2SRX_BCLK_MST, "i2srx_bclk_mst",
	    i2srx_bclk_mst_p, 32),
	JH7110_INV(JH7110_SYSCLK_I2SRX_BCLK_MST_INV,
	    "i2srx_bclk_mst_inv", i2srx_bclk_mst_p),

	/* PDM */
	JH7110_GATEDIV(JH7110_SYSCLK_PDM_DMIC, "pdm_dmic", pdm_dmic_p, 64),

	/* I2S TX0 remaining: LRCK_MST(MDIV→GATEDIV), BCLK(MUX), BCLK_INV, LRCK(MUX) */
	JH7110_GATEDIV(JH7110_SYSCLK_I2STX0_LRCK_MST, "i2stx0_lrck_mst",
	    i2stx0_lrck_mst_p, 64),
	JH7110_MUX(JH7110_SYSCLK_I2STX0_BCLK, "i2stx0_bclk",
	    i2stx0_bclk_p),
	JH7110_INV(JH7110_SYSCLK_I2STX0_BCLK_INV, "i2stx0_bclk_inv",
	    i2stx0_bclk_inv_p),
	JH7110_MUX(JH7110_SYSCLK_I2STX0_LRCK, "i2stx0_lrck",
	    i2stx0_lrck_p),

	/* I2S TX1 remaining */
	JH7110_GATEDIV(JH7110_SYSCLK_I2STX1_LRCK_MST, "i2stx1_lrck_mst",
	    i2stx1_lrck_mst_p, 64),
	JH7110_MUX(JH7110_SYSCLK_I2STX1_BCLK, "i2stx1_bclk",
	    i2stx1_bclk_p),
	JH7110_INV(JH7110_SYSCLK_I2STX1_BCLK_INV, "i2stx1_bclk_inv",
	    i2stx1_bclk_inv_p),
	JH7110_MUX(JH7110_SYSCLK_I2STX1_LRCK, "i2stx1_lrck",
	    i2stx1_lrck_p),

	/* I2S RX remaining */
	JH7110_GATEDIV(JH7110_SYSCLK_I2SRX_LRCK_MST, "i2srx_lrck_mst",
	    i2srx_lrck_mst_p, 64),
	JH7110_MUX(JH7110_SYSCLK_I2SRX_BCLK, "i2srx_bclk",
	    i2srx_bclk_p),
	JH7110_INV(JH7110_SYSCLK_I2SRX_BCLK_INV, "i2srx_bclk_inv",
	    i2srx_bclk_inv_p),
	JH7110_MUX(JH7110_SYSCLK_I2SRX_LRCK, "i2srx_lrck",
	    i2srx_lrck_p),

	/* TDM remaining */
	JH7110_MUX(JH7110_SYSCLK_TDM_TDM, "tdm_tdm", tdm_tdm_p),
	JH7110_INV(JH7110_SYSCLK_TDM_TDM_INV, "tdm_tdm_inv",
	    tdm_tdm_inv_p),
};

static int
jh7110_clk_sys_probe(device_t dev)
{
	if (!ofw_bus_status_okay(dev))
		return (ENXIO);

	if (ofw_bus_search_compatible(dev, compat_data)->ocd_data == 0)
		return (ENXIO);

	device_set_desc(dev, "StarFive JH7110 SYS clock generator");

	return (BUS_PROBE_DEFAULT);
}

static int
jh7110_clk_sys_attach(device_t dev)
{
	struct jh7110_clkgen_softc *sc;
	int i, error;

	sc = device_get_softc(dev);

	sc->reset_status_offset = SYSCRG_RESET_STATUS;
	sc->reset_selector_offset = SYSCRG_RESET_SELECTOR;

	mtx_init(&sc->mtx, device_get_nameunit(dev), NULL, MTX_DEF);

	/* Allocate memory groups */
	error = bus_alloc_resources(dev, res_spec, &sc->mem_res);
	if (error != 0) {
		device_printf(dev, "Couldn't allocate resources, error %d\n",
		    error);
		return (ENXIO);
	}

	/* Create clock domain */
	sc->clkdom = clkdom_create(dev);
	if (sc->clkdom == NULL) {
		device_printf(dev, "Couldn't create clkdom\n");
		return (ENXIO);
	}

	/* Register clocks */
	for (i = 0; i < nitems(sys_clks); i++) {
		error = jh7110_clk_register(sc->clkdom, &sys_clks[i]);
		if (error != 0) {
			device_printf(dev, "Couldn't register clock %s: %d\n",
			    sys_clks[i].clkdef.name, error);
			return (ENXIO);
		}
	}

	if (clkdom_finit(sc->clkdom) != 0)
		panic("Cannot finalize clkdom initialization\n");

	if (bootverbose)
		clkdom_dump(sc->clkdom);

	hwreset_register_ofw_provider(dev);

	return (0);
}

static int
jh7110_clk_sys_detach(device_t dev)
{
	/* Detach not supported */
	return (EBUSY);
}

static void
jh7110_clk_sys_device_lock(device_t dev)
{
	struct jh7110_clkgen_softc *sc;

	sc = device_get_softc(dev);
	mtx_lock(&sc->mtx);
}

static void
jh7110_clk_sys_device_unlock(device_t dev)
{
	struct jh7110_clkgen_softc *sc;

	sc = device_get_softc(dev);
	mtx_unlock(&sc->mtx);
}

static device_method_t jh7110_clk_sys_methods[] = {
	/* Device interface */
	DEVMETHOD(device_probe,		jh7110_clk_sys_probe),
	DEVMETHOD(device_attach,	jh7110_clk_sys_attach),
	DEVMETHOD(device_detach,	jh7110_clk_sys_detach),

	/* clkdev interface */
	DEVMETHOD(clkdev_device_lock,	jh7110_clk_sys_device_lock),
	DEVMETHOD(clkdev_device_unlock,	jh7110_clk_sys_device_unlock),

	/* Reset interface */
	DEVMETHOD(hwreset_assert,	jh7110_reset_assert),
	DEVMETHOD(hwreset_is_asserted,	jh7110_reset_is_asserted),

	DEVMETHOD_END
};

DEFINE_CLASS_0(jh7110_clk_sys, jh7110_clk_sys_driver, jh7110_clk_sys_methods,
    sizeof(struct jh7110_clkgen_softc));
EARLY_DRIVER_MODULE(jh7110_clk_sys, simplebus, jh7110_clk_sys_driver, 0, 0,
    BUS_PASS_BUS + BUS_PASS_ORDER_LATE);
MODULE_VERSION(jh7110_clk_sys, 1);
