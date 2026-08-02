/*-
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * Copyright (c) 2026 Tej <itej89@github.com>
 *
 * ARM PL022 PrimeCell SSP/SPI controller driver.
 * Used on the StarFive JH7110 SoC (VisionFive 2).
 * Based on Linux spi-pl022.c and FreeBSD aw_spi.c.
 */

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/bus.h>
#include <sys/kernel.h>
#include <sys/lock.h>
#include <sys/module.h>
#include <sys/mutex.h>
#include <sys/rman.h>

#include <machine/bus.h>

#include <dev/ofw/ofw_bus.h>
#include <dev/ofw/ofw_bus_subr.h>

#include <dev/spibus/spi.h>
#include <dev/spibus/spibusvar.h>

#include <dev/clk/clk.h>
#include <dev/hwreset/hwreset.h>

#include "spibus_if.h"

/* PL022 SSP registers */
#define	SSPCR0		0x000	/* Control register 0 */
#define	SSPCR1		0x004	/* Control register 1 */
#define	SSPDR		0x008	/* Data register */
#define	SSPSR		0x00C	/* Status register */
#define	SSPCPSR		0x010	/* Clock prescale register */
#define	SSPIMSC		0x014	/* Interrupt mask set/clear */
#define	SSPRIS		0x018	/* Raw interrupt status */
#define	SSPMIS		0x01C	/* Masked interrupt status */
#define	SSPICR		0x020	/* Interrupt clear register */
#define	SSPDMACR	0x024	/* DMA control register */

/* SSPCR0 bits */
#define	CR0_DSS_MASK	0x0f	/* Data size select (3 = 4-bit, 15 = 16-bit) */
#define	CR0_DSS_8BIT	0x07	/* 8-bit data */
#define	CR0_FRF_SPI	0x00	/* Frame format: SPI */
#define	CR0_SPO		(1 << 6)	/* Clock polarity */
#define	CR0_SPH		(1 << 7)	/* Clock phase */
#define	CR0_SCR_SHIFT	8	/* Serial clock rate (divider - 1) */
#define	CR0_SCR_MASK	(0xff << CR0_SCR_SHIFT)

/* SSPCR1 bits */
#define	CR1_LBM		(1 << 0)	/* Loopback mode */
#define	CR1_SSE		(1 << 1)	/* SSP enable */
#define	CR1_MS		(1 << 2)	/* Master/slave (0 = master) */
#define	CR1_SOD		(1 << 3)	/* Slave output disable */

/* SSPSR bits */
#define	SR_TFE		(1 << 0)	/* TX FIFO empty */
#define	SR_TNF		(1 << 1)	/* TX FIFO not full */
#define	SR_RNE		(1 << 2)	/* RX FIFO not empty */
#define	SR_RFF		(1 << 3)	/* RX FIFO full */
#define	SR_BSY		(1 << 4)	/* Busy */

/* Clock prescale: must be even, 2-254 */
#define	CPSR_MIN	2
#define	CPSR_MAX	254

/* FIFO depth */
#define	PL022_FIFO_DEPTH	8

#define	RD4(sc, off)	bus_read_4((sc)->res, (off))
#define	WR4(sc, off, v)	bus_write_4((sc)->res, (off), (v))

struct pl022_spi_softc {
	device_t	dev;
	struct resource	*res;
	int		rid;
	struct mtx	mtx;
	clk_t		clk_ssp;
	clk_t		clk_apb;
	uint64_t	ssp_freq;
};

static struct ofw_compat_data compat_data[] = {
	{ "arm,pl022",		1 },
	{ NULL,			0 }
};

static void
pl022_spi_enable(struct pl022_spi_softc *sc)
{
	uint32_t val;

	val = RD4(sc, SSPCR1);
	val |= CR1_SSE;
	WR4(sc, SSPCR1, val);
}

static void
pl022_spi_disable(struct pl022_spi_softc *sc)
{
	uint32_t val;

	val = RD4(sc, SSPCR1);
	val &= ~CR1_SSE;
	WR4(sc, SSPCR1, val);
}

static void
pl022_spi_flush_fifo(struct pl022_spi_softc *sc)
{

	while (RD4(sc, SSPSR) & SR_RNE)
		(void)RD4(sc, SSPDR);
}

static int
pl022_spi_setup(struct pl022_spi_softc *sc, uint32_t mode, uint32_t clock)
{
	uint32_t cr0, cpsr, scr;
	uint64_t target;

	/* Calculate clock dividers: freq = ssp_freq / (CPSR * (1 + SCR)) */
	target = clock;
	if (target == 0 || sc->ssp_freq == 0)
		return (EINVAL);

	cpsr = CPSR_MIN;
	scr = 0;
	while (cpsr <= CPSR_MAX) {
		scr = (sc->ssp_freq / (cpsr * target)) - 1;
		if (scr <= 255)
			break;
		cpsr += 2;
	}
	if (cpsr > CPSR_MAX)
		return (EINVAL);

	pl022_spi_disable(sc);

	/* Set clock prescale */
	WR4(sc, SSPCPSR, cpsr);

	/* Set CR0: 8-bit, SPI format, clock rate, polarity, phase */
	cr0 = CR0_DSS_8BIT | CR0_FRF_SPI;
	cr0 |= (scr & 0xff) << CR0_SCR_SHIFT;
	if (mode & SPIBUS_MODE_CPOL)
		cr0 |= CR0_SPO;
	if (mode & SPIBUS_MODE_CPHA)
		cr0 |= CR0_SPH;
	WR4(sc, SSPCR0, cr0);

	/* Master mode, no loopback */
	WR4(sc, SSPCR1, 0);

	/* Disable all interrupts and DMA */
	WR4(sc, SSPIMSC, 0);
	WR4(sc, SSPDMACR, 0);

	/* Flush RX FIFO */
	pl022_spi_flush_fifo(sc);

	return (0);
}

static int
pl022_spi_xfer_poll(struct pl022_spi_softc *sc, uint8_t *rxbuf,
    uint8_t *txbuf, uint32_t len)
{
	uint32_t i, timeout;

	for (i = 0; i < len; i++) {
		/* Wait for TX FIFO not full */
		timeout = 10000;
		while (!(RD4(sc, SSPSR) & SR_TNF) && --timeout > 0)
			DELAY(1);
		if (timeout == 0)
			return (EIO);

		/* Write data (or dummy 0 if no TX buffer) */
		WR4(sc, SSPDR, txbuf ? txbuf[i] : 0);

		/* Wait for RX FIFO not empty */
		timeout = 10000;
		while (!(RD4(sc, SSPSR) & SR_RNE) && --timeout > 0)
			DELAY(1);
		if (timeout == 0)
			return (EIO);

		/* Read data */
		if (rxbuf)
			rxbuf[i] = RD4(sc, SSPDR) & 0xff;
		else
			(void)RD4(sc, SSPDR);
	}

	/* Wait for not busy */
	timeout = 10000;
	while ((RD4(sc, SSPSR) & SR_BSY) && --timeout > 0)
		DELAY(1);

	return (0);
}

static int
pl022_spi_transfer(device_t dev, device_t child, struct spi_command *cmd)
{
	struct pl022_spi_softc *sc;
	uint32_t cs, mode, clock;
	int err;

	sc = device_get_softc(dev);

	spibus_get_cs(child, &cs);
	spibus_get_clock(child, &clock);
	spibus_get_mode(child, &mode);

	cs &= ~SPIBUS_CS_HIGH;

	mtx_lock(&sc->mtx);

	err = pl022_spi_setup(sc, mode, clock);
	if (err != 0) {
		mtx_unlock(&sc->mtx);
		return (err);
	}

	pl022_spi_enable(sc);

	/* Transfer command */
	err = 0;
	if (cmd->tx_cmd_sz > 0)
		err = pl022_spi_xfer_poll(sc, cmd->rx_cmd, cmd->tx_cmd,
		    cmd->tx_cmd_sz);

	/* Transfer data */
	if (cmd->tx_data_sz > 0 && err == 0)
		err = pl022_spi_xfer_poll(sc, cmd->rx_data, cmd->tx_data,
		    cmd->tx_data_sz);

	pl022_spi_disable(sc);

	mtx_unlock(&sc->mtx);

	return (err);
}

static int
pl022_spi_probe(device_t dev)
{

	if (!ofw_bus_status_okay(dev))
		return (ENXIO);
	if (ofw_bus_search_compatible(dev, compat_data)->ocd_data == 0)
		return (ENXIO);

	device_set_desc(dev, "ARM PL022 SPI Controller");
	return (BUS_PROBE_DEFAULT);
}

static int
pl022_spi_attach(device_t dev)
{
	struct pl022_spi_softc *sc;
	hwreset_t rst;

	sc = device_get_softc(dev);
	sc->dev = dev;

	sc->rid = 0;
	sc->res = bus_alloc_resource_any(dev, SYS_RES_MEMORY, &sc->rid,
	    RF_ACTIVE);
	if (sc->res == NULL) {
		device_printf(dev, "could not allocate registers\n");
		return (ENXIO);
	}

	/* Enable clocks */
	if (clk_get_by_ofw_name(dev, 0, "sspclk", &sc->clk_ssp) == 0) {
		clk_enable(sc->clk_ssp);
		clk_get_freq(sc->clk_ssp, &sc->ssp_freq);
	}
	if (clk_get_by_ofw_name(dev, 0, "apb_pclk", &sc->clk_apb) == 0)
		clk_enable(sc->clk_apb);

	if (sc->ssp_freq == 0) {
		device_printf(dev, "could not determine SSP clock frequency\n");
		sc->ssp_freq = 100000000;
	}

	/* Deassert reset */
	for (int i = 0; hwreset_get_by_ofw_idx(dev, 0, i, &rst) == 0; i++)
		hwreset_deassert(rst);

	mtx_init(&sc->mtx, device_get_nameunit(dev), NULL, MTX_DEF);

	/* Disable the controller */
	pl022_spi_disable(sc);
	WR4(sc, SSPIMSC, 0);
	pl022_spi_flush_fifo(sc);

	device_add_child(dev, "spibus", DEVICE_UNIT_ANY);
	bus_attach_children(dev);

	device_printf(dev, "PL022 SPI (clock %ju Hz)\n",
	    (uintmax_t)sc->ssp_freq);

	return (0);
}

static int
pl022_spi_detach(device_t dev)
{
	struct pl022_spi_softc *sc;

	sc = device_get_softc(dev);

	bus_generic_detach(dev);

	pl022_spi_disable(sc);
	mtx_destroy(&sc->mtx);

	if (sc->res != NULL)
		bus_release_resource(dev, SYS_RES_MEMORY, sc->rid, sc->res);

	return (0);
}

static phandle_t
pl022_spi_get_node(device_t bus, device_t dev)
{

	return (ofw_bus_get_node(bus));
}

static device_method_t pl022_spi_methods[] = {
	DEVMETHOD(device_probe,		pl022_spi_probe),
	DEVMETHOD(device_attach,	pl022_spi_attach),
	DEVMETHOD(device_detach,	pl022_spi_detach),

	DEVMETHOD(spibus_transfer,	pl022_spi_transfer),

	DEVMETHOD(ofw_bus_get_node,	pl022_spi_get_node),

	DEVMETHOD_END,
};

static driver_t pl022_spi_driver = {
	"pl022_spi",
	pl022_spi_methods,
	sizeof(struct pl022_spi_softc),
};

DRIVER_MODULE(pl022_spi, simplebus, pl022_spi_driver, 0, 0);
DRIVER_MODULE(ofw_spibus, pl022_spi, ofw_spibus_driver, 0, 0);
MODULE_DEPEND(pl022_spi, ofw_spibus, 1, 1, 1);
SIMPLEBUS_PNP_INFO(compat_data);
