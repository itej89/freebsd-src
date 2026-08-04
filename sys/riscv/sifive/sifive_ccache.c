/*-
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * Copyright (c) 2024 Ruslan Bukin <br@bsdpad.com>
 * Copyright (c) 2026 Tej Kiran
 *
 * SiFive / StarFive L2 cache controller driver.
 * Provides cache flush via FLUSH64 register and uncached-offset
 * remapping for DMA coherency on non-coherent RISC-V SoCs.
 *
 * On JH7110: does NOT install as cache hooks (T-Head L1 ops already
 * installed). Instead exports sifive_ccache_flush_range() for use
 * by busdma and LinuxKPI.
 */

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/bus.h>
#include <sys/rman.h>
#include <sys/kernel.h>
#include <sys/module.h>
#include <machine/bus.h>

#include <dev/ofw/ofw_bus.h>
#include <dev/ofw/ofw_bus_subr.h>
#include <dev/ofw/openfirm.h>

#include <vm/vm.h>
#include <vm/vm_extern.h>
#include <vm/pmap.h>

#define	SIFIVE_CCACHE_CONFIG	0x000
#define	 CCACHE_CONFIG_WAYS_S	8
#define	 CCACHE_CONFIG_WAYS_M	(0xff << CCACHE_CONFIG_WAYS_S)
#define	SIFIVE_CCACHE_WAYENABLE	0x008
#define	SIFIVE_CCACHE_FLUSH64	0x200

#define	SIFIVE_CCACHE_LINE_SIZE	64

#define	RD8(sc, off)		(bus_read_8((sc)->res, (off)))
#define	WR8(sc, off, val)	(bus_write_8((sc)->res, (off), (val)))
#define	CC_WR8(offset, value)	\
    *(volatile uint64_t *)((uintptr_t)ccache_va + (offset)) = (value)

static struct ofw_compat_data compat_data[] = {
	{ "sifive,ccache0",		1 },
	{ "starfive,jh7110-ccache",	1 },
	{ "sifive,fu740-c000-ccache",	1 },
	{ "sifive,eic7700",		1 },
	{ NULL,				0 }
};

struct ccache_softc {
	struct resource	*res;
};

static void *ccache_va = NULL;
static bool ccache_probed = false;
static uint64_t ccache_uncached_offset = 0;

static struct resource_spec ccache_spec[] = {
	{ SYS_RES_MEMORY,	0,	RF_ACTIVE },
	{ -1, 0 }
};

void
sifive_ccache_flush_range(vm_paddr_t paddr, size_t len)
{
	uint64_t line;

	if (ccache_va == NULL || len == 0)
		return;

	mb();

	for (line = rounddown2(paddr, SIFIVE_CCACHE_LINE_SIZE);
	    line < paddr + len;
	    line += SIFIVE_CCACHE_LINE_SIZE)
		CC_WR8(SIFIVE_CCACHE_FLUSH64, line);

	mb();
}

bool
sifive_ccache_is_available(void)
{

	return (ccache_probed);
}

uint64_t
sifive_ccache_uncached_offset(void)
{

	return (ccache_uncached_offset);
}

static int
ccache_probe(device_t dev)
{

	if (!ofw_bus_status_okay(dev))
		return (ENXIO);

	if (ofw_bus_search_compatible(dev, compat_data)->ocd_data == 0)
		return (ENXIO);

	if (device_get_unit(dev) != 0)
		return (ENXIO);

	device_set_desc(dev, "SiFive Cache Controller");

	return (BUS_PROBE_DEFAULT);
}

static int
ccache_attach(device_t dev)
{
	struct ccache_softc *sc;
	phandle_t node;
	size_t config, ways;
	pcell_t cells[2];

	sc = device_get_softc(dev);

	if (bus_alloc_resources(dev, ccache_spec, &sc->res) != 0) {
		device_printf(dev, "cannot allocate resources for device\n");
		return (ENXIO);
	}

	config = RD8(sc, SIFIVE_CCACHE_CONFIG);
	ways = (config & CCACHE_CONFIG_WAYS_M) >> CCACHE_CONFIG_WAYS_S;
	WR8(sc, SIFIVE_CCACHE_WAYENABLE, (ways - 1));

	ccache_va = rman_get_virtual(sc->res);

	node = ofw_bus_get_node(dev);
	if (OF_getencprop(node, "uncached-offset", (pcell_t *)cells,
	    sizeof(cells)) == sizeof(cells)) {
		ccache_uncached_offset =
		    ((uint64_t)cells[0] << 32) | cells[1];
		device_printf(dev, "uncached-offset: 0x%lx\n",
		    (unsigned long)ccache_uncached_offset);
	}

	ccache_probed = true;

	device_printf(dev, "L2 cache: %zu ways, line %d bytes\n",
	    ways, SIFIVE_CCACHE_LINE_SIZE);

	return (0);
}

static device_method_t ccache_methods[] = {
	DEVMETHOD(device_probe,		ccache_probe),
	DEVMETHOD(device_attach,	ccache_attach),
	DEVMETHOD_END
};

static driver_t ccache_driver = {
	"ccache",
	ccache_methods,
	sizeof(struct ccache_softc),
};

EARLY_DRIVER_MODULE(ccache, simplebus, ccache_driver, 0, 0,
    BUS_PASS_BUS + BUS_PASS_ORDER_FIRST);
MODULE_VERSION(ccache, 1);
