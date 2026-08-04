/*-
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * Copyright (c) 2026 Tej Kiran
 *
 * Power domain consumer implementation for FreeBSD.
 * Follows the same pattern as dev/hwreset/hwreset.c.
 */

#include "opt_platform.h"

#include <sys/param.h>
#include <sys/kernel.h>
#include <sys/kobj.h>
#include <sys/malloc.h>
#include <sys/systm.h>

#ifdef FDT
#include <dev/ofw/ofw_bus.h>
#include <dev/ofw/ofw_bus_subr.h>
#endif

#include <dev/pwrdom/pwrdom.h>

#include "pwrdom_if.h"

struct pwrdom {
	device_t	consumer_dev;
	device_t	provider_dev;
	intptr_t	pd_id;
};

MALLOC_DEFINE(M_PWRDOM, "pwrdom", "Power domain framework");

int
pwrdom_enable(pwrdom_t pd)
{

	return (PWRDOM_ENABLE(pd->provider_dev, pd->pd_id));
}

int
pwrdom_disable(pwrdom_t pd)
{

	return (PWRDOM_DISABLE(pd->provider_dev, pd->pd_id));
}

int
pwrdom_is_enabled(pwrdom_t pd, bool *value)
{

	return (PWRDOM_IS_ENABLED(pd->provider_dev, pd->pd_id, value));
}

void
pwrdom_release(pwrdom_t pd)
{

	free(pd, M_PWRDOM);
}

int
pwrdom_get_by_id(device_t consumer_dev, device_t provider_dev, intptr_t id,
    pwrdom_t *pd_out)
{
	pwrdom_t pd;

	pd = malloc(sizeof(struct pwrdom), M_PWRDOM, M_WAITOK | M_ZERO);
	pd->consumer_dev = consumer_dev;
	pd->provider_dev = provider_dev;
	pd->pd_id = id;
	*pd_out = pd;
	return (0);
}

#ifdef FDT

int
pwrdom_default_ofw_map(device_t provider_dev, phandle_t xref, int ncells,
    pcell_t *cells, intptr_t *id)
{

	if (ncells == 0)
		*id = 0;
	else if (ncells == 1)
		*id = cells[0];
	else
		return (ERANGE);

	return (0);
}

void
pwrdom_register_ofw_provider(device_t provider_dev)
{

	OF_device_register_xref(
	    OF_xref_from_node(ofw_bus_get_node(provider_dev)),
	    provider_dev);
}

void
pwrdom_unregister_ofw_provider(device_t provider_dev)
{

	OF_device_register_xref(
	    OF_xref_from_node(ofw_bus_get_node(provider_dev)),
	    NULL);
}

int
pwrdom_get_by_ofw_idx(device_t consumer_dev, phandle_t cnode, int idx,
    pwrdom_t *pd)
{
	phandle_t xnode;
	pcell_t *cells;
	device_t pddev;
	int ncells, rv;
	intptr_t id;

	if (cnode <= 0)
		cnode = ofw_bus_get_node(consumer_dev);
	if (cnode <= 0) {
		device_printf(consumer_dev,
		    "%s called on not ofw based device\n", __func__);
		return (ENXIO);
	}

	rv = ofw_bus_parse_xref_list_alloc(cnode, "power-domains",
	    "#power-domain-cells", idx, &xnode, &ncells, &cells);
	if (rv != 0)
		return (rv);

	pddev = OF_device_from_xref(xnode);
	if (pddev == NULL) {
		OF_prop_free(cells);
		return (ENODEV);
	}

	rv = PWRDOM_MAP(pddev, xnode, ncells, cells, &id);
	OF_prop_free(cells);
	if (rv != 0)
		return (rv);

	return (pwrdom_get_by_id(consumer_dev, pddev, id, pd));
}

#endif /* FDT */
