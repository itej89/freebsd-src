/*-
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * Copyright (c) 2026 Tej Kiran
 *
 * LinuxKPI platform_device/platform_driver shim.
 * Bridges Linux platform_driver to FreeBSD newbus + OFW,
 * following the same pattern as linux_pci.c.
 */

#include "opt_platform.h"

#include <sys/param.h>
#include <sys/kernel.h>
#include <sys/module.h>
#include <sys/bus.h>
#include <sys/rman.h>

#include <linux/kernel.h>
#include <linux/device.h>
#include <linux/platform_device.h>
#include <linux/dma-mapping.h>
#include <linux/of.h>
#include <linux/list.h>
#include <linux/slab.h>

#ifdef FDT

#include <dev/ofw/openfirm.h>
#include <dev/ofw/ofw_bus.h>
#include <dev/ofw/ofw_bus_subr.h>
#include <dev/pwrdom/pwrdom.h>

#define	LKPI_IORESOURCE_MEM	(1 << SYS_RES_MEMORY)
#define	LKPI_IORESOURCE_IO	(1 << SYS_RES_IOPORT)
#define	LKPI_IORESOURCE_IRQ	(1 << SYS_RES_IRQ)

static int linux_platform_probe(device_t dev);
static int linux_platform_attach(device_t dev);
static int linux_platform_detach(device_t dev);

static device_method_t platform_methods[] = {
	DEVMETHOD(device_probe,		linux_platform_probe),
	DEVMETHOD(device_attach,	linux_platform_attach),
	DEVMETHOD(device_detach,	linux_platform_detach),
	DEVMETHOD_END
};

static struct platform_driver *
linux_platform_get_driver(device_t dev)
{
	driver_t *drv;

	drv = device_get_driver(dev);
	if (drv == NULL)
		return (NULL);
	return (container_of(drv, struct platform_driver, bsddriver));
}

static const struct of_device_id *
linux_platform_match_of(device_t dev, const struct of_device_id *table)
{
	const struct of_device_id *id;

	if (table == NULL)
		return (NULL);
	for (id = table; id->compatible[0] != '\0'; id++) {
		if (ofw_bus_is_compatible(dev, id->compatible))
			return (id);
	}
	return (NULL);
}

static int
linux_platform_probe(device_t dev)
{
	struct platform_driver *pdrv;
	const struct of_device_id *id;

	pdrv = linux_platform_get_driver(dev);
	if (pdrv == NULL)
		return (ENXIO);

	if (!ofw_bus_status_okay(dev))
		return (ENXIO);

	id = linux_platform_match_of(dev, pdrv->driver.of_match_table);
	if (id == NULL)
		return (ENXIO);

	device_set_desc(dev, pdrv->driver.name);
	return (BUS_PROBE_DEFAULT);
}

static int
linux_platform_attach(device_t dev)
{
	struct platform_driver *pdrv;
	struct platform_device *pdev;
	struct device_node *np;
	int error;

	pdrv = linux_platform_get_driver(dev);
	MPASS(pdrv != NULL);

	pdev = device_get_softc(dev);
	MPASS(pdev != NULL);

	memset(pdev, 0, sizeof(*pdev));
	pdev->dev.bsddev = dev;
	pdev->name = pdrv->driver.name;
	pdev->id = -1;

	spin_lock_init(&pdev->dev.devres_lock);
	INIT_LIST_HEAD(&pdev->dev.devres_head);
	INIT_LIST_HEAD(&pdev->dev.irqents);

	pdev->dev.driver = &pdrv->driver;
	pdev->dev.parent = &linux_root_device;

	pdev->node = ofw_bus_get_node(dev);

	np = kmalloc(sizeof(*np), GFP_KERNEL);
	if (np == NULL)
		return (ENOMEM);
	np->phandle = pdev->node;
	np->full_name = pdrv->driver.name;
	pdev->dev.of_node = np;

	/* Check dma-coherent property (walk up DT tree) */
	{
		phandle_t n = pdev->node;

		pdev->dev.dma_coherent = false;
		while (n > 0) {
			if (OF_hasprop(n, "dma-coherent")) {
				pdev->dev.dma_coherent = true;
				break;
			}
			n = OF_parent(n);
		}
	}

	/* Auto-attach power domain if present in DT */
	{
		pwrdom_t pd;

		if (pwrdom_get_by_ofw_idx(dev, pdev->node, 0, &pd) == 0) {
			pwrdom_enable(pd);
			pdev->pwrdom = pd;
		}
	}

	/* Initialize DMA support */
	{
		struct linux_dma_priv *priv;

		priv = kzalloc(sizeof(*priv), GFP_KERNEL);
		if (priv != NULL) {
			mtx_init(&priv->lock, "lkpi-plat-dma", NULL, MTX_DEF);
			pctrie_init(&priv->ptree);
			pdev->dev.dma_priv = priv;
			linux_dma_tag_init(&pdev->dev, DMA_BIT_MASK(64));
			linux_dma_tag_init_coherent(&pdev->dev,
			    DMA_BIT_MASK(32));
		}
	}

	if (pdrv->probe == NULL) {
		error = ENXIO;
		goto err;
	}

	error = pdrv->probe(pdev);
	if (error != 0) {
		error = -error;
		goto err;
	}

	return (0);

err:
	if (pdev->dev.dma_priv != NULL) {
		struct linux_dma_priv *priv = pdev->dev.dma_priv;
		if (priv->dmat)
			bus_dma_tag_destroy(priv->dmat);
		if (priv->dmat_coherent)
			bus_dma_tag_destroy(priv->dmat_coherent);
		mtx_destroy(&priv->lock);
		kfree(priv);
		pdev->dev.dma_priv = NULL;
	}
	if (pdev->pwrdom != NULL) {
		pwrdom_disable(pdev->pwrdom);
		pwrdom_release(pdev->pwrdom);
		pdev->pwrdom = NULL;
	}
	kfree(pdev->dev.of_node);
	pdev->dev.of_node = NULL;
	return (error);
}

static int
linux_platform_detach(device_t dev)
{
	struct platform_driver *pdrv;
	struct platform_device *pdev;
	struct resource *bsd_res;
	int i;

	pdrv = linux_platform_get_driver(dev);
	pdev = device_get_softc(dev);

	if (pdrv != NULL && pdrv->remove != NULL)
		pdrv->remove(pdev);

	for (i = 0; i < pdev->bsd_nres; i++) {
		bsd_res = pdev->bsd_res[i];
		if (bsd_res != NULL) {
			bus_release_resource(dev, rman_get_type(bsd_res),
			    pdev->bsd_rid[i], bsd_res);
			pdev->bsd_res[i] = NULL;
		}
	}
	pdev->bsd_nres = 0;

	if (pdev->dev.dma_priv != NULL) {
		struct linux_dma_priv *priv = pdev->dev.dma_priv;
		if (priv->dmat)
			bus_dma_tag_destroy(priv->dmat);
		if (priv->dmat_coherent)
			bus_dma_tag_destroy(priv->dmat_coherent);
		mtx_destroy(&priv->lock);
		kfree(priv);
		pdev->dev.dma_priv = NULL;
	}

	if (pdev->pwrdom != NULL) {
		pwrdom_disable(pdev->pwrdom);
		pwrdom_release(pdev->pwrdom);
		pdev->pwrdom = NULL;
	}

	kfree(pdev->dev.of_node);
	pdev->dev.of_node = NULL;

	return (0);
}

struct lkpi_platform_resource *
platform_get_resource(struct platform_device *pdev, unsigned int type,
    unsigned int index)
{
	struct resource *bsd_res;
	int bsd_type;
	int rid;

	switch (type) {
	case LKPI_IORESOURCE_MEM:
		bsd_type = SYS_RES_MEMORY;
		break;
	case LKPI_IORESOURCE_IRQ:
		bsd_type = SYS_RES_IRQ;
		break;
	case LKPI_IORESOURCE_IO:
		bsd_type = SYS_RES_IOPORT;
		break;
	default:
		return (NULL);
	}

	rid = index;
	bsd_res = bus_alloc_resource_any(pdev->dev.bsddev, bsd_type,
	    &rid, RF_ACTIVE | RF_SHAREABLE);
	if (bsd_res == NULL)
		return (NULL);

	if (pdev->bsd_nres >= LKPI_PLATFORM_MAX_RES) {
		bus_release_resource(pdev->dev.bsddev, bsd_type, rid, bsd_res);
		return (NULL);
	}

	pdev->bsd_res[pdev->bsd_nres] = bsd_res;
	pdev->bsd_rid[pdev->bsd_nres] = rid;
	pdev->bsd_nres++;

	pdev->lkpi_res[index].start = rman_get_start(bsd_res);
	pdev->lkpi_res[index].end = rman_get_end(bsd_res);
	pdev->lkpi_res[index].flags = type;
	if (pdev->num_resources <= (int)index)
		pdev->num_resources = index + 1;

	return (&pdev->lkpi_res[index]);
}

int
platform_get_irq(struct platform_device *pdev, unsigned int index)
{
	struct resource *bsd_res;
	int rid;

	rid = index;
	bsd_res = bus_alloc_resource_any(pdev->dev.bsddev, SYS_RES_IRQ,
	    &rid, RF_ACTIVE | RF_SHAREABLE);
	if (bsd_res == NULL)
		return (-ENXIO);

	if (pdev->bsd_nres < LKPI_PLATFORM_MAX_RES) {
		pdev->bsd_res[pdev->bsd_nres] = bsd_res;
		pdev->bsd_rid[pdev->bsd_nres] = rid;
		pdev->bsd_nres++;
	}

	return (rman_get_start(bsd_res));
}

void __iomem *
devm_platform_ioremap_resource(struct platform_device *pdev,
    unsigned int index)
{
	struct resource *bsd_res;
	int rid;

	rid = index;
	bsd_res = bus_alloc_resource_any(pdev->dev.bsddev, SYS_RES_MEMORY,
	    &rid, RF_ACTIVE);
	if (bsd_res == NULL)
		return (ERR_PTR(-ENOMEM));

	if (pdev->bsd_nres < LKPI_PLATFORM_MAX_RES) {
		pdev->bsd_res[pdev->bsd_nres] = bsd_res;
		pdev->bsd_rid[pdev->bsd_nres] = rid;
		pdev->bsd_nres++;
	}

	pdev->lkpi_res[index].start = rman_get_start(bsd_res);
	pdev->lkpi_res[index].end = rman_get_end(bsd_res);
	pdev->lkpi_res[index].flags = LKPI_IORESOURCE_MEM;
	if (pdev->num_resources <= (int)index)
		pdev->num_resources = index + 1;

	return ((void __iomem *)rman_get_bushandle(bsd_res));
}

int
linux_platform_register_driver(struct platform_driver *pdrv)
{
	devclass_t dc;

	if (pdrv->driver.name == NULL)
		return (-EINVAL);

	pdrv->bsddriver.name = pdrv->driver.name;
	pdrv->bsddriver.methods = platform_methods;
	pdrv->bsddriver.size = sizeof(struct platform_device);

	dc = devclass_find("simplebus");
	if (dc == NULL)
		return (-ENOENT);

	return (-devclass_add_driver(dc, &pdrv->bsddriver,
	    BUS_PASS_DEFAULT, &pdrv->bsdclass));
}

void
linux_platform_unregister_driver(struct platform_driver *pdrv)
{
	devclass_t dc;

	dc = devclass_find("simplebus");
	if (dc == NULL)
		return;
	devclass_delete_driver(dc, &pdrv->bsddriver);
}

#else /* !FDT */

int
linux_platform_register_driver(struct platform_driver *pdrv __unused)
{
	return (-ENOENT);
}

void
linux_platform_unregister_driver(struct platform_driver *pdrv __unused)
{
}

#endif /* FDT */
