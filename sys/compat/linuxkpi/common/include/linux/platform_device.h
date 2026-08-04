/*-
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * Copyright (c) 2020-2022 Bjoern A. Zeeb
 * Copyright (c) 2026 Tej Kiran
 */

#ifndef	_LINUXKPI_LINUX_PLATFORM_DEVICE_H
#define	_LINUXKPI_LINUX_PLATFORM_DEVICE_H

#include <linux/kernel.h>
#include <linux/device.h>
#include <linux/mod_devicetable.h>

#define	LKPI_PLATFORM_MAX_RES	8

struct lkpi_platform_resource {
	resource_size_t		start;
	resource_size_t		end;
	unsigned long		flags;
};

struct platform_device {
	const char		*name;
	int			id;
	bool			id_auto;
	struct device		dev;
	struct lkpi_platform_resource lkpi_res[LKPI_PLATFORM_MAX_RES];
	int			num_resources;
	uint32_t		node;
	void			*bsd_res[LKPI_PLATFORM_MAX_RES];
	int			bsd_rid[LKPI_PLATFORM_MAX_RES];
	int			bsd_nres;
	void			*pwrdom;
};

struct platform_driver {
	int  (*probe)(struct platform_device *);
	void (*remove)(struct platform_device *);
	void (*shutdown)(struct platform_device *);
	struct device_driver	driver;
	driver_t		bsddriver;
	devclass_t		bsdclass;
};

#define	to_platform_device(d) \
	container_of(d, struct platform_device, dev)

#define	platform_get_drvdata(pdev) \
	dev_get_drvdata(&(pdev)->dev)
#define	platform_set_drvdata(pdev, data) \
	dev_set_drvdata(&(pdev)->dev, data)

int linux_platform_register_driver(struct platform_driver *);
void linux_platform_unregister_driver(struct platform_driver *);

struct lkpi_platform_resource *platform_get_resource(
    struct platform_device *, unsigned int, unsigned int);
int platform_get_irq(struct platform_device *, unsigned int);
void __iomem *devm_platform_ioremap_resource(struct platform_device *,
    unsigned int);

#define	module_platform_driver(_drv)					\
	module_driver(_drv, linux_platform_register_driver,		\
	    linux_platform_unregister_driver)

#define	builtin_platform_driver(_drv) module_platform_driver(_drv)

static __inline int
platform_driver_register(struct platform_driver *pdrv)
{
	return (linux_platform_register_driver(pdrv));
}

static __inline void
platform_driver_unregister(struct platform_driver *pdrv)
{
	linux_platform_unregister_driver(pdrv);
}

static __inline int
platform_device_register(struct platform_device *pdev __unused)
{
	return (0);
}

static __inline void
platform_device_unregister(struct platform_device *pdev __unused)
{
}

static __inline void *
dev_get_platdata(struct device *dev __unused)
{
	return (NULL);
}

static __inline bool
dev_is_platform(struct device *dev __unused)
{
	return (false);
}

#endif	/* _LINUXKPI_LINUX_PLATFORM_DEVICE_H */
