/*-
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * Copyright (c) 2026 Tej Kiran
 *
 * Firmware-agnostic property API.
 * Redirects to of_property_read_* when dev->of_node is set.
 */

#ifndef _LINUXKPI_LINUX_PROPERTY_H
#define	_LINUXKPI_LINUX_PROPERTY_H

#include <linux/fwnode.h>
#include <linux/types.h>
#include <linux/of.h>

struct device;

static inline int
device_property_read_u32(struct device *dev, const char *propname, u32 *val)
{

	if (dev != NULL && dev->of_node != NULL)
		return (of_property_read_u32(dev->of_node, propname, val));
	return (-ENOSYS);
}

static inline int
device_property_read_string(struct device *dev, const char *propname,
    const char **val)
{

	if (dev != NULL && dev->of_node != NULL)
		return (of_property_read_string(dev->of_node, propname, val));
	return (-ENOSYS);
}

static inline bool
device_property_present(struct device *dev, const char *propname)
{

	if (dev != NULL && dev->of_node != NULL)
		return (of_property_read_bool(dev->of_node, propname));
	return (false);
}

#endif /* _LINUXKPI_LINUX_PROPERTY_H */
