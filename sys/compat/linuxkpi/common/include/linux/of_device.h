/*-
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * Copyright (c) 2026 Tej Kiran
 */

#ifndef _LINUXKPI_LINUX_OF_DEVICE_H
#define	_LINUXKPI_LINUX_OF_DEVICE_H

#include <linux/device.h>
#include <linux/of.h>
#include <linux/mod_devicetable.h>

static inline const struct of_device_id *
of_match_device(const struct of_device_id *matches,
    const struct device *dev)
{
#ifdef FDT
	const struct of_device_id *id;

	if (matches == NULL || dev == NULL || dev->of_node == NULL)
		return (NULL);
	for (id = matches; id->compatible[0] != '\0'; id++) {
		if (of_device_is_compatible(dev->of_node, id->compatible))
			return (id);
	}
#endif
	return (NULL);
}

static inline const void *
of_device_get_match_data(const struct device *dev)
{
	const struct of_device_id *id;

	if (dev == NULL || dev->driver == NULL)
		return (NULL);
	id = of_match_device(dev->driver->of_match_table, dev);
	if (id == NULL)
		return (NULL);
	return (id->data);
}

#endif /* _LINUXKPI_LINUX_OF_DEVICE_H */
