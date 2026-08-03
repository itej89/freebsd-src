/*-
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * Copyright (c) 2026 Tej Kiran
 */

#ifndef _LINUXKPI_LINUX_PROPERTY_H
#define	_LINUXKPI_LINUX_PROPERTY_H

#include <linux/fwnode.h>
#include <linux/types.h>

static inline int
device_property_read_u32(struct device *dev __unused,
    const char *propname __unused, u32 *val __unused)
{
	return (-ENOSYS);
}

static inline int
device_property_read_string(struct device *dev __unused,
    const char *propname __unused, const char **val __unused)
{
	return (-ENOSYS);
}

static inline bool
device_property_present(struct device *dev __unused,
    const char *propname __unused)
{
	return (false);
}

#endif /* _LINUXKPI_LINUX_PROPERTY_H */
