/*-
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * Copyright (c) 2026 Tej Kiran
 *
 * OF address translation stubs. Most Linux platform drivers use
 * devm_platform_ioremap_resource() which is fully implemented.
 * of_iomap() is an alternative path — stubbed until needed.
 */

#ifndef _LINUXKPI_LINUX_OF_ADDRESS_H
#define	_LINUXKPI_LINUX_OF_ADDRESS_H

#include <linux/of.h>
#include <linux/io.h>

static inline void __iomem *
of_iomap(struct device_node *np __unused, int index __unused)
{
	return (NULL);
}

static inline int
of_address_to_resource(struct device_node *np __unused, int index __unused,
    void *r __unused)
{
	return (-ENOSYS);
}

#endif /* _LINUXKPI_LINUX_OF_ADDRESS_H */
