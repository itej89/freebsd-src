/*-
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * Copyright (c) 2026 Tej Kiran
 *
 * OF IRQ stubs. Most Linux platform drivers use platform_get_irq()
 * which is fully implemented. of_irq_get() is an alternative path
 * that requires mapping device_node back to a device_t — deferred
 * until a driver needs it.
 */

#ifndef _LINUXKPI_LINUX_OF_IRQ_H
#define	_LINUXKPI_LINUX_OF_IRQ_H

#include <linux/of.h>

static inline int
of_irq_get(struct device_node *np __unused, int index __unused)
{
	return (-ENOSYS);
}

static inline unsigned int
irq_of_parse_and_map(struct device_node *np __unused, int index __unused)
{
	return (0);
}

static inline int
of_irq_count(struct device_node *np __unused)
{
	return (0);
}

#endif /* _LINUXKPI_LINUX_OF_IRQ_H */
