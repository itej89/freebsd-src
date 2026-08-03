/*-
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * Copyright (c) 2026 Tej Kiran
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

#endif /* _LINUXKPI_LINUX_OF_IRQ_H */
