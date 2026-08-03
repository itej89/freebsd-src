/*-
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * Copyright (c) 2026 Tej Kiran
 *
 * LinuxKPI component framework stubs.
 * Real implementation deferred until DRM subsystem work.
 */

#ifndef _LINUXKPI_LINUX_COMPONENT_H
#define	_LINUXKPI_LINUX_COMPONENT_H

#include <linux/device.h>

struct component_ops {
	int  (*bind)(struct device *, struct device *, void *);
	void (*unbind)(struct device *, struct device *, void *);
};

struct component_master_ops {
	int  (*bind)(struct device *);
	void (*unbind)(struct device *);
};

struct component_match;

static inline int
component_add(struct device *dev __unused,
    const struct component_ops *ops __unused)
{
	return (0);
}

static inline void
component_del(struct device *dev __unused,
    const struct component_ops *ops __unused)
{
}

static inline int
component_bind_all(struct device *master __unused, void *data __unused)
{
	return (0);
}

static inline void
component_unbind_all(struct device *master __unused, void *data __unused)
{
}

static inline int
component_master_add_with_match(struct device *master __unused,
    const struct component_master_ops *ops __unused,
    struct component_match *match __unused)
{
	return (0);
}

static inline void
component_master_del(struct device *master __unused,
    const struct component_master_ops *ops __unused)
{
}

#endif /* _LINUXKPI_LINUX_COMPONENT_H */
