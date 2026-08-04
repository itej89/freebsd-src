/*-
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * Copyright (c) 2026 Tej Kiran
 *
 * LinuxKPI component framework.
 * Coordinates binding of aggregate drivers (e.g. DRM display
 * pipeline) where a logical device spans multiple sub-devices.
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

int lkpi_component_add(struct device *dev, const struct component_ops *ops);
void lkpi_component_del(struct device *dev, const struct component_ops *ops);
int lkpi_component_bind_all(struct device *master, void *data);
void lkpi_component_unbind_all(struct device *master, void *data);
int lkpi_component_master_add_with_match(struct device *dev,
    const struct component_master_ops *ops, struct component_match *match);
void lkpi_component_master_del(struct device *dev,
    const struct component_master_ops *ops);

void lkpi_component_match_add_release(struct device *master,
    struct component_match **matchptr,
    void (*release)(struct device *, void *),
    int (*compare)(struct device *, void *),
    void *compare_data);

#define	component_add		lkpi_component_add
#define	component_del		lkpi_component_del
#define	component_bind_all	lkpi_component_bind_all
#define	component_unbind_all	lkpi_component_unbind_all
#define	component_master_add_with_match lkpi_component_master_add_with_match
#define	component_master_del	lkpi_component_master_del
#define	component_match_add_release lkpi_component_match_add_release

static inline void
component_match_add(struct device *master, struct component_match **matchptr,
    int (*compare)(struct device *, void *), void *compare_data)
{

	lkpi_component_match_add_release(master, matchptr, NULL,
	    compare, compare_data);
}

static inline int
component_compare_of(struct device *dev, void *data)
{

	return (dev->of_node == (struct device_node *)data);
}

static inline int
component_compare_dev_name(struct device *dev, void *data)
{
	const char *name = data;

	return (strcmp(dev_name(dev), name) == 0);
}

#endif /* _LINUXKPI_LINUX_COMPONENT_H */
