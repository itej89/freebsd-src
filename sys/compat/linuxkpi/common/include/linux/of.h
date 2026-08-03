/*-
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * Copyright (c) 2023 Serenity Cyber Security, LLC.
 * Copyright (c) 2026 Tej Kiran
 */

#ifndef _LINUXKPI_LINUX_OF_H
#define	_LINUXKPI_LINUX_OF_H

#include <linux/kobject.h>
#include <linux/types.h>
#include <linux/errno.h>

#ifdef FDT
#include <dev/ofw/openfirm.h>
#include <dev/ofw/ofw_bus.h>
#include <dev/ofw/ofw_bus_subr.h>
#endif

struct device_node {
	uint32_t	phandle;
	const char	*full_name;
};

static inline struct device_node *
of_node_get(struct device_node *node)
{
	return (node);
}

static inline void
of_node_put(struct device_node *node)
{
}

#ifdef FDT

static inline int
of_property_read_u32(const struct device_node *np, const char *propname,
    u32 *out_value)
{
	pcell_t val;

	if (np == NULL)
		return (-EINVAL);
	if (OF_getencprop(np->phandle, propname, &val, sizeof(val)) <= 0)
		return (-EINVAL);
	*out_value = (u32)val;
	return (0);
}

static inline int
of_property_read_u32_array(const struct device_node *np, const char *propname,
    u32 *out_values, size_t sz)
{
	ssize_t ret;

	if (np == NULL)
		return (-EINVAL);
	ret = OF_getencprop(np->phandle, propname, (pcell_t *)out_values,
	    sz * sizeof(u32));
	if (ret <= 0)
		return (-EINVAL);
	if ((size_t)ret != sz * sizeof(u32))
		return (-EOVERFLOW);
	return (0);
}

static inline int
of_property_read_string(const struct device_node *np, const char *propname,
    const char **out_string)
{
	char *buf;
	ssize_t len;

	if (np == NULL)
		return (-EINVAL);
	len = OF_getprop_alloc(np->phandle, propname, (void **)&buf);
	if (len <= 0)
		return (-EINVAL);
	*out_string = buf;
	return (0);
}

static inline bool
of_property_read_bool(const struct device_node *np, const char *propname)
{

	if (np == NULL)
		return (false);
	return (OF_hasprop(np->phandle, propname));
}

static inline bool
of_device_is_compatible(const struct device_node *np, const char *compat)
{

	if (np == NULL)
		return (false);
	return (ofw_bus_node_is_compatible(np->phandle, compat));
}

static inline int
of_property_count_elems_of_size(const struct device_node *np,
    const char *propname, int elem_size)
{
	ssize_t len;

	if (np == NULL || elem_size == 0)
		return (-EINVAL);
	len = OF_getproplen(np->phandle, propname);
	if (len < 0)
		return (-EINVAL);
	return (len / elem_size);
}

static inline int
of_get_child_count(const struct device_node *np)
{
	phandle_t child;
	int count;

	if (np == NULL)
		return (0);
	count = 0;
	for (child = OF_child(np->phandle); child != 0;
	    child = OF_peer(child))
		count++;
	return (count);
}

#else /* !FDT */

static inline int
of_property_read_u32(const struct device_node *np __unused,
    const char *propname __unused, u32 *out_value __unused)
{
	return (-ENOSYS);
}

static inline int
of_property_read_u32_array(const struct device_node *np __unused,
    const char *propname __unused, u32 *out_values __unused,
    size_t sz __unused)
{
	return (-ENOSYS);
}

static inline int
of_property_read_string(const struct device_node *np __unused,
    const char *propname __unused, const char **out_string __unused)
{
	return (-ENOSYS);
}

static inline bool
of_property_read_bool(const struct device_node *np __unused,
    const char *propname __unused)
{
	return (false);
}

static inline bool
of_device_is_compatible(const struct device_node *np __unused,
    const char *compat __unused)
{
	return (false);
}

static inline int
of_property_count_elems_of_size(const struct device_node *np __unused,
    const char *propname __unused, int elem_size __unused)
{
	return (-ENOSYS);
}

static inline int
of_get_child_count(const struct device_node *np __unused)
{
	return (0);
}

#endif /* FDT */

#endif /* _LINUXKPI_LINUX_OF_H */
