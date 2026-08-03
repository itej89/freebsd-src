/*-
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * Copyright (c) 2026 Tej Kiran
 *
 * LinuxKPI reset control API shim.
 * Wraps FreeBSD's <dev/hwreset/hwreset.h> for Linux drivers.
 */

#ifndef _LINUXKPI_LINUX_RESET_H
#define	_LINUXKPI_LINUX_RESET_H

#include <linux/device.h>
#include <linux/types.h>
#include <linux/err.h>

struct reset_control;

struct reset_control *lkpi_reset_control_get(struct device *dev,
    const char *id);
struct reset_control *lkpi_devm_reset_control_get(struct device *dev,
    const char *id);
struct reset_control *lkpi_devm_reset_control_get_optional(struct device *dev,
    const char *id);
void lkpi_reset_control_put(struct reset_control *rstc);
int lkpi_reset_control_assert(struct reset_control *rstc);
int lkpi_reset_control_deassert(struct reset_control *rstc);
int lkpi_reset_control_reset(struct reset_control *rstc);

#define	reset_control_get		lkpi_reset_control_get
#define	reset_control_put		lkpi_reset_control_put
#define	reset_control_assert		lkpi_reset_control_assert
#define	reset_control_deassert		lkpi_reset_control_deassert
#define	reset_control_reset		lkpi_reset_control_reset
#define	devm_reset_control_get		lkpi_devm_reset_control_get

static inline struct reset_control *
devm_reset_control_get_exclusive(struct device *dev, const char *id)
{
	return (lkpi_devm_reset_control_get(dev, id));
}

static inline struct reset_control *
devm_reset_control_get_optional_exclusive(struct device *dev, const char *id)
{
	return (lkpi_devm_reset_control_get_optional(dev, id));
}

static inline struct reset_control *
devm_reset_control_get_shared(struct device *dev, const char *id)
{
	return (lkpi_devm_reset_control_get(dev, id));
}

#endif /* _LINUXKPI_LINUX_RESET_H */
