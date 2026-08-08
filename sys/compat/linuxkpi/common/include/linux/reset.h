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

static inline struct reset_control *
devm_reset_control_get_optional_shared(struct device *dev, const char *id)
{
	return (lkpi_devm_reset_control_get_optional(dev, id));
}

struct reset_control_bulk_data {
	const char *id;
	struct reset_control *rstc;
};

static inline int
devm_reset_control_bulk_get_optional_shared(struct device *dev,
    int num_rstcs, struct reset_control_bulk_data *rstcs)
{
	int i;

	for (i = 0; i < num_rstcs; i++) {
		rstcs[i].rstc = lkpi_devm_reset_control_get_optional(dev,
		    rstcs[i].id);
		if (IS_ERR(rstcs[i].rstc))
			rstcs[i].rstc = NULL;
	}
	return (0);
}

static inline int
reset_control_bulk_deassert(int num_rstcs, struct reset_control_bulk_data *rstcs)
{
	int i, error;

	for (i = 0; i < num_rstcs; i++) {
		if (!rstcs[i].rstc)
			continue;
		error = lkpi_reset_control_deassert(rstcs[i].rstc);
		if (error != 0) {
			while (--i >= 0)
				if (rstcs[i].rstc)
					lkpi_reset_control_assert(rstcs[i].rstc);
			return (error);
		}
	}
	return (0);
}

static inline int
reset_control_bulk_assert(int num_rstcs, struct reset_control_bulk_data *rstcs)
{
	int i;

	for (i = num_rstcs - 1; i >= 0; i--)
		if (rstcs[i].rstc)
			lkpi_reset_control_assert(rstcs[i].rstc);
	return (0);
}

#endif /* _LINUXKPI_LINUX_RESET_H */
