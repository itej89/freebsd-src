/*-
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * Copyright (c) 2026 Tej Kiran
 *
 * LinuxKPI reset control shim implementation.
 * Wraps FreeBSD's hwreset subsystem.
 */

#include <sys/param.h>
#include <sys/kernel.h>
#include <sys/malloc.h>
#include <sys/bus.h>

#ifdef FDT
#include <dev/ofw/openfirm.h>
#include <dev/ofw/ofw_bus.h>
#include <dev/hwreset/hwreset.h>
#endif

#include <linux/kernel.h>
#include <linux/device.h>
#include <linux/slab.h>
#include <linux/err.h>

/* Forward declarations — must match <linux/reset.h> exactly. */
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

struct reset_control {
#ifdef FDT
	hwreset_t	bsd_rst;
#else
	void		*bsd_rst;
#endif
};

struct reset_control *
lkpi_reset_control_get(struct device *dev, const char *id)
{
#ifdef FDT
	struct reset_control *rstc;
	int error;

	if (dev == NULL || dev->bsddev == NULL)
		return (ERR_PTR(-EINVAL));

	rstc = kmalloc(sizeof(*rstc), GFP_KERNEL);
	if (rstc == NULL)
		return (ERR_PTR(-ENOMEM));

	if (id != NULL)
		error = hwreset_get_by_ofw_name(dev->bsddev, 0,
		    __DECONST(char *, id), &rstc->bsd_rst);
	else
		error = hwreset_get_by_ofw_idx(dev->bsddev, 0, 0,
		    &rstc->bsd_rst);

	if (error != 0) {
		kfree(rstc);
		return (ERR_PTR(-ENOENT));
	}
	return (rstc);
#else
	return (ERR_PTR(-ENOSYS));
#endif
}

static void
lkpi_devm_reset_release(struct device *dev __unused, void *res)
{
	struct reset_control *rstc = *(struct reset_control **)res;

	if (rstc == NULL)
		return;
#ifdef FDT
	hwreset_release(rstc->bsd_rst);
#endif
	kfree(rstc);
}

struct reset_control *
lkpi_devm_reset_control_get(struct device *dev, const char *id)
{
	struct reset_control *rstc;
	struct reset_control **devres;

	rstc = lkpi_reset_control_get(dev, id);
	if (IS_ERR(rstc))
		return (rstc);

	devres = lkpi_devres_alloc(lkpi_devm_reset_release,
	    sizeof(*devres), GFP_KERNEL);
	if (devres == NULL) {
		lkpi_reset_control_put(rstc);
		return (ERR_PTR(-ENOMEM));
	}
	*devres = rstc;
	lkpi_devres_add(dev, devres);

	return (rstc);
}

struct reset_control *
lkpi_devm_reset_control_get_optional(struct device *dev, const char *id)
{
	struct reset_control *rstc;

	rstc = lkpi_devm_reset_control_get(dev, id);
	if (IS_ERR(rstc) && PTR_ERR(rstc) == -ENOENT)
		return (NULL);
	return (rstc);
}

void
lkpi_reset_control_put(struct reset_control *rstc)
{

	if (rstc == NULL || IS_ERR(rstc))
		return;
#ifdef FDT
	hwreset_release(rstc->bsd_rst);
#endif
	kfree(rstc);
}

int
lkpi_reset_control_assert(struct reset_control *rstc)
{

	if (rstc == NULL || IS_ERR(rstc))
		return (rstc == NULL ? 0 : PTR_ERR(rstc));
#ifdef FDT
	return (-hwreset_assert(rstc->bsd_rst));
#else
	return (-ENOSYS);
#endif
}

int
lkpi_reset_control_deassert(struct reset_control *rstc)
{

	if (rstc == NULL || IS_ERR(rstc))
		return (rstc == NULL ? 0 : PTR_ERR(rstc));
#ifdef FDT
	return (-hwreset_deassert(rstc->bsd_rst));
#else
	return (-ENOSYS);
#endif
}

int
lkpi_reset_control_reset(struct reset_control *rstc)
{
	int error;

	error = lkpi_reset_control_assert(rstc);
	if (error != 0)
		return (error);

	DELAY(10);

	return (lkpi_reset_control_deassert(rstc));
}
