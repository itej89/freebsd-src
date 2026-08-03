/*-
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * Copyright (c) 2026 Tej Kiran
 *
 * LinuxKPI clock consumer shim implementation.
 *
 * This file includes FreeBSD's <dev/clk/clk.h> which defines
 * clk_t as struct clk *. We must NOT include <linux/clk.h> here
 * because it would redefine struct clk via the #define macros.
 * Instead we forward-declare the lkpi_* prototypes locally.
 */

#include <sys/param.h>
#include <sys/kernel.h>
#include <sys/malloc.h>
#include <sys/bus.h>

#ifdef FDT
#include <dev/ofw/openfirm.h>
#include <dev/ofw/ofw_bus.h>
#include <dev/clk/clk.h>
#endif

#include <linux/kernel.h>
#include <linux/device.h>
#include <linux/slab.h>
#include <linux/err.h>

/* Forward declarations — must match <linux/clk.h> exactly. */
struct clk;
struct clk *lkpi_clk_get(struct device *dev, const char *con_id);
struct clk *lkpi_devm_clk_get(struct device *dev, const char *con_id);
struct clk *lkpi_devm_clk_get_optional(struct device *dev, const char *con_id);
void lkpi_clk_put(struct clk *clk);
int lkpi_clk_enable(struct clk *clk);
void lkpi_clk_disable(struct clk *clk);
int lkpi_clk_prepare(struct clk *clk);
void lkpi_clk_unprepare(struct clk *clk);
int lkpi_clk_prepare_enable(struct clk *clk);
void lkpi_clk_disable_unprepare(struct clk *clk);
int lkpi_clk_set_rate(struct clk *clk, unsigned long rate);
unsigned long lkpi_clk_get_rate(struct clk *clk);
int lkpi_clk_set_parent(struct clk *clk, struct clk *parent);
bool lkpi_clk_is_enabled(struct clk *clk);

struct lkpi_clk {
#ifdef FDT
	clk_t		bsd_clk;
#else
	void		*bsd_clk;
#endif
	bool		prepared;
	bool		enabled;
};

#define	LKPI_CLK(p)	((struct lkpi_clk *)(p))
#define	TO_CLK(p)	((struct clk *)(void *)(p))

struct clk *
lkpi_clk_get(struct device *dev, const char *con_id)
{
#ifdef FDT
	struct lkpi_clk *lc;
	int error;

	if (dev == NULL || dev->bsddev == NULL)
		return (ERR_PTR(-EINVAL));

	lc = kmalloc(sizeof(*lc), GFP_KERNEL);
	if (lc == NULL)
		return (ERR_PTR(-ENOMEM));

	lc->prepared = false;
	lc->enabled = false;

	if (con_id != NULL)
		error = clk_get_by_ofw_name(dev->bsddev, 0, con_id,
		    &lc->bsd_clk);
	else
		error = clk_get_by_ofw_index(dev->bsddev, 0, 0,
		    &lc->bsd_clk);

	if (error != 0) {
		kfree(lc);
		return (ERR_PTR(-ENOENT));
	}
	return (TO_CLK(lc));
#else
	return (ERR_PTR(-ENOSYS));
#endif
}

static void
lkpi_devm_clk_release(struct device *dev __unused, void *res)
{
	struct lkpi_clk *lc = *(struct lkpi_clk **)res;

	if (lc == NULL)
		return;
#ifdef FDT
	if (lc->enabled)
		clk_disable(lc->bsd_clk);
	clk_release(lc->bsd_clk);
#endif
	kfree(lc);
}

struct clk *
lkpi_devm_clk_get(struct device *dev, const char *con_id)
{
	struct clk *c;
	struct lkpi_clk **devres;

	c = lkpi_clk_get(dev, con_id);
	if (IS_ERR(c))
		return (c);

	devres = lkpi_devres_alloc(lkpi_devm_clk_release,
	    sizeof(*devres), GFP_KERNEL);
	if (devres == NULL) {
		lkpi_clk_put(c);
		return (ERR_PTR(-ENOMEM));
	}
	*devres = LKPI_CLK(c);
	lkpi_devres_add(dev, devres);

	return (c);
}

struct clk *
lkpi_devm_clk_get_optional(struct device *dev, const char *con_id)
{
	struct clk *c;

	c = lkpi_devm_clk_get(dev, con_id);
	if (IS_ERR(c) && PTR_ERR(c) == -ENOENT)
		return (NULL);
	return (c);
}

void
lkpi_clk_put(struct clk *c)
{
	struct lkpi_clk *lc = LKPI_CLK(c);

	if (c == NULL || IS_ERR(c))
		return;
#ifdef FDT
	if (lc->enabled)
		clk_disable(lc->bsd_clk);
	clk_release(lc->bsd_clk);
#endif
	kfree(lc);
}

int
lkpi_clk_enable(struct clk *c)
{
	struct lkpi_clk *lc = LKPI_CLK(c);

	if (c == NULL || IS_ERR(c))
		return (c == NULL ? 0 : PTR_ERR(c));
	if (lc->enabled)
		return (0);
#ifdef FDT
	{
		int error;
		error = clk_enable(lc->bsd_clk);
		if (error != 0)
			return (-error);
	}
#endif
	lc->enabled = true;
	return (0);
}

void
lkpi_clk_disable(struct clk *c)
{
	struct lkpi_clk *lc = LKPI_CLK(c);

	if (c == NULL || IS_ERR(c))
		return;
	if (!lc->enabled)
		return;
#ifdef FDT
	clk_disable(lc->bsd_clk);
#endif
	lc->enabled = false;
}

int
lkpi_clk_prepare(struct clk *c)
{
	struct lkpi_clk *lc = LKPI_CLK(c);

	if (c == NULL || IS_ERR(c))
		return (c == NULL ? 0 : PTR_ERR(c));
	lc->prepared = true;
	return (0);
}

void
lkpi_clk_unprepare(struct clk *c)
{
	struct lkpi_clk *lc = LKPI_CLK(c);

	if (c == NULL || IS_ERR(c))
		return;
	lc->prepared = false;
}

int
lkpi_clk_prepare_enable(struct clk *c)
{
	int error;

	error = lkpi_clk_prepare(c);
	if (error != 0)
		return (error);
	error = lkpi_clk_enable(c);
	if (error != 0)
		lkpi_clk_unprepare(c);
	return (error);
}

void
lkpi_clk_disable_unprepare(struct clk *c)
{

	lkpi_clk_disable(c);
	lkpi_clk_unprepare(c);
}

int
lkpi_clk_set_rate(struct clk *c, unsigned long rate)
{
#ifdef FDT
	struct lkpi_clk *lc = LKPI_CLK(c);

	if (c == NULL || IS_ERR(c))
		return (c == NULL ? 0 : PTR_ERR(c));
	return (-clk_set_freq(lc->bsd_clk, (uint64_t)rate,
	    CLK_SET_ROUND_ANY));
#else
	return (-ENOSYS);
#endif
}

unsigned long
lkpi_clk_get_rate(struct clk *c)
{
#ifdef FDT
	struct lkpi_clk *lc = LKPI_CLK(c);
	uint64_t freq;

	if (c == NULL || IS_ERR(c))
		return (0);
	if (clk_get_freq(lc->bsd_clk, &freq) != 0)
		return (0);
	return ((unsigned long)freq);
#else
	return (0);
#endif
}

int
lkpi_clk_set_parent(struct clk *c, struct clk *parent)
{
#ifdef FDT
	struct lkpi_clk *lc = LKPI_CLK(c);
	struct lkpi_clk *lp = LKPI_CLK(parent);

	if (c == NULL || IS_ERR(c) || parent == NULL || IS_ERR(parent))
		return (-EINVAL);
	return (-clk_set_parent_by_clk(lc->bsd_clk, lp->bsd_clk));
#else
	return (-ENOSYS);
#endif
}

bool
lkpi_clk_is_enabled(struct clk *c)
{
	struct lkpi_clk *lc = LKPI_CLK(c);

	if (c == NULL || IS_ERR(c))
		return (false);
	return (lc->enabled);
}
