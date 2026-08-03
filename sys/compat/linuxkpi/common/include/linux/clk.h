/*-
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * Copyright (c) 2026 Tej Kiran
 *
 * LinuxKPI clock consumer API shim.
 * All public functions use the lkpi_ prefix, with #defines mapping
 * the Linux names. This avoids namespace collision with FreeBSD's
 * identically-named clock functions in <dev/clk/clk.h>.
 */

#ifndef _LINUXKPI_LINUX_CLK_H
#define	_LINUXKPI_LINUX_CLK_H

#include <linux/device.h>
#include <linux/types.h>
#include <linux/err.h>

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

#define	clk_get			lkpi_clk_get
#define	devm_clk_get		lkpi_devm_clk_get
#define	devm_clk_get_optional	lkpi_devm_clk_get_optional
#define	clk_put			lkpi_clk_put
#define	clk_enable		lkpi_clk_enable
#define	clk_disable		lkpi_clk_disable
#define	clk_prepare		lkpi_clk_prepare
#define	clk_unprepare		lkpi_clk_unprepare
#define	clk_prepare_enable	lkpi_clk_prepare_enable
#define	clk_disable_unprepare	lkpi_clk_disable_unprepare
#define	clk_set_rate		lkpi_clk_set_rate
#define	clk_get_rate		lkpi_clk_get_rate
#define	clk_set_parent		lkpi_clk_set_parent
#define	__clk_is_enabled	lkpi_clk_is_enabled

#endif /* _LINUXKPI_LINUX_CLK_H */
