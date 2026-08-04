/*-
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * Copyright (c) 2026 Tej Kiran
 *
 * LinuxKPI component framework implementation.
 *
 * Tracks registered components and match lists. When a master
 * registers and all its required components are present, the
 * master's bind callback is invoked. When a new component registers,
 * pending masters are re-checked.
 */

#include <sys/param.h>
#include <sys/kernel.h>
#include <sys/malloc.h>
#include <sys/mutex.h>
#include <sys/queue.h>

#include <linux/kernel.h>
#include <linux/device.h>
#include <linux/slab.h>
#include <linux/err.h>

struct component_ops;
struct component_master_ops;
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

struct component_ops {
	int  (*bind)(struct device *, struct device *, void *);
	void (*unbind)(struct device *, struct device *, void *);
};

struct component_master_ops {
	int  (*bind)(struct device *);
	void (*unbind)(struct device *);
};

#define	LKPI_COMP_MAX_MATCH	16
#define	LKPI_COMP_MAX		32

struct component_match_entry {
	int		(*compare)(struct device *, void *);
	void		(*release)(struct device *, void *);
	void		*compare_data;
	struct device	*matched_dev;
};

struct component_match {
	int				num;
	struct component_match_entry	entry[LKPI_COMP_MAX_MATCH];
};

struct lkpi_component {
	TAILQ_ENTRY(lkpi_component)	link;
	struct device			*dev;
	const struct component_ops	*ops;
	bool				bound;
};

struct lkpi_master {
	TAILQ_ENTRY(lkpi_master)	link;
	struct device			*dev;
	const struct component_master_ops *ops;
	struct component_match		*match;
	bool				bound;
	void				*bind_data;
};

static TAILQ_HEAD(, lkpi_component) lkpi_components =
    TAILQ_HEAD_INITIALIZER(lkpi_components);
static TAILQ_HEAD(, lkpi_master) lkpi_masters =
    TAILQ_HEAD_INITIALIZER(lkpi_masters);
static struct mtx lkpi_comp_mtx;
MTX_SYSINIT(lkpi_comp, &lkpi_comp_mtx, "lkpi_comp", MTX_DEF);

static struct lkpi_component *
lkpi_comp_find(struct device *dev)
{
	struct lkpi_component *c;

	TAILQ_FOREACH(c, &lkpi_components, link) {
		if (c->dev == dev)
			return (c);
	}
	return (NULL);
}

static bool
lkpi_master_all_matched(struct lkpi_master *m)
{
	struct component_match *match = m->match;
	struct lkpi_component *c;
	int i;

	if (match == NULL || match->num == 0)
		return (true);

	for (i = 0; i < match->num; i++) {
		match->entry[i].matched_dev = NULL;
		TAILQ_FOREACH(c, &lkpi_components, link) {
			if (match->entry[i].compare(c->dev,
			    match->entry[i].compare_data)) {
				match->entry[i].matched_dev = c->dev;
				break;
			}
		}
		if (match->entry[i].matched_dev == NULL)
			return (false);
	}
	return (true);
}

static void
lkpi_try_bind_masters(void)
{
	struct lkpi_master *m;

	TAILQ_FOREACH(m, &lkpi_masters, link) {
		if (m->bound)
			continue;
		if (!lkpi_master_all_matched(m))
			continue;
		mtx_unlock(&lkpi_comp_mtx);
		if (m->ops->bind(m->dev) == 0)
			m->bound = true;
		mtx_lock(&lkpi_comp_mtx);
	}
}

int
lkpi_component_add(struct device *dev, const struct component_ops *ops)
{
	struct lkpi_component *c;

	c = kmalloc(sizeof(*c), GFP_KERNEL);
	if (c == NULL)
		return (-ENOMEM);

	c->dev = dev;
	c->ops = ops;
	c->bound = false;

	mtx_lock(&lkpi_comp_mtx);
	TAILQ_INSERT_TAIL(&lkpi_components, c, link);
	lkpi_try_bind_masters();
	mtx_unlock(&lkpi_comp_mtx);

	return (0);
}

void
lkpi_component_del(struct device *dev,
    const struct component_ops *ops __unused)
{
	struct lkpi_component *c;

	mtx_lock(&lkpi_comp_mtx);
	c = lkpi_comp_find(dev);
	if (c != NULL) {
		TAILQ_REMOVE(&lkpi_components, c, link);
		kfree(c);
	}
	mtx_unlock(&lkpi_comp_mtx);
}

int
lkpi_component_bind_all(struct device *master, void *data)
{
	struct lkpi_master *m;
	struct lkpi_component *c;
	int i, error;

	mtx_lock(&lkpi_comp_mtx);
	TAILQ_FOREACH(m, &lkpi_masters, link) {
		if (m->dev == master)
			break;
	}
	if (m == NULL || m->match == NULL) {
		mtx_unlock(&lkpi_comp_mtx);
		return (0);
	}

	m->bind_data = data;

	for (i = 0; i < m->match->num; i++) {
		struct device *cdev = m->match->entry[i].matched_dev;
		if (cdev == NULL)
			continue;
		c = lkpi_comp_find(cdev);
		if (c == NULL || c->ops == NULL || c->ops->bind == NULL)
			continue;
		mtx_unlock(&lkpi_comp_mtx);
		error = c->ops->bind(c->dev, master, data);
		mtx_lock(&lkpi_comp_mtx);
		if (error != 0) {
			mtx_unlock(&lkpi_comp_mtx);
			return (error);
		}
		c->bound = true;
	}
	mtx_unlock(&lkpi_comp_mtx);
	return (0);
}

void
lkpi_component_unbind_all(struct device *master, void *data)
{
	struct lkpi_master *m;
	struct lkpi_component *c;
	int i;

	mtx_lock(&lkpi_comp_mtx);
	TAILQ_FOREACH(m, &lkpi_masters, link) {
		if (m->dev == master)
			break;
	}
	if (m == NULL || m->match == NULL) {
		mtx_unlock(&lkpi_comp_mtx);
		return;
	}

	for (i = m->match->num - 1; i >= 0; i--) {
		struct device *cdev = m->match->entry[i].matched_dev;
		if (cdev == NULL)
			continue;
		c = lkpi_comp_find(cdev);
		if (c == NULL || !c->bound || c->ops == NULL ||
		    c->ops->unbind == NULL)
			continue;
		mtx_unlock(&lkpi_comp_mtx);
		c->ops->unbind(c->dev, master, data);
		mtx_lock(&lkpi_comp_mtx);
		c->bound = false;
	}
	mtx_unlock(&lkpi_comp_mtx);
}

void
lkpi_component_match_add_release(struct device *master __unused,
    struct component_match **matchptr,
    void (*release)(struct device *, void *),
    int (*compare)(struct device *, void *),
    void *compare_data)
{
	struct component_match *match;

	if (*matchptr == NULL) {
		*matchptr = kmalloc(sizeof(**matchptr), GFP_KERNEL);
		if (*matchptr == NULL)
			return;
		(*matchptr)->num = 0;
	}

	match = *matchptr;
	if (match->num >= LKPI_COMP_MAX_MATCH)
		return;

	match->entry[match->num].compare = compare;
	match->entry[match->num].release = release;
	match->entry[match->num].compare_data = compare_data;
	match->entry[match->num].matched_dev = NULL;
	match->num++;
}

int
lkpi_component_master_add_with_match(struct device *dev,
    const struct component_master_ops *ops, struct component_match *match)
{
	struct lkpi_master *m;

	m = kmalloc(sizeof(*m), GFP_KERNEL);
	if (m == NULL)
		return (-ENOMEM);

	m->dev = dev;
	m->ops = ops;
	m->match = match;
	m->bound = false;
	m->bind_data = NULL;

	mtx_lock(&lkpi_comp_mtx);
	TAILQ_INSERT_TAIL(&lkpi_masters, m, link);
	lkpi_try_bind_masters();
	mtx_unlock(&lkpi_comp_mtx);

	return (0);
}

void
lkpi_component_master_del(struct device *dev,
    const struct component_master_ops *ops __unused)
{
	struct lkpi_master *m;

	mtx_lock(&lkpi_comp_mtx);
	TAILQ_FOREACH(m, &lkpi_masters, link) {
		if (m->dev == dev)
			break;
	}
	if (m != NULL) {
		if (m->bound && m->ops->unbind != NULL) {
			mtx_unlock(&lkpi_comp_mtx);
			m->ops->unbind(m->dev);
			mtx_lock(&lkpi_comp_mtx);
		}
		TAILQ_REMOVE(&lkpi_masters, m, link);
		if (m->match != NULL)
			kfree(m->match);
		kfree(m);
	}
	mtx_unlock(&lkpi_comp_mtx);
}
