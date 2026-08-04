/*-
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * Copyright (c) 2026 Tej Kiran
 *
 * LinuxKPI regmap MMIO implementation.
 * Wraps readl/writel with register stride from regmap_config.
 */

#include <sys/param.h>
#include <sys/kernel.h>
#include <sys/malloc.h>

#include <linux/kernel.h>
#include <linux/device.h>
#include <linux/slab.h>
#include <linux/err.h>
#include <linux/io.h>

struct regmap_config {
	int		reg_bits;
	int		val_bits;
	int		reg_stride;
	unsigned int	max_register;
	bool		(*writeable_reg)(struct device *dev, unsigned int reg);
	bool		(*readable_reg)(struct device *dev, unsigned int reg);
	bool		(*volatile_reg)(struct device *dev, unsigned int reg);
};

struct regmap *lkpi_devm_regmap_init_mmio(struct device *dev,
    void __iomem *regs, const struct regmap_config *config);
int lkpi_regmap_read(struct regmap *map, unsigned int reg, unsigned int *val);
int lkpi_regmap_write(struct regmap *map, unsigned int reg, unsigned int val);
int lkpi_regmap_update_bits(struct regmap *map, unsigned int reg,
    unsigned int mask, unsigned int val);
int lkpi_regmap_bulk_read(struct regmap *map, unsigned int reg,
    void *val, size_t val_count);
int lkpi_regmap_bulk_write(struct regmap *map, unsigned int reg,
    const void *val, size_t val_count);

struct regmap {
	uint8_t __iomem	*base;
	int		reg_stride;
	int		val_bytes;
	unsigned int	max_register;
};

static void
lkpi_devm_regmap_release(struct device *dev __unused, void *res)
{
	struct regmap *map = *(struct regmap **)res;

	if (map != NULL)
		kfree(map);
}

struct regmap *
lkpi_devm_regmap_init_mmio(struct device *dev, void __iomem *regs,
    const struct regmap_config *config)
{
	struct regmap *map;
	struct regmap **devres;

	if (dev == NULL || regs == NULL || config == NULL)
		return (ERR_PTR(-EINVAL));

	map = kmalloc(sizeof(*map), GFP_KERNEL);
	if (map == NULL)
		return (ERR_PTR(-ENOMEM));

	map->base = (uint8_t __iomem *)regs;
	map->reg_stride = config->reg_stride ? config->reg_stride : 4;
	map->val_bytes = config->val_bits / 8;
	if (map->val_bytes == 0)
		map->val_bytes = 4;
	map->max_register = config->max_register;

	devres = lkpi_devres_alloc(lkpi_devm_regmap_release,
	    sizeof(*devres), GFP_KERNEL);
	if (devres == NULL) {
		kfree(map);
		return (ERR_PTR(-ENOMEM));
	}
	*devres = map;
	lkpi_devres_add(dev, devres);

	return (map);
}

int
lkpi_regmap_read(struct regmap *map, unsigned int reg, unsigned int *val)
{

	if (map == NULL || IS_ERR(map))
		return (-EINVAL);

	switch (map->val_bytes) {
	case 1:
		*val = readb(map->base + reg);
		break;
	case 2:
		*val = readw(map->base + reg);
		break;
	case 4:
		*val = readl(map->base + reg);
		break;
	default:
		return (-EINVAL);
	}
	return (0);
}

int
lkpi_regmap_write(struct regmap *map, unsigned int reg, unsigned int val)
{

	if (map == NULL || IS_ERR(map))
		return (-EINVAL);

	switch (map->val_bytes) {
	case 1:
		writeb(val, map->base + reg);
		break;
	case 2:
		writew(val, map->base + reg);
		break;
	case 4:
		writel(val, map->base + reg);
		break;
	default:
		return (-EINVAL);
	}
	return (0);
}

int
lkpi_regmap_update_bits(struct regmap *map, unsigned int reg,
    unsigned int mask, unsigned int val)
{
	unsigned int orig, tmp;
	int error;

	error = lkpi_regmap_read(map, reg, &orig);
	if (error != 0)
		return (error);

	tmp = (orig & ~mask) | (val & mask);
	if (tmp != orig)
		return (lkpi_regmap_write(map, reg, tmp));

	return (0);
}

int
lkpi_regmap_bulk_read(struct regmap *map, unsigned int reg,
    void *val, size_t val_count)
{
	unsigned int *buf = val;
	size_t i;
	int error;

	for (i = 0; i < val_count; i++) {
		error = lkpi_regmap_read(map, reg + (i * map->reg_stride),
		    &buf[i]);
		if (error != 0)
			return (error);
	}
	return (0);
}

int
lkpi_regmap_bulk_write(struct regmap *map, unsigned int reg,
    const void *val, size_t val_count)
{
	const unsigned int *buf = val;
	size_t i;
	int error;

	for (i = 0; i < val_count; i++) {
		error = lkpi_regmap_write(map, reg + (i * map->reg_stride),
		    buf[i]);
		if (error != 0)
			return (error);
	}
	return (0);
}
