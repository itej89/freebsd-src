/*-
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * Copyright (c) 2026 Tej Kiran
 *
 * LinuxKPI regmap shim for MMIO-based drivers.
 * Provides register abstraction over readl/writel with configurable
 * register stride and width. Caching is not implemented.
 */

#ifndef _LINUXKPI_LINUX_REGMAP_H
#define	_LINUXKPI_LINUX_REGMAP_H

#include <linux/device.h>
#include <linux/types.h>
#include <linux/err.h>
#include <linux/io.h>

struct regmap;
struct regmap_config;

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

#define	devm_regmap_init_mmio		lkpi_devm_regmap_init_mmio
#define	devm_regmap_init_mmio_clk	lkpi_devm_regmap_init_mmio_clk
#define	regmap_read			lkpi_regmap_read
#define	regmap_write			lkpi_regmap_write
#define	regmap_update_bits		lkpi_regmap_update_bits
#define	regmap_bulk_read		lkpi_regmap_bulk_read
#define	regmap_bulk_write		lkpi_regmap_bulk_write

struct reg_field {
	unsigned int	reg;
	unsigned int	lsb;
	unsigned int	msb;
};

#define	REG_FIELD(_reg, _lsb, _msb) {			\
	.reg = (_reg),					\
	.lsb = (_lsb),					\
	.msb = (_msb),					\
}

struct regmap_config {
	int		reg_bits;
	int		val_bits;
	int		reg_stride;
	unsigned int	max_register;
	bool		(*writeable_reg)(struct device *dev, unsigned int reg);
	bool		(*readable_reg)(struct device *dev, unsigned int reg);
	bool		(*volatile_reg)(struct device *dev, unsigned int reg);
};

struct reg_sequence {
	unsigned int	reg;
	unsigned int	def;
	unsigned int	delay_us;
};

static inline struct regmap *
lkpi_devm_regmap_init_mmio_clk(struct device *dev, const char *clk_id,
    void __iomem *regs, const struct regmap_config *config)
{
	return (lkpi_devm_regmap_init_mmio(dev, regs, config));
}

static inline int
regmap_register_patch(struct regmap *map, const struct reg_sequence *regs,
    int num_regs)
{
	int i, error;

	for (i = 0; i < num_regs; i++) {
		error = lkpi_regmap_write(map, regs[i].reg, regs[i].def);
		if (error != 0)
			return (error);
		if (regs[i].delay_us > 0)
			DELAY(regs[i].delay_us);
	}
	return (0);
}

#endif /* _LINUXKPI_LINUX_REGMAP_H */
