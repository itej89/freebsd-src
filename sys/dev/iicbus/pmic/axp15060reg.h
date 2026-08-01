/*-
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * Copyright (c) 2026 Tej <itej89@github.com>
 *
 * Register definitions for X-Powers AXP15060 PMIC.
 * Based on Linux include/linux/mfd/axp20x.h and the AXP15060 datasheet.
 */

#ifndef	_AXP15060REG_H_
#define	_AXP15060REG_H_

/* Chip identification */
#define	AXP15060_STARTUP_SRC		0x00

/* Power output control registers (enable/disable regulators) */
#define	AXP15060_PWR_OUT_CTRL1		0x10	/* DCDC1-6 enable */
#define	AXP15060_PWR_OUT_CTRL2		0x11	/* ALDO1-5, BLDO1-3 enable */
#define	AXP15060_PWR_OUT_CTRL3		0x12	/* BLDO4-5, CLDO1-4, SW enable */

/* DC-DC converter voltage control */
#define	AXP15060_DCDC1_V_CTRL		0x13
#define	AXP15060_DCDC2_V_CTRL		0x14
#define	AXP15060_DCDC3_V_CTRL		0x15
#define	AXP15060_DCDC4_V_CTRL		0x16
#define	AXP15060_DCDC5_V_CTRL		0x17
#define	AXP15060_DCDC6_V_CTRL		0x18

/* LDO voltage control */
#define	AXP15060_ALDO1_V_CTRL		0x19
#define	AXP15060_ALDO2_V_CTRL		0x20
#define	AXP15060_ALDO3_V_CTRL		0x21
#define	AXP15060_ALDO4_V_CTRL		0x22
#define	AXP15060_ALDO5_V_CTRL		0x23
#define	AXP15060_BLDO1_V_CTRL		0x24
#define	AXP15060_BLDO2_V_CTRL		0x25
#define	AXP15060_BLDO3_V_CTRL		0x26
#define	AXP15060_BLDO4_V_CTRL		0x27
#define	AXP15060_BLDO5_V_CTRL		0x28
#define	AXP15060_CLDO1_V_CTRL		0x29
#define	AXP15060_CLDO2_V_CTRL		0x2a
#define	AXP15060_CLDO3_V_CTRL		0x2b
#define	AXP15060_CLDO4_V_CTRL		0x2d
#define	AXP15060_CPUSLDO_V_CTRL	0x2e

/* DCDC mode control */
#define	AXP15060_DCDC_MODE_CTRL1	0x1a
#define	AXP15060_DCDC_MODE_CTRL2	0x1b

/* Power control */
#define	AXP15060_PWR_WAKEUP_CTRL	0x31
#define	AXP15060_PWR_DISABLE_DOWN_SEQ	0x32
#define	AXP15060_PEK_KEY		0x36

/* IRQ registers */
#define	AXP15060_IRQ1_EN		0x40
#define	AXP15060_IRQ2_EN		0x41
#define	AXP15060_IRQ1_STATE		0x48
#define	AXP15060_IRQ2_STATE		0x49

/* Misc */
#define	AXP15060_OUTPUT_MONITOR		0x1e
#define	AXP15060_IRQ_PWROK_VOFF		0x1f
#define	AXP15060_CLDO4_GPIO2_MODESET	0x2c

/* Shutdown: write bit 7 of PWR_DISABLE_DOWN_SEQ (0x32) */
#define	AXP15060_POWEROFF		(1 << 7)

/* PWR_OUT_CTRL1 bits (DCDC enables) */
#define	AXP15060_DCDC1_EN		(1 << 0)
#define	AXP15060_DCDC2_EN		(1 << 1)
#define	AXP15060_DCDC3_EN		(1 << 2)
#define	AXP15060_DCDC4_EN		(1 << 3)
#define	AXP15060_DCDC5_EN		(1 << 4)
#define	AXP15060_DCDC6_EN		(1 << 5)

/* PWR_OUT_CTRL2 bits (ALDO + BLDO1-3 enables) */
#define	AXP15060_ALDO1_EN		(1 << 0)
#define	AXP15060_ALDO2_EN		(1 << 1)
#define	AXP15060_ALDO3_EN		(1 << 2)
#define	AXP15060_ALDO4_EN		(1 << 3)
#define	AXP15060_ALDO5_EN		(1 << 4)
#define	AXP15060_BLDO1_EN		(1 << 5)
#define	AXP15060_BLDO2_EN		(1 << 6)
#define	AXP15060_BLDO3_EN		(1 << 7)

/* PWR_OUT_CTRL3 bits (BLDO4-5, CLDO1-4, SW enables) */
#define	AXP15060_BLDO4_EN		(1 << 0)
#define	AXP15060_BLDO5_EN		(1 << 1)
#define	AXP15060_CLDO1_EN		(1 << 2)
#define	AXP15060_CLDO2_EN		(1 << 3)
#define	AXP15060_CLDO3_EN		(1 << 4)
#define	AXP15060_CLDO4_EN		(1 << 5)
#define	AXP15060_SW_EN			(1 << 6)

/* Voltage control masks */
#define	AXP15060_DCDC1_V_MASK		0x1f	/* 5 bits */
#define	AXP15060_DCDC2_V_MASK		0x7f	/* 7 bits */
#define	AXP15060_DCDC3_V_MASK		0x7f
#define	AXP15060_DCDC4_V_MASK		0x7f
#define	AXP15060_DCDC5_V_MASK		0x7f
#define	AXP15060_DCDC6_V_MASK		0x1f
#define	AXP15060_ALDO_V_MASK		0x1f	/* ALDOs all use 5 bits */
#define	AXP15060_BLDO_V_MASK		0x1f	/* BLDOs all use 5 bits */
#define	AXP15060_CLDO1_V_MASK		0x1f
#define	AXP15060_CLDO2_V_MASK		0x1f
#define	AXP15060_CLDO3_V_MASK		0x1f
#define	AXP15060_CLDO4_V_MASK		0x3f	/* 6 bits */
#define	AXP15060_CPUSLDO_V_MASK		0x0f	/* 4 bits */

/* Regulator IDs */
enum axp15060_reg_id {
	AXP15060_REG_DCDC1 = 0,
	AXP15060_REG_DCDC2,
	AXP15060_REG_DCDC3,
	AXP15060_REG_DCDC4,
	AXP15060_REG_DCDC5,
	AXP15060_REG_DCDC6,
	AXP15060_REG_ALDO1,
	AXP15060_REG_ALDO2,
	AXP15060_REG_ALDO3,
	AXP15060_REG_ALDO4,
	AXP15060_REG_ALDO5,
	AXP15060_REG_BLDO1,
	AXP15060_REG_BLDO2,
	AXP15060_REG_BLDO3,
	AXP15060_REG_BLDO4,
	AXP15060_REG_BLDO5,
	AXP15060_REG_CLDO1,
	AXP15060_REG_CLDO2,
	AXP15060_REG_CLDO3,
	AXP15060_REG_CLDO4,
	AXP15060_REG_CPUSLDO,
	AXP15060_REG_SW,
	AXP15060_REG_ID_MAX,
};

#endif /* _AXP15060REG_H_ */
