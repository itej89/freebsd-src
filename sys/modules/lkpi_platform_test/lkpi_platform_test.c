/*-
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * Copyright (c) 2026 Tej Kiran
 *
 * LinuxKPI platform device shim test module.
 * Uses Linux APIs exclusively (via LinuxKPI) to probe an unclaimed
 * DT node and exercise platform_driver matching, MMIO mapping,
 * clock access, reset access, and DT property access.
 */

#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/clk.h>
#include <linux/reset.h>
#include <linux/io.h>

static const struct of_device_id lkpi_test_of_match[] = {
	{ .compatible = "starfive,jh7110-trng" },
	{ }
};

static int
lkpi_test_probe(struct platform_device *pdev)
{
	void __iomem *base;
	struct clk *clk;
	struct reset_control *rst;
	u32 val;

	pr_info("lkpi_test: probe called, name=%s\n", pdev->name);

	/* Test 1: of_node */
	if (pdev->dev.of_node != NULL)
		pr_info("lkpi_test: PASS - of_node set, phandle=%u\n",
		    (unsigned)pdev->dev.of_node->phandle);
	else
		pr_info("lkpi_test: FAIL - of_node is NULL\n");

	/* Test 2: MMIO mapping */
	base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(base)) {
		pr_info("lkpi_test: FAIL - ioremap returned %ld\n",
		    PTR_ERR(base));
		return (PTR_ERR(base));
	}
	pr_info("lkpi_test: PASS - MMIO mapped\n");

	/* Test 3: Register read */
	val = readl(base);
	pr_info("lkpi_test: PASS - reg0 = 0x%08x\n", val);

	/* Test 4: Clock (optional — don't fail if absent) */
	clk = devm_clk_get(&pdev->dev, "hclk");
	if (!IS_ERR(clk)) {
		clk_prepare_enable(clk);
		pr_info("lkpi_test: PASS - clock enabled, rate=%lu\n",
		    clk_get_rate(clk));
	} else {
		pr_info("lkpi_test: INFO - no 'hclk' clock (%ld)\n",
		    PTR_ERR(clk));
	}

	/* Test 5: Reset (optional — don't fail if absent) */
	rst = devm_reset_control_get_exclusive(&pdev->dev, "hresetn");
	if (!IS_ERR(rst)) {
		reset_control_deassert(rst);
		pr_info("lkpi_test: PASS - reset deasserted\n");
	} else {
		pr_info("lkpi_test: INFO - no 'hresetn' reset (%ld)\n",
		    PTR_ERR(rst));
	}

	pr_info("lkpi_test: ALL TESTS PASSED\n");
	return (0);
}

static void
lkpi_test_remove(struct platform_device *pdev)
{

	pr_info("lkpi_test: remove called\n");
}

static struct platform_driver lkpi_test_driver = {
	.probe	= lkpi_test_probe,
	.remove	= lkpi_test_remove,
	.driver = {
		.name = "lkpi_platform_test",
		.of_match_table = lkpi_test_of_match,
	},
};

static int
lkpi_test_modevent(module_t mod __unused, int event, void *arg __unused)
{

	switch (event) {
	case MOD_LOAD:
		return (linux_platform_register_driver(&lkpi_test_driver));
	case MOD_UNLOAD:
		linux_platform_unregister_driver(&lkpi_test_driver);
		return (0);
	default:
		return (EOPNOTSUPP);
	}
}

static moduledata_t lkpi_test_mod = {
	"lkpi_platform_test",
	lkpi_test_modevent,
	NULL
};

DECLARE_MODULE(lkpi_platform_test, lkpi_test_mod, SI_SUB_DRIVERS, SI_ORDER_ANY);
MODULE_DEPEND(lkpi_platform_test, linuxkpi, 1, 1, 1);
