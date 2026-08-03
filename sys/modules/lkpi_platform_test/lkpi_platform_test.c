/*-
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * Copyright (c) 2026 Tej Kiran
 *
 * LinuxKPI platform device shim test module.
 * Exercises the full LinuxKPI platform device API surface against
 * the JH7110 TRNG (unclaimed DT node).
 */

#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/clk.h>
#include <linux/reset.h>
#include <linux/property.h>
#include <linux/io.h>

static const struct of_device_id lkpi_test_of_match[] = {
	{ .compatible = "starfive,jh7110-trng", .data = (void *)0xCAFE },
	{ }
};

static int
lkpi_test_probe(struct platform_device *pdev)
{
	void __iomem *base;
	struct clk *clk;
	struct clk_bulk_data bulk_clks[2];
	struct reset_control *rst;
	const struct of_device_id *match;
	u32 val;
	int error;

	pr_info("lkpi_test: probe called, name=%s\n", pdev->name);

	/* Test 1: of_node */
	if (pdev->dev.of_node != NULL)
		pr_info("lkpi_test: T1 PASS - of_node phandle=%u\n",
		    (unsigned)pdev->dev.of_node->phandle);
	else
		pr_info("lkpi_test: T1 FAIL - of_node is NULL\n");

	/* Test 2: of_match_device / of_device_get_match_data */
	match = of_match_device(lkpi_test_of_match, &pdev->dev);
	if (match != NULL && match->data == (void *)0xCAFE)
		pr_info("lkpi_test: T2 PASS - of_match_device found, data=0x%lx\n",
		    (unsigned long)(uintptr_t)match->data);
	else
		pr_info("lkpi_test: T2 FAIL - of_match_device returned %p\n",
		    match);

	/* Test 3: device_property_present (property.h) */
	if (device_property_present(&pdev->dev, "reg"))
		pr_info("lkpi_test: T3 PASS - device_property_present(reg)=true\n");
	else
		pr_info("lkpi_test: T3 FAIL - device_property_present(reg)=false\n");

	/* Test 4: MMIO mapping */
	base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(base)) {
		pr_info("lkpi_test: T4 FAIL - ioremap returned %ld\n",
		    PTR_ERR(base));
		return (PTR_ERR(base));
	}
	pr_info("lkpi_test: T4 PASS - MMIO mapped\n");

	/* Test 5: Register read */
	val = readl(base);
	pr_info("lkpi_test: T5 PASS - reg0 = 0x%08x\n", val);

	/* Test 6: Individual clock get */
	clk = devm_clk_get(&pdev->dev, "hclk");
	if (!IS_ERR(clk)) {
		clk_prepare_enable(clk);
		pr_info("lkpi_test: T6 PASS - clk(hclk) rate=%lu\n",
		    clk_get_rate(clk));
	} else {
		pr_info("lkpi_test: T6 INFO - clk(hclk) not found (%ld)\n",
		    PTR_ERR(clk));
	}

	/* Test 7: Bulk clock get */
	bulk_clks[0].id = "hclk";
	bulk_clks[0].clk = NULL;
	bulk_clks[1].id = "ahb";
	bulk_clks[1].clk = NULL;
	error = devm_clk_bulk_get(&pdev->dev, 2, bulk_clks);
	if (error == 0) {
		error = clk_bulk_prepare_enable(2, bulk_clks);
		if (error == 0)
			pr_info("lkpi_test: T7 PASS - bulk clocks enabled\n");
		else
			pr_info("lkpi_test: T7 FAIL - bulk enable error %d\n",
			    error);
	} else {
		pr_info("lkpi_test: T7 INFO - bulk clk_get error %d\n", error);
	}

	/* Test 8: Reset control */
	rst = devm_reset_control_get_exclusive(&pdev->dev, NULL);
	if (!IS_ERR(rst)) {
		reset_control_deassert(rst);
		pr_info("lkpi_test: T8 PASS - reset deasserted\n");
	} else {
		pr_info("lkpi_test: T8 INFO - reset not found (%ld)\n",
		    PTR_ERR(rst));
	}

	/* Test 9: of_property_read_u32 */
	if (of_property_read_u32(pdev->dev.of_node, "interrupts", &val) == 0)
		pr_info("lkpi_test: T9 PASS - interrupts = %u\n", val);
	else
		pr_info("lkpi_test: T9 INFO - no 'interrupts' property\n");

	pr_info("lkpi_test: ALL TESTS COMPLETE\n");
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
