/*-
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * Copyright (c) 2026 Tej Kiran
 *
 * LinuxKPI platform device shim test module.
 * Exercises the full LinuxKPI platform device API surface against
 * the JH7110 TRNG (unclaimed DT node).
 */

#include "opt_platform.h"

#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/clk.h>
#include <linux/reset.h>
#include <linux/property.h>
#include <linux/regmap.h>
#include <linux/component.h>
#include <linux/io.h>

#ifdef FDT
#include <dev/pwrdom/pwrdom.h>
#endif

/* Component framework test helpers */
static bool comp_bind_called;
static bool comp_unbind_called;
static bool master_bind_called;
static bool master_unbind_called;

static int
lkpi_test_comp_bind(struct device *comp, struct device *master, void *data)
{
	comp_bind_called = true;
	return (0);
}

static void
lkpi_test_comp_unbind(struct device *comp, struct device *master, void *data)
{
	comp_unbind_called = true;
}

static const struct component_ops lkpi_test_comp_ops = {
	.bind	= lkpi_test_comp_bind,
	.unbind	= lkpi_test_comp_unbind,
};

static int
lkpi_test_master_bind(struct device *dev)
{
	int error;

	error = component_bind_all(dev, NULL);
	if (error == 0)
		master_bind_called = true;
	return (error);
}

static void
lkpi_test_master_unbind(struct device *dev)
{
	component_unbind_all(dev, NULL);
	master_unbind_called = true;
}

static const struct component_master_ops lkpi_test_master_ops = {
	.bind	= lkpi_test_master_bind,
	.unbind	= lkpi_test_master_unbind,
};

static int
lkpi_test_comp_compare(struct device *dev, void *data)
{
	return (dev == (struct device *)data);
}

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

	/* Test 10: regmap */
	{
		static const struct regmap_config rmap_cfg = {
			.reg_bits = 32,
			.val_bits = 32,
			.reg_stride = 4,
		};
		struct regmap *rmap;
		unsigned int rval;

		rmap = devm_regmap_init_mmio(&pdev->dev, base, &rmap_cfg);
		if (!IS_ERR(rmap)) {
			error = regmap_read(rmap, 0, &rval);
			if (error == 0)
				pr_info("lkpi_test: T10 PASS - regmap_read(0) = 0x%08x\n",
				    rval);
			else
				pr_info("lkpi_test: T10 FAIL - regmap_read error %d\n",
				    error);
		} else {
			pr_info("lkpi_test: T10 FAIL - regmap_init error %ld\n",
			    PTR_ERR(rmap));
		}
	}

	/* Test 11: Component framework self-test */
	{
		struct component_match *match = NULL;
		bool t11_pass = true;

		comp_bind_called = false;
		comp_unbind_called = false;
		master_bind_called = false;
		master_unbind_called = false;

		/* Register our device as a component */
		error = component_add(&pdev->dev, &lkpi_test_comp_ops);
		if (error != 0) {
			pr_info("lkpi_test: T11 FAIL - component_add error %d\n",
			    error);
			t11_pass = false;
		}

		/* Build match list that matches our own device */
		component_match_add(&pdev->dev, &match,
		    lkpi_test_comp_compare, &pdev->dev);

		/* Register as master — should trigger bind immediately */
		error = component_master_add_with_match(&pdev->dev,
		    &lkpi_test_master_ops, match);
		if (error != 0) {
			pr_info("lkpi_test: T11 FAIL - master_add error %d\n",
			    error);
			t11_pass = false;
		}

		if (t11_pass && master_bind_called && comp_bind_called)
			pr_info("lkpi_test: T11 PASS - component: add->match->bind all fired\n");
		else if (t11_pass)
			pr_info("lkpi_test: T11 FAIL - master_bind=%d comp_bind=%d\n",
			    master_bind_called, comp_bind_called);

		/* Cleanup */
		component_master_del(&pdev->dev, &lkpi_test_master_ops);
		component_del(&pdev->dev, &lkpi_test_comp_ops);

		if (t11_pass && master_unbind_called && comp_unbind_called)
			pr_info("lkpi_test: T11 PASS - component: unbind+del clean\n");
	}

	/* Test 12: Power domain framework */
#ifdef FDT
	{
		phandle_t pmu_node;
		pwrdom_t pd;
		bool enabled;

		pmu_node = OF_finddevice("/soc/power-controller");
		if (pmu_node <= 0)
			pmu_node = OF_finddevice("/soc/power-controller@17030000");

		if (pmu_node > 0) {
			/*
			 * Get GPU power domain (id=2) from the PMU.
			 * The PMU already enabled it at boot — verify
			 * the framework can query its status.
			 */
			error = pwrdom_get_by_ofw_idx(pdev->dev.bsddev,
			    pmu_node, 0, &pd);
			if (error == 0) {
				error = pwrdom_is_enabled(pd, &enabled);
				if (error == 0)
					pr_info("lkpi_test: T12 PASS - pwrdom query ok, enabled=%d\n",
					    enabled);
				else
					pr_info("lkpi_test: T12 FAIL - pwrdom_is_enabled error %d\n",
					    error);
				pwrdom_release(pd);
			} else {
				pr_info("lkpi_test: T12 INFO - pwrdom_get error %d (PMU may not have power-domains property)\n",
				    error);
			}
		} else {
			pr_info("lkpi_test: T12 INFO - PMU node not found in DT\n");
		}
	}
#endif

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
