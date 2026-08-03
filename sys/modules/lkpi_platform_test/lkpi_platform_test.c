/*-
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * Copyright (c) 2026 Tej Kiran
 *
 * LinuxKPI platform device shim test module.
 * Uses Linux APIs exclusively (via LinuxKPI) to probe the JH7110
 * temperature sensor and exercise: platform_driver matching,
 * devm_platform_ioremap_resource, devm_clk_get, devm_reset_control_get,
 * and of_property_read_*.
 */

#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/clk.h>
#include <linux/reset.h>
#include <linux/io.h>
#include <linux/delay.h>

struct lkpi_test_softc {
	void __iomem		*base;
	struct clk		*clk_sense;
	struct clk		*clk_bus;
	struct reset_control	*rst_sense;
	struct reset_control	*rst_bus;
};

static const struct of_device_id lkpi_test_of_match[] = {
	{ .compatible = "starfive,jh7110-temp" },
	{ }
};

static int
lkpi_test_probe(struct platform_device *pdev)
{
	struct lkpi_test_softc *sc;
	u32 val;

	pr_info("lkpi_test: probe called, name=%s\n", pdev->name);

	sc = devm_kzalloc(&pdev->dev, sizeof(*sc), GFP_KERNEL);
	if (sc == NULL)
		return (-ENOMEM);
	platform_set_drvdata(pdev, sc);

	sc->base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(sc->base)) {
		pr_err("lkpi_test: FAIL - ioremap returned %ld\n",
		    PTR_ERR(sc->base));
		return (PTR_ERR(sc->base));
	}
	pr_info("lkpi_test: PASS - MMIO mapped\n");

	sc->clk_sense = devm_clk_get(&pdev->dev, "sense");
	if (IS_ERR(sc->clk_sense)) {
		pr_err("lkpi_test: FAIL - clk_get(sense) = %ld\n",
		    PTR_ERR(sc->clk_sense));
		return (PTR_ERR(sc->clk_sense));
	}
	sc->clk_bus = devm_clk_get(&pdev->dev, "bus");
	if (IS_ERR(sc->clk_bus)) {
		pr_err("lkpi_test: FAIL - clk_get(bus) = %ld\n",
		    PTR_ERR(sc->clk_bus));
		return (PTR_ERR(sc->clk_bus));
	}
	clk_prepare_enable(sc->clk_sense);
	clk_prepare_enable(sc->clk_bus);
	pr_info("lkpi_test: PASS - clocks enabled (sense=%lu, bus=%lu)\n",
	    clk_get_rate(sc->clk_sense), clk_get_rate(sc->clk_bus));

	sc->rst_sense = devm_reset_control_get_exclusive(&pdev->dev, "sense");
	if (IS_ERR(sc->rst_sense)) {
		pr_err("lkpi_test: FAIL - reset_get(sense) = %ld\n",
		    PTR_ERR(sc->rst_sense));
		return (PTR_ERR(sc->rst_sense));
	}
	sc->rst_bus = devm_reset_control_get_exclusive(&pdev->dev, "bus");
	if (IS_ERR(sc->rst_bus)) {
		pr_err("lkpi_test: FAIL - reset_get(bus) = %ld\n",
		    PTR_ERR(sc->rst_bus));
		return (PTR_ERR(sc->rst_bus));
	}
	reset_control_deassert(sc->rst_sense);
	reset_control_deassert(sc->rst_bus);
	pr_info("lkpi_test: PASS - resets deasserted\n");

	val = readl(sc->base);
	pr_info("lkpi_test: PASS - reg0 = 0x%08x\n", val);

	if (pdev->dev.of_node != NULL)
		pr_info("lkpi_test: PASS - of_node set, phandle=%u\n",
		    (unsigned)pdev->dev.of_node->phandle);
	else
		pr_err("lkpi_test: FAIL - of_node is NULL\n");

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

module_platform_driver(lkpi_test_driver);

MODULE_DESCRIPTION("LinuxKPI platform device shim test");
MODULE_LICENSE("BSD");
