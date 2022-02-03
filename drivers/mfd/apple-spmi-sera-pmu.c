// SPDX-License-Identifier: GPL-2.0
/*
 * Apple Sera PMU over SPMI device driver
 *
 * Copyright The Asahi Linux Contributors
 *
 * Inspired by:
 *		Correllium support Copyright (C) 2021 Corellium LLC
 *		hi6421-spmi-pmic.c
 *			Copyright (c) 2013 Linaro Ltd.
 *			Copyright (c) 2011 Hisilicon.
 *			Copyright (c) 2020-2021 Huawei Technologies Co., Ltd.
 */

#include <linux/bits.h>
#include <linux/module.h>
#include <linux/of_platform.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>
#include <linux/spmi.h>


static const struct regmap_config regmap_config = {
	.reg_bits	= 16,
	.val_bits	= BITS_PER_BYTE,
	.max_register	= 0xffff,
	.fast_io	= true
};

static int apple_spmi_sera_pmu_probe(struct spmi_device *sdev)
{
	struct device *dev = &sdev->dev;
	struct regmap *regmap;
	int ret;

	regmap = devm_regmap_init_spmi_ext(sdev, &regmap_config);
	if (IS_ERR(regmap)){
		dev_err_probe(dev, PTR_ERR(regmap), "Failed to initialize regmap for spmi sera pmu.\n");
		return PTR_ERR(regmap);
	}

	dev_set_drvdata(&sdev->dev, regmap);

	ret = devm_of_platform_populate(&sdev->dev);
	if (ret < 0)
		dev_err(dev, "Failed to add child devices: %d\n", ret);

	return ret;
}

static const struct of_device_id sera_pmu_id_table[] = {
	{ .compatible = "apple,spmi-sera-pmu" },
	{}
};
MODULE_DEVICE_TABLE(of, sera_pmu_id_table);

static struct spmi_driver apple_spmi_sera_pmu_driver = {
	.driver = {
		.name	= "apple-spmi-sera-pmu",
		.of_match_table = sera_pmu_id_table,
	},
	.probe	= apple_spmi_sera_pmu_probe,
};
module_spmi_driver(apple_spmi_sera_pmu_driver);

MODULE_AUTHOR("Jean-Francois Bortolotti <jeff@borto.fr>");
MODULE_DESCRIPTION("Apple SERA PMU SPMI driver");
MODULE_LICENSE("GPL");
