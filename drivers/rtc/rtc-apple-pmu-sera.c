// SPDX-License-Identifier: GPL-2.0
/*
 * Apple SERA PMU RTC Driver
 *
 * Copyright The Asahi Linux Contributors
 *
 * Inspired by:
 *		OpenBSD support Copyright (c) 2021 Mark Kettenis <kettenis@openbsd.org>
 *		Corellium support Copyright (C) 2021 Corellium LLC
 */

#include <linux/mod_devicetable.h>
#include <linux/module.h>
#include <linux/byteorder/generic.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>
#include <linux/rtc.h>

#define RTC_RUNNING_TIME_ADDR 0xd002
#define RTC_OFFSET_ADDR 0xd100

/**
 * struct pmu_sera_rtc -  rtc driver internal structure
 * @rtc:		rtc device for this driver.
 * @regmap:		regmap used to access RTC registers
 */
struct pmu_sera_rtc {
	struct rtc_device *rtc;
	struct regmap *regmap;
};

static u64 read_rtc(struct device *dev)
{
	int rc;
	u64 secs;
	u64 rtc_time = 0;
	u64 rtc_offset = 0;
	struct pmu_sera_rtc *rtc_dd = dev_get_drvdata(dev);

	rc = regmap_bulk_read(rtc_dd->regmap, RTC_RUNNING_TIME_ADDR,
							&rtc_time, 6);
	if (rc) {
		dev_err(dev, "RTC read data register failed: error: %d\n",rc);
		return rc;
	}
	//rtc_time = be64_to_cpu(rtc_time);

	rc = regmap_bulk_read(rtc_dd->regmap, RTC_OFFSET_ADDR,
							&rtc_offset, 6);
	if (rc) {
		dev_err(dev, "RTC read data register failed: error: %d\n",rc);
		return rc;
	}

	//rtc_offset = be64_to_cpu(rtc_offset);

	/* rtc_time is 32.16 fixed point
		rtc__offset is 32.15 fixed point */

	secs = (rtc_time+(rtc_offset<<1))>>16;

	return secs;
}

static int set_rtc(struct device *dev,u64 t)
{
	int rc;
	u64 rtc_time = 0;
	u64 new_rtc_offset = 0;
	struct pmu_sera_rtc *rtc_dd = dev_get_drvdata(dev);

	/* Let's read current time register */
	rc = regmap_bulk_read(rtc_dd->regmap, RTC_RUNNING_TIME_ADDR,
						 &rtc_time, 6);
	if (rc) {
		dev_err(dev, "RTC read data register failed: error: %d\n",rc);
		return rc;
	}
	//rtc_time = be64_to_cpu(rtc_time);

	/* rtc_time is 32.16 fixed point
		rtc__offset is 32.15 fixed point */
	new_rtc_offset = ((t<<16)-rtc_time)>>1;

	//new_rtc_offset = cpu_to_be64(new_rtc_offset);

	rc = regmap_bulk_write(rtc_dd->regmap, RTC_OFFSET_ADDR,
							&new_rtc_offset, 6);
	if (rc) {
		dev_err(dev, "RTC read data register failed: error: %d\n",rc);
		return rc;
	}

	return 0;
}

static int pmu_sera_get_time(struct device *dev, struct rtc_time *tm)
{
	unsigned long secs = read_rtc(dev);

	rtc_time64_to_tm(secs , tm);
	return 0;
}

static int pmu_sera_set_time(struct device *dev, struct rtc_time *tm)
{
	int rc;

	rc = set_rtc(dev,rtc_tm_to_time64(tm));
	return rc;
}

static const struct rtc_class_ops pmu_sera_rtc_ops = {
	.read_time = pmu_sera_get_time,
	.set_time = pmu_sera_set_time,
};

static int pmu_sera_rtc_probe(struct platform_device *pdev)
{
	struct pmu_sera_rtc *rtc_dd;

	rtc_dd = devm_kzalloc(&pdev->dev, sizeof(*rtc_dd), GFP_KERNEL);
	if (rtc_dd == NULL) {
		dev_err_probe(&pdev->dev, -ENOMEM, "Can't allocate device structure.\n");
		return -ENOMEM;
	}

	rtc_dd->regmap = dev_get_regmap(pdev->dev.parent, NULL);
	if (!rtc_dd->regmap) {
		dev_err_probe(&pdev->dev, -ENXIO, "Parent regmap unavailable.\n");
		return -ENXIO;
	}
#if 0
	if (IS_ERR(rtc_dd->regmap)) {
		dev_err_probe(&pdev->dev, PTR_ERR(rtc_dd->regmap), "Parent regmap unavailable.\n");
		return PTR_ERR(rtc_dd->regmap);
	}
#endif
	rtc_dd->rtc = devm_rtc_allocate_device(&pdev->dev);
	if (IS_ERR(rtc_dd->rtc)){
		dev_err_probe(&pdev->dev, PTR_ERR(rtc_dd->rtc), "Can't allocate rtc device.\n");
		return PTR_ERR(rtc_dd->rtc);
	}

	rtc_dd->rtc->ops = &pmu_sera_rtc_ops;
	rtc_dd->rtc->range_max = U64_MAX;

	platform_set_drvdata(pdev, rtc_dd);

	return devm_rtc_register_device(rtc_dd->rtc);
}

static const struct of_device_id apple_pmu_sera_id[] = {
	{ .compatible = "apple,sera-pmu-rtc", },
	{},
};
MODULE_DEVICE_TABLE(of, apple_pmu_sera_id);

static struct platform_driver pmu_sera_rtc_driver = {
    .probe = pmu_sera_rtc_probe,
	.driver = {
		.name = "apple-sera-pmu-rtc",
        .of_match_table	= apple_pmu_sera_id,

	},
};

module_platform_driver(pmu_sera_rtc_driver);

MODULE_AUTHOR("Jean-Francois Bortolotti <jeff@borto.fr>");
MODULE_DESCRIPTION("Apple SERA PMU RTC driver");
MODULE_LICENSE("GPL");
