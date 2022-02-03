// SPDX-License-Identifier: GPL-2.0
/*
 * Apple SERA PMU RTC Driver
 *
 * Copyright The Asahi Linux Contributors
 *
 * Inspired by:
 *		OpenBSD support Copyright (c) 2021 Mark Kettenis <kettenis@openbsd.org>
 *		Correllium support Copyright (C) 2021 Corellium LLC
 */

#include <linux/mod_devicetable.h>
#include <linux/module.h>
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
	u8 value[6];
	u64 secs,rtc_time,rtc_offset;
	struct pmu_sera_rtc *rtc_dd = dev_get_drvdata(dev);

	rc = regmap_bulk_read(rtc_dd->regmap, RTC_RUNNING_TIME_ADDR,
							value, sizeof(value));
	if (rc) {
		dev_err(dev, "RTC read data register failed: error: %d\n",rc);
		return rc;
	}

	rtc_time = (((u64)value[5])<<40) | (((u64)value[4])<<32)
				| (((u64)value[3])<<24) | (((u64)value[2])<<16)
				| (((u64)value[1])<<8) | ((u64)value[0]);


	rc = regmap_bulk_read(rtc_dd->regmap, RTC_OFFSET_ADDR,
							value, sizeof(value));
	if (rc) {
		dev_err(dev, "RTC read data register failed: error: %d\n",rc);
		return rc;
	}

	rtc_offset = (((u64)value[5])<<40) | (((u64)value[4])<<32)
				| (((u64)value[3])<<24) | (((u64)value[2])<<16)
				| (((u64)value[1])<<8) | ((u64)value[0]);

	/* rtc_time is 32.16 fixed point
		rtc__offset is 32.15 fixed point */

	secs = (rtc_time+(rtc_offset<<1))>>16;

	return secs;
}

static int set_rtc(struct device *dev,u64 t)
{
	int rc,idx;
	u8 value[6];
	u64 rtc_time,new_rtc_offset;
	struct pmu_sera_rtc *rtc_dd = dev_get_drvdata(dev);

	/* Let's read current time register */
	rc = regmap_bulk_read(rtc_dd->regmap, RTC_RUNNING_TIME_ADDR,
						 value, sizeof(value));
	if (rc) {
		dev_err(dev, "RTC read data register failed: error: %d\n",rc);
		return rc;
	}

	rtc_time = (((u64)value[5])<<40) | (((u64)value[4])<<32)
				| (((u64)value[3])<<24) | (((u64)value[2])<<16)
				| (((u64)value[1])<<8) | ((u64)value[0]);

	/* rtc_time is 32.16 fixed point
		rtc__offset is 32.15 fixed point */
    new_rtc_offset = ((t<<16)-rtc_time)>>1;

	for(idx=0;idx<6;idx+=1)
		value[idx] = ((0xffull<<(idx*8)) & new_rtc_offset)>>(idx*8);

	rc = regmap_bulk_write(rtc_dd->regmap, RTC_OFFSET_ADDR,
							value, sizeof(value));
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
	{ .compatible = "apple,pmu-sera-rtc", },
	{},
};
MODULE_DEVICE_TABLE(of, apple_pmu_sera_id);

static struct platform_driver pmu_sera_rtc_driver = {
    .probe = pmu_sera_rtc_probe,
	.driver = {
		.name = "apple-pmu-sera-rtc",
        .of_match_table	= apple_pmu_sera_id,

	},
};

module_platform_driver(pmu_sera_rtc_driver);

MODULE_AUTHOR("Jean-Francois Bortolotti <jeff@borto.fr>");
MODULE_DESCRIPTION("Apple SERA PMU RTC driver");
MODULE_LICENSE("GPL");
