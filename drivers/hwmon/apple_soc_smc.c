// SPDX-License-Identifier: GPL-2.0-only OR MIT
/*
 * Apple SoC SMC Hw Monitoring module
 * Copyright The Asahi Linux Contributors
 */

#include <linux/ctype.h>
#include <linux/minmax.h>
#include <linux/of.h>
#include <linux/of_platform.h>
#include <linux/device.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/mfd/core.h>
#include <linux/mfd/macsmc.h>
#include <linux/hwmon.h>
#include <linux/hwmon-sysfs.h>

struct macsmc_hwmon {
	struct device *dev;
	struct apple_smc *smc;
	struct device *hwmon_dev;
	struct channel_info *power_table;
	struct channel_info *temp_table;
	struct channel_info *voltage_table;
	struct channel_info *current_table;
	u32 power_keys_cnt, temp_keys_cnt, voltage_keys_cnt, current_keys_cnt;
};

#define MAX_LABEL_LEN 30

struct channel_info {
	u32 smc_key;
	char label[MAX_LABEL_LEN];
};


static int apple_soc_smc_read_labels(struct device *dev,
			       enum hwmon_sensor_types type,
			       u32 attr, int channel, const char **str)
{
	struct macsmc_hwmon *hwmon = dev_get_drvdata(dev);

	switch (type) {
	case hwmon_temp:
		if (channel < hwmon->temp_keys_cnt)
			*str = hwmon->temp_table[channel].label;
		else
			return -EOPNOTSUPP;
		break;
	case hwmon_curr:
		if (channel < hwmon->current_keys_cnt)
			*str = hwmon->current_table[channel].label;
		else
			return -EOPNOTSUPP;
		break;
	case hwmon_in:
		if (channel < hwmon->voltage_keys_cnt)
			*str = hwmon->voltage_table[channel].label;
		else
			return -EOPNOTSUPP;
		break;
	case hwmon_power:
		if (channel < hwmon->power_keys_cnt)
			*str = hwmon->power_table[channel].label;
		else
			return -EOPNOTSUPP;
		break;
	default:
		return -EOPNOTSUPP;
	}
	return 0;
}

static int apple_soc_smc_read(struct device *dev, enum hwmon_sensor_types type,
			      u32 attr, int channel, long *val)
{
	struct macsmc_hwmon *hwmon = dev_get_drvdata(dev);
	struct apple_smc *smc = hwmon->smc;
	struct apple_smc_key_info key_info;

	int ret = 0;
	u8 vu8 = 0;
	u32 vu32 = 0;
	u64 vu64 = 0;

	switch (type) {
	case hwmon_temp:
		printk("Jeff: hwmon_temp read: channel: %d cnt: %d key: 0x%x\n", channel, hwmon->temp_keys_cnt, hwmon->temp_table[channel].smc_key);
		if ((channel < hwmon->temp_keys_cnt) && (hwmon->temp_table[channel].smc_key != 0)) {
			ret = apple_smc_get_key_info(smc, hwmon->temp_table[channel].smc_key, &key_info);
			printk("Jeff: apple_smc_get_key_info ret: %d key_info.type_code: 0x%x \n",ret,key_info.type_code);
			if (ret == 0 ) {
				if (key_info.type_code == 0x666C7420) {
					ret = apple_smc_read_f32_scaled(smc, hwmon->temp_table[channel].smc_key, &vu32, 1000);
					if (ret == 0)
						*val = vu32;
				} else if (key_info.type_code == 0x696f6674) {
					ret = apple_smc_read_u64(smc, hwmon->temp_table[channel].smc_key, &vu64);
					if (ret == 0)
						*val = vu64;
				} else
					ret = -EOPNOTSUPP;
			} else
				dev_err(dev, "Got an error on apple_smc_get_key_info\n");
		} else
			ret = -EOPNOTSUPP;
	break;

	case hwmon_curr:
		if ((channel < hwmon->current_keys_cnt) && (hwmon->current_table[channel].smc_key != 0)) {
			ret = apple_smc_read_f32_scaled(smc, hwmon->current_table[channel].smc_key, &vu32, 1000);
			if (ret == 0)
				*val = vu32;
		} else
			ret = -EOPNOTSUPP;
	break;

	case hwmon_in:
		if ((channel < hwmon->voltage_keys_cnt) && (hwmon->voltage_table[channel].smc_key != 0)) {
			ret = apple_smc_read_f32_scaled(smc, hwmon->voltage_table[channel].smc_key, &vu32, 1000);
			if (ret == 0)
				*val = vu32;
		} else
			ret = -EOPNOTSUPP;
	break;

	case hwmon_power:
		if ((channel < hwmon->power_keys_cnt) && (hwmon->power_table[channel].smc_key != 0)) {
			ret = apple_smc_get_key_info(smc, hwmon->power_table[channel].smc_key, &key_info);
			if (ret == 0) {
				if (key_info.type_code == 0x75693820) {
					ret = apple_smc_read_u8(smc, hwmon->power_table[channel].smc_key, &vu8);
					if (ret == 0)
						*val = vu8;
				} else if (key_info.type_code == 0x666C7420) {
					ret = apple_smc_read_f32_scaled(smc, hwmon->power_table[channel].smc_key, &vu32, 1000);
					if (ret == 0)
						*val = vu32;
				} else
					ret = -EOPNOTSUPP;
			} else
				dev_err(dev, "Got an error on apple_smc_get_key_info\n");
		} else
			ret = -EOPNOTSUPP;
	break;

	default:
		ret = -EOPNOTSUPP;
	}
	return ret;
}


static int  apple_soc_smc_write(struct device *dev, enum hwmon_sensor_types type,
			       u32 attr, int channel, long val)
{
		return -EOPNOTSUPP;
}

static umode_t apple_soc_smc_is_visible(const void *data,
					enum hwmon_sensor_types type,
					u32 attr, int channel)
{
	umode_t mode = 0444;
	return mode;
}


static const struct hwmon_ops apple_soc_smc_hwmon_ops = {
		.is_visible = apple_soc_smc_is_visible,
		.read = apple_soc_smc_read,
		.read_string = apple_soc_smc_read_labels,
		.write = apple_soc_smc_write,
};

static struct hwmon_chip_info apple_soc_smc_chip_info = {
		.ops = &apple_soc_smc_hwmon_ops,
		.info = NULL,
};

static int fill_key_table(struct platform_device *pdev, char *keys_node_name, u32 *keys_cnt, struct channel_info **chan_table)
{
	struct device_node *sensors_node = NULL;
	struct device_node *keys_np = NULL;
	struct device_node *key_desc = NULL;
	const char *label, *key_str;
	int i, ret;


	/* Set default values */
	*keys_cnt = 0;
	*chan_table = NULL;

	sensors_node = of_find_node_by_name(NULL, "hwmon-sensors");
	if (!sensors_node) {
		dev_err(&pdev->dev, "No hwmon-sensors entry in device tree\n");
		return 0;
	}

	keys_np = of_find_node_by_name(sensors_node, keys_node_name);
	if (!keys_np) {
		dev_err(&pdev->dev, "No %s entry in device tree\n", keys_node_name);
		return 0;
	}

	*keys_cnt = of_get_child_count(keys_np);

	dev_info(&pdev->dev, "Found %d SMC keys for %s\n", *keys_cnt, keys_node_name);

	if (*keys_cnt > 0) {
		*chan_table = devm_kzalloc(&pdev->dev, sizeof(struct channel_info)*(*keys_cnt), GFP_KERNEL);
		if (!(*chan_table)) {
			of_node_put(keys_np);
			return -ENOMEM;
		}

		i = 0;
		for_each_child_of_node(keys_np, key_desc) {

			dev_info(&pdev->dev, "Node child loop: %s\n", of_node_full_name(key_desc));

			ret = of_property_read_string(key_desc, "key", &key_str);
			if (ret != 0)
				dev_err(&pdev->dev, "Missing key property for %s\n", of_node_full_name(key_desc));
			else {
				(*chan_table)[i].smc_key = _SMC_KEY(key_str);
				dev_info(&pdev->dev, "Found SMC key: %s\n", key_str);
			}

			ret = of_property_read_string(key_desc, "label", &label);
			if (ret != 0) {
				dev_err(&pdev->dev, "Missing label property for %s. Using node name.\n", of_node_full_name(key_desc));
				strncpy((*chan_table)[i].label, of_node_full_name(key_desc), min(strlen(of_node_full_name(key_desc)), (size_t)MAX_LABEL_LEN-1));
			} else {
				strncpy((*chan_table)[i].label, label, min(strlen(label), (size_t)MAX_LABEL_LEN-1));
				dev_info(&pdev->dev, "Found label: %s\n", label);
			}

			i += 1;
		}
	}
	of_node_put(keys_np);
	return 0;
}

static void fill_config_entries(u32 *config, u32 flags, u32 cnt)
{
	int i;

	for (i = 0; i < cnt; i += 1)
		config[i] = flags;

	config[i] = 0;
}

static int apple_soc_smc_hwmon_probe(struct platform_device *pdev)
{
	struct apple_smc *smc = dev_get_drvdata(pdev->dev.parent);
	struct device *dev = &pdev->dev;
	struct macsmc_hwmon *smc_hwmon;
	struct hwmon_channel_info **apple_soc_smc_info;
	int index, offset, n_chan_info_ptr;
	unsigned int size;

	if (!smc) {
		dev_err(&pdev->dev, "No smc device pointer from parent device (check device tree)\n");
		return -ENODEV;
	}

	smc_hwmon = devm_kzalloc(&pdev->dev, sizeof(*smc_hwmon), GFP_KERNEL);
	if (!smc_hwmon)
		return -ENOMEM;

	smc_hwmon->dev = &pdev->dev;
	smc_hwmon->smc = smc;

	if (fill_key_table(pdev, "pwr_keys", &smc_hwmon->power_keys_cnt, &smc_hwmon->power_table) != 0) {
		dev_err(&pdev->dev, "Failed to get pwr_keys property\n");
		return -ENODEV;
	}

	if (fill_key_table(pdev, "temp_keys", &smc_hwmon->temp_keys_cnt, &smc_hwmon->temp_table) != 0) {
		dev_err(&pdev->dev, "Failed to get temp_keys property\n");
		return -ENODEV;
	}

	if (fill_key_table(pdev, "voltage_keys", &smc_hwmon->voltage_keys_cnt, &smc_hwmon->voltage_table) != 0) {
		dev_err(&pdev->dev, "Failed to get voltage_keys property\n");
		return -ENODEV;
	}

	if (fill_key_table(pdev, "current_keys", &smc_hwmon->current_keys_cnt, &smc_hwmon->current_table) != 0) {
		dev_err(&pdev->dev, "Failed to get current_keys property\n");
		return -ENODEV;
	}

	/* Building the channel infos needed: 1 chip, then temp, input(voltage), current, power
	 *	The chip info is a list of null terminated pointer to channel_info struct, so we need 5 pointers for chip, temp, input,
	 *	current and power plus a null termination, so a total of 6. This number could be a parameter coming from the DT.
	 *	Then, we need one channel info struct per type, so another 5 channel_info struct, and then we need the u32 config
	 *	entries for each type of channel, also null terminated: 1 config for chip plus a null entry, 40 entries for power plus a
	 *	null entry, and so on.
	 */

	size = 2*sizeof(struct hwmon_channel_info *) + sizeof(struct hwmon_channel_info) + 2*sizeof(u32); // 2 pointers for chip and null entry
	n_chan_info_ptr =2;

	if (smc_hwmon->power_keys_cnt > 0) {
		size += sizeof(struct hwmon_channel_info *) + sizeof(struct hwmon_channel_info) + sizeof(u32)*(smc_hwmon->power_keys_cnt+1);
		n_chan_info_ptr += 1;
	}
	if (smc_hwmon->voltage_keys_cnt > 0) {
		size += sizeof(struct hwmon_channel_info *) + sizeof(struct hwmon_channel_info) + sizeof(u32)*(smc_hwmon->voltage_keys_cnt+1);
		n_chan_info_ptr += 1;
	}
	if (smc_hwmon->current_keys_cnt > 0) {
		size += sizeof(struct hwmon_channel_info *) + sizeof(struct hwmon_channel_info) + sizeof(u32)*(smc_hwmon->current_keys_cnt+1);
		n_chan_info_ptr += 1;
	}
	if (smc_hwmon->temp_keys_cnt > 0) {
		size += sizeof(struct hwmon_channel_info *) + sizeof(struct hwmon_channel_info) + sizeof(u32)*(smc_hwmon->temp_keys_cnt+1);
		n_chan_info_ptr += 1;
	}


	apple_soc_smc_info = devm_kzalloc(&pdev->dev, size, GFP_KERNEL);
	if (!apple_soc_smc_info)
		return -ENOMEM;

	/* Filling Chip info entries (type and config)*/
	/* Set the chip info entry to point after the channel info pointer list and the null entry */
	apple_soc_smc_info[0] = (struct hwmon_channel_info *)(apple_soc_smc_info + n_chan_info_ptr); // 1 for chip info, 1 for power info, 1 for voltage, 1 for current, 1 for temp and 1 null entry so + 6

	/* Set the channel info type and pointer to the config list just after */
	apple_soc_smc_info[0]->type = hwmon_chip;
	apple_soc_smc_info[0]->config = (u32 *)(apple_soc_smc_info[0] + 1);

	fill_config_entries((u32 *)apple_soc_smc_info[0]->config, HWMON_C_REGISTER_TZ, 1);

	index = 1;
	offset = 2;

	if (smc_hwmon->power_keys_cnt > 0) {
		/* Filling Power sensors info entries (channel and configs)*/
		/* Set the next channel info pointer just after the previous entry ie config + 3 in this case */
		apple_soc_smc_info[index] = (struct hwmon_channel_info *)(apple_soc_smc_info[index-1]->config + offset);

		/* Set the channel info type and pointer to the config list just after */
		apple_soc_smc_info[index]->type = hwmon_power;
		apple_soc_smc_info[index]->config = (u32 *)(apple_soc_smc_info[index] + 1);

		fill_config_entries((u32 *)apple_soc_smc_info[index]->config, HWMON_P_INPUT|HWMON_P_LABEL, smc_hwmon->power_keys_cnt);

		index += 1;
		offset = smc_hwmon->power_keys_cnt+1;
	}

	if (smc_hwmon->voltage_keys_cnt > 0) {
		/* Filling Voltage(Input) sensors info entries (channel and config)*/
		/* Set the next channel info pointer just after the previous entry ie config + smc_hwmon->power_keys_cnt + 1 for null +1 in this case */
		apple_soc_smc_info[index] = (struct hwmon_channel_info *)(apple_soc_smc_info[index-1]->config + offset);

		/* Set the channel info type and pointer to the config list just after */
		apple_soc_smc_info[index]->type = hwmon_in;
		apple_soc_smc_info[index]->config = (u32 *)(apple_soc_smc_info[index] + 1);

		fill_config_entries((u32 *)apple_soc_smc_info[index]->config, HWMON_I_INPUT|HWMON_I_LABEL, smc_hwmon->voltage_keys_cnt);

		index += 1;
		offset = smc_hwmon->voltage_keys_cnt+1;
	}

	if (smc_hwmon->current_keys_cnt > 0) {
		/* Filling Current sensors info entries (channel and config)*/
		/* Set the next channel info pointer just after the previous entry ie config + smc_hwmon->voltage_keys_cnt + 1 for null +1 in this case */
		apple_soc_smc_info[index] = (struct hwmon_channel_info *)(apple_soc_smc_info[index-1]->config + offset);

		/* Set the channel info type and pointer to the config list just after */
		apple_soc_smc_info[index]->type = hwmon_curr;
		apple_soc_smc_info[index]->config = (u32 *)(apple_soc_smc_info[index] + 1);

		fill_config_entries((u32 *)apple_soc_smc_info[index]->config, HWMON_C_INPUT|HWMON_C_LABEL, smc_hwmon->current_keys_cnt);

		index += 1;
		offset = smc_hwmon->current_keys_cnt+1;
	}

	if (smc_hwmon->temp_keys_cnt > 0) {
		/* Filling Temp sensors info entries (channel and config)*/
		/* Set the next channel info pointer just after the previous entry ie config + smc_hwmon->voltage_keys_cnt + 1 for null +1 in this case */

		apple_soc_smc_info[index] = (struct hwmon_channel_info *)(apple_soc_smc_info[index-1]->config + offset);

		/* Set the channel info type and pointer to the config list just after */
		apple_soc_smc_info[index]->type = hwmon_temp;
		apple_soc_smc_info[index]->config = (u32 *)(apple_soc_smc_info[index] + 1);

		fill_config_entries((u32 *)apple_soc_smc_info[index]->config, HWMON_T_INPUT|HWMON_T_LABEL, smc_hwmon->temp_keys_cnt);

		index += 1;
	}

	/* This is a null terminated pointer list, so last entry must be set to NULL */
	apple_soc_smc_info[index] = NULL;

	/* This table must be set as the chip info entry to be register */
	apple_soc_smc_chip_info.info = (const struct hwmon_channel_info **)apple_soc_smc_info;

	smc_hwmon->hwmon_dev = devm_hwmon_device_register_with_info(&pdev->dev,
				"apple_soc_smc_hwmon", smc_hwmon,
				&apple_soc_smc_chip_info, NULL);
	if (IS_ERR(smc_hwmon->hwmon_dev))
		return dev_err_probe(dev, PTR_ERR(smc_hwmon->hwmon_dev),
				     "failed to register hwmon device\n");

	return 0;
}


static struct platform_driver apple_soc_smc_hwmon_driver = {
	.probe = apple_soc_smc_hwmon_probe,
	.driver = {
		.name = "macsmc-hwmon",
		.owner = THIS_MODULE,
	},
};

module_platform_driver(apple_soc_smc_hwmon_driver);

MODULE_DESCRIPTION("Apple SoC SMC Hwmon driver");
MODULE_AUTHOR("Jean-Francois Bortolotti <jeff@borto.fr>");
MODULE_LICENSE("GPL");

