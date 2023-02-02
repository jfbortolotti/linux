// SPDX-License-Identifier: GPL-2.0-only OR MIT
/*
 * Apple SoC SMC Hw Monitoring module
 * Copyright The Asahi Linux Contributors
 */

#include <linux/ctype.h>
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
	u32 power_keys_cnt,temp_keys_cnt,voltage_keys_cnt,current_keys_cnt;
};

static u32 apple_soc_smc_float_to_int(u32 flt)
{
	unsigned int sign, exp, mant;
	unsigned long val;
	int i, b;
	s32 result;

	sign = flt>>31;
	exp = flt>>23;
	mant = flt<<9>>9;

	result = 0;
	val = 0;
	if (exp == 0 && mant != 0) {
		for (i = 22; i >= 0; i -= 1) {
			b = (mant&(1<<i))>>i;
			val += b*(1000000000>>(23-i));
		}
		if (exp > 127)
			result = (val<<(exp-127))/1000000;
		else
			result = (val>>(127-exp))/1000000;
	} else if (!(exp == 0 && mant == 0)) {
		for (i = 22; i >= 0; i -= 1) {
			b = (mant&(1<<i))>>i;
			val += b*(1000000000>>(23-i));
		}
		if (exp > 127)
			result = ((val+1000000000)<<(exp-127))/1000000;
		else
			result = ((val+1000000000)>>(127-exp))/1000000;
	}

	if (sign == 1)
		result *= -1;

	return result;
}


struct channel_info {
	u32 smc_key;
	char label[60];
};


static int apple_soc_smc_read_labels(struct device *dev,
			       enum hwmon_sensor_types type,
			       u32 attr, int channel, const char **str)
{
	struct macsmc_hwmon *hwmon = dev_get_drvdata(dev);

	switch (type) {
	case hwmon_temp:
		if (channel < hwmon->temp_keys_cnt) {
			*str = hwmon->temp_table[channel].label;
		} else {
			return -EOPNOTSUPP;
		}
		break;
	case hwmon_curr:
		if (channel < hwmon->current_keys_cnt) {
			*str = hwmon->current_table[channel].label;
		} else {
			return -EOPNOTSUPP;
		}
		break;
	case hwmon_in:
		if (channel < hwmon->voltage_keys_cnt) {
			*str = hwmon->voltage_table[channel].label;
		} else {
			return -EOPNOTSUPP;
		}
		break;
	case hwmon_power:
		if (channel < hwmon->power_keys_cnt) {
			*str = hwmon->power_table[channel].label;
		} else {
			return -EOPNOTSUPP;
		}
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
	u32 vu32 = 0;
	u8 vu8 = 0;

	switch (type) {
		case hwmon_temp:
			if (channel <= hwmon->temp_keys_cnt) {
				ret = apple_smc_read_u32(smc, hwmon->temp_table[channel].smc_key, &vu32);
				if (ret == 0)
					*val = apple_soc_smc_float_to_int(vu32);
			} else
				ret = -EOPNOTSUPP;
		break;

		case hwmon_curr:
			if (channel < hwmon->temp_keys_cnt) {
				ret = apple_smc_read_u32(smc, hwmon->current_table[channel].smc_key, &vu32);
				if (ret == 0)
					*val = apple_soc_smc_float_to_int(vu32);
			} else
				ret = -EOPNOTSUPP;
		break;

		case hwmon_in:
			if (channel < hwmon->voltage_keys_cnt) {
				ret = apple_smc_read_u32(smc, hwmon->voltage_table[channel].smc_key, &vu32);
				if (ret == 0)
					*val = apple_soc_smc_float_to_int(vu32);
			} else
				ret = -EOPNOTSUPP;
		break;

		case hwmon_power:
			if (channel < hwmon->power_keys_cnt) {
				ret = apple_smc_get_key_info(smc, hwmon->power_table[channel].smc_key, &key_info);
				if (ret == 0) {
					if (key_info.type_code == 0x75693820){
						ret = apple_smc_read_u8(smc, hwmon->power_table[channel].smc_key, &vu8);
						if (ret == 0)
							*val = vu8;
					} else if (key_info.type_code == 0x666C7420) {
						ret = apple_smc_read_u32(smc, hwmon->power_table[channel].smc_key, &vu32);
						if (ret == 0)
							*val = apple_soc_smc_float_to_int(vu32);
					} else
						ret = -EOPNOTSUPP;
				} else
					printk("Got an error on apple_smc_get_key_info \n");
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

static int fill_key_table(struct platform_device *pdev, char *prop,u32 *keys_cnt,struct channel_info **chan_table,char *label)
{
	struct device_node *np = pdev->dev.of_node;
	const char * keys;
	const char * key;
	int size,i;

	keys = of_get_property(np, prop, &size);
	if (!keys) {
		dev_err(&pdev->dev, "Failed to get %s property\n",prop);
		return -ENODEV;
	}

	/* Building the Power channel_info list */
	*keys_cnt = size/5; // each key is 4 characters + null string termination => 5
	*chan_table = devm_kzalloc(&pdev->dev, sizeof(struct channel_info)*(*keys_cnt), GFP_KERNEL);
	if (!(*chan_table))
		return -ENOMEM;

	printk("*chan_table: %p size:%ld\n",*chan_table,sizeof(struct channel_info)*(*keys_cnt));

	/* Filling the channel_info list with Power keys */
	key = keys;
	for(i=0;i<*keys_cnt;i+=1) {

		(*chan_table)[i].smc_key = _SMC_KEY(key);
		strcpy((*chan_table)[i].label,label);
		strcat((*chan_table)[i].label," (");
		strcat((*chan_table)[i].label,key);
		strcat((*chan_table)[i].label,")");

		key = key + strlen(key)+1;

	}

	return 0;
}

static int apple_soc_smc_hwmon_probe(struct platform_device *pdev)
{
	struct apple_smc *smc = dev_get_drvdata(pdev->dev.parent);
	struct device *dev = &pdev->dev;
	struct macsmc_hwmon *smc_hwmon;
	struct hwmon_channel_info **apple_soc_smc_info;
	int  i;
	struct hwmon_channel_info * chan_info_p;
	u32 *config ;
	struct apple_smc_key_info key_info;
	int ret;

	printk("Jeff: smc:%p\n",smc);
	dump_stack();

	if (!smc){
		dev_err(&pdev->dev, "No smc device pointer from parent device (check device tree) \n");
		return -ENODEV;
	}
	pdev->dev.of_node = of_find_node_by_name(NULL, "hwmon-sensors");

	smc_hwmon = devm_kzalloc(&pdev->dev, sizeof(*smc_hwmon), GFP_KERNEL);
	if (!smc_hwmon)
		return -ENOMEM;

	smc_hwmon->dev = &pdev->dev;
	smc_hwmon->smc = smc;

	ret = apple_smc_get_key_info(smc,SMC_KEY(PBLR), &key_info);
	printk("ret: %d key_info: %x\n",ret,key_info.type_code);

	if (fill_key_table(pdev,"pwr_keys",&smc_hwmon->power_keys_cnt,&smc_hwmon->power_table,"Power") != 0) {
		dev_err(&pdev->dev, "Failed to get pwr_keys property\n");
		return -ENODEV;
	}

	if (fill_key_table(pdev,"temp_keys",&smc_hwmon->temp_keys_cnt,&smc_hwmon->temp_table,"Temp") != 0) {
		dev_err(&pdev->dev, "Failed to get temp_keys property\n");
		return -ENODEV;
	}

	if (fill_key_table(pdev,"voltage_keys",&smc_hwmon->voltage_keys_cnt,&smc_hwmon->voltage_table,"Voltage") != 0) {
		dev_err(&pdev->dev, "Failed to get voltage_keys property\n");
		return -ENODEV;
	}

	if (fill_key_table(pdev,"current_keys",&smc_hwmon->current_keys_cnt,&smc_hwmon->current_table,"Current") != 0) {
		dev_err(&pdev->dev, "Failed to get current_keys property\n");
		return -ENODEV;
	}

	/* Building the channel infos needed: 1 chip, then temp, input(voltage), current, power
		The chip info is a list of null terminated pointer to channel_info struct, so we need 5 pointers for chip, temp, input, current and power plus a null termination, so a total of 6. This number could be a parameter coming from the DT.
		Then, we need one channel info struct per type, so another 5 channel_info struct, and then we need the u32 config entries for each
		type of channel, also null terminated: 1 config for chip plus a null entry, 40 entries for power plus a null entry, and so on.
	*/

	apple_soc_smc_info = (struct hwmon_channel_info **)devm_kzalloc(&pdev->dev, 3*sizeof(struct hwmon_channel_info *) + sizeof(struct hwmon_channel_info)*(1+1)+sizeof(u32)*(2+smc_hwmon->power_keys_cnt+1+smc_hwmon->voltage_keys_cnt+1+smc_hwmon->current_keys_cnt+1+smc_hwmon->temp_keys_cnt+1), GFP_KERNEL);
	if (!apple_soc_smc_info)
		return -ENOMEM;
	
	/* Set the chip info entry to point after the channel info pointer list and the null entry */
	chan_info_p = (struct hwmon_channel_info *)apple_soc_smc_info + 6; // 1 for chip info, 1 for power info, 1 for voltage, 1 for current, 1 for temp and 1 null entry so + 6

	/* Set the config for this entry just at the end of the chan_info struct */
	config = (u32 *)(chan_info_p + 1);
	/* Set the config entries to their value. Here for the chip info and the null entry */
	config[0] = HWMON_C_REGISTER_TZ;
	config[1] = 0; // Null terminated config list

	/* Set the channel info value and pointer to the config list for the chip */
	chan_info_p->type = hwmon_chip;
	chan_info_p->config = config;

	/* Set this channel info struct in the chip info struct */
	apple_soc_smc_info[0] = chan_info_p;

	/* Set the next channel info pointer just after the previous entry ie config + 3 in this case */
	chan_info_p = ( struct hwmon_channel_info *) (config + 3); 

	/* Set the config for this entry just at the end of the chan_info struct */
	config = (u32 *)(chan_info_p + 1);

	/* Set the power config value for each power entry */
	for(i=0;i<smc_hwmon->power_keys_cnt;i+=1) {
		config[i] = HWMON_P_INPUT|HWMON_P_LABEL;
	}
	config[i] = 0; // Null terminated config list

	/* Set the channel info value and pointer to the config list for the power entry */
	chan_info_p->type = hwmon_power;
	chan_info_p->config = config;

	/* Set this channel info struct in the chip info struct */
	apple_soc_smc_info[1] = chan_info_p;

	/* Set the next channel info pointer just after the previous entry ie config + smc_hwmon->power_keys_cnt + 1 for null +1 in this case */
	chan_info_p = ( struct hwmon_channel_info *) (config + smc_hwmon->power_keys_cnt+1+1); 

	/* Set the config for this entry just at the end of the chan_info struct */
	config = (u32 *)(chan_info_p + 1);

	/* Set the power config value for each power entry */
	for(i=0;i<smc_hwmon->voltage_keys_cnt;i+=1) {
		config[i] = HWMON_I_INPUT|HWMON_I_LABEL;
	}
	config[i] = 0; // Null terminated config list

	/* Set the channel info value and pointer to the config list for the power entry */
	chan_info_p->type = hwmon_in;
	chan_info_p->config = config;

	/* Set this channel info struct in the chip info struct */
	apple_soc_smc_info[2] = chan_info_p;

	/* Set the next channel info pointer just after the previous entry ie config + smc_hwmon->voltage_keys_cnt + 1 for null +1 in this case */
	chan_info_p = ( struct hwmon_channel_info *) (config + smc_hwmon->voltage_keys_cnt+1+1); 

	/* Set the config for this entry just at the end of the chan_info struct */
	config = (u32 *)(chan_info_p + 1);

	/* Set the power config value for each power entry */
	for(i=0;i<smc_hwmon->current_keys_cnt;i+=1) {
		config[i] = HWMON_C_INPUT|HWMON_C_LABEL;
	}
	config[i] = 0; // Null terminated config list

	/* Set the channel info value and pointer to the config list for the power entry */
	chan_info_p->type = hwmon_curr;
	chan_info_p->config = config;

	/* Set this channel info struct in the chip info struct */
	apple_soc_smc_info[3] = chan_info_p;

	/* Set the next channel info pointer just after the previous entry ie config + smc_hwmon->voltage_keys_cnt + 1 for null +1 in this case */
	chan_info_p = ( struct hwmon_channel_info *) (config + smc_hwmon->current_keys_cnt+1+1); 

	/* Set the config for this entry just at the end of the chan_info struct */
	config = (u32 *)(chan_info_p + 1);

	/* Set the power config value for each power entry */
	for(i=0;i<smc_hwmon->temp_keys_cnt;i+=1) {
		config[i] = HWMON_T_INPUT|HWMON_T_LABEL;
	}
	config[i] = 0; // Null terminated config list

	/* Set the channel info value and pointer to the config list for the power entry */
	chan_info_p->type = hwmon_temp;
	chan_info_p->config = config;

	/* Set this channel info struct in the chip info struct */
	apple_soc_smc_info[4] = chan_info_p;

	/* This is a null terminated pointer list, so last entry must be set to NULL */
	apple_soc_smc_info[5] = NULL;

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

