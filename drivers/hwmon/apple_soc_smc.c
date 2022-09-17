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

/* MBA M1 Platform name: #680: RPlt = (ch8*, 0x84) b'j313_8fs' */
static const struct channel_info apple_soc_smc_power_table[] = {
	{SMC_KEY(PBLR), "Power (PBLR)"},
	{SMC_KEY(PDTR), "Power (PDTR)"},
	{SMC_KEY(PGDP), "Power (PGDP)"},
	{SMC_KEY(PHPB), "Power (PHPB)"},
	{SMC_KEY(PHPC), "Power (PHPC)"},
	{SMC_KEY(PHPM), "Power (PHPM)"},
	{SMC_KEY(PHPS), "Power (PHPS)"},
	{SMC_KEY(PKBC), "Power (PKBC)"},
	{SMC_KEY(PMVC), "Power (PMVC)"},
	{SMC_KEY(PO3R), "Power (PO3R)"},
	{SMC_KEY(PO5R), "Power (PO5R)"},
	{SMC_KEY(PP0b), "Power (PP0b)"},
	{SMC_KEY(PP0l), "Power (PP0l)"},
	{SMC_KEY(PP1b), "Power (PP1b)"},
	{SMC_KEY(PP2b), "Power (PP2b)"},
	{SMC_KEY(PP3b), "Power (PP3b)"},
	{SMC_KEY(PP3l), "Power (PP3l)"},
	{SMC_KEY(PP7b), "Power (PP7b)"},
	{SMC_KEY(PP7l), "Power (PP7l)"},
	{SMC_KEY(PP8b), "Power (PP8b)"},
	{SMC_KEY(PP9b), "Power (PP9b)"},
	{SMC_KEY(PP9l), "Power (PP9l)"},
	{SMC_KEY(PPBR), "Power (PPBR)"},
	{SMC_KEY(PPbb), "Power (PPbb)"},
	{SMC_KEY(PPeb), "Power (PPeb)"},
	{SMC_KEY(PR4b), "Power (PR4b)"},
	{SMC_KEY(PR4l), "Power (PR4l)"},
	{SMC_KEY(PR5b), "Power (PR5b)"},
	{SMC_KEY(PR6b), "Power (PR6b)"},
	{SMC_KEY(PR8l), "Power (PR8l)"},
	{SMC_KEY(PRab), "Power (PRab)"},
	{SMC_KEY(PRbl), "Power (PRbl)"},
	{SMC_KEY(PRcb), "Power (PRcb)"},
	{SMC_KEY(PRcl), "Power (PRcl)"},
	{SMC_KEY(PRdb), "Power (PRdb)"},
	{SMC_KEY(PRkl), "Power (PRkl)"},
	{SMC_KEY(PSTR), "Power (PSTR)"},
	{SMC_KEY(PW3C), "Power (PW3C)"},
	{SMC_KEY(PZl0), "Power (PZl0)"},
	{SMC_KEY(PZlF), "Power (PZlF)"},
	{SMC_KEY(Pb0f), "Power (Pb0f)"}
};

static const struct channel_info apple_soc_smc_volt_table[] = {
	{SMC_KEY(VD0R), "Voltage (VD0R)"},
	{SMC_KEY(VP0R), "Voltage (VP0R)"},
	{SMC_KEY(VP0b), "Voltage (VP0b)"},
	{SMC_KEY(VP0l), "Voltage (VP0l)"},
	{SMC_KEY(VP1b), "Voltage (VP1b)"},
	{SMC_KEY(VP2b), "Voltage (VP2b)"},
	{SMC_KEY(VP3b), "Voltage (VP3b)"},
	{SMC_KEY(VP3l), "Voltage (VP3l)"},
	{SMC_KEY(VP7b), "Voltage (VP7b)"},
	{SMC_KEY(VP7l), "Voltage (VP7l)"},
	{SMC_KEY(VP8b), "Voltage (VP8b)"},
	{SMC_KEY(VP9b), "Voltage (VP9b)"},
	{SMC_KEY(VP9l), "Voltage (VP9l)"},
	{SMC_KEY(VPbb), "Voltage (VPbb)"},
	{SMC_KEY(VPeb), "Voltage (VPeb)"},
	{SMC_KEY(VR4b), "Voltage (VR4b)"},
	{SMC_KEY(VR4l), "Voltage (VR4l)"},
	{SMC_KEY(VR5b), "Voltage (VR5b)"},
	{SMC_KEY(VR6b), "Voltage (VR6b)"},
	{SMC_KEY(VR8l), "Voltage (VR8l)"},
	{SMC_KEY(VRab), "Voltage (VRab)"},
	{SMC_KEY(VRbl), "Voltage (VRbl)"},
	{SMC_KEY(VRcb), "Voltage (VRcb)"},
	{SMC_KEY(VRcl), "Voltage (VRcl)"},
	{SMC_KEY(VRdb), "Voltage (VRdb)"},
	{SMC_KEY(VRkl), "Voltage (VRkl)"},
	{SMC_KEY(Vb0f), "Voltage (Vb0f)"},
	{SMC_KEY(Vb1f), "Voltage (Vb1f)"}
};

static const struct channel_info apple_soc_smc_curr_table[] = {
	{SMC_KEY(IBLR), "Current (IBLR)"},
	{SMC_KEY(ID0R), "Current (ID0R)"},
	{SMC_KEY(IKBC), "Current (IKBC)"},
	{SMC_KEY(IMVC), "Current (IMVC)"},
	{SMC_KEY(IO3R), "Current (IO3R)"},
	{SMC_KEY(IO5R), "Current (IO5R)"},
	{SMC_KEY(IP0b), "Current (IP0b)"},
	{SMC_KEY(IP0l), "Current (IP0l)"},
	{SMC_KEY(IP1b), "Current (IP1b)"},
	{SMC_KEY(IP2b), "Current (IP2b)"},
	{SMC_KEY(IP3b), "Current (IP3b)"},
	{SMC_KEY(IP3l), "Current (IP3l)"},
	{SMC_KEY(IP7b), "Current (IP7b)"},
	{SMC_KEY(IP7l), "Current (IP7l)"},
	{SMC_KEY(IP8b), "Current (IP8b)"},
	{SMC_KEY(IP9b), "Current (IP9b)"},
	{SMC_KEY(IP9l), "Current (IP9l)"},
	{SMC_KEY(IPBR), "Current (IPBR)"},
	{SMC_KEY(IPbb), "Current (IPbb)"},
	{SMC_KEY(IPeb), "Current (IPeb)"},
	{SMC_KEY(IR4b), "Current (IR4b)"},
	{SMC_KEY(IR4l), "Current (IR4l)"},
	{SMC_KEY(IR5b), "Current (IR5b)"},
	{SMC_KEY(IR6b), "Current (IR6b)"},
	{SMC_KEY(IR8l), "Current (IR8l)"},
	{SMC_KEY(IRab), "Current (IRab)"},
	{SMC_KEY(IRbl), "Current (IRbl)"},
	{SMC_KEY(IRcb), "Current (IRcb)"},
	{SMC_KEY(IRcl), "Current (IRcl)"},
	{SMC_KEY(IRdb), "Current (IRdb)"},
	{SMC_KEY(IRkl), "Current (IRkl)"},
	{SMC_KEY(IW3C), "Current (IW3C)"},
	{SMC_KEY(Ib0f), "Current (Ib0f)"},
	{SMC_KEY(Ib4f), "Current (Ib4f)"},
	{SMC_KEY(Ib8f), "Current (Ib8f)"}
};

static const struct channel_info apple_soc_smc_temp_table[] = {
	{SMC_KEY(TSCD), "Temp SOC CORE AREA BACKSIDE(TSCD)"},
	{SMC_KEY(TB0T), "Temp Battery TS_MAX(TB0T)"},
	{SMC_KEY(TB1T), "Temp Battery TS1(TB1T)"},
	{SMC_KEY(TB1T), "Temp Battery TS2(TB2T)"},
	{SMC_KEY(TCHP), "Temp CHARGER, BETWEEN INDUCTOR AND MOSFETS(TCHP)"},
	{SMC_KEY(TCMb), "Temp (TCMb)"},
	{SMC_KEY(TCMz), "Temp (TCMz)"},
	{SMC_KEY(TH0T), "Temp LOWERLEFTCORNEROFNANDDEVICES(TH0T)"},
	{SMC_KEY(TH0x), "Temp MaxNANDProximityTemp(TH0x)"},
	{SMC_KEY(TSCD), "Temp SOCCOREAREABACKSIDE(TOP)(TSCD)"},
	{SMC_KEY(TVS0), "Temp P1V8VDDH_THMSNS1?(TVS0)"},
	{SMC_KEY(TVS1), "Temp P1V8VDDH_THMSNS2?(TVS1)"},
	{SMC_KEY(TVSx), "Temp P1V8VDDH_THMSNSxmax(TVSx)"},
	{SMC_KEY(Tc0b), "Temp SOCAvgw/offsetClusterDieTemp(Tc0b)"},
	{SMC_KEY(Th0x), "Temp MaxNANDProximityTemp(Th0x)"},
	{SMC_KEY(Th1a), "Temp GPUTemp(Th1a)"},
	{SMC_KEY(Th2a), "Temp PCPUTemp(Th2a)"},
	{SMC_KEY(Ts1a), "Temp GPUMTRDieTemp(Ts1a)"},
	{SMC_KEY(Ts2a), "Temp PCPUMTRDieTemp(Ts2a)"},
	{SMC_KEY(TIOP), "Temp ThunderboltProximity(TIOP)"},
	{SMC_KEY(TMVR), "Temp 3.8VAONVRBETWEENPHASE2AND(TMVR)"},
	{SMC_KEY(TPMP), "Temp MASTERPMU,BETWEENBUCK0ANDBUCK1INDUCTORS(TPMP)"},
	{SMC_KEY(TPSP), "Temp SLAVEPMU,BUCK10INDUCTORPROXIMITY(TPSP)"},
	{SMC_KEY(TW0P), "Temp WLBTtemperature(TW0P)"},
	{SMC_KEY(Tc0a), "Temp (Tc0a)"},
	{SMC_KEY(Tc0x), "Temp (Tc0x)"},
	{SMC_KEY(Tc0z), "Temp (Tc0z)"},
	{SMC_KEY(Tc7a), "Temp (Tc7a)"},
	{SMC_KEY(Tc7b), "Temp (Tc7b)"},
	{SMC_KEY(Tc7x), "Temp (Tc7x)"},
	{SMC_KEY(Tc7z), "Temp (Tc7z)"},
	{SMC_KEY(Tc8a), "Temp (Tc8a)"},
	{SMC_KEY(Tc8b), "Temp (Tc8b)"},
	{SMC_KEY(Tc8x), "Temp (Tc8x)"},
	{SMC_KEY(Tc8z), "Temp (Tc8z)"},
	{SMC_KEY(Tc9a), "Temp (Tc9a)"},
	{SMC_KEY(Tc9b), "Temp (Tc9b)"},
	{SMC_KEY(Tc9x), "Temp (Tc9x)"},
	{SMC_KEY(Tc9z), "Temp (Tc9z)"},
	{SMC_KEY(Tcaa), "Temp (Tcaa)"},
	{SMC_KEY(Tcab), "Temp (Tcab)"},
	{SMC_KEY(Tcax), "Temp (Tcax)"},
	{SMC_KEY(Tcaz), "Temp (Tcaz)"},
	{SMC_KEY(Te0a), "Temp (Te0a)"},
	{SMC_KEY(Te0b), "Temp (Te0b)"},
	{SMC_KEY(Te0x), "Temp (Te0x)"},
	{SMC_KEY(Te0z), "Temp (Te0z)"},
	{SMC_KEY(Te3a), "Temp (Te3a)"},
	{SMC_KEY(Te3b), "Temp (Te3b)"},
	{SMC_KEY(Te3x), "Temp (Te3x)"},
	{SMC_KEY(Te3z), "Temp (Te3z)"},
	{SMC_KEY(Th0a), "Temp (Th0a)"},
	{SMC_KEY(Th0b), "Temp (Th0b)"},
	{SMC_KEY(Th0z), "Temp (Th0z)"},
	{SMC_KEY(Th1b), "Temp (Th1b)"},
	{SMC_KEY(Th1x), "Temp (Th1x)"},
	{SMC_KEY(Th1z), "Temp (Th1z)"},
	{SMC_KEY(Th2b), "Temp (Th2b)"},
	{SMC_KEY(Th2x), "Temp (Th2x)"},
	{SMC_KEY(Th2z), "Temp (Th2z)"},
	{SMC_KEY(Tp2a), "Temp (Tp2a)"},
	{SMC_KEY(Tp2b), "Temp (Tp2b)"},
	{SMC_KEY(Tp2x), "Temp (Tp2x)"},
	{SMC_KEY(Tp2z), "Temp (Tp2z)"},
	{SMC_KEY(Tp3a), "Temp (Tp3a)"},
	{SMC_KEY(Tp3b), "Temp (Tp3b)"},
	{SMC_KEY(Tp3x), "Temp (Tp3x)"},
	{SMC_KEY(Tp3z), "Temp (Tp3z)"},
	{SMC_KEY(Tp4a), "Temp (Tp4a)"},
	{SMC_KEY(Tp4b), "Temp (Tp4b)"},
	{SMC_KEY(Tp4x), "Temp (Tp4x)"},
	{SMC_KEY(Tp4z), "Temp (Tp4z)"},
	{SMC_KEY(Tp5a), "Temp (Tp5a)"},
	{SMC_KEY(Tp5b), "Temp (Tp5b)"},
	{SMC_KEY(Tp5x), "Temp (Tp5x)"},
	{SMC_KEY(Tp5z), "Temp (Tp5z)"},
	{SMC_KEY(Tp7a), "Temp (Tp7a)"},
	{SMC_KEY(Tp7b), "Temp (Tp7b)"},
	{SMC_KEY(Tp7x), "Temp (Tp7x)"},
	{SMC_KEY(Tp7z), "Temp (Tp7z)"},
	{SMC_KEY(Tp8a), "Temp (Tp8a)"},
	{SMC_KEY(Tp8b), "Temp (Tp8b)"},
	{SMC_KEY(Tp8x), "Temp (Tp8x)"},
	{SMC_KEY(Tp8z), "Temp (Tp8z)"},
	{SMC_KEY(Tp9a), "Temp (Tp9a)"},
	{SMC_KEY(Tp9b), "Temp (Tp9b)"},
	{SMC_KEY(Tp9x), "Temp (Tp9x)"},
	{SMC_KEY(Tp9z), "Temp (Tp9z)"},
	{SMC_KEY(Ts0a), "Temp (Ts0a)"},
	{SMC_KEY(Ts0b), "Temp (Ts0b)"},
	{SMC_KEY(Ts0x), "Temp (Ts0x)"},
	{SMC_KEY(Ts0z), "Temp (Ts0z)"},
	{SMC_KEY(Ts1b), "Temp (Ts1b)"},
	{SMC_KEY(Ts1x), "Temp (Ts1x)"},
	{SMC_KEY(Ts1z), "Temp (Ts1z)"},
	{SMC_KEY(Ts2b), "Temp (Ts2b)"},
	{SMC_KEY(Ts2x), "Temp (Ts2x)"},
	{SMC_KEY(Ts2z), "Temp (Ts2z)"},
	{SMC_KEY(TVA0), "Temp (TVA0)"},
	{SMC_KEY(TVD0), "Temp (TVD0)"}
};

static int apple_soc_smc_read_labels(struct device *dev,
			       enum hwmon_sensor_types type,
			       u32 attr, int channel, const char **str)
{
	switch (type) {
	case hwmon_temp:
		*str = apple_soc_smc_temp_table[channel].label;
		break;
	case hwmon_curr:
		*str = apple_soc_smc_curr_table[channel].label;
		break;
	case hwmon_in:
		*str = apple_soc_smc_volt_table[channel].label;
		break;
	case hwmon_power:
		*str = apple_soc_smc_power_table[channel].label;
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

	int ret = 0;
	u32 vu32 = 0;

	switch (type) {
	case hwmon_temp:
		if (channel < (sizeof(apple_soc_smc_temp_table)/sizeof(struct channel_info))) {
			ret = apple_smc_read_u32(smc, apple_soc_smc_temp_table[channel].smc_key, &vu32);
			if (ret == 0)
				*val = apple_soc_smc_float_to_int(vu32);
		} else
			ret = -EOPNOTSUPP;
	break;

	case hwmon_curr:
		if (channel < (sizeof(apple_soc_smc_curr_table)/sizeof(struct channel_info))) {
			ret = apple_smc_read_u32(smc, apple_soc_smc_curr_table[channel].smc_key, &vu32);
			if (ret == 0)
				*val = apple_soc_smc_float_to_int(vu32);
		} else
			ret = -EOPNOTSUPP;
	break;

	case hwmon_in:
		if (channel < (sizeof(apple_soc_smc_volt_table)/sizeof(struct channel_info))) {
			ret = apple_smc_read_u32(smc, apple_soc_smc_volt_table[channel].smc_key, &vu32);
			if (ret == 0)
				*val = apple_soc_smc_float_to_int(vu32);
		} else
			ret = -EOPNOTSUPP;
	break;

	case hwmon_power:
		if (channel < (sizeof(apple_soc_smc_power_table)/sizeof(struct channel_info))) {
			ret = apple_smc_read_u32(smc, apple_soc_smc_power_table[channel].smc_key, &vu32);
			if (ret == 0)
				*val = apple_soc_smc_float_to_int(vu32);
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

static const struct hwmon_channel_info *apple_soc_smc_info[] = {
		HWMON_CHANNEL_INFO(chip,
						HWMON_C_REGISTER_TZ),

		HWMON_CHANNEL_INFO(temp,
						HWMON_T_INPUT|HWMON_T_LABEL,
						HWMON_T_INPUT|HWMON_T_LABEL,
						HWMON_T_INPUT|HWMON_T_LABEL,
						HWMON_T_INPUT|HWMON_T_LABEL,
						HWMON_T_INPUT|HWMON_T_LABEL,
						HWMON_T_INPUT|HWMON_T_LABEL,
						HWMON_T_INPUT|HWMON_T_LABEL,
						HWMON_T_INPUT|HWMON_T_LABEL,
						HWMON_T_INPUT|HWMON_T_LABEL,
						HWMON_T_INPUT|HWMON_T_LABEL,
						HWMON_T_INPUT|HWMON_T_LABEL,
						HWMON_T_INPUT|HWMON_T_LABEL,
						HWMON_T_INPUT|HWMON_T_LABEL,
						HWMON_T_INPUT|HWMON_T_LABEL,
						HWMON_T_INPUT|HWMON_T_LABEL,
						HWMON_T_INPUT|HWMON_T_LABEL,
						HWMON_T_INPUT|HWMON_T_LABEL,
						HWMON_T_INPUT|HWMON_T_LABEL,
						HWMON_T_INPUT|HWMON_T_LABEL,
						HWMON_T_INPUT|HWMON_T_LABEL,
						HWMON_T_INPUT|HWMON_T_LABEL,
						HWMON_T_INPUT|HWMON_T_LABEL,
						HWMON_T_INPUT|HWMON_T_LABEL,
						HWMON_T_INPUT|HWMON_T_LABEL,
						HWMON_T_INPUT|HWMON_T_LABEL,
						HWMON_T_INPUT|HWMON_T_LABEL,
						HWMON_T_INPUT|HWMON_T_LABEL,
						HWMON_T_INPUT|HWMON_T_LABEL,
						HWMON_T_INPUT|HWMON_T_LABEL,
						HWMON_T_INPUT|HWMON_T_LABEL,
						HWMON_T_INPUT|HWMON_T_LABEL,
						HWMON_T_INPUT|HWMON_T_LABEL,
						HWMON_T_INPUT|HWMON_T_LABEL,
						HWMON_T_INPUT|HWMON_T_LABEL,
						HWMON_T_INPUT|HWMON_T_LABEL,
						HWMON_T_INPUT|HWMON_T_LABEL,
						HWMON_T_INPUT|HWMON_T_LABEL,
						HWMON_T_INPUT|HWMON_T_LABEL,
						HWMON_T_INPUT|HWMON_T_LABEL,
						HWMON_T_INPUT|HWMON_T_LABEL,
						HWMON_T_INPUT|HWMON_T_LABEL,
						HWMON_T_INPUT|HWMON_T_LABEL,
						HWMON_T_INPUT|HWMON_T_LABEL,
						HWMON_T_INPUT|HWMON_T_LABEL,
						HWMON_T_INPUT|HWMON_T_LABEL,
						HWMON_T_INPUT|HWMON_T_LABEL,
						HWMON_T_INPUT|HWMON_T_LABEL,
						HWMON_T_INPUT|HWMON_T_LABEL,
						HWMON_T_INPUT|HWMON_T_LABEL,
						HWMON_T_INPUT|HWMON_T_LABEL,
						HWMON_T_INPUT|HWMON_T_LABEL,
						HWMON_T_INPUT|HWMON_T_LABEL,
						HWMON_T_INPUT|HWMON_T_LABEL,
						HWMON_T_INPUT|HWMON_T_LABEL,
						HWMON_T_INPUT|HWMON_T_LABEL,
						HWMON_T_INPUT|HWMON_T_LABEL,
						HWMON_T_INPUT|HWMON_T_LABEL,
						HWMON_T_INPUT|HWMON_T_LABEL,
						HWMON_T_INPUT|HWMON_T_LABEL,
						HWMON_T_INPUT|HWMON_T_LABEL,
						HWMON_T_INPUT|HWMON_T_LABEL,
						HWMON_T_INPUT|HWMON_T_LABEL,
						HWMON_T_INPUT|HWMON_T_LABEL,
						HWMON_T_INPUT|HWMON_T_LABEL,
						HWMON_T_INPUT|HWMON_T_LABEL,
						HWMON_T_INPUT|HWMON_T_LABEL,
						HWMON_T_INPUT|HWMON_T_LABEL,
						HWMON_T_INPUT|HWMON_T_LABEL,
						HWMON_T_INPUT|HWMON_T_LABEL,
						HWMON_T_INPUT|HWMON_T_LABEL,
						HWMON_T_INPUT|HWMON_T_LABEL,
						HWMON_T_INPUT|HWMON_T_LABEL,
						HWMON_T_INPUT|HWMON_T_LABEL,
						HWMON_T_INPUT|HWMON_T_LABEL,
						HWMON_T_INPUT|HWMON_T_LABEL,
						HWMON_T_INPUT|HWMON_T_LABEL,
						HWMON_T_INPUT|HWMON_T_LABEL,
						HWMON_T_INPUT|HWMON_T_LABEL,
						HWMON_T_INPUT|HWMON_T_LABEL,
						HWMON_T_INPUT|HWMON_T_LABEL,
						HWMON_T_INPUT|HWMON_T_LABEL,
						HWMON_T_INPUT|HWMON_T_LABEL,
						HWMON_T_INPUT|HWMON_T_LABEL,
						HWMON_T_INPUT|HWMON_T_LABEL,
						HWMON_T_INPUT|HWMON_T_LABEL,
						HWMON_T_INPUT|HWMON_T_LABEL,
						HWMON_T_INPUT|HWMON_T_LABEL,
						HWMON_T_INPUT|HWMON_T_LABEL,
						HWMON_T_INPUT|HWMON_T_LABEL,
						HWMON_T_INPUT|HWMON_T_LABEL,
						HWMON_T_INPUT|HWMON_T_LABEL,
						HWMON_T_INPUT|HWMON_T_LABEL,
						HWMON_T_INPUT|HWMON_T_LABEL,
						HWMON_T_INPUT|HWMON_T_LABEL,
						HWMON_T_INPUT|HWMON_T_LABEL,
						HWMON_T_INPUT|HWMON_T_LABEL,
						HWMON_T_INPUT|HWMON_T_LABEL,
						HWMON_T_INPUT|HWMON_T_LABEL,
						HWMON_T_INPUT|HWMON_T_LABEL,
						HWMON_T_INPUT|HWMON_T_LABEL),

		HWMON_CHANNEL_INFO(curr,
						HWMON_C_INPUT|HWMON_C_LABEL,
						HWMON_C_INPUT|HWMON_C_LABEL,
						HWMON_C_INPUT|HWMON_C_LABEL,
						HWMON_C_INPUT|HWMON_C_LABEL,
						HWMON_C_INPUT|HWMON_C_LABEL,
						HWMON_C_INPUT|HWMON_C_LABEL,
						HWMON_C_INPUT|HWMON_C_LABEL,
						HWMON_C_INPUT|HWMON_C_LABEL,
						HWMON_C_INPUT|HWMON_C_LABEL,
						HWMON_C_INPUT|HWMON_C_LABEL,
						HWMON_C_INPUT|HWMON_C_LABEL,
						HWMON_C_INPUT|HWMON_C_LABEL,
						HWMON_C_INPUT|HWMON_C_LABEL,
						HWMON_C_INPUT|HWMON_C_LABEL,
						HWMON_C_INPUT|HWMON_C_LABEL,
						HWMON_C_INPUT|HWMON_C_LABEL,
						HWMON_C_INPUT|HWMON_C_LABEL,
						HWMON_C_INPUT|HWMON_C_LABEL,
						HWMON_C_INPUT|HWMON_C_LABEL,
						HWMON_C_INPUT|HWMON_C_LABEL,
						HWMON_C_INPUT|HWMON_C_LABEL,
						HWMON_C_INPUT|HWMON_C_LABEL,
						HWMON_C_INPUT|HWMON_C_LABEL,
						HWMON_C_INPUT|HWMON_C_LABEL,
						HWMON_C_INPUT|HWMON_C_LABEL,
						HWMON_C_INPUT|HWMON_C_LABEL,
						HWMON_C_INPUT|HWMON_C_LABEL,
						HWMON_C_INPUT|HWMON_C_LABEL,
						HWMON_C_INPUT|HWMON_C_LABEL,
						HWMON_C_INPUT|HWMON_C_LABEL,
						HWMON_C_INPUT|HWMON_C_LABEL,
						HWMON_C_INPUT|HWMON_C_LABEL,
						HWMON_C_INPUT|HWMON_C_LABEL,
						HWMON_C_INPUT|HWMON_C_LABEL),

		HWMON_CHANNEL_INFO(in,
						HWMON_I_INPUT|HWMON_I_LABEL,
						HWMON_I_INPUT|HWMON_I_LABEL,
						HWMON_I_INPUT|HWMON_I_LABEL,
						HWMON_I_INPUT|HWMON_I_LABEL,
						HWMON_I_INPUT|HWMON_I_LABEL,
						HWMON_I_INPUT|HWMON_I_LABEL,
						HWMON_I_INPUT|HWMON_I_LABEL,
						HWMON_I_INPUT|HWMON_I_LABEL,
						HWMON_I_INPUT|HWMON_I_LABEL,
						HWMON_I_INPUT|HWMON_I_LABEL,
						HWMON_I_INPUT|HWMON_I_LABEL,
						HWMON_I_INPUT|HWMON_I_LABEL,
						HWMON_I_INPUT|HWMON_I_LABEL,
						HWMON_I_INPUT|HWMON_I_LABEL,
						HWMON_I_INPUT|HWMON_I_LABEL,
						HWMON_I_INPUT|HWMON_I_LABEL,
						HWMON_I_INPUT|HWMON_I_LABEL,
						HWMON_I_INPUT|HWMON_I_LABEL,
						HWMON_I_INPUT|HWMON_I_LABEL,
						HWMON_I_INPUT|HWMON_I_LABEL,
						HWMON_I_INPUT|HWMON_I_LABEL,
						HWMON_I_INPUT|HWMON_I_LABEL,
						HWMON_I_INPUT|HWMON_I_LABEL,
						HWMON_I_INPUT|HWMON_I_LABEL,
						HWMON_I_INPUT|HWMON_I_LABEL,
						HWMON_I_INPUT|HWMON_I_LABEL,
						HWMON_I_INPUT|HWMON_I_LABEL,
						HWMON_I_INPUT|HWMON_I_LABEL),

		HWMON_CHANNEL_INFO(power,
						HWMON_P_INPUT|HWMON_P_LABEL,
						HWMON_P_INPUT|HWMON_P_LABEL,
						HWMON_P_INPUT|HWMON_P_LABEL,
						HWMON_P_INPUT|HWMON_P_LABEL,
						HWMON_P_INPUT|HWMON_P_LABEL,
						HWMON_P_INPUT|HWMON_P_LABEL,
						HWMON_P_INPUT|HWMON_P_LABEL,
						HWMON_P_INPUT|HWMON_P_LABEL,
						HWMON_P_INPUT|HWMON_P_LABEL,
						HWMON_P_INPUT|HWMON_P_LABEL,
						HWMON_P_INPUT|HWMON_P_LABEL,
						HWMON_P_INPUT|HWMON_P_LABEL,
						HWMON_P_INPUT|HWMON_P_LABEL,
						HWMON_P_INPUT|HWMON_P_LABEL,
						HWMON_P_INPUT|HWMON_P_LABEL,
						HWMON_P_INPUT|HWMON_P_LABEL,
						HWMON_P_INPUT|HWMON_P_LABEL,
						HWMON_P_INPUT|HWMON_P_LABEL,
						HWMON_P_INPUT|HWMON_P_LABEL,
						HWMON_P_INPUT|HWMON_P_LABEL,
						HWMON_P_INPUT|HWMON_P_LABEL,
						HWMON_P_INPUT|HWMON_P_LABEL,
						HWMON_P_INPUT|HWMON_P_LABEL,
						HWMON_P_INPUT|HWMON_P_LABEL,
						HWMON_P_INPUT|HWMON_P_LABEL,
						HWMON_P_INPUT|HWMON_P_LABEL,
						HWMON_P_INPUT|HWMON_P_LABEL,
						HWMON_P_INPUT|HWMON_P_LABEL,
						HWMON_P_INPUT|HWMON_P_LABEL,
						HWMON_P_INPUT|HWMON_P_LABEL,
						HWMON_P_INPUT|HWMON_P_LABEL,
						HWMON_P_INPUT|HWMON_P_LABEL,
						HWMON_P_INPUT|HWMON_P_LABEL,
						HWMON_P_INPUT|HWMON_P_LABEL,
						HWMON_P_INPUT|HWMON_P_LABEL,
						HWMON_P_INPUT|HWMON_P_LABEL,
						HWMON_P_INPUT|HWMON_P_LABEL,
						HWMON_P_INPUT|HWMON_P_LABEL,
						HWMON_P_INPUT|HWMON_P_LABEL,
						HWMON_P_INPUT|HWMON_P_LABEL,
						HWMON_P_INPUT|HWMON_P_LABEL),
		NULL
};

static const struct hwmon_ops apple_soc_smc_hwmon_ops = {
		.is_visible = apple_soc_smc_is_visible,
		.read = apple_soc_smc_read,
		.read_string = apple_soc_smc_read_labels,
		.write = apple_soc_smc_write,
};

static const struct hwmon_chip_info apple_soc_smc_chip_info = {
		.ops = &apple_soc_smc_hwmon_ops,
		.info = apple_soc_smc_info,
};


static int apple_soc_smc_hwmon_probe(struct platform_device *pdev)
{
	struct apple_smc *smc = dev_get_drvdata(pdev->dev.parent);
	struct device *dev = &pdev->dev;
	struct macsmc_hwmon *smc_hwmon;

	smc_hwmon = devm_kzalloc(&pdev->dev, sizeof(*smc_hwmon), GFP_KERNEL);
	if (!smc_hwmon)
		return -ENOMEM;

	smc_hwmon->dev = &pdev->dev;
	smc_hwmon->smc = smc;

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
