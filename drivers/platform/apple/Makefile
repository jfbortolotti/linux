# SPDX-License-Identifier: GPL-2.0
#
# Makefile for linux/drivers/platform/apple
# Apple Platform-Specific Drivers
#

macsmc-y				+= smc_core.o
macsmc-rtkit-y				+= smc_rtkit.o

obj-$(CONFIG_APPLE_SMC)			+= macsmc.o
obj-$(CONFIG_APPLE_SMC_RTKIT)		+= macsmc-rtkit.o
obj-$(CONFIG_APPLE_SUSPEND)		+= apple-suspend.o
obj-$(CONFIG_APPLE_SUSPEND_LEVEL4)	+= apple-suspend-l4.o apple-suspend-l4-asm.o
