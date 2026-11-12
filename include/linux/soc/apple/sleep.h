/* SPDX-License-Identifier: GPL-2.0-only OR MIT */
#ifndef _LINUX_SOC_APPLE_SLEEP_H_
#define _LINUX_SOC_APPLE_SLEEP_H_

/* EXPERIMENTAL (apple-s2idle): cpuidle-apple.c, for apple-sleep's suspend_ops->enter */
int apple_cpuidle_system_suspend(void);

#endif
