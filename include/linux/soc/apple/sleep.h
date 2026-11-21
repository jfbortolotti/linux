/* SPDX-License-Identifier: GPL-2.0-only OR MIT */
#ifndef _LINUX_SOC_APPLE_SLEEP_H_
#define _LINUX_SOC_APPLE_SLEEP_H_

/* EXPERIMENTAL (apple-s2idle): cpuidle-apple.c, for apple-sleep's suspend_ops->enter */
int apple_cpuidle_system_suspend(void);
void apple_cpuidle_system_suspend_log(void);
int apple_cpuhp_park_deep(unsigned long mask);
int apple_cpuhp_park_hold(unsigned long mask, unsigned long after_cpu0, bool deep);
int apple_cpuhp_park_release(void);
void apple_cpuhp_park_unhold(void);

#endif
