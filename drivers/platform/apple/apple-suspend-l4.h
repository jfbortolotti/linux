/* SPDX-License-Identifier: GPL-2.0-only OR MIT */
#ifndef APPLE_SUSPEND_L4_H
#define APPLE_SUSPEND_L4_H

#include <linux/types.h>

#ifdef CONFIG_APPLE_SUSPEND_LEVEL4
bool apple_l4_enabled(void);
int apple_l4_suspend(void);
void apple_l4_resume(void);
#else
static inline bool apple_l4_enabled(void) { return false; }
static inline int apple_l4_suspend(void) { return -ENOSYS; }
static inline void apple_l4_resume(void) { }
#endif

/* apple-suspend-l4-asm.S */
void apple_l4_clean_dcache_all(void);
void apple_l4_final_wfi(void);

#endif
