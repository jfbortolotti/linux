// SPDX-License-Identifier: GPL-2.0-only OR MIT
/*
 * Apple Silicon platform suspend (S2RAM) support
 *
 * Registers platform_suspend_ops so that PM_SUSPEND_MEM ("deep" in
 * /sys/power/mem_sleep) is available on Apple Silicon SoCs.
 *
 * Copyright The Asahi Linux Contributors
 */

#include <linux/cpu.h>
#include <linux/init.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/suspend.h>
#include <asm/suspend.h>
#include "apple-suspend-l4.h"

/*
 * Low-level S2RAM finisher.  Called by cpu_suspend() after all CPU state
 * (callee-saved registers, SP, MMU context) has been saved by
 * __cpu_suspend_enter() / cpu_do_suspend() in arch/arm64/kernel/sleep.S
 * and arch/arm64/mm/proc.S.
 *
 * Preconditions when this runs:
 *   - All secondary CPUs are offline.
 *   - IRQs are disabled (DAIF.I set).
 *   - All device drivers have completed their suspend callbacks.
 *   - The MMU identity map is active on this CPU.
 *
 * This function must make the SoC enter the deep retention state while
 * keeping DRAM in self-refresh.  It should never return on success; the
 * resume path re-enters the kernel at cpu_resume (sleep.S) which returns
 * 0 from cpu_suspend() on the way back up the call stack.
 *
 * TODO: implement the Apple-specific low-level entry (SMC/firmware call).
 */
static int apple_suspend_finish(unsigned long unused)
{
	/* Placeholder — replace with Apple firmware / PMU call. */
	return -ENOSYS;
}

static int apple_suspend_enter(suspend_state_t state)
{
	/*
	 * cpu_suspend() saves all CPU state and calls apple_suspend_finish.
	 * On wakeup it restores that state and returns 0 here.
	 */
	/* Experimental power-gated (Level 4) path, off unless explicitly armed */
	if (apple_l4_enabled())
		return apple_l4_suspend();

	return cpu_suspend(0, apple_suspend_finish);
}

static int apple_suspend_valid(suspend_state_t state)
{
	return state == PM_SUSPEND_MEM;
}

static const struct platform_suspend_ops apple_suspend_ops = {
	.valid	= apple_suspend_valid,
	.enter	= apple_suspend_enter,
};

static int apple_pm_probe(struct platform_device *pdev)
{
	suspend_set_ops(&apple_suspend_ops);
	dev_info(&pdev->dev, "Apple Silicon S2RAM registered\n");
	return 0;
}

static struct platform_driver apple_pm_driver = {
	.driver = {
		.name = "apple-pm",
	},
	.probe = apple_pm_probe,
};

static int __init apple_pm_init(void)
{
	struct platform_device *pdev;
	int ret;

	if (!IS_ENABLED(CONFIG_SUSPEND))
		return 0;

	if (!of_machine_is_compatible("apple,arm-platform"))
		return 0;

	ret = platform_driver_register(&apple_pm_driver);
	if (ret)
		return ret;

	pdev = platform_device_register_simple("apple-pm", -1, NULL, 0);
	if (IS_ERR(pdev)) {
		platform_driver_unregister(&apple_pm_driver);
		return PTR_ERR(pdev);
	}

	return 0;
}
device_initcall(apple_pm_init);
