// SPDX-License-Identifier: GPL-2.0-only OR MIT
/*
 * EXPERIMENTAL: Apple Silicon "Level 4" sleep (power-gated CPU core)
 *
 * Ports the bare-metal Level 4 test of m1n1 (src/s2r.c: test_level4_deep_sleep,
 * s2r_enter_deep) and XNU's arm64_prepare_for_sleep() to Linux's
 * cpu_suspend()/cpu_resume() machinery:
 *
 *  - ACC_OVRD.DEEP_SLEEP (bit 34) + the other "ok to power down" fields, and
 *    CYC_OVRD deep WFI without retention, make the next WFI power-gate the core.
 *  - Nothing in the AIC (peripheral IRQs, timer, IPIs) wakes a gated core. In
 *    m1n1 the only thing that restarted one was a PMGR cpu-start write issued
 *    by another core. Whether the SMC/AON can do it is what this experiment
 *    tries to find out.
 *  - A restarted core begins at RVBAR (cpu-impl-reg), which is locked and points
 *    at m1n1's _vectors_start. m1n1 therefore has to forward the reset to
 *    Linux's cpu_resume(): see the "m1n1 handoff" below. Without that handoff
 *    a wake reboots into m1n1_main(), so this code refuses to sleep unless it
 *    finds it (or the user forces it).
 *
 * T8103 only. Default off: needs CONFIG_APPLE_SUSPEND_LEVEL4=y and
 * "apple_suspend_l4.enable=1" on the kernel command line.
 *
 * Two triggers:
 *  - the regular suspend path ("deep" in /sys/power/mem_sleep). It needs a
 *    single online CPU (spin-table CPUs cannot be offlined: boot with
 *    maxcpus=1). Wake sources are whatever is armed by the drivers (SMC power
 *    button/lid via macsmc-hid, RTC alarm, "rtcwake -m mem -s N").
 *  - debugfs apple_l4/run ("<waker_cpu> <delay_ms>"): CPU0 sleeps directly
 *    through cpu_suspend() while the waker CPU (cluster 1) does the PMGR
 *    cpu-start write after the delay. Needs SMP.
 *
 * Copyright The Asahi Linux Contributors
 */

#define pr_fmt(fmt) "apple-l4: " fmt

#include <linux/bitfield.h>
#include <linux/cpu.h>
#include <linux/cpu_pm.h>
#include <linux/debugfs.h>
#include <linux/delay.h>
#include <linux/io.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/printk.h>
#include <linux/smp.h>
#include <linux/workqueue.h>
#include <asm/cputype.h>
#include <asm/sysreg.h>
#include <asm/suspend.h>
#include "apple-suspend-l4.h"

/* Registers: same encodings as m1n1 src/cpu_regs.h and drivers/cpuidle/cpuidle-apple.c */
#define SYS_IMP_APL_CYC_OVRD		sys_reg(3, 5, 15, 5, 0)
#define CYC_OVRD_WFI_MODE_MASK		GENMASK_ULL(25, 24)
#define CYC_OVRD_WFI_MODE(x)		FIELD_PREP(CYC_OVRD_WFI_MODE_MASK, x)
#define CYC_OVRD_DISABLE_WFI_RET	BIT_ULL(0)

#define SYS_IMP_APL_ACC_OVRD		sys_reg(3, 5, 15, 6, 0)
#define ACC_OVRD_PWR_DN_SRM_MASK	GENMASK_ULL(14, 13)
#define ACC_OVRD_DIS_L2_FLUSH_MASK	GENMASK_ULL(16, 15)
#define ACC_OVRD_TRAIN_DOWN_LINK_MASK	GENMASK_ULL(18, 17)
#define ACC_OVRD_POWER_DOWN_CPM_MASK	GENMASK_ULL(26, 25)
#define ACC_OVRD_DISABLE_PIO_ON_WFI	BIT_ULL(32)
#define ACC_OVRD_DEEP_SLEEP		BIT_ULL(34)

/*
 * T8103 PMGR CPU start block (m1n1 smp.c: pmgr reg 0 + CPU_START_OFF_T8103).
 *   +0x0              stop request:  BIT(4 * cluster + core)
 *   +0x4              start enable:  BIT(4 * cluster + core)
 *   +0x8 + 4*cluster  start:         BIT(core)
 * The Linux DT only maps the first 0x14000 of the PMGR, so map the block by hand.
 */
#define T8103_PMGR_CPU_START		0x23b754000ULL
#define T8103_PMGR_CPU_START_SIZE	0x20
/* cpu-impl-reg (RVBAR at +0): 0x210050000 + 0x1000000 * cluster + 0x100000 * core (adt.txt) */
#define T8103_CPU_IMPL(cl, co)		(0x210050000ULL + 0x1000000ULL * (cl) + 0x100000ULL * (co))
#define RVBAR_LOCK			BIT_ULL(0)

/*
 * m1n1 handoff (PROPOSED, not in m1n1 yet): m1n1 would publish the physical
 * address of this structure in /chosen "asahi,m1n1-resume" and, on a reset of
 * the boot CPU with magic set, run its init_cpu() and jump to ->entry with the
 * MMU off.
 */
struct m1n1_resume {
	u64 magic;
	u64 entry;
};
#define M1N1_RESUME_MAGIC	0x4c4e58524553554dULL /* "LNXRESUM" */

static bool enable;
module_param(enable, bool, 0444);
MODULE_PARM_DESC(enable, "Arm the experimental Level 4 sleep path");

static bool force;
module_param(force, bool, 0444);
MODULE_PARM_DESC(force, "Sleep even without the m1n1 resume handoff (WILL reboot into m1n1 on wake)");

static struct {
	bool ready;
	int cluster, core;
	void __iomem *cpu_start;
	void __iomem *rvbar;
	struct m1n1_resume *handoff;
	u64 acc_saved, cyc_saved;
	bool overrides_applied;
} l4;

bool apple_l4_enabled(void)
{
	return enable;
}

static void l4_set_overrides(void)
{
	u64 acc, cyc;

	l4.acc_saved = read_sysreg_s(SYS_IMP_APL_ACC_OVRD);
	l4.cyc_saved = read_sysreg_s(SYS_IMP_APL_CYC_OVRD);

	/* m1n1 test_level4_deep_sleep / XNU arm64_prepare_for_sleep */
	acc = l4.acc_saved;
	acc &= ~ACC_OVRD_DIS_L2_FLUSH_MASK;
	acc |= FIELD_PREP(ACC_OVRD_PWR_DN_SRM_MASK, 3) |
	       FIELD_PREP(ACC_OVRD_DIS_L2_FLUSH_MASK, 2) |
	       FIELD_PREP(ACC_OVRD_TRAIN_DOWN_LINK_MASK, 3) |
	       FIELD_PREP(ACC_OVRD_POWER_DOWN_CPM_MASK, 3) |
	       ACC_OVRD_DISABLE_PIO_ON_WFI | ACC_OVRD_DEEP_SLEEP;
	write_sysreg_s(acc, SYS_IMP_APL_ACC_OVRD);

	cyc = l4.cyc_saved;
	cyc &= ~CYC_OVRD_WFI_MODE_MASK;
	cyc |= CYC_OVRD_WFI_MODE(3) | CYC_OVRD_DISABLE_WFI_RET;
	write_sysreg_s(cyc, SYS_IMP_APL_CYC_OVRD);
	isb();

	l4.overrides_applied = true;
}

static void l4_restore_overrides(void)
{
	if (!l4.overrides_applied)
		return;

	write_sysreg_s(l4.cyc_saved, SYS_IMP_APL_CYC_OVRD);
	write_sysreg_s(l4.acc_saved, SYS_IMP_APL_ACC_OVRD);
	isb();
	l4.overrides_applied = false;
}

static int l4_find_handoff(void)
{
	struct device_node *np = of_find_node_by_path("/chosen");
	u64 pa;
	int ret;

	if (!np)
		return -ENODEV;
	ret = of_property_read_u64(np, "asahi,m1n1-resume", &pa);
	of_node_put(np);
	if (ret)
		return ret;

	l4.handoff = memremap(pa, sizeof(*l4.handoff), MEMREMAP_WB);
	return l4.handoff ? 0 : -ENOMEM;
}

/* Called with the boot CPU as the executing CPU; IRQs may be on. */
static int l4_prepare(void)
{
	u64 mpidr = read_cpuid_mpidr();
	u64 rvbar;
	int ret;

	if (!enable)
		return -EPERM;

	if (!of_machine_is_compatible("apple,t8103")) {
		pr_err("only T8103 is supported (PMGR/RVBAR addresses are hard-coded)\n");
		return -ENODEV;
	}

	l4.cluster = MPIDR_AFFINITY_LEVEL(mpidr, 1);
	l4.core = MPIDR_AFFINITY_LEVEL(mpidr, 0);

	if (!l4.cpu_start)
		l4.cpu_start = ioremap(T8103_PMGR_CPU_START, T8103_PMGR_CPU_START_SIZE);
	if (!l4.rvbar)
		l4.rvbar = ioremap(T8103_CPU_IMPL(l4.cluster, l4.core), 8);
	if (!l4.cpu_start || !l4.rvbar)
		return -ENOMEM;

	rvbar = readq(l4.rvbar);
	pr_info("cpu%d (cluster %d core %d) RVBAR=%#llx%s cpu_start=%#llx\n",
		smp_processor_id(), l4.cluster, l4.core, rvbar,
		(rvbar & RVBAR_LOCK) ? " (locked)" : "", T8103_PMGR_CPU_START);

	if (!l4.handoff) {
		ret = l4_find_handoff();
		if (ret)
			pr_warn("no m1n1 resume handoff (/chosen asahi,m1n1-resume): %d\n", ret);
	}

	if (l4.handoff) {
		l4.handoff->entry = __pa_symbol(cpu_resume);
		/* magic last; the full D-cache clean in the finisher pushes it to DRAM */
		WRITE_ONCE(l4.handoff->magic, M1N1_RESUME_MAGIC);
		pr_info("m1n1 handoff armed: entry=%#llx\n", l4.handoff->entry);
	} else if (!force) {
		pr_err("RVBAR points at m1n1 and cannot forward to cpu_resume(): refusing "
		       "to power-gate (use apple_suspend_l4.force=1 to try anyway)\n");
		return -ENOEXEC;
	} else {
		pr_warn("forced: the wake will land in m1n1's reset path, not in Linux\n");
	}

	l4.ready = true;
	return 0;
}

/* PMGR cpu-start sequence of m1n1 smp_wake_start() */
static void l4_pmgr_start(void)
{
	writel(BIT(4 * l4.cluster + l4.core), l4.cpu_start + 0x4);
	writel(BIT(l4.core), l4.cpu_start + 0x8 + 4 * l4.cluster);
}

/*
 * cpu_suspend() finisher. Runs with DAIF masked and the MMU on; never returns
 * on success (cpu_resume() returns 0 from cpu_suspend()).
 */
static int l4_finisher(unsigned long unused)
{
	l4_set_overrides();

	pr_info("ACC_OVRD=%#llx CYC_OVRD=%#llx, entering final WFI (cpu%d)\n",
		read_sysreg_s(SYS_IMP_APL_ACC_OVRD), read_sysreg_s(SYS_IMP_APL_CYC_OVRD),
		smp_processor_id());

	/* PMGR "stop request" for this core, as m1n1's smp_wake_stop_request() */
	writel(BIT(4 * l4.cluster + l4.core), l4.cpu_start + 0x0);

	apple_l4_clean_dcache_all();
	apple_l4_final_wfi();

	/* Only reached if the core woke up without being gated */
	l4_restore_overrides();
	pr_err("final WFI returned without power-gating\n");
	return -EAGAIN;
}

/* Hooks for apple-suspend.c (regular suspend path) */
int apple_l4_suspend(void)
{
	int ret = l4_prepare();

	if (ret)
		return ret;
	if (num_online_cpus() != 1) {
		pr_err("%u CPUs online: spin-table CPUs cannot be offlined, boot with maxcpus=1\n",
		       num_online_cpus());
		return -EBUSY;
	}

	ret = cpu_pm_enter();
	if (ret)
		return ret;

	ret = cpu_suspend(0, l4_finisher);
	apple_l4_resume();
	cpu_pm_exit();
	return ret;
}

void apple_l4_resume(void)
{
	/* After a reset the overrides come back at their reset value: put ours back */
	l4_restore_overrides();
	pr_info("back from Level 4 sleep on cpu%d\n", smp_processor_id());
}

/* ---- debugfs trigger with a waker CPU (PMGR cpu-start fallback) ---- */

struct l4_run {
	int waker;
	unsigned int delay_ms;
	int ret;
};

static void l4_waker_fn(void *info)
{
	struct l4_run *run = info;

	/* IRQ context on the waker: busy-wait so the sleeper is surely in its WFI */
	mdelay(run->delay_ms);
	l4_pmgr_start();
	pr_info("waker cpu%d: PMGR cpu-start written\n", smp_processor_id());
}

static long l4_run_on_cpu0(void *data)
{
	struct l4_run *run = data;
	int ret;

	ret = l4_prepare();
	if (ret)
		return ret;

	if (run->waker >= 0)
		smp_call_function_single(run->waker, l4_waker_fn, run, 0);

	ret = cpu_pm_enter();
	if (ret)
		return ret;

	ret = cpu_suspend(0, l4_finisher);
	apple_l4_resume();
	cpu_pm_exit();
	return ret;
}

static ssize_t l4_run_write(struct file *file, const char __user *ubuf, size_t len, loff_t *ppos)
{
	struct l4_run run = { .waker = -1, .delay_ms = 3000 };
	char buf[32];
	int n;

	if (len >= sizeof(buf))
		return -EINVAL;
	if (copy_from_user(buf, ubuf, len))
		return -EFAULT;
	buf[len] = 0;

	n = sscanf(buf, "%d %u", &run.waker, &run.delay_ms);
	if (n < 1)
		return -EINVAL;

	if (run.waker >= 0) {
		if (!cpu_online(run.waker) || run.waker == 0)
			return -EINVAL;
		if (MPIDR_AFFINITY_LEVEL(cpu_logical_map(run.waker), 1) ==
		    MPIDR_AFFINITY_LEVEL(cpu_logical_map(0), 1)) {
			pr_err("waker cpu%d is in the sleeping cluster; it would go unresponsive\n",
			       run.waker);
			return -EINVAL;
		}
	}

	run.ret = work_on_cpu(0, l4_run_on_cpu0, &run);
	pr_info("run finished: %d\n", run.ret);
	return run.ret ? run.ret : len;
}

static const struct file_operations l4_run_fops = {
	.write = l4_run_write,
	.llseek = noop_llseek,
};

static int __init apple_l4_init(void)
{
	if (!enable)
		return 0;

	if (!of_machine_is_compatible("apple,t8103"))
		return 0;

	debugfs_create_file("run", 0200, debugfs_create_dir("apple_l4", NULL), NULL, &l4_run_fops);
	pr_warn("EXPERIMENTAL Level 4 sleep armed (power-gates the boot core)\n");
	return 0;
}
late_initcall(apple_l4_init);
