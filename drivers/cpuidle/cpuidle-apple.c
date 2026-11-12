// SPDX-License-Identifier: GPL-2.0-only OR MIT
/*
 * Copyright The Asahi Linux Contributors
 *
 * CPU idle support for Apple SoCs
 */

#include <linux/init.h>
#include <linux/bits.h>
#include <linux/cpuhotplug.h>
#include <linux/cpuidle.h>
#include <linux/cpu_pm.h>
#include <linux/iopoll.h>
#include <linux/moduleparam.h>
#include <linux/platform_device.h>
#include <linux/of.h>
#include <linux/smp.h>
#include <linux/soc/apple/sleep.h>
#include <linux/io.h>
#include <asm/cacheflush.h>
#include <asm/cpuidle.h>
#include <asm/memory.h>
#include <asm/cputype.h>
#include <asm/sections.h>
#include <asm/smp_plat.h>
#include <asm/spin_table.h>
#include <asm/suspend.h>
#include <asm/sysreg.h>

/*
 * Experimental (apple-sleep): during s2idle only, add the ACC_OVRD
 * "ok to power down" fields m1n1's Level 3 sets on the boot CPU before its
 * WFI (as XNU's arm64_prepare_for_sleep, without ACC_OVRD.DEEP_SLEEP, which
 * is chip-wide and power-gates the cores). Retention is kept.
 *   cpuidle_apple.s2idle_acc=0: off, 1: on,
 *     2: + CYC_OVRD.DISABLE_WFI_RET on the boot CPU, as m1n1's Level 3: the core is
 *        power-gated by the WFI and comes back through its reset vector (RVBAR, locked to
 *        m1n1), which hands over to cpu_resume() through /chosen "asahi,m1n1-resume".
 *        Needs an m1n1 that publishes it; without it, 2 behaves as 1. Default 2.
 *   cpuidle_apple.s2idle_all_cpus=0: boot CPU only (as m1n1), 1: every CPU (ACC bits only)
 */
static unsigned int s2idle_acc = 2;
module_param(s2idle_acc, uint, 0644);
MODULE_PARM_DESC(s2idle_acc, "s2idle: 0 off, 1 ACC_OVRD power-down fields, 2 (default) + power-gated boot CPU (m1n1 resume)");
static unsigned int s2idle_gated;
module_param(s2idle_gated, uint, 0444);
MODULE_PARM_DESC(s2idle_gated, "s2idle_acc=2: wakes that came back through the reset vector");
static unsigned int s2idle_not_gated;
module_param(s2idle_not_gated, uint, 0444);
MODULE_PARM_DESC(s2idle_not_gated, "s2idle_acc=2: WFIs that returned without power-gating");
static bool s2idle_all_cpus;
module_param(s2idle_all_cpus, bool, 0644);
MODULE_PARM_DESC(s2idle_all_cpus, "s2idle_acc on every CPU instead of the boot CPU only");

#define SYS_IMP_APL_CYC_OVRD		sys_reg(3, 5, 15, 5, 0)
#define SYS_IMP_APL_ACC_OVRD		sys_reg(3, 5, 15, 6, 0)
#define ACC_OVRD_PWR_DN_SRM(x)		((u64)(x) << 13)
#define ACC_OVRD_DIS_L2_FLUSH_MASK	GENMASK_ULL(16, 15)
#define ACC_OVRD_DIS_L2_FLUSH(x)	((u64)(x) << 15)
#define ACC_OVRD_TRAIN_DOWN_LINK(x)	((u64)(x) << 17)
#define ACC_OVRD_POWER_DOWN_CPM(x)	((u64)(x) << 25)
#define ACC_OVRD_DISABLE_PIO_ON_WFI	BIT_ULL(32)
#define ACC_OVRD_DEEP_SLEEP		BIT_ULL(34)

enum idle_state {
	STATE_WFI,
	STATE_PWRDOWN,
	STATE_COUNT
};

asm(
	".pushsection .cpuidle.text, \"ax\"\n"
	".type apple_cpu_deep_wfi, @function\n"
	"apple_cpu_deep_wfi:\n"
		"str x30, [sp, #-16]!\n"
		"stp x28, x29, [sp, #-16]!\n"
		"stp x26, x27, [sp, #-16]!\n"
		"stp x24, x25, [sp, #-16]!\n"
		"stp x22, x23, [sp, #-16]!\n"
		"stp x20, x21, [sp, #-16]!\n"
		"stp x18, x19, [sp, #-16]!\n"

		"mrs x0, s3_5_c15_c5_0\n"
		"orr x0, x0, #(3L << 24)\n"
		"msr s3_5_c15_c5_0, x0\n"

	"1:\n"
		"dsb sy\n"
		"wfi\n"

		"mrs x0, ISR_EL1\n"
		"cbz x0, 1b\n"

		"mrs x0, s3_5_c15_c5_0\n"
		"bic x0, x0, #(1L << 24)\n"
		"msr s3_5_c15_c5_0, x0\n"

		"ldp x18, x19, [sp], #16\n"
		"ldp x20, x21, [sp], #16\n"
		"ldp x22, x23, [sp], #16\n"
		"ldp x24, x25, [sp], #16\n"
		"ldp x26, x27, [sp], #16\n"
		"ldp x28, x29, [sp], #16\n"
		"ldr x30, [sp], #16\n"

		"ret\n"
	".popsection\n"
);

void apple_cpu_deep_wfi(void);

/*
 * apple_cpu_gate_wfi(): clean every data cache level by set/way (the core's L1 is lost when
 * it is power-gated; same loop as m1n1's dcsw_op_all), then WFI with CYC_OVRD deep WFI and
 * DISABLE_WFI_RET. No memory is written after the clean. Returns 1 only if the WFI returned
 * without power-gating; otherwise the core restarts at RVBAR and cpu_suspend() returns 0.
 * Clobbers x0-x13.
 */
asm(
	".pushsection .cpuidle.text, \"ax\"\n"
	".type apple_cpu_gate_wfi, @function\n"
	"apple_cpu_gate_wfi:\n"
		"dmb sy\n"
		"mrs x0, clidr_el1\n"
		"and x3, x0, #0x7000000\n"
		"lsr x3, x3, #23\n"
		"cbz x3, 5f\n"
		"mov x10, #0\n"
	"1:\n"
		"add x2, x10, x10, lsr #1\n"
		"lsr x1, x0, x2\n"
		"and x1, x1, #7\n"
		"cmp x1, #2\n"
		"b.lt 4f\n"
		"msr csselr_el1, x10\n"
		"isb\n"
		"mrs x1, ccsidr_el1\n"
		"and x2, x1, #7\n"
		"add x2, x2, #4\n"
		"mov x4, #0x3ff\n"
		"and x4, x4, x1, lsr #3\n"
		"clz w5, w4\n"
		"mov x7, #0x7fff\n"
		"and x7, x7, x1, lsr #13\n"
	"2:\n"
		"mov x9, x4\n"
	"3:\n"
		"lsl x6, x9, x5\n"
		"lsl x12, x7, x2\n"
		"orr x11, x10, x6\n"
		"orr x11, x11, x12\n"
		"dc csw, x11\n"
		"subs x9, x9, #1\n"
		"b.ge 3b\n"
		"subs x7, x7, #1\n"
		"b.ge 2b\n"
	"4:\n"
		"add x10, x10, #2\n"
		"cmp x3, x10\n"
		"b.gt 1b\n"
	"5:\n"
		"mov x10, #0\n"
		"msr csselr_el1, x10\n"
		"dsb sy\n"
		"isb\n"

		"mrs x0, s3_5_c15_c5_0\n"
		"mov x13, x0\n"
		"orr x0, x0, #(3L << 24)\n"
		"orr x0, x0, #1\n"
		"msr s3_5_c15_c5_0, x0\n"
		"isb\n"
		"dsb sy\n"
		"wfi\n"
		"msr s3_5_c15_c5_0, x13\n"
		"isb\n"
		"mov x0, #1\n"
		"ret\n"
	".popsection\n"
);

int apple_cpu_gate_wfi(void);
void apple_aic_cpu_reset_resume(void);	/* irq-apple-aic.c */

/* m1n1 src/s2r.h struct s2r_linux_resume */
#define M1N1_RESUME_MAGIC	0x4c4e58524553554dULL /* "LNXRESUM" */
#define M1N1_RESUME_CPUS	16
struct m1n1_resume_cpu {
	u64 mpidr;
	u64 magic;
	u64 entry;
	u64 count;
	u64 acc_ovrd;	/* m1n1 writes it back to ACC_OVRD if not 0 */
	u64 pad[3];
};
struct m1n1_resume {
	u64 magic;
	u64 entry;
	u64 count;
	u64 pad[5];
	/* only if m1n1 publishes /chosen "asahi,m1n1-resume-cpus" */
	struct m1n1_resume_cpu cpu[M1N1_RESUME_CPUS];
};
static struct m1n1_resume *m1n1_resume;
static unsigned int m1n1_resume_cpus;	/* slots m1n1 has, 0 = boot CPU handoff only */

static void m1n1_resume_set(u64 magic)
{
	WRITE_ONCE(m1n1_resume->magic, magic);
	dcache_clean_poc((unsigned long)m1n1_resume,
			 (unsigned long)m1n1_resume + sizeof(*m1n1_resume));
}

static int apple_gate_finisher(unsigned long unused)
{
	m1n1_resume_set(M1N1_RESUME_MAGIC);
	return apple_cpu_gate_wfi();
}

static void apple_cpuidle_find_m1n1_resume(void)
{
	struct device_node *np;
	size_t size;
	u32 ncpus = 0;
	u64 pa;

	if (m1n1_resume)
		return;
	np = of_find_node_by_path("/chosen");
	if (!np)
		return;
	if (of_property_read_u64(np, "asahi,m1n1-resume", &pa)) {
		of_node_put(np);
		return;
	}
	of_property_read_u32(np, "asahi,m1n1-resume-cpus", &ncpus);
	of_node_put(np);

	/* an older m1n1 has the boot CPU slot only: never touch what follows it */
	m1n1_resume_cpus = min_t(u32, ncpus, M1N1_RESUME_CPUS);
	size = m1n1_resume_cpus ? sizeof(*m1n1_resume) : offsetof(struct m1n1_resume, cpu);
	m1n1_resume = memremap(pa, size, MEMREMAP_WB);
	if (!m1n1_resume) {
		pr_warn("cpuidle-apple: cannot map the m1n1 resume handoff at %#llx\n", pa);
		return;
	}
	m1n1_resume->magic = 0;
	m1n1_resume->entry = __pa_symbol(cpu_resume);
	for (int i = 0; i < m1n1_resume_cpus; i++)
		m1n1_resume->cpu[i].magic = 0;
	dcache_clean_poc((unsigned long)m1n1_resume, (unsigned long)m1n1_resume + size);
	pr_info("cpuidle-apple: m1n1 resume handoff at %#llx, entry %#llx (s2idle_acc=2 usable), %u CPU slots\n",
		pa, m1n1_resume->entry, m1n1_resume_cpus);
}

static __cpuidle int apple_enter_wfi(struct cpuidle_device *dev, struct cpuidle_driver *drv, int index)
{
	cpu_do_idle();
	return index;
}

static __cpuidle int apple_enter_idle(struct cpuidle_device *dev, struct cpuidle_driver *drv, int index)
{
	/*
	 * Deep WFI will clobber FP state, among other things.
	 * The CPU PM notifier will take care of saving that and anything else
	 * that needs to be notified of the CPU powering down.
	 */
	if (cpu_pm_enter())
		return -1;

	ct_cpuidle_enter();

	switch(index) {
	case STATE_PWRDOWN:
		apple_cpu_deep_wfi();
		break;
	default:
		WARN_ON(1);
		break;
	}

	ct_cpuidle_exit();

	cpu_pm_exit();

	return index;
}

static u64 apple_acc_ovrd_set(void)
{
	u64 acc = read_sysreg_s(SYS_IMP_APL_ACC_OVRD);

	write_sysreg_s((acc & ~ACC_OVRD_DIS_L2_FLUSH_MASK) |
		       ACC_OVRD_PWR_DN_SRM(3) | ACC_OVRD_DIS_L2_FLUSH(2) |
		       ACC_OVRD_TRAIN_DOWN_LINK(3) | ACC_OVRD_POWER_DOWN_CPM(3) |
		       ACC_OVRD_DISABLE_PIO_ON_WFI, SYS_IMP_APL_ACC_OVRD);
	isb();
	return acc;
}

/*
 * Boot CPU power-gated WFI, resumed by m1n1 through cpu_resume(). Returns true if the core
 * came back through RVBAR, false if the WFI returned without power-gating.
 */
static __cpuidle bool apple_boot_cpu_gated_wfi(void)
{
	u64 acc, cyc;
	int ret;

	cyc = read_sysreg_s(SYS_IMP_APL_CYC_OVRD);
	acc = apple_acc_ovrd_set();

	/* cpu_suspend() does ct_cpuidle_enter/exit itself (a second pair = WARN in ct_kernel_exit) */
	ret = cpu_suspend(0, apple_gate_finisher);

	if (!ret) {
		/* back through RVBAR + m1n1: per-CPU state is at its reset values */
		apple_aic_cpu_reset_resume();
	} else {
		m1n1_resume_set(0);
	}

	write_sysreg_s(cyc, SYS_IMP_APL_CYC_OVRD);
	write_sysreg_s(acc, SYS_IMP_APL_ACC_OVRD);
	isb();
	return !ret;
}

/* s2idle_acc=2 on the boot CPU */
static __cpuidle int apple_enter_s2idle_gated(struct cpuidle_device *dev,
					      struct cpuidle_driver *drv, int index)
{
	if (cpu_pm_enter())
		return -1;

	if (apple_boot_cpu_gated_wfi())
		s2idle_gated++;
	else
		s2idle_not_gated++;

	cpu_pm_exit();
	return index;
}

static unsigned int sys_gated, sys_not_gated;
module_param(sys_gated, uint, 0444);
MODULE_PARM_DESC(sys_gated, "mem_sleep deep: sleeps that came back through the reset vector");
module_param(sys_not_gated, uint, 0444);
MODULE_PARM_DESC(sys_not_gated, "mem_sleep deep: WFIs that returned without power-gating");

/*
 * EXPERIMENTAL, for apple-sleep's suspend_ops->enter (mem_sleep "deep"): the boot CPU, last one
 * online, IRQs off, after syscore_suspend() (which ran the CPU PM notifiers). Power-gated WFI
 * as s2idle_acc=2; any unmasked interrupt ends it. Returns 0 if the core was power-gated,
 * 1 if the WFI returned without, -ENODEV without the m1n1 handoff.
 */
int apple_cpuidle_system_suspend(void)
{
	if (!m1n1_resume)
		return -ENODEV;
	if (apple_boot_cpu_gated_wfi()) {
		sys_gated++;
		return 0;
	}
	sys_not_gated++;
	return 1;
}
EXPORT_SYMBOL_GPL(apple_cpuidle_system_suspend);

static __cpuidle int apple_enter_s2idle(struct cpuidle_device *dev,
					 struct cpuidle_driver *drv, int index)
{
	unsigned int cpu = smp_processor_id();
	bool acc_set = s2idle_acc && (s2idle_all_cpus || !cpu);
	u64 acc = 0;
	int ret;

	if (s2idle_acc >= 2 && !cpu && m1n1_resume)
		return apple_enter_s2idle_gated(dev, drv, index);

	if (acc_set)
		acc = apple_acc_ovrd_set();

	ret = apple_enter_idle(dev, drv, index);

	if (acc_set) {
		write_sysreg_s(acc, SYS_IMP_APL_ACC_OVRD);
		isb();
	}
	return ret;
}

/*
 * EXPERIMENTAL CPU hotplug (T8103), as XNU takes a secondary down (PE_cpu_machine_quiesce /
 * PE_cpu_down + arm64_prepare_for_sleep): the core writes its own bit to the PMGR CPU stop
 * register (CPUSTART+0, XNU disableCPUCore), sets the ACC_OVRD power-down fields, and does a
 * power-gated WFI (CYC_OVRD deep WFI + DISABLE_WFI_RET). It is restarted with the PMGR start
 * sequence of m1n1's smp_start_cpu(); it then comes back through its RVBAR (m1n1), which sends
 * it to the Linux holding pen through its per-CPU handoff slot.
 *   cpuidle_apple.cpuhp=0 (command line): don't register the hotplug ops.
 *   cpuidle_apple.cluster_deep=1: the last core of a cluster also sets ACC_OVRD.DEEP_SLEEP,
 *     as XNU does when it powers a cluster down (XNU then calls PMGR disableCPUCluster, not
 *     done here). Never for the boot CPU's cluster.
 */
static bool cpuhp = true;
module_param(cpuhp, bool, 0444);
MODULE_PARM_DESC(cpuhp, "CPU hotplug through PMGR + m1n1 (boot-time only)");
static bool cluster_deep;
module_param(cluster_deep, bool, 0644);
MODULE_PARM_DESC(cluster_deep, "last core of a cluster going offline sets ACC_OVRD.DEEP_SLEEP");
static bool cpuhp_verbose = true;
module_param(cpuhp_verbose, bool, 0644);
MODULE_PARM_DESC(cpuhp_verbose, "log the PMGR power states at each CPU off/on");

#define T8103_PMGR_PS		0x23b700000ULL	/* ECPU0-3 +0x00, PCPU0-3 +0x20, ECPM +0x40, PCPM +0x48 */
#define T8103_CPU_START		0x23b754000ULL	/* +0 stop, +4 global start, +8/+0xc cluster start */
#define PS_ACTUAL		GENMASK(7, 4)

static void __iomem *cpu_start_base, *cpu_ps_base;
static u64 acc_boot[NR_CPUS];
static bool acc_saved[NR_CPUS];

static unsigned int apl_cluster(unsigned int cpu)
{
	return MPIDR_AFFINITY_LEVEL(cpu_logical_map(cpu), 1);
}

static unsigned int apl_core(unsigned int cpu)
{
	return MPIDR_AFFINITY_LEVEL(cpu_logical_map(cpu), 0);
}

static void __iomem *apl_core_ps(unsigned int cpu)
{
	return cpu_ps_base + 8 * (4 * apl_cluster(cpu) + apl_core(cpu));
}

static void __iomem *apl_cluster_ps(unsigned int cluster)
{
	return cpu_ps_base + 0x40 + 8 * cluster;
}

static void apple_cpuhp_die(unsigned int cpu, phys_addr_t entry)
{
	struct m1n1_resume_cpu *slot = &m1n1_resume->cpu[cpu];
	unsigned int cl = apl_cluster(cpu), i;
	bool last = true;

	/*
	 * One-shot slot: m1n1 clears the magic, clears CYC_OVRD.DISABLE_WFI_RET and writes acc_ovrd
	 * back (both survive the power-gate: without that the core's first idle WFI gated it again,
	 * and it booted a second time through the slot and the pen).
	 */
	slot->mpidr = cpu_logical_map(cpu);
	slot->entry = entry;
	slot->acc_ovrd = acc_saved[cpu] ? acc_boot[cpu] : 0;
	WRITE_ONCE(slot->magic, M1N1_RESUME_MAGIC);
	dcache_clean_poc((unsigned long)slot, (unsigned long)(slot + 1));

	/* this CPU is already out of cpu_online_mask */
	for_each_online_cpu(i)
		if (apl_cluster(i) == cl)
			last = false;

	writel(BIT(4 * cl + apl_core(cpu)), cpu_start_base + 0x0);
	apple_acc_ovrd_set();
	if (last && cluster_deep && cl != apl_cluster(0)) {
		sysreg_clear_set_s(SYS_IMP_APL_ACC_OVRD, 0, ACC_OVRD_DEEP_SLEEP);
		isb();
	}

	/* the WFI returns only if it did not power-gate (an interrupt was pending): try again */
	for (;;)
		apple_cpu_gate_wfi();
}

static int apple_cpuhp_kill(unsigned int cpu)
{
	unsigned int cl = apl_cluster(cpu), i;
	bool last = true;
	u32 ps, cps;
	ktime_t t0;
	int ret = readl_poll_timeout_atomic(apl_core_ps(cpu), ps, (ps & PS_ACTUAL) != PS_ACTUAL,
					    10, 50000);

	if (!cpuhp_verbose)
		return 0;
	for_each_online_cpu(i)
		if (apl_cluster(i) == cl)
			last = false;
	if (!last) {
		pr_info("cpuidle-apple: CPU%u down: PS %#x%s, cluster %u PS %#x\n", cpu, ps,
			ret ? " (actual still on)" : "", cl, readl(apl_cluster_ps(cl)));
		return 0;
	}
	/* last core of its cluster: how long until the cluster itself is off (up to 1 s) */
	t0 = ktime_get();
	readl_poll_timeout_atomic(apl_cluster_ps(cl), cps, (cps & PS_ACTUAL) != PS_ACTUAL,
				  10, 1000000);
	pr_info("cpuidle-apple: CPU%u down: PS %#x%s, last of cluster %u%s: cluster PS %#x after %lld us\n",
		cpu, ps, ret ? " (actual still on)" : "", cl,
		cluster_deep && cl != apl_cluster(0) ? " (DEEP_SLEEP set)" : "", cps,
		ktime_us_delta(ktime_get(), t0));
	return 0;
}

static int apple_cpuhp_boot(unsigned int cpu)
{
	unsigned int cl = apl_cluster(cpu), core = apl_core(cpu);
	struct m1n1_resume_cpu *slot = &m1n1_resume->cpu[cpu];

	/* m1n1 counts the restarts with its caches off */
	dcache_inval_poc((unsigned long)slot, (unsigned long)(slot + 1));
	if (cpuhp_verbose)
		pr_info("cpuidle-apple: CPU%u up: PS %#x, cluster %u PS %#x, m1n1 restarts %llu\n",
			cpu, readl(apl_core_ps(cpu)), cl, readl(apl_cluster_ps(cl)),
			READ_ONCE(slot->count));
	writel(BIT(4 * cl + core), cpu_start_base + 0x4);
	writel(BIT(core), cpu_start_base + 0x8 + 4 * cl);
	return 0;
}

static const struct spin_table_hotplug_ops apple_cpuhp_ops = {
	.cpu_die	= apple_cpuhp_die,
	.cpu_kill	= apple_cpuhp_kill,
	.cpu_boot	= apple_cpuhp_boot,
};

/* on the CPU coming online: first time, remember ACC_OVRD; after a hotplug, put it back */
static int apple_cpuhp_online(unsigned int cpu)
{
	u64 acc = read_sysreg_s(SYS_IMP_APL_ACC_OVRD);

	if (!acc_saved[cpu]) {
		acc_boot[cpu] = acc;
		acc_saved[cpu] = true;
		return 0;
	}
	if (acc != acc_boot[cpu]) {
		if (cpuhp_verbose)
			pr_info("cpuidle-apple: CPU%u ACC_OVRD %#llx after restart, back to %#llx\n",
				cpu, acc, acc_boot[cpu]);
		write_sysreg_s(acc_boot[cpu], SYS_IMP_APL_ACC_OVRD);
		isb();
	}
	return 0;
}

static int __init apple_cpuhp_init(void)
{
	int ret;

	if (!cpuhp || !of_machine_is_compatible("apple,t8103"))
		return 0;
	apple_cpuidle_find_m1n1_resume();
	if (!m1n1_resume || m1n1_resume_cpus < num_possible_cpus()) {
		pr_info("cpuidle-apple: no m1n1 per-CPU handoff, no CPU hotplug\n");
		return 0;
	}
	cpu_start_base = ioremap_np(T8103_CPU_START, 0x20);
	cpu_ps_base = ioremap_np(T8103_PMGR_PS, 0x50);
	if (!cpu_start_base || !cpu_ps_base)
		return -ENOMEM;
	ret = cpuhp_setup_state_nocalls(CPUHP_AP_ONLINE_DYN, "cpuidle/apple:acc",
					apple_cpuhp_online, NULL);
	if (ret < 0)
		return ret;
	smp_spin_table_set_hotplug_ops(&apple_cpuhp_ops);
	pr_info("cpuidle-apple: CPU hotplug through PMGR + m1n1 enabled\n");
	return 0;
}
early_initcall(apple_cpuhp_init);

static struct cpuidle_driver apple_idle_driver = {
	.name = "apple_idle",
	.owner = THIS_MODULE,
	.states = {
		[STATE_WFI] = {
			.enter			= apple_enter_wfi,
			.enter_s2idle		= apple_enter_wfi,
			.exit_latency		= 1,
			.target_residency	= 1,
			.power_usage            = UINT_MAX,
			.name			= "WFI",
			.desc			= "CPU clock-gated",
			.flags			= 0,
		},
		[STATE_PWRDOWN] = {
			.enter			= apple_enter_idle,
			.enter_s2idle		= apple_enter_s2idle,
			.exit_latency		= 10,
			.target_residency	= 10000,
			.power_usage            = 0,
			.name			= "CPU PD",
			.desc			= "CPU/cluster powered down",
			.flags			= CPUIDLE_FLAG_RCU_IDLE,
		},
	},
	.safe_state_index = STATE_WFI,
	.state_count = STATE_COUNT,
};

static int apple_cpuidle_probe(struct platform_device *pdev)
{
	apple_cpuidle_find_m1n1_resume();
	return cpuidle_register(&apple_idle_driver, NULL);
}

static struct platform_driver apple_cpuidle_driver = {
	.driver = {
		.name = "cpuidle-apple",
	},
	.probe = apple_cpuidle_probe,
};

static int __init apple_cpuidle_init(void)
{
	struct platform_device *pdev;
	int ret;

	ret = platform_driver_register(&apple_cpuidle_driver);
	if (ret)
		return ret;

	if (!of_machine_is_compatible("apple,arm-platform"))
		return 0;

	if (!(of_machine_is_compatible("apple,t8103") ||
	      of_machine_is_compatible("apple,t8112") ||
	      of_machine_is_compatible("apple,t8122") ||
	      of_machine_is_compatible("apple,t6000") ||
	      of_machine_is_compatible("apple,t6001") ||
	      of_machine_is_compatible("apple,t6002") ||
	      of_machine_is_compatible("apple,t6020") ||
	      of_machine_is_compatible("apple,t6021") ||
	      of_machine_is_compatible("apple,t6022") ||
	      of_machine_is_compatible("apple,t6030") ||
	      of_machine_is_compatible("apple,t6031") ||
	      of_machine_is_compatible("apple,t6032") ||
	      of_machine_is_compatible("apple,t6034")))
		return 0;

	pdev = platform_device_register_simple("cpuidle-apple", -1, NULL, 0);
	if (IS_ERR(pdev)) {
		platform_driver_unregister(&apple_cpuidle_driver);
		return PTR_ERR(pdev);
	}

	return 0;
}
device_initcall(apple_cpuidle_init);
