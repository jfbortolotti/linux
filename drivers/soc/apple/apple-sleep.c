// SPDX-License-Identifier: GPL-2.0-only OR MIT
/*
 * EXPERIMENTAL: Apple T8103 suspend-to-idle extras (port of m1n1's Level 3
 * battery test, m1n1 src/s2r.c, branch s2r-test).
 *
 * Linux s2idle already freezes tasks, suspends the devices and leaves every
 * CPU in cpuidle's deepest state (deep WFI). This driver adds, through
 * platform_s2idle_ops:
 *
 *  - a dump of every PMGR power state left on once all devices are suspended
 *    (noirq), to compare with what macOS / m1n1 leave on;
 *  - the m1n1 low-power steps, selected with the "lp" mask (same bits as the
 *    m1n1 battery test, s2r_test.py --lp):
 *      0x004  power domains as macOS leaves them (E2o)
 *      0x010  auto power states on memory/fabric/NUB (E2h)
 *      0x020  every power state as in the macOS sleep snapshot (E2p)
 *      0x040  APCIE_GP / DEBUG_USB / DEBUG_AUTH forced off (E2q)
 *      0x080  ATC0 + ATC1 (USB-C ports) off
 *      0x100  SMC sleep keys (MBSE='slpw', WKTP=2, MBSE='slpa')
 *      0x200  + RTKit SetAPPower(0x201 IDLE) to the SMC
 *    (0x001 DCP sleep, 0x002 NVMe and 0x008 secondaries have no meaning here:
 *    NVMe is suspended by its driver, the DCP is kept, the CPUs are cpuidle's.)
 *    Every register written is saved and restored, in reverse order, before
 *    the devices resume. Domains named in "skip" are never touched.
 *  - a lid wake from aop-gpio pin 22 (any edge, IRQF_NO_SUSPEND): after the
 *    SMC sleep keys the SMC sends no more button/lid notifications. The lid
 *    level at probe time is taken as "open"; s2idle ends when the lid is back
 *    open after it was seen closed (at entry or during the sleep).
 *
 * EXPERIMENTAL mem_sleep "deep" (apple_sleep.deep=1, then "deep" in /sys/power/mem_sleep):
 * the same steps as platform_suspend_ops; Linux takes the secondaries offline (CPU hotplug
 * through PMGR + m1n1, cpuidle-apple) and suspend_ops->enter power-gates the boot CPU as
 * s2idle_acc=2 does. Any unmasked interrupt ends the sleep, so the coprocessor mailbox IRQs
 * (IRQF_NO_SUSPEND) other than the SMC's are masked from prepare_late to wake.
 *
 * The parameters are writable at runtime, e.g.
 *   echo 0x2f4 > /sys/module/apple_sleep/parameters/lp
 * and everything is reported in dmesg after the resume.
 *
 * T8103 (M1) only: the tables come from macOS 13.6.2 hypervisor traces.
 */

#define pr_fmt(fmt) "apple-sleep: " fmt

#include <linux/bitfield.h>
#include <linux/bitmap.h>
#include <linux/bits.h>
#include <linux/ctype.h>
#include <linux/delay.h>
#include <clocksource/arm_arch_timer.h>
#include <linux/gpio/consumer.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/iopoll.h>
#include <linux/kthread.h>
#include <linux/irq.h>
#include <linux/irqdesc.h>
#include <linux/irqnr.h>
#include <linux/ktime.h>
#include <linux/math64.h>
#include <linux/mfd/macsmc.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <linux/of_platform.h>
#include <linux/platform_device.h>
#include <linux/pm_wakeup.h>
#include <linux/slab.h>
#include <linux/spmi.h>
#include <linux/soc/apple/mailbox.h>
#include <linux/soc/apple/rtkit.h>
#include <linux/soc/apple/sleep.h>
#include <linux/string.h>
#include <linux/suspend.h>

#define LP_DOMAINS	BIT(2)
#define LP_PS_AUTO	BIT(4)
#define LP_PS_SNAP	BIT(5)
#define LP_FORCE_OFF	BIT(6)
#define LP_ATC		BIT(7)
#define LP_SMC_HS	BIT(8)
#define LP_SMC_IDLE	BIT(9)
#define LP_PS_MASK	(LP_DOMAINS | LP_PS_AUTO | LP_PS_SNAP | LP_FORCE_OFF | LP_ATC)

/* default 0xf4: domains + auto + snapshot + force off + ATC, tested in s2idle 2026-11-11 */
static unsigned int lp = LP_DOMAINS | LP_PS_AUTO | LP_PS_SNAP | LP_FORCE_OFF | LP_ATC;
module_param(lp, uint, 0644);
MODULE_PARM_DESC(lp, "low-power steps (default 0xf4), m1n1 battery-test bits (0x4 domains, 0x10 auto, 0x20 snapshot, 0x40 force off, 0x80 ATC, 0x100 SMC keys, 0x200 SMC AP idle)");

static unsigned int dump = 1;
module_param(dump, uint, 0644);
MODULE_PARM_DESC(dump, "power-state dump at s2idle entry: 0 none, 1 domains not off, 2 all");

/*
 * DISP0_* / RMX / DISPEXT_*: kept on by the DCP driver.
 * FPWM0+FPWM2 off together hung the s2idle resume (each alone did not, 2026-11-10).
 * APCIE_GP (Wi-Fi PCIe) off lost the Wi-Fi once out of a few resumes.
 */
static char *skip = "DISP0_CPU0,DISP0_FE,RMX,DISPEXT_CPU0,DISPEXT_FE,FPWM0,FPWM2,APCIE_GP";
module_param(skip, charp, 0644);
MODULE_PARM_DESC(skip, "comma-separated power-state names never touched (case-insensitive)");

static char *only = "";
module_param(only, charp, 0644);
MODULE_PARM_DESC(only, "if set, comma-separated power-state names: only those are written (bisecting)");

static bool apply_test;
module_param(apply_test, bool, 0644);
MODULE_PARM_DESC(apply_test, "apply the lp power states, wait 100 ms, restore them at once, then sleep with nothing changed");

static bool dry_run;
module_param(dry_run, bool, 0644);
MODULE_PARM_DESC(dry_run, "log the lp writes that would be done in s2idle, write nothing");

/* both default on since 2026-11-11: the freezes that motivated these guards were posted
 * PMGR writes (fixed with ioremap_np), not the domains themselves */
static bool allow_aon = true;
module_param(allow_aon, bool, 0644);
MODULE_PARM_DESC(allow_aon, "allow lowering the target of DT apple,always-on domains (default on; ATC*_USB_AON)");

static bool allow_cg_off = true;
module_param(allow_cg_off, bool, 0644);
MODULE_PARM_DESC(allow_cg_off, "allow powering off domains left clock-gated (4) (default on)");

static bool lid_wake = true;
module_param(lid_wake, bool, 0644);
MODULE_PARM_DESC(lid_wake, "end s2idle on lid open (aop-gpio IRQ)");

/* PMGR power-state register */
#define PS_TARGET	GENMASK(3, 0)
#define PS_ACTUAL	GENMASK(7, 4)
#define PS_FLAGS	(BIT(9) | BIT(8))	/* WAS_CLKGATED / WAS_PWRGATED, never write 1 */
#define PS_AUTO		GENMASK(27, 24)
#define PS_AUTO_ENABLE	BIT(28)
#define PS_RESET	BIT(31)
#define PS_MODE_MASK	(PS_AUTO_ENABLE | PS_AUTO | PS_TARGET)

#define T8103_PMGR	0x23b700000ULL

struct ps_ent {
	const char *name;
	u64 addr;
	u32 val;
};

/* E2o: last value macOS writes to each PMGR power state before sleep (offsets in PMGR) */
static const struct ps_ent ps_macos_final[] = {
	{ "FPWM1", 0x001e0, 0x000003f0 },	{ "AVD_SYS", 0x00410, 0x04002340 },
	{ "AES", 0x00238, 0x000003f0 },		{ "ISP_CPU_CORE1", 0x04010, 0x00000000 },
	{ "ISP_CPU_CORE0", 0x04008, 0x00000000 }, { "ISPSENS3", 0x00198, 0x04000340 },
	{ "ISPSENS2", 0x00190, 0x04000340 },	{ "ISPSENS1", 0x00188, 0x04000340 },
	{ "ISPSENS0", 0x00180, 0x04000340 },	{ "ISP_SYS", 0x00400, 0x000023f0 },
	{ "MSR", 0x003c8, 0x04002340 },		{ "MMX", 0x00358, 0x04000340 },
	{ "ANE_SYS_CPU", 0x0c000, 0x00000300 },	{ "ANE_SYS", 0x00470, 0x0f0023f0 },
	{ "GFX", 0x003f8, 0x0f0023f0 },		{ "DISPEXT_CPU0", 0x00378, 0x0f0403f0 },
	{ "DISPEXT_FE", 0x00368, 0x000023f0 },	{ "DISP0_CPU0", 0x10018, 0x0f0403f0 },
	{ "DISP0_FE", 0x00360, 0x000023f0 },	{ "RMX", 0x00350, 0x00000300 },
	{ "SPI2", 0x00250, 0x000003f0 },	{ "I2C0", 0x001f0, 0x000003f0 },
	{ "SPI3", 0x00258, 0x000003f0 },	{ "I2C3", 0x00208, 0x000003f0 },
	{ "APCIE_ST", 0x00418, 0x0f0003f0 },	{ "SIO_CPU", 0x001d0, 0x0400024f },
};

/* E2h: AUTO_ENABLE/PS_AUTO as running macOS (only those bits are changed) */
static const struct ps_ent ps_auto[] = {
	{ "DCS0", 0x23b700300, 0x1000030f },	{ "DCS1", 0x23b700310, 0x1000030f },
	{ "DCS2", 0x23b700308, 0x1000030f },	{ "DCS3", 0x23b700318, 0x1000030f },
	{ "DCS4", 0x23b700320, 0x1000030f },	{ "DCS5", 0x23b700330, 0x1000030f },
	{ "DCS6", 0x23b700328, 0x1000030f },	{ "DCS7", 0x23b700338, 0x1000030f },
	{ "MCC", 0x23b7002f8, 0x1f0000ff },	{ "SBR", 0x23b700100, 0x1f0000ff },
	{ "IMX", 0x23b7001b8, 0x1f0000ff },	{ "SMX", 0x23b700340, 0x1f0000ff },
	{ "RMX", 0x23b700350, 0x1000030f },	{ "PMP", 0x23b7003d8, 0x1f0f03ff },
	{ "PMS", 0x23b700140, 0x1f0002ff },	{ "PMS_BUSIF", 0x23b700138, 0x1f0002ff },
	{ "PMS_SRAM", 0x23b7003e0, 0x1f0003ff }, { "NUB_AON", 0x23d280070, 0x1f0002ff },
	{ "NUB_FABRIC", 0x23d2800a8, 0x1f0003ff }, { "NUB_SRAM", 0x23d2800b0, 0x1000030f },
	{ "NUB_SPMI0", 0x23d280060, 0x1400024f }, { "NUB_SPMI1", 0x23d280068, 0x1400024f },
};

/* E2p: remaining differences with the macOS sleep snapshot (bits 28:24 and 3:0) */
static const struct ps_ent ps_snap[] = {
	{ "PMGR_SOC_OCLA", 0x23b700178, 0 },	{ "I2C1", 0x23b7001f8, 0 },
	{ "I2C2", 0x23b700200, 0 },		{ "I2C4", 0x23b700210, 0 },
	{ "SPI0", 0x23b700240, 0 },		{ "SPI1", 0x23b700248, 0 },
	{ "SPI4", 0x23b700260, 0 },		{ "MCA1", 0x23b7002c0, 0 },
	{ "DPA0", 0x23b7002e8, 0 },		{ "SIO_ADMA", 0x23b700230, 0 },
	{ "AFT0", 0x23b7001a8, 0 },		{ "DEVC0_IVDMC", 0x23b7001b0, 0 },
	{ "FPWM0", 0x23b7001d8, 0 },		{ "FPWM2", 0x23b7001e8, 0 },
	{ "PMS_FPWM0", 0x23b700148, 0 },	{ "PMS_FPWM1", 0x23b700150, 0 },
	{ "PMS_FPWM2", 0x23b700158, 0 },	{ "PMS_FPWM3", 0x23b700160, 0 },
	{ "PMS_FPWM4", 0x23b700168, 0 },	{ "SOC_SPMI0", 0x23b700118, 0 },
	{ "SOC_SPMI1", 0x23b700120, 0 },	{ "SOC_SPMI2", 0x23b700128, 0 },
	{ "MSG", 0x23d280078, 0 },		{ "RMX", 0x23b700350, 0x0f000000 },
	{ "PCIE_REF", 0x23b7001a0, 0x1400000f }, { "SIO_BUSIF", 0x23b7001c0, 0x1400000f },
	{ "SIO", 0x23b7001c8, 0x1f00000f },	{ "APCIE_GP", 0x23b7003e8, 0x1000000f },
	{ "ANS2", 0x23b7003f0, 0x1f00000f },	{ "DEBUG_USB", 0x23d2800b8, 0x1000000f },
	{ "DEBUG_AUTH", 0x23d2800c0, 0x1000000f },
};

/* E2q: auto domains that stay ACTIVE outside macOS, forced off */
static const struct ps_ent ps_force_off[] = {
	{ "APCIE_GP", 0x23b7003e8, 0 },
	{ "DEBUG_AUTH", 0x23d2800c0, 0 },
	{ "DEBUG_USB", 0x23d2800b8, 0 },
};

static const struct ps_ent ps_atc[] = {
	{ "ATC1_USB", 0x23d2800a0, 0 },		{ "ATC1_USB_AON", 0x23d280090, 0 },
	{ "ATC1_COMMON", 0x23b700448, 0 },	{ "ATC0_USB", 0x23d280098, 0 },
	{ "ATC0_USB_AON", 0x23d280088, 0 },	{ "ATC0_COMMON", 0x23b700420, 0 },
};

struct ps_map {
	u64 base, size;
	void __iomem *va;
};

struct ps_dom {
	const char *name;
	u64 addr;
	void __iomem *va;
	bool aon;	/* DT apple,always-on */
};

#define MAX_MAPS	4
#define MAX_SAVED	128
#define MAX_MASK_IRQS	32

static struct apple_sleep {
	struct device *dev;
	struct apple_smc *smc;
	struct device *smc_dev;

	struct ps_map maps[MAX_MAPS];
	int nmaps;
	struct ps_dom *doms;
	int ndoms;

	struct {
		const char *name;
		void __iomem *va;
		u32 old;
	} saved[MAX_SAVED];
	int nsaved;

	unsigned int lp_active;

	struct gpio_desc *lid;
	int lid_irq;
	void __iomem *lid_reg;	/* aop-gpio pin 22 config register (diagnostics) */
	int lid_open_level;
	bool armed;
	bool lid_seen_closed;
	bool lid_woke;
	unsigned int lid_irqs;

	unsigned int loops;
	ktime_t t_boot, t_mono;
	const char *mode;

	int mask_irq[MAX_MASK_IRQS];
	int nmask_irqs;
	int deep_ret;
} as;

static bool name_in(const char *p, const char *name)
{
	size_t n = strlen(name);

	while (p && *p) {
		const char *e = strchrnul(p, ','), *t = e;

		/* "echo a,b > param" leaves a newline; ignore blanks around names */
		while (p < t && isspace(*p))
			p++;
		while (t > p && isspace(t[-1]))
			t--;
		if (t - p == n && !strncasecmp(p, name, n))
			return true;
		p = *e ? e + 1 : e;
	}
	return false;
}

static bool name_skipped(const char *name)
{
	if (name_in(skip, name))
		return true;
	return only && *only && !name_in(only, name);
}

static void __iomem *ps_va(u64 addr)
{
	for (int i = 0; i < as.nmaps; i++)
		if (addr >= as.maps[i].base && addr + 4 <= as.maps[i].base + as.maps[i].size)
			return as.maps[i].va + (addr - as.maps[i].base);
	return NULL;
}

static bool ps_aon(void __iomem *va)
{
	for (int i = 0; i < as.ndoms; i++)
		if (as.doms[i].va == va)
			return as.doms[i].aon;
	return false;
}

static bool ps_saved(void __iomem *va)
{
	for (int i = 0; i < as.nsaved; i++)
		if (as.saved[i].va == va)
			return true;
	return false;
}

/*
 * Write one power state, remember the first value seen for the restore, and
 * wait (up to timeout_us) for the actual state to reach the target unless the
 * new value is an auto mode.
 */
/* set by the "poke" trigger: ps_write() logs, writes, restores at once, with pauses */
static bool poking;

static void ps_write(const struct ps_ent *e, u64 addr, u32 (*newval)(u32 cur, u32 val),
		     unsigned int timeout_us)
{
	void __iomem *va = ps_va(addr);
	u32 before, w, now;
	bool reached = true;

	if (!va) {
		pr_warn("  %-14s %#llx not mapped\n", e->name, addr);
		return;
	}
	if (name_skipped(e->name)) {
		pr_info("  %-14s skipped (%08x)\n", e->name, readl(va));
		return;
	}

	before = readl(va);
	w = newval(before, e->val) & ~PS_FLAGS;
	if (w == (before & ~PS_FLAGS)) {
		pr_info("  %-14s %08x unchanged\n", e->name, before);
		return;
	}
	/*
	 * Never power up what Linux turned off: SIO_CPU 0x0f000200 -> target 0xf
	 * (its parents off) hung the Mac in s2idle (2026-11-09). The macOS tables
	 * keep some coprocessors running; that is not ours to do.
	 */
	if (FIELD_GET(PS_TARGET, w) > FIELD_GET(PS_TARGET, before)) {
		pr_info("  %-14s %08x -> %08x would power up, not touched\n", e->name, before, w);
		return;
	}
	/*
	 * Linux drivers rely on the DT always-on domains keeping their state:
	 * ATC*_USB_AON off then on again hung the resume (dwc3, 2026-11-09).
	 */
	if (!allow_aon && ps_aon(va) &&
	    (FIELD_GET(PS_TARGET, w) < FIELD_GET(PS_TARGET, before) ||
	     /* a lower auto minimum lets the hardware gate it on its own */
	     ((w & PS_AUTO_ENABLE) && FIELD_GET(PS_AUTO, w) < FIELD_GET(PS_AUTO, before)))) {
		pr_info("  %-14s %08x -> %08x always-on in DT, not touched\n", e->name, before, w);
		return;
	}
	/*
	 * Domains left clock-gated (4) by the firmware, with no Linux user: AFT0
	 * 4 -> 0 froze the Mac even awake (2026-11-09), the next one likely too.
	 */
	if (!allow_cg_off && FIELD_GET(PS_ACTUAL, before) == 4 && FIELD_GET(PS_TARGET, w) < 4) {
		pr_info("  %-14s %08x -> %08x clock-gated, not powered off\n", e->name, before, w);
		return;
	}
	if (dry_run) {
		pr_info("  %-14s %08x -> %08x (dry run, not written)\n", e->name, before, w);
		return;
	}
	if (poking) {
		u32 now2;
		int bad;

		/* awake, a domain fully on is in use: not the s2idle situation */
		if (FIELD_GET(PS_ACTUAL, before) == 0xf && FIELD_GET(PS_TARGET, w) < 0xf) {
			pr_info("poke %-14s %08x on while awake, not poked\n", e->name, before);
			return;
		}
		pr_info("poke %-14s %08x -> %08x ...\n", e->name, before, w);
		msleep(3000);	/* Wi-Fi power save can hold the log line back > 1 s */
		writel(w, va);
		bad = !(w & PS_AUTO_ENABLE) &&
		      readl_poll_timeout(va, now, FIELD_GET(PS_ACTUAL, now) == FIELD_GET(PS_TARGET, w),
					 10, 20000);
		now = readl(va);
		msleep(100);
		writel(before & ~PS_FLAGS, va);
		msleep(20);
		now2 = readl(va);
		pr_info("poke %-14s   written: now %08x%s; restored: now %08x\n", e->name, now,
			bad ? " NOT REACHED" : "", now2);
		msleep(1000);
		return;
	}
	if (!ps_saved(va)) {
		if (as.nsaved >= MAX_SAVED) {
			pr_err("  %-14s save table full, not written\n", e->name);
			return;
		}
		as.saved[as.nsaved].name = e->name;
		as.saved[as.nsaved].va = va;
		as.saved[as.nsaved].old = before;
		as.nsaved++;
	}

	writel(w, va);
	if (!(w & PS_AUTO_ENABLE))
		reached = !readl_poll_timeout_atomic(va, now,
						     FIELD_GET(PS_ACTUAL, now) == FIELD_GET(PS_TARGET, w),
						     1, timeout_us);
	now = readl(va);
	pr_info("  %-14s %08x -> %08x (now %08x)%s\n", e->name, before, w, now,
		reached ? "" : " NOT REACHED");
}

/*
 * ps_macos_final values are register reads from the macOS trace, ACTUAL field
 * included: writing them whole (ACTUAL = 0xf into an off domain) locked the
 * Mac up (AES, 2026-11-09). Only the target field is taken from them.
 */
static u32 nv_target(u32 cur, u32 val) { return (cur & ~PS_TARGET) | (val & PS_TARGET); }
static u32 nv_auto(u32 cur, u32 val)
{
	return (cur & ~(PS_AUTO_ENABLE | PS_AUTO)) | (val & (PS_AUTO_ENABLE | PS_AUTO));
}
static u32 nv_mode(u32 cur, u32 val) { return (cur & ~(PS_MODE_MASK | PS_RESET)) | val; }

static void ps_apply(unsigned int mask)
{
	if (mask & LP_DOMAINS) {
		pr_info(" lp 0x004: power domains as macOS leaves them\n");
		for (int i = 0; i < ARRAY_SIZE(ps_macos_final); i++) {
			void __iomem *va = ps_va(T8103_PMGR + ps_macos_final[i].addr);

			/*
			 * Auto domains are gated by hardware/firmware (GFX: the GPU
			 * firmware still runs). Forcing one to 0 locked the Mac up
			 * (GFX, 2026-11-09); macOS stops its user first.
			 */
			if (va && (readl(va) & PS_AUTO_ENABLE)) {
				pr_info("  %-14s %08x auto, not touched\n", ps_macos_final[i].name,
					readl(va));
				continue;
			}
			ps_write(&ps_macos_final[i], T8103_PMGR + ps_macos_final[i].addr,
				 nv_target, 2000);
		}
	}
	if (mask & LP_PS_AUTO) {
		pr_info(" lp 0x010: auto power states (macOS values)\n");
		for (int i = 0; i < ARRAY_SIZE(ps_auto); i++)
			ps_write(&ps_auto[i], ps_auto[i].addr, nv_auto, 2000);
	}
	if (mask & LP_PS_SNAP) {
		pr_info(" lp 0x020: power states as in the macOS snapshot\n");
		for (int i = 0; i < ARRAY_SIZE(ps_snap); i++)
			ps_write(&ps_snap[i], ps_snap[i].addr, nv_mode, 2000);
	}
	if (mask & LP_FORCE_OFF) {
		pr_info(" lp 0x040: stuck auto domains forced off\n");
		for (int i = 0; i < ARRAY_SIZE(ps_force_off); i++)
			ps_write(&ps_force_off[i], ps_force_off[i].addr, nv_mode, 5000);
	}
	if (mask & LP_ATC) {
		pr_info(" lp 0x080: ATC0/ATC1 off\n");
		for (int i = 0; i < ARRAY_SIZE(ps_atc); i++)
			ps_write(&ps_atc[i], ps_atc[i].addr, nv_mode, 2000);
	}
}

static void ps_restore(void)
{
	int bad = 0;

	for (int i = as.nsaved; i-- > 0;) {
		u32 w = as.saved[i].old & ~PS_FLAGS, now;	/* RESET kept as it was */

		writel(w, as.saved[i].va);
		if (!(w & PS_AUTO_ENABLE) &&
		    readl_poll_timeout_atomic(as.saved[i].va, now,
					      FIELD_GET(PS_ACTUAL, now) == FIELD_GET(PS_TARGET, w),
					      1, 5000)) {
			pr_err("restore %-14s %08x: now %08x, NOT REACHED\n", as.saved[i].name, w,
			       readl(as.saved[i].va));
			bad++;
		}
	}
	pr_info("%d power states restored, %d not reached\n", as.nsaved, bad);
	as.nsaved = 0;
}

static void ps_dump(unsigned int level)
{
	int on = 0;

	pr_info("power states with all devices suspended (%s):\n",
		level >= 2 ? "all" : "not off");
	for (int i = 0; i < as.ndoms; i++) {
		u32 v = readl(as.doms[i].va);

		if (FIELD_GET(PS_ACTUAL, v))
			on++;
		/* level 1: also the ones off but with a non-zero target (parent off, auto...) */
		if (level < 2 && !FIELD_GET(PS_ACTUAL, v) && !FIELD_GET(PS_TARGET, v) &&
		    !(v & PS_RESET))
			continue;
		pr_info("  ps %-16s %#llx %08x target %lu actual %lu%s%s\n", as.doms[i].name,
			as.doms[i].addr, v, FIELD_GET(PS_TARGET, v), FIELD_GET(PS_ACTUAL, v),
			(v & PS_AUTO_ENABLE) ? " auto" : "", (v & PS_RESET) ? " RESET" : "");
	}
	pr_info("%d of %d power states not off\n", on, as.ndoms);
}

/*
 * Read-only view of the DRAM-side power states (DCS0-7 = DRAM interfaces, MCC, fabric, PMP)
 * at s2idle entry (after the drivers, PMP sleep messages included) and first thing on wake:
 * actual (7:4), auto minimum (27:24) and the sticky WAS_CLKGATED/WAS_PWRGATED flags (9:8).
 * Nothing is written: a flag that reads 0 at entry and 1 at wake means the domain gated
 * during the sleep. A flag already 1 at entry says nothing (it is never cleared here).
 */
static const struct { const char *name; u64 addr; } mem_ps_regs[] = {
	{ "DCS0", 0x23b700300 }, { "DCS1", 0x23b700310 }, { "DCS2", 0x23b700308 },
	{ "DCS3", 0x23b700318 }, { "DCS4", 0x23b700320 }, { "DCS5", 0x23b700330 },
	{ "DCS6", 0x23b700328 }, { "DCS7", 0x23b700338 }, { "MCC", 0x23b7002f8 },
	{ "SBR", 0x23b700100 }, { "SMX", 0x23b700340 }, { "PMP", 0x23b7003d8 },
};
static u32 mem_ps_entry[ARRAY_SIZE(mem_ps_regs)];

static bool mem_ps = true;
module_param(mem_ps, bool, 0644);
MODULE_PARM_DESC(mem_ps, "log DCS0-7/MCC/SBR/SMX/PMP power states at s2idle entry and wake (read only, default on)");

static void mem_ps_log(const char *tag, bool wake)
{
	char line[512];
	int len = 0;

	for (int i = 0; i < ARRAY_SIZE(mem_ps_regs); i++) {
		void __iomem *va = ps_va(mem_ps_regs[i].addr);
		u32 v = va ? readl(va) : 0;
		bool gated = wake && (v & ~mem_ps_entry[i] & PS_FLAGS);

		if (!wake)
			mem_ps_entry[i] = v;
		len += scnprintf(line + len, sizeof(line) - len, " %s=%08x%s", mem_ps_regs[i].name,
				 v, gated ? "*" : "");
	}
	pr_info("mem ps %s:%s\n", tag, line);
}

static void smc_sleep_keys(void)
{
	int r1, r2, r3;

	r1 = apple_smc_write_u32(as.smc, SMC_KEY(MBSE), 0x736c7077);	/* 'slpw' */
	r2 = apple_smc_write_u8(as.smc, SMC_KEY(WKTP), 2);
	r3 = apple_smc_write_u32(as.smc, SMC_KEY(MBSE), 0x736c7061);	/* 'slpa' */
	pr_info("SMC sleep keys: MBSE=slpw %d, WKTP=2 %d, MBSE=slpa %d\n", r1, r2, r3);

	if (lp & LP_SMC_IDLE)
		pr_info("SMC SetAPPower(0x201): %d\n", apple_rtkit_set_ap_power(as.smc->rtk, 0x201));
}


/*
 * Awake bisecting aid: "echo 0x10 > .../poke" runs the lp steps of that mask
 * through the same ps_write() checks as s2idle, but one entry at a time: log,
 * write, restore 100 ms later, log, 1 s pause, so that "dmesg -w" over ssh
 * shows the entry being written when the Mac locks up. "1" means 0x04.
 */
static int poke_set(const char *val, const struct kernel_param *kp)
{
	unsigned int mask;
	int ret;

	if (!as.dev)
		return -ENODEV;
	ret = kstrtouint(val, 0, &mask);
	if (ret)
		return ret;
	if (mask == 1)
		mask = LP_DOMAINS;
	pr_info("poke: lp %#x, one entry at a time (awake)\n", mask);
	poking = true;
	ps_apply(mask & LP_PS_MASK);
	poking = false;
	pr_info("poke: done\n");
	return 0;
}

static const struct kernel_param_ops poke_ops = {
	.set = poke_set,
};
module_param_cb(poke, &poke_ops, NULL, 0200);
MODULE_PARM_DESC(poke, "write an lp mask: try its power-state writes one by one while awake (logged)");

/*
 * SMC power meter: battery gauge and power keys read at s2idle entry (.prepare) and exit
 * (.restore), plus "cat /sys/module/apple_sleep/parameters/meter" for a reading at any time.
 * BAAC (si64) looks like the gauge's accumulated charge (coulomb counter): if so, its change
 * over a suspend measures the sleep charge far finer than B0RM (1 mAh). Units to be confirmed
 * against B0AC (mA) while awake.
 */
static bool smc_meter = true;
module_param(smc_meter, bool, 0644);
MODULE_PARM_DESC(smc_meter, "log SMC battery/power keys at s2idle entry and exit (default on)");

struct as_meter {
	ktime_t t;
	s64 baac;
	s32 b0ap;
	s16 b0ac;
	u16 b0av, b0rm;
	int pstr, phpc, pmvr, psvr, ppmc, ppsc;	/* mW */
	bool ok;
};

static struct as_meter meter_entry;

/* SMC 'flt ' (IEEE754 single) -> value * 1000, as macsmc-power's apple_smc_read_f32_scaled */
static int smc_read_f32_milli(smc_key key, int *p)
{
	u32 f;
	u64 v;
	int exp, ret = apple_smc_read_u32(as.smc, key, &f);

	*p = 0;
	if (ret < 0)
		return ret;
	if (!(f & 0x7fffffff))
		return 0;
	v = (u64)((f & 0x7fffff) | BIT(23)) * 1000;
	exp = (int)((f >> 23) & 0xff) - 127 - 23;
	if (exp <= -64)
		v = 0;
	else if (exp < 0)
		v >>= -exp;
	else if (exp > 8)
		v = INT_MAX;
	else
		v <<= exp;
	v = min_t(u64, v, INT_MAX);
	*p = (f & BIT(31)) ? -(int)v : (int)v;
	return 0;
}

static void meter_read(struct as_meter *m)
{
	memset(m, 0, sizeof(*m));
	m->t = ktime_get_boottime();
	if (!as.smc)
		return;
	m->ok = !apple_smc_read_s64(as.smc, SMC_KEY(BAAC), &m->baac);
	apple_smc_read_s16(as.smc, SMC_KEY(B0AC), &m->b0ac);
	apple_smc_read_s32(as.smc, SMC_KEY(B0AP), &m->b0ap);
	apple_smc_read_u16(as.smc, SMC_KEY(B0AV), &m->b0av);
	apple_smc_read_u16(as.smc, SMC_KEY(B0RM), &m->b0rm);
	smc_read_f32_milli(SMC_KEY(PSTR), &m->pstr);
	smc_read_f32_milli(SMC_KEY(PHPC), &m->phpc);
	smc_read_f32_milli(SMC_KEY(PMVR), &m->pmvr);
	smc_read_f32_milli(SMC_KEY(PSVR), &m->psvr);
	smc_read_f32_milli(SMC_KEY(PPMC), &m->ppmc);
	smc_read_f32_milli(SMC_KEY(PPSC), &m->ppsc);
}

static int meter_fmt(char *buf, size_t size, const struct as_meter *m)
{
	return scnprintf(buf, size,
			 "t %lld ms BAAC %lld%s B0AC %d mA B0AP %d mW B0AV %u mV B0RM %u mAh | "
			 "PSTR %d PHPC %d PMVR %d PSVR %d PPMC %d PPSC %d mW\n",
			 ktime_to_ms(m->t), m->baac, m->ok ? "" : "(err)", m->b0ac, m->b0ap,
			 m->b0av, m->b0rm, m->pstr, m->phpc, m->pmvr, m->psvr, m->ppmc, m->ppsc);
}

static int meter_get(char *buf, const struct kernel_param *kp)
{
	struct as_meter m;

	if (!as.smc)
		return scnprintf(buf, PAGE_SIZE, "no SMC\n");
	meter_read(&m);
	return meter_fmt(buf, PAGE_SIZE, &m);
}

static const struct kernel_param_ops meter_ops = {
	.get = meter_get,
};
module_param_cb(meter, &meter_ops, NULL, 0444);
MODULE_PARM_DESC(meter, "read: one SMC battery/power reading (BAAC, B0AC, B0AP, PSTR, rails)");

static void meter_log(const char *tag, const struct as_meter *m)
{
	char line[256];

	meter_fmt(line, sizeof(line), m);
	pr_info("meter %s: %s", tag, line);
}

static void meter_exit(void)
{
	struct as_meter m;
	s64 dt_ms, dbaac;

	meter_read(&m);
	meter_log("exit ", &m);
	dt_ms = ktime_ms_delta(m.t, meter_entry.t);
	if (!meter_entry.ok || !m.ok || dt_ms <= 0)
		return;
	dbaac = m.baac - meter_entry.baac;
	pr_info("meter: %lld.%03lld s between readings, BAAC %+lld (%+lld per hour), B0RM %+d mAh\n",
		dt_ms / 1000, dt_ms % 1000, dbaac, div64_s64(dbaac * 3600000, dt_ms),
		(int)m.b0rm - (int)meter_entry.b0rm);
}

static void lid_reg_log(const char *tag);

static int as_s2idle_begin(void)
{
	as.mode = "s2idle";
	as.loops = 0;
	as.lid_irqs = 0;
	as.lid_woke = false;
	as.lp_active = lp;
	return 0;
}

/*
 * AOP sleep (macOS traces 10 and 11, 2026-11-14). After the drivers suspend, macOS sends every
 * AFK endpoint 0xa4 (ack 0x87), in the order 0x21 0x22 0x26, 0x20 0x28, 0x24 0x25, 0x27, then
 * RTKit SetAPPower(0x201) (183.36-183.46 s). Before that it sends one EPIC message (0xa2) each to
 * 0x24 and 0x25, whose ring content is not in the traces: skipped. The resume side was never
 * traced (macOS does not wake under the HV), so it is a guess behind its own bit. aop_sleep mask:
 *   0x1  0xa4 to each AFK endpoint the AOP announced
 *   0x2  then SetAPPower(0x201); SetAPPower(0x20) at resume
 *   0x4  at resume, 0xa3 (AFK START, ack 0x86) to each endpoint that took 0xa4
 * Runs in .prepare (after dpm_suspend_late, mailbox IRQs on) and .restore (before the AOP users,
 * ALS and audio, resume), for s2idle and deep. aop_poke=<mask> runs it awake, resume after 3 s.
 */
static unsigned int aop_sleep = 0x7;
module_param(aop_sleep, uint, 0644);
MODULE_PARM_DESC(aop_sleep, "AOP sleep as macOS: 0x1 AFK 0xa4 per endpoint, 0x2 SetAPPower 0x201, 0x4 AFK START at resume (default 0x7, 2026-11-14)");

static const u8 aop_sleep_eps[] = { 0x21, 0x22, 0x26, 0x20, 0x28, 0x24, 0x25, 0x27 };
static struct apple_rtkit *aop_rtk;
static u32 aop_eps_done;	/* BIT(ep - 0x20) */
static bool aop_idled;

/*
 * The AOP's RTKit belongs to the Rust apple_aop driver; the RTKit core sets itself as the cookie
 * of its mailbox (rtk->mbox->cookie = rtk), so take it from the AOP mailbox's drvdata.
 * Only valid while apple_aop stays bound: looked up again at every suspend.
 */
static struct apple_rtkit *aop_find_rtk(void)
{
	struct device_node *np = of_find_compatible_node(NULL, NULL, "apple,t8103-aop");
	struct device_node *mb = np ? of_parse_phandle(np, "mboxes", 0) : NULL;
	struct platform_device *aop = np ? of_find_device_by_node(np) : NULL;
	struct platform_device *mpdev = mb ? of_find_device_by_node(mb) : NULL;
	struct apple_rtkit *rtk = NULL;

	if (aop && aop->dev.driver && mpdev) {
		struct apple_mbox *mbox = platform_get_drvdata(mpdev);

		if (mbox && mbox->active)
			rtk = mbox->cookie;
	}
	if (mpdev)
		put_device(&mpdev->dev);
	if (aop)
		put_device(&aop->dev);
	of_node_put(mb);
	of_node_put(np);
	return rtk;
}

static void aop_sleep_enter(unsigned int mask)
{
	int ret = 0;

	aop_eps_done = 0;
	aop_idled = false;
	aop_rtk = NULL;
	if (!(mask & 0x3))
		return;
	aop_rtk = aop_find_rtk();
	if (!aop_rtk) {
		pr_warn("aop_sleep: AOP RTKit not found (apple_aop not bound?)\n");
		return;
	}
	if (mask & 0x1) {
		for (unsigned int i = 0; i < ARRAY_SIZE(aop_sleep_eps); i++) {
			u8 ep = aop_sleep_eps[i];

			if (!apple_rtkit_has_endpoint(aop_rtk, ep))
				continue;
			ret = apple_rtkit_send_message(aop_rtk, ep, 0xa4ULL << 48, NULL, false);
			if (ret)
				pr_warn("aop_sleep: ep %#x 0xa4: %d\n", ep, ret);
			else
				aop_eps_done |= BIT(ep - 0x20);
		}
		msleep(100);	/* the 0x87 acks (logged by apple_aop) take up to ~50 ms */
	}
	if (mask & 0x2) {
		ret = apple_rtkit_set_ap_power(aop_rtk, 0x201);
		aop_idled = !ret;
	}
	if (mask & 0x2)
		pr_info("aop_sleep: 0xa4 sent to eps %#x (bit = ep - 0x20), SetAPPower(0x201): %d\n",
			aop_eps_done, ret);
	else
		pr_info("aop_sleep: 0xa4 sent to eps %#x (bit = ep - 0x20), no SetAPPower\n", aop_eps_done);
}

static void aop_sleep_exit(unsigned int mask)
{
	if (!aop_rtk)
		return;
	if (aop_idled)
		pr_info("aop_sleep: SetAPPower(0x20): %d\n", apple_rtkit_set_ap_power(aop_rtk, 0x20));
	if (mask & 0x4) {
		for (unsigned int ep = 0x20; ep < 0x40; ep++) {
			int ret;

			if (!(aop_eps_done & BIT(ep - 0x20)))
				continue;
			ret = apple_rtkit_send_message(aop_rtk, ep, 0xa3ULL << 48, NULL, false);
			if (ret)
				pr_warn("aop_sleep: ep %#x 0xa3: %d\n", ep, ret);
		}
		pr_info("aop_sleep: 0xa3 (AFK START) sent to eps %#x\n", aop_eps_done);
	}
	aop_rtk = NULL;
	aop_idled = false;
	aop_eps_done = 0;
}

static int aop_poke_set(const char *val, const struct kernel_param *kp)
{
	unsigned int mask;
	int ret = kstrtouint(val, 0, &mask);

	if (ret)
		return ret;
	if (!mask || (mask & ~0x07u))
		return -EINVAL;
	pr_info("aop_poke: mask %#x awake\n", mask);
	aop_sleep_enter(mask);
	msleep(3000);
	pr_info("aop_poke: resuming\n");
	aop_sleep_exit(mask);
	pr_info("aop_poke: done\n");
	return 0;
}

static const struct kernel_param_ops aop_poke_ops = {
	.set = aop_poke_set,
};
module_param_cb(aop_poke, &aop_poke_ops, NULL, 0200);
MODULE_PARM_DESC(aop_poke, "awake: run aop_sleep <mask>, resume after 3 s (logged)");

/* After dpm_suspend_late(): device IRQs still on, so the SMC can still be talked to */
static int as_s2idle_prepare(void)
{
	if (as.lid) {
		int lvl = gpiod_get_value(as.lid);

		as.lid_seen_closed = lvl != as.lid_open_level;
		pr_info("lid level %d at entry (%s)\n", lvl, as.lid_seen_closed ? "closed" : "open");
		lid_reg_log("at entry");
	}
	if (smc_meter) {
		meter_read(&meter_entry);
		meter_log("entry", &meter_entry);
	}
	if (as.lp_active & (LP_SMC_HS | LP_SMC_IDLE))
		smc_sleep_keys();
	aop_sleep_enter(aop_sleep);
	as.armed = true;
	return 0;
}

/* After dpm_suspend_noirq(): genpd has turned off what Linux turns off */
static int as_s2idle_prepare_late(void)
{
	if (dump)
		ps_dump(dump);
	if (mem_ps)
		mem_ps_log("entry", false);
	if (as.lp_active & LP_PS_MASK) {
		pr_info("applying lp %#x%s%s%s%s\n", as.lp_active, dry_run ? " (DRY RUN)" : "", only && *only ? ", only " : "", only && *only ? only : "",
			apply_test ? " (apply_test: restored after 100 ms)" : "");
		ps_apply(as.lp_active);
		if (apply_test) {
			mdelay(100);
			ps_restore();
			if (dump)
				ps_dump(dump);
		}
	}
	as.t_boot = ktime_get_boottime();
	as.t_mono = ktime_get();
	return 0;
}

static void as_s2idle_check(void)
{
	as.loops++;
}

/* Before dpm_resume_noirq(): power back what we turned off before any driver runs */
static void as_s2idle_restore_early(void)
{
	s64 slept = ktime_ms_delta(ktime_get_boottime(), as.t_boot);
	s64 awake = ktime_ms_delta(ktime_get(), as.t_mono);

	as.armed = false;
	if (mem_ps)
		mem_ps_log("wake ", true);
	if (as.nsaved)
		ps_restore();
	pr_info("%s: %lld.%03lld s, of which awake %lld ms, %u wakeups, wake IRQ %u%s, lid IRQs %u\n",
		as.mode, slept / 1000, slept % 1000, awake, as.loops ? as.loops - 1 : 0, pm_wakeup_irq(),
		as.lid_woke ? " (lid open)" : "", as.lid_irqs);
}

static void as_s2idle_restore(void)
{
	/* after dpm_resume_noirq(): device IRQs on again, the SMC answers; drivers not resumed yet */
	aop_sleep_exit(aop_sleep);
	if (smc_meter)
		meter_exit();
	if (as.lp_active & LP_SMC_IDLE)
		pr_info("SMC SetAPPower(0x20): %d\n", apple_rtkit_set_ap_power(as.smc->rtk, 0x20));
	if (as.lp_active & (LP_SMC_HS | LP_SMC_IDLE))
		pr_info("SMC NTAP=1: %d\n", apple_smc_write_flag(as.smc, SMC_KEY(NTAP), true));
}

static const struct platform_s2idle_ops as_s2idle_ops = {
	.begin		= as_s2idle_begin,
	.prepare	= as_s2idle_prepare,
	.prepare_late	= as_s2idle_prepare_late,
	.check		= as_s2idle_check,
	.restore_early	= as_s2idle_restore_early,
	.restore	= as_s2idle_restore,
};

/*
 * PMGR wake sources (HV trace 11, 2026-11-14). macOS programs them once at boot (0.80 s) and
 * never at sleep:
 *   AON PMGR 0x23d2b4100 + 0x10 * i, i = 0..42: 1 (i = 34, 36, 38, 40, 42: 3), then
 *   0x23d2b4000 = 0x30001,
 *   then the wake-source masks 0x23d004008..0x23d004018 (one bit per ADT pmgr ap-wake-sources
 *   number, 0 = enabled): exactly the 17 ADT sources (12/14/26/27 AOP, 40/47/49/50/54 NUB,
 *   64 AOP SPMI, 76/81 SMC, 131-134 ATC) are 0.
 * With the SMC told MBSE='slpw', the power button only reaches the AP through source 76
 * (SMC.CPUWakeupAP), so a deep sleep with slpw needs this arming. wake_log logs the registers
 * at probe (read only); wake_arm=1 writes the macOS values at deep entry and puts the previous
 * ones back at the end of the resume.
 */
#define WAKE_MASK_PHYS	0x23d004000ULL
#define WAKE_MASK_OFF	0x08
#define WAKE_MASK_N	5
#define WAKE_TBL_PHYS	0x23d2b4000ULL
#define WAKE_TBL_CTRL	0x000
#define WAKE_TBL_OFF	0x100
#define WAKE_TBL_N	43

static const u32 wake_mask_macos[WAKE_MASK_N] = {
	0xf3ffafff, 0xffb97eff, 0xfffdeffe, 0xffffffff, 0x03ffff87,
};

static bool wake_arm;
module_param(wake_arm, bool, 0644);
MODULE_PARM_DESC(wake_arm, "deep: write the macOS PMGR wake-source table + masks (0x23d2b4100, 0x23d004008) at entry, restore at resume");

static void __iomem *wake_mask_va, *wake_tbl_va;
static u32 wake_mask_saved[WAKE_MASK_N], wake_tbl_saved[WAKE_TBL_N], wake_ctrl_saved;
static bool wake_armed;

static u32 wake_tbl_macos(int i)
{
	return (i >= 34 && !(i & 1)) ? 3 : 1;
}

static void wake_log(const char *tag)
{
	char line[256];
	int len = 0, ndiff = 0;

	if (!wake_mask_va || !wake_tbl_va)
		return;
	for (int i = 0; i < WAKE_MASK_N; i++)
		len += scnprintf(line + len, sizeof(line) - len, " %08x",
				 readl(wake_mask_va + WAKE_MASK_OFF + 4 * i));
	for (int i = 0; i < WAKE_TBL_N; i++)
		if (readl(wake_tbl_va + WAKE_TBL_OFF + 0x10 * i) != wake_tbl_macos(i))
			ndiff++;
	pr_info("wake sources %s: masks%s, table ctrl %08x, %d of %d entries differ from macOS\n", tag,
		line, readl(wake_tbl_va + WAKE_TBL_CTRL), ndiff, WAKE_TBL_N);
}

static void wake_arm_now(void)
{
	if (!wake_mask_va || !wake_tbl_va)
		return;
	wake_log("before arming");
	wake_ctrl_saved = readl(wake_tbl_va + WAKE_TBL_CTRL);
	for (int i = 0; i < WAKE_TBL_N; i++) {
		wake_tbl_saved[i] = readl(wake_tbl_va + WAKE_TBL_OFF + 0x10 * i);
		writel(wake_tbl_macos(i), wake_tbl_va + WAKE_TBL_OFF + 0x10 * i);
	}
	writel(0x30001, wake_tbl_va + WAKE_TBL_CTRL);
	for (int i = 0; i < WAKE_MASK_N; i++) {
		wake_mask_saved[i] = readl(wake_mask_va + WAKE_MASK_OFF + 4 * i);
		writel(wake_mask_macos[i], wake_mask_va + WAKE_MASK_OFF + 4 * i);
	}
	wake_armed = true;
	wake_log("armed (macOS values)");
}

static void wake_restore(void)
{
	if (!wake_armed)
		return;
	for (int i = 0; i < WAKE_MASK_N; i++)
		writel(wake_mask_saved[i], wake_mask_va + WAKE_MASK_OFF + 4 * i);
	for (int i = 0; i < WAKE_TBL_N; i++)
		writel(wake_tbl_saved[i], wake_tbl_va + WAKE_TBL_OFF + 0x10 * i);
	writel(wake_ctrl_saved, wake_tbl_va + WAKE_TBL_CTRL);
	wake_armed = false;
	wake_log("restored");
}

/*
 * Awake wake-path probe (2026-11-14): deep with slpw + wake_arm did not wake, so find where
 * the SMC wake (SMC.CPUWakeupAP 76) lands while the SoC is still up. wake_watch=N polls the
 * PMGR[4] page (0x23d004000, wake masks + presumably status), the AON page 0x23d2b0000 and
 * 0x23d2c4000 every ~2 ms for N seconds (read only) and logs every register change; a 300 ms
 * calibration first leaves out the registers that move by themselves (0x23d2b0000/10/20 are
 * ~24 MHz counters, 2026-11-14).
 * smc_mbse=slpw (4 chars) writes the SMC key MBSE awake, as macOS does at sleep entry.
 */
static const struct { phys_addr_t pa; u32 len; } watch_rng[] = {
	{ 0x23d004000ULL, 0x1000 },
	{ 0x23d2b0000ULL, 0x100 },
	{ 0x23d2c4000ULL, 0x100 },
};
#define WATCH_NRNG	ARRAY_SIZE(watch_rng)
#define WATCH_MAXLOG	300
#define WATCH_CALIB_MS	300

static struct task_struct *watch_task;
static bool watch_clear;
module_param(watch_clear, bool, 0644);
MODULE_PARM_DESC(watch_clear, "wake_watch: write 1 to the 0x23d2c4000 latch before polling");

static int wake_watch_fn(void *arg)
{
	unsigned long secs = (unsigned long)arg;
	void __iomem *va[WATCH_NRNG] = {};
	u32 *old[WATCH_NRNG] = {};
	unsigned long *noisy[WATCH_NRNG] = {};
	ktime_t t0, end, calib;
	int nlog = 0, rounds = 0, nnoisy = 0;

	for (int r = 0; r < WATCH_NRNG; r++) {
		va[r] = ioremap_np(watch_rng[r].pa, watch_rng[r].len);
		old[r] = kmalloc(watch_rng[r].len, GFP_KERNEL);
		noisy[r] = bitmap_zalloc(watch_rng[r].len / 4, GFP_KERNEL);
		if (!va[r] || !old[r] || !noisy[r])
			goto out;
		for (u32 o = 0; o < watch_rng[r].len; o += 4)
			old[r][o / 4] = readl(va[r] + o);
	}

	/* calibration: registers that change by themselves (counters, timers) are left out */
	calib = ktime_add_ms(ktime_get(), WATCH_CALIB_MS);
	while (!kthread_should_stop() && ktime_before(ktime_get(), calib)) {
		for (int r = 0; r < WATCH_NRNG; r++)
			for (u32 o = 0; o < watch_rng[r].len; o += 4) {
				u32 v = readl(va[r] + o);

				if (v != old[r][o / 4] && !test_bit(o / 4, noisy[r])) {
					set_bit(o / 4, noisy[r]);
					nnoisy++;
					pr_info("wake_watch: %09llx changes by itself (%08x -> %08x), ignored\n",
						(u64)watch_rng[r].pa + o, old[r][o / 4], v);
				}
				old[r][o / 4] = v;
			}
		usleep_range(1500, 2500);
	}

	/* 0x23d2c4000 reads 1 before macOS-style "write 1" and 0 after (2026-11-14, T1a): a latch */
	pr_info("wake_watch: 23d2c4000 = %08x%s\n", readl(va[2]), watch_clear ? ", clearing (write 1)" : "");
	if (watch_clear) {
		writel(1, va[2]);
		old[2][0] = readl(va[2]);
		pr_info("wake_watch: 23d2c4000 now %08x\n", old[2][0]);
	}
	t0 = ktime_get();
	end = ktime_add_ms(t0, secs * 1000);
	pr_info("wake_watch: polling for %lu s (%d registers ignored)\n", secs, nnoisy);
	while (!kthread_should_stop() && ktime_before(ktime_get(), end)) {
		for (int r = 0; r < WATCH_NRNG; r++) {
			for (u32 o = 0; o < watch_rng[r].len; o += 4) {
				u32 v;

				if (test_bit(o / 4, noisy[r]))
					continue;
				v = readl(va[r] + o);
				if (v == old[r][o / 4])
					continue;
				if (nlog++ < WATCH_MAXLOG)
					pr_info("wake_watch: +%lld ms %09llx: %08x -> %08x\n",
						ktime_ms_delta(ktime_get(), t0),
						(u64)watch_rng[r].pa + o, old[r][o / 4], v);
				old[r][o / 4] = v;
			}
		}
		rounds++;
		usleep_range(1500, 2500);
	}
	pr_info("wake_watch: done, %d rounds, %d changes%s\n", rounds, nlog,
		nlog > WATCH_MAXLOG ? " (log capped)" : "");
out:
	for (int r = 0; r < WATCH_NRNG; r++) {
		if (va[r])
			iounmap(va[r]);
		kfree(old[r]);
		bitmap_free(noisy[r]);
	}
	watch_task = NULL;
	return 0;
}

static int wake_watch_set(const char *val, const struct kernel_param *kp)
{
	unsigned long secs;
	struct task_struct *t;
	int ret = kstrtoul(val, 0, &secs);

	if (ret)
		return ret;
	if (watch_task)
		return -EBUSY;
	if (!secs || secs > 600)
		return -EINVAL;
	t = kthread_run(wake_watch_fn, (void *)secs, "apple-sleep-watch");
	if (IS_ERR(t))
		return PTR_ERR(t);
	watch_task = t;
	return 0;
}

static const struct kernel_param_ops wake_watch_ops = {
	.set = wake_watch_set,
};
module_param_cb(wake_watch, &wake_watch_ops, NULL, 0200);
MODULE_PARM_DESC(wake_watch, "awake: poll the PMGR wake/AON registers for N s (max 600) and log every change");

static int smc_mbse_set(const char *val, const struct kernel_param *kp)
{
	char k[4];
	size_t n = strcspn(val, "\n");
	int ret;

	if (!as.smc)
		return -ENODEV;
	if (n != 4)
		return -EINVAL;
	memcpy(k, val, 4);
	ret = apple_smc_write_u32(as.smc, SMC_KEY(MBSE),
				  (u32)k[0] << 24 | (u32)k[1] << 16 | (u32)k[2] << 8 | k[3]);
	pr_info("awake: SMC MBSE='%.4s': %d\n", k, ret);
	return ret;
}

static const struct kernel_param_ops smc_mbse_ops = {
	.set = smc_mbse_set,
};
module_param_cb(smc_mbse, &smc_mbse_ops, NULL, 0200);
MODULE_PARM_DESC(smc_mbse, "awake: write the SMC key MBSE (4 chars, e.g. slpw)");

/* awake helpers for the wake-path tests (2026-11-14) */
static int smc_wktp_set(const char *val, const struct kernel_param *kp)
{
	u8 v;
	int ret = kstrtou8(val, 0, &v);

	if (ret)
		return ret;
	if (!as.smc)
		return -ENODEV;
	ret = apple_smc_write_u8(as.smc, SMC_KEY(WKTP), v);
	pr_info("awake: SMC WKTP=%u: %d\n", v, ret);
	return ret;
}

static const struct kernel_param_ops smc_wktp_ops = {
	.set = smc_wktp_set,
};
module_param_cb(smc_wktp, &smc_wktp_ops, NULL, 0200);
MODULE_PARM_DESC(smc_wktp, "awake: write the SMC key WKTP (u8, macOS writes 2)");

static int smc_ap_power_set(const char *val, const struct kernel_param *kp)
{
	u32 v;
	int ret = kstrtou32(val, 0, &v);

	if (ret)
		return ret;
	if (!as.smc)
		return -ENODEV;
	if (v != 0x201 && v != 0x20)
		return -EINVAL;
	ret = apple_rtkit_set_ap_power(as.smc->rtk, v);
	pr_info("awake: SMC RTKit SetAPPower(%#x): %d\n", v, ret);
	return ret;
}

static const struct kernel_param_ops smc_ap_power_ops = {
	.set = smc_ap_power_set,
};
module_param_cb(smc_ap_power, &smc_ap_power_ops, NULL, 0200);
MODULE_PARM_DESC(smc_ap_power, "awake: SMC RTKit SetAPPower, 0x201 (AP idle, macOS after slpa) or 0x20 (back on)");

static int wake_arm_awake_set(const char *val, const struct kernel_param *kp)
{
	bool on;
	int ret = kstrtobool(val, &on);

	if (ret)
		return ret;
	if (on && !wake_armed)
		wake_arm_now();
	else if (!on)
		wake_restore();
	return 0;
}

static const struct kernel_param_ops wake_arm_awake_ops = {
	.set = wake_arm_awake_set,
};
module_param_cb(wake_arm_awake, &wake_arm_awake_ops, NULL, 0200);
MODULE_PARM_DESC(wake_arm_awake, "awake: 1 = write the macOS wake-source masks/table now, 0 = put the old ones back");

/* ---- mem_sleep "deep" ---- */

/*
 * SMC side of the macOS sleep, in the trace 10 order, for mem_sleep deep only:
 *   begin (before the drivers suspend, 183.27): MBSE = 'slpw'
 *   prepare (after suspend_late, i.e. after the PMP DevPwr messages, 183.37): WKTP = 2
 *   prepare_late (last coprocessor step before the CPUs stop, 183.88): MBSE = 'slpa', then
 *     RTKit SetAPPower 0x201
 *   wake: SetAPPower 0x20 if it was idled; finish: NTAP = 1 (if any step was sent)
 * Bit mask, one bit per step: 0x1 slpw, 0x2 WKTP=2, 0x4 slpa, 0x8 SetAPPower 0x201.
 * m1n1 hs3/hs4: after slpa + 0x201 the SMC stops its mailbox notifications. 2026-11-14:
 * slpw + WKTP=2 (0x3) already gave a deep sleep that neither the power button nor the lid
 * woke: default 0 until the bits are bisected. The lp 0x100/0x200 bits are ignored in deep.
 */
#define DEEP_SMC_SLPW	BIT(0)
#define DEEP_SMC_WKTP	BIT(1)
#define DEEP_SMC_SLPA	BIT(2)
#define DEEP_SMC_IDLE	BIT(3)
static unsigned int deep_smc;
module_param(deep_smc, uint, 0644);
MODULE_PARM_DESC(deep_smc, "mem_sleep deep, SMC steps as macOS (mask): 0x1 slpw, 0x2 WKTP=2, 0x4 slpa, 0x8 SetAPPower 0x201 (default 0; 0x3 did not wake)");

static bool deep_smc_idled;

static void deep_smc_step(const char *what, int ret)
{
	pr_info("deep SMC %s: %d\n", what, ret);
}

static void as_mask_irqs(bool mask)
{
	for (int i = 0; i < as.nmask_irqs; i++) {
		int irq = as.mask_irq[i];

		if (mask) {
			/* mask in the AIC now, not at the next interrupt: the CPU is in WFI by then */
			irq_set_status_flags(irq, IRQ_DISABLE_UNLAZY);
			disable_irq_nosync(irq);
		} else {
			enable_irq(irq);
			irq_clear_status_flags(irq, IRQ_DISABLE_UNLAZY);
		}
	}
}

/*
 * macOS SoC sleep request, HV trace 11 (2026-11-14), replayed at the end of prepare_late
 * (macOS: IOPlatformQuiesceAction, CPU0, before the cores shut down over the next 1.3 s).
 * soc_sleep mask, default 0:
 *   0x01 PMGR quiesce (AppleT8103PMGR, 202.153-202.163): 0x23b04014x = 0x80100000,
 *        0x23b734000/7c000/78000 = 0, 0x68c/0x234/0x279 008014 = 1 + 0x82bc = 1, 2, 4,
 *        0x23b748400/800/c00/000 = 0, 0x23b71c080/88 = 0x80000000, 0x23b718000/4 = 0
 *   0x02 CPU clusters: 0x210e20660/0x211e20660 = 0, 0x210e48080/0x211e48080 = 0 (64 bit),
 *        cluster +0x20 = 0x101101 then 0x40000501101 (the DVFS rewrites are left out)
 *   0x04 PS SIO_CPU 0x23b7001d0 = 0x400024f
 *   0x08 0x23d2c4000 = 1 (PMGR[1]+0x44000, the sleep request; macOS writes it 3 times)
 *   0x10 PMU "go hib" over SPMI (Sera PMIC, slave 0xf): 0xd00e = 0, 0xd000 = 0x41, then 0x08 again
 *   0x20 PMU S2R timer 0xd008 = 36 d0 22 01 (macOS value, meaning unknown; with 0x10)
 *   0x40 PMU boot_stage 0x9f01 = 0x50 (macOS, IOPlatformSleepAction; macsmc-reboot writes 0x30/0)
 * Every write is logged with its old value and read back; the 0x01/0x02/0x04 settings are put back
 * at .wake (the trigger registers 0x8014/0x82bc/0x23d2c4000 are not).
 */
static unsigned int soc_sleep;
module_param(soc_sleep, uint, 0644);
MODULE_PARM_DESC(soc_sleep, "deep: replay the macOS SoC sleep request (mask, see source): 0x1 PMGR quiesce, 0x2 CPU clusters, 0x4 SIO_CPU, 0x8 0x23d2c4000=1, 0x10 PMU go hib, 0x20 PMU S2R timer, 0x40 PMU boot_stage 0x50");

#define SOC_SAVE_MAX	40
static struct { phys_addr_t pa; u64 old; bool w64; } soc_saved[SOC_SAVE_MAX];
static int soc_nsaved;
/*
 * soc_skip: leave out groups of the 0x01 quiesce block, to bisect it (2026-11-14: awake soc_poke 0x01
 * froze during the restore; 0x23b718000/4 went f -> 0 (reads 10) and back to f (reads 1f)):
 *   0x01 0x23b04014x = 0x80100000      0x02 0x23b734000/7c000/78000 = 0
 *   0x04 0x8014/0x82bc triggers         0x08 0x23b748xxx = 0
 *   0x10 0x23b71c080/88 = 0x80000000    0x20 0x23b718000/4 = 0
 */
static unsigned int soc_skip;
module_param(soc_skip, uint, 0644);
MODULE_PARM_DESC(soc_skip, "soc_sleep/soc_poke: groups of the 0x01 quiesce block to leave out (mask, see source)");
#define SOC_G(g)	(!(soc_skip & (g)))
static bool soc_slow;	/* soc_poke: 300 ms after every write, so that dmesg -w shows a freeze */

static void soc_wr(phys_addr_t pa, u64 val, bool w64, bool save)
{
	void __iomem *va = ioremap_np(pa & PAGE_MASK, PAGE_SIZE);
	unsigned long off = pa & ~PAGE_MASK;
	u64 old, rb;

	if (!va) {
		pr_err("soc_sleep: ioremap %09llx failed\n", (u64)pa);
		return;
	}
	old = w64 ? readq(va + off) : readl(va + off);
	if (save && soc_nsaved < SOC_SAVE_MAX) {
		soc_saved[soc_nsaved].pa = pa;
		soc_saved[soc_nsaved].old = old;
		soc_saved[soc_nsaved].w64 = w64;
		soc_nsaved++;
	}
	if (w64)
		writeq(val, va + off);
	else
		writel(val, va + off);
	rb = w64 ? readq(va + off) : readl(va + off);
	pr_info("soc_sleep: %09llx %llx -> %llx (reads %llx)\n", (u64)pa, old, val, rb);
	iounmap(va);
	if (soc_slow)
		msleep(300);
}

static void soc_restore(void)
{
	while (soc_nsaved > 0) {
		soc_nsaved--;
		soc_wr(soc_saved[soc_nsaved].pa, soc_saved[soc_nsaved].old,
		       soc_saved[soc_nsaved].w64, false);
	}
}

static int pmu_wr(struct spmi_device *pmu, u16 addr, const u8 *buf, size_t len)
{
	u8 old[4] = {};
	int r0 = spmi_ext_register_readl(pmu, addr, old, len);
	int ret = spmi_ext_register_writel(pmu, addr, buf, len);

	pr_info("soc_sleep: PMU %04x (%zu bytes, read %d: %*ph) <- %*ph: %d\n", addr, len, r0,
		(int)len, old, (int)len, buf, ret);
	return ret;
}

static void soc_sleep_request(void)
{
	static const u64 cl[] = { 0x210e20000ULL, 0x211e20000ULL };	/* E, P cluster */
	static const u64 blk[] = { 0x68c008000ULL, 0x234008000ULL, 0x279008000ULL };

	if (!soc_sleep)
		return;
	pr_info("soc_sleep: mask %#x, skip %#x\n", soc_sleep, soc_skip);
	soc_nsaved = 0;
	if ((soc_sleep & 0x01) && SOC_G(0x01))
		for (int i = 0; i < 5; i++)
			soc_wr(0x23b040140ULL + (i < 4 ? 4 * i : 0x14), 0x80100000, false, true);
	if ((soc_sleep & 0x01) && SOC_G(0x02)) {
		soc_wr(0x23b734000ULL, 0, false, true);
		soc_wr(0x23b77c000ULL, 0, false, true);
		soc_wr(0x23b778000ULL, 0, false, true);
	}
	if (soc_sleep & 0x02) {
		for (int i = 0; i < 2; i++) {
			soc_wr(cl[i] + 0x660ULL, 0, true, true);
			soc_wr(cl[i] + 0x28080ULL, 0, true, true);
		}
	}
	if ((soc_sleep & 0x01) && SOC_G(0x04)) {
		for (int i = 0; i < 3; i++) {
			phys_addr_t b = blk[i];

			soc_wr(b + 0x14, 1, false, false);
			soc_wr(b + 0x2bc, 1, false, false);
			soc_wr(b + 0x2bc, 2, false, false);
			soc_wr(b + 0x2bc, 4, false, false);
		}
	}
	if (soc_sleep & 0x02)
		for (int i = 0; i < 2; i++)
			soc_wr(cl[i] + 0x20ULL, 0x101101, true, true);
	if ((soc_sleep & 0x01) && SOC_G(0x08)) {
		soc_wr(0x23b748400ULL, 0, false, true);
		soc_wr(0x23b748800ULL, 0, false, true);
		soc_wr(0x23b748c00ULL, 0, false, true);
		soc_wr(0x23b748000ULL, 0, false, true);
	}
	if (soc_sleep & 0x02)
		for (int i = 0; i < 2; i++)
			soc_wr(cl[i] + 0x20ULL, 0x40000501101ULL, true, false);
	if ((soc_sleep & 0x01) && SOC_G(0x10)) {
		soc_wr(0x23b71c080ULL, 0x80000000, false, true);
		soc_wr(0x23b71c088ULL, 0x80000000, false, true);
	}
	if ((soc_sleep & 0x01) && SOC_G(0x20)) {
		soc_wr(0x23b718000ULL, 0, false, true);
		soc_wr(0x23b718004ULL, 0, false, true);
	}
	if (soc_sleep & 0x04)
		soc_wr(0x23b7001d0ULL, 0x400024f, false, true);
	if (soc_sleep & 0x08)
		soc_wr(0x23d2c4000ULL, 1, false, false);

	if (soc_sleep & 0x70) {
		struct device_node *np = of_find_compatible_node(NULL, NULL, "apple,sera-pmic");
		struct spmi_device *pmu = np ? spmi_find_device_by_of_node(np) : NULL;

		of_node_put(np);
		if (IS_ERR_OR_NULL(pmu)) {
			pr_err("soc_sleep: PMU SPMI device not found\n");
			return;
		}
		if (soc_sleep & 0x40)
			pmu_wr(pmu, 0x9f01, (const u8[]){ 0x50 }, 1);
		if (soc_sleep & 0x10) {
			if (soc_sleep & 0x20)
				pmu_wr(pmu, 0xd008, (const u8[]){ 0x36, 0xd0, 0x22, 0x01 }, 4);
			pmu_wr(pmu, 0xd00e, (const u8[]){ 0x00 }, 1);
			pmu_wr(pmu, 0xd000, (const u8[]){ 0x41 }, 1);
			if (soc_sleep & 0x08)
				soc_wr(0x23d2c4000ULL, 1, false, false);
		}
		put_device(&pmu->dev);
	}
	pr_info("soc_sleep: request done\n");
}

/*
 * soc_poke=<mask>: awake bisecting aid for soc_sleep (2026-11-14, T1 = 0x09 never woke). Runs the
 * register part of the mask (0x01/0x02/0x04 only: the request 0x08 and the PMU bits are refused)
 * with 300 ms after every write, waits 2 s, then puts the settings back. Watch with "dmesg -w".
 */
static int soc_poke_set(const char *val, const struct kernel_param *kp)
{
	unsigned int mask, saved = soc_sleep;
	int ret = kstrtouint(val, 0, &mask);

	if (ret)
		return ret;
	if (!mask || (mask & ~0x07u))
		return -EINVAL;
	pr_info("soc_poke: mask %#x awake, 300 ms per write\n", mask);
	soc_sleep = mask;
	soc_slow = true;
	soc_sleep_request();
	msleep(2000);
	pr_info("soc_poke: still alive, restoring %d settings\n", soc_nsaved);
	soc_restore();
	soc_slow = false;
	soc_sleep = saved;
	pr_info("soc_poke: done\n");
	return 0;
}

static const struct kernel_param_ops soc_poke_ops = {
	.set = soc_poke_set,
};
module_param_cb(soc_poke, &soc_poke_ops, NULL, 0200);
MODULE_PARM_DESC(soc_poke, "awake: run the soc_sleep register writes of a mask (0x1/0x2/0x4) slowly, then restore (logged)");

/*
 * Deep wake-source report (2026-11-14: a deep sleep returned after 2.6 s with nothing plugged in;
 * pm_wakeup_irq() is only set by s2idle). At prepare_late, after dpm_suspend_noirq(), list every
 * IRQ that is still enabled (= what can still wake CPU0); in .enter, with IRQs off, snapshot the
 * CPU0 counts; in .wake (IRQs on again, the waking IRQ has run) log the IRQs whose count moved.
 */
static unsigned int *irq_snap;
static unsigned int irq_nsnap;

/* irq_to_desc() is not exported to modules; irq_get_irq_data() is */
static struct irq_desc *as_irq_desc(unsigned int irq)
{
	struct irq_data *d = irq_get_irq_data(irq);

	return d ? irq_data_to_desc(d) : NULL;
}

static const char *irq_name(struct irq_desc *desc)
{
	return desc->action && desc->action->name ? desc->action->name : "-";
}

static void irq_live_log(void)
{
	unsigned int n = 0;

	for (unsigned int irq = 1; irq < irq_get_nr_irqs(); irq++) {
		struct irq_desc *desc = as_irq_desc(irq);

		if (!desc || !desc->action || irqd_irq_disabled(&desc->irq_data))
			continue;
		pr_info("deep: live IRQ %u hw %lu %s%s\n", irq, desc->irq_data.hwirq, irq_name(desc),
			irqd_is_wakeup_set(&desc->irq_data) ? " (wake)" : "");
		n++;
	}
	pr_info("deep: %u IRQs still enabled at entry\n", n);
}

static void irq_snap_take(void)
{
	irq_nsnap = irq_get_nr_irqs();
	if (!irq_snap)
		irq_snap = kcalloc(irq_nsnap, sizeof(*irq_snap), GFP_ATOMIC);
	if (!irq_snap)
		return;
	for (unsigned int irq = 0; irq < irq_nsnap; irq++) {
		struct irq_desc *desc = as_irq_desc(irq);

		irq_snap[irq] = desc && desc->kstat_irqs ? per_cpu_ptr(desc->kstat_irqs, 0)->cnt : 0;
	}
}

static void irq_snap_diff(void)
{
	unsigned int n = 0;

	if (!irq_snap)
		return;
	for (unsigned int irq = 0; irq < irq_nsnap; irq++) {
		struct irq_desc *desc = as_irq_desc(irq);
		unsigned int c;

		if (!desc || !desc->kstat_irqs)
			continue;
		c = per_cpu_ptr(desc->kstat_irqs, 0)->cnt;
		if (c != irq_snap[irq]) {
			pr_info("deep: woken by IRQ %u hw %lu %s: +%u\n", irq, desc->irq_data.hwirq,
				irq_name(desc), c - irq_snap[irq]);
			n++;
		}
	}
	if (!n)
		pr_info("deep: no IRQ ran on CPU0 at wake\n");
}

static int as_suspend_begin(suspend_state_t state)
{
	as_s2idle_begin();
	as.mode = "deep";
	as.deep_ret = -EINVAL;
	as.lp_active &= ~(LP_SMC_HS | LP_SMC_IDLE);
	deep_smc_idled = false;
	if (wake_arm)
		wake_arm_now();
	if ((deep_smc & DEEP_SMC_SLPW) && as.smc)
		deep_smc_step("MBSE=slpw", apple_smc_write_u32(as.smc, SMC_KEY(MBSE), 0x736c7077));
	return 0;
}

static int as_suspend_prepare(void)
{
	as_s2idle_prepare();
	if ((deep_smc & DEEP_SMC_WKTP) && as.smc)
		deep_smc_step("WKTP=2", apple_smc_write_u8(as.smc, SMC_KEY(WKTP), 2));
	return 0;
}

static int as_suspend_prepare_late(void)
{
	/* called after dpm_suspend_noirq(), before the secondaries go offline */
	as_s2idle_prepare_late();
	if ((deep_smc & DEEP_SMC_SLPA) && as.smc)
		deep_smc_step("MBSE=slpa", apple_smc_write_u32(as.smc, SMC_KEY(MBSE), 0x736c7061));
	if ((deep_smc & DEEP_SMC_IDLE) && as.smc) {
		int ret = apple_rtkit_set_ap_power(as.smc->rtk, 0x201);

		deep_smc_idled = !ret;
		deep_smc_step("SetAPPower 0x201", ret);
	}
	as_mask_irqs(true);
	soc_sleep_request();
	irq_live_log();
	return 0;
}

/*
 * Deep dark-wake filter (2026-11-14). In deep the only live device IRQs are the SMC mailbox and
 * the lid GPIO (live-IRQ report), and every SMC message wakes CPU0, so a battery or thermal
 * notification gives a full resume (one deep sleep came back after 2.6 s, nothing plugged in).
 * s2idle filters through pm_wakeup_dev_event() in macsmc-input; deep cannot run the drivers
 * (timekeeping and the other CPUs are down), so with deep_filter=Y .enter pops the SMC I2A
 * FIFO itself after each wake and goes back to the gated WFI unless a message is a power-button
 * press (event 0x7201, button 1/6, state 1) or a lid open (0x7203, closed = 0). Any message on
 * another endpoint, or a wake with an empty FIFO and no stale mailbox AIC event, ends the sleep too. Every popped message is
 * kept and handed to the mailbox driver's rx callback at .wake (IRQs on, as its IRQ handler
 * would), so the SMC driver still sees them all, in order (RTKit syslog acks included).
 */
static bool deep_filter = true;
module_param(deep_filter, bool, 0644);
MODULE_PARM_DESC(deep_filter, "deep: go back to sleep on SMC messages that are not a button press or a lid open (default Y, 2026-11-14)");

#define SMC_I2A_CONTROL	0x114
#define SMC_I2A_RECV0	0x830
#define SMC_I2A_RECV1	0x838
#define SMC_I2A_EMPTY	BIT(17)
#define DF_LOG_MAX	128
#define DF_LOOPS_MAX	2000

static void __iomem *smc_mbox_va;
static struct apple_mbox *smc_mbox;
static struct apple_mbox_msg df_log[DF_LOG_MAX];
static unsigned int df_nlog, df_lost, df_filtered, df_stale;
static const char *df_reason;
static u32 df_event;

/* AIC v1 (t8103): per-CPU event register, IRQ mask clear bank */
#define AIC1_EVENT		0x2004
#define AIC1_EVENT_TYPE_IRQ	1
#define AIC1_MASK_CLR		0x4180
static void __iomem *aic_va;
static u32 smc_mbox_hwirq;	/* AIC event word of the SMC I2A not-empty IRQ */

/*
 * The AIC latches an event when a line asserts; the WFI ends while one is pending, and reading
 * AIC_EVENT acks it and masks that IRQ. Draining the FIFO drops the mailbox line but leaves its
 * event latched, so the next gated WFI returned at once with an empty FIFO (two runs, each after
 * one 0x71020000 battery notification). Ack it as aic_handle_irq() + aic_irq_eoi() would.
 * Returns true if an event other than the SMC mailbox was pending: it is unmasked again (a level
 * line still asserted re-raises it for the kernel after resume) and ends the sleep.
 */
static bool deep_filter_ack_aic(void)
{
	for (int i = 0; i < 8; i++) {
		u32 ev = readl(aic_va + AIC1_EVENT);
		u32 irq = ev & 0xffff;

		if (!ev)
			return false;
		if (((ev >> 16) & 0xff) == AIC1_EVENT_TYPE_IRQ)
			writel(BIT(irq & 31), aic_va + AIC1_MASK_CLR + 4 * (irq >> 5));
		if (ev != smc_mbox_hwirq) {
			df_event = ev;
			return true;
		}
		df_stale++;
	}
	return false;
}

/* true: a real wake; false: only notifications to ignore */
static bool deep_filter_check(void)
{
	bool any = false, wake = false;

	while (!(readl(smc_mbox_va + SMC_I2A_CONTROL) & SMC_I2A_EMPTY)) {
		u64 msg0 = readq(smc_mbox_va + SMC_I2A_RECV0);
		u32 msg1 = lower_32_bits(readq(smc_mbox_va + SMC_I2A_RECV1));
		u8 ep = msg1 & 0xff;
		u32 ev = msg0 >> 32;

		any = true;
		if (df_nlog < DF_LOG_MAX) {
			df_log[df_nlog].msg0 = msg0;
			df_log[df_nlog].msg1 = msg1;
			df_nlog++;
		} else {
			df_lost++;
		}
		if (ep != 0x20 || (msg0 & 0xff) != 0x18) {
			wake = true;
			df_reason = "non-notification message";
		} else if ((ev >> 16) == 0x7201 && (((ev >> 8) & 0xff) == 1 || ((ev >> 8) & 0xff) == 6) &&
			   (ev & 0xff)) {
			wake = true;
			df_reason = "power button";
		} else if ((ev >> 16) == 0x7203 && !((ev >> 8) & 0xff)) {
			wake = true;
			df_reason = "lid open";
		}
	}
	unsigned int stale = df_stale;

	if (aic_va && deep_filter_ack_aic()) {
		df_reason = "other IRQ (AIC event below)";
		return true;
	}
	/* nothing popped and no stale mailbox event: something else (a FIQ?) ended the WFI */
	if (!any && df_stale == stale) {
		df_reason = aic_va ? "no SMC message, no AIC event (FIQ?)" : "no SMC message (other IRQ)";
		return true;
	}
	return wake;
}

/*
 * deep_wake_s=N (test aid, 2026-11-14): the SMC RTC has no alarm, so nothing but the button, the
 * lid or an SMC notification ends a deep sleep. Arm the boot CPU's timer (hyp-virt, CNTV_*_EL0 under
 * VHE, the kernel's clockevent, shut down by then) N s ahead before the gated WFI, and restore its
 * registers after. Whether the timer can wake a power-gated core is what the first run tells.
 */
static unsigned int deep_wake_s;
module_param(deep_wake_s, uint, 0644);
MODULE_PARM_DESC(deep_wake_s, "deep test aid: wake after N s by the CPU timer (0 = off)");
static bool dw_fired;

/* per gated-WFI return: what was pending (ISR_EL1 I = bit 7, F = bit 6), the timer state, messages */
#define DF_ITER_MAX	32
struct df_iter {
	u64 t;		/* CNTVCT at return */
	u32 ctl;	/* CNTV_CTL_EL0 at return */
	u8 isr;		/* ISR_EL1 */
	s8 ret;		/* apple_cpuidle_system_suspend(): 0 gated, 1 not gated */
	u8 nmsg;	/* SMC messages popped */
};
static struct df_iter df_iter[DF_ITER_MAX];
static unsigned int df_niter;
static u64 df_t0;

static void dw_arm(u64 deadline)
{
	write_sysreg(deadline, cntv_cval_el0);
	write_sysreg(ARCH_TIMER_CTRL_ENABLE, cntv_ctl_el0);
	isb();
}

static int as_suspend_enter(suspend_state_t state)
{
	u64 dw_ctl = 0, dw_cval = 0, deadline = 0;

	dw_fired = false;
	df_niter = 0;
	df_t0 = read_sysreg(cntvct_el0);
	if (deep_wake_s) {
		dw_ctl = read_sysreg(cntv_ctl_el0);
		dw_cval = read_sysreg(cntv_cval_el0);
		deadline = df_t0 + (u64)deep_wake_s * read_sysreg(cntfrq_el0);
	}
	irq_snap_take();
	df_nlog = 0;
	df_lost = 0;
	df_filtered = 0;
	df_stale = 0;
	df_event = 0;
	df_reason = NULL;
	for (;;) {
		unsigned int nlog0 = df_nlog;
		struct df_iter *it = df_niter < DF_ITER_MAX ? &df_iter[df_niter] : NULL;
		bool wake;

		/* power-gating may lose the timer registers: re-arm before every gated WFI */
		if (deep_wake_s)
			dw_arm(deadline);
		as.deep_ret = apple_cpuidle_system_suspend();
		if (it) {
			it->t = read_sysreg(cntvct_el0);
			it->ctl = read_sysreg(cntv_ctl_el0);
			it->isr = read_sysreg(isr_el1);
			it->ret = as.deep_ret;
			df_niter++;
		}
		if (as.deep_ret < 0 || !deep_filter || !smc_mbox_va || !smc_mbox)
			break;
		wake = deep_filter_check();
		if (it)
			it->nmsg = df_nlog - nlog0;
		if (deep_wake_s && read_sysreg(cntvct_el0) >= deadline) {
			dw_fired = true;
			if (!wake || !strncmp(df_reason, "no SMC", 6))
				df_reason = "deep_wake_s deadline";
			break;
		}
		if (wake)
			break;
		if (++df_filtered >= DF_LOOPS_MAX) {
			df_reason = "too many filtered wakes";
			break;
		}
	}
	if (deep_wake_s) {
		write_sysreg(dw_ctl | ARCH_TIMER_CTRL_IT_MASK, cntv_ctl_el0);
		write_sysreg(dw_cval, cntv_cval_el0);
		isb();
	}
	return as.deep_ret < 0 ? as.deep_ret : 0;
}

static void df_iter_log(void)
{
	u64 f = read_sysreg(cntfrq_el0);

	for (unsigned int i = 0; i < df_niter; i++) {
		u64 ms = div64_u64((df_iter[i].t - df_t0) * 1000, f);

		pr_info("deep: WFI return %u at %llu.%03llu s: %s, ISR_EL1 %#x%s%s, CNTV_CTL %#x, %u SMC msgs\n",
			i, ms / 1000, ms % 1000, df_iter[i].ret == 0 ? "gated" : df_iter[i].ret == 1 ? "not gated" : "error",
			df_iter[i].isr, df_iter[i].isr & BIT(7) ? " IRQ" : "", df_iter[i].isr & BIT(6) ? " FIQ" : "",
			df_iter[i].ctl, df_iter[i].nmsg);
	}
	if (df_niter == DF_ITER_MAX)
		pr_info("deep: (only the first %u WFI returns logged)\n", DF_ITER_MAX);
}

static void deep_filter_log(void)
{
	unsigned long flags;

	if (!deep_filter || !df_reason)
		return;
	pr_info("deep_filter: %u wakes slept through (%u stale mailbox AIC events acked), woke on: %s, %u messages kept%s\n",
		df_filtered, df_stale, df_reason, df_nlog, df_lost ? " (some lost: buffer full)" : "");
	if (df_event)
		pr_info("deep_filter:   AIC event %#x (type %u, irq %u)\n", df_event, (df_event >> 16) & 0xff,
			df_event & 0xffff);
	for (unsigned int i = 0; i < df_nlog; i++)
		pr_info("deep_filter:   ep %#x msg %016llx\n", df_log[i].msg1 & 0xff, df_log[i].msg0);
	if (df_lost)
		pr_warn("deep_filter: %u messages dropped\n", df_lost);
	/* hand them to the mailbox client (RTKit) as the mailbox IRQ handler would */
	spin_lock_irqsave(&smc_mbox->rx_lock, flags);
	for (unsigned int i = 0; i < df_nlog; i++)
		smc_mbox->rx(smc_mbox, df_log[i], smc_mbox->cookie);
	spin_unlock_irqrestore(&smc_mbox->rx_lock, flags);
	df_nlog = 0;
	df_reason = NULL;
}

static void as_suspend_wake(void)
{
	if (soc_nsaved)
		pr_info("soc_sleep: back, restoring %d settings\n", soc_nsaved);
	soc_restore();
	deep_filter_log();
	if (deep_wake_s)
		pr_info("deep_wake_s: deadline %s\n", dw_fired ? "reached" : "not reached at wake");
	df_iter_log();
	irq_snap_diff();
	as_mask_irqs(false);
	if (deep_smc_idled) {
		deep_smc_step("SetAPPower 0x20", apple_rtkit_set_ap_power(as.smc->rtk, 0x20));
		deep_smc_idled = false;
	}
	as_s2idle_restore_early();
	pr_info("deep: boot CPU %s\n", as.deep_ret == 0 ? "power-gated (back through m1n1)" :
		as.deep_ret == 1 ? "WFI returned without power-gating" : "enter failed");
}

static void as_suspend_finish(void)
{
	as_s2idle_restore();
	wake_restore();
	if (deep_smc && as.smc)
		deep_smc_step("NTAP=1", apple_smc_write_flag(as.smc, SMC_KEY(NTAP), true));
}

static const struct platform_suspend_ops as_suspend_ops = {
	.valid		= suspend_valid_only_mem,
	.begin		= as_suspend_begin,
	.prepare	= as_suspend_prepare,
	.prepare_late	= as_suspend_prepare_late,
	.enter		= as_suspend_enter,
	.wake		= as_suspend_wake,
	.finish		= as_suspend_finish,
};

static bool deep = true;
static int deep_set(const char *val, const struct kernel_param *kp)
{
	int ret = param_set_bool(val, kp);

	if (!ret && as.dev) {
		suspend_set_ops(deep ? &as_suspend_ops : NULL);
		pr_info("mem_sleep deep %s\n", deep ? "registered" : "unregistered (echo s2idle > /sys/power/mem_sleep)");
	}
	return ret;
}

static const struct kernel_param_ops deep_ops = {
	.set = deep_set,
	.get = param_get_bool,
};
module_param_cb(deep, &deep_ops, &deep, 0644);
MODULE_PARM_DESC(deep, "EXPERIMENTAL: register mem_sleep \"deep\" (all CPUs power-gated, wake by SMC/lid; default Y, needs the m1n1 resume handoff)");

/* every coprocessor mailbox IRQ but the SMC's (its button/lid notifications are the wake) */
static void as_find_mask_irqs(void)
{
	struct device_node *smc = of_find_compatible_node(NULL, NULL, "apple,smc");
	struct device_node *smc_mbox = smc ? of_parse_phandle(smc, "mboxes", 0) : NULL;
	struct device_node *np;

	for_each_compatible_node(np, NULL, "apple,asc-mailbox-v4") {
		if (np == smc_mbox || !of_device_is_available(np))
			continue;
		for (int i = 0; as.nmask_irqs < MAX_MASK_IRQS; i++) {
			int irq = of_irq_get(np, i);

			if (irq <= 0)
				break;
			as.mask_irq[as.nmask_irqs++] = irq;
		}
	}
	of_node_put(smc_mbox);
	of_node_put(smc);
}

/*
 * The lid pin (aop-gpio 22) never raised its IRQ under Linux, awake or asleep (2026-11-14), while
 * its level reads fine. pinctrl-apple-gpio puts it in IRQ group 0 (AIC 268); m1n1 --lidwake used
 * group 6 (AIC 274) and woke from it. Log the pin register (bits 3:1 mode, 6:5 periph, 18:16
 * group, 21 LOCK) and let lid_grp move it to another group (the pinctrl driver demuxes all 7).
 */
#define LID_PIN_REG	(0x24a820000ULL + 4 * 22)
#define LID_GRP		GENMASK(18, 16)
#define LID_LOCK	BIT(21)
#define LID_MODE	GENMASK(3, 1)

static void lid_reg_log(const char *tag)
{
	u32 v;

	if (!as.lid_reg)
		return;
	v = readl(as.lid_reg);
	pr_info("lid pin reg %s: %08x (mode %lu, group %lu%s)\n", tag, v, FIELD_GET(LID_MODE, v),
		FIELD_GET(LID_GRP, v), (v & LID_LOCK) ? ", LOCKED" : "");
}

static int lid_grp_set(const char *val, const struct kernel_param *kp)
{
	unsigned int grp;
	u32 v;
	int ret = kstrtouint(val, 0, &grp);

	if (ret)
		return ret;
	if (grp > 6)
		return -EINVAL;
	if (!as.lid_reg)
		return -ENODEV;
	lid_reg_log("before");
	v = readl(as.lid_reg);
	/* also re-arm any-edge (6): the mode was found back at 7 (off) minutes after request_irq */
	writel((v & ~(LID_GRP | LID_MODE)) | FIELD_PREP(LID_GRP, grp) | FIELD_PREP(LID_MODE, 6),
	       as.lid_reg);
	lid_reg_log("after ");
	return 0;
}

static const struct kernel_param_ops lid_grp_ops = {
	.set = lid_grp_set,
};
module_param_cb(lid_grp, &lid_grp_ops, NULL, 0200);
MODULE_PARM_DESC(lid_grp, "write 0-6: move the lid pin (aop-gpio 22) IRQ to that group and re-arm any-edge now (logged)");

static int lid_reg_get(char *buf, const struct kernel_param *kp)
{
	u32 v;

	if (!as.lid_reg)
		return scnprintf(buf, PAGE_SIZE, "none\n");
	v = readl(as.lid_reg);
	return scnprintf(buf, PAGE_SIZE, "%08x mode %lu group %lu%s level %u\n", v,
			 FIELD_GET(LID_MODE, v), FIELD_GET(LID_GRP, v),
			 (v & LID_LOCK) ? " LOCKED" : "", v & 1);
}

static const struct kernel_param_ops lid_reg_ops = {
	.get = lid_reg_get,
};
module_param_cb(lid_reg, &lid_reg_ops, NULL, 0444);
MODULE_PARM_DESC(lid_reg, "read: the lid pin (aop-gpio 22) register now");

static irqreturn_t as_lid_irq(int irq, void *data)
{
	int lvl = gpiod_get_value(as.lid);

	as.lid_irqs++;
	if (!as.armed || !lid_wake)
		return IRQ_HANDLED;

	if (lvl != as.lid_open_level) {
		as.lid_seen_closed = true;
	} else if (as.lid_seen_closed) {
		as.lid_woke = true;
		pm_wakeup_dev_event(as.dev, 0, true);
	}
	return IRQ_HANDLED;
}

static void as_iounmap(void *va)
{
	iounmap(va);
}

/*
 * Apple SoC MMIO must be mapped non-posted (nGnRnE, DT "nonposted-mmio"): a
 * posted write to the PMGR ends in an SError and the Mac freezes. A plain
 * devm_ioremap() was the cause of every "this write hangs" so far
 * (2026-11-10): reads were fine, any write froze the Mac.
 */
static int as_add_map(struct device_node *np)
{
	struct resource res;
	void __iomem *va;
	int ret;

	if (of_address_to_resource(np, 0, &res))
		return -EINVAL;
	for (int i = 0; i < as.nmaps; i++)
		if (as.maps[i].base == res.start)
			return 0;
	if (as.nmaps >= MAX_MAPS)
		return -ENOSPC;
	va = ioremap_np(res.start, resource_size(&res));
	if (!va)
		return -ENOMEM;
	ret = devm_add_action_or_reset(as.dev, as_iounmap, (void __force *)va);
	if (ret)
		return ret;
	as.maps[as.nmaps].va = va;
	as.maps[as.nmaps].base = res.start;
	as.maps[as.nmaps].size = resource_size(&res);
	as.nmaps++;
	return 0;
}

static void as_add_dom(const char *name, u64 addr, bool aon)
{
	void __iomem *va = ps_va(addr);

	if (!va)
		return;
	for (int i = 0; i < as.ndoms; i++)
		if (as.doms[i].va == va)
			return;
	as.doms[as.ndoms].name = name;
	as.doms[as.ndoms].addr = addr;
	as.doms[as.ndoms].va = va;
	as.doms[as.ndoms].aon = aon;
	as.ndoms++;
}

/* Every pmgr-pwrstate node of the DT, plus the table entries the DT does not have */
static int as_build_doms(void)
{
	struct device_node *np;
	int n = 0, ret;

	for_each_compatible_node(np, NULL, "apple,pmgr-pwrstate")
		n++;
	n += ARRAY_SIZE(ps_macos_final) + ARRAY_SIZE(ps_auto) + ARRAY_SIZE(ps_snap) +
	     ARRAY_SIZE(ps_atc);
	as.doms = devm_kcalloc(as.dev, n, sizeof(*as.doms), GFP_KERNEL);
	if (!as.doms)
		return -ENOMEM;

	for_each_compatible_node(np, NULL, "apple,t8103-pmgr") {
		ret = as_add_map(np);
		if (ret) {
			of_node_put(np);
			return ret;
		}
	}

	for_each_compatible_node(np, NULL, "apple,pmgr-pwrstate") {
		struct device_node *parent = of_get_parent(np);
		struct resource res;
		const char *name;
		u32 off;

		if (parent && !of_address_to_resource(parent, 0, &res) &&
		    !of_property_read_u32(np, "reg", &off)) {
			if (of_property_read_string(np, "label", &name))
				name = np->name;
			as_add_dom(name, res.start + off,
				   of_property_read_bool(np, "apple,always-on"));
		}
		of_node_put(parent);
	}
	for (int i = 0; i < ARRAY_SIZE(ps_macos_final); i++)
		as_add_dom(ps_macos_final[i].name, T8103_PMGR + ps_macos_final[i].addr, false);
	for (int i = 0; i < ARRAY_SIZE(ps_auto); i++)
		as_add_dom(ps_auto[i].name, ps_auto[i].addr, false);
	for (int i = 0; i < ARRAY_SIZE(ps_snap); i++)
		as_add_dom(ps_snap[i].name, ps_snap[i].addr, false);
	for (int i = 0; i < ARRAY_SIZE(ps_atc); i++)
		as_add_dom(ps_atc[i].name, ps_atc[i].addr, false);
	return 0;
}

static int as_get_smc(void)
{
	struct device_node *np = of_find_compatible_node(NULL, NULL, "apple,smc");
	struct platform_device *pdev;

	if (!np)
		return -ENODEV;
	pdev = of_find_device_by_node(np);
	of_node_put(np);
	if (!pdev)
		return -EPROBE_DEFER;
	as.smc = platform_get_drvdata(pdev);
	if (!as.smc) {
		put_device(&pdev->dev);
		return -EPROBE_DEFER;
	}
	as.smc_dev = &pdev->dev;
	return 0;
}

static int apple_sleep_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	int ret;

	as.dev = dev;

	ret = as_get_smc();
	if (ret)
		return dev_err_probe(dev, ret, "no SMC\n");

	as.lid = devm_gpiod_get_optional(dev, "lid", GPIOD_IN);
	if (IS_ERR(as.lid)) {
		ret = PTR_ERR(as.lid);
		goto err_smc;
	}

	ret = as_build_doms();
	if (ret)
		goto err_smc;

	device_init_wakeup(dev, true);

	if (as.lid) {
		as.lid_reg = ioremap_np(LID_PIN_REG, 4);
		lid_reg_log("at probe, before request_irq");
		as.lid_open_level = gpiod_get_value(as.lid);
		as.lid_irq = gpiod_to_irq(as.lid);
		if (as.lid_irq < 0) {
			ret = as.lid_irq;
			goto err_smc;
		}
		ret = devm_request_irq(dev, as.lid_irq, as_lid_irq,
				       IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING | IRQF_NO_SUSPEND,
				       "apple-sleep-lid", &as);
		if (ret)
			goto err_smc;
		lid_reg_log("after request_irq");
	}

	wake_mask_va = ioremap_np(WAKE_MASK_PHYS, WAKE_MASK_OFF + 4 * WAKE_MASK_N);
	wake_tbl_va = ioremap_np(WAKE_TBL_PHYS, WAKE_TBL_OFF + 0x10 * WAKE_TBL_N);
	wake_log("at probe");

	as_find_mask_irqs();
	{
		struct device_node *smc = of_find_compatible_node(NULL, NULL, "apple,smc");
		struct device_node *mb = smc ? of_parse_phandle(smc, "mboxes", 0) : NULL;
		struct resource r;

		if (mb && !of_address_to_resource(mb, 0, &r))
			smc_mbox_va = ioremap_np(r.start, resource_size(&r));
		if (mb) {
			struct platform_device *mpdev = of_find_device_by_node(mb);

			if (mpdev) {
				smc_mbox = platform_get_drvdata(mpdev);
				put_device(&mpdev->dev);
			}
		}
		of_node_put(mb);
		of_node_put(smc);
		if (!smc_mbox_va)
			dev_warn(dev, "SMC mailbox not mapped: deep_filter off\n");
	}
	{
		struct device_node *aic = of_find_compatible_node(NULL, NULL, "apple,t8103-aic");
		struct irq_data *d = smc_mbox ? irq_get_irq_data(smc_mbox->irq_recv_not_empty) : NULL;
		struct resource r;

		if (aic && d && !of_address_to_resource(aic, 0, &r)) {
			smc_mbox_hwirq = irqd_to_hwirq(d);
			aic_va = ioremap_np(r.start, resource_size(&r));
		}
		of_node_put(aic);
		dev_info(dev, "deep_filter: AIC %s, SMC mailbox event %#x\n", aic_va ? "mapped" : "not mapped",
			 smc_mbox_hwirq);
	}
	s2idle_set_ops(&as_s2idle_ops);
	if (deep)
		suspend_set_ops(&as_suspend_ops);
	dev_info(dev, "s2idle ops registered: %d power states, lid %s (open level %d, IRQ %d), lp %#x; deep %s, %d mailbox IRQs masked in deep sleep\n",
		 as.ndoms, as.lid ? "gpio" : "none", as.lid_open_level, as.lid_irq, lp,
		 deep ? "registered" : "off", as.nmask_irqs);
	return 0;

err_smc:
	if (as.lid_reg) {
		iounmap(as.lid_reg);
		as.lid_reg = NULL;
	}
	put_device(as.smc_dev);
	return dev_err_probe(dev, ret, "probe failed\n");
}

static void wake_unmap(void)
{
	if (smc_mbox_va)
		iounmap(smc_mbox_va);
	smc_mbox_va = NULL;
	if (aic_va)
		iounmap(aic_va);
	aic_va = NULL;
	kfree(irq_snap);
	irq_snap = NULL;
	if (watch_task)
		kthread_stop(watch_task);
	wake_restore();
	if (wake_mask_va)
		iounmap(wake_mask_va);
	if (wake_tbl_va)
		iounmap(wake_tbl_va);
	wake_mask_va = wake_tbl_va = NULL;
}

static void apple_sleep_remove(struct platform_device *pdev)
{
	wake_unmap();
	if (deep)
		suspend_set_ops(NULL);
	s2idle_set_ops(NULL);
	device_init_wakeup(&pdev->dev, false);
	if (as.lid_reg) {
		iounmap(as.lid_reg);
		as.lid_reg = NULL;
	}
	put_device(as.smc_dev);
}

static const struct of_device_id apple_sleep_of_match[] = {
	{ .compatible = "apple,t8103-sleep" },
	{}
};
MODULE_DEVICE_TABLE(of, apple_sleep_of_match);

static struct platform_driver apple_sleep_driver = {
	.driver = {
		.name = "apple-sleep",
		.of_match_table = apple_sleep_of_match,
	},
	.probe = apple_sleep_probe,
	.remove = apple_sleep_remove,
};
module_platform_driver(apple_sleep_driver);

MODULE_DESCRIPTION("EXPERIMENTAL Apple T8103 suspend-to-idle extras");
MODULE_LICENSE("Dual MIT/GPL");
