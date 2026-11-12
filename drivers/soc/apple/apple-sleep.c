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
#include <linux/bits.h>
#include <linux/ctype.h>
#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/iopoll.h>
#include <linux/irq.h>
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

static int as_s2idle_begin(void)
{
	as.mode = "s2idle";
	as.loops = 0;
	as.lid_irqs = 0;
	as.lid_woke = false;
	as.lp_active = lp;
	return 0;
}

/* After dpm_suspend_late(): device IRQs still on, so the SMC can still be talked to */
static int as_s2idle_prepare(void)
{
	if (as.lid) {
		int lvl = gpiod_get_value(as.lid);

		as.lid_seen_closed = lvl != as.lid_open_level;
		pr_info("lid level %d at entry (%s)\n", lvl, as.lid_seen_closed ? "closed" : "open");
	}
	if (smc_meter) {
		meter_read(&meter_entry);
		meter_log("entry", &meter_entry);
	}
	if (as.lp_active & (LP_SMC_HS | LP_SMC_IDLE))
		smc_sleep_keys();
	as.armed = true;
	return 0;
}

/* After dpm_suspend_noirq(): genpd has turned off what Linux turns off */
static int as_s2idle_prepare_late(void)
{
	if (dump)
		ps_dump(dump);
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
	if (as.nsaved)
		ps_restore();
	pr_info("%s: %lld.%03lld s, of which awake %lld ms, %u wakeups, wake IRQ %u%s, lid IRQs %u\n",
		as.mode, slept / 1000, slept % 1000, awake, as.loops ? as.loops - 1 : 0, pm_wakeup_irq(),
		as.lid_woke ? " (lid open)" : "", as.lid_irqs);
}

static void as_s2idle_restore(void)
{
	/* after dpm_resume_noirq(): device IRQs on again, the SMC answers; drivers not resumed yet */
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

/* ---- mem_sleep "deep" ---- */

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

static int as_suspend_begin(suspend_state_t state)
{
	as_s2idle_begin();
	as.mode = "deep";
	as.deep_ret = -EINVAL;
	return 0;
}

static int as_suspend_prepare_late(void)
{
	/* called after dpm_suspend_noirq(), before the secondaries go offline */
	as_s2idle_prepare_late();
	as_mask_irqs(true);
	return 0;
}

static int as_suspend_enter(suspend_state_t state)
{
	as.deep_ret = apple_cpuidle_system_suspend();
	return as.deep_ret < 0 ? as.deep_ret : 0;
}

static void as_suspend_wake(void)
{
	as_mask_irqs(false);
	as_s2idle_restore_early();
	pr_info("deep: boot CPU %s\n", as.deep_ret == 0 ? "power-gated (back through m1n1)" :
		as.deep_ret == 1 ? "WFI returned without power-gating" : "enter failed");
}

static const struct platform_suspend_ops as_suspend_ops = {
	.valid		= suspend_valid_only_mem,
	.begin		= as_suspend_begin,
	.prepare	= as_s2idle_prepare,
	.prepare_late	= as_suspend_prepare_late,
	.enter		= as_suspend_enter,
	.wake		= as_suspend_wake,
	.finish		= as_s2idle_restore,
};

static bool deep;
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
MODULE_PARM_DESC(deep, "EXPERIMENTAL: register mem_sleep \"deep\" (all CPUs power-gated, wake by SMC/lid)");

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
	}

	as_find_mask_irqs();
	s2idle_set_ops(&as_s2idle_ops);
	if (deep)
		suspend_set_ops(&as_suspend_ops);
	dev_info(dev, "s2idle ops registered: %d power states, lid %s (open level %d, IRQ %d), lp %#x; deep %s, %d mailbox IRQs masked in deep sleep\n",
		 as.ndoms, as.lid ? "gpio" : "none", as.lid_open_level, as.lid_irq, lp,
		 deep ? "registered" : "off", as.nmask_irqs);
	return 0;

err_smc:
	put_device(as.smc_dev);
	return dev_err_probe(dev, ret, "probe failed\n");
}

static void apple_sleep_remove(struct platform_device *pdev)
{
	if (deep)
		suspend_set_ops(NULL);
	s2idle_set_ops(NULL);
	device_init_wakeup(&pdev->dev, false);
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
