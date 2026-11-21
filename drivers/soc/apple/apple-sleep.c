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
#include <linux/hex.h>
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
#include <linux/stop_machine.h>
#include <linux/suspend.h>

#include "apple-sleep-macos-boot.h"

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
 * DISP0_CPU0 / DISPEXT_*: kept on by the DCP driver (DISP0_CPU0 holds the DCP and its DART: needs a
 * DCP full restart). DISP0_FE and RMX off in suspend: display back fine, ~1.3 mA (with the PCIe
 * refclk release; deep 15 min 19.2 -> 17.9 mA, 2026-11-15), so no longer skipped.
 * FPWM0+FPWM2 off together hung the s2idle resume (each alone did not, 2026-11-10).
 * APCIE_GP (Wi-Fi PCIe) off lost the Wi-Fi once out of a few resumes (and loses the m1n1 PCIe init).
 */
static char *skip = "DISP0_CPU0,DISPEXT_CPU0,DISPEXT_FE,FPWM0,FPWM2,APCIE_GP";
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
	s64 dt_ms, dbaac, ma_c;

	meter_read(&m);
	meter_log("exit ", &m);
	dt_ms = ktime_ms_delta(m.t, meter_entry.t);
	if (!meter_entry.ok || !m.ok || dt_ms <= 0)
		return;
	dbaac = m.baac - meter_entry.baac;
	/* BAAC counts mA*s (negative = discharge): average current = -dBAAC / dt_s, here in 1/100 mA */
	ma_c = div64_s64(-dbaac * 100000, dt_ms);
	pr_info("meter: %lld.%03lld s between readings, BAAC %+lld (%+lld per hour), average %s%lld.%02lld mA, B0RM %+d mAh\n",
		dt_ms / 1000, dt_ms % 1000, dbaac, div64_s64(dbaac * 3600000, dt_ms),
		ma_c < 0 ? "-" : "", abs(ma_c) / 100, abs(ma_c) % 100,
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

/*
 * smc_gpio=<pin>=<0|1> (2026-11-14, Wi-Fi power test): drive an SMC GPIO behind gpiolib's back,
 * as gpio-macsmc does (key gPxx, CMD_OUTPUT | value). Pin 13 is the Wi-Fi/BT PWREN, claimed by
 * pcie-apple: "13=0" after removing 01:00.0 and 01:00.1 cuts the chip power for a meter run
 * (Wi-Fi then stays dead until reboot; "13=1" alone does not bring it back).
 */
static int smc_gpio_set(const char *val, const struct kernel_param *kp)
{
	unsigned int pin, v;
	smc_key key;
	int ret;

	if (!as.smc)
		return -ENODEV;
	if (sscanf(val, "%u=%u", &pin, &v) != 2 || pin > 0xff || v > 1)
		return -EINVAL;
	key = (SMC_KEY(gP00) & 0xffff0000) | hex_asc_hi(pin) << 8 | hex_asc_lo(pin);
	ret = apple_smc_write_u32(as.smc, key, (1u << 24) | v);
	pr_info("awake: SMC GPIO %u (%p4ch) = %u: %d\n", pin, &key, v, ret);
	return ret;
}

static const struct kernel_param_ops smc_gpio_ops = {
	.set = smc_gpio_set,
};
module_param_cb(smc_gpio, &smc_gpio_ops, NULL, 0200);
MODULE_PARM_DESC(smc_gpio, "awake: drive an SMC GPIO, <pin>=<0|1> (13 = Wi-Fi/BT PWREN)");

/*
 * reg_poke="<addr> <value>" (2026-11-14, DCS step): awake, write one 32-bit MMIO register (non-posted)
 * and read it back at once, after 100 ms and after 1 s, to see whether the hardware keeps it or
 * someone (PMP, a driver) rewrites it. No restore: write the old value back by hand.
 */
static int reg_poke_set(const char *val, const struct kernel_param *kp)
{
	u64 addr;
	u32 v, before, r0, r1, r2;
	void __iomem *va;

	if (sscanf(val, "%llx %x", &addr, &v) != 2 || (addr & 3))
		return -EINVAL;
	va = ioremap_np(addr, 4);
	if (!va)
		return -ENOMEM;
	before = readl(va);
	writel(v, va);
	r0 = readl(va);
	msleep(100);
	r1 = readl(va);
	msleep(900);
	r2 = readl(va);
	iounmap(va);
	pr_info("reg_poke %#llx: %08x, wrote %08x: now %08x, 100 ms %08x, 1 s %08x\n", addr, before, v,
		r0, r1, r2);
	return 0;
}

/* reg_peek="<addr> [count]": awake, read up to 16 consecutive 32-bit registers (non-posted) */
static int reg_peek_set(const char *val, const struct kernel_param *kp)
{
	u64 addr;
	u32 n = 1;
	void __iomem *va;
	char line[200];
	int len = 0;

	if (sscanf(val, "%llx %u", &addr, &n) < 1 || (addr & 3) || !n || n > 16)
		return -EINVAL;
	va = ioremap_np(addr, 4 * n);
	if (!va)
		return -ENOMEM;
	for (u32 i = 0; i < n; i++)
		len += scnprintf(line + len, sizeof(line) - len, " %08x", readl(va + 4 * i));
	iounmap(va);
	pr_info("reg_peek %#llx:%s\n", addr, line);
	return 0;
}

static const struct kernel_param_ops reg_peek_ops = {
	.set = reg_peek_set,
};
module_param_cb(reg_peek, &reg_peek_ops, NULL, 0200);
MODULE_PARM_DESC(reg_peek, "awake: \"<addr> [count]\" read up to 16 32-bit registers");

static const struct kernel_param_ops reg_poke_ops = {
	.set = reg_poke_set,
};
module_param_cb(reg_poke, &reg_poke_ops, NULL, 0200);
MODULE_PARM_DESC(reg_poke, "awake: \"<addr> <value>\" write a 32-bit register, read back now/100 ms/1 s");

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
MODULE_PARM_DESC(soc_sleep, "deep: replay the macOS SoC sleep request (mask, see source): 0x1 PMGR quiesce, 0x2 CPU clusters, 0x4 SIO_CPU, 0x8 0x23d2c4000=1, 0x10 PMU go hib, 0x20 PMU S2R timer, 0x40 PMU boot_stage 0x50, 0x80 SPMI controllers +0x38 = 1 (bus sleep, as macOS), 0x100 SEP sleep message (ep 0 type 0x0c) in .enter");

#define SOC_SAVE_MAX	128
static struct { phys_addr_t pa; u64 old; bool w64; } soc_saved[SOC_SAVE_MAX];
static int soc_nsaved;
/*
 * soc_skip: leave out groups of the 0x01 quiesce block, to bisect it (2026-11-14: awake soc_poke 0x01
 * froze during the restore; 0x23b718000/4 went f -> 0 (reads 10) and back to f (reads 1f)):
 *   0x01 0x23b04014x = 0x80100000      0x02 0x23b734000/7c000/78000 = 0
 *   0x04 0x8014/0x82bc triggers         0x08 0x23b748xxx = 0
 *   0x10 0x23b71c080/88 = 0x80000000    0x20 0x23b718000/4 = 0
 *   0x40/0x80/0x100: one block of group 0x04 each (0x68c008000 PCIe axi, 0x234008000, 0x279008000);
 *   2026-11-17 timer test: group 0x04 alone stops the wake (watchdog), 0x01 and 0x02 do not
 */
static unsigned int soc_skip;
module_param(soc_skip, uint, 0644);
MODULE_PARM_DESC(soc_skip, "soc_sleep/soc_poke: groups of the 0x01 quiesce block to leave out (mask, see source)");
#define SOC_G(g)	(!(soc_skip & (g)))
static bool soc_slow;	/* soc_poke: 300 ms after every write, so that dmesg -w shows a freeze */

/*
 * soc_late=1 (2026-11-17): macOS runs its SoC sleep request on CPU0 alone, after CPUs 1-7 wrote
 * CPUSTART and went down (trace 10: 183.98 CPUSTART, 184.06 quiesce, 184.07-185.03 the last WFIs;
 * trace 11: quiesce 202.153, 0x23d2c4000 + PMU go hib 202.17-202.21 during the final core
 * shutdowns). soc_sleep_request() in prepare_late runs with all 8 cores online (and 0x23b718000/4
 * = 0 froze an awake Mac). With soc_late, prepare_late only builds the plan (MMIO mapped now, PMU
 * SPMI controller found now) and .enter runs it with IRQs off, right before the last WFI; the PMU
 * writes then go through an atomic raw SPMI command (the SPMI driver sleeps: mutex + poll).
 */
static bool soc_late;
module_param(soc_late, bool, 0644);
MODULE_PARM_DESC(soc_late, "deep: run the soc_sleep writes in .enter (CPU0 alone, IRQs off, before the last WFI) instead of prepare_late, as macOS");
/*
 * soc_early (2026-11-18): with soc_late, the PMGR/SPMI part of soc_sleep (0x80 SPMI +0x38, 0x01
 * quiesce incl. the 0x23b718000/4 'go', 0x02 clusters, 0x04 SIO_CPU) is written at once in
 * prepare_late, i.e. right before the secondaries go offline; only 0x23d2c4000 (0x08) and the PMU
 * writes stay in .enter. HV trace 15 order: quiesce + go, then CPU7..1 ACC_OVRD DEEP_SLEEP, then
 * CPU0 PMU go hib. A non-last core setting DEEP_SLEEP without the go hangs the Mac at once
 * (cpu7 offline test, 2026-11-18): use with cpuidle_apple.secondary_deep.
 */
static bool soc_early;
module_param(soc_early, bool, 0644);
MODULE_PARM_DESC(soc_early, "deep, with soc_late: write the PMGR/SPMI part of soc_sleep (quiesce + go) in prepare_late, before the secondaries go down; PMU part stays in .enter");
/*
 * soc_cpustop (2026-11-18): with soc_early, CPU0 first writes the PMGR CPUSTART stop bits of
 * CPUs 1-7 (0x23b754000 = 0xfe, the register each core writes for itself in cpuidle-apple's
 * apple_cpuhp_die). XNU trace 10: the secondaries write CPUSTART (183.98) before CPU0's quiesce
 * and go (184.06), and only then do their last DEEP_SLEEP WFI. The go in prepare_late without it
 * reset the SoC within seconds (socE2/E3/E4, with and without secondary_deep).
 */
static bool soc_cpustop;
module_param(soc_cpustop, bool, 0644);
MODULE_PARM_DESC(soc_cpustop, "deep, with soc_early: write the CPUSTART stop bits of CPUs 1-7 before the early quiesce and go, as XNU");
/* soc_go_delay_ms (2026-11-18): soc_early, CPU0 spins N ms after the go (timing the fast reset) */
static unsigned int soc_go_delay_ms;
module_param(soc_go_delay_ms, uint, 0644);
MODULE_PARM_DESC(soc_go_delay_ms, "deep, with soc_early: busy-wait N ms right after the early go (diagnostic)");
/*
 * soc_pre (2026-11-18): with soc_early, macOS steps that come before its quiesce + go and that Linux
 * does not do (HV trace 15, 322.90-323.47). The go in prepare_late resets the SoC at once without them.
 *   0x1 temp sensor quiesce (AppleT8015TempSensor IOPlatformQuiesceAction, 323.29-323.42): 12 blocks,
 *       +4 = 0xffffffff, +8 = 0x1013c00 then 0x2003c00 (iBoot/m1n1 leave +4 = 0, +8 = 0x13c00); restored at wake.
 *       Since 2026-11-19 (review S1) also with soc_late alone: planned for .enter, before the quiesce, as macOS.
 *   0x2 (review S2, 2026-11-19) the two fabric ports macOS quiesces and soc_sleep 0x01 did not: 0x26a008000 and
 *       0x205008000 (+0x14 = 1, +0x2bc = 1, 2, 4; trace 16 226.58), first, as macOS; +0x14 restored at wake
 *   0x100 diagnostic: the go (0x23b718000/4 = 0) and the soc_go_delay_ms spin run inside stop_machine(), every
 *       other CPU spinning with IRQs off (XNU's secondaries are parked at the go; Linux's take ticks/IPIs in WFI)
 */
/*
 * soc_park (2026-11-18): CPU mask. In .enter (soc_late, go not early), right after the planned go
 * write, cpuidle-apple's apple_cpuhp_park_deep() restarts these offline secondaries (7 down to 1)
 * into an m1n1 "park deep" (CPUSTART stop, ACC_OVRD DEEP_SLEEP, CYC_OVRD 0x3000001, gated WFI),
 * as XNU takes its cores down after the go; then the rest of the plan (0x23d2c4000, PMU) runs.
 */
static unsigned int soc_park;
module_param(soc_park, uint, 0644);
MODULE_PARM_DESC(soc_park, "deep, soc_late: CPU mask to restart into an m1n1 DEEP_SLEEP park right after the go in .enter (0xfe = CPUs 1-7)");
/*
 * soc_hold / soc_hold_last (review K1, 2026-11-19): CPU masks. In .enter (soc_late), before the first
 * planned write, cpuidle-apple's apple_cpuhp_park_hold() restarts these offline secondaries into
 * m1n1's park deep with a hold flag: they stay online (WFE in m1n1) across the go and the PMU writes,
 * as XNU's cores do. After the plan they are released, 7 down to 1, each down before the next; those
 * also in soc_hold_last wait in m1n1 for CPU0 to power off, so they go down last (XNU: E 3-1, CPU0,
 * P 7-4). Cores in soc_hold are left out of soc_park. 0 = off (default).
 */
static unsigned int soc_hold, soc_hold_last;
module_param(soc_hold, uint, 0644);
MODULE_PARM_DESC(soc_hold, "deep, soc_late: CPU mask held online in m1n1 across the go, released after the plan (0xfe = CPUs 1-7)");
module_param(soc_hold_last, uint, 0644);
MODULE_PARM_DESC(soc_hold_last, "with soc_hold: CPUs that wait for CPU0 to power off before going down (0xf0 = P cores, as XNU)");
static bool soc_hold_nodeep;
module_param(soc_hold_nodeep, bool, 0644);
MODULE_PARM_DESC(soc_hold_nodeep, "with soc_hold, diagnostic: park the held cores without ACC_OVRD_DEEP_SLEEP (needed without the go: a non-last core with DEEP_SLEEP hangs the Mac)");
static unsigned int soc_pre;
module_param(soc_pre, uint, 0644);
MODULE_PARM_DESC(soc_pre, "deep, soc_late or soc_early: macOS pre-go steps (mask): 0x1 temp sensor quiesce, 0x2 fabric ports 0x26a/0x205 quiesce, 0x100 (soc_early) go + soc_go_delay_ms inside stop_machine (diagnostic)");
static const u64 soc_tsens[] = {
	0x211ef1600ULL, 0x211ef1400ULL, 0x211ee1600ULL, 0x211ef1200ULL, 0x211ee1400ULL, 0x211ee9000ULL,
	0x210ee1600ULL, 0x211ee1800ULL, 0x210ee1000ULL, 0x23b281200ULL, 0x23b281000ULL, 0x23b281400ULL,
};

#define SOC_PLAN_MAX	128
static struct soc_op {
	bool pmu, w64, save;
	phys_addr_t pa;
	void __iomem *va;
	u64 val;
	u16 addr;
	u8 len, buf[4];
} soc_plan[SOC_PLAN_MAX];
static int soc_nplan;
static bool soc_defer;		/* soc_sleep_request(): build soc_plan[] instead of writing */
static void __iomem *soc_spmi_va;
static u8 soc_pmu_sid;

static void soc_plan_add(struct soc_op *op)
{
	if (soc_nplan >= SOC_PLAN_MAX) {
		pr_err("soc_late: plan full, %09llx/%04x dropped\n", (u64)op->pa, op->addr);
		return;
	}
	soc_plan[soc_nplan++] = *op;
}

static void soc_wr(phys_addr_t pa, u64 val, bool w64, bool save)
{
	void __iomem *va;
	unsigned long off = pa & ~PAGE_MASK;
	u64 old, rb;

	if (soc_defer) {
		struct soc_op op = { .pa = pa, .val = val, .w64 = w64, .save = save };

		op.va = ioremap_np(pa & PAGE_MASK, PAGE_SIZE);
		if (!op.va) {
			pr_err("soc_late: ioremap %09llx failed\n", (u64)pa);
			return;
		}
		op.va += off;
		soc_plan_add(&op);
		return;
	}
	va = ioremap_np(pa & PAGE_MASK, PAGE_SIZE);
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
	int r0, ret;

	if (soc_defer) {
		struct soc_op op = { .pmu = true, .addr = addr, .len = len };

		memcpy(op.buf, buf, len);
		r0 = spmi_ext_register_readl(pmu, addr, old, len);
		pr_info("soc_late: planned PMU %04x <- %*ph (now %*ph, read %d)\n", addr, (int)len, buf,
			(int)len, old, r0);
		soc_plan_add(&op);
		return 0;
	}
	r0 = spmi_ext_register_readl(pmu, addr, old, len);
	ret = spmi_ext_register_writel(pmu, addr, buf, len);

	pr_info("soc_sleep: PMU %04x (%zu bytes, read %d: %*ph) <- %*ph: %d\n", addr, len, r0,
		(int)len, old, (int)len, buf, ret);
	return ret;
}

static const u32 soc_ps_off[] = { 0x3f0 /* ans2 */, 0x418 /* apcie_st */, 0x348 /* apcie */, 0x3e8 /* apcie_gp */ };
static void __iomem *soc_ps_va;
#define SEP_MBOX_PA		0x242408000ULL
static void __iomem *soc_sep_mbox;

static void soc_ps_log(const char *when)
{
	if (!soc_ps_va)
		return;
	pr_info("soc_late: %s: ps ans2 %08x apcie_st %08x apcie %08x apcie_gp %08x\n", when,
		readl(soc_ps_va + soc_ps_off[0]), readl(soc_ps_va + soc_ps_off[1]),
		readl(soc_ps_va + soc_ps_off[2]), readl(soc_ps_va + soc_ps_off[3]));
}

static void __iomem *soc_go_va;

static int soc_go_stopped(void *unused)
{
	writel(0, soc_go_va);
	writel(0, soc_go_va + 4);
	for (unsigned int i = 0; i < soc_go_delay_ms; i++)
		udelay(1000);
	return 0;
}

/*
 * disp_cycle (review S3, 2026-11-19): mask, 0x1 disp0, 0x2 dispext, 0 = off (default).
 * Both HV traces (16: 226.54-226.63, 17: 264.91-265.11), 3.7 s before the CPUs go down, macOS power-
 * cycles the display fabric although all of it is already off: RMX on (+ the 0x225000000 blocks
 * programmed), DISP0_FE on, fabric port 0x232008000 reprogrammed, DISP0_CPU0 on, same for DISPEXT
 * (0x272008000); ~150 ms later DISP0_CPU0 off, port quiesce (+0x14 = 1, +0x2bc = 1, 2, 4), DISP0_FE off,
 * same for dispext, RMX off. It leaves both ports quiesced before their domain is gated; Linux gates
 * DISP0_FE (if it does) without that quiesce, and never touches the 0x232/0x272 ports.
 * Here, in prepare_late (devices suspended): RMX on, for each selected pipe FE on if off (CPU0 left as is if on,
 * else not touched), port quiesce, FE off; RMX back off. At wake FE is put back on if it was on (port state is then
 * whatever the power-up gives; the display resume will tell whether that is enough).
 * 0x4 (2026-11-20): also the macOS power-up init tables (apple-sleep-macos-disp.h: RMX blocks after RMX on,
 * each port after its FE on); 0x8: CPU0 domains on after the init, off before the quiesce, as macOS (only
 * a CPU0 whose target is off, the macOS start state: Linux keeps DISP0_CPU0 on for the warm DCP).
 * Both phases run dispext first, as macOS.
 */
static unsigned int disp_cycle;
module_param(disp_cycle, uint, 0644);
MODULE_PARM_DESC(disp_cycle, "prepare_late: macOS display fabric cycle (mask): 0x1 disp0 (FE on, port 0x232008000 quiesce, FE off), 0x2 dispext (port 0x272008000), 0x4 + macOS RMX/port init tables after power-up, 0x8 + CPU0 domains on/off as macOS; FE restored at wake");

#include "apple-sleep-macos-disp.h"

#define DC_PS_ACTUAL(v)	(((v) >> 4) & 0xf)
/* macOS order: dispext first, then disp0, in both phases */
static const struct {
	u32 fe, cpu0; phys_addr_t port; const char *name;
	const struct as_disp_wr *init; size_t ninit;
} dc_pipe[] = {
	{ 0x360, 0x10018, 0x232008000ULL, "disp0", as_disp_port_disp0, ARRAY_SIZE(as_disp_port_disp0) },
	{ 0x368, 0x378, 0x272008000ULL, "dispext", as_disp_port_dispext, ARRAY_SIZE(as_disp_port_dispext) },
};
static const int dc_order[] = { 1, 0 };
static u32 dc_fe_old[2];
static bool dc_done[2];

static int dc_ps_set(void __iomem *ps, u32 target)
{
	u32 v = readl(ps);

	/* keep the other fields, clear the W1C WAS_* flags (bits 9:8), new target in 3:0 */
	writel((v & ~0x30fu) | target, ps);
	for (int t = 0; t < 1000; t++) {
		if (DC_PS_ACTUAL(readl(ps)) == target)
			return 0;
		udelay(10);
	}
	return -ETIMEDOUT;
}

/* write a macOS table, one 16 KiB mapping per page as boot_replay */
static int dc_table(const char *what, const struct as_disp_wr *w, size_t n)
{
	void __iomem *va = NULL;
	u64 page = 0;
	size_t k;

	for (k = 0; k < n; k++) {
		u64 pa = 0x200000000ULL + w[k].pa_lo;

		if (!va || (pa & ~0x3fffULL) != page) {
			if (va)
				iounmap(va);
			page = pa & ~0x3fffULL;
			va = ioremap_np(page, 0x4000);
			if (!va) {
				pr_err("disp_cycle: %s: ioremap %09llx failed after %zu writes\n", what, page, k);
				return -ENOMEM;
			}
		}
		writel(w[k].val, va + (pa & 0x3fff));
	}
	if (va)
		iounmap(va);
	pr_info("disp_cycle: %s: %zu macOS init writes\n", what, n);
	return 0;
}

static void disp_cycle_run(void)
{
	void __iomem *ps = ioremap_np(0x23b700000ULL, 0x11000);
	bool cpu0_on[2] = { false, false };
	u32 rmx_old;

	if (!disp_cycle || !ps)
		goto out;
	pr_info("disp_cycle: before: RMX %08x DISP0_FE %08x DISPEXT_FE %08x DISP0_CPU0 %08x DISPEXT_CPU0 %08x\n",
		readl(ps + 0x350), readl(ps + 0x360), readl(ps + 0x368), readl(ps + 0x10018),
		readl(ps + 0x378));
	/* RMX is the FEs' parent (FE PS bit 11 PARENT_OFF): macOS powers it first, gates it last */
	rmx_old = readl(ps + 0x350);
	if (DC_PS_ACTUAL(rmx_old) != 0xf && dc_ps_set(ps + 0x350, 0xf)) {
		pr_err("disp_cycle: RMX did not power on (%08x)\n", readl(ps + 0x350));
		dc_ps_set(ps + 0x350, rmx_old & 0xf);
		goto out;
	}
	if (disp_cycle & 0x4)
		dc_table("RMX 0x225000000", as_disp_rmx, ARRAY_SIZE(as_disp_rmx));

	/* phase 1, as macOS: per pipe FE on, port init (0x4), CPU0 on (0x8) */
	for (int j = 0; j < ARRAY_SIZE(dc_order); j++) {
		int i = dc_order[j];
		void __iomem *fe = ps + dc_pipe[i].fe;

		dc_done[i] = false;
		if (!(disp_cycle & BIT(i)))
			continue;
		dc_fe_old[i] = readl(fe);
		if (DC_PS_ACTUAL(dc_fe_old[i]) != 0xf && dc_ps_set(fe, 0xf)) {
			pr_err("disp_cycle: %s FE did not power on (%08x), skipped\n", dc_pipe[i].name, readl(fe));
			dc_ps_set(fe, dc_fe_old[i] & 0xf);
			continue;
		}
		dc_done[i] = true;
		if (disp_cycle & 0x4)
			dc_table(dc_pipe[i].name, dc_pipe[i].init, dc_pipe[i].ninit);
		/* only a CPU0 that is off (macOS start state); Linux keeps DISP0_CPU0 on for the warm DCP */
		if ((disp_cycle & 0x8) && !(readl(ps + dc_pipe[i].cpu0) & 0xf)) {
			int r = dc_ps_set(ps + dc_pipe[i].cpu0, 0xf);

			cpu0_on[i] = !r;
			pr_info("disp_cycle: %s CPU0 on: %08x%s\n", dc_pipe[i].name,
				readl(ps + dc_pipe[i].cpu0), r ? " (TIMEOUT)" : "");
		}
	}

	/* phase 2, as macOS: per pipe CPU0 off, port quiesce, FE off */
	for (int j = 0; j < ARRAY_SIZE(dc_order); j++) {
		int i = dc_order[j];
		void __iomem *fe = ps + dc_pipe[i].fe, *port;
		u32 q0, s0;
		int off;

		if (!dc_done[i])
			continue;
		if (cpu0_on[i]) {
			off = dc_ps_set(ps + dc_pipe[i].cpu0, 0);
			pr_info("disp_cycle: %s CPU0 off: %08x%s\n", dc_pipe[i].name,
				readl(ps + dc_pipe[i].cpu0), off ? " (TIMEOUT)" : "");
		}
		port = ioremap_np(dc_pipe[i].port, 0x1000);
		if (port) {
			q0 = readl(port + 0x14);
			s0 = readl(port + 0x2bc);
			writel(1, port + 0x14);
			writel(1, port + 0x2bc);
			writel(2, port + 0x2bc);
			writel(4, port + 0x2bc);
			pr_info("disp_cycle: %s FE %08x -> %08x, port +0x14 %x +0x2bc %x -> +0x14 %x +0x2bc %x\n",
				dc_pipe[i].name, dc_fe_old[i], readl(fe), q0, s0, readl(port + 0x14),
				readl(port + 0x2bc));
			iounmap(port);
		}
		off = dc_ps_set(fe, 0);
		pr_info("disp_cycle: %s FE off: %08x%s\n", dc_pipe[i].name, readl(fe), off ? " (TIMEOUT)" : "");
	}
	if (DC_PS_ACTUAL(rmx_old) != 0xf)
		dc_ps_set(ps + 0x350, rmx_old & 0xf);
	pr_info("disp_cycle: after: RMX %08x DISP0_FE %08x DISPEXT_FE %08x\n",
		readl(ps + 0x350), readl(ps + 0x360), readl(ps + 0x368));
out:
	if (ps)
		iounmap(ps);
}

static void disp_cycle_restore(void)
{
	void __iomem *ps;

	if (!dc_done[0] && !dc_done[1])
		return;
	ps = ioremap_np(0x23b700000ULL, 0x1000);
	if (!ps)
		return;
	for (int i = 0; i < ARRAY_SIZE(dc_pipe); i++) {
		if (!dc_done[i])
			continue;
		if (DC_PS_ACTUAL(dc_fe_old[i]) == 0xf) {
			int ret = dc_ps_set(ps + dc_pipe[i].fe, dc_fe_old[i] & 0xf);

			pr_info("disp_cycle: %s FE back on: %08x%s\n", dc_pipe[i].name,
				readl(ps + dc_pipe[i].fe), ret ? " (TIMEOUT)" : "");
		}
		dc_done[i] = false;
	}
	iounmap(ps);
}

static void soc_sleep_request(void)
{
	static const u64 cl[] = { 0x210e20000ULL, 0x211e20000ULL };	/* E, P cluster */
	static const u64 blk[] = { 0x68c008000ULL, 0x234008000ULL, 0x279008000ULL };
	static const u32 blk_ps[] = { 0x348 /* apcie */, 0 /* pmgr[45], agent unknown */, 0x3f0 /* ans2 */ };

	if (!soc_sleep)
		return;
	pr_info("soc_sleep: mask %#x, skip %#x\n", soc_sleep, soc_skip);
	bool early = soc_defer && soc_early;

	if (early) {
		pr_info("soc_early: PMGR/SPMI part now, PMU part planned for .enter\n");
		soc_defer = false;
		if (soc_cpustop)
			soc_wr(0x23b754000ULL, 0xfe, false, false);	/* CPUSTART stop, CPUs 1-7 */
	}
	/* review S2: macOS quiesces these two ports at 226.58, before the PMP stop */
	if (soc_pre & 0x2)
		for (int i = 0; i < 2; i++) {
			phys_addr_t b = i ? 0x205008000ULL : 0x26a008000ULL;

			soc_wr(b + 0x14, 1, false, true);
			soc_wr(b + 0x2bc, 1, false, false);
			soc_wr(b + 0x2bc, 2, false, false);
			soc_wr(b + 0x2bc, 4, false, false);
		}
	/* review S1: temp sensors (macOS 228.16-228.27), early or planned for .enter */
	if (soc_pre & 0x1)
		for (int i = 0; i < ARRAY_SIZE(soc_tsens); i++) {
			soc_wr(soc_tsens[i] + 4, 0xffffffff, false, true);
			soc_wr(soc_tsens[i] + 8, 0x1013c00, false, true);
			soc_wr(soc_tsens[i] + 8, 0x2003c00, false, false);
		}
	if (!soc_ps_va)
		soc_ps_va = ioremap_np(0x23b700000ULL, 0x1000);
	soc_ps_log("prepare_late");
	if ((soc_sleep & 0x100) && !soc_sep_mbox)
		soc_sep_mbox = ioremap_np(SEP_MBOX_PA, 0x4000);
	soc_nsaved = 0;
	/*
	 * 0x80 (2026-11-18, HV trace 15 322.98-323.05): macOS's AppleGen1SPMIController sleep action
	 * writes +0x38 = 1 on SPMI1, SPMI0 and the AOP SPMI (0 under iBoot/Linux; an SPMI bus sleep /
	 * idle control?), before the PMGR quiesce. Saved, so back to 0 at wake for Linux's SPMI driver.
	 * soc_skip 0x200 leaves out the AOP SPMI.
	 */
	if (soc_sleep & 0x80) {
		soc_wr(0x23d0e1338ULL, 1, false, true);		/* SPMI1 */
		soc_wr(0x23d0d9338ULL, 1, false, true);		/* SPMI0 (PMU) */
		if (SOC_G(0x200))
			soc_wr(0x24a829338ULL, 1, false, true);	/* AOP SPMI */
	}
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

			if (!SOC_G(0x40 << i))	/* 0x40 0x68c (PCIe axi), 0x80 0x234, 0x100 0x279 */
				continue;
			/*
			 * 2026-11-17: Linux has already power-gated some agents here (genpd, noirq), macOS
			 * has not. Quiescing the fabric port of an off agent (ans2 0f002300 at .enter,
			 * block 0x279 = ans[4]) stops the wake; nothing to drain from an off agent, skip.
			 */
			if (blk_ps[i] && soc_ps_va && !(readl(soc_ps_va + blk_ps[i]) & 0xf0)) {
				pr_info("soc_sleep: %09llx quiesce skipped, agent ps +%#x off (%08x)\n",
					(u64)b, blk_ps[i], readl(soc_ps_va + blk_ps[i]));
				continue;
			}
			/* restored at wake: 0x279008000 is ans[4] (NVMe); left quiesced, the resume hangs
			 * (2026-11-17 timer test). macOS gets the block back reset from SoC sleep. */
			soc_wr(b + 0x14, 1, false, true);
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
	if (early && (soc_pre & 0x100) && (soc_sleep & 0x01) && SOC_G(0x20)) {
		soc_go_va = ioremap_np(0x23b718000ULL, 0x10);
		if (soc_go_va && soc_nsaved + 2 <= SOC_SAVE_MAX) {
			for (int i = 0; i < 2; i++) {
				soc_saved[soc_nsaved].pa = 0x23b718000ULL + 4 * i;
				soc_saved[soc_nsaved].old = readl(soc_go_va + 4 * i);
				soc_saved[soc_nsaved].w64 = false;
				soc_nsaved++;
			}
			pr_info("soc_early: go inside stop_machine, %u ms spin, 718000 %x 718004 %x\n",
				soc_go_delay_ms, readl(soc_go_va), readl(soc_go_va + 4));
			stop_machine(soc_go_stopped, NULL, NULL);
			pr_info("soc_early: stop_machine go done, 718000 reads %x\n", readl(soc_go_va));
		}
	} else if ((soc_sleep & 0x01) && SOC_G(0x20)) {
		soc_wr(0x23b718000ULL, 0, false, true);
		soc_wr(0x23b718004ULL, 0, false, true);
	}
	if (early && soc_go_delay_ms && !(soc_pre & 0x100) && (soc_sleep & 0x01) && SOC_G(0x20)) {
		pr_info("soc_early: go done, spinning %u ms\n", soc_go_delay_ms);
		for (unsigned int i = 0; i < soc_go_delay_ms; i++)
			udelay(1000);
		pr_info("soc_early: spin done\n");
	}
	if (soc_sleep & 0x04)
		soc_wr(0x23b7001d0ULL, 0x400024f, false, true);
	if (early)
		soc_defer = true;	/* soc_early: the rest goes to .enter */
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
		if (soc_defer && !soc_spmi_va) {
			struct device_node *ctl = of_get_parent(pmu->dev.of_node);
			struct resource res;

			if (ctl && !of_address_to_resource(ctl, 0, &res))
				soc_spmi_va = ioremap_np(res.start, resource_size(&res));
			of_node_put(ctl);
			soc_pmu_sid = pmu->usid;
			if (!soc_spmi_va) {
				pr_err("soc_late: SPMI controller not mapped, no PMU writes\n");
				put_device(&pmu->dev);
				return;
			}
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
 * Raw SPMI extended register write, IRQs off (spmi-apple-controller.c protocol: command word,
 * data words, then one reply word with the ACK bit). The notify IRQ it raises is acked here.
 */
static int soc_spmi_writel_atomic(u8 sid, u16 addr, const u8 *buf, size_t len)
{
	void __iomem *r = soc_spmi_va;
	u32 w, reply;
	size_t i, j;
	int t, ret;

	if (soc_sleep & 0x80)
		writel(0, r + 0x38);	/* macOS: bus awake for the command (trace 15, 323.563) */
	writel(0x30 | (len - 1) | (u32)sid << 8 | (u32)addr << 16 | BIT(15), r + 0x4);
	for (i = 0; i < len; i += j) {
		j = min_t(size_t, 4, len - i);
		w = 0;
		memcpy(&w, buf + i, j);
		writel(w, r + 0x4);
	}
	for (t = 0; t < 5000 && (readl(r) & BIT(24)); t++)	/* RX FIFO empty, <= 50 ms */
		udelay(10);
	if (readl(r) & BIT(24)) {
		ret = -ETIMEDOUT;
	} else {
		reply = readl(r + 0x8);
		writel(BIT(0), r + 0x60 + (256 / 32) * 4);	/* ack SPMI_IRQ_NOTIFY */
		ret = (reply & BIT(15)) ? 0 : -EIO;
	}
	if (soc_sleep & 0x80)
		writel(1, r + 0x38);	/* and back to bus sleep after it */
	return ret;
}

/*
 * Agent power states seen by the quiesce in .enter (2026-11-17: the ANS fabric quiesce 0x279008xxx
 * stops the wake; is ps_ans2 already off by then, i.e. are we quiescing a power-gated agent?)
 */


/*
 * soc_sleep 0x100 (2026-11-18, HV trace 15 322.12 s): macOS AppleSEPManager::_setPowerState(2 -> 0)
 * sends the SEP a sleep message on endpoint 0 (tag 8, type 0x0c, param 2) and the SEP answers type
 * 0x01 (322.23 s). Linux never sends it. Sent here through the SEP mailbox registers (ASC v4, as
 * drivers/soc/apple/mailbox.c): CPU0 alone, IRQs off, so the replies are drained and logged here and
 * the Linux SEP driver never sees them. The wake message is unknown: the SEP stays asleep.
 */
#define SEP_MSG_SLEEP		0x00000000020c0800ULL

static void soc_sep_sleep(void)
{
	void __iomem *m = soc_sep_mbox;
	u64 r0, r1;
	int t, n = 0;
	bool ack = false;

	if (!(soc_sleep & 0x100) || !m)
		return;
	for (t = 0; t < 1000 && (readl(m + 0x110) & BIT(16)); t++)	/* A2I full */
		udelay(100);
	if (readl(m + 0x110) & BIT(16)) {
		pr_err("soc_late: SEP mailbox A2I full, sleep message not sent\n");
		return;
	}
	writeq(SEP_MSG_SLEEP, m + 0x800);
	writeq(0, m + 0x808);
	for (t = 0; t < 5000 && !ack; t++) {				/* <= 500 ms */
		while (!(readl(m + 0x114) & BIT(17)) && n < 16) {	/* I2A not empty */
			r0 = readq(m + 0x830);
			r1 = readq(m + 0x838);
			n++;
			pr_info("soc_late: SEP reply %016llx %016llx after %d00 us\n", r0, r1, t);
			if ((r0 & 0xff) == 0 && ((r0 >> 16) & 0xff) == 0x01)
				ack = true;
		}
		if (!ack)
			udelay(100);
	}
	pr_info("soc_late: SEP sleep message %016llx: %s (%d replies)\n", SEP_MSG_SLEEP,
		ack ? "acked" : "NO ack", n);
}

/* .enter, CPU0 alone, IRQs off: the planned soc_sleep writes, in the macOS order */
static void soc_plan_run(void)
{
	unsigned int park = soc_park & ~soc_hold;
	bool parked = false;

	if (soc_hold)
		pr_info("soc_late: park_hold before the plan: %d cores held (mask %#x, last %#x)\n",
			apple_cpuhp_park_hold(soc_hold, soc_hold_last, !soc_hold_nodeep), soc_hold, soc_hold_last);
	soc_sep_sleep();
	soc_ps_log(".enter");
	for (int i = 0; i < soc_nplan; i++) {
		struct soc_op *op = &soc_plan[i];
		u64 old, rb;

		if (op->pmu) {
			int ret = soc_spmi_writel_atomic(soc_pmu_sid, op->addr, op->buf, op->len);

			pr_info("soc_late: PMU %04x <- %*ph: %d\n", op->addr, op->len, op->buf, ret);
			continue;
		}
		old = op->w64 ? readq(op->va) : readl(op->va);
		if (op->save && soc_nsaved < SOC_SAVE_MAX) {
			soc_saved[soc_nsaved].pa = op->pa;
			soc_saved[soc_nsaved].old = old;
			soc_saved[soc_nsaved].w64 = op->w64;
			soc_nsaved++;
		}
		if (op->w64)
			writeq(op->val, op->va);
		else
			writel(op->val, op->va);
		rb = op->w64 ? readq(op->va) : readl(op->va);
		pr_info("soc_late: %09llx %llx -> %llx (reads %llx)\n", (u64)op->pa, old, op->val, rb);
		if (op->pa == 0x23b718004ULL && park && !parked) {
			pr_info("soc_late: park_deep after the go: %d cores parked (mask %#x)\n",
				apple_cpuhp_park_deep(park), park);
			parked = true;
		}
	}
	/* no go in the plan (soc_skip 0x20): park at the end, to test the mechanism alone */
	if (park && !parked)
		pr_info("soc_late: park_deep, no go: %d cores parked (mask %#x)\n",
			apple_cpuhp_park_deep(park), park);
	if (soc_hold)
		pr_info("soc_late: park_release after the plan: %d cores down, %#x wait for CPU0\n",
			apple_cpuhp_park_release(), soc_hold_last & soc_hold);
}

static void soc_plan_free(void)
{
	for (int i = 0; i < soc_nplan; i++)
		if (soc_plan[i].va)
			iounmap((void __iomem *)((unsigned long)soc_plan[i].va & PAGE_MASK));
	soc_nplan = 0;
	if (soc_spmi_va)
		iounmap(soc_spmi_va);
	soc_spmi_va = NULL;
}

/*
 * soc_spmi_echo=<addr>: awake check of soc_spmi_writel_atomic(): read one PMU byte through the
 * SPMI driver, write the same value back through the raw path with IRQs off, read it again.
 */
static int soc_spmi_echo_set(const char *val, const struct kernel_param *kp)
{
	struct device_node *np, *ctl;
	struct spmi_device *pmu;
	struct resource res;
	unsigned long flags;
	u8 v0 = 0, v1 = 0;
	unsigned int addr;
	int r0, r1, ret = kstrtouint(val, 0, &addr);

	if (ret || addr > 0xffff)
		return -EINVAL;
	np = of_find_compatible_node(NULL, NULL, "apple,sera-pmic");
	pmu = np ? spmi_find_device_by_of_node(np) : NULL;
	of_node_put(np);
	if (IS_ERR_OR_NULL(pmu))
		return -ENODEV;
	ctl = of_get_parent(pmu->dev.of_node);
	if (ctl && !of_address_to_resource(ctl, 0, &res))
		soc_spmi_va = ioremap_np(res.start, resource_size(&res));
	of_node_put(ctl);
	if (!soc_spmi_va) {
		put_device(&pmu->dev);
		return -ENOMEM;
	}
	r0 = spmi_ext_register_readl(pmu, addr, &v0, 1);
	local_irq_save(flags);
	ret = soc_spmi_writel_atomic(pmu->usid, addr, &v0, 1);
	local_irq_restore(flags);
	r1 = spmi_ext_register_readl(pmu, addr, &v1, 1);
	pr_info("soc_spmi_echo: PMU sid %u %04x = %02x (read %d), atomic write back: %d, now %02x (read %d)\n",
		pmu->usid, addr, v0, r0, ret, v1, r1);
	iounmap(soc_spmi_va);
	soc_spmi_va = NULL;
	put_device(&pmu->dev);
	return 0;
}

static const struct kernel_param_ops soc_spmi_echo_ops = {
	.set = soc_spmi_echo_set,
};
module_param_cb(soc_spmi_echo, &soc_spmi_echo_ops, NULL, 0200);
MODULE_PARM_DESC(soc_spmi_echo, "awake: check the atomic SPMI path, write a PMU register's own value back (logged)");

/*
 * boot_replay=<mask> (2026-11-18): write now (awake) the macOS boot-time PMGR set-up that iBoot leaves
 * at 0 (HV trace 15; apple-sleep-macos-boot.h). Candidates for the missing SoC-sleep wake, and 0x8 for
 * power. The sleep quiesce (soc_sleep 0x01, groups 0x02/0x08) clears the enables at entry and restores
 * them at wake, as macOS. Order as macOS: 0x8 (1.73 s), 0x1, 0x2 (1.82-1.90 s), 0x4 (1.91 s).
 *   0x1 0x23b734000 table (+0x120.., enable +0 last)   0x2 0x23b778000 / 0x23b77c000 tables + enables
 *   0x4 0x23b748000/400/800/c00 = 1, 0x23b71c000/040 = 0x80000000
 *   0x8 PMGR[31] 0x23b204000 thermal/voltage-monitor thresholds
 */
static unsigned int boot_replay_done;

static void boot_replay_group(unsigned int g, const struct as_boot_wr *w, size_t n)
{
	void __iomem *va = NULL;
	u64 page = ~0ULL;
	size_t k;

	for (k = 0; k < n; k++) {
		u64 pa = 0x200000000ULL + w[k].pa_lo;

		if ((pa & ~0xfffULL) != page) {
			if (va)
				iounmap(va);
			page = pa & ~0xfffULL;
			va = ioremap_np(page, 0x1000);
			if (!va) {
				pr_err("boot_replay 0x%x: ioremap %09llx failed after %zu writes\n", g, page, k);
				return;
			}
		}
		writel(w[k].val, va + (pa & 0xfff));
	}
	if (va)
		iounmap(va);
	boot_replay_done |= g;
	pr_info("boot_replay 0x%x: %zu writes done\n", g, n);
}

static u32 boot_replay_peek(u64 pa)
{
	void __iomem *va = ioremap_np(pa & ~0xfffULL, 0x1000);
	u32 v;

	if (!va)
		return 0xdeadbeef;
	v = readl(va + (pa & 0xfff));
	iounmap(va);
	return v;
}

static int boot_replay_set(const char *val, const struct kernel_param *kp)
{
	unsigned int mask;
	int ret = kstrtouint(val, 0, &mask);

	if (ret)
		return ret;
	if (mask & 0x8)
		boot_replay_group(0x8, as_boot_g8, ARRAY_SIZE(as_boot_g8));
	if (mask & 0x1)
		boot_replay_group(0x1, as_boot_g1, ARRAY_SIZE(as_boot_g1));
	if (mask & 0x2)
		boot_replay_group(0x2, as_boot_g2, ARRAY_SIZE(as_boot_g2));
	if (mask & 0x4)
		boot_replay_group(0x4, as_boot_g4, ARRAY_SIZE(as_boot_g4));
	pr_info("boot_replay: now 734000 %x 734120 %x 778000 %x 778100 %x 77c000 %x 748000 %x 71c000 %x 204004 %x\n",
		boot_replay_peek(0x23b734000ULL), boot_replay_peek(0x23b734120ULL),
		boot_replay_peek(0x23b778000ULL), boot_replay_peek(0x23b778100ULL),
		boot_replay_peek(0x23b77c000ULL), boot_replay_peek(0x23b748000ULL),
		boot_replay_peek(0x23b71c000ULL), boot_replay_peek(0x23b204004ULL));
	return 0;
}

static int boot_replay_get(char *buf, const struct kernel_param *kp)
{
	return sysfs_emit(buf, "%#x\n", boot_replay_done);
}

static const struct kernel_param_ops boot_replay_ops = {
	.set = boot_replay_set,
	.get = boot_replay_get,
};
module_param_cb(boot_replay, &boot_replay_ops, NULL, 0644);
MODULE_PARM_DESC(boot_replay, "awake: replay macOS boot PMGR set-up missing under iBoot/Linux (mask: 0x1 734 table, 0x2 778/77c tables, 0x4 748/71c000, 0x8 PMGR[31] thermal table); reads back the groups done");

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
	disp_cycle_run();
	soc_nsaved = 0;
	if (soc_late && soc_sleep) {
		soc_defer = true;
		soc_sleep_request();
		soc_defer = false;
		pr_info("soc_late: %d writes planned for .enter\n", soc_nplan);
	} else {
		soc_sleep_request();
	}
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

/*
 * deep_wdt_s=N (test aid for cpuidle_apple.ecluster_deep, 2026-11-15): if the E cluster in
 * ACC_OVRD_DEEP_SLEEP never wakes, nothing is left running to notice. Arm the AON watchdog
 * (0x23d2b0000, as apple_wdt / m1n1's wake beacon: +0x10 count, +0x14 bite, +0x1c ctrl bit 2
 * reset) to reset the SoC N s after the deep entry; disarmed again at wake. apple_wdt is already
 * suspended by then (it re-arms itself at resume if it was running). 24 MHz, max 178 s.
 */
#define AS_WDT_PHYS		0x23d2b0000ULL
#define AS_WDT_CUR		0x10
#define AS_WDT_BITE		0x14
#define AS_WDT_CTRL		0x1c
#define AS_WDT_RESET_EN		BIT(2)
static unsigned int deep_wdt_s;
module_param(deep_wdt_s, uint, 0644);
MODULE_PARM_DESC(deep_wdt_s, "deep test aid: AON watchdog resets the SoC N s (max 178) after the deep entry if it has not woken (0 = off)");
static void __iomem *as_wdt_va;
static u32 as_wdt_saved[3];	/* cur, bite, ctrl */
static bool as_wdt_armed;

static void as_wdt_arm(void)
{
	u64 bite;

	if (!deep_wdt_s)
		return;
	if (!as_wdt_va)
		as_wdt_va = ioremap_np(AS_WDT_PHYS, 0x20);
	if (!as_wdt_va)
		return;
	bite = min_t(u64, (u64)deep_wdt_s * 24000000ULL, U32_MAX);
	as_wdt_saved[0] = readl(as_wdt_va + AS_WDT_CUR);
	as_wdt_saved[1] = readl(as_wdt_va + AS_WDT_BITE);
	as_wdt_saved[2] = readl(as_wdt_va + AS_WDT_CTRL);
	writel(0, as_wdt_va + AS_WDT_CUR);
	writel(bite, as_wdt_va + AS_WDT_BITE);
	writel(AS_WDT_RESET_EN, as_wdt_va + AS_WDT_CTRL);
	as_wdt_armed = true;
}

static void as_wdt_disarm(void)
{
	u32 cur;

	if (!as_wdt_armed)
		return;
	as_wdt_armed = false;
	cur = readl(as_wdt_va + AS_WDT_CUR);
	writel(0, as_wdt_va + AS_WDT_CTRL);
	writel(as_wdt_saved[1], as_wdt_va + AS_WDT_BITE);
	writel(0, as_wdt_va + AS_WDT_CUR);
	writel(as_wdt_saved[2], as_wdt_va + AS_WDT_CTRL);
	pr_info("deep_wdt_s: disarmed after %u ms (ctrl back to %#x)\n", cur / 24000, as_wdt_saved[2]);
}

/*
 * deep_wd0_s=N (review W4, 2026-11-19): the AON timer block 0x23d2b0000 has three apple_wdt-style
 * timers: WD0 (+0x00 count, +0x04 bite, +0x08 bark, +0x0c ctrl), WD1 (+0x10, the reset watchdog above),
 * WD2 (+0x20). Under iBoot/m1n1/Linux WD0 counts at 24 MHz with ctrl 0. macOS zeroes WD0 count + ctrl
 * (and WD1 count, 0x23d2b8008/c) at sleep; the ADT ap-wake-sources have 49 NUB.APWatchdog and
 * 50 NUB.APWakeupTime. Guess: WD0 with IRQ_EN only (no RESET_EN) is the AP wake timer. Armed in .enter
 * (after the drivers, IRQs off), N s, max 178; at wake its ctrl (bit 1 IRQ_STATUS = it fired) is logged
 * and it goes back to idle (count 0, ctrl 0). The WD0 AIC IRQ is not requested by Linux (stays masked).
 */
#define AS_WD0_CUR		0x00
#define AS_WD0_BITE		0x04
#define AS_WD0_CTRL		0x0c
#define AS_WDT_IRQ_EN		BIT(0)
#define AS_WDT_IRQ_STATUS	BIT(1)
static unsigned int deep_wd0_s;
module_param(deep_wd0_s, uint, 0644);
MODULE_PARM_DESC(deep_wd0_s, "deep test: arm the AON WD0 timer with IRQ only (no reset) N s (max 178) after the deep entry, as a possible hardware wake timer (APWakeupTime?); 0 = off");
static void __iomem *as_wd0_va;
static u32 as_wd0_saved[3];	/* cur, bite, ctrl */
static bool as_wd0_armed;

static void as_wd0_arm(void)
{
	if (!deep_wd0_s || !as_wd0_va)
		return;
	as_wd0_saved[0] = readl(as_wd0_va + AS_WD0_CUR);
	as_wd0_saved[1] = readl(as_wd0_va + AS_WD0_BITE);
	as_wd0_saved[2] = readl(as_wd0_va + AS_WD0_CTRL);
	writel(0, as_wd0_va + AS_WD0_CTRL);
	writel(0, as_wd0_va + AS_WD0_CUR);
	writel(min_t(u64, (u64)deep_wd0_s * 24000000ULL, U32_MAX), as_wd0_va + AS_WD0_BITE);
	writel(AS_WDT_IRQ_STATUS, as_wd0_va + AS_WD0_CTRL);	/* clear a stale status (W1C, as apple_wdt) */
	writel(AS_WDT_IRQ_EN, as_wd0_va + AS_WD0_CTRL);
	as_wd0_armed = true;
}

static void as_wd0_disarm(void)
{
	u32 cur, ctrl;

	if (!as_wd0_armed)
		return;
	as_wd0_armed = false;
	cur = readl(as_wd0_va + AS_WD0_CUR);
	ctrl = readl(as_wd0_va + AS_WD0_CTRL);
	writel(0, as_wd0_va + AS_WD0_CTRL);
	writel(AS_WDT_IRQ_STATUS, as_wd0_va + AS_WD0_CTRL);
	writel(0, as_wd0_va + AS_WD0_CTRL);
	writel(as_wd0_saved[1], as_wd0_va + AS_WD0_BITE);
	writel(0, as_wd0_va + AS_WD0_CUR);
	writel(as_wd0_saved[2], as_wd0_va + AS_WD0_CTRL);
	pr_info("deep_wd0_s: at wake count %u ms, ctrl %#x (%s), was cur %#x bite %#x ctrl %#x before\n",
		cur / 24000, ctrl, (ctrl & AS_WDT_IRQ_STATUS) ? "FIRED" : "not fired",
		as_wd0_saved[0], as_wd0_saved[1], as_wd0_saved[2]);
}

/*
 * Keep the safety net through device resume (2026-11-16: pcie-apple power_off_gp froze in
 * resume_noirq, after the deep wake had already disarmed it): at wake the count restarts with the
 * full deep_wdt_s, the watchdog is disarmed only at PM_POST_SUSPEND, once every device has resumed.
 */
static void as_wdt_rearm_for_resume(void)
{
	if (!as_wdt_armed)
		return;
	writel(0, as_wdt_va + AS_WDT_CUR);
	pr_info("deep_wdt_s: woken, kept armed (%u s) until the end of resume\n", deep_wdt_s);
}

static int as_pm_notify(struct notifier_block *nb, unsigned long action, void *data)
{
	if (action == PM_POST_SUSPEND)
		as_wdt_disarm();
	return NOTIFY_DONE;
}

static struct notifier_block as_pm_nb = {
	.notifier_call = as_pm_notify,
};

/*
 * slc_off (2026-11-16): the MCC writes macOS makes on CPU0 right before its SoC sleep (HV trace 13,
 * 327.93 s), minus the boot-time tables Linux never programs (+0x158000/+0x24000 entry enables,
 * DCS0 counters). Bit 0: system level cache off, PLANE_CACHE_ENABLE (+0x1c00) = 0 between the
 * +0x10000c/+0x100014 = 1/0 pulses (macOS brackets every MCC config write with them), plane 0
 * only (2026-11-16: writing the saved 0 to planes 1-3 turned the cache off again), status polled. Bit 1: plane
 * +0x21008/+0x21000 = 0. Bit 2: +0x155000 = 0. Bit 3: DCS n +0x7a0 = 0 (8 channels). All saved and
 * restored in reverse order after the deep WFI loop (cache back on as m1n1 does). Only registers
 * read safely one by one under Linux are touched (some MCC reads hang the SoC).
 */
#define MCC_PHYS		0x200000000ULL
#define MCC_SIZE		0x400000
#define MCC_PLANE(p)		((p) * 0x40000)
#define MCC_DCS(n)		(0x200000 + (n) * 0x40000)
#define MCC_LOCK0		0x10000c
#define MCC_LOCK1		0x100014
#define MCC_CACHE_EN		0x1c00
#define MCC_CACHE_ST		0x1c04
#define MCC_CACHE_ST_WAYS	GENMASK(14, 5)
static unsigned int slc_off;
module_param(slc_off, uint, 0644);
MODULE_PARM_DESC(slc_off, "deep: MCC as macOS at SoC sleep: 0x1 system level cache off, 0x2 plane +0x21000/8 = 0, 0x4 +0x155000 = 0, 0x8 DCS +0x7a0 = 0 (restored at wake)");
static void __iomem *mcc_va;
static unsigned int slc_done;
static u32 slc_en[4], slc_st_on[4], slc_st_off[4], slc_irq[4][2], slc_155, slc_7a0[8];
static int slc_err;

static void mcc_cfg(u32 v)
{
	writel(v, mcc_va + MCC_LOCK0);
	writel(v, mcc_va + MCC_LOCK1);
}

static u32 mcc_poll_ways(int p, u32 want)
{
	u32 v = 0;

	for (int i = 0; i < 10000; i++) {
		v = readl(mcc_va + MCC_PLANE(p) + MCC_CACHE_ST);
		if ((v & MCC_CACHE_ST_WAYS) == want)
			return v;
		udelay(1);
	}
	slc_err++;
	return v;
}

static void slc_disable(void)
{
	int i;

	slc_done = 0;
	slc_err = 0;
	if (!slc_off)
		return;
	if (!mcc_va)
		mcc_va = ioremap_np(MCC_PHYS, MCC_SIZE);
	if (!mcc_va)
		return;
	slc_done = slc_off & 0xf;
	if (slc_done & 0x1) {
		/* plane 0 only, as macOS: planes 1-3 read 0 but a write there acts on the same cache */
		for (i = 0; i < 1; i++) {
			slc_en[i] = readl(mcc_va + MCC_PLANE(i) + MCC_CACHE_EN);
			slc_st_on[i] = readl(mcc_va + MCC_PLANE(i) + MCC_CACHE_ST);
			mcc_cfg(1);
			writel(0, mcc_va + MCC_PLANE(i) + MCC_CACHE_EN);
			mcc_cfg(0);
			slc_st_off[i] = mcc_poll_ways(i, 0);
		}
	}
	if (slc_done & 0x2) {
		for (i = 0; i < 4; i++) {
			slc_irq[i][1] = readl(mcc_va + MCC_PLANE(i) + 0x21008);
			slc_irq[i][0] = readl(mcc_va + MCC_PLANE(i) + 0x21000);
			writel(0, mcc_va + MCC_PLANE(i) + 0x21008);
			writel(0, mcc_va + MCC_PLANE(i) + 0x21000);
		}
	}
	if (slc_done & 0x4) {
		slc_155 = readl(mcc_va + 0x155000);
		writel(0, mcc_va + 0x155000);
	}
	if (slc_done & 0x8) {
		for (i = 0; i < 8; i++) {
			slc_7a0[i] = readl(mcc_va + MCC_DCS(i) + 0x7a0);
			writel(0, mcc_va + MCC_DCS(i) + 0x7a0);
		}
	}
}

static void slc_restore(void)
{
	int i;

	if (slc_done & 0x8)
		for (i = 0; i < 8; i++)
			writel(slc_7a0[i], mcc_va + MCC_DCS(i) + 0x7a0);
	if (slc_done & 0x4)
		writel(slc_155, mcc_va + 0x155000);
	if (slc_done & 0x2) {
		for (i = 0; i < 4; i++) {
			writel(slc_irq[i][0], mcc_va + MCC_PLANE(i) + 0x21000);
			writel(slc_irq[i][1], mcc_va + MCC_PLANE(i) + 0x21008);
		}
	}
	if (slc_done & 0x1) {
		for (i = 0; i < 1; i++) {
			mcc_cfg(1);
			writel(slc_en[i], mcc_va + MCC_PLANE(i) + MCC_CACHE_EN);
			mcc_cfg(0);
			mcc_poll_ways(i, slc_st_on[i] & MCC_CACHE_ST_WAYS);
		}
	}
}

static void slc_log(void)
{
	if (!slc_done)
		return;
	pr_info("slc_off %#x: cache en %x %x %x %x, status on %x %x %x %x, off %x %x %x %x, now %x; 0x21000 %x 0x155000 %x 0x7a0 %x; %d poll timeouts\n",
		slc_done, slc_en[0], slc_en[1], slc_en[2], slc_en[3],
		slc_st_on[0], slc_st_on[1], slc_st_on[2], slc_st_on[3],
		slc_st_off[0], slc_st_off[1], slc_st_off[2], slc_st_off[3],
		readl(mcc_va + MCC_CACHE_ST), slc_irq[0][0], slc_155, slc_7a0[0], slc_err);
	slc_done = 0;
}

/*
 * wake_gpio (review 2026-11-19): the ADT ap-wake-sources include the GPIO *interrupt groups*
 * 12 AOP.GPIO.IRQ4, 14 AOP.GPIO.IRQ6 and 47 NUB.GPIO.IRQ6 (aop/nub-gpio: supported-int-groups 4,5,6,
 * wake-no-interrupt-group 4). pinctrl-apple-gpio puts every IRQ pin in group 0, which is no wake source.
 * macOS (HV trace 16): keyboard IRQ pin nub 13 in group 6 from boot (0x66a0f), at sleep 226.48 ack
 * group 6 (+0x980 = BIT(13)) then 0x66a07 (IRQ level low, group 6); 228.28 ack group 4 on AOP
 * (+0x900/+0x904) and NUB (+0x900). aop-gpio wake-events lists pin 22 (lid) any-edge.
 * wake_gpio=mask (0x1 lid, 0x2 keyboard): the same at .enter (IRQs off, all drivers suspended), right
 * before the SoC plan; restored at .wake. wake_gpio_now=mask/0: move/restore awake.
 * 2026-11-19: awake, keyboard in group 6 works (IRQs via AIC 336); lid in group 6 + the first lid edge
 * = hard SoC hang (no IRQ counted, no log), twice. Wake source 14 (AOP.GPIO.IRQ6) is enabled under
 * iBoot/Linux masks, or the AOP owns pin 22 / AOP group 6 (macOS's AP never writes pin 22).
 */
#define WG_AOP_PA	0x24a820000ULL
#define WG_NUB_PA	0x23d1f0000ULL
#define WG_GRP		GENMASK(18, 16)
#define WG_MODE		GENMASK(3, 1)
#define WG_IRQ(g, pin)	(0x800 + 0x40 * (g) + 4 * ((pin) >> 5))

static const struct { bool nub; u8 pin, mode; const char *name; } wg_pins[] = {
	{ false, 22, 6, "lid (aop 22, any edge)" },
	{ true,  13, 3, "keyboard (nub 13, level low)" },
};
#define WG_N	ARRAY_SIZE(wg_pins)

static unsigned int wake_gpio;
module_param(wake_gpio, uint, 0644);
MODULE_PARM_DESC(wake_gpio, "deep: mask, 0x1 lid (aop 22; HANGS the SoC on a lid edge awake), 0x2 keyboard (nub 13): move the IRQ to GPIO group 6 (ADT wake sources 14/47) + ack group 4 in .enter, as macOS; restored at wake");

static void __iomem *wg_va[2];	/* 0 aop, 1 nub */
static u32 wg_saved[WG_N];
static unsigned int wg_moved;	/* mask of moved pins */

static int wg_map(void)
{
	if (!wg_va[0])
		wg_va[0] = ioremap_np(WG_AOP_PA, 0x1000);
	if (!wg_va[1])
		wg_va[1] = ioremap_np(WG_NUB_PA, 0x1000);
	return wg_va[0] && wg_va[1] ? 0 : -ENOMEM;
}

static void wg_log(const char *tag)
{
	if (!wg_va[0] || !wg_va[1])
		return;
	for (int i = 0; i < WG_N; i++) {
		u32 v = readl(wg_va[wg_pins[i].nub] + 4 * wg_pins[i].pin);

		pr_info("wake_gpio %s: %s %08x (mode %lu, group %lu)\n", tag, wg_pins[i].name, v,
			FIELD_GET(WG_MODE, v), FIELD_GET(WG_GRP, v));
	}
}

/* in .enter / awake knob; no sleeping, no allocation */
static void wg_move(unsigned int mask)
{
	mask &= GENMASK(WG_N - 1, 0);
	if (wg_moved || !mask || !wg_va[0] || !wg_va[1])
		return;
	for (int i = 0; i < WG_N; i++) {
		void __iomem *b = wg_va[wg_pins[i].nub];
		u8 pin = wg_pins[i].pin;
		u32 v = readl(b + 4 * pin);

		if (!(mask & BIT(i)))
			continue;

		wg_saved[i] = v;
		writel(BIT(pin % 32), b + WG_IRQ(6, pin));	/* ack group 6 first, as macOS */
		writel((v & ~(WG_GRP | WG_MODE)) | FIELD_PREP(WG_GRP, 6) |
		       FIELD_PREP(WG_MODE, wg_pins[i].mode), b + 4 * pin);
	}
	/* ack group 4 (wake-no-interrupt group), as macOS at 228.28 */
	writel(~0U, wg_va[0] + WG_IRQ(4, 0));
	writel(~0U, wg_va[0] + WG_IRQ(4, 32));
	writel(~0U, wg_va[1] + WG_IRQ(4, 0));
	wg_moved = mask;
}

static void wg_restore(void)
{
	if (!wg_moved)
		return;
	for (int i = 0; i < WG_N; i++) {
		void __iomem *b = wg_va[wg_pins[i].nub];
		u8 pin = wg_pins[i].pin;
		u32 now;

		if (!(wg_moved & BIT(i)))
			continue;
		now = readl(b + 4 * pin);
		/* back to the Linux group; keep a mode the drivers changed meanwhile, else the saved one */
		writel((now & ~WG_GRP) | (wg_saved[i] & WG_GRP), b + 4 * pin);
		/* a pending bit latched in group 6 would never be acked by the group-0 handler */
		writel(BIT(pin % 32), b + WG_IRQ(6, pin));
	}
	wg_moved = 0;
}

static int wake_gpio_now_set(const char *val, const struct kernel_param *kp)
{
	unsigned int mask;
	int ret = kstrtouint(val, 0, &mask);

	if (ret)
		return ret;
	ret = wg_map();
	if (ret)
		return ret;
	wg_log("before");
	wg_restore();
	wg_move(mask);
	wg_log("after ");
	return 0;
}

static int wake_gpio_now_get(char *buf, const struct kernel_param *kp)
{
	int len = 0;

	if (wg_map())
		return scnprintf(buf, PAGE_SIZE, "unmapped\n");
	for (int i = 0; i < WG_N; i++) {
		u32 v = readl(wg_va[wg_pins[i].nub] + 4 * wg_pins[i].pin);

		len += scnprintf(buf + len, PAGE_SIZE - len, "%s %08x mode %lu group %lu%s", wg_pins[i].name,
				 v, FIELD_GET(WG_MODE, v), FIELD_GET(WG_GRP, v), i + 1 < WG_N ? " | " : "\n");
	}
	return len;
}

static const struct kernel_param_ops wake_gpio_now_ops = {
	.set = wake_gpio_now_set,
	.get = wake_gpio_now_get,
};
module_param_cb(wake_gpio_now, &wake_gpio_now_ops, NULL, 0644);
MODULE_PARM_DESC(wake_gpio_now, "awake: mask as wake_gpio (0x1 lid: hangs on a lid edge, 0x2 keyboard) = move to group 6 now, 0 = put back; read = pin registers");

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
	as_wdt_arm();
	as_wd0_arm();
	wg_move(wake_gpio);
	slc_disable();
	soc_plan_run();
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
	/* before ->wake: the secondaries are brought back online in between */
	if (soc_hold)
		apple_cpuhp_park_unhold();
	slc_restore();
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
	as_wdt_rearm_for_resume();
	as_wd0_disarm();
	if (wg_moved) {
		wg_log("at wake");
		wg_restore();
	}
	slc_log();
	apple_cpuidle_system_suspend_log();
	if (soc_nsaved)
		pr_info("soc_sleep: back, restoring %d settings\n", soc_nsaved);
	soc_restore();
	soc_plan_free();
	disp_cycle_restore();
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
	if (wg_map())
		dev_warn(&pdev->dev, "wake_gpio: cannot map the GPIO blocks\n");
	else
		wg_log("at probe");
	as_wd0_va = ioremap_np(AS_WDT_PHYS, 0x10);

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
	register_pm_notifier(&as_pm_nb);
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
	if (as_wd0_va)
		iounmap(as_wd0_va);
	as_wd0_va = NULL;
	wg_restore();
	for (int i = 0; i < 2; i++) {
		if (wg_va[i])
			iounmap(wg_va[i]);
		wg_va[i] = NULL;
	}
}

static void apple_sleep_remove(struct platform_device *pdev)
{
	unregister_pm_notifier(&as_pm_nb);
	as_wdt_disarm();
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
