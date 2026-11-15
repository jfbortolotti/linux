// SPDX-License-Identifier: GPL-2.0-only OR MIT
/*
 * Apple M1 (t8103) Power Management Processor.
 *
 * iBoot leaves the PMP firmware (t8103pmp) pre-loaded with its CPU stopped, and the Rust pmp
 * driver only speaks the t6000/t8112 "v2" protocol, so under Linux on t8103 nothing runs the
 * PMP's DRAM/fabric power management. This boots it the way macOS does (hypervisor trace 10):
 *
 *   PMP  ep 0x20 type 0x000 Startup
 *   AP   type 0x010 Configure(dva of a 64 KiB buffer):
 *          +0x2000  the ADT energy-model-dram-configs entry for this DRAM (capacity, vendor)
 *          +0xe000  {dva, size} per ADT pmp reg[3..] PIO window, mapped from 0xc0000000:
 *                   one 16 MiB slot per 16 MiB physical granule, packed in order inside it
 *   PMP  type 0x020 Configure ack
 *   AP   type 0x200 Init1(1, 3) -> 0x201,  type 0x202 Init2(1, 0) -> 0x203
 *   AP   type 0x20e DevPwr(dev, state), 0x208 DevPwr sync -> 0x209; dev = PMGR ADT device id
 *        (0x54 DISP0_FE, 0x55 DISPEXT_FE, 0x5c JPG, 0x5d MSR, 0x63 GFX, 0x64 ISP_SYS,
 *         0x65 VENC_SYS, 0x66 AVD_SYS, 0x68 ANE_SYS, 0xe3 ATC1_PCIE)
 *
 * Message: type = bits 56:44, arg1 = bits 43:16, arg2 = bits 15:0.
 *
 * System sleep (param sleep=1), as macOS (traces 5, 7, 8, 10): DevPwr off for the display
 * front-ends, GFX, ISP, ANE, then RTKit SetAPPower/SetIOPPower 0x201 (idle, state kept). macOS
 * then stops the PMP CPU (CPU_CONTROL RUN=0) and never wakes it: its SoC sleep reboots the PMP.
 * Under Linux the SoC stays up, and (2026-11-14):
 *  - the idled PMP does not answer RTKit wake (IOP 0x220 -> -ETIME);
 *  - RUN=0 then RUN=1 does not reboot the firmware (no HELLO), it continues where it stopped.
 *  - the idled PMP does not answer IOP 0x20 + AP 0x20 either (AP ack -ETIME).
 * Default now: only SetAPPower 0x201 at suspend, 0x20 at resume (IOP left on): 36.8 mA overnight
 * s2idle vs 47.3 without, PMP back after 5.6 h. sleep_iop=1 (not needed for the saving) also
 * idles the IOP and, on resume, rings the PMP doorbell (0x23bc34000, which macOS writes 0x2 to on
 * every CPU idle exit) after IOP 0x20, in case the firmware waits on it. stop_cpu needs sleep_iop.
 */

#include <linux/bitfield.h>
#include <linux/completion.h>
#include <linux/debugfs.h>
#include <linux/delay.h>
#include <linux/dma-mapping.h>
#include <linux/io.h>
#include <linux/iommu.h>
#include <linux/iopoll.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/pm.h>
#include <linux/soc/apple/rtkit.h>
#include <linux/unaligned.h>

#define ASC_CPU_CONTROL		0x44
#define ASC_CPU_CONTROL_RUN	BIT(4)
#define ASC_CPU_STATUS		0x48
#define ASC_CPU_STATUS_IDLE	BIT(5)
#define PMP_DOORBELL_PHYS	0x23bc34000ULL

#define PMP_EP			0x20
#define RTK_EP_MGMT		0x00
#define RTK_MGMT_TYPE		GENMASK_ULL(59, 52)
#define RTK_MGMT_SET_IOP_PWR	0x06
#define RTK_PWR_ON		0x20
#define RTK_PWR_IDLE		0x201
#define PMP_TYPE		GENMASK_ULL(56, 44)
#define PMP_ARG1		GENMASK_ULL(43, 16)
#define PMP_ARG2		GENMASK_ULL(15, 0)
#define PMP_DVA			GENMASK_ULL(43, 0)

#define PMP_STARTUP		0x000
#define PMP_CONFIGURE		0x010
#define PMP_CONFIGURE_ACK	0x020
#define PMP_INIT1		0x200
#define PMP_INIT1_ACK		0x201
#define PMP_INIT2		0x202
#define PMP_INIT2_ACK		0x203
#define PMP_DEVPWR_SYNC		0x208
#define PMP_DEVPWR_SYNC_ACK	0x209
#define PMP_DEVPWR		0x20e

#define PMP_BUF_SIZE		0x10000
#define PMP_BUF_DRAMCFG		0x2000
#define PMP_BUF_PIOMAP		0xe000
#define PMP_DRAMCFG_SIZE	0x80
#define PMP_PIO_VM_BASE		0xc0000000ULL
#define PMP_PIO_GRANULE		SZ_16M
#define PMP_PIO_MAX		64
#define ARM_IO_BASE		0x200000000ULL

struct apple_pmp {
	struct device *dev;
	struct apple_rtkit *rtk;
	void __iomem *asc;
	void *buf;
	dma_addr_t buf_dva;
	struct iommu_domain *domain;
	struct {
		u64 dva, size;
	} pio[PMP_PIO_MAX];
	int npio;

	struct completion startup, cfg_ack, init1_ack, init2_ack, sync_ack;
	u64 cfg_ack_msg, sync_ack_msg;
	struct mutex lock; /* one sync request at a time */
	void __iomem *doorbell;
	bool ready, asleep, stopped, iop_idle;
	unsigned int wakes;
	struct debugfs_blob_wrapper buf_blob;
	struct dentry *dbg;
};

static struct apple_pmp *the_pmp;

static bool sleep_enable = true;
module_param_named(sleep, sleep_enable, bool, 0644);
MODULE_PARM_DESC(sleep, "system sleep as macOS: DevPwr off messages + RTKit AP idle (0x201), AP 0x20 on resume (default Y)");

static bool sleep_iop;
module_param(sleep_iop, bool, 0644);
MODULE_PARM_DESC(sleep_iop, "with sleep=1: also SetIOPPower 0x201 (macOS; no known Linux wake yet)");

static bool stop_cpu;
module_param(stop_cpu, bool, 0644);
MODULE_PARM_DESC(stop_cpu, "with sleep=1 sleep_iop=1: also stop the PMP CPU (RUN=0) while asleep, as macOS");

static bool verbose = true;
module_param(verbose, bool, 0644);
MODULE_PARM_DESC(verbose, "log every ep 0x20 message");

static u64 pmp_msg(u32 type, u32 arg1, u32 arg2)
{
	return FIELD_PREP(PMP_TYPE, type) | FIELD_PREP(PMP_ARG1, arg1) | FIELD_PREP(PMP_ARG2, arg2);
}

static int pmp_send(struct apple_pmp *pmp, u64 msg)
{
	if (verbose)
		dev_info(pmp->dev, "> %#018llx (type %#llx)\n", msg, FIELD_GET(PMP_TYPE, msg));
	return apple_rtkit_send_message(pmp->rtk, PMP_EP, msg, NULL, false);
}

static int pmp_wait(struct apple_pmp *pmp, struct completion *c, unsigned int ms,
		    const char *what)
{
	if (!wait_for_completion_timeout(c, msecs_to_jiffies(ms))) {
		dev_err(pmp->dev, "no %s within %u ms\n", what, ms);
		return -ETIMEDOUT;
	}
	return 0;
}

static void pmp_recv(void *cookie, u8 ep, u64 msg)
{
	struct apple_pmp *pmp = cookie;
	u32 type = FIELD_GET(PMP_TYPE, msg);

	if (ep != PMP_EP) {
		dev_info(pmp->dev, "ep %#x message %#018llx\n", ep, msg);
		return;
	}
	if (verbose)
		dev_info(pmp->dev, "< %#018llx (type %#x)\n", msg, type);

	switch (type) {
	case PMP_STARTUP:
		complete(&pmp->startup);
		break;
	case PMP_CONFIGURE_ACK:
		pmp->cfg_ack_msg = msg;
		complete(&pmp->cfg_ack);
		break;
	case PMP_INIT1_ACK:
		complete(&pmp->init1_ack);
		break;
	case PMP_INIT2_ACK:
		complete(&pmp->init2_ack);
		break;
	case PMP_DEVPWR_SYNC_ACK:
		pmp->sync_ack_msg = msg;
		complete(&pmp->sync_ack);
		break;
	default:
		if (!verbose)
			dev_info(pmp->dev, "unhandled message %#018llx (type %#x)\n", msg, type);
		break;
	}
}

static void pmp_crashed(void *cookie, const void *crashlog, size_t size)
{
	struct apple_pmp *pmp = cookie;

	pmp->ready = false;
	dev_err(pmp->dev, "PMP firmware crashed\n");
}

static const struct apple_rtkit_ops pmp_rtkit_ops = {
	.crashed = pmp_crashed,
	.recv_message = pmp_recv,
};

/* tell the PMP a device's power state; sync waits for its ack */
static int pmp_devpwr(struct apple_pmp *pmp, u32 dev, u32 state, bool sync)
{
	int ret;

	if (!pmp->ready)
		return -ENODEV;
	if (!sync)
		return pmp_send(pmp, pmp_msg(PMP_DEVPWR, dev, state));

	mutex_lock(&pmp->lock);
	reinit_completion(&pmp->sync_ack);
	ret = pmp_send(pmp, pmp_msg(PMP_DEVPWR_SYNC, dev, state));
	if (!ret)
		ret = pmp_wait(pmp, &pmp->sync_ack, 1000, "DevPwr sync ack");
	mutex_unlock(&pmp->lock);
	return ret;
}

/* ADT pio windows -> DART mappings + the {dva, size} table, laid out as macOS does */
static int pmp_map_pio(struct apple_pmp *pmp)
{
	struct device_node *np = pmp->dev->of_node;
	u64 slot_base[PMP_PIO_MAX], slot_fill[PMP_PIO_MAX], next = PMP_PIO_VM_BASE;
	u32 slot_key[PMP_PIO_MAX], ranges[2 * PMP_PIO_MAX];
	__le64 *table = pmp->buf + PMP_BUF_PIOMAP;
	int n, i, j, nslots = 0, ret;

	n = of_property_count_u32_elems(np, "apple,pio-ranges");
	if (n <= 0 || n % 2 || n > 2 * PMP_PIO_MAX)
		return dev_err_probe(pmp->dev, -EINVAL, "bad apple,pio-ranges\n");
	of_property_read_u32_array(np, "apple,pio-ranges", ranges, n);
	n /= 2;

	pmp->domain = iommu_get_domain_for_dev(pmp->dev);
	if (!pmp->domain)
		return dev_err_probe(pmp->dev, -ENODEV, "no IOMMU domain\n");

	for (i = 0; i < n; i++) {
		u64 phys = ARM_IO_BASE + ranges[2 * i], size = ranges[2 * i + 1], dva;
		u32 key = ranges[2 * i] >> 24;

		if (!size)
			continue;
		for (j = 0; j < nslots && slot_key[j] != key; j++)
			;
		if (j == nslots) {
			slot_key[j] = key;
			slot_base[j] = next;
			slot_fill[j] = 0;
			next += PMP_PIO_GRANULE;
			nslots++;
		}
		dva = slot_base[j] + slot_fill[j];
		slot_fill[j] += ALIGN(size, SZ_16K);

		ret = iommu_map(pmp->domain, dva, phys, size,
				IOMMU_READ | IOMMU_WRITE | IOMMU_MMIO, GFP_KERNEL);
		if (ret) {
			dev_err(pmp->dev, "map %#llx -> %#llx failed: %d\n", phys, dva, ret);
			return ret;
		}
		pmp->pio[pmp->npio].dva = dva;
		pmp->pio[pmp->npio].size = size;
		pmp->npio++;
		table[2 * i] = cpu_to_le64(dva);
		table[2 * i + 1] = cpu_to_le64(size);
	}
	dev_info(pmp->dev, "%d PIO windows mapped in %d slots\n", pmp->npio, nslots);
	return 0;
}

static void pmp_unmap_pio(struct apple_pmp *pmp)
{
	while (pmp->npio > 0) {
		pmp->npio--;
		iommu_unmap(pmp->domain, pmp->pio[pmp->npio].dva, pmp->pio[pmp->npio].size);
	}
}

/* the energy-model-dram-configs entry for this machine's capacity and vendor (0xff = any) */
static int pmp_dramcfg(struct apple_pmp *pmp)
{
	struct device_node *np = pmp->dev->of_node;
	const u8 *cfg, *pick = NULL;
	u32 cap, vendor;
	int len, i;

	cfg = of_get_property(np, "apple,energy-model-dram-configs", &len);
	if (!cfg || len % PMP_DRAMCFG_SIZE ||
	    of_property_read_u32(np, "apple,dram-capacity", &cap) ||
	    of_property_read_u32(np, "apple,dram-vendor-id", &vendor))
		return dev_err_probe(pmp->dev, -EINVAL, "DRAM energy model missing\n");

	for (i = 0; i < len; i += PMP_DRAMCFG_SIZE) {
		u32 c = get_unaligned_le32(cfg + i), v = get_unaligned_le32(cfg + i + 4);

		if (c != cap)
			continue;
		if (v == vendor) {
			pick = cfg + i;
			break;
		}
		if (v == 0xff && !pick)
			pick = cfg + i;
	}
	if (!pick)
		return dev_err_probe(pmp->dev, -EINVAL, "no DRAM config for %u GB vendor %u\n",
				     cap, vendor);
	memcpy(pmp->buf + PMP_BUF_DRAMCFG, pick, PMP_DRAMCFG_SIZE);
	dev_info(pmp->dev, "DRAM config entry %d (%u GB, vendor %u)\n",
		 (int)((pick - cfg) / PMP_DRAMCFG_SIZE), cap, get_unaligned_le32(pick + 4));
	return 0;
}

/* start the PMP CPU and run the macOS boot handshake */
static int pmp_start(struct apple_pmp *pmp)
{
	struct device *dev = pmp->dev;
	u32 ctrl;
	int ret;

	reinit_completion(&pmp->startup);
	reinit_completion(&pmp->cfg_ack);
	reinit_completion(&pmp->init1_ack);
	reinit_completion(&pmp->init2_ack);

	ctrl = readl(pmp->asc + ASC_CPU_CONTROL);
	dev_info(dev, "CPU_CONTROL %#x before boot\n", ctrl);
	if (ctrl & ASC_CPU_CONTROL_RUN)
		dev_warn(dev, "PMP CPU already running, trying to boot it anyway\n");
	else
		writel(ctrl | ASC_CPU_CONTROL_RUN, pmp->asc + ASC_CPU_CONTROL);

	ret = apple_rtkit_boot(pmp->rtk);
	if (ret) {
		dev_err(dev, "RTKit boot failed: %d\n", ret);
		return ret;
	}
	ret = apple_rtkit_start_ep(pmp->rtk, PMP_EP);
	if (ret) {
		dev_err(dev, "start ep 0x20 failed: %d\n", ret);
		return ret;
	}

	ret = pmp_wait(pmp, &pmp->startup, 5000, "Startup");
	if (ret)
		return ret;
	ret = pmp_send(pmp, FIELD_PREP(PMP_TYPE, PMP_CONFIGURE) |
			    FIELD_PREP(PMP_DVA, pmp->buf_dva));
	if (!ret)
		ret = pmp_wait(pmp, &pmp->cfg_ack, 15000, "Configure ack");
	if (ret)
		return ret;

	ret = pmp_send(pmp, pmp_msg(PMP_INIT1, 1, 3));
	if (!ret)
		ret = pmp_wait(pmp, &pmp->init1_ack, 1000, "Init1 ack");
	if (!ret)
		ret = pmp_send(pmp, pmp_msg(PMP_INIT2, 1, 0));
	if (!ret)
		ret = pmp_wait(pmp, &pmp->init2_ack, 1000, "Init2 ack");
	if (ret)
		return ret;

	pmp->ready = true;
	dev_info(dev, "PMP running (configure ack %#llx, buffer at %pad)\n",
		 pmp->cfg_ack_msg, &pmp->buf_dva);
	return 0;
}

static int pmp_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct apple_pmp *pmp;
	int ret;

	pmp = devm_kzalloc(dev, sizeof(*pmp), GFP_KERNEL);
	if (!pmp)
		return -ENOMEM;
	pmp->dev = dev;
	mutex_init(&pmp->lock);
	init_completion(&pmp->startup);
	init_completion(&pmp->cfg_ack);
	init_completion(&pmp->init1_ack);
	init_completion(&pmp->init2_ack);
	init_completion(&pmp->sync_ack);
	platform_set_drvdata(pdev, pmp);

	pmp->asc = devm_platform_ioremap_resource_byname(pdev, "asc");
	if (IS_ERR(pmp->asc))
		return PTR_ERR(pmp->asc);
	/* experiment: not in the DT; Apple MMIO must be non-posted */
	pmp->doorbell = ioremap_np(PMP_DOORBELL_PHYS, 4);

	ret = dma_set_mask_and_coherent(dev, DMA_BIT_MASK(32));
	if (ret)
		return ret;
	/* not devm: once the PMP has the buffer it must outlive the driver */
	pmp->buf = dma_alloc_coherent(dev, PMP_BUF_SIZE, &pmp->buf_dva, GFP_KERNEL);
	if (!pmp->buf)
		return -ENOMEM;
	ret = pmp_dramcfg(pmp);
	if (!ret)
		ret = pmp_map_pio(pmp);
	if (ret) {
		/* the PMP has not seen any of it yet */
		pmp_unmap_pio(pmp);
		dma_free_coherent(dev, PMP_BUF_SIZE, pmp->buf, pmp->buf_dva);
		return ret;
	}

	pmp->rtk = devm_apple_rtkit_init(dev, pmp, NULL, 0, &pmp_rtkit_ops);
	if (IS_ERR(pmp->rtk)) {
		ret = dev_err_probe(dev, PTR_ERR(pmp->rtk), "RTKit init failed\n");
		goto err_unmap;
	}

	ret = pmp_start(pmp);
	if (ret)
		goto err_unmap;
	the_pmp = pmp;
	/* read only: the shared buffer as the PMP left it, to compare with the macOS dumps */
	pmp->buf_blob.data = pmp->buf;
	pmp->buf_blob.size = PMP_BUF_SIZE;
	pmp->dbg = debugfs_create_dir("pmp_t8103", NULL);
	debugfs_create_blob("buf", 0400, pmp->dbg, &pmp->buf_blob);
	return 0;

err_unmap:
	/* the PMP may still DMA into the buffer: leave it and the mappings in place */
	dev_err(dev, "PMP not started (%d); PIO mappings and buffer leaked on purpose\n", ret);
	return ret;
}

static void pmp_remove(struct platform_device *pdev)
{
	struct apple_pmp *pmp = platform_get_drvdata(pdev);

	the_pmp = NULL;
	debugfs_remove_recursive(pmp->dbg);
	pmp->ready = false;
	if (apple_rtkit_is_running(pmp->rtk))
		apple_rtkit_idle(pmp->rtk);
	if (pmp->doorbell)
		iounmap(pmp->doorbell);
	/* the firmware keeps the buffer and PIO windows: keep them mapped */
}

/* what macOS tells the PMP right before sleep (trace 10, 183.37-183.82) */
static const struct {
	u16 dev;
	bool sync;
} pmp_sleep_devs[] = {
	{ 0x54, false },	/* DISP0_FE */
	{ 0x55, false },	/* DISPEXT_FE */
	{ 0x63, true },		/* GFX */
	{ 0x64, false },	/* ISP_SYS */
	{ 0x68, true },		/* ANE_SYS */
};

static int pmp_suspend_late(struct device *dev)
{
	struct apple_pmp *pmp = dev_get_drvdata(dev);
	u32 status;
	int i, ret;

	if (!sleep_enable || !pmp->ready)
		return 0;
	for (i = 0; i < ARRAY_SIZE(pmp_sleep_devs); i++)
		pmp_devpwr(pmp, pmp_sleep_devs[i].dev, 0, pmp_sleep_devs[i].sync);

	pmp->iop_idle = sleep_iop;
	ret = sleep_iop ? apple_rtkit_idle(pmp->rtk) :
			  apple_rtkit_set_ap_power(pmp->rtk, RTK_PWR_IDLE);
	if (ret) {
		dev_err(dev, "RTKit %s idle (0x201) failed: %d, PMP left running\n",
			sleep_iop ? "AP+IOP" : "AP", ret);
		return 0;
	}
	pmp->ready = false;
	pmp->asleep = true;
	if (!sleep_iop || !stop_cpu) {
		dev_info(dev, "PMP %s idle (0x201)\n", sleep_iop ? "AP+IOP" : "AP");
		return 0;
	}

	/* macOS: the CPU is stopped right after the IOP 0x201 ack */
	ret = readl_poll_timeout(pmp->asc + ASC_CPU_STATUS, status,
				 status & ASC_CPU_STATUS_IDLE, 100, 100000);
	writel(readl(pmp->asc + ASC_CPU_CONTROL) & ~ASC_CPU_CONTROL_RUN,
	       pmp->asc + ASC_CPU_CONTROL);
	pmp->stopped = true;
	dev_info(dev, "PMP idle (0x201), CPU stopped (CPU_STATUS %#x%s)\n", status,
		 ret ? ", not idle after 100 ms" : "");
	return 0;
}

static int pmp_resume_early(struct device *dev)
{
	struct apple_pmp *pmp = dev_get_drvdata(dev);
	int ret, i;

	if (!pmp->asleep)
		return 0;
	pmp->asleep = false;
	if (pmp->stopped) {
		pmp->stopped = false;
		writel(readl(pmp->asc + ASC_CPU_CONTROL) | ASC_CPU_CONTROL_RUN,
		       pmp->asc + ASC_CPU_CONTROL);
	}

	/* IOP first (its ack updates rtkit's state), then AP; both must be ON to send again */
	ret = 0;
	if (pmp->iop_idle) {
		ret = apple_rtkit_send_message(pmp->rtk, RTK_EP_MGMT,
					       FIELD_PREP(RTK_MGMT_TYPE, RTK_MGMT_SET_IOP_PWR) |
					       RTK_PWR_ON, NULL, false);
		if (!ret && pmp->doorbell)
			writel(0x2, pmp->doorbell);
	}
	if (!ret)
		ret = apple_rtkit_set_ap_power(pmp->rtk, RTK_PWR_ON);
	for (i = 0; !ret && i < 100 && !apple_rtkit_is_running(pmp->rtk); i++)
		usleep_range(1000, 2000);
	if (!ret && !apple_rtkit_is_running(pmp->rtk))
		ret = -ETIMEDOUT;
	if (ret) {
		dev_err(dev, "PMP wake (%s 0x20) failed: %d (PMP off until reboot)\n",
			pmp->iop_idle ? "IOP+doorbell, AP" : "AP", ret);
		return 0;
	}
	pmp->ready = true;
	pmp->wakes++;
	/*
	 * The display front end is on again before any driver resumes (and a DCP restarted by
	 * appledrm full_suspend asks the PMP for display bandwidth right away): tell the PMP, it
	 * was told DISP0_FE off at suspend (2026-11-15: PMP crash at resume without this).
	 */
	ret = pmp_devpwr(pmp, 0x54, 1, true);
	if (ret)
		dev_warn(dev, "DevPwr DISP0_FE on at resume: %d\n", ret);
	ret = pmp_devpwr(pmp, 0x68, 0, true);
	dev_info(dev, "PMP awake (wake %u), ANE sync check %s\n", pmp->wakes, ret ? "FAILED" : "acked");
	return 0;
}

static const struct dev_pm_ops pmp_pm_ops = {
	LATE_SYSTEM_SLEEP_PM_OPS(pmp_suspend_late, pmp_resume_early)
};

/* echo "<dev> <state> [sync]" > /sys/module/pmp_t8103/parameters/devpwr  (experiments) */
static int devpwr_set(const char *val, const struct kernel_param *kp)
{
	u32 dev, state, sync = 0;

	if (sscanf(val, "%i %i %i", &dev, &state, &sync) < 2)
		return -EINVAL;
	if (!the_pmp)
		return -ENODEV;
	return pmp_devpwr(the_pmp, dev, state, sync);
}

static const struct kernel_param_ops devpwr_ops = {
	.set = devpwr_set,
};
module_param_cb(devpwr, &devpwr_ops, NULL, 0200);
MODULE_PARM_DESC(devpwr, "send DevPwr: \"<dev> <state> [sync]\"");

/*
 * echo "<type> <arg1> <arg2>" > /sys/module/pmp_t8103/parameters/raw  (2026-11-14, DCS step): send
 * one ep 0x20 message (type bits 56:44, arg1 43:16, arg2 15:0) to replay macOS-only messages,
 * e.g. 0x20a "0x54 0x22" (DISP0_FE); the reply is logged as an unhandled message.
 */
static int raw_set(const char *val, const struct kernel_param *kp)
{
	u32 type, a1, a2;
	int ret;

	if (sscanf(val, "%i %i %i", &type, &a1, &a2) != 3 || type > 0x1fff || a1 > 0xfffffff ||
	    a2 > 0xffff)
		return -EINVAL;
	if (!the_pmp || !the_pmp->ready)
		return -ENODEV;
	ret = pmp_send(the_pmp, pmp_msg(type, a1, a2));
	dev_info(the_pmp->dev, "raw > %#018llx: %d\n", pmp_msg(type, a1, a2), ret);
	return ret;
}

static const struct kernel_param_ops raw_ops = {
	.set = raw_set,
};
module_param_cb(raw, &raw_ops, NULL, 0200);
MODULE_PARM_DESC(raw, "send one ep 0x20 message: \"<type> <arg1> <arg2>\" (experiments)");

static const struct of_device_id pmp_of_match[] = {
	{ .compatible = "apple,t8103-pmp" },
	{}
};
MODULE_DEVICE_TABLE(of, pmp_of_match);

static struct platform_driver pmp_driver = {
	.driver = {
		.name = "apple-pmp-t8103",
		.of_match_table = pmp_of_match,
		.pm = pm_sleep_ptr(&pmp_pm_ops),
		.probe_type = PROBE_PREFER_ASYNCHRONOUS,
		.suppress_bind_attrs = true,
	},
	.probe = pmp_probe,
	.remove = pmp_remove,
};
module_platform_driver(pmp_driver);

MODULE_DESCRIPTION("Apple t8103 Power Management Processor");
MODULE_LICENSE("Dual MIT/GPL");
