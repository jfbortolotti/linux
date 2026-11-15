// SPDX-License-Identifier: GPL-2.0-only OR MIT
/* Copyright 2023 */

#include <linux/completion.h>
#include <linux/delay.h>
#include <linux/slab.h>

#include "afk.h"
#include "dcp.h"

#include "dcp-internal.h"

static void disp_service_init(struct apple_epic_service *service, const char *name,
			const char *class, s64 unit)
{
	service->ep->dcp->iboot_service = service;
}


static const struct apple_epic_service_ops ibootep_ops[] = {
	{
		.name = "disp0-service",
		.init = disp_service_init,
	},
	{}
};

int ibootep_init(struct apple_dcp *dcp)
{
	dcp->ibootep = afk_init(dcp, DISP0_ENDPOINT, ibootep_ops);
	afk_start(dcp->ibootep);

	return 0;
}

/*
 * apple-s2idle full_suspend: after a cold DCP boot (pristine __DATA), IOMFB refuses to start
 * ("start_ios_gated(): Driver didn't load in iBoot", 2026-11-15) unless the panel was first
 * brought up through the iBoot interface, as iBoot (and m1n1 src/display.c) do: set_power,
 * get_hpd, timing/color modes, set_mode. Same command format as m1n1 src/dcp_iboot.c.
 */
enum {
	IBOOT_SET_POWER = 2,
	IBOOT_GET_HPD = 3,
	IBOOT_GET_TIMING_MODES = 4,
	IBOOT_GET_COLOR_MODES = 5,
	IBOOT_SET_MODE = 6,
};

struct ib_timing_mode {
	u32 valid, width, height, fps;
	u8 pad[8];
} __packed;

struct ib_color_mode {
	u32 valid, colorimetry, eotf, encoding, bpp;
	u8 pad[4];
} __packed;

#define IB_BUF_LEN 0x4000

struct ib_bufs {
	u8 tx[IB_BUF_LEN];
	u8 rx[IB_BUF_LEN];
};

static int ib_cmd(struct apple_dcp *dcp, struct ib_bufs *b, u32 op, const void *in, size_t in_len)
{
	u32 *hdr = (u32 *)b->tx;
	u32 retcode = 0;
	int ret;

	memset(b->tx, 0, 16);
	hdr[0] = op;
	hdr[1] = 16 + in_len;
	if (in_len)
		memcpy(b->tx + 16, in, in_len);
	memset(b->rx, 0, IB_BUF_LEN);
	ret = afk_send_command(dcp->iboot_service, 0xc0, b->tx, 16 + in_len, b->rx, IB_BUF_LEN,
			       &retcode);
	if (ret)
		dev_err(dcp->dev, "iboot: op %u failed: %d\n", op, ret);
	return ret;
}

int dcp_iboot_bringup(struct apple_dcp *dcp)
{
	struct ib_timing_mode tbest, *tm;
	struct ib_color_mode cbest, *cm;
	struct {
		struct ib_timing_mode t;
		struct ib_color_mode c;
	} __packed setmode;
	struct ib_bufs *b;
	u32 hpd = 0, tcnt = 0, ccnt = 0, n, i, pwr = 1;
	int ret, tries;

	b = kzalloc(sizeof(*b), GFP_KERNEL);
	if (!b)
		return -ENOMEM;

	dcp->iboot_service = NULL;
	if (!dcp->ibootep)
		ibootep_init(dcp);
	for (tries = 0; tries < 100 && !dcp->iboot_service; tries++)
		msleep(10);
	if (!dcp->iboot_service) {
		dev_err(dcp->dev, "iboot: no disp0-service\n");
		ret = -ETIMEDOUT;
		goto out;
	}

	ret = ib_cmd(dcp, b, IBOOT_SET_POWER, &pwr, 1);
	if (ret)
		goto out;
	for (tries = 0; tries < 20; tries++) {
		ret = ib_cmd(dcp, b, IBOOT_GET_HPD, NULL, 0);
		if (ret)
			goto out;
		/* rx: u32 op, u32 len, then {u8 hpd, pad[3], u32 timing_cnt, u32 color_cnt} */
		hpd = b->rx[8];
		tcnt = *(u32 *)(b->rx + 12);
		ccnt = *(u32 *)(b->rx + 16);
		if (hpd && tcnt && ccnt)
			break;
		msleep(100);
	}
	dev_info(dcp->dev, "iboot: hpd %u, %u timing / %u color modes after %d tries\n",
		 hpd, tcnt, ccnt, tries);
	if (!hpd || !tcnt || !ccnt) {
		ret = -ENODEV;
		goto out;
	}

	ret = ib_cmd(dcp, b, IBOOT_SET_POWER, &pwr, 1);
	if (ret)
		goto out;
	msleep(100);

	ret = ib_cmd(dcp, b, IBOOT_GET_TIMING_MODES, NULL, 0);
	if (ret)
		goto out;
	n = min_t(u32, *(u32 *)(b->rx + 8), (IB_BUF_LEN - 12) / sizeof(*tm));
	tm = (void *)(b->rx + 12);
	tbest = tm[0];
	for (i = 1; i < n; i++)	/* m1n1: valid first, then largest mode */
		if ((tm[i].valid && !tbest.valid) ||
		    (tm[i].valid == tbest.valid &&
		     (u64)tm[i].width * tm[i].height > (u64)tbest.width * tbest.height))
			tbest = tm[i];

	ret = ib_cmd(dcp, b, IBOOT_GET_COLOR_MODES, NULL, 0);
	if (ret)
		goto out;
	n = min_t(u32, *(u32 *)(b->rx + 8), (IB_BUF_LEN - 12) / sizeof(*cm));
	cm = (void *)(b->rx + 12);
	cbest = cm[0];
	for (i = 1; i < n; i++)	/* m1n1: valid, bpp <= 32, highest bpp, lowest colorimetry/encoding/eotf */
		if ((cm[i].valid && !cbest.valid) ||
		    (cm[i].valid == cbest.valid && cm[i].bpp <= 32 &&
		     (cbest.bpp > 32 || cm[i].bpp > cbest.bpp ||
		      (cm[i].bpp == cbest.bpp && cm[i].colorimetry + cm[i].encoding + cm[i].eotf <
		       cbest.colorimetry + cbest.encoding + cbest.eotf))))
			cbest = cm[i];

	dev_info(dcp->dev, "iboot: set_mode %ux%u fps %#x, bpp %u colorimetry %u eotf %u encoding %u\n",
		 tbest.width, tbest.height, tbest.fps, cbest.bpp, cbest.colorimetry, cbest.eotf,
		 cbest.encoding);
	setmode.t = tbest;
	setmode.c = cbest;
	ret = ib_cmd(dcp, b, IBOOT_SET_MODE, &setmode, sizeof(setmode));
	if (ret) {
		msleep(500);
		ret = ib_cmd(dcp, b, IBOOT_SET_MODE, &setmode, sizeof(setmode));
	}
out:
	if (dcp->ibootep) {
		afk_shutdown(dcp->ibootep);
		dcp->ibootep = NULL;
	}
	dcp->iboot_service = NULL;
	kfree(b);
	return ret;
}
