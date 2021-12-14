/*
 * SPDX-License-Identifier: GPL-2.0-or-later
 *
 * Apple Force Touch trackpad driver
 *
 * Copyright (C) The Asahi Linux Contributors
 *
 * Based on hid-magicmouse.c:
 *
 *   Copyright (c) 2010 Michael Poole <mdpoole@troilus.org>
 *   Copyright (c) 2010 Chase Douglas <chase.douglas@canonical.com>
 *
 *   Apple "Magic" Wireless Mouse driver
 *
 *   Copyright (c) 2010 Michael Poole <mdpoole@troilus.org>
 *   Copyright (c) 2010 Chase Douglas <chase.douglas@canonical.com>
 *
 *
 * Based on applespi.c:
 *
 * MacBook (Pro) SPI keyboard and touchpad driver
 *
 * Copyright (c) 2015-2018 Federico Lorenzi
 * Copyright (c) 2017-2018 Ronald Tschalär
 *
 */

//#define DEBUG 1

#include <linux/device.h>
#include <linux/hid.h>
#include <linux/input/mt.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/workqueue.h>

#include "hid-ids.h"

#define MAX_CONTACTS 16

#define TRACKPAD_SPI_REPORT_ID 0x02
#define MAX_FINGER_ORIENTATION 16384

/**
 * struct magicmouse_sc - Tracks Magic Mouse-specific data.
 * @input: Input device through which we report events.
 * @quirks: Currently unused.
 * @ntouches: Number of touches in most recent touch report.
 * @scroll_accel: Number of consecutive scroll motions.
 * @scroll_jiffies: Time of last scroll motion.
 * @touches: Most recent data for a touch, indexed by tracking ID.
 * @tracking_ids: Mapping of current touch input data to @touches.
 */
struct appleft_sc {
	struct input_dev *input;
	unsigned long quirks;

	int ntouches;

	struct input_mt_pos pos[MAX_CONTACTS];
	int slots[MAX_CONTACTS];
	u8 map_contacs[MAX_CONTACTS];

	struct hid_device *hdev;
	struct delayed_work work;

	int x_min, y_min, x_max, y_max;
};

/**
 * struct tp_finger - single trackpad finger structure, le16-aligned
 *
 * @origin:		zero when switching track finger
 * @abs_x:		absolute x coordinate
 * @abs_y:		absolute y coordinate
 * @rel_x:		relative x coordinate
 * @rel_y:		relative y coordinate
 * @tool_major:		tool area, major axis
 * @tool_minor:		tool area, minor axis
 * @orientation:	16384 when point, else 15 bit angle
 * @touch_major:	touch area, major axis
 * @touch_minor:	touch area, minor axis
 * @unused:		zeros
 * @pressure:		pressure on forcetouch touchpad
 * @multi:		one finger: varies, more fingers: constant
 * @crc16:		on last finger: crc over the whole message struct
 *			(i.e. message header + this struct) minus the last
 *			@crc16 field; unknown on all other fingers.
 */
struct tp_finger {
	__le16 unknown_or_origin1;
	__le16 unknown_or_origin2;
	__le16 abs_x;
	__le16 abs_y;
	__le16 rel_x;
	__le16 rel_y;
	__le16 tool_major;
	__le16 tool_minor;
	__le16 orientation;
	__le16 touch_major;
	__le16 touch_minor;
	__le16 unused[2];
	__le16 pressure;
	__le16 multi;
} __attribute__((packed, aligned(2)));

/**
 * struct trackpad report
 *
 * @report_id:		reportid
 * @buttons:		HID Usage Buttons 3 1-bit reports
 * @num_fingers:	the number of fingers being reported in @fingers
 * @clicked:		same as @buttons
 */
struct tp_header {
	// HID mouse report
	u8 report_id;
	u8 buttons;
	u8 rel_x;
	u8 rel_y;
	u8 padding[4];
	// HID vendor part, up to 1751 bytes
	u8 unknown[22];
	u8 num_fingers;
	u8 clicked;
	u8 unknown3[14];
};

static inline int le16_to_int(__le16 x)
{
	return (signed short)le16_to_cpu(x);
}

static void report_finger_data(struct input_dev *input, int slot,
			       const struct input_mt_pos *pos,
			       const struct tp_finger *f)
{
	input_mt_slot(input, slot);
	input_mt_report_slot_state(input, MT_TOOL_FINGER, true);

	input_report_abs(input, ABS_MT_TOUCH_MAJOR,
			 le16_to_int(f->touch_major) << 1);
	input_report_abs(input, ABS_MT_TOUCH_MINOR,
			 le16_to_int(f->touch_minor) << 1);
	input_report_abs(input, ABS_MT_WIDTH_MAJOR,
			 le16_to_int(f->tool_major) << 1);
	input_report_abs(input, ABS_MT_WIDTH_MINOR,
			 le16_to_int(f->tool_minor) << 1);
	input_report_abs(input, ABS_MT_ORIENTATION,
			 MAX_FINGER_ORIENTATION - le16_to_int(f->orientation));
	input_report_abs(input, ABS_MT_PRESSURE, le16_to_int(f->pressure));
	input_report_abs(input, ABS_MT_POSITION_X, pos->x);
	input_report_abs(input, ABS_MT_POSITION_Y, pos->y);
}

static int appleft_raw_event(struct hid_device *hdev, struct hid_report *report,
			     u8 *data, int size)
{
	struct appleft_sc *asc = hid_get_drvdata(hdev);
	struct input_dev *input = asc->input;
	struct tp_header *tp_hdr;
	struct tp_finger *f;
	int i, n;
	u32 npoints;
	const size_t hdr_sz = sizeof(struct tp_header);
	const size_t touch_sz = sizeof(struct tp_finger);

	// hid_warn(hdev, "%s\n", __func__);
	// print_hex_dump_debug("appleft ev: ", DUMP_PREFIX_OFFSET, 16, 1, data,
	// 		     size, false);

	if (data[0] != TRACKPAD_SPI_REPORT_ID)
		return 0;

	/* Expect 46 bytes of prefix, and N * 30 bytes of touch data. */
	if (size < hdr_sz || ((size - hdr_sz) % touch_sz) != 0)
		return 0;

	tp_hdr = (struct tp_header *)data;

	npoints = (size - hdr_sz) / touch_sz;
	if (npoints < tp_hdr->num_fingers || npoints > MAX_CONTACTS) {
		hid_warn(hdev,
			 "unexpected number of touches (%u) for "
			 "report\n",
			 npoints);
		return 0;
	}

	n = 0;
	for (i = 0; i < tp_hdr->num_fingers; i++) {
		f = (struct tp_finger *)(data + hdr_sz + i * touch_sz);
		if (le16_to_int(f->touch_major) == 0)
			continue;

		hid_dbg(hdev, "ev x:%04hx y:%04hx\n", le16_to_int(f->abs_x),
			le16_to_int(f->abs_y));
		asc->pos[n].x = le16_to_int(f->abs_x);
		asc->pos[n].y = asc->y_min + asc->y_max - le16_to_int(f->abs_y);
		asc->map_contacs[n] = i;
		n++;
	}

	input_mt_assign_slots(input, asc->slots, asc->pos, n, 0);

	for (i = 0; i < n; i++) {
		int idx = asc->map_contacs[i];
		f = (struct tp_finger *)(data + hdr_sz + idx * touch_sz);
		report_finger_data(input, asc->slots[i], &asc->pos[i], f);
	}

	input_mt_sync_frame(input);
	input_report_key(input, BTN_MOUSE, data[1] & 1);

	input_sync(input);
	return 1;
}

static int appleft_setup_input(struct input_dev *input, struct hid_device *hdev)
{
	int error;
	int mt_flags = 0;
	struct appleft_sc *asc = hid_get_drvdata(hdev);

	__set_bit(INPUT_PROP_BUTTONPAD, input->propbit);
	__clear_bit(BTN_0, input->keybit);
	__clear_bit(BTN_RIGHT, input->keybit);
	__clear_bit(BTN_MIDDLE, input->keybit);
	__clear_bit(EV_REL, input->evbit);
	__clear_bit(REL_X, input->relbit);
	__clear_bit(REL_Y, input->relbit);

	mt_flags = INPUT_MT_POINTER | INPUT_MT_DROP_UNUSED | INPUT_MT_TRACK;

	/* finger touch area */
	input_set_abs_params(input, ABS_MT_TOUCH_MAJOR, 0, 5000, 0, 0);
	input_set_abs_params(input, ABS_MT_TOUCH_MINOR, 0, 5000, 0, 0);

	/* finger approach area */
	input_set_abs_params(input, ABS_MT_WIDTH_MAJOR, 0, 5000, 0, 0);
	input_set_abs_params(input, ABS_MT_WIDTH_MINOR, 0, 5000, 0, 0);

	/* Note: Touch Y position from the device is inverted relative
	 * to how pointer motion is reported (and relative to how USB
	 * HID recommends the coordinates work).  This driver keeps
	 * the origin at the same position, and just uses the additive
	 * inverse of the reported Y.
	 */

	input_set_abs_params(input, ABS_MT_PRESSURE, 0, 6000, 0, 0);

	/*
	 * This makes libinput recognize this as a PressurePad and
	 * stop trying to use pressure for touch size. Pressure unit
	 * seems to be ~grams on these touchpads.
	 */
	input_abs_set_res(input, ABS_MT_PRESSURE, 1);

	/* finger orientation */
	input_set_abs_params(input, ABS_MT_ORIENTATION, -MAX_FINGER_ORIENTATION,
			     MAX_FINGER_ORIENTATION, 0, 0);

	/* finger position */
	input_set_abs_params(input, ABS_MT_POSITION_X, asc->x_min, asc->x_max,
			     0, 0);
	input_set_abs_params(input, ABS_MT_POSITION_Y, asc->y_min, asc->y_max,
			     0, 0);

	/* touchpad button */
	input_set_capability(input, EV_KEY, BTN_MOUSE);

	/*
	 * hid-input may mark device as using autorepeat, but the trackpad does
	 * not actually want it.
	 */
	__clear_bit(EV_REP, input->evbit);

	error = input_mt_init_slots(input, 16, mt_flags);
	if (error)
		return error;

	return 0;
}

static int appleft_input_mapping(struct hid_device *hdev, struct hid_input *hi,
				 struct hid_field *field,
				 struct hid_usage *usage, unsigned long **bit,
				 int *max)
{
	struct appleft_sc *msc = hid_get_drvdata(hdev);

	if (!msc->input)
		msc->input = hi->input;

	return 0;
}

static int appleft_input_configured(struct hid_device *hdev,
				    struct hid_input *hi)

{
	struct appleft_sc *msc = hid_get_drvdata(hdev);
	int ret;
	hid_err(hdev, "%s msc:%p\n", __func__, msc);
	if (!msc)
		return -1;
	hid_err(hdev, "%s msc:%p input:%p\n", __func__, msc, msc->input);
	if (!msc->input)
		return -1;

	ret = appleft_setup_input(msc->input, hdev);
	if (ret) {
		hid_err(hdev, "appleft setup input failed (%d)\n", ret);
		/* clean msc->input to notify probe() of the failure */
		msc->input = NULL;
		return ret;
	}

	return 0;
}

static int appleft_enable_multitouch(struct hid_device *hdev)
{
	const u8 feature_mt_trackpad[] = { 0x02, 0x01 };
	u8 *buf;
	int ret;
	hid_err(hdev, "%s\n", __func__);

	buf = kmemdup(feature_mt_trackpad, sizeof(feature_mt_trackpad),
		      GFP_KERNEL);
	if (!buf)
		return -ENOMEM;

	ret = hid_hw_raw_request(hdev, buf[0], buf, sizeof(feature_mt_trackpad),
				 HID_FEATURE_REPORT, HID_REQ_SET_REPORT);
	kfree(buf);
	hid_err(hdev, "hid_hw_raw_request returned: %d\n", ret);
	return ret;
}

static void appleft_enable_mt_work(struct work_struct *work)
{
	struct appleft_sc *msc =
		container_of(work, struct appleft_sc, work.work);
	int ret;

	ret = appleft_enable_multitouch(msc->hdev);
	if (ret < 0)
		hid_err(msc->hdev, "unable to request touch data (%d)\n", ret);
}

static int appleft_probe(struct hid_device *hdev,
			 const struct hid_device_id *id)
{
	struct appleft_sc *msc;
	struct hid_report *report;
	int ret;

	msc = devm_kzalloc(&hdev->dev, sizeof(*msc), GFP_KERNEL);
	if (msc == NULL) {
		hid_err(hdev, "can't alloc appleft descriptor\n");
		return -ENOMEM;
	}

	msc->x_min = -5896;
	msc->x_max = 6416;
	msc->y_min = -163;
	msc->y_max = 7363;

	msc->hdev = hdev;
	INIT_DEFERRABLE_WORK(&msc->work, appleft_enable_mt_work);

	msc->quirks = id->driver_data;
	hid_set_drvdata(hdev, msc);

	ret = hid_parse(hdev);
	if (ret) {
		hid_err(hdev, "appleft hid parse failed\n");
		return ret;
	}

	ret = hid_hw_start(hdev, HID_CONNECT_DEFAULT);
	if (ret) {
		hid_err(hdev, "appleft hw start failed\n");
		return ret;
	}

	if (!msc->input) {
		hid_err(hdev, "appleft input not registered\n");
		ret = -ENOMEM;
		goto err_stop_hw;
	}

	report = hid_register_report(hdev, HID_INPUT_REPORT,
				     TRACKPAD_SPI_REPORT_ID, 0);

	if (!report) {
		hid_err(hdev, "unable to register touch report\n");
		ret = -ENOMEM;
		goto err_stop_hw;
	}
	report->size = 6;

	/*
	 * Some devices repond with 'invalid report id' when feature
	 * report switching it into multitouch mode is sent to it.
	 *
	 * This results in -EIO from the _raw low-level transport callback,
	 * but there seems to be no other way of switching the mode.
	 * Thus the super-ugly hacky success check below.
	 */
	ret = appleft_enable_multitouch(hdev);
	if (ret != -EIO && ret < 0) {
		hid_err(hdev, "unable to request touch data (%d)\n", ret);
		goto err_stop_hw;
	}

	return 0;
err_stop_hw:
	hid_hw_stop(hdev);
	return ret;
}

static const struct hid_device_id apple_ft[] = {
	{ HID_SPI_DEVICE(SPI_VENDOR_ID_APPLE, SPI_DEVICE_ID_APPLE_FT_TRACKPAD),
	  .driver_data = 0 },
	{}
};
MODULE_DEVICE_TABLE(hid, apple_ft);

static struct hid_driver appleft_driver = {
	.name = "appleft",
	.id_table = apple_ft,
	.probe = appleft_probe,
	.raw_event = appleft_raw_event,
	.input_configured = appleft_input_configured,
	.input_mapping = appleft_input_mapping,
};
module_hid_driver(appleft_driver);

MODULE_LICENSE("GPL");
