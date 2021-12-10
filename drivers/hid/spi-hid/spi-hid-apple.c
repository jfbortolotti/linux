/*
 * SPDX-License-Identifier: GPL-2.0
 *
 * Apple SPI HID transport driver
 *
 * Copyright (C) The Asahi Linux Contributors
 *
 * Based on: drivers/input/applespi.c
 *
 * MacBook (Pro) SPI keyboard and touchpad driver
 *
 * Copyright (c) 2015-2018 Federico Lorenzi
 * Copyright (c) 2017-2018 Ronald Tschalär
 *
 */

//#define DEBUG 2

#include <asm/unaligned.h>
#include <linux/crc16.h>
#include <linux/delay.h>
#include <linux/device/driver.h>
#include <linux/hid.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_irq.h>
#include <linux/printk.h>
#include <linux/platform_device.h>
#include <linux/spi/spi.h>

#include "../hid-ids.h"

#define SPIHID_MAX_INPUT_REPORT_SIZE 0x800

#define SPI_HID_DEVICE_ID_KBD 0x1
#define SPI_HID_DEVICE_ID_TP 0x2
#define SPI_HID_DEVICE_ID_INFO 0xd0

#define SPIHID_READ_PACKET 0x20
#define SPIHID_WRITE_PACKET 0x40

#define SPIHID_REQUEST_DESC 0x1020
#define SPIHID_DESC_MAX 512

#define SPIHID_KBD_REPORT 0x0110
#define SPIHID_TP_REPORT 0x0210

#define SPIHID_SET_LEDS 0x0151 /* caps lock */

#define SPIHID_SET_TP_MODE 0x0252
#define SPIHID_TP_MODE_HID 0x00
#define SPIHID_TP_MODE_RAW 0x01

#define SPI_RW_CHG_DELAY_US 200 /* 'Inter Stage Us'? */

static const u8 spi_hid_apple_status_ok[4] = { 0xac, 0x27, 0x68, 0xd5 };

struct spihid_input_dev {
	struct hid_device *hid;
	u8 *hid_desc;
	u32 hid_desc_len;
	u32 id;
	bool ready;
};

struct spihid_input_report {
	u8 *buf;
	u32 length;
	u32 offset;
	u8 device;
	u8 flags;
};

struct spihid_apple {
	struct spi_device *spidev;

	struct spihid_input_dev kbd;
	struct spihid_input_dev tp;

	struct gpio_desc *enable_gpio;
	int irq;

	struct spi_message rx_msg;
	struct spi_message tx_msg;
	struct spi_transfer rx_transfer;
	struct spi_transfer tx_transfer;
	struct spi_transfer status_transfer;

	u8 *rx_buf;
	u8 *tx_buf;
	u8 *status_buf;

	u8 msg_id;

	/* fragmented HID report */
	struct spihid_input_report report;
};

/**
 * struct spihid_msg_hdr - common header of protocol messages.
 *
 * Each message begins with fixed header, followed by a message-type specific
 * payload, and ends with a 16-bit crc. Because of the varying lengths of the
 * payload, the crc is defined at the end of each payload struct, rather than
 * in this struct.
 *
 * @type:	the message type
 * @device:	device id
 * @msgid:	incremented on each message, rolls over after 255; there is a
 *		separate counter for each message type.
 * @rsplen:	response length (the exact nature of this field is quite
 *		speculative). On a request/write this is often the same as
 *		@length, though in some cases it has been seen to be much larger
 *		(e.g. 0x400); on a response/read this the same as on the
 *		request; for reads that are not responses it is 0.
 * @length:	length of the remainder of the data in the whole message
 *		structure (after re-assembly in case of being split over
 *		multiple spi-packets), minus the trailing crc. The total size
 *		of a message is therefore @length + 10.
 */

struct spihid_msg_hdr {
	__le16 type;
	u8 device;
	u8 id;
	__le16 rsplen;
	__le16 length;
};

struct spihid_msg_req_desc {
	struct spihid_msg_hdr hdr;
	__le16 crc16;
};

struct spihid_msg_tp_set_mode {
	struct spihid_msg_hdr hdr;
	u8 device;
	u8 mode;
	__le16 crc16;
};

/**
 * struct spihid_transfer_packet - a complete spi packet; always 256 bytes. This carries
 * the (parts of the) message in the data. But note that this does not
 * necessarily contain a complete message, as in some cases (e.g. many
 * fingers pressed) the message is split over multiple packets (see the
 * @offset, @remain, and @length fields). In general the data parts in
 * spihid_transfer_packet's are concatenated until @remaining is 0, and the
 * result is an message.
 *
 * @flags:	0x40 = write (to device), 0x20 = read (from device); note that
 *		the response to a write still has 0x40.
 * @device:	1 = keyboard, 2 = touchpad
 * @offset:	specifies the offset of this packet's data in the complete
 *		message; i.e. > 0 indicates this is a continuation packet (in
 *		the second packet for a message split over multiple packets
 *		this would then be the same as the @length in the first packet)
 * @remain:	number of message bytes remaining in subsequents packets (in
 *		the first packet of a message split over two packets this would
 *		then be the same as the @length in the second packet)
 * @length:	length of the valid data in the @data in this packet
 * @data:	all or part of a message
 * @crc16:	crc over this whole structure minus this @crc16 field. This
 *		covers just this packet, even on multi-packet messages (in
 *		contrast to the crc in the message).
 */
struct spihid_transfer_packet {
	u8 flags;
	u8 device;
	__le16 offset;
	__le16 remain;
	__le16 length;
	u8 data[246];
	__le16 crc16;
};

struct spihid_apple *spihid_get_data(struct spihid_input_dev *idev)
{
	switch (idev->id) {
	case SPI_HID_DEVICE_ID_KBD:
		return container_of(idev, struct spihid_apple, kbd);
	case SPI_HID_DEVICE_ID_TP:
		return container_of(idev, struct spihid_apple, tp);
	default:
		return NULL;
	}
}

int apple_ll_start(struct hid_device *hdev)
{
	/* no-op SPI transport is already setup */
	return 0;
};

void apple_ll_stop(struct hid_device *hdev)
{
	/* no-op, devices will be desstroyed on driver destruction */
	struct spihid_input_dev *idev = hdev->driver_data;
	printk(KERN_DEBUG "spihid_apple %s - dev:%hhu", __func__, idev->id);
}

int apple_ll_open(struct hid_device *hdev)
{
	struct spihid_apple *spihid;
	struct spihid_input_dev *idev = hdev->driver_data;
	printk(KERN_DEBUG "spihid_apple %s - dev:%hhu", __func__, idev->id);

	if (idev->hid_desc_len == 0) {
		spihid = spihid_get_data(idev);
		dev_warn(&spihid->spidev->dev,
			 "HID descriptor missing for dev %u", idev->id);
	} else
		idev->ready = true;

	return 0;
}

void apple_ll_close(struct hid_device *hdev)
{
	struct spihid_input_dev *idev = hdev->driver_data;
	printk(KERN_DEBUG "spihid_apple %s - dev:%hhu", __func__, idev->id);
	idev->ready = false;
}

int apple_ll_parse(struct hid_device *hdev)
{
	struct spihid_input_dev *idev = hdev->driver_data;
	printk(KERN_DEBUG "spihid_apple %s idev->id:%hhu", __func__, idev->id);

	return hid_parse_report(hdev, idev->hid_desc, idev->hid_desc_len);
}

int apple_ll_raw_request(struct hid_device *hdev, unsigned char reportnum,
			 __u8 *buf, size_t len, unsigned char rtype,
			 int reqtype)
{
	struct spihid_input_dev *idev = hdev->driver_data;
	struct spihid_apple *spihid = spihid_get_data(idev);

	dev_dbg(&spihid->spidev->dev, "%s reqtype:%d size:%zu", __func__,
		reqtype, len);

	switch (reqtype) {
	case HID_REQ_GET_REPORT:
		return 0;
		// return i2c_hid_get_raw_report(hid, reportnum, buf, len, rtype);
	case HID_REQ_SET_REPORT:
		if (buf[0] != reportnum)
			return -EINVAL;
		return 0;
		// return i2c_hid_output_raw_report(hid, buf, len, rtype, true);
	default:
		return -EIO;
	}
}

int apple_ll_output_report(struct hid_device *hdev, __u8 *buf, size_t len)
{
	struct spihid_input_dev *idev = hdev->driver_data;
	struct spihid_apple *spihid = spihid_get_data(idev);
	if (!spihid)
		return -1;

	dev_dbg(&spihid->spidev->dev, "%s", __func__);

	return 0;
}

static struct hid_ll_driver apple_hid_ll = {
	.start = &apple_ll_start,
	.stop = &apple_ll_stop,
	.open = &apple_ll_open,
	.close = &apple_ll_close,
	.parse = &apple_ll_parse,
	.raw_request = &apple_ll_raw_request,
	.output_report = &apple_ll_output_report,
};

static int spihid_verify_msg(struct spihid_apple *spihid, u8 *buf, size_t len)
{
	u16 msg_crc, crc;
	struct device *dev = &spihid->spidev->dev;

	crc = crc16(0, buf, len - sizeof(__le16));
	msg_crc = get_unaligned_le16(buf + len - sizeof(__le16));
	if (crc != msg_crc) {
		dev_warn_ratelimited(dev, "Read message crc mismatch\n");
		return 0;
	}
	return 1;
}

static bool spihid_process_report(struct spihid_apple *spihid,
				  struct spihid_msg_hdr *hdr, u8 *payload,
				  size_t len)
{
	if (hdr->type == SPIHID_KBD_REPORT) {
		if (spihid->kbd.hid && spihid->kbd.ready) {
			hid_input_report(spihid->kbd.hid, HID_INPUT_REPORT,
					 payload, hdr->length, 1);
			return true;
		}
	}
	if (hdr->type == SPIHID_TP_REPORT) {
		if (spihid->tp.hid && spihid->tp.ready) {
			hid_input_report(spihid->tp.hid, HID_INPUT_REPORT,
					 payload, hdr->length, 1);
			return true;
		}
	}
	return false;
}

static bool spihid_process_response(struct spihid_apple *spihid,
				    struct spihid_msg_hdr *hdr, u8 *payload,
				    size_t len)
{
	struct device *dev = &spihid->spidev->dev;

	if (hdr->type == SPIHID_REQUEST_DESC) {
		switch (hdr->device) {
		case SPI_HID_DEVICE_ID_KBD:
			memcpy(spihid->kbd.hid_desc, payload, hdr->length);
			spihid->kbd.hid_desc_len = hdr->length;
			return true;
		case SPI_HID_DEVICE_ID_TP:
			memcpy(spihid->tp.hid_desc, payload, hdr->length);
			spihid->tp.hid_desc_len = hdr->length;
			return true;
		default:
			dev_dbg(dev,
				"R msg: unexpected device:%hhu for HID descriptor\n",
				hdr->device);
			break;
		}
	}
	return false;
}

static void spihid_process_message(struct spihid_apple *spihid, u8 *data,
				   size_t length, u8 device, u8 flags)
{
	struct device *dev = &spihid->spidev->dev;
	struct spihid_msg_hdr *hdr;
	bool handled = false;
	u8 *payload;

	if (!spihid_verify_msg(spihid, data, length))
		return;

	hdr = (struct spihid_msg_hdr *)data;

	if (hdr->length == 0)
		return;

	payload = data + sizeof(struct spihid_msg_hdr);

	switch (flags) {
	case SPIHID_READ_PACKET:
		if (device == SPI_HID_DEVICE_ID_KBD ||
		    device == SPI_HID_DEVICE_ID_TP)
			handled = spihid_process_report(spihid, hdr, payload,
							hdr->length);
		break;
	case SPIHID_WRITE_PACKET:
		handled = spihid_process_response(spihid, hdr, payload,
						  hdr->length);
		break;
	default:
		break;
	}

	if (!handled) {
		dev_dbg(dev,
			"R unhandled msg: type:%04hx dev:%02hhx id:%hu len:%hu\n",
			hdr->type, hdr->device, hdr->id, hdr->length);
		print_hex_dump_debug("spihid msg: ", DUMP_PREFIX_OFFSET, 16, 1,
				     payload, hdr->length, true);
	}
}

static void spihid_assemble_meesage(struct spihid_apple *spihid,
				    struct spihid_transfer_packet *pkt)
{
	size_t length, offset, remain;
	struct device *dev = &spihid->spidev->dev;
	struct spihid_input_report *rep = &spihid->report;

	length = le16_to_cpu(pkt->length);

#if defined(DEBUG) && DEBUG > 1
	dev_dbg(dev,
		"R pkt: flags:%02hhx dev:%02hhx off:%hu remain:%hu, len:%zu\n",
		pkt->flags, pkt->device, pkt->offset, pkt->remain, length);
	print_hex_dump_debug("spihid pkt: ", DUMP_PREFIX_OFFSET, 16, 1,
			     spihid->rx_buf,
			     sizeof(struct spihid_transfer_packet), true);
#endif

	remain = le16_to_cpu(pkt->remain);
	offset = le16_to_cpu(pkt->offset);

	if (offset + length + remain > U16_MAX) {
		return;
	}

	if (pkt->device != rep->device || pkt->flags != rep->flags ||
	    pkt->offset != rep->offset) {
		rep->device = 0;
		rep->flags = 0;
		rep->offset = 0;
		rep->length = 0;
	}

	if (pkt->offset == 0) {
		if (rep->offset != 0) {
			dev_warn(dev, "incomplete report off:%u len:%u",
				 rep->offset, rep->length);
		}
		memcpy(rep->buf, pkt->data, length);
		rep->offset = length;
		rep->length = length + pkt->remain;
		rep->device = pkt->device;
		rep->flags = pkt->flags;
	} else if (pkt->offset == rep->offset) {
		if (pkt->offset + length + pkt->remain != rep->length) {
			dev_warn(dev, "incomplete report off:%u len:%u",
				 rep->offset, rep->length);
			return;
		}
		memcpy(rep->buf + pkt->offset, pkt->data, pkt->length);
		rep->offset += pkt->length;

		if (rep->offset == rep->length) {
			spihid_process_message(spihid, rep->buf, rep->length,
					       rep->device, rep->flags);
			rep->device = 0;
			rep->flags = 0;
			rep->offset = 0;
			rep->length = 0;
		}
	}
}

static void spihid_process_read(struct spihid_apple *spihid)
{
	u16 crc;
	size_t length;
	struct device *dev = &spihid->spidev->dev;
	struct spihid_transfer_packet *pkt;

	pkt = (struct spihid_transfer_packet *)spihid->rx_buf;

	/* check trnasfer packet crc */
	crc = crc16(0, spihid->rx_buf,
		    offsetof(struct spihid_transfer_packet, crc16));
	if (crc != pkt->crc16) {
		dev_warn_ratelimited(dev, "Read package crc mismatch\n");
		return;
	}

	length = le16_to_cpu(pkt->length);

	// dev_dbg(dev, "R pkt: flags:%02hhx dev:%02hhx off:%hu remain:%hu, len:%zu\n",
	// 	pkt->flags, pkt->device, pkt->offset, pkt->remain, length);

	if (length < sizeof(struct spihid_msg_hdr) + 2) {
		dev_info(dev, "R short packet: len:%zu\n", length);
		print_hex_dump_debug("spihid pkt:", DUMP_PREFIX_OFFSET, 16, 1,
				     pkt->data, length, false);
		return;
	}

	if (length > sizeof(pkt->data)) {
		dev_warn_ratelimited(dev, "Invalid pkt len:%zu", length);
		return;
	}

	/* short message */
	if (pkt->offset == 0 && pkt->remain == 0) {
		spihid_process_message(spihid, pkt->data, length, pkt->device,
				       pkt->flags);
	} else {
		spihid_assemble_meesage(spihid, pkt);
	}
}

static void spihid_read_packet_sync(struct spihid_apple *spihid)
{
	int err;

	err = spi_sync(spihid->spidev, &spihid->rx_msg);
	if (!err) {
		spihid_process_read(spihid);
	} else {
		dev_warn(&spihid->spidev->dev, "RX failed: %d\n", err);
	}
}

static irqreturn_t spi_hid_apple_irq(int irq, void *data)
{
	struct spihid_apple *spihid = data;

	spihid_read_packet_sync(spihid);

	return IRQ_HANDLED;
}

static void spihid_apple_setup_spi_msgs(struct spihid_apple *spihid)
{
	memset(&spihid->rx_transfer, 0, sizeof(spihid->rx_transfer));

	spihid->rx_transfer.rx_buf = spihid->rx_buf;
	spihid->rx_transfer.len = sizeof(struct spihid_transfer_packet);

	spi_message_init(&spihid->rx_msg);
	spi_message_add_tail(&spihid->rx_transfer, &spihid->rx_msg);

	memset(&spihid->tx_transfer, 0, sizeof(spihid->rx_transfer));
	memset(&spihid->status_transfer, 0, sizeof(spihid->status_transfer));

	spihid->tx_transfer.tx_buf = spihid->tx_buf;
	spihid->tx_transfer.len = sizeof(struct spihid_transfer_packet);
	spihid->tx_transfer.delay.unit = SPI_DELAY_UNIT_USECS;
	spihid->tx_transfer.delay.value = SPI_RW_CHG_DELAY_US;

	spihid->status_transfer.rx_buf = spihid->status_buf;
	spihid->status_transfer.len = sizeof(spi_hid_apple_status_ok);

	spi_message_init(&spihid->tx_msg);
	spi_message_add_tail(&spihid->tx_transfer, &spihid->tx_msg);
	spi_message_add_tail(&spihid->status_transfer, &spihid->tx_msg);
}

static int spihid_apple_setup_spi(struct spihid_apple *spihid)
{
	spihid_apple_setup_spi_msgs(spihid);

	/* reset the controller on boot */
	gpiod_direction_output(spihid->enable_gpio, 1);
	msleep(5);
	gpiod_direction_output(spihid->enable_gpio, 0);
	msleep(5);

	return 0;
}

static int spihid_apple_spi_poweron(struct spihid_apple *spihid)
{
	/* turn SPI device on */
	gpiod_direction_output(spihid->enable_gpio, 1);
	msleep(50);

	return 0;
}

static int spihid_apple_tp_set_mode(struct spihid_apple *spihid, u8 device,
				    u8 mode)
{
	struct spihid_transfer_packet *pkt;
	struct spihid_msg_tp_set_mode *msg;

	dev_dbg(&spihid->spidev->dev, "%s:%d\n", __func__, __LINE__);

	pkt = (struct spihid_transfer_packet *)spihid->tx_buf;

	memset(pkt, 0, sizeof(*pkt));
	pkt->flags = SPIHID_WRITE_PACKET;
	pkt->device = device;
	pkt->length = sizeof(*msg);

	msg = (struct spihid_msg_tp_set_mode *)&pkt->data[0];
	msg->hdr.type = SPIHID_SET_TP_MODE;
	msg->hdr.device = device;
	msg->hdr.id = spihid->msg_id++;
	msg->hdr.rsplen = cpu_to_le16(32);
	msg->hdr.length = cpu_to_le16(2);
	msg->device = device;
	msg->mode = mode;
	msg->crc16 = crc16(0, &pkt->data[0],
			   offsetof(struct spihid_msg_tp_set_mode, crc16));

	pkt->crc16 = crc16(0, spihid->tx_buf,
			   offsetof(struct spihid_transfer_packet, crc16));

	return spi_sync(spihid->spidev, &spihid->tx_msg);
}

static int spihid_apple_request_descriptor(struct spihid_apple *spihid,
					   u8 device)
{
	struct spihid_transfer_packet *pkt;
	struct spihid_msg_req_desc *msg;

	dev_dbg(&spihid->spidev->dev, "%s:%d\n", __func__, __LINE__);

	pkt = (struct spihid_transfer_packet *)spihid->tx_buf;

	memset(pkt, 0, sizeof(*pkt));
	pkt->flags = SPIHID_WRITE_PACKET;
	pkt->device = SPI_HID_DEVICE_ID_INFO;
	pkt->length = sizeof(*msg);

	msg = (struct spihid_msg_req_desc *)&pkt->data[0];
	msg->hdr.type = cpu_to_le16(SPIHID_REQUEST_DESC);
	msg->hdr.device = device;
	msg->hdr.id = spihid->msg_id++;
	msg->hdr.rsplen = cpu_to_le16(SPIHID_DESC_MAX);
	msg->hdr.length = 0;
	msg->crc16 = crc16(0, &pkt->data[0],
			   offsetof(struct spihid_msg_req_desc, crc16));

	pkt->crc16 = crc16(0, spihid->tx_buf,
			   offsetof(struct spihid_transfer_packet, crc16));

	return spi_sync(spihid->spidev, &spihid->tx_msg);
}

static int spihid_register_hid_device(struct spihid_apple *spihid,
				      struct spihid_input_dev *idev, u8 device)
{
	int ret;
	struct hid_device *hid;

	idev->id = device;

	dev_dbg(&spihid->spidev->dev, "%s:%d idev->id:%hhu device:%hhu\n",
		__func__, __LINE__, idev->id, device);

	hid = hid_allocate_device();
	if (IS_ERR(hid))
		return PTR_ERR(hid);

	if (device == SPI_HID_DEVICE_ID_KBD) {
		strscpy(hid->name, "MacBook Magic Keyboard", sizeof(hid->name));
		/* TODO: use distinctive product IDs for j293, j313, j31[46]? */
		hid->product = USB_DEVICE_ID_APPLE_MAGIC_KEYBOARD_2021;
	} else if (device == SPI_HID_DEVICE_ID_TP) {
		strscpy(hid->name, "MacBook Force Touch trackpad",
			sizeof(hid->name));
		hid->product = SPI_DEVICE_ID_APPLE_FT_TRACKPAD;
	}

	snprintf(hid->phys, sizeof(hid->phys), "%s (%hhx)",
		 dev_name(&spihid->spidev->dev), device);
	// 	strscpy(hid->uniq, <device-uniq-src>, sizeof(hid->uniq));

	hid->ll_driver = &apple_hid_ll;
	hid->bus = BUS_SPI;
	hid->vendor = SPI_VENDOR_ID_APPLE;
	hid->version = le16_to_cpu(1);
	/* hid->country = TODO: import from device tree; */
	hid->dev.parent = &spihid->spidev->dev;
	hid->driver_data = idev;

	ret = hid_add_device(hid);
	if (ret < 0)
		hid_destroy_device(hid);

	idev->hid = hid;

	return ret;
}

static void spihid_destroy_hid_device(struct spihid_input_dev *idev)
{
	if (idev->hid) {
		hid_destroy_device(idev->hid);
		idev->hid = NULL;
	}
	idev->ready = false;
}

static int spi_hid_apple_probe(struct spi_device *spi)
{
	struct device *dev = &spi->dev;
	struct spihid_apple *spihid;
	int err, count;

	dev_dbg(dev, "%s:%d\n", __func__, __LINE__);

	spihid = devm_kzalloc(dev, sizeof(*spihid), GFP_KERNEL);
	if (!spihid)
		return -ENOMEM;

	spihid->spidev = spi;

	spihid->enable_gpio = devm_gpiod_get_index(dev, "spien", 0, 0);
	if (IS_ERR(spihid->enable_gpio)) {
		err = PTR_ERR(spihid->enable_gpio);
		dev_err(dev, "failed to get 'spien' gpio pin: %d", err);
		goto error;
	}

	// init spi
	spi_set_drvdata(spi, spihid);

	/* allocate SPI buffers */
	spihid->rx_buf = devm_kmalloc(
		&spi->dev, sizeof(struct spihid_transfer_packet), GFP_KERNEL);
	spihid->tx_buf = devm_kmalloc(
		&spi->dev, sizeof(struct spihid_transfer_packet), GFP_KERNEL);
	spihid->status_buf = devm_kmalloc(
		&spi->dev, sizeof(spi_hid_apple_status_ok), GFP_KERNEL);

	if (!spihid->rx_buf || !spihid->tx_buf || !spihid->status_buf)
		return -ENOMEM;

	spihid->report.buf =
		devm_kmalloc(dev, SPIHID_MAX_INPUT_REPORT_SIZE, GFP_KERNEL);

	spihid->kbd.hid_desc = devm_kmalloc(dev, SPIHID_DESC_MAX, GFP_KERNEL);
	spihid->tp.hid_desc = devm_kmalloc(dev, SPIHID_DESC_MAX, GFP_KERNEL);

	if (!spihid->report.buf || !spihid->kbd.hid_desc ||
	    !spihid->tp.hid_desc)
		return -ENOMEM;

	err = spihid_apple_setup_spi(spihid);
	if (err < 0)
		goto error;

	/* power device on */
	err = spihid_apple_spi_poweron(spihid);
	if (err < 0)
		goto error;

	/* request HID irq */
	spihid->irq = of_irq_get(dev->of_node, 0);
	if (spihid->irq < 0) {
		err = spihid->irq;
		dev_err(dev, "failed to get 'extended-irq': %d", err);
		goto error;
	}
	err = devm_request_threaded_irq(dev, spihid->irq, NULL,
					spi_hid_apple_irq, IRQF_ONESHOT,
					"spi-hid-apple-irq", spihid);
	if (err < 0) {
		dev_err(dev, "failed to request extended-irq %d: %d",
			spihid->irq, err);
		goto error;
	}

	/* request HID descriptors and poll */
	if (spihid_apple_request_descriptor(spihid, SPI_HID_DEVICE_ID_KBD))
		dev_warn(dev, "req keyboard desc failed");

	for (count = 0; count < 3 && !spihid->kbd.hid_desc_len; count++) {
		msleep(1);
	}
	dev_dbg(dev, "keyboard hid desc len:%u after %d tries",
		spihid->kbd.hid_desc_len, count);

	if (spihid->kbd.hid_desc_len > 0) {
		err = spihid_register_hid_device(spihid, &spihid->kbd,
						 SPI_HID_DEVICE_ID_KBD);
		if (err < 0)
			dev_warn(dev, "Failed to add HID keyboard device: %d",
				 err);
	}

	if (spihid_apple_request_descriptor(spihid, SPI_HID_DEVICE_ID_TP))
		dev_warn(dev, "req touchpad desc failed");

	for (count = 0; count < 3 && !spihid->tp.hid_desc_len; count++) {
		msleep(1);
	}
	dev_dbg(dev, "touchpad hid desc len:%d after %d tries",
		spihid->tp.hid_desc_len, count);

	if (spihid->tp.hid_desc_len > 0) {
		err = spihid_register_hid_device(spihid, &spihid->tp,
						 SPI_HID_DEVICE_ID_TP);
		if (err < 0)
			dev_warn(dev, "Failed to add HID Touchpad device: %d",
				 err);
	}

	/* switch to raw trackpad events for multi touch support */
	spihid_apple_tp_set_mode(spihid, SPI_HID_DEVICE_ID_TP,
				 SPIHID_TP_MODE_RAW);
	//spihid_apple_tp_set_mode(spihid, SPI_HID_DEVICE_ID_TP, SPIHID_TP_MODE_HID);

	return 0;
error:
	return err;
}

static int spi_hid_apple_remove(struct spi_device *spi)
{
	struct spihid_apple *spihid = spi_get_drvdata(spi);

	/* disable irq */
	disable_irq(spihid->irq);

	/* power down  SPI device */
	gpiod_direction_output(spihid->enable_gpio, 0);

	/* destroy input devices */

	spihid_destroy_hid_device(&spihid->tp);
	spihid_destroy_hid_device(&spihid->kbd);

	return 0;
}

#ifdef CONFIG_PM_SLEEP
static void spi_hid_apple_shutdown(struct spi_device *spi)
{
}
#endif

static const struct of_device_id spi_hid_apple_match[] = {
	{ .compatible = "apple,spi-hid-transport" },
	{},
};
MODULE_DEVICE_TABLE(of, spi_hid_apple_match);

static struct spi_driver spi_hid_apple_driver = {
	.driver = {
		.name	= "spi_hid_apple",
		//.pm	= &spi_hid_apple_pm,
		.of_match_table = of_match_ptr(spi_hid_apple_match),
	},

	.probe		= spi_hid_apple_probe,
	.remove		= spi_hid_apple_remove,
#ifdef CONFIG_PM_SLEEP
	.shutdown	= spi_hid_apple_shutdown,
#endif
};

module_spi_driver(spi_hid_apple_driver);

MODULE_DESCRIPTION("Apple SPI HID transport driver");
MODULE_AUTHOR("Janne Grunau <j@jannau.net>");
MODULE_LICENSE("GPL");
