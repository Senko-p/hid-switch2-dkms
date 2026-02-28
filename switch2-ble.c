// SPDX-License-Identifier: GPL-2.0+
/*
 * HID driver for Nintendo Switch controllers — BLE transport
 *
 * Implements the BLE transport for controllers connected via the BlueZ
 * procon2 plugin, which creates a uhid HID device and handles all
 * GATT-level communication.
 *
 * This module is the entry point for the BLE path.  When the BlueZ plugin
 * creates the uhid device (BUS_BLUETOOTH, VID 0x057E, PID 0x2069), the
 * kernel fires a uevent that modprobe resolves to this module.  Loading
 * this module implicitly pulls in hid-switch2 as a dependency (via ELF
 * symbol references), mirroring the USB path where switch2-usb is the
 * entry point and hid-switch2 is its dependency.
 *
 * send_command path (LED, grip, rumble-arm, …):
 *   Wraps the payload in a standard 0x91 frame with NS2_TRANS_BT and
 *   calls hid_hw_output_report().  The plugin forwards it as-is to
 *   GATT 0x0014.
 *
 * send_rumble path (per-frame haptic from switch2_rumble_work):
 *   Extracts the 5-byte left-channel HD-rumble encoding from the
 *   64-byte buffer produced by switch2_rumble_work, wraps it in a
 *   0x91 NS2_CMD_VIBRATE/0x02 frame, and calls hid_hw_output_report().
 *   The plugin forwards it to GATT 0x0014 where the controller plays
 *   it through its LRA haptic motor.
 *
 *   GC (ERM motor, buf[0]==3): not implemented for BLE; returns 0.
 *
 * The vibrate-arm command (NS2_CMD_VIBRATE/0x02 + [0x01,0,0,0]) is
 * sent once after input registration via send_command in switch2_ble_probe.
 */

#include "hid-switch2.h"
#include <linux/hid.h>
#include <linux/module.h>
#include <linux/slab.h>

static int switch2_ble_send_cmd(enum switch2_cmd command, uint8_t subcommand,
	const void *msg, size_t len, struct switch2_cfg_intf *intf)
{
	struct switch2_controller *ns2 = intf->parent;
	struct switch2_cmd_header header = {
		command, NS2_DIR_OUT | NS2_FLAG_OK, NS2_TRANS_BT,
		subcommand, 0, len
	};
	uint8_t *frame;
	int ret;

	if (WARN_ON(len > 56))
		return -EINVAL;

	if (!ns2->hdev)
		return -ENODEV;

	frame = kzalloc(sizeof(header) + len, GFP_KERNEL);
	if (!frame)
		return -ENOMEM;

	memcpy(frame, &header, sizeof(header));
	if (msg && len)
		memcpy(frame + sizeof(header), msg, len);

	ret = hid_hw_output_report(ns2->hdev, frame, sizeof(header) + len);
	kfree(frame);
	return ret;
}

static int switch2_ble_send_rumble(const uint8_t *buf, size_t len,
	struct switch2_cfg_intf *intf)
{
	struct switch2_controller *ns2 = intf->parent;
	/*
	 * BLE haptic frame: 8-byte 0x91 header + 5-byte HD-rumble payload.
	 * Subcmd 0x02 is the per-frame vibrate command (armed once at init).
	 */
	struct switch2_cmd_header header = {
		NS2_CMD_VIBRATE, NS2_DIR_OUT | NS2_FLAG_OK, NS2_TRANS_BT,
		0x02, 0, 5
	};
	uint8_t frame[sizeof(header) + 5];

	if (!ns2->hdev)
		return -ENODEV;

	if (len < 7)
		return -EINVAL;

	/* buf[0]: controller type.  GC uses ERM — not implemented for BLE. */
	if (buf[0] == 3)
		return 0;

	/*
	 * JC (buf[0]==1) and PRO (buf[0]==2) both use HD rumble.
	 * Extract the 5-byte left-channel encoding at buf[0x02..0x06].
	 * PRO right channel (buf[0x12..0x16]) is ignored for now; both LRAs
	 * receive the same pattern since switch2_rumble_work copies
	 * ns2->rumble.hd to both sides anyway.
	 */
	memcpy(frame, &header, sizeof(header));
	memcpy(frame + sizeof(header), &buf[0x02], 5);

	return hid_hw_output_report(ns2->hdev, frame, sizeof(frame));
}

static int switch2_ble_probe(struct hid_device *hdev,
	const struct hid_device_id *id)
{
	struct switch2_controller *ns2;
	struct switch2_ble *ns2_ble;
	char phys[64];
	int ret;
#ifdef CONFIG_SWITCH2_FF
	static const uint8_t vibrate_arm[] = { 0x01, 0x00, 0x00, 0x00 };
#endif

	snprintf(phys, sizeof(phys), "bt%s", hdev->uniq);

	ret = hid_parse(hdev);
	if (ret) {
		hid_err(hdev, "parse failed %d\n", ret);
		return ret;
	}

	/*
	 * Do not expose a hidraw node for BLE controllers.  Steam/SDL would
	 * otherwise write USB-format 0x91 commands (with a HID report-ID prefix
	 * and NS2_TRANS_USB transport byte) directly to hidraw, bypassing
	 * switch2_ble_send_rumble and sending malformed data to GATT 0x0014.
	 * Without hidraw, Steam falls back to the evdev FF_RUMBLE interface
	 * which goes through switch2_rumble_work → switch2_ble_send_rumble and
	 * produces correctly-formatted BLE VIBRATE frames.
	 */
	ret = hid_hw_start(hdev, 0);
	if (ret) {
		hid_err(hdev, "hw_start failed %d\n", ret);
		return ret;
	}

	ret = hid_hw_open(hdev);
	if (ret) {
		hid_err(hdev, "hw_open failed %d\n", ret);
		goto err_stop;
	}

	ns2 = switch2_get_controller(phys);
	if (!ns2) {
		ret = -ENOMEM;
		goto err_close;
	}

	ns2_ble = devm_kzalloc(&hdev->dev, sizeof(*ns2_ble), GFP_KERNEL);
	if (!ns2_ble) {
		ret = -ENOMEM;
		goto err_put;
	}

	mutex_lock(&ns2->lock);
	ns2->hdev = hdev;

	ns2->player_id = U32_MAX;
	ret = switch2_alloc_player_id();
	if (ret < 0)
		hid_warn(hdev, "Failed to allocate player ID, skipping; ret=%d\n", ret);
	else
		ns2->player_id = ret;

	/*
	 * BLE fast-path: the BlueZ procon2 plugin has already completed the
	 * full initialization sequence.  Skip the USB transport rendezvous
	 * and flash calibration reads.
	 *
	 * ctlr_type must be set before switch2_init_rumble (which checks for
	 * NS2_CTLR_TYPE_GC).  Zero-initialised stick_calib means
	 * switch2_report_axis() will use the "(value - 2048) * 16" fallback,
	 * which is correct because the BlueZ plugin pre-normalises stick
	 * values to centre=2048.
	 */
	ns2_ble->cfg.parent       = ns2;
	ns2_ble->cfg.send_command = switch2_ble_send_cmd;
	ns2_ble->cfg.send_rumble  = switch2_ble_send_rumble;

	ns2->ctlr_type = NS2_CTLR_TYPE_PRO;
	ns2->cfg       = (struct switch2_cfg_intf *)ns2_ble;
	ns2->init_step = NS2_INIT_DONE;
	/*
	 * The BlueZ plugin inverts Y (4095 - y) before packing the uhid
	 * payload so that hid-generic sees correct axis direction.
	 * Tell hid-switch2 not to negate Y a second time.
	 */
	ns2->y_pre_inverted = true;
#ifdef CONFIG_SWITCH2_FF
	switch2_init_rumble(ns2);
#endif
	hid_set_drvdata(hdev, ns2);

	ret = switch2_init_input(ns2);
	if (ret) {
		mutex_unlock(&ns2->lock);
		goto err_close;
	}

#ifdef CONFIG_SWITCH2_FF
	/*
	 * Arm the HD-rumble engine.  Sent post-init via the uhid output path;
	 * the BlueZ plugin forwards it to GATT 0x0014.
	 */
	if (ns2->ctlr_type != NS2_CTLR_TYPE_GC)
		ns2->cfg->send_command(NS2_CMD_VIBRATE, 0x02,
			vibrate_arm, sizeof(vibrate_arm), ns2->cfg);
#endif

	mutex_unlock(&ns2->lock);
	return 0;

err_put:
	switch2_controller_put(ns2);
err_close:
	hid_hw_close(hdev);
err_stop:
	hid_hw_stop(hdev);
	return ret;
}

static void switch2_ble_remove(struct hid_device *hdev)
{
	struct switch2_controller *ns2 = hid_get_drvdata(hdev);
#ifdef CONFIG_SWITCH2_FF
	unsigned long flags;

	spin_lock_irqsave(&ns2->rumble_lock, flags);
	cancel_delayed_work_sync(&ns2->rumble_work);
	spin_unlock_irqrestore(&ns2->rumble_lock, flags);
#endif
	mutex_lock(&ns2->lock);
	ns2->hdev  = NULL;
	ns2->cfg   = NULL;
	mutex_unlock(&ns2->lock);
	hid_hw_close(hdev);
	switch2_free_player_id(ns2->player_id);
	switch2_controller_put(ns2);
	hid_hw_stop(hdev);
}

static const struct hid_device_id switch2_ble_devices[] = {
	/* uhid devices created by BlueZ plugins for BLE connections.
	 * GameCube (0x2073) is wired-only and intentionally omitted. */
	{ HID_BLUETOOTH_DEVICE(USB_VENDOR_ID_NINTENDO,
			       USB_DEVICE_ID_NINTENDO_NS2_JOYCONL) },
	{ HID_BLUETOOTH_DEVICE(USB_VENDOR_ID_NINTENDO,
			       USB_DEVICE_ID_NINTENDO_NS2_JOYCONR) },
	{ HID_BLUETOOTH_DEVICE(USB_VENDOR_ID_NINTENDO,
			       USB_DEVICE_ID_NINTENDO_NS2_PROCON) },
	{}
};
MODULE_DEVICE_TABLE(hid, switch2_ble_devices);

static struct hid_driver switch2_ble_hid_driver = {
	.name      = "switch2-ble",
	.id_table  = switch2_ble_devices,
	.probe     = switch2_ble_probe,
	.remove    = switch2_ble_remove,
	.raw_event = switch2_event,
};
module_hid_driver(switch2_ble_hid_driver);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Martin BTS <martinbts@gmx.net>");
MODULE_DESCRIPTION("BLE transport for Nintendo Switch 2 Controllers");
