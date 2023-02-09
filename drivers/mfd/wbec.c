// SPDX-License-Identifier: GPL-2.0-only
/*
 * TODO
 * Write description
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/i2c.h>
#include <linux/rtc.h>
#include <linux/mfd/core.h>
#include <linux/interrupt.h>
#include <linux/reboot.h>
#include "wbec.h"


static const struct regmap_config wbec_regmap_config_8 = {
	.name = "regmap_8",
	.reg_bits = 8,
	.val_bits = 8,
	// .cache_type = REGCACHE_RBTREE,
};

static const struct regmap_config wbec_regmap_config_16 = {
	.name = "regmap_16",
	.reg_bits = 8,
	.val_bits = 16,
};

static const struct mfd_cell wbec_cells[] = {
	{ .name = "wbec-rtc", .id = PLATFORM_DEVID_NONE, },
	{ .name = "wbec-iio", .id = PLATFORM_DEVID_NONE, },
	{ .name = "wbec-pwrkey", .id = PLATFORM_DEVID_NONE, },
	{ .name = "wbec-watchdog", .id = PLATFORM_DEVID_NONE, },
	{ .name = "wbec-gpio", .id = PLATFORM_DEVID_NONE, },
};

static struct i2c_client *wbec_i2c_client;

static void wbec_pm_power_off(void)
{
	int ret;
	struct wbec *wbec = i2c_get_clientdata(wbec_i2c_client);

	// TODO Remove debug
	dev_info(&wbec_i2c_client->dev, "%s function\n", __func__);

	ret = regmap_update_bits(wbec->regmap_8, WBEC_REG_POWERCTRL, WBEC_REG_BIT_POWEROFF, 1);
	if (ret)
		dev_err(&wbec_i2c_client->dev, "Failed to shutdown device!\n");

	/* Give capacitors etc. time to drain to avoid kernel panic msg. */
	msleep(1000);
	dev_err(&wbec_i2c_client->dev, "Device not actually shutdown. Check EC and FETs!\n");
}

static int wbec_restart_notify(struct notifier_block *this, unsigned long mode, void *cmd)
{
	struct wbec *wbec = i2c_get_clientdata(wbec_i2c_client);

	// TODO Remove debug
	dev_info(&wbec_i2c_client->dev, "%s function\n", __func__);

	return NOTIFY_DONE;
}

static struct notifier_block wbec_restart_handler = {
	.notifier_call = wbec_restart_notify,
	.priority = 192,
};

static void wbec_shutdown(struct i2c_client *client)
{
	struct wbec *wbec = i2c_get_clientdata(client);

	// TODO Remove debug
	dev_info(&client->dev, "%s function\n", __func__);
}

static int wbec_probe(struct i2c_client *client)
{
	struct wbec *wbec;
	int ret;

	wbec = devm_kzalloc(&client->dev, sizeof(*wbec), GFP_KERNEL);
	if (!wbec)
		return -ENOMEM;

	wbec->i2c = client;
	i2c_set_clientdata(client, wbec);

	wbec->regmap_cfg_8 = &wbec_regmap_config_8;
	wbec->regmap_cfg_16 = &wbec_regmap_config_16;

	wbec->regmap_8 = devm_regmap_init_i2c(client, wbec->regmap_cfg_8);
	if (IS_ERR(wbec->regmap_8)) {
		dev_err(&client->dev, "regmap initialization failed\n");
		return PTR_ERR(wbec->regmap_8);
	}

	wbec->regmap_16 = devm_regmap_init_i2c(client, wbec->regmap_cfg_16);
	if (IS_ERR(wbec->regmap_16)) {
		dev_err(&client->dev, "regmap initialization failed\n");
		return PTR_ERR(wbec->regmap_16);
	}

	ret = devm_mfd_add_devices(&client->dev, PLATFORM_DEVID_NONE,
			      wbec_cells, ARRAY_SIZE(wbec_cells), NULL, 0,
			      NULL);
	if (ret) {
		dev_err(&client->dev, "failed to add MFD devices %d\n", ret);
		return ret;
	}

	wbec_i2c_client = client;
	pm_power_off = wbec_pm_power_off;

	return ret;
}

static int wbec_remove(struct i2c_client *client)
{
	// TODO Remove debug
	dev_info(&client->dev, "%s function\n", __func__);
	return 0;
}

#ifdef CONFIG_OF
static const struct of_device_id wbec_of_match[] = {
	// TODO DT compatible string "wbec" appears un-documented -- check ./Documentation/devicetree/bindings/
	{ .compatible = "wbec" },
	{}
};
MODULE_DEVICE_TABLE(of, wbec_of_match);
#endif

static struct i2c_driver wbec_driver = {
	.driver = {
		.name = "wbec",
		.of_match_table = of_match_ptr(wbec_of_match),
	},
	.probe_new = wbec_probe,
	.remove = wbec_remove,
	.shutdown = wbec_shutdown,
};


module_i2c_driver(wbec_driver);

MODULE_AUTHOR("Pavel Gasheev <pavel.gasheev@wirenboard.ru>");
MODULE_DESCRIPTION("Wiren Board 7 Embedded Controller MFD driver");
MODULE_LICENSE("GPL");
