// SPDX-License-Identifier: GPL-2.0-only
/*
TODO
Write description
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


/* WB EC IRQ Definitions */
#define WBEC_IRQ_PWRON_RISE		0
#define WBEC_IRQ_PWRON_FALL		1


#define WBEC_IRQ_PWRON_RISE_MSK	BIT(0)
#define WBEC_IRQ_PWRON_FALL_MSK	BIT(2)


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

static const struct resource wbec_key_resources[] = {
	DEFINE_RES_IRQ(WBEC_IRQ_PWRON_RISE),
	DEFINE_RES_IRQ(WBEC_IRQ_PWRON_FALL),
};

static const struct mfd_cell wbec_cells[] = {
	{ .name = "wbec-rtc", .id = PLATFORM_DEVID_NONE, },
	{ .name = "wbec-iio", .id = PLATFORM_DEVID_NONE, },
	{	.name = "wbec-pwrkey",
		.num_resources = ARRAY_SIZE(wbec_key_resources),
		.resources = &wbec_key_resources[0],
		.id = PLATFORM_DEVID_NONE,
	},
};

static const struct regmap_irq wbec_irqs[] = {
	[WBEC_IRQ_PWRON_RISE] = {
		.mask = WBEC_IRQ_PWRON_RISE_MSK,
		.reg_offset = 0,
	},
	[WBEC_IRQ_PWRON_FALL] = {
		.mask = WBEC_IRQ_PWRON_FALL_MSK,
		.reg_offset = 0,
	},
};

static struct regmap_irq_chip wbec_irq_chip = {
	.name = "wbec",
	.irqs = wbec_irqs,
	.num_irqs = ARRAY_SIZE(wbec_irqs),
	.num_regs = 1,
	.status_base = WBEC_REG_INT,
	.mask_base = WBEC_REG_INT_MSK,
	.ack_base = WBEC_REG_INT,
	.init_ack_masked = true,
};

static struct i2c_client *wbec_i2c_client;

static void wbec_pm_power_off(void)
{
	int ret;
	struct wbec *wbec = i2c_get_clientdata(wbec_i2c_client);

	printk(KERN_INFO "%s function\n", __func__);

	ret = regmap_update_bits(wbec->regmap_8, DEV_POWER_REG, DEV_OFF, 1);
	if (ret)
		dev_err(&wbec_i2c_client->dev, "Failed to shutdown device!\n");
}

static int wbec_restart_notify(struct notifier_block *this, unsigned long mode, void *cmd)
{
	struct wbec *wbec = i2c_get_clientdata(wbec_i2c_client);

	printk(KERN_INFO "%s function\n", __func__);

	return NOTIFY_DONE;
}

static struct notifier_block wbec_restart_handler = {
	.notifier_call = wbec_restart_notify,
	.priority = 192,
};

static void wbec_shutdown(struct i2c_client *client)
{
	struct wbec *wbec = i2c_get_clientdata(client);

	printk(KERN_INFO "%s function\n", __func__);
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
	wbec->regmap_irq_chip = &wbec_irq_chip;

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

	// if (!client->irq) {
	// 	dev_err(&client->dev, "No interrupt support, no core IRQ\n");
	// 	return -EINVAL;
	// }

	// ret = regmap_add_irq_chip(wbec->regmap, client->irq,
	// 			  IRQF_ONESHOT, -1,
	// 			  wbec->regmap_irq_chip, &wbec->irq_data);
	// if (ret) {
	// 	dev_err(&client->dev, "Failed to add irq_chip %d\n", ret);
	// 	return ret;
	// }

	ret = devm_mfd_add_devices(&client->dev, PLATFORM_DEVID_NONE,
			      wbec_cells, ARRAY_SIZE(wbec_cells), NULL, 0,
			      NULL);
	if (ret) {
		dev_err(&client->dev, "failed to add MFD devices %d\n", ret);
		return ret;
	}

	return ret;
}

static int wbec_remove(struct i2c_client *client)
{
	printk(KERN_INFO "wbec_remove function\n");
	return 0;
}

#ifdef CONFIG_OF
static const struct of_device_id wbec_of_match[] = {
	{ .compatible = "wbec" },
	{}
};
MODULE_DEVICE_TABLE(of, wbec_of_match);
#endif

static struct i2c_driver wbec_driver = {
      .driver = {
			.name   = "wbec",
			.of_match_table = of_match_ptr(wbec_of_match),
      },
      .probe_new      = wbec_probe,
      .remove         = wbec_remove,
	  .shutdown = wbec_shutdown,
};


module_i2c_driver(wbec_driver);

MODULE_AUTHOR("Pavel Gasheev <pavel.gasheev@wirenboard.ru>");
MODULE_DESCRIPTION("Wiren Board 7 Embedded Controller MFD driver");
MODULE_LICENSE("GPL");