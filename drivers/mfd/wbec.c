// SPDX-License-Identifier: GPL-2.0-only
/*
 * wbec.c - Wiren Board Embedded Controller MFD driver
 *
 * Copyright (c) 2023 Wiren Board LLC
 *
 * Author: Pavel Gasheev <pavel.gasheev@wirenboard.ru>
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/spi/spi.h>
#include <linux/rtc.h>
#include <linux/mfd/core.h>
#include <linux/interrupt.h>
#include <linux/reboot.h>
#include <linux/notifier.h>
#include <linux/mfd/wbec.h>

#define WBEC_ID						0x3CD2
#define WBEC_POWER_RESET_DELAY_MS	500

static const struct regmap_config wbec_regmap_config = {
	.reg_bits = 16,
	.read_flag_mask = BIT(7),
	.val_bits = 16,
};

static const struct mfd_cell wbec_cells[] = {
	{
		.name = "wbec-iio",
		.id = PLATFORM_DEVID_NONE,
		.of_compatible = "wirenboard,wbec-iio"
	},
	{
		.name = "wbec-gpio",
		.id = PLATFORM_DEVID_NONE,
		.of_compatible = "wirenboard,wbec-gpio"
	},
	{ .name = "wbec-watchdog", .id = PLATFORM_DEVID_NONE, },
	{ .name = "wbec-rtc", .id = PLATFORM_DEVID_NONE, },
	{ .name = "wbec-pwrkey", .id = PLATFORM_DEVID_NONE, },
};

static struct wbec *wbec_pm;

static void wbec_pm_power_off(void)
{
	int ret;
	struct wbec *wbec = wbec_pm;

	if (!wbec)
		return;

	dev_dbg(wbec->dev, "%s function\n", __func__);

	ret = regmap_write(wbec->regmap, WBEC_REG_POWER_CTRL, WBEC_REG_POWER_CTRL_OFF_MSK);
	if (ret)
		dev_err(wbec->dev, "Failed to shutdown device!\n");

	/* Give capacitors etc. time to drain to avoid kernel panic msg. */
	msleep(WBEC_POWER_RESET_DELAY_MS);
	dev_err(wbec->dev, "Device not actually shutdown. Check EC and FETs!\n");
}

static int wbec_restart_notify(struct notifier_block *this, unsigned long mode, void *cmd)
{
	struct wbec *wbec = wbec_pm;
	int ret;

	if (!wbec)
		return NOTIFY_STOP;

	dev_dbg(wbec->dev, "%s function\n", __func__);

	ret = regmap_write(wbec->regmap, WBEC_REG_POWER_CTRL, WBEC_REG_POWER_CTRL_REBOOT_MSK);
	if (ret)
		dev_err(wbec->dev, "Failed to reboot device!\n");

	/* Give capacitors etc. time to drain to avoid kernel panic msg. */
	msleep(WBEC_POWER_RESET_DELAY_MS);
	dev_err(wbec->dev, "Device not actually rebooted. Check EC and FETs!\n");

	return NOTIFY_DONE;
}

static struct notifier_block wbec_restart_handler = {
	.notifier_call = wbec_restart_notify,
	.priority = 255,
};

static int wbec_probe(struct spi_device *spi)
{
	struct wbec *wbec;
	int ret;
	int wbec_id;
	u16 test[4];

	dev_dbg(&spi->dev, "%s function. irq=%d\n", __func__, spi->irq);

	wbec = devm_kzalloc(&spi->dev, sizeof(*wbec), GFP_KERNEL);
	if (!wbec)
		return -ENOMEM;

	wbec->dev = &spi->dev;

	spi->mode = SPI_MODE_0;
	spi->bits_per_word = 8;
	spi_setup(spi);

	spi_set_drvdata(spi, wbec);

	wbec->regmap = devm_regmap_init_spi(spi, &wbec_regmap_config);
	if (IS_ERR(wbec->regmap)) {
		dev_err(&spi->dev, "regmap initialization failed\n");
		return PTR_ERR(wbec->regmap);
	}

	ret = regmap_bulk_read(wbec->regmap, 0, test, ARRAY_SIZE(test));

	ret = regmap_read(wbec->regmap, WBEC_REG_INFO_WBEC_ID, &wbec_id);
	if (ret < 0) {
		dev_err(&spi->dev, "failed to read the wbec id at 0x%X\n",
			WBEC_REG_INFO_WBEC_ID);
		return ret;
	}
	if (wbec_id != WBEC_ID) {
		dev_err(&spi->dev, "wrong wbec ID at 0x%X. Get 0x%X istead of 0x%X\n",
			WBEC_REG_INFO_WBEC_ID, wbec_id, WBEC_ID);
		return -ENOTSUPP;
	}

	ret = devm_mfd_add_devices(&spi->dev, PLATFORM_DEVID_NONE,
			      wbec_cells, ARRAY_SIZE(wbec_cells), NULL, 0, NULL);
	if (ret) {
		dev_err(&spi->dev, "failed to add MFD devices %d\n", ret);
		return ret;
	}

	wbec_pm = wbec;
	pm_power_off = wbec_pm_power_off;

	ret = register_restart_handler(&wbec_restart_handler);
	if (ret)
		dev_warn(&spi->dev, "failed to register restart handler\n");

	dev_info(&spi->dev, "WBEC device added\n");

	return ret;
}

static int wbec_remove(struct spi_device *spi)
{
	dev_dbg(&spi->dev, "%s function\n", __func__);

	/**
	 * pm_power_off may points to a function from another module.
	 * Check if the pointer is set by us and only then overwrite it.
	 */
	if (pm_power_off == wbec_pm_power_off) {
		pm_power_off = NULL;
		unregister_restart_handler(&wbec_restart_handler);
	}

	return 0;
}

static const struct of_device_id wbec_of_match[] = {
	{ .compatible = "wirenboard,wbec" },
	{}
};
MODULE_DEVICE_TABLE(of, wbec_of_match);

static struct spi_driver wbec_driver = {
	.driver = {
		.name = "wbec",
		.of_match_table = wbec_of_match,
	},
	.probe = wbec_probe,
	.remove = wbec_remove,
};


module_spi_driver(wbec_driver);

MODULE_AUTHOR("Pavel Gasheev <pavel.gasheev@wirenboard.ru>");
MODULE_DESCRIPTION("Wiren Board 7 Embedded Controller MFD driver");
MODULE_LICENSE("GPL");
