// SPDX-License-Identifier: GPL-2.0-only
/*
 * led-wbec.c - Wiren Board Embedded Controller LED driver
 *
 * Copyright (c) 2024 Wiren Board LLC
 *
 * Author: Vitalii Gaponov <vg@wirenboard.com>
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/leds.h>
#include <linux/regmap.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/mfd/wbec.h>



struct wbec_led {
    struct led_classdev cdev;
    struct regmap *regmap;
};

static void wbec_led_set(struct led_classdev *led_cdev,
                         enum led_brightness value)
{
    struct wbec_led *wbec_led = container_of(led_cdev, struct wbec_led, cdev);
    regmap_write(wbec_led->regmap, WBEC_REG_LED_CTRL, value ? WBEC_REG_LED_CTRL_STATE_MSK : 0);
}

static int wbec_led_probe(struct platform_device *pdev)
{
    struct wbec *wbec = dev_get_drvdata(pdev->dev.parent);
    struct wbec_led *wbec_led;
	int err;

    if (!pdev->dev.parent)
        return -ENODEV;

    wbec_led = devm_kzalloc(&pdev->dev, sizeof(struct wbec_led), GFP_KERNEL);
    if (!wbec_led)
        return -ENOMEM;

    platform_set_drvdata(pdev, wbec_led);
    wbec_led->regmap = wbec->regmap;
    wbec_led->cdev.name = "wbec-led";
    wbec_led->cdev.brightness_set = wbec_led_set;

    err = devm_led_classdev_register(&pdev->dev, &wbec_led->cdev);
	if (err) {
		dev_err(&pdev->dev, "Couldn't register LED: %d", err);
		return err;
	}

    dev_info(&pdev->dev, "WBEC LED device initialized successfully");

    return 0;
}

static const struct of_device_id wbec_led_of_match[] = {
	{ .compatible = "wirenboard,wbec-led" },
	{}
};
MODULE_DEVICE_TABLE(of, wbec_led_of_match);

static struct platform_driver wbec_led_driver = {
	.driver		= {
		.name	= "wbec-led",
		.of_match_table = wbec_led_of_match,
	},
	.probe	= wbec_led_probe,
};
module_platform_driver(wbec_led_driver);

MODULE_AUTHOR("Vitalii Gaponov <vg@wirenboard.com>");
MODULE_DESCRIPTION("Wiren Board 7 Embedded Controller LED driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:wbec-led");
