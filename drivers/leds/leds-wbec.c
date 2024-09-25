// SPDX-License-Identifier: GPL-2.0-only
/*
 * led-wbec.c - Wiren Board Embedded Controller System LED driver
 *
 * Copyright (c) 2024 Wiren Board LLC
 *
 * Author: Ilia Skochilov <ilia.skochilov@wirenboard.com>
 */

#include <linux/errno.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/leds.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>
#include <linux/mfd/wbec.h>

enum led_mode {
    WBEC_LED_OFF,
    WBEC_LED_ON,
    WBEC_LED_BLINK,
};

struct wbec_led {
	struct regmap *regmap;
	struct led_classdev cdev;
	unsigned long blink_delay_on;
	unsigned long blink_delay_off;
};

static void wbec_led_set(struct led_classdev *led_cdev, enum led_brightness brightness)
{
    struct wbec_led *wbec_led = container_of(led_cdev, struct wbec_led, cdev);
	int value;
	if (brightness > 0) {
		value = WBEC_LED_ON;
	} else {
		value = WBEC_LED_OFF;
	}
    regmap_write(wbec_led->regmap, WBEC_REG_EC_SYSTEM_LED, value);
}

static int wbec_blink_set(struct led_classdev *led_cdev,
                          unsigned long *delay_on,
                          unsigned long *delay_off)`
{
    struct wbec_led *wbec_led = container_of(led_cdev, struct wbec_led, cdev);
    int err;

    err = regmap_write(wbec_led->regmap, WBEC_REG_EC_SYSTEM_LED, WBEC_LED_BLINK);
    if (err)
        return err;

    err = regmap_write(wbec_led->regmap, WBEC_REG_EC_SYSTEM_LED_ON_MS, *delay_on);
    if (err)
        return err;

    err = regmap_write(wbec_led->regmap, WBEC_REG_EC_SYSTEM_LED_OFF_MS, *delay_off);
    if (err)
        return err;

	wbec_led->blink_delay_on = *delay_on;
	wbec_led->blink_delay_off = *delay_off;

    return 0;
}

static ssize_t blink_on_show(struct device *dev,
                             struct device_attribute *attr, char *buf)
{
    struct led_classdev *led_cdev = dev_get_drvdata(dev);
    struct wbec_led *wbec_led = container_of(led_cdev, struct wbec_led, cdev);

    return sprintf(buf, "%lu\n", wbec_led->blink_delay_on);
}

static ssize_t blink_on_store(struct device *dev,
                              struct device_attribute *attr,
                              const char *buf, size_t size)
{
    struct led_classdev *led_cdev = dev_get_drvdata(dev);
    struct wbec_led *wbec_led = container_of(led_cdev, struct wbec_led, cdev);
    unsigned long value;
    int ret;

    ret = kstrtoul(buf, 10, &value);
    if (ret)
        return ret;

    wbec_led->blink_delay_on = value;
    wbec_blink_set(led_cdev, &wbec_led->blink_delay_on, &wbec_led->blink_delay_off);

    return size;
}

static ssize_t blink_off_show(struct device *dev,
                              struct device_attribute *attr, char *buf)
{
    struct led_classdev *led_cdev = dev_get_drvdata(dev);
    struct wbec_led *wbec_led = container_of(led_cdev, struct wbec_led, cdev);

    return sprintf(buf, "%lu\n", wbec_led->blink_delay_off);
}

static ssize_t blink_off_store(struct device *dev,
                               struct device_attribute *attr,
                               const char *buf, size_t size)
{
    struct led_classdev *led_cdev = dev_get_drvdata(dev);
    struct wbec_led *wbec_led = container_of(led_cdev, struct wbec_led, cdev);
    unsigned long value;
    int ret;

    ret = kstrtoul(buf, 10, &value);
    if (ret)
        return ret;

    wbec_led->blink_delay_off = value;
    wbec_blink_set(led_cdev, &wbec_led->blink_delay_on, &wbec_led->blink_delay_off);

    return size;
}

static DEVICE_ATTR_RW(blink_on);
static DEVICE_ATTR_RW(blink_off);

static struct attribute *wbec_led_attrs[] = {
    &dev_attr_blink_on.attr,
    &dev_attr_blink_off.attr,
    NULL
};

ATTRIBUTE_GROUPS(wbec_led);

static int wbec_led_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct wbec *wbec = dev_get_drvdata(pdev->dev.parent);
	struct wbec_led *wbec_led;
	int err;

	if (!pdev->dev.parent)
		return -ENODEV;

	wbec_led = devm_kzalloc(dev, sizeof(struct wbec_led), GFP_KERNEL);
	if (!wbec_led)
		return -ENOMEM;

	wbec_led->regmap = wbec->regmap;
    if (!wbec_led->regmap) {
        return -ENODEV;
    }

    if (!of_device_is_compatible(dev->of_node, "wirenboard,wbec-led")) {
        dev_err(dev, "Incompatible device\n");
        return -EINVAL;
    }

	wbec_led->cdev.name = "wbec_led";
	wbec_led->cdev.brightness_set = wbec_led_set;
	wbec_led->cdev.blink_set = wbec_blink_set;
	wbec_led->cdev.groups = wbec_led_groups;

	err = led_classdev_register(dev, &wbec_led->cdev);
    if (err) {
        dev_err(dev, "Failed to register System LED device\n");
        return err;
    }

	platform_set_drvdata(pdev, wbec_led);

	dev_info(dev, "System LED device probed successfully\n");
	return 0;
}

static int wbec_led_remove(struct platform_device *pdev)
{
    struct wbec_led *wbec_led = platform_get_drvdata(pdev);

    led_classdev_unregister(&wbec_led->cdev);

    dev_info(&pdev->dev, "System LED device removed\n");

    return 0;
}

static const struct of_device_id wbec_led_of_match[] = {
	{ .compatible = "wirenboard,wbec-led" },
	{ }
};
MODULE_DEVICE_TABLE(of, wbec_led_of_match);

static struct platform_driver wbec_led_driver = {
	.driver	= {
		.name = "wbec-led",
		.of_match_table = wbec_led_of_match,
	},
	.probe	= wbec_led_probe,
	.remove = wbec_led_remove,
};
module_platform_driver(wbec_led_driver);

MODULE_ALIAS("platform:wbec-led");
MODULE_AUTHOR("Ilia Skochilov <ilia.skochilov@wirenboard.com>");
MODULE_DESCRIPTION("Wiren Board Embedded Controller System LED driver");
MODULE_LICENSE("GPL");
