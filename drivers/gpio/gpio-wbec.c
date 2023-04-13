// SPDX-License-Identifier: GPL-2.0-only
/*
 * gpio-wbec.c - Wiren Board Embedded Controller GPIO driver
 *
 * Copyright (c) 2023 Wiren Board LLC
 *
 * Author: Pavel Gasheev <pavel.gasheev@wirenboard.ru>
 */

#include <linux/gpio/driver.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/pm.h>
#include <linux/property.h>
#include <linux/slab.h>
#include <linux/regmap.h>
#include <linux/gpio/regmap.h>
#include <linux/mfd/wbec.h>

#define WBEC_GPIO_PER_REG		16

struct wbec_gpio {
	int gpio_per_reg;
	struct regmap *regmap;
	struct device *dev;
	struct gpio_chip gpio_chip;
};


/* generic gpio chip */
static int wbec_gpio_get(struct gpio_chip *chip, unsigned int offset)
{
	struct wbec_gpio *wbec_gpio = gpiochip_get_data(chip);
	int ret, val, reg, mask;

	reg = WBEC_REG_GPIO + (offset / wbec_gpio->gpio_per_reg);
	mask = BIT(offset % wbec_gpio->gpio_per_reg);

	dev_dbg(wbec_gpio->dev, "%s function. offset=%u\n", __func__, offset);

	ret = regmap_read(wbec_gpio->regmap, reg, &val);
	if (ret < 0) {
		dev_err(wbec_gpio->dev, "error when read gpio %u\n", offset);
		return ret;
	}

	return !!(val & mask);
}

static void wbec_gpio_set(struct gpio_chip *chip,
			   unsigned int offset,
			   int value)
{
	struct wbec_gpio *wbec_gpio = gpiochip_get_data(chip);
	int ret, reg, mask;

	reg = WBEC_REG_GPIO + (offset / wbec_gpio->gpio_per_reg);
	mask = BIT(offset % wbec_gpio->gpio_per_reg);

	dev_dbg(wbec_gpio->dev, "%s function. offset=%u, value=%d\n", __func__, offset, value);

	ret = regmap_update_bits(wbec_gpio->regmap, reg, mask, value ? mask : 0);
	if (ret < 0)
		dev_err(wbec_gpio->dev, "error when set gpio %u\n", offset);
}

static int wbec_gpio_probe(struct platform_device *pdev)
{
	struct wbec *wbec = dev_get_drvdata(pdev->dev.parent);
	struct wbec_gpio *wbec_gpio;
	int ret, ngpios, base;

	wbec_gpio = devm_kzalloc(&pdev->dev, sizeof(*wbec_gpio), GFP_KERNEL);
	if (!wbec_gpio)
		return -ENOMEM;

	platform_set_drvdata(pdev, wbec_gpio);

	wbec_gpio->dev = &pdev->dev;
	wbec_gpio->regmap = wbec->regmap;

	ret = device_property_read_u32(&pdev->dev, "ngpios", &ngpios);
	if (ret) {
		dev_err(&pdev->dev, "No ngpios property in device tree");
		return -EINVAL;
	}

	ret = device_property_read_u32(&pdev->dev, "linux,base", &base);
	if (ret) {
		dev_warn(&pdev->dev, "No linux,base property in device tree. Using -1 as gpio_base");
		base = -1;
	}

	ret = device_property_read_u32(&pdev->dev, "gpio-per-reg", &wbec_gpio->gpio_per_reg);
	if (ret) {
		dev_dbg(&pdev->dev, "No gpio-per-reg property in device tree. Using %d", WBEC_GPIO_PER_REG);
		wbec_gpio->gpio_per_reg = WBEC_GPIO_PER_REG;
	}

	/* Register gpio controller */
	wbec_gpio->gpio_chip.label = dev_name(wbec_gpio->dev);
	wbec_gpio->gpio_chip.owner = THIS_MODULE;
	wbec_gpio->gpio_chip.get = wbec_gpio_get;
	wbec_gpio->gpio_chip.set = wbec_gpio_set;
	wbec_gpio->gpio_chip.can_sleep = true;
	wbec_gpio->gpio_chip.base = base;
	wbec_gpio->gpio_chip.ngpio = ngpios;
	wbec_gpio->gpio_chip.parent = &pdev->dev;
	wbec_gpio->gpio_chip.of_node = pdev->dev.of_node;

	/* Add gpio chip */
	ret = devm_gpiochip_add_data(&pdev->dev, &wbec_gpio->gpio_chip, wbec_gpio);
	if (ret < 0) {
		dev_err(&pdev->dev, "Couldn't add gpiochip\n");
		return ret;
	}

	return 0;
}

static struct platform_driver wbec_gpio_driver = {
	.driver = {
		.name = "wbec-gpio",
	},
	.probe = wbec_gpio_probe,
};
module_platform_driver(wbec_gpio_driver);

MODULE_AUTHOR("Pavel Gasheev <pavel.gasheev@wirenboard.ru>");
MODULE_DESCRIPTION("Wiren Board 7 Embedded Controller GPIO driver");
MODULE_LICENSE("GPL");
