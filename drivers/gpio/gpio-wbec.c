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
	struct regmap *regmap;
	struct device *dev;
	struct gpio_chip gpio_chip;
};

static inline unsigned int offset_to_reg_addr(unsigned int offset)
{
	return WBEC_REG_GPIO + (offset / WBEC_GPIO_PER_REG);
}

static inline unsigned int offset_to_reg_mask(unsigned int offset)
{
	return BIT(offset % WBEC_GPIO_PER_REG);
}

static int wbec_gpio_get(struct gpio_chip *chip, unsigned int offset)
{
	struct wbec_gpio *wbec_gpio = gpiochip_get_data(chip);
	int ret, val, reg, mask;

	reg = offset_to_reg_addr(offset);
	mask = offset_to_reg_mask(offset);

	ret = regmap_read(wbec_gpio->regmap, reg, &val);
	if (ret < 0) {
		dev_err(wbec_gpio->dev, "error when reading gpio with offset %u\n", offset);
		return ret;
	}

	return (val & mask) ? 1 : 0;
}

static void wbec_gpio_set(struct gpio_chip *chip,
			   unsigned int offset,
			   int value)
{
	struct wbec_gpio *wbec_gpio = gpiochip_get_data(chip);
	int ret, reg, mask;

	reg = offset_to_reg_addr(offset);
	mask = offset_to_reg_mask(offset);

	ret = regmap_update_bits(wbec_gpio->regmap, reg, mask, value ? mask : 0);
	if (ret < 0)
		dev_err(wbec_gpio->dev, "error when set gpio with offset %u\n", offset);
}

static int wbec_gpio_probe(struct platform_device *pdev)
{
	struct wbec *wbec = dev_get_drvdata(pdev->dev.parent);
	struct wbec_gpio *wbec_gpio;
	struct device *dev = &pdev->dev;
	int ret, ngpios, base;

	wbec_gpio = devm_kzalloc(&pdev->dev, sizeof(*wbec_gpio), GFP_KERNEL);
	if (!wbec_gpio)
		return -ENOMEM;

	platform_set_drvdata(pdev, wbec_gpio);

	wbec_gpio->dev = dev;
	wbec_gpio->regmap = wbec->regmap;

	ret = device_property_read_u32(dev, "ngpios", &ngpios);
	if (ret) {
		dev_err(dev, "No ngpios property in device tree");
		return -EINVAL;
	}

	ret = device_property_read_u32(dev, "linux,gpio-base", &base);
	if (ret) {
		dev_warn(dev, "No linux,gpio-base property in device tree. Using -1 as gpio_base");
		base = -1;
	}

	/* Register gpio controller */
	wbec_gpio->gpio_chip.label = dev_name(wbec_gpio->dev);
	wbec_gpio->gpio_chip.owner = THIS_MODULE;
	wbec_gpio->gpio_chip.get = wbec_gpio_get;
	wbec_gpio->gpio_chip.set = wbec_gpio_set;
	wbec_gpio->gpio_chip.can_sleep = true;
	wbec_gpio->gpio_chip.base = base;
	wbec_gpio->gpio_chip.ngpio = ngpios;
	wbec_gpio->gpio_chip.parent = dev;
	wbec_gpio->gpio_chip.of_node = dev->of_node;

	/* Add gpio chip */
	ret = devm_gpiochip_add_data(dev, &wbec_gpio->gpio_chip, wbec_gpio);
	if (ret < 0) {
		dev_err(dev, "Couldn't add gpiochip\n");
		return ret;
	}

	return 0;
}

static const struct of_device_id wbec_gpio_of_match[] = {
	{ .compatible = "wirenboard,wbec-gpio" },
	{}
};
MODULE_DEVICE_TABLE(of, wbec_gpio_of_match);

static struct platform_driver wbec_gpio_driver = {
	.driver = {
		.name = "wbec-gpio",
		.of_match_table = wbec_gpio_of_match,
	},
	.probe = wbec_gpio_probe,
};
module_platform_driver(wbec_gpio_driver);

MODULE_AUTHOR("Pavel Gasheev <pavel.gasheev@wirenboard.ru>");
MODULE_DESCRIPTION("Wiren Board 7 Embedded Controller GPIO driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:wbec-gpio");
