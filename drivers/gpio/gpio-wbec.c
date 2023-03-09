// SPDX-License-Identifier: GPL-2.0-only
/*
 * TODO
 * Write description
 */

#include <linux/gpio/driver.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/pm.h>
#include <linux/property.h>
#include <linux/slab.h>
#include <linux/regmap.h>
#include "wbec.h"

enum {
	WBEC_GPIO_A1,
	WBEC_GPIO_A2,
	WBEC_GPIO_A3,
	WBEC_GPIO_A4,
	WBEC_GPIO_V_OUT,

	WBEC_GPIO_COUNT
};

struct wbec_gpio_info {
	struct wbec *wbec;
	struct regmap *regmap;
	struct device *dev;
	struct gpio_chip gpio_chip;
};


/* generic gpio chip */
static int wbec_gpio_get(struct gpio_chip *chip, unsigned int offset)
{
	struct wbec_gpio_info *pci = gpiochip_get_data(chip);
	int ret, val;

    // TODO Remove debug
	dev_info(pci->dev, "%s function. offset=%u\n", __func__, offset);

	ret = regmap_read(pci->regmap, WBEC_REG_GPIO, &val);
	if (ret < 0) {
		dev_err(pci->dev, "error when read gpio %u\n", offset);
		return ret;
	}

	return !!(val & BIT(offset));
}

static void wbec_gpio_set(struct gpio_chip *chip,
			   unsigned int offset,
			   int value)
{
	struct wbec_gpio_info *pci = gpiochip_get_data(chip);
	int ret;

	// TODO Remove debug
	dev_info(pci->dev, "%s function. offset=%u, value=%d\n", __func__, offset, value);

	ret = regmap_update_bits(pci->regmap, WBEC_REG_GPIO, BIT(offset), value ? BIT(offset) : 0);
	if (ret < 0)
		dev_err(pci->dev, "error when set gpio %u\n", offset);
}

static int wbec_gpio_get_direction(struct gpio_chip *chip, unsigned int offset)
{
	switch (offset) {
	case WBEC_GPIO_V_OUT:
		return GPIO_LINE_DIRECTION_OUT;

	case WBEC_GPIO_A1:
	case WBEC_GPIO_A2:
	case WBEC_GPIO_A3:
	case WBEC_GPIO_A4:
		return GPIO_LINE_DIRECTION_IN;
	}

	return -ENOTSUPP;
}

static const struct gpio_chip wbec_gpio_chip = {
	.label = "wbec-gpio",
	.owner = THIS_MODULE,
	.get = wbec_gpio_get,
	.set = wbec_gpio_set,
	.get_direction = wbec_gpio_get_direction,
	.can_sleep = true,
	.base = 0x400,	// TODO must be -1
	.ngpio = WBEC_GPIO_COUNT,
};

static int wbec_gpio_probe(struct platform_device *pdev)
{
	struct wbec_gpio_info *gpio;
	struct wbec *wbec;
	int ret;

	gpio = devm_kzalloc(&pdev->dev, sizeof(*gpio), GFP_KERNEL);
	if (!gpio)
		return -ENOMEM;

	platform_set_drvdata(pdev, gpio);

	wbec = dev_get_drvdata(pdev->dev.parent);
	gpio->dev = &pdev->dev;
	gpio->wbec = wbec;
	gpio->regmap = wbec->regmap_8;

	/* Register gpio controller */
	gpio->gpio_chip = wbec_gpio_chip;
	gpio->gpio_chip.parent = &pdev->dev;
	gpio->gpio_chip.of_node = pdev->dev.parent->of_node;

	/* Add gpio chip */
	ret = devm_gpiochip_add_data(&pdev->dev, &gpio->gpio_chip, gpio);
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
