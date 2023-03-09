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
#include <linux/gpio/regmap.h>
#include "wbec.h"

static int wbec_gpio_probe(struct platform_device *pdev)
{
	struct gpio_regmap_config config = {};
	struct wbec *wbec;

	if (!pdev->dev.parent)
		return -ENODEV;

	wbec = dev_get_drvdata(pdev->dev.parent);

	config.regmap = wbec->regmap_8;
	config.parent = &pdev->dev;
	config.reg_dat_base = WBEC_REG_GPIO;
	config.reg_set_base = WBEC_REG_GPIO;
	config.label = "wbec-gpio";
	config.ngpio_per_reg = 8;
	config.reg_stride = 1;

	return PTR_ERR_OR_ZERO(devm_gpio_regmap_register(&pdev->dev, &config));
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
