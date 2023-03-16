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
#include <linux/mfd/wbec.h>

static int wbec_gpio_probe(struct platform_device *pdev)
{
	struct gpio_regmap_config config = {};
	struct wbec *wbec;
	u32 ngpios;
	int ret;

	if (!pdev->dev.parent)
		return -ENODEV;

	wbec = dev_get_drvdata(pdev->dev.parent);

	ret = device_property_read_u32(pdev->dev.parent, "ngpios", &ngpios);
	if (ret) {
		dev_err(&pdev->dev, "No ngpios property in device tree");
		return -EINVAL;
	}

	config.ngpio = ngpios;
	config.regmap = wbec->regmap;
	config.parent = pdev->dev.parent;
	config.reg_dat_base = WBEC_REG_GPIO;
	config.reg_set_base = WBEC_REG_GPIO;
	config.ngpio_per_reg = 16;
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
