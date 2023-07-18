// SPDX-License-Identifier: GPL-2.0-or-later
/*
 *  Copyright (C) 2010, Lars-Peter Clausen <lars@metafoo.de>
 *  Driver for chargers which report their online status through a GPIO pin
 */

#include <linux/device.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/power_supply.h>
#include <linux/slab.h>
#include <linux/of.h>
#include <linux/regmap.h>

#include <linux/mfd/wbec.h>

struct wbec_charger {
	struct regmap *regmap;

	struct power_supply *charger;
	struct power_supply_desc charger_desc;
};

static inline struct wbec_charger *psy_to_wbec_charger(struct power_supply *psy)
{
	return power_supply_get_drvdata(psy);
}

static int wbec_charger_get_property(struct power_supply *psy,
		enum power_supply_property psp, union power_supply_propval *val)
{
	struct wbec_charger *wbec_charger = psy_to_wbec_charger(psy);
	int ret;

	if (psp == POWER_SUPPLY_PROP_ONLINE) {
		// TODO regmap
		ret = regmap_test_bits(wbec_charger->regmap, WBEC_REG_GPIO, BIT(5));
		if (ret < 0)
			return ret;
		/* If bit is set, it means that WB is powered from battery and main power is off */
		val->intval = ret ? 0 : 1;
	} else {
		return -EINVAL;
	}

	return 0;
}

static int wbec_charger_set_property(struct power_supply *psy,
	enum power_supply_property psp, const union power_supply_propval *val)
{
	return -EINVAL;
}

static int wbec_charger_property_is_writeable(struct power_supply *psy,
					      enum power_supply_property psp)
{
	return 0;
}

static enum power_supply_type gpio_charger_get_type(struct device *dev)
{
	return POWER_SUPPLY_TYPE_MAINS;
}

/*
 * The entries will be overwritten by driver's probe routine depending
 * on the available features. This list ensures, that the array is big
 * enough for all optional features.
 */
static enum power_supply_property gpio_charger_properties[] = {
	POWER_SUPPLY_PROP_ONLINE,
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT_MAX,
};

static int gpio_charger_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct wbec *wbec;
	const struct gpio_charger_platform_data *pdata = dev->platform_data;
	struct power_supply_config psy_cfg = {};
	struct power_supply_desc *charger_desc;
	struct wbec_charger *wbec_charger;
	int charge_status_irq;
	int ret;
	int num_props = 0;

	if (!dev->parent)
		return -ENODEV;

	wbec_charger = devm_kzalloc(dev, sizeof(*wbec_charger), GFP_KERNEL);
	if (!wbec_charger)
		return -ENOMEM;

	wbec = dev_get_drvdata(dev->parent);
	wbec_charger->regmap = wbec->regmap;

	gpio_charger_properties[num_props] = POWER_SUPPLY_PROP_ONLINE;
	num_props++;

	charger_desc = &wbec_charger->charger_desc;
	charger_desc->properties = gpio_charger_properties;
	charger_desc->num_properties = num_props;
	charger_desc->get_property = wbec_charger_get_property;
	charger_desc->set_property = wbec_charger_set_property;
	charger_desc->property_is_writeable =
					wbec_charger_property_is_writeable;

	psy_cfg.of_node = dev->of_node;
	psy_cfg.drv_data = wbec_charger;

	// if (pdata) {
	// 	charger_desc->name = pdata->name;
	// 	charger_desc->type = pdata->type;
	// 	psy_cfg.supplied_to = pdata->supplied_to;
	// 	psy_cfg.num_supplicants = pdata->num_supplicants;
	// } else {
	// 	charger_desc->name = dev->of_node->name;
	// 	charger_desc->type = gpio_charger_get_type(dev);
	// }
	charger_desc->type = POWER_SUPPLY_TYPE_MAINS;

	if (!charger_desc->name)
		charger_desc->name = pdev->name;

	wbec_charger->charger = devm_power_supply_register(dev, charger_desc,
							   &psy_cfg);
	if (IS_ERR(wbec_charger->charger)) {
		ret = PTR_ERR(wbec_charger->charger);
		dev_err(dev, "Failed to register power supply: %d\n", ret);
		return ret;
	}

	// gpio_charger->irq = gpio_charger_get_irq(dev, gpio_charger->charger,
	// 					 gpio_charger->gpiod);

	// charge_status_irq = gpio_charger_get_irq(dev, gpio_charger->charger,
	// 					 gpio_charger->charge_status);
	// gpio_charger->charge_status_irq = charge_status_irq;

	platform_set_drvdata(pdev, wbec_charger);

	return 0;
}

static const struct of_device_id gpio_charger_match[] = {
	{ .compatible = "wbec-charger" },
	{ }
};
MODULE_DEVICE_TABLE(of, gpio_charger_match);

static struct platform_driver gpio_charger_driver = {
	.probe = gpio_charger_probe,
	.driver = {
		.name = "wbec-charger",
		.of_match_table = gpio_charger_match,
	},
};

module_platform_driver(gpio_charger_driver);

MODULE_AUTHOR("Lars-Peter Clausen <lars@metafoo.de>");
MODULE_DESCRIPTION("Driver for chargers only communicating via GPIO(s)");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:wbec-charger");
