// SPDX-License-Identifier: GPL-2.0+
// Copyright (c) 2024 Anton Tarasov <anton.tarasov@wirenboard.com>
//
// Allwinner H616 MMC Regulator Driver
#include <linux/io.h>
#include <linux/module.h>
#include <linux/regulator/driver.h>
#include <linux/regulator/machine.h>
#include <linux/regulator/of_regulator.h>
#include <linux/platform_device.h>
#include <linux/io.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/regmap.h>
#include <linux/mfd/syscon.h>
#include <linux/mod_devicetable.h>
#include <linux/of_address.h>

#define VSEL_REG   0x0350  // Voltage selection register
#define VSEL_MASK  0x1     // Voltage selection mask (bit 0)

#define MOD_SEL_REG   0x0340  // Withstand voltage selection register
#define MOD_SEL_MASK  0x20    // Withstand voltage selection mask (bit 5)

static const unsigned int h616_regulator_voltages[] = {
	1800000, // 1.8V
	3300000, // 3.3V
};

static int h616_regulator_set_voltage(struct regulator_dev *rdev,
                                      int min_uV, int max_uV,
                                      unsigned int *selector)
{
    struct regmap *regmap = rdev_get_regmap(rdev);
    int ret;

    *selector = regulator_map_voltage_iterate(rdev, min_uV, max_uV);
    if (*selector < 0)
        return -EINVAL;

    // Set the withstand voltage level
    ret = regmap_update_bits(regmap, MOD_SEL_REG, MOD_SEL_MASK, 
            (*selector == 0) ? MOD_SEL_MASK : 0);
    if (ret)
        return ret;
    
    // Update the main voltage level
    ret = regmap_update_bits(regmap, VSEL_REG, VSEL_MASK, *selector);
    return ret;

    return 0;
}

static const struct regulator_ops h616_regulator_ops = {
	.list_voltage = regulator_list_voltage_table,
	.map_voltage = regulator_map_voltage_iterate,
	.get_voltage_sel = regulator_get_voltage_sel_regmap,
    .set_voltage = h616_regulator_set_voltage,
};

static const struct regulator_desc h616_regulator_desc = {
    .name = "h616-mmc-vqmmc",
    .of_match = "allwinner,h616-mmc-regulator",
	.type = REGULATOR_VOLTAGE,
	.owner = THIS_MODULE,
    .ops = &h616_regulator_ops,
	.n_voltages = 2,
	.volt_table = h616_regulator_voltages,
	.vsel_reg = VSEL_REG,
	.vsel_mask = VSEL_MASK,
};

static int h616_regulator_probe(struct platform_device *pdev)
{

    struct regulator_config config = {};
    struct regulator_desc *desc;
    struct regmap *regmap;

    regmap = syscon_regmap_lookup_by_phandle(pdev->dev.of_node, "syscon");
    if (IS_ERR(regmap))
        return PTR_ERR(regmap);

    desc = devm_kmemdup(&pdev->dev, &h616_regulator_desc, sizeof(*desc), GFP_KERNEL);
    if (!desc)
        return -ENOMEM;

    config.dev = &pdev->dev;
    config.of_node = pdev->dev.of_node;
    config.regmap = regmap;

    config.init_data = of_get_regulator_init_data(&pdev->dev, 
            pdev->dev.of_node, &h616_regulator_desc);
    if (!config.init_data) 
        return -ENOMEM;

    if (IS_ERR(devm_regulator_register(&pdev->dev, desc, &config)))
        return PTR_ERR(devm_regulator_register(&pdev->dev, desc, &config));

    return 0;
}

static const struct of_device_id h616_regulator_of_match[] = {
    { .compatible = "allwinner,h616-mmc-regulator", },
	{},
};
MODULE_DEVICE_TABLE(of, h616_regulator_of_match);

static struct platform_driver h616_regulator_driver = {
    .probe = h616_regulator_probe,
    .driver = {
        .name = "h616-mmc-regulator",
        .of_match_table = h616_regulator_of_match,
    },
};

module_platform_driver(h616_regulator_driver);

MODULE_AUTHOR("Anton Tarasov <anton.tarasov@wirenboard.com>");
MODULE_DESCRIPTION("H616 MMC Regulator Driver");
MODULE_LICENSE("GPL");

