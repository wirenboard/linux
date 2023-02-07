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

#include <linux/pinctrl/consumer.h>
#include <linux/pinctrl/machine.h>
#include <linux/pinctrl/pinctrl.h>
#include <linux/pinctrl/pinconf-generic.h>
#include <linux/pinctrl/pinconf.h>
#include <linux/pinctrl/pinmux.h>

#include "wbec.h"

#include "../linux/drivers/pinctrl/pinctrl-utils.h"

struct wbec_pin_function {
	const char *name;
	const char *const *groups;
	unsigned int ngroups;
	int mux_option;
};

struct wbec_pin_group {
	const char *name;
	const unsigned int pins[1];
	unsigned int npins;
};

/*
 * @reg: gpio setting register;
 * @fun_mask: functions select mask value, when set is gpio;
 * @dir_mask: input or output mask value, when set is output, otherwise input;
 * @val_mask: gpio set value, when set is level high, otherwise low;
 */
struct wbec_pin_config {
	u8 reg;
	u8 fun_msk;
	u8 dir_msk;
	u8 val_msk;
};

struct wbec_pctrl_info {
	struct wbec *wbec;
	struct device *dev;
	struct pinctrl_dev *pctl;
	struct gpio_chip gpio_chip;
	struct pinctrl_desc pinctrl_desc;
	const struct wbec_pin_function *functions;
	unsigned int num_functions;
	const struct wbec_pin_group *groups;
	int num_pin_groups;
	const struct pinctrl_pin_desc *pins;
	unsigned int num_pins;
	const struct wbec_pin_config *pin_cfg;
};

enum {
	WBEC_GPIO_VOUT,
};

static const char *const wbec_gpio_groups[] = {
	"V_OUT",
};

static const struct pinctrl_pin_desc wbec_pins_desc[] = {
	PINCTRL_PIN(WBEC_GPIO_VOUT, "V_OUT"),
};

static const struct wbec_pin_function wbec_pin_functions[] = {
	{
		.name = "gpio",
		.groups = wbec_gpio_groups,
		.ngroups = ARRAY_SIZE(wbec_gpio_groups),
	},
};

static const struct wbec_pin_group wbec_pin_groups[] = {
	{
		.name = "V_OUT",
		.pins = { WBEC_GPIO_VOUT },
		.npins = 1,
	},
};

#define WBEC_GPIO_V_OUT_VAL_MSK	BIT(0)
// #define WBEC_GPIO1_VAL_MSK	BIT(1)

static const struct wbec_pin_config wbec_gpio_cfgs[] = {
	{
		.reg = WBEC_REG_GPIO,
		.val_msk = WBEC_GPIO_V_OUT_VAL_MSK,
	},
};

/* generic gpio chip */
static int wbec_gpio_get(struct gpio_chip *chip, unsigned int offset)
{
	struct wbec_pctrl_info *pci = gpiochip_get_data(chip);
	int ret, val;

    // TODO Remove debug
	dev_info(pci->dev, "%s function\n", __func__);

	ret = regmap_read(pci->wbec->regmap_8, pci->pin_cfg[offset].reg, &val);
	if (ret) {
		dev_err(pci->dev, "get gpio%d value failed\n", offset);
		return ret;
	}

	return !!(val & pci->pin_cfg[offset].val_msk);
}

static void wbec_gpio_set(struct gpio_chip *chip,
			   unsigned int offset,
			   int value)
{
	struct wbec_pctrl_info *pci = gpiochip_get_data(chip);
	int ret;

	ret = regmap_update_bits(pci->wbec->regmap_8,
				 pci->pin_cfg[offset].reg,
				 pci->pin_cfg[offset].val_msk,
				 value ? pci->pin_cfg[offset].val_msk : 0);
	if (ret)
		dev_err(pci->dev, "set gpio%d value %d failed\n",
			offset, value);
}

static int wbec_gpio_direction_input(struct gpio_chip *chip,
				      unsigned int offset)
{
	return pinctrl_gpio_direction_input(chip->base + offset);
}

static int wbec_gpio_direction_output(struct gpio_chip *chip,
				       unsigned int offset, int value)
{
	wbec_gpio_set(chip, offset, value);
	return pinctrl_gpio_direction_output(chip->base + offset);
}

static int wbec_gpio_get_direction(struct gpio_chip *chip, unsigned int offset)
{
	struct wbec_pctrl_info *pci = gpiochip_get_data(chip);
	unsigned int val;
	int ret;

	return GPIO_LINE_DIRECTION_OUT;
}

static const struct gpio_chip wbec_gpio_chip = {
	.label			= "wbec-gpio",
	.request		= gpiochip_generic_request,
	.free			= gpiochip_generic_free,
	.get_direction		= wbec_gpio_get_direction,
	.get			= wbec_gpio_get,
	.set			= wbec_gpio_set,
	.direction_input	= wbec_gpio_direction_input,
	.direction_output	= wbec_gpio_direction_output,
	.can_sleep		= true,
	.base			= 0x400,	// TODO must be -1
	.owner			= THIS_MODULE,
};

/* generic pinctrl */
static int wbec_pinctrl_get_groups_count(struct pinctrl_dev *pctldev)
{
	struct wbec_pctrl_info *pci = pinctrl_dev_get_drvdata(pctldev);

	return pci->num_pin_groups;
}

static const char *wbec_pinctrl_get_group_name(struct pinctrl_dev *pctldev,
						unsigned int group)
{
	struct wbec_pctrl_info *pci = pinctrl_dev_get_drvdata(pctldev);

	return pci->groups[group].name;
}

static int wbec_pinctrl_get_group_pins(struct pinctrl_dev *pctldev,
					unsigned int group,
					const unsigned int **pins,
					unsigned int *num_pins)
{
	struct wbec_pctrl_info *pci = pinctrl_dev_get_drvdata(pctldev);

	*pins = pci->groups[group].pins;
	*num_pins = pci->groups[group].npins;

	return 0;
}

static const struct pinctrl_ops wbec_pinctrl_ops = {
	.get_groups_count = wbec_pinctrl_get_groups_count,
	.get_group_name = wbec_pinctrl_get_group_name,
	.get_group_pins = wbec_pinctrl_get_group_pins,
	.dt_node_to_map = pinconf_generic_dt_node_to_map_pin,
	.dt_free_map = pinctrl_utils_free_map,
};

static int wbec_pinctrl_get_funcs_count(struct pinctrl_dev *pctldev)
{
	struct wbec_pctrl_info *pci = pinctrl_dev_get_drvdata(pctldev);

	return pci->num_functions;
}

static const char *wbec_pinctrl_get_func_name(struct pinctrl_dev *pctldev,
					       unsigned int function)
{
	struct wbec_pctrl_info *pci = pinctrl_dev_get_drvdata(pctldev);

	return pci->functions[function].name;
}

static int wbec_pinctrl_get_func_groups(struct pinctrl_dev *pctldev,
					 unsigned int function,
					 const char *const **groups,
					 unsigned int *const num_groups)
{
	struct wbec_pctrl_info *pci = pinctrl_dev_get_drvdata(pctldev);

	*groups = pci->functions[function].groups;
	*num_groups = pci->functions[function].ngroups;

	return 0;
}

static const struct pinctrl_desc wbec_pinctrl_desc = {
	.name = "wbec-pinctrl",
	.pctlops = &wbec_pinctrl_ops,
	.owner = THIS_MODULE,
};

static int wbec_pinctrl_probe(struct platform_device *pdev)
{
	struct wbec_pctrl_info *pci;
	int ret;

	pci = devm_kzalloc(&pdev->dev, sizeof(*pci), GFP_KERNEL);
	if (!pci)
		return -ENOMEM;

	platform_set_drvdata(pdev, pci);

	pci->dev = &pdev->dev;
	pci->wbec = dev_get_drvdata(pdev->dev.parent);

	/* Register pin controller */
	pci->pinctrl_desc = wbec_pinctrl_desc;
    pci->pinctrl_desc.pins = wbec_pins_desc;
    pci->pinctrl_desc.npins = ARRAY_SIZE(wbec_pins_desc);

	ret = devm_pinctrl_register_and_init(pci->dev, &pci->pinctrl_desc,
					     pci, &pci->pctl);
	if (ret) {
		dev_err(pci->dev, "pinctrl registration failed\n");
		return ret;
	}

	ret = pinctrl_enable(pci->pctl);
	if (ret) {
		dev_err(pci->dev, "pinctrl enable failed\n");
		return ret;
	}

	/* Register gpio controller */
	pci->gpio_chip = wbec_gpio_chip;
	pci->gpio_chip.parent = &pdev->dev;
	pci->gpio_chip.of_node = pdev->dev.parent->of_node;
    pci->gpio_chip.ngpio = ARRAY_SIZE(wbec_gpio_cfgs);

	/* Add gpio chip */
	ret = devm_gpiochip_add_data(&pdev->dev, &pci->gpio_chip, pci);
	if (ret < 0) {
		dev_err(&pdev->dev, "Couldn't add gpiochip\n");
		return ret;
	}

    pci->pins = wbec_pins_desc;
    pci->num_pins = ARRAY_SIZE(wbec_pins_desc);
    pci->functions = wbec_pin_functions;
    pci->num_functions = ARRAY_SIZE(wbec_pin_functions);
    pci->groups = wbec_pin_groups;
    pci->num_pin_groups = ARRAY_SIZE(wbec_pin_groups);
    pci->pin_cfg = wbec_gpio_cfgs;

	return 0;

	/* Add pin range */
	ret = gpiochip_add_pin_range(&pci->gpio_chip, dev_name(&pdev->dev),
				     0, 0, pci->gpio_chip.ngpio);
	if (ret < 0) {
		dev_err(&pdev->dev, "Couldn't add gpiochip pin range\n");
		return ret;
	}

	return 0;
}

static struct platform_driver wbec_pinctrl_driver = {
	.driver = {
		.name = "wbec-pinctrl",
	},
	.probe = wbec_pinctrl_probe,
};
module_platform_driver(wbec_pinctrl_driver);

MODULE_AUTHOR("Pavel Gasheev <pavel.gasheev@wirenboard.ru>");
MODULE_DESCRIPTION("Wiren Board 7 Embedded Controller GPIO driver");
MODULE_LICENSE("GPL");