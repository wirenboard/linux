// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) STMicroelectronics 2016
 *
 * Author: Gerald Baeza <gerald.baeza@st.com>
 *
 * Inspired by timer-stm32.c from Maxime Coquelin
 *             pwm-atmel.c from Bo Shen
 */

#include <linux/bitfield.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/pinctrl/consumer.h>
#include <linux/platform_device.h>
#include <linux/pwm.h>
#include <linux/mxc_icap.h>
#include <linux/interrupt.h>

#define CCMR_CHANNEL_SHIFT 8
#define CCMR_CHANNEL_MASK  0xFF
#define MAX_BREAKINPUT 2

struct stm32_breakinput {
	u32 index;
	u32 level;
	u32 filter;
};

struct imx_gpt_pwm {
	struct pwm_chip chip;
	struct mutex lock; /* protect pwm config/enable */
	struct clk *clk;
	struct regmap *regmap;
	u32 max_arr;
	u32 capture[4] ____cacheline_aligned; /* DMA'able buffer */
};

static inline struct imx_gpt_pwm *to_pwm_imx_gpt_dev(struct pwm_chip *chip)
{
	return container_of(chip, struct imx_gpt_pwm, chip);
}

static int pwm_imx_gpt_capture(struct pwm_chip *chip, struct pwm_device *pwm,
			     struct pwm_capture *result, unsigned long tmo_ms)
{
	struct imx_gpt_pwm *priv = to_pwm_imx_gpt_dev(chip);
	unsigned long long prd, div, dty;
	unsigned long rate;
	unsigned int psc = 0, icpsc, scale;
	u32 raw_prd = 0, raw_dty = 0;
	int ret = 0;

    result->period = 123;
    result->duty_cycle = 15;

	return ret;
}

static int pwm_imx_gpt_apply_locked(struct pwm_chip *chip, struct pwm_device *pwm,
				  struct pwm_state *state)
{
	// struct imx_gpt_pwm *priv = to_pwm_imx_gpt_dev(chip);
	int ret = 0;

	// /* protect common prescaler for all active channels */
	// mutex_lock(&priv->lock);
	// ret = pwm_imx_gpt_apply(chip, pwm, state);
	// mutex_unlock(&priv->lock);

	return ret;
}

static const struct pwm_ops imxgptpwm_opts = {
	.owner = THIS_MODULE,
	.apply = pwm_imx_gpt_apply_locked,
	.capture = pwm_imx_gpt_capture,
};

static int pwm_imx_gpt_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct device_node *np = dev->of_node;
	// struct stm32_timers *ddata = dev_get_drvdata(pdev->dev.parent);
	struct imx_gpt_pwm *priv;
	int ret;

	priv = devm_kzalloc(dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	// mutex_init(&priv->lock);
	// priv->clk = ddata->clk;
	// priv->max_arr = ddata->max_arr;
	// // priv->chip.of_xlate = of_pwm_xlate_with_flags;
	// priv->chip.of_pwm_n_cells = 3;

	// if (!priv->regmap || !priv->clk)
	// 	return -EINVAL;

	priv->chip.base = -1;
	priv->chip.dev = dev;
	priv->chip.ops = &imxgptpwm_opts;
	priv->chip.npwm = 2;

	ret = pwmchip_add(&priv->chip);
	if (ret < 0)
		return ret;

	platform_set_drvdata(pdev, priv);

	return 0;
}

static int pwm_imx_gpt_remove(struct platform_device *pdev)
{
	struct imx_gpt_pwm *priv = platform_get_drvdata(pdev);
	unsigned int i;

	for (i = 0; i < priv->chip.npwm; i++)
		pwm_disable(&priv->chip.pwms[i]);

	pwmchip_remove(&priv->chip);

	return 0;
}


static const struct of_device_id pwm_imx_gpt_of_match[] = {
	{ .compatible = "fsl,pwm-imx-gpt",	},
	{ /* end node */ },
};
MODULE_DEVICE_TABLE(of, pwm_imx_gpt_of_match);

static struct platform_driver pwm_imx_gpt_driver = {
	.probe	= pwm_imx_gpt_probe,
	.remove	= pwm_imx_gpt_remove,
	.driver	= {
		.name = "pwm-imx-gpt",
		.of_match_table = pwm_imx_gpt_of_match,
		// .pm = &pwm_imx_gpt_pm_ops,
	},
};
module_platform_driver(pwm_imx_gpt_driver);

MODULE_ALIAS("platform:pwm-imx-gpt");
MODULE_DESCRIPTION("STMicroelectronics STM32 PWM driver");
MODULE_LICENSE("GPL v2");
