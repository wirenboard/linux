// SPDX-License-Identifier: GPL-2.0-only
/*
 * TODO
 * Write description
 */

#include <linux/errno.h>
#include <linux/init.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/platform_device.h>

static irqreturn_t pwrkey_fall_irq(int irq, void *_pwr)
{
	struct input_dev *pwr = _pwr;

	input_report_key(pwr, KEY_POWER, 1);
	input_sync(pwr);

	return IRQ_HANDLED;
}

static irqreturn_t pwrkey_rise_irq(int irq, void *_pwr)
{
	struct input_dev *pwr = _pwr;

	input_report_key(pwr, KEY_POWER, 0);
	input_sync(pwr);

	return IRQ_HANDLED;
}

static int wbec_pwrkey_probe(struct platform_device *pdev)
{
	struct input_dev *pwr;
	int fall_irq, rise_irq;
	int err;

	// TODO Remove debug
	dev_info(&pdev->dev, "%s function\n", __func__);

	pwr = devm_input_allocate_device(&pdev->dev);
	if (!pwr) {
		dev_err(&pdev->dev, "Can't allocate power button\n");
		return -ENOMEM;
	}

	pwr->name = "wbec pwrkey";
	pwr->phys = "wbec_pwrkey/input0";
	pwr->id.bustype = BUS_HOST;
	input_set_capability(pwr, EV_KEY, KEY_POWER);

	fall_irq = platform_get_irq(pdev, 0);
	if (fall_irq < 0)
		return fall_irq;

	rise_irq = platform_get_irq(pdev, 1);
	if (rise_irq < 0)
		return rise_irq;

	err = devm_request_any_context_irq(&pwr->dev, fall_irq,
					   pwrkey_fall_irq,
					   IRQF_TRIGGER_FALLING | IRQF_ONESHOT,
					   "wbec_pwrkey_fall", pwr);
	if (err < 0) {
		dev_err(&pdev->dev, "Can't register fall irq: %d\n", err);
		return err;
	}

	err = devm_request_any_context_irq(&pwr->dev, rise_irq,
					   pwrkey_rise_irq,
					   IRQF_TRIGGER_RISING | IRQF_ONESHOT,
					   "wbec_pwrkey_rise", pwr);
	if (err < 0) {
		dev_err(&pdev->dev, "Can't register rise irq: %d\n", err);
		return err;
	}

	err = input_register_device(pwr);
	if (err) {
		dev_err(&pdev->dev, "Can't register power button: %d\n", err);
		return err;
	}

	platform_set_drvdata(pdev, pwr);
	device_init_wakeup(&pdev->dev, true);

	return 0;
}

static struct platform_driver wbec_pwrkey_driver = {
	.probe	= wbec_pwrkey_probe,
	.driver	= {
		.name = "wbec-pwrkey",
	},
};
module_platform_driver(wbec_pwrkey_driver);

MODULE_ALIAS("platform:wbec-pwrkey");
MODULE_AUTHOR("Pavel Gasheev <pavel.gasheev@wirenboard.ru>");
MODULE_DESCRIPTION("Wiren Board 7 Embedded Controller Power Key driver");
MODULE_LICENSE("GPL");
