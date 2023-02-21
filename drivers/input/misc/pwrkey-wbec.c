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
#include <linux/regmap.h>
#include "wbec.h"

struct wbec_pwrkey {
	struct input_dev *pwr;
	struct regmap *regmap;
};


static irqreturn_t pwrkey_poweroff_req_irq(int irq, void *_pwr)
{
	struct input_dev *pwr = _pwr;

	// TODO Remove debug
	dev_info(&pwr->dev, "pwrkey irq handled\n");

	input_report_key(pwr, KEY_POWER, 1);
	input_sync(pwr);

	return IRQ_HANDLED;
}

static int wbec_pwrkey_probe(struct platform_device *pdev)
{
	struct wbec *wbec = dev_get_drvdata(pdev->dev.parent);
	struct wbec_pwrkey *wbec_pwrkey;
	int err, poff_irq;

	// TODO Remove debug
	dev_info(&pdev->dev, "%s function\n", __func__);

	wbec_pwrkey = devm_kzalloc(&pdev->dev, sizeof(struct wbec_pwrkey),
				GFP_KERNEL);
	if (!wbec_pwrkey)
		return -ENOMEM;

	wbec_pwrkey->regmap = wbec->regmap_8;

	wbec_pwrkey->pwr = devm_input_allocate_device(&pdev->dev);
	if (!wbec_pwrkey->pwr) {
		dev_err(&pdev->dev, "Can't allocate power button\n");
		return -ENOMEM;
	}

	wbec_pwrkey->pwr->name = "wbec pwrkey";
	wbec_pwrkey->pwr->phys = "wbec_pwrkey/input0";
	wbec_pwrkey->pwr->id.bustype = BUS_HOST;
	input_set_capability(wbec_pwrkey->pwr, EV_KEY, KEY_POWER);

	poff_irq = platform_get_irq(pdev, 0);
	if (poff_irq < 0)
		return poff_irq;

	err = devm_request_any_context_irq(&wbec_pwrkey->pwr->dev, poff_irq,
					   pwrkey_poweroff_req_irq,
					   IRQF_TRIGGER_RISING | IRQF_ONESHOT,
					   "wbec_pwrkey_pressed", wbec_pwrkey->pwr);
	if (err < 0) {
		dev_err(&pdev->dev, "Can't register pwrkey press irq: %d\n", err);
		return err;
	}

	err = input_register_device(wbec_pwrkey->pwr);
	if (err) {
		dev_err(&pdev->dev, "Can't register power button: %d\n", err);
		return err;
	}

	platform_set_drvdata(pdev, wbec_pwrkey);
	device_init_wakeup(&pdev->dev, true);

	return 0;
}

static struct platform_driver wbec_pwrkey_driver = {
	.driver	= {
		.name = "wbec-pwrkey",
	},
	.probe	= wbec_pwrkey_probe,
};
module_platform_driver(wbec_pwrkey_driver);

MODULE_ALIAS("platform:wbec-pwrkey");
MODULE_AUTHOR("Pavel Gasheev <pavel.gasheev@wirenboard.ru>");
MODULE_DESCRIPTION("Wiren Board 7 Embedded Controller Power Key driver");
MODULE_LICENSE("GPL");
