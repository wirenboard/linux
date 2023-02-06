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
#include <linux/hrtimer.h>
#include <linux/regmap.h>
#include "wbec.h"

#define WBEC_PWRKEY_POLL_PERIOD_NS		100000000

#define WBEC_IRQ_PWRON_RISE_MSK			BIT(0)
#define WBEC_IRQ_PWRON_FALL_MSK			BIT(1)

struct wbec_pwrkey {
	struct input_dev *pwr;
	struct hrtimer poll_timer;
	struct regmap *regmap;
};

static enum hrtimer_restart pwrkey_poll_cb(struct hrtimer *hrtimer)
{
	struct wbec_pwrkey *wbec_pwrkey =
		container_of(hrtimer, typeof(*wbec_pwrkey), poll_timer);
	struct input_dev *pwr = wbec_pwrkey->pwr;

	int val;
	int ret = regmap_read(wbec_pwrkey->regmap, WBEC_REG_IRQ_FLAGS, &val);

	if ((ret == 0) && (val & WBEC_IRQ_PWRON_FALL_MSK)) {
		// TODO Remove debug
		dev_info(&pwr->dev, "Power key press detected\n");
		// Clear irq flag
		regmap_update_bits(wbec_pwrkey->regmap, WBEC_REG_IRQ_MSK, WBEC_IRQ_PWRON_FALL_MSK, 1);
		input_report_key(pwr, KEY_POWER, 1);
		input_sync(pwr);

		return HRTIMER_NORESTART;
	}

	hrtimer_forward_now(hrtimer,
			    ns_to_ktime(WBEC_PWRKEY_POLL_PERIOD_NS));

	return HRTIMER_RESTART;
}

static int wbec_pwrkey_probe(struct platform_device *pdev)
{
	struct wbec *wbec = dev_get_drvdata(pdev->dev.parent);
	struct wbec_pwrkey *wbec_pwrkey;
	int err;

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

	hrtimer_init(&wbec_pwrkey->poll_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	wbec_pwrkey->poll_timer.function = pwrkey_poll_cb;

	err = input_register_device(wbec_pwrkey->pwr);
	if (err) {
		dev_err(&pdev->dev, "Can't register power button: %d\n", err);
		return err;
	}

	platform_set_drvdata(pdev, wbec_pwrkey);
	device_init_wakeup(&pdev->dev, true);

	hrtimer_start(&wbec_pwrkey->poll_timer,
			      ns_to_ktime(WBEC_PWRKEY_POLL_PERIOD_NS),
			      HRTIMER_MODE_REL_PINNED);

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
