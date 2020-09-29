/*
 * pps-imx-gpt.c -- PPS client driver using GPIO
 *
 *
 * Copyright (C) 2010 Ricardo Martins <rasm@fe.up.pt>
 * Copyright (C) 2011 James Nuss <jamesnuss@nanometrics.ca>
 *
 *   This program is free software; you can redistribute it and/or modify
 *   it under the terms of the GNU General Public License as published by
 *   the Free Software Foundation; either version 2 of the License, or
 *   (at your option) any later version.
 *
 *   This program is distributed in the hope that it will be useful,
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *   GNU General Public License for more details.
 *
 *   You should have received a copy of the GNU General Public License
 *   along with this program; if not, write to the Free Software
 *   Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 */

#define pps_gpt_NAME "pps-imx-gpt"
#define pr_fmt(fmt) pps_gpt_NAME ": " fmt

#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/pps_kernel.h>
#include <linux/list.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>
#include <linux/mxc_icap.h>
#include <linux/interrupt.h>

/* Info for each registered platform device */
struct pps_gpt_device_data {
	struct pps_device *pps;		/* PPS source device */
	struct pps_source_info info;	/* PPS source information */
	bool assert_falling_edge;
	bool capture_clear;
};

/*
 * Report the PPS event
 */

static void pps_gpt_irq_handler(int chan, void * data, ktime_t event)
{
	const struct pps_gpt_device_data *info = data;
	struct pps_event_time ts;
	struct pps_event_time ts_now;
	int rising_edge;

    pps_get_ts(&ts_now);
    // PPS event was captured earlier, calculate the delta
    struct timespec64 delta = timespec64_sub(ts_now.ts_raw, ktime_to_timespec64(event));
    
    pps_sub_ts(&ts_now, delta);

    pps_event(info->pps, &ts_now, PPS_CAPTUREASSERT, NULL);
    printk("event ktime_to_ns: %lld\n", ktime_to_ns(event));
    // printk("ts raw ns: %lld\n", timespec64_to_ns(&ts_now.ts_raw));
    // printk("ts real ns: %lld\n", timespec64_to_ns(&ts_now.ts_real));
    

	return;
}


static unsigned long
get_irqf_trigger_flags(const struct pps_gpt_device_data *data)
{
	unsigned long flags = data->assert_falling_edge ?
		IRQF_TRIGGER_FALLING : IRQF_TRIGGER_RISING;

	if (data->capture_clear) {
		flags |= ((flags & IRQF_TRIGGER_RISING) ?
				IRQF_TRIGGER_FALLING : IRQF_TRIGGER_RISING);
	}

	return flags;
}

static int pps_gpt_probe(struct platform_device *pdev)
{
	struct pps_gpt_device_data *data;
	int ret;
	int pps_default_params;
	const struct pps_gpt_platform_data *pdata = pdev->dev.platform_data;
	struct device_node *np = pdev->dev.of_node;

	/* allocate space for device info */
	data = devm_kzalloc(&pdev->dev, sizeof(struct pps_gpt_device_data),
			GFP_KERNEL);
	if (!data)
		return -ENOMEM;

    if (of_get_property(np, "assert-falling-edge", NULL)) {
        data->assert_falling_edge = true;
    }

	/* initialize PPS specific parts of the bookkeeping data structure. */
	data->info.mode = PPS_CAPTUREASSERT | PPS_OFFSETASSERT |
		PPS_ECHOASSERT | PPS_CANWAIT | PPS_TSFMT_TSPEC;

	data->info.owner = THIS_MODULE;
	snprintf(data->info.name, PPS_MAX_NAME_LEN - 1, "%s.%d",
		 pdev->name, pdev->id);

	/* register PPS source */
	pps_default_params = PPS_CAPTUREASSERT | PPS_OFFSETASSERT;

	data->pps = pps_register_source(&data->info, pps_default_params);
	if (data->pps == NULL) {
		dev_err(&pdev->dev, "failed to register gpt as PPS source\n");
		return -EINVAL;
	}

	platform_set_drvdata(pdev, data);

	/* register IRQ interrupt handler */
	ret = mxc_request_input_capture(0, pps_gpt_irq_handler, IRQF_TRIGGER_RISING, data);
    
	if (ret) {
		pps_unregister_source(data->pps);
		dev_err(&pdev->dev, "failed to mxc_request_input_capture\n");
		return -EINVAL;
	}

	dev_info(data->pps->dev, "Registered mxc_request_input_capture as PPS source\n");

	return 0;
}

static int pps_gpt_remove(struct platform_device *pdev)
{
	struct pps_gpt_device_data *data = platform_get_drvdata(pdev);

	pps_unregister_source(data->pps);
	dev_info(&pdev->dev, "removed mxc_request_input_capture as PPS source\n");
    mxc_free_input_capture(0, data);

	return 0;
}

static const struct of_device_id pps_gpt_dt_ids[] = {
	{ .compatible = "pps-imx-gpt", },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, pps_gpt_dt_ids);

static struct platform_driver pps_gpt_driver = {
	.probe		= pps_gpt_probe,
	.remove		= pps_gpt_remove,
	.driver		= {
		.name	= pps_gpt_NAME,
		.of_match_table	= pps_gpt_dt_ids,
	},
};

module_platform_driver(pps_gpt_driver);
MODULE_AUTHOR("Ricardo Martins <rasm@fe.up.pt>");
MODULE_AUTHOR("James Nuss <jamesnuss@nanometrics.ca>");
MODULE_DESCRIPTION("Use GPIO pin as PPS source");
MODULE_LICENSE("GPL");
MODULE_VERSION("1.0.0");
