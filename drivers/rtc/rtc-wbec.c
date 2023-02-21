// SPDX-License-Identifier: GPL-2.0-only
/*
 * TODO
 * Write description
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/i2c.h>
#include <linux/rtc.h>
#include <linux/platform_device.h>
#include <linux/time.h>
#include <linux/regmap.h>
#include <linux/of_device.h>
#include <linux/hrtimer.h>
#include "wbec.h"

#define WBEC_RTC_ALARM_FLAG_POLL_PERIOD_MS		500

struct wbec_rtc_config {
	struct regmap_config regmap;
};

struct wbec_rtc {
	struct rtc_device	*rtc;
	struct regmap		*regmap;
	struct hrtimer poll_timer;
};

static int wbec_rtc_read_time(struct device *dev, struct rtc_time *tm)
{
	struct wbec_rtc *wbec_rtc = dev_get_drvdata(dev);

	int rc;
	u8 regs[7];

	/*
	 * while reading, the time/date registers are blocked and not updated
	 * anymore until the access is finished. To not lose a second
	 * event, the access must be finished within one second. So, read all
	 * time/date registers in one turn.
	 */
	rc = regmap_bulk_read(wbec_rtc->regmap, WBEC_REG_RTC_TIME_SECONDS, regs,
				  sizeof(regs));
	if (rc)
		return rc;

	/* if the clock has lost its power it makes no sense to use its time */
	// TODO Check power loss?


	tm->tm_sec = regs[0];
	tm->tm_min = regs[1];
	tm->tm_hour = regs[2];
	tm->tm_mday = regs[3];
	tm->tm_wday = regs[4];
	tm->tm_mon = regs[5];
	tm->tm_year = regs[6];
	tm->tm_year += 100;

	return 0;
}

static int wbec_rtc_set_time(struct device *dev, struct rtc_time *tm)
{
	struct wbec_rtc *wbec_rtc = dev_get_drvdata(dev);
	int rc;
	u8 regs[7];

	// TODO Remove debug
	dev_info(dev, "%s function\n", __func__);

	/* hours, minutes and seconds */
	regs[0] = tm->tm_sec;
	regs[1] = tm->tm_min;
	regs[2] = tm->tm_hour;

	/* Day of month, 1 - 31 */
	regs[3] = tm->tm_mday;

	/* Day, 0 - 6 */
	regs[4] = tm->tm_wday & 0x07;

	/* month, 1 - 12 */
	regs[5] = tm->tm_mon + 1;

	/* year and century */
	regs[6] = tm->tm_year - 100;

	/* write all registers at once */
	rc = regmap_bulk_write(wbec_rtc->regmap, WBEC_REG_RTC_TIME_SECONDS,
				   regs, sizeof(regs));
	if (rc)
		return rc;

	return 0;
}

static int wbec_rtc_read_alarm(struct device *dev, struct rtc_wkalrm *alrm)
{
	struct wbec_rtc *wbec_rtc = dev_get_drvdata(dev);
	u8 buf[5];
	int ret;

	// TODO Remove debug
	dev_info(dev, "%s function\n", __func__);

	ret = regmap_bulk_read(wbec_rtc->regmap, WBEC_REG_RTC_ALARM_SECONDS,
				   buf, sizeof(buf));
	if (ret)
		return ret;

	alrm->time.tm_sec = buf[0];
	alrm->time.tm_min = buf[1];
	alrm->time.tm_hour = buf[2];
	alrm->time.tm_mday = buf[3];

	alrm->enabled =  !!(buf[4] & WBEC_REG_RTC_ALARM_17_EN_MSK);

	return 0;
}

static int wbec_rtc_set_alarm(struct device *dev, struct rtc_wkalrm *alrm)
{
	struct wbec_rtc *wbec_rtc = dev_get_drvdata(dev);
	u8 buf[5] = {};
	int ret;

	// TODO Remove debug
	dev_info(dev, "%s function, mday=%d, hour=%d, min=%d, sec=%d, en=%d\n", __func__,
		alrm->time.tm_mday, alrm->time.tm_hour, alrm->time.tm_min, alrm->time.tm_sec, alrm->enabled);

	buf[0] = alrm->time.tm_sec;
	buf[1] = alrm->time.tm_min;
	buf[2] = alrm->time.tm_hour;
	buf[3] = alrm->time.tm_mday;

	if (alrm->enabled)
		buf[4] |= WBEC_REG_RTC_ALARM_17_EN_MSK;

	ret = regmap_bulk_write(wbec_rtc->regmap, WBEC_REG_RTC_ALARM_SECONDS,
				buf, sizeof(buf));
	if (ret < 0)
		return ret;

	if (alrm->enabled) {
		dev_info(dev, "%s function: start poll timer\n", __func__);
		hrtimer_start(&wbec_rtc->poll_timer,
						ns_to_ktime(WBEC_RTC_ALARM_FLAG_POLL_PERIOD_MS * 1000000),
						HRTIMER_MODE_REL_PINNED);
	} else {
		hrtimer_cancel(&wbec_rtc->poll_timer);
	}

	return 0;
}

static int wbec_rtc_alarm_irq_enable(struct device *dev,
					 unsigned int enabled)
{
	struct wbec_rtc *wbec_rtc = dev_get_drvdata(dev);
	int ret;

	// TODO Remove debug
	dev_info(dev, "%s function, en=%d\n", __func__, enabled);

	ret = regmap_update_bits(wbec_rtc->regmap, WBEC_REG_RTC_ALARM_17,
				  WBEC_REG_RTC_ALARM_17_EN_MSK,
				  enabled ? WBEC_REG_RTC_ALARM_17_EN_MSK : 0);

	if (ret < 0) {
		return ret;
	}

	if (enabled) {
		hrtimer_start(&wbec_rtc->poll_timer,
					ns_to_ktime(WBEC_RTC_ALARM_FLAG_POLL_PERIOD_MS * 1000000),
					HRTIMER_MODE_REL_PINNED);
	} else {
		hrtimer_cancel(&wbec_rtc->poll_timer);
	}

	return 0;
}

static irqreturn_t wbec_rtc_handle_irq(int irq, void *dev_id)
{
	struct wbec_rtc *wbec_rtc = dev_id;
	unsigned int val;
	int err;

	// TODO Remove debug
	dev_info(regmap_get_device(wbec_rtc->regmap), "%s function\n", __func__);
	return IRQ_HANDLED;
}

static enum hrtimer_restart wbec_rtc_poll_cb(struct hrtimer *hrtimer)
{
	struct wbec_rtc *wbec_rtc =
		container_of(hrtimer, typeof(*wbec_rtc), poll_timer);
	int ret, val;

	dev_info(regmap_get_device(wbec_rtc->regmap), "%s function\n", __func__);
	// rtc_update_irq(wbec_rtc->rtc, 1, RTC_IRQF | RTC_AF);

	// return HRTIMER_NORESTART;

	/* Read alarm flag */
	ret = regmap_read(wbec_rtc->regmap, WBEC_REG_RTC_ALARM_17, &val);

	if (ret < 0) {
		dev_err(regmap_get_device(wbec_rtc->regmap), "Error when reading ALARM flag\n");
		return HRTIMER_NORESTART;
	}

	return HRTIMER_NORESTART;

	if (val & WBEC_REG_RTC_ALARM_17_FLAG_MSK) {
		// TODO Remove debug
		dev_info(regmap_get_device(wbec_rtc->regmap), "Alarm flag is set\n");
		// Clear flag
		ret = regmap_update_bits(wbec_rtc->regmap, WBEC_REG_RTC_ALARM_17, WBEC_REG_RTC_ALARM_17_FLAG_MSK, 0);
		if (ret < 0) {
			dev_err(regmap_get_device(wbec_rtc->regmap), "Error when clearing ALARM flag\n");
			return HRTIMER_NORESTART;
		}

		rtc_update_irq(wbec_rtc->rtc, 1, RTC_IRQF | RTC_AF);

		return HRTIMER_NORESTART;
	}

	hrtimer_forward_now(hrtimer,
			    ns_to_ktime(WBEC_RTC_ALARM_FLAG_POLL_PERIOD_MS * 1000000));

	return HRTIMER_RESTART;
}

static int wbec_rtc_read_offset(struct device *dev, long *offset)
{
	// TODO Remove debug
	dev_info(dev, "%s function\n", __func__);

	*offset = 0;

	return 0;
}

static int wbec_rtc_set_offset(struct device *dev, long offset)
{
	// TODO Remove debug
	dev_info(dev, "%s function\n", __func__);

	return 0;
}

static const struct rtc_class_ops wbec_rtc_ops = {
	.read_time	= wbec_rtc_read_time,
	.set_time	= wbec_rtc_set_time,
	.read_offset = wbec_rtc_read_offset,
	.set_offset	= wbec_rtc_set_offset,
	.read_alarm = wbec_rtc_read_alarm,
	.set_alarm = wbec_rtc_set_alarm,
	.alarm_irq_enable = wbec_rtc_alarm_irq_enable,
};

static int wbec_rtc_probe(struct platform_device *pdev)
{
	struct wbec *wbec = dev_get_drvdata(pdev->dev.parent);
	struct wbec_rtc *wbec_rtc;

	unsigned int tmp;
	int err;

	// TODO Remove debug
	dev_info(&pdev->dev, "%s function\n", __func__);
	dev_dbg(&pdev->dev, "%s\n", __func__);

	wbec_rtc = devm_kzalloc(&pdev->dev, sizeof(struct wbec_rtc),
				GFP_KERNEL);
	if (!wbec_rtc)
		return -ENOMEM;

	platform_set_drvdata(pdev, wbec_rtc);
	wbec_rtc->regmap = wbec->regmap_8;

	err = regmap_read(wbec_rtc->regmap, WBEC_REG_RTC_TIME_SECONDS, &tmp);
	if (err) {
		dev_err(&pdev->dev, "RTC chip is not present\n");
		return err;
	}

	wbec_rtc->rtc = devm_rtc_allocate_device(&pdev->dev);
	if (IS_ERR(wbec_rtc->rtc))
		return PTR_ERR(wbec_rtc->rtc);

	device_init_wakeup(&pdev->dev, true);

	wbec_rtc->rtc->ops = &wbec_rtc_ops;


	/* Alarm flag poll timer */

	hrtimer_init(&wbec_rtc->poll_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	wbec_rtc->poll_timer.function = wbec_rtc_poll_cb;

	/* Check alarm flag */
	err = regmap_test_bits(wbec_rtc->regmap, WBEC_REG_RTC_ALARM_17, WBEC_REG_RTC_ALARM_17_FLAG_MSK);
	if (err < 0) {
		dev_err(&pdev->dev, "Error when reading alarm flag\n");
		return err;
	} else if (err) {
		hrtimer_start(&wbec_rtc->poll_timer,
					ns_to_ktime(WBEC_RTC_ALARM_FLAG_POLL_PERIOD_MS * 1000000),
					HRTIMER_MODE_REL_PINNED);
	}


	return rtc_register_device(wbec_rtc->rtc);
}

static struct platform_driver wbec_rtc_driver = {
	.driver		= {
		.name	= "wbec-rtc",
	},
	.probe	= wbec_rtc_probe,
};

module_platform_driver(wbec_rtc_driver);

MODULE_AUTHOR("Pavel Gasheev <pavel.gasheev@wirenboard.ru>");
MODULE_DESCRIPTION("Wiren Board 7 Embedded Controller RTC driver");
MODULE_LICENSE("GPL");

