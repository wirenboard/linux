// SPDX-License-Identifier: GPL-2.0-only
/*
 * rtc-wbec.c - Wiren Board Embedded Controller RTC driver
 *
 * Copyright (c) 2023 Wiren Board LLC
 *
 * Author: Pavel Gasheev <pavel.gasheev@wirenboard.ru>
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
#include <linux/mfd/wbec.h>

#define WBEC_RTC_ALARM_FLAG_POLL_PERIOD_MS		500

struct wbec_rtc_config {
	struct regmap_config regmap;
};

struct wbec_rtc {
	struct rtc_device	*rtc;
	struct regmap		*regmap;
};

static int wbec_rtc_read_time(struct device *dev, struct rtc_time *tm)
{
	struct wbec_rtc *wbec_rtc = dev_get_drvdata(dev);

	int rc;
	u16 regs[4];

	/*
	 * while reading, the time/date registers are blocked and not updated
	 * anymore until the access is finished. To not lose a second
	 * event, the access must be finished within one second. So, read all
	 * time/date registers in one turn.
	 */
	rc = regmap_bulk_read(wbec_rtc->regmap, WBEC_REG_RTC_TIME_SECS_MINS, regs,
				  ARRAY_SIZE(regs));
	if (rc)
		return rc;

	tm->tm_sec = regs[0] & 0x00FF;
	tm->tm_min = regs[0] >> 8;
	tm->tm_hour = regs[1] & 0x00FF;
	tm->tm_mday = regs[1] >> 8;
	tm->tm_wday = regs[2] & 0x00FF;
	tm->tm_mon = (regs[2] >> 8) - 1;
	tm->tm_year = regs[3];
	tm->tm_year += 100;

	return 0;
}

static int wbec_rtc_set_time(struct device *dev, struct rtc_time *tm)
{
	struct wbec_rtc *wbec_rtc = dev_get_drvdata(dev);
	int rc;
	u16 regs[4];

	dev_dbg(dev, "%s function\n", __func__);

	/* hours, minutes and seconds */
	regs[0] = tm->tm_sec;
	regs[0] |= tm->tm_min << 8;
	regs[1] = tm->tm_hour;

	/* Day of month, 1 - 31 */
	regs[1] |= tm->tm_mday << 8;

	/* Day, 0 - 6 */
	regs[2] = tm->tm_wday & 0x07;

	/* month, 1 - 12 */
	regs[2] |= (tm->tm_mon + 1) << 8;

	/* year and century */
	regs[3] = tm->tm_year - 100;

	/* write all registers at once */
	rc = regmap_bulk_write(wbec_rtc->regmap, WBEC_REG_RTC_TIME_SECS_MINS,
				   regs, ARRAY_SIZE(regs));
	if (rc)
		return rc;

	return 0;
}

static int wbec_rtc_read_alarm(struct device *dev, struct rtc_wkalrm *alrm)
{
	struct wbec_rtc *wbec_rtc = dev_get_drvdata(dev);
	u16 buf[4];
	int ret;

	dev_dbg(dev, "%s function\n", __func__);

	ret = regmap_bulk_read(wbec_rtc->regmap, WBEC_REG_RTC_ALARM_SECS_MINS,
				   buf, ARRAY_SIZE(buf));
	if (ret)
		return ret;

	alrm->time.tm_sec = buf[0] & 0x00FF;
	alrm->time.tm_min = buf[0] >> 8;
	alrm->time.tm_hour = buf[1] & 0x00FF;
	alrm->time.tm_mday = buf[1] >> 8;

	alrm->enabled =  !!(buf[2] & WBEC_REG_RTC_ALARM_STATUS_EN_MSK);

	return 0;
}

static int wbec_rtc_set_alarm(struct device *dev, struct rtc_wkalrm *alrm)
{
	struct wbec_rtc *wbec_rtc = dev_get_drvdata(dev);
	u16 buf[4] = {};

	dev_dbg(dev, "%s function, mday=%d, hour=%d, min=%d, sec=%d, en=%d\n", __func__,
		alrm->time.tm_mday, alrm->time.tm_hour, alrm->time.tm_min, alrm->time.tm_sec, alrm->enabled);

	buf[0] = alrm->time.tm_sec;
	buf[0] |= alrm->time.tm_min << 8;
	buf[1] = alrm->time.tm_hour;
	buf[1] |= alrm->time.tm_mday << 8;

	if (alrm->enabled)
		buf[2] |= WBEC_REG_RTC_ALARM_STATUS_EN_MSK;

	return regmap_bulk_write(wbec_rtc->regmap, WBEC_REG_RTC_ALARM_SECS_MINS,
				buf, ARRAY_SIZE(buf));
}

static int wbec_rtc_alarm_irq_enable(struct device *dev,
					 unsigned int enabled)
{
	struct wbec_rtc *wbec_rtc = dev_get_drvdata(dev);

	dev_dbg(dev, "%s function, en=%d\n", __func__, enabled);

	return regmap_update_bits(wbec_rtc->regmap, WBEC_REG_RTC_ALARM_STATUS,
				  WBEC_REG_RTC_ALARM_STATUS_EN_MSK,
				  enabled ? WBEC_REG_RTC_ALARM_STATUS_EN_MSK : 0);
}

static int wbec_rtc_read_offset(struct device *dev, long *offset)
{
	struct wbec_rtc *wbec_rtc = dev_get_drvdata(dev);
	int ret;
	long tmp = 0;
	int reg;

	dev_dbg(dev, "%s function\n", __func__);

	ret = regmap_read(wbec_rtc->regmap, WBEC_REG_RTC_CFG_OFFSET, &reg);
	if (ret)
		return ret;

	/* Linux units is ppb
	 * wbec offset register represents STM32G0 RTC_CALR register:
	 * Bit 15 CALP: Increase frequency of RTC by 488.5ppm
	 * Bit 14 CALW8: Use an 8-second calibration cycle period
	 * Bit 13 CALW16: Use a 16-second calibration cycle period
	 * Bits 8:0 CALM[8:0]: Calibration minus
	 * 1 lsb in wbec is 0.9537 ppm
	 */

	if (reg & WBEC_REG_RTC_CFG_OFFSET_CALP_BIT)
		tmp = 488500;

	tmp -= ((reg & WBEC_REG_RTC_CFG_OFFSET_CALM_MASK) * 9537) / 10;

	/* CALW8 and CALW16 unused */
	*offset = tmp;

	return 0;
}

static int wbec_rtc_set_offset(struct device *dev, long offset)
{
	struct wbec_rtc *wbec_rtc = dev_get_drvdata(dev);
	u16 reg = 0;

	dev_dbg(dev, "%s function\n", __func__);

	if (offset > 0) {
		reg = (488500 - offset) * 10 / 9537;
		reg &= WBEC_REG_RTC_CFG_OFFSET_CALM_MASK;
		reg |= WBEC_REG_RTC_CFG_OFFSET_CALP_BIT;
	} else if (offset < 0) {
		reg = -offset;
		reg &= WBEC_REG_RTC_CFG_OFFSET_CALM_MASK;
	}

	return regmap_write(wbec_rtc->regmap, WBEC_REG_RTC_CFG_OFFSET, reg);
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

	dev_dbg(&pdev->dev, "%s function\n", __func__);

	wbec_rtc = devm_kzalloc(&pdev->dev, sizeof(struct wbec_rtc),
				GFP_KERNEL);
	if (!wbec_rtc)
		return -ENOMEM;

	platform_set_drvdata(pdev, wbec_rtc);
	wbec_rtc->regmap = wbec->regmap;

	err = regmap_read(wbec_rtc->regmap, WBEC_REG_RTC_TIME_SECS_MINS, &tmp);
	if (err) {
		dev_err(&pdev->dev, "RTC chip is not present\n");
		return err;
	}

	wbec_rtc->rtc = devm_rtc_allocate_device(&pdev->dev);
	if (IS_ERR(wbec_rtc->rtc))
		return PTR_ERR(wbec_rtc->rtc);

	wbec_rtc->rtc->ops = &wbec_rtc_ops;

	device_init_wakeup(&pdev->dev, true);

	wbec_rtc->rtc->uie_unsupported = 1;

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

