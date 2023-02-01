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
#include <linux/bcd.h>
#include <linux/of_device.h>
#include "wbec.h"


#define WBEC_REG_RTC_TIME_S			0x00 /* status */
#define WBEC_REG_RTC_ALARM_S		0x07 /* datetime */
#define WBEC_REG_RTC_ALARM_S_AEN	BIT(7) /* alarm enable bit */

struct wbec_rtc_config {
	struct regmap_config regmap;
};

struct wbec_rtc {
	struct rtc_device	*rtc;
	struct regmap		*regmap;
	// struct clk_hw		clkout_hw;
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
	rc = regmap_bulk_read(wbec_rtc->regmap, WBEC_REG_RTC_TIME_S, regs,
				  sizeof(regs));
	if (rc)
		return rc;

	/* if the clock has lost its power it makes no sense to use its time */
	// TODO Check power loss?


	tm->tm_sec = bcd2bin(regs[0] & 0x7F);
	tm->tm_min = bcd2bin(regs[1] & 0x7F);
	tm->tm_hour = bcd2bin(regs[2] & 0x3F); /* rtc hr 0-23 */
	tm->tm_mday = bcd2bin(regs[3] & 0x3F);
	tm->tm_wday = regs[4] & 0x07;
	tm->tm_mon = bcd2bin(regs[5] & 0x1F) - 1; /* rtc mn 1-12 */
	tm->tm_year = bcd2bin(regs[6]);
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
	regs[0] = bin2bcd(tm->tm_sec) & 0x7F; /* clear OS flag */

	regs[1] = bin2bcd(tm->tm_min);
	regs[2] = bin2bcd(tm->tm_hour);

	/* Day of month, 1 - 31 */
	regs[3] = bin2bcd(tm->tm_mday);

	/* Day, 0 - 6 */
	regs[4] = tm->tm_wday & 0x07;

	/* month, 1 - 12 */
	regs[5] = bin2bcd(tm->tm_mon + 1);

	/* year and century */
	regs[6] = bin2bcd(tm->tm_year - 100);

	/* write all registers at once */
	rc = regmap_bulk_write(wbec_rtc->regmap, WBEC_REG_RTC_TIME_S,
				   regs, sizeof(regs));
	if (rc)
		return rc;

	return 0;
}

static int wbec_rtc_read_alarm(struct device *dev, struct rtc_wkalrm *alrm)
{
	struct wbec_rtc *wbec_rtc = dev_get_drvdata(dev);
	u8 buf[4];
	int ret;

	ret = regmap_bulk_read(wbec_rtc->regmap, WBEC_REG_RTC_ALARM_S,
				   buf, sizeof(buf));
	if (ret)
		return ret;

	alrm->time.tm_sec = bcd2bin(buf[0] & 0x7F);
	alrm->time.tm_min = bcd2bin(buf[1]);
	alrm->time.tm_hour = bcd2bin(buf[2]);
	alrm->time.tm_mday = bcd2bin(buf[3]);

	alrm->enabled =  !!(buf[0] & 0x80);

	return 0;
}

static int wbec_rtc_set_alarm(struct device *dev, struct rtc_wkalrm *alrm)
{
	struct wbec_rtc *wbec_rtc = dev_get_drvdata(dev);
	u8 buf[4];

	buf[0] = bin2bcd(alrm->time.tm_sec);
	buf[1] = bin2bcd(alrm->time.tm_min);
	buf[2] = bin2bcd(alrm->time.tm_hour);
	buf[3] = bin2bcd(alrm->time.tm_mday);

	if (alrm->enabled)
		buf[0] |= 0x80;

	return regmap_bulk_write(wbec_rtc->regmap, WBEC_REG_RTC_ALARM_S,
				buf, sizeof(buf));
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

	err = regmap_read(wbec_rtc->regmap, WBEC_REG_RTC_TIME_S, &tmp);
	if (err) {
		dev_err(&pdev->dev, "RTC chip is not present\n");
		return err;
	}

	wbec_rtc->rtc = devm_rtc_allocate_device(&pdev->dev);
	if (IS_ERR(wbec_rtc->rtc))
		return PTR_ERR(wbec_rtc->rtc);

	wbec_rtc->rtc->ops = &wbec_rtc_ops;

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

