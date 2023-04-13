// SPDX-License-Identifier: GPL-2.0-only
/*
 * iio-wbec.c - Wiren Board Embedded Controller IIO driver
 *
 * Copyright (c) 2023 Wiren Board LLC
 *
 * Author: Pavel Gasheev <pavel.gasheev@wirenboard.ru>
 */

#include <linux/err.h>
#include <linux/i2c.h>
#include <linux/delay.h>
#include <linux/module.h>
#include <linux/pm.h>
#include <linux/bitops.h>
#include <linux/regmap.h>
#include <linux/platform_device.h>

#include <linux/iio/iio.h>
#include <linux/iio/sysfs.h>
#include <linux/mfd/wbec.h>

enum wbec_iio_channel {
	WBEC_IIO_CH_V_A1,
	WBEC_IIO_CH_V_A2,
	WBEC_IIO_CH_V_A3,
	WBEC_IIO_CH_V_A4,
	WBEC_IIO_CH_V_IN,
	WBEC_IIO_CH_V_3_3,
	WBEC_IIO_CH_V_5_0,
	WBEC_IIO_CH_VBUS_CONSOLE,
	WBEC_IIO_CH_VBUS_NETWORK,
	WBEC_IIO_CH_TEMP,
};


struct wbec_iio {
	struct regmap *regmap;
};

union wbec_iio_value {
	s16 s_value;
	u16 u_value;
	u8 regs[2];
};

static int wbec_read_raw(struct iio_dev *indio_dev,
				struct iio_chan_spec const *channel, int *val,
				int *val2, long mask)
{
	struct wbec_iio *wbec_iio = iio_priv(indio_dev);
	s32 ret;

	dev_dbg(&indio_dev->dev, "%s function. addr=0x%lX\n", __func__, channel->address);

	switch (mask) {
	case IIO_CHAN_INFO_RAW:
		if ((channel->type == IIO_TEMP) || (channel->type == IIO_VOLTAGE)) {
			/* Temperature in deg C x100 */
			ret = regmap_read(wbec_iio->regmap, channel->address, val);
			if (ret < 0) {
				dev_err(&indio_dev->dev, "error reading value from reg 0x%lX", channel->address);
				return ret;
			}
		} else {
			break;
		}
		return IIO_VAL_INT;

	case IIO_CHAN_INFO_SCALE:
		if (channel->type == IIO_VOLTAGE) {
			*val = 1;
			*val2 = 0;
		} else if (channel->type == IIO_TEMP) {
			*val = 100;
			*val2 = 0;
		} else {
			break;
		}
		return IIO_VAL_INT_PLUS_MICRO;
	default:
		break;
	}

	return -EINVAL;
}


#define WBEC_IIO_CHANNEL(_id, _type, chan_info,	\
				 _ext_name) {			\
	.type = _type,						\
	.indexed = 1,						\
	.channel = WBEC_IIO_CH_##_id,			\
	.address = WBEC_REG_ADC_DATA_##_id,		\
	.info_mask_separate = chan_info,			\
	.extend_name = _ext_name,				\
	.datasheet_name = #_id,					\
}


static const struct iio_chan_spec wbec_iio_channels[] = {
	WBEC_IIO_CHANNEL(V_A1, IIO_VOLTAGE, BIT(IIO_CHAN_INFO_RAW) | BIT(IIO_CHAN_INFO_SCALE), "A1"),
	WBEC_IIO_CHANNEL(V_A2, IIO_VOLTAGE, BIT(IIO_CHAN_INFO_RAW) | BIT(IIO_CHAN_INFO_SCALE), "A2"),
	WBEC_IIO_CHANNEL(V_A3, IIO_VOLTAGE, BIT(IIO_CHAN_INFO_RAW) | BIT(IIO_CHAN_INFO_SCALE), "A3"),
	WBEC_IIO_CHANNEL(V_A4, IIO_VOLTAGE, BIT(IIO_CHAN_INFO_RAW) | BIT(IIO_CHAN_INFO_SCALE), "A4"),
	WBEC_IIO_CHANNEL(V_IN, IIO_VOLTAGE, BIT(IIO_CHAN_INFO_RAW) | BIT(IIO_CHAN_INFO_SCALE), "IN"),
	WBEC_IIO_CHANNEL(V_3_3, IIO_VOLTAGE, BIT(IIO_CHAN_INFO_RAW) | BIT(IIO_CHAN_INFO_SCALE), "3V3"),
	WBEC_IIO_CHANNEL(V_5_0, IIO_VOLTAGE, BIT(IIO_CHAN_INFO_RAW) | BIT(IIO_CHAN_INFO_SCALE), "5V"),
	WBEC_IIO_CHANNEL(VBUS_CONSOLE, IIO_VOLTAGE, BIT(IIO_CHAN_INFO_RAW) | BIT(IIO_CHAN_INFO_SCALE), "VBUS_CONSOLE"),
	WBEC_IIO_CHANNEL(VBUS_NETWORK, IIO_VOLTAGE, BIT(IIO_CHAN_INFO_RAW) | BIT(IIO_CHAN_INFO_SCALE), "VBUS_DEBUG"),
	WBEC_IIO_CHANNEL(TEMP, IIO_TEMP, BIT(IIO_CHAN_INFO_RAW) | BIT(IIO_CHAN_INFO_SCALE), "BOARD"),
};

static const struct iio_info wbec_iio_info = {
	.read_raw = wbec_read_raw,
};

static int wbec_iio_probe(struct platform_device *pdev)
{
	struct wbec *wbec = dev_get_drvdata(pdev->dev.parent);
	struct iio_dev *indio_dev;
	struct wbec_iio *wbec_iio;

	dev_info(&pdev->dev, "%s function\n", __func__);
	dev_dbg(&pdev->dev, "%s\n", __func__);

	indio_dev = devm_iio_device_alloc(&pdev->dev, sizeof(*wbec_iio));
	if (!indio_dev)
		return -ENOMEM;

	wbec_iio = iio_priv(indio_dev);

	platform_set_drvdata(pdev, indio_dev);
	wbec_iio->regmap = wbec->regmap;

	indio_dev->name = "wbec";
	indio_dev->modes = INDIO_DIRECT_MODE;
	indio_dev->info = &wbec_iio_info;

	indio_dev->channels = wbec_iio_channels;
	indio_dev->num_channels = ARRAY_SIZE(wbec_iio_channels);

	return devm_iio_device_register(&pdev->dev, indio_dev);
}

static struct platform_driver wbec_iio_driver = {
	.driver = {
		.name	= "wbec-iio",
	},
	.probe = wbec_iio_probe,
};

module_platform_driver(wbec_iio_driver);

MODULE_AUTHOR("Pavel Gasheev <pavel.gasheev@wirenboard.ru>");
MODULE_DESCRIPTION("Wiren Board 7 Embedded Controller IIO driver");
MODULE_LICENSE("GPL");
