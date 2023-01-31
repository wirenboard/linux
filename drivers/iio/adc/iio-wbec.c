// SPDX-License-Identifier: GPL-2.0-only
/*
TODO
Write description
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
#include "wbec.h"

enum wbec_iio_channel_regs {
	WBEC_IIO_REG_V_IN = 0x10,
	WBEC_IIO_REG_V_BAT = 0x12,
	WBEC_IIO_REG_V_3_3 = 0x14,
	WBEC_IIO_REG_V_5_0 = 0x16,
	WBEC_IIO_REG_V_A1 = 0x18,
	WBEC_IIO_REG_V_A2 = 0x1A,
	WBEC_IIO_REG_V_A3 = 0x1C,
	WBEC_IIO_REG_V_A4 = 0x1E,
	WBEC_IIO_REG_TEMP = 0x20,
};

enum wbec_iio_channel {
	WBEC_IIO_CH_V_A0,
	WBEC_IIO_CH_V_A1,
	WBEC_IIO_CH_V_A2,
	WBEC_IIO_CH_V_A3,
	WBEC_IIO_CH_V_A4,
	WBEC_IIO_CH_V_IN,
	WBEC_IIO_CH_V_BAT,
	WBEC_IIO_CH_V_3_3,
	WBEC_IIO_CH_V_5_0,
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

    // TODO Remove debug
    printk(KERN_INFO "%s function. ch=%d, addr=%ld\n", __func__, channel->channel, channel->address);

	switch (mask) {
    case IIO_CHAN_INFO_PROCESSED:
		if ((channel->type == IIO_VOLTAGE) || (channel->type == IIO_TEMP)) {
			/* Voltage in mV */
			/* Temperature in deg C x100 */
			ret = regmap_read(wbec_iio->regmap, channel->address, val);
            if (ret < 0)
				return ret;
		} else {
			break;
		}
		return IIO_VAL_INT;
	case IIO_CHAN_INFO_SCALE:
		if (channel->type == IIO_VOLTAGE) {
			*val = 1000;
			*val2 = 0;
		} else if (channel->type == IIO_TEMP) {
			*val = 10;
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
	.address = WBEC_IIO_REG_##_id,		\
	.info_mask_separate = chan_info,			\
	.extend_name = _ext_name,				\
	.datasheet_name = #_id,					\
}


static const struct iio_chan_spec wbec_iio_channels[] = {
    WBEC_IIO_CHANNEL(V_A1, IIO_VOLTAGE, BIT(IIO_CHAN_INFO_PROCESSED), "v_a1"),
    WBEC_IIO_CHANNEL(V_A2, IIO_VOLTAGE, BIT(IIO_CHAN_INFO_PROCESSED), "v_a2"),
    WBEC_IIO_CHANNEL(V_A3, IIO_VOLTAGE, BIT(IIO_CHAN_INFO_PROCESSED), "v_a3"),
    WBEC_IIO_CHANNEL(V_A4, IIO_VOLTAGE, BIT(IIO_CHAN_INFO_PROCESSED), "v_a4"),
    WBEC_IIO_CHANNEL(V_IN, IIO_VOLTAGE, BIT(IIO_CHAN_INFO_PROCESSED), "v_in"),
    WBEC_IIO_CHANNEL(V_BAT, IIO_VOLTAGE, BIT(IIO_CHAN_INFO_PROCESSED), "v_bat"),
    WBEC_IIO_CHANNEL(V_3_3, IIO_VOLTAGE, BIT(IIO_CHAN_INFO_PROCESSED), "v_3.3"),
    WBEC_IIO_CHANNEL(V_5_0, IIO_VOLTAGE, BIT(IIO_CHAN_INFO_PROCESSED), "v_5.0"),
    WBEC_IIO_CHANNEL(TEMP, IIO_TEMP, BIT(IIO_CHAN_INFO_RAW) | BIT(IIO_CHAN_INFO_SCALE), "temp"),
};

static const struct iio_info wbec_iio_info = {
	.read_raw = wbec_read_raw,
};

static int wbec_iio_probe(struct platform_device *pdev)
{
    struct wbec *wbec = dev_get_drvdata(pdev->dev.parent);
    struct i2c_client *client = wbec->i2c;
	struct iio_dev *indio_dev;
	struct wbec_iio *wbec_iio;

    printk(KERN_INFO "%s function\n", __func__);
    dev_dbg(&pdev->dev, "%s\n", __func__);

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_SMBUS_WORD_DATA))
		return -EOPNOTSUPP;

	indio_dev = devm_iio_device_alloc(&client->dev, sizeof(*wbec_iio));
	if (!indio_dev)
		return -ENOMEM;

	wbec_iio = iio_priv(indio_dev);

    platform_set_drvdata(pdev, indio_dev);
    wbec_iio->regmap = wbec->regmap_16;

	indio_dev->name = "wbec";
	indio_dev->modes = INDIO_DIRECT_MODE;
	indio_dev->info = &wbec_iio_info;

	indio_dev->channels = wbec_iio_channels;
	indio_dev->num_channels = ARRAY_SIZE(wbec_iio_channels);

	return iio_device_register(indio_dev);
}

static int wbec_iio_remove(struct platform_device *pdev)
{
	struct iio_dev *indio_dev = platform_get_drvdata(pdev);

    printk(KERN_INFO "%s function\n", __func__);

	iio_device_unregister(indio_dev);

	return 0;
}

static struct platform_driver wbec_iio_driver = {
	.driver = {
		.name	= "wbec-iio",
	},
	.probe = wbec_iio_probe,
	.remove = wbec_iio_remove,
};

module_platform_driver(wbec_iio_driver);

MODULE_AUTHOR("Pavel Gasheev <pavel.gasheev@wirenboard.ru>");
MODULE_DESCRIPTION("Wiren Board 7 Embedded Controller IIO driver");
MODULE_LICENSE("GPL");
