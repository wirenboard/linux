/*
 * lirc_gpio.c
 *
 * lirc_gpio - driver that captures pulse- and space-lengths with GPIO
 *
 * Author: Alexey Ignatov <lexszero@gmail.com>
 */
#include <linux/module.h>
#include <linux/sched.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>
#include <media/lirc.h>
#include <media/lirc_dev.h>

#define DRIVER_NAME	"lirc_gpio"

#define RBUF_LEN	4096

struct lirc_gpio_priv {
	struct lirc_driver *driver;

	unsigned int gpio;
	struct lirc_buffer rbuf;
	ktime_t last;
	int sense;
	int pulse, space;
	unsigned int ptr;
};

static void rbuf_write(struct lirc_gpio_priv *priv, int l)
{
	if (lirc_buffer_full(&priv->rbuf)) {
		/* no new signals will be accepted */
		dev_warn(priv->driver->dev, "buffer overrun\n");
		return;
	}
	lirc_buffer_write(&priv->rbuf, (void *)&l);
}

static void rbuf_write_filtered(struct lirc_gpio_priv *priv, int l)
{
	/* simple noise filter */
	if (priv->ptr > 0 && (l & PULSE_BIT)) {
		priv->pulse += l & PULSE_MASK;
		if (priv->pulse > 250) {
			rbuf_write(priv, priv->space);
			rbuf_write(priv, priv->pulse | PULSE_BIT);
			priv->ptr = 0;
			priv->pulse = 0;
		}
		return;
	}
	if (!(l & PULSE_BIT)) {
		if (priv->ptr == 0) {
			if (l > 20000) {
				priv->space = l;
				priv->ptr++;
				return;
			}
		} else {
			if (l > 20000) {
				priv->space += priv->pulse;
				if (priv->space > PULSE_MASK)
					priv->space = PULSE_MASK;
				priv->space += l;
				if (priv->space > PULSE_MASK)
					priv->space = PULSE_MASK;
				priv->pulse = 0;
				return;
			}
			rbuf_write(priv, priv->space);
			rbuf_write(priv, priv->pulse | PULSE_BIT);
			priv->ptr = 0;
			priv->pulse = 0;
		}
	}
	rbuf_write(priv, l);
}

static irqreturn_t irq_handler(int irq, void *data)
{
	struct lirc_gpio_priv *priv = data;
	struct device *dev = priv->driver->dev;
	int duration, value = gpio_get_value(priv->gpio);
	ktime_t now;
	s64 delta;

	if (priv->sense != -1) {
		now = ktime_get();
		delta = ktime_us_delta(now, priv->last);
		if (delta > 15000000) {
			duration = PULSE_MASK;
			if (!(value ^ priv->sense)) {
				dev_warn(dev, "wtf? value=%d, last=%lld, now=%lld, delta=%lld",
						value, ktime_to_us(priv->last), ktime_to_us(now), delta);
						
				priv->sense = priv->sense ? 0 : 1;
			}
		}
		else {
			duration = (int)delta;
		}
		rbuf_write_filtered(priv,
				(value ^ priv->sense) ? duration : (duration | PULSE_BIT));
		priv->last = now;
		wake_up_interruptible(&priv->rbuf.wait_poll);
	}

	return IRQ_HANDLED;
}

static int lirc_gpio_set_use_inc(void *data)
{
	struct lirc_gpio_priv *priv = data;
	struct device *dev = priv->driver->dev;

	dev_dbg(dev, "set_use_inc");

	return devm_request_irq(dev, gpio_to_irq(priv->gpio), irq_handler,
			IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING, dev_name(dev), priv);
}

static void lirc_gpio_set_use_dec(void *data)
{
	struct lirc_gpio_priv *priv = (struct lirc_gpio_priv *)data;
	struct device *dev = priv->driver->dev;

	dev_dbg(priv->driver->dev, "set_use_dec");

	devm_free_irq(dev, gpio_to_irq(priv->gpio), priv);
}

static int detect_active_low(struct lirc_gpio_priv *priv) {
	struct device *dev = priv->driver->dev;
	int ret, i, nlow = 0, nhigh = 0;

	ret = devm_gpio_request(dev, priv->gpio, DRIVER_NAME);
	if (ret < 0)
		return ret;

	gpio_direction_input(priv->gpio);

	/* wait for all transitions to settle, just for sure */
	mdelay(10);

	for (i = 0; i < 9; i++) {
		if (gpio_get_value(priv->gpio))
			nlow++;
		else
			nhigh++;
		msleep(40);
	}
	ret = (nlow >= nhigh) ? 1 : 0;
	
	devm_gpio_free(dev, priv->gpio);
	return ret;
}

static struct lirc_driver lirc_gpio_template = {
	.name		= DRIVER_NAME,
	.minor		= -1,
	.features	= LIRC_CAN_REC_MODE2,
	.code_length	= 1,
	.sample_rate	= 0,
	.data		= NULL,
	.add_to_buf	= NULL,
	.rbuf		= NULL,
	.set_use_inc	= lirc_gpio_set_use_inc,
	.set_use_dec	= lirc_gpio_set_use_dec,
	.fops		= NULL,
	.dev		= NULL,
	.owner		= THIS_MODULE,
};

static int lirc_gpio_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct device_node *np = dev->of_node;
	struct lirc_driver *driver;
	struct lirc_gpio_priv *priv;
	unsigned long flags;
	int gpio, ret = -EINVAL;

	priv = devm_kzalloc(dev, sizeof(struct lirc_gpio_priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	ret = lirc_buffer_init(&priv->rbuf, sizeof(int), RBUF_LEN);
	if (ret < 0)
		return ret;

	gpio = of_get_gpio(np, 0);
	if (!gpio_is_valid(gpio)) {
		dev_err(dev, "invalid gpio: %d\n", gpio);
		if (gpio < 0)
			ret = gpio;
		else
			ret = -EINVAL;
		goto exit_rbuf_free;
	}
	priv->gpio = gpio;

	flags = GPIOF_DIR_IN;
	if (of_find_property(np, "active-low", NULL))
		flags |= GPIOF_ACTIVE_LOW;
	else if (of_find_property(np, "active-detect", NULL)) {
		ret = detect_active_low(priv);
		if (ret < 0)
			dev_warn(dev, "unable to autodetect active level (ret=%d), assume high\n", ret);
		else {
			flags |= ret ? GPIOF_ACTIVE_LOW : 0;
			priv->sense = ret;
		}
	}
	dev_info(dev, "using active %s receiver on GPIO pin %d\n",
			flags & GPIOF_ACTIVE_LOW ? "low" : "high",
			gpio);

	ret = devm_gpio_request_one(dev, gpio, flags, np->name);
	if (ret < 0) {
		dev_err(dev, "unable to request gpio %d: %d\n", gpio, ret);
		return ret;
	}

	driver = devm_kzalloc(dev, sizeof(struct lirc_driver), GFP_KERNEL);
	if (!driver)
		return -ENOMEM;

	memcpy(driver, &lirc_gpio_template, sizeof(struct lirc_driver));

	driver->dev = dev;
	driver->rbuf = &priv->rbuf;

	priv->driver = driver;
	driver->data = priv;

	ret = lirc_register_driver(driver);
	if (ret < 0) {
		dev_err(dev, "lirc_register_driver failed: %d\n", ret);
		goto exit_rbuf_free;
	}
	driver->minor = ret;
	platform_set_drvdata(pdev, priv);

	dev_info(dev, "probed\n");
	return 0;

exit_rbuf_free:
	lirc_buffer_free(&priv->rbuf);
	return ret;
}

static int lirc_gpio_remove(struct platform_device *pdev)
{
	struct lirc_gpio_priv *priv = platform_get_drvdata(pdev);
	int ret;

	dev_dbg(&pdev->dev, "removing\n");

	lirc_buffer_free(&priv->rbuf);

	ret = lirc_unregister_driver(priv->driver->minor);
	if (ret)
		dev_err(&pdev->dev, "unable to unregister from lirc\n");
	else
		dev_info(&pdev->dev, "unregistered from lirc\n");

	return ret;
}

static struct of_device_id lirc_gpio_dt_ids[] = {
	{ .compatible = "lirc-gpio" },
	{}
};
MODULE_DEVICE_TABLE(of, lirc_gpio_dt_ids);

static struct platform_driver lirc_gpio_driver = {
	.driver = {
		.name	= DRIVER_NAME,
		.of_match_table = of_match_ptr(lirc_gpio_dt_ids),
	},
	.probe = lirc_gpio_probe,
	.remove	= lirc_gpio_remove,
};

module_platform_driver(lirc_gpio_driver);

MODULE_DESCRIPTION("LIRC GPIO Receiver driver");
MODULE_AUTHOR("Alexey Ignatov <lexszero@gmail.com>");
MODULE_LICENSE("GPL");
