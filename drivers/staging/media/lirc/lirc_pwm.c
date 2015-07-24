/*
 * lirc_pwm.c
 *
 * lirc_pwm - driver that replays pulse- and space-lengths,
 *	      generating carrier with PWM.
 *
 * Author: Alexey Ignatov <lexszero@gmail.com>
 */

#include <linux/module.h>
#include <linux/sched.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/of.h>
#include <linux/of_platform.h>
#include <linux/pwm.h>
#include <media/lirc.h>
#include <media/lirc_dev.h>

#define DRIVER_NAME	"lirc_pwm"

#define DEFAULT_DUTY_CYCLE	50
#define DEFAULT_FREQ		38000

struct lirc_pwm_priv {
	struct lirc_driver *driver;

	struct pwm_device *pwm;
	unsigned int duty_cycle;
	unsigned int freq;

	struct mutex lock;
};

#ifndef MAX_UDELAY_MS
#define MAX_UDELAY_US 5000
#else
#define MAX_UDELAY_US (MAX_UDELAY_MS*1000)
#endif

static void safe_udelay(unsigned long usecs)
{
	while (usecs > MAX_UDELAY_US) {
		udelay(MAX_UDELAY_US);
		usecs -= MAX_UDELAY_US;
	}
	udelay(usecs);
}

static int update_pwm_config(struct lirc_pwm_priv *priv,
		unsigned int duty_cycle, unsigned int freq)
{
	unsigned int duty, period, ret;

	if (duty_cycle > 100)
		return -EINVAL;
	if (freq < 20000 || freq > 500000)
		return -EINVAL;

	mutex_lock(&priv->lock);

	priv->duty_cycle = duty_cycle;
	priv->freq = freq;
	period = 1000000000 / freq;
	duty = period * duty_cycle / 100;
	dev_dbg(priv->driver->dev, "period: %d ns, duty: %d ns\n",
			period, duty);

	ret = pwm_config(priv->pwm, duty, period);

	mutex_unlock(&priv->lock);
	return ret;
}

static inline int send_pulse(struct lirc_pwm_priv *priv)
{
	return pwm_enable(priv->pwm);
}

static inline void send_space(struct lirc_pwm_priv *priv)
{
	pwm_disable(priv->pwm);
}

static int lirc_pwm_set_use_inc(void *data)
{
	struct lirc_pwm_priv *priv = data;

	dev_dbg(priv->driver->dev, "set_use_inc");

	return update_pwm_config(priv, DEFAULT_DUTY_CYCLE, DEFAULT_FREQ);
}

static void lirc_pwm_set_use_dec(void *data)
{
	struct lirc_pwm_priv *priv = (struct lirc_pwm_priv *)data;

	dev_dbg(priv->driver->dev, "set_use_dec");
}

static ssize_t lirc_pwm_write(struct file *filep, const char *buf,
	size_t n, loff_t *ppos)
{
	struct lirc_driver *driver = filep->private_data;
	struct lirc_pwm_priv *priv = driver->data;
	int i, count, ret = 0;
	int *wbuf;
	ktime_t ns;

	count = n / sizeof(int);
	if (n % sizeof(int) || count % 2 == 0)
		return -EINVAL;

	mutex_lock(&priv->lock);

	wbuf = memdup_user(buf, n);
	if (IS_ERR(wbuf)) {
		ret = PTR_ERR(wbuf);
		goto exit_unlock;
	}

	for (i = 0; i < count; i++) {
		if (i & 1)
			send_space(priv);
		else
			send_pulse(priv);

		safe_udelay(wbuf[i]);
		/*
		set_current_state(TASK_UNINTERRUPTIBLE);
		ns = ns_to_ktime(wbuf[i] * 1000);
		schedule_hrtimeout(&ns, HRTIMER_MODE_REL);
		*/
	}
	send_space(priv);

	kfree(wbuf);
	wbuf = NULL;

exit_unlock:
	mutex_unlock(&priv->lock);
	return ret;
}

static long lirc_pwm_ioctl(struct file *filep, unsigned int cmd,
		unsigned long arg)
{
	struct lirc_driver *driver = filep->private_data;
	struct lirc_pwm_priv *priv = driver->data;
	int ret;
	uint32_t value;

	switch (cmd) {
	case LIRC_GET_SEND_MODE:
		return -ENOIOCTLCMD;

	case LIRC_SET_SEND_MODE:
		ret = get_user(value, (uint32_t *) arg);
		if (ret)
			return ret;
		/* only LIRC_MODE_PULSE supported */
		if (value != LIRC_MODE_PULSE)
			return -ENOSYS;
		break;

	case LIRC_GET_LENGTH:
		return -ENOSYS;

	case LIRC_SET_SEND_DUTY_CYCLE:
		ret = get_user(value, (uint32_t *) arg);
		if (ret)
			return ret;
		dev_dbg(driver->dev, "SET_SEND_DUTY_CYCLE %d\n", value);
		return update_pwm_config(priv, value, priv->freq);

	case LIRC_SET_SEND_CARRIER:
		ret = get_user(value, (__u32 *) arg);
		if (ret)
			return ret;
		dev_dbg(driver->dev, "SET_SEND_CARRIER %d\n", value);
		return update_pwm_config(priv, priv->duty_cycle, value);

	default:
		return lirc_dev_fop_ioctl(filep, cmd, arg);
	}
	return 0;
}

static const struct file_operations lirc_pwm_fops = {
	.owner		= THIS_MODULE,
	.write		= lirc_pwm_write,
	.unlocked_ioctl	= lirc_pwm_ioctl,
	.read		= lirc_dev_fop_read,
	.poll		= lirc_dev_fop_poll,
	.open		= lirc_dev_fop_open,
	.release	= lirc_dev_fop_close,
	.llseek		= no_llseek,
};

static struct lirc_driver lirc_pwm_template = {
	.name		= DRIVER_NAME,
	.minor		= -1,
	.features	= LIRC_CAN_SET_SEND_DUTY_CYCLE |
		LIRC_CAN_SET_SEND_CARRIER |
		LIRC_CAN_SEND_PULSE,
	.code_length	= 1,
	.sample_rate	= 0,
	.data		= NULL,
	.add_to_buf	= NULL,
	.rbuf		= NULL,
	.set_use_inc	= lirc_pwm_set_use_inc,
	.set_use_dec	= lirc_pwm_set_use_dec,
	.fops		= &lirc_pwm_fops,
	.dev		= NULL,
	.owner		= THIS_MODULE,
};

static int lirc_pwm_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct device_node *np = dev->of_node;
	struct lirc_driver *driver;
	struct lirc_pwm_priv *priv;
	int ret = -EINVAL;

	priv = devm_kzalloc(dev, sizeof(struct lirc_pwm_priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	mutex_init(&priv->lock);

	priv->pwm = devm_of_pwm_get(dev, np, NULL);

	if (IS_ERR(priv->pwm)) {
		ret = PTR_ERR(priv->pwm);
		dev_err(dev, "unable to request PWM: %d\n", ret);
		return ret;
	}

	driver = devm_kzalloc(dev, sizeof(struct lirc_driver), GFP_KERNEL);
	if (!driver)
		return -ENOMEM;

	memcpy(driver, &lirc_pwm_template, sizeof(struct lirc_driver));

	driver->dev = dev;
	priv->driver = driver;
	driver->data = priv;

	ret = lirc_register_driver(driver);
	if (ret < 0) {
		dev_err(dev, "lirc_register_driver failed: %d\n", ret);
		return ret;
	}
	driver->minor = ret;
	platform_set_drvdata(pdev, priv);

	dev_info(dev, "probed\n");
	return 0;
}

static int lirc_pwm_remove(struct platform_device *pdev)
{
	struct lirc_pwm_priv *priv = platform_get_drvdata(pdev);
	int ret;

	dev_dbg(&pdev->dev, "removing\n");

	ret = lirc_unregister_driver(priv->driver->minor);
	if (ret)
		dev_err(&pdev->dev, "unable to unregister from lirc\n");
	else
		dev_info(&pdev->dev, "unregistered from lirc\n");

	return ret;
}

static struct of_device_id lirc_pwm_dt_ids[] = {
	{ .compatible = "lirc-pwm" },
	{}
};
MODULE_DEVICE_TABLE(of, lirc_pwm_dt_ids);

static struct platform_driver lirc_pwm_driver = {
	.driver = {
		.name	= DRIVER_NAME,
		.of_match_table = of_match_ptr(lirc_pwm_dt_ids),
	},
	.probe = lirc_pwm_probe,
	.remove	= lirc_pwm_remove,
};

module_platform_driver(lirc_pwm_driver);

MODULE_DESCRIPTION("LIRC PWM Transmitter driver");
MODULE_AUTHOR("Alexey Ignatov <lexszero@gmail.com>");
MODULE_LICENSE("GPL");
