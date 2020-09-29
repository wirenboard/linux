#include <linux/platform_device.h>
#include <linux/uio_driver.h>
#include <linux/spinlock.h>
#include <linux/bitops.h>
#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/stringify.h>
#include <linux/pm_runtime.h>
#include <linux/slab.h>

#include <linux/of.h>
#include <linux/of_platform.h>
#include <linux/of_address.h>

#define DRIVER_NAME "imx_gpt_freq"


#ifdef CONFIG_OF
static struct of_device_id imx_gpt_freq_match[] = {
	{.compatible = "fsl,imx-gpt-freq"},
	{ /* Sentinel */ },
};


static ssize_t mydrv_myparam_show(struct device *dev,
        struct device_attribute *attr, char *buf)
{
    // struct mydrv *mydrv = dev_get_drvdata(dev);
    // int len;

    // len = sprintf(buf, "%d\n", mydrv->myparam);
    // if (len <= 0)
    //     dev_err(dev, "mydrv: Invalid sprintf len: %d\n", len);

	static int counter = 0;
	counter++;
    ssize_t len = sprintf(buf, "%d\n", counter);

    return len;
}

static DEVICE_ATTR(myparam, S_IRUGO , mydrv_myparam_show,
                   NULL);

static struct attribute *mydrv_attrs[] = {
    &dev_attr_myparam.attr,
    NULL
};


ATTRIBUTE_GROUPS(mydrv);

static int imx_gpt_freq_probe(struct platform_device *pdev)
{
	printk("probe imx_gpt_freq_probe\n");
	// pdev->dev.groups = mydrv_groups;
	int result = sysfs_create_group(&pdev->dev.kobj,
				&mydrv_group);

	return 0;
}

static int imx_gpt_freq_remove(struct platform_device *pdev)
{
	 sysfs_remove_group(&pdev->dev.kobj,
				&mydrv_group);
	return 0;
}

MODULE_DEVICE_TABLE(of, imx_gpt_freq_match);
module_param_string(of_id, imx_gpt_freq_match[0].compatible, 128, 0);
MODULE_PARM_DESC(of_id, "Openfirmware id of the device to be handled by uio");
#endif

static struct platform_driver imx_gpt_freq = {
	.probe = imx_gpt_freq_probe,
	.remove = imx_gpt_freq_remove,
	.driver = {
		.name = DRIVER_NAME,
		.of_match_table = of_match_ptr(imx_gpt_freq_match),
	},
};

module_platform_driver(imx_gpt_freq);

MODULE_AUTHOR("Magnus Damm");
MODULE_DESCRIPTION("Userspace I/O platform driver with generic IRQ handling");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("platform:" DRIVER_NAME);
