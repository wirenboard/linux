#ifndef __WBEC_H
#define __WBEC_H

#include <linux/regulator/machine.h>
#include <linux/regmap.h>

#define DEV_POWER_REG	0x10

#define DEV_OFF		BIT(0)

/* INTERRUPT REGISTER */
// TODO
#define WBEC_REG_INT		0x4C
#define WBEC_REG_INT_MSK		0x4D

struct wbec {
	struct i2c_client		*i2c;
	struct regmap_irq_chip_data	*irq_data;
	struct regmap			*regmap_8;
	struct regmap			*regmap_16;
	const struct regmap_config	*regmap_cfg_8;
	const struct regmap_config	*regmap_cfg_16;
	const struct regmap_irq_chip	*regmap_irq_chip;
};

#endif
