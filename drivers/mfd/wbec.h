#ifndef __WBEC_H
#define __WBEC_H

#include <linux/regulator/machine.h>
#include <linux/regmap.h>

#define WBEC_REG_POWERCTRL			0x10
#define WBEC_REG_BIT_POWEROFF		BIT(0)

/* INTERRUPT REGISTER */
// TODO
#define WBEC_REG_IRQ_FLAGS			49
#define WBEC_REG_IRQ_MSK			50

struct wbec {
	struct i2c_client		*i2c;
	struct regmap			*regmap_8;
	struct regmap			*regmap_16;
	const struct regmap_config	*regmap_cfg_8;
	const struct regmap_config	*regmap_cfg_16;
};

#endif
