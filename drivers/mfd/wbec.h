#ifndef __WBEC_H
#define __WBEC_H

#include <linux/regulator/machine.h>
#include <linux/regmap.h>
#include <linux/debugfs.h>


/* Region INFO: RW */
#define WBEC_REG_INFO_WBEC_ID                               0
#define WBEC_REG_INFO_BOARD_REV                             1
#define WBEC_REG_INFO_FW_VER_MAJOR                          2
#define WBEC_REG_INFO_FW_VER_MINOR                          3
#define WBEC_REG_INFO_FW_VER_PATCH                          4
#define WBEC_REG_INFO_FW_VER_SUFFIX                         5

/* Region RTC_TIME: RW */
#define WBEC_REG_RTC_TIME_SECONDS                           6
#define WBEC_REG_RTC_TIME_MINUTES                           7
#define WBEC_REG_RTC_TIME_HOURS                             8
#define WBEC_REG_RTC_TIME_DAYS                              9
#define WBEC_REG_RTC_TIME_WEEKDAYS                          10
#define WBEC_REG_RTC_TIME_MONTHS                            11
#define WBEC_REG_RTC_TIME_YEARS                             12

/* Region RTC_ALARM: RW */
#define WBEC_REG_RTC_ALARM_13                               13
   #define WBEC_REG_RTC_ALARM_13_SECONDS_MSK                GENMASK(6, 0)
   #define WBEC_REG_RTC_ALARM_13_EN_MSK                     GENMASK(7, 7)
#define WBEC_REG_RTC_ALARM_MINUTES                          14
#define WBEC_REG_RTC_ALARM_HOURS                            15
#define WBEC_REG_RTC_ALARM_DAYS                             16

/* Region RTC_CFG: RW */
#define WBEC_REG_RTC_CFG_OFFSET                             17

/* Region ADC_DATA: RW */
#define WBEC_REG_ADC_DATA_V_IN                              18
#define WBEC_REG_ADC_DATA_V_BAT                             20
#define WBEC_REG_ADC_DATA_V_3_3                             22
#define WBEC_REG_ADC_DATA_V_5_0                             24
#define WBEC_REG_ADC_DATA_V_A1                              26
#define WBEC_REG_ADC_DATA_V_A2                              28
#define WBEC_REG_ADC_DATA_V_A3                              30
#define WBEC_REG_ADC_DATA_V_A4                              32
#define WBEC_REG_ADC_DATA_TEMP                              34
#define WBEC_REG_ADC_DATA_V_USB_DEBUG_CONSOLE               36
#define WBEC_REG_ADC_DATA_V_USB_DEBUG_NETWORK               38

/* Region ADC_CFG: RW */
#define WBEC_REG_ADC_CFG_LOWPASS_RC_A1                      40
#define WBEC_REG_ADC_CFG_LOWPASS_RC_A2                      41
#define WBEC_REG_ADC_CFG_LOWPASS_RC_A3                      42
#define WBEC_REG_ADC_CFG_LOWPASS_RC_A4                      43
#define WBEC_REG_ADC_CFG_V_IN_UVP                           44
#define WBEC_REG_ADC_CFG_V_IN_OVP                           45
#define WBEC_REG_ADC_CFG_V_OUT_UVP                          46
#define WBEC_REG_ADC_CFG_V_OUT_OVP                          47

/* Region GPIO: RW */
#define WBEC_REG_GPIO_48                                    48
   #define WBEC_REG_GPIO_48_A1_MSK                          GENMASK(0, 0)
   #define WBEC_REG_GPIO_48_A2_MSK                          GENMASK(1, 1)
   #define WBEC_REG_GPIO_48_A3_MSK                          GENMASK(2, 2)
   #define WBEC_REG_GPIO_48_A4_MSK                          GENMASK(3, 3)
   #define WBEC_REG_GPIO_48_V_OUT_MSK                       GENMASK(4, 4)

/* Region WDT: RW */
#define WBEC_REG_WDT_49                                     49
   #define WBEC_REG_WDT_49_TIMEOUT_MSK                      GENMASK(3, 0)
   #define WBEC_REG_WDT_49_RESET_MSK                        GENMASK(4, 4)
   #define WBEC_REG_WDT_49_RUN_MSK                          GENMASK(5, 5)

/* Region POWER_CTRL: RW */
#define WBEC_REG_POWER_CTRL_50                              50
   #define WBEC_REG_POWER_CTRL_50_OFF_MSK                   GENMASK(0, 0)
   #define WBEC_REG_POWER_CTRL_50_PWRKEY_PRESSED_MSK        GENMASK(1, 1)


struct wbec {
	struct i2c_client		*i2c;
	struct regmap			*regmap_8;
	struct regmap			*regmap_16;
	const struct regmap_config	*regmap_cfg_8;
	const struct regmap_config	*regmap_cfg_16;
	struct dentry 			*debug_dir;
};

#endif
