#ifndef __WBEC_H
#define __WBEC_H

#include <linux/regulator/machine.h>
#include <linux/regmap.h>



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
#define WBEC_REG_RTC_ALARM_SECONDS                          13
#define WBEC_REG_RTC_ALARM_MINUTES                          14
#define WBEC_REG_RTC_ALARM_HOURS                            15
#define WBEC_REG_RTC_ALARM_DAYS                             16
#define WBEC_REG_RTC_ALARM_17                               17
   #define WBEC_REG_RTC_ALARM_17_EN_MSK                     GENMASK(0, 0)
   #define WBEC_REG_RTC_ALARM_17_FLAG_MSK                   GENMASK(1, 1)

/* Region RTC_CFG: RW */
#define WBEC_REG_RTC_CFG_OFFSET                             18
#define WBEC_REG_RTC_CFG_RES                                19

/* Region ADC_DATA: RW */
#define WBEC_REG_ADC_DATA_V_IN                              20
#define WBEC_REG_ADC_DATA_V_3_3                             24
#define WBEC_REG_ADC_DATA_V_5_0                             26
#define WBEC_REG_ADC_DATA_V_A1                              28
#define WBEC_REG_ADC_DATA_V_A2                              30
#define WBEC_REG_ADC_DATA_V_A3                              32
#define WBEC_REG_ADC_DATA_V_A4                              34
#define WBEC_REG_ADC_DATA_TEMP                              36
#define WBEC_REG_ADC_DATA_V_USB_CONSOLE                     38
#define WBEC_REG_ADC_DATA_V_USB_NETWORK                     40

/* Region ADC_CFG: RW */
#define WBEC_REG_ADC_CFG_LOWPASS_RC_A1                      42
#define WBEC_REG_ADC_CFG_LOWPASS_RC_A2                      43
#define WBEC_REG_ADC_CFG_LOWPASS_RC_A3                      44
#define WBEC_REG_ADC_CFG_LOWPASS_RC_A4                      45
#define WBEC_REG_ADC_CFG_V_IN_UVP                           46
#define WBEC_REG_ADC_CFG_V_IN_OVP                           47
#define WBEC_REG_ADC_CFG_V_OUT_UVP                          48
#define WBEC_REG_ADC_CFG_V_OUT_OVP                          49

/* Region GPIO: RW */
#define WBEC_REG_GPIO_50                                    50
   #define WBEC_REG_GPIO_50_A1_MSK                          GENMASK(0, 0)
   #define WBEC_REG_GPIO_50_A2_MSK                          GENMASK(1, 1)
   #define WBEC_REG_GPIO_50_A3_MSK                          GENMASK(2, 2)
   #define WBEC_REG_GPIO_50_A4_MSK                          GENMASK(3, 3)
   #define WBEC_REG_GPIO_50_V_OUT_MSK                       GENMASK(4, 4)

/* Region WDT: RW */
#define WBEC_REG_WDT_51                                     51
   #define WBEC_REG_WDT_51_TIMEOUT_MSK                      GENMASK(3, 0)
   #define WBEC_REG_WDT_51_RESET_MSK                        GENMASK(4, 4)
   #define WBEC_REG_WDT_51_RUN_MSK                          GENMASK(5, 5)

/* Region POWER_CTRL: RW */
#define WBEC_REG_POWER_CTRL_52                              52
   #define WBEC_REG_POWER_CTRL_52_OFF_MSK                   GENMASK(0, 0)
   #define WBEC_REG_POWER_CTRL_52_PWRKEY_PRESSED_MSK        GENMASK(1, 1)

/* Region IRQ_FLAGS: RW */
#define WBEC_REG_IRQ_FLAGS_53                               53
   #define WBEC_REG_IRQ_FLAGS_53_RTC_ALARM_MSK              GENMASK(0, 0)
   #define WBEC_REG_IRQ_FLAGS_53_PWROFF_REQ_MSK             GENMASK(1, 1)

/* Region IRQ_MSK: RW */
#define WBEC_REG_IRQ_MSK_54                                 54
   #define WBEC_REG_IRQ_MSK_54_RTC_ALARM_MSK                GENMASK(0, 0)
   #define WBEC_REG_IRQ_MSK_54_PWROFF_REQ_MSK               GENMASK(1, 1)



struct wbec {
	struct i2c_client		*i2c;
	struct regmap			*regmap_8;
	struct regmap			*regmap_16;
	const struct regmap_config	*regmap_cfg_8;
	const struct regmap_config	*regmap_cfg_16;

   struct regmap_irq_chip_data	*irq_data;
};

#endif
