// SPDX-License-Identifier: GPL-2.0-only
/*
 * LED Kernel Default OFF Trigger
 *
 * Copyright 2008 Nick Forbes <nick.forbes@incepta.com>
 * Copy-paste ivz, original Evgeny Boger
 *
 * Based on Richard Purdie's ledtrig-timer.c.
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/leds.h>
#include "../leds.h"

static int defoff_trig_activate(struct led_classdev *led_cdev)
{
	led_set_brightness_nosleep(led_cdev, 0);
	return 0;
}

static struct led_trigger defoff_led_trigger = {
	.name     = "default-off",
	.activate = defoff_trig_activate,
};
module_led_trigger(defoff_led_trigger);

MODULE_AUTHOR("Nick Forbes <nick.forbes@incepta.com>");
MODULE_DESCRIPTION("Default-OFF LED trigger");
MODULE_LICENSE("GPL v2");
