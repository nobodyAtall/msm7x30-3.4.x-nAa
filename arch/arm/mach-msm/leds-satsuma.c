/* /kernel/arch/arm/mach-msm/leds-satsuma.c
 *
 * Copyright 2013 Vassilis Tsogkas (tsogkas@ceid.upatras.gr)
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2, as
 * published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 */

#ifdef CONFIG_LEDS_AS3676
#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/leds-as3676.h>

struct as3676_platform_data as3676_platform_data = {
	.step_up_vtuning = 18,	/* 0 .. 31 uA on DCDC_FB */
	.audio_speed_down = 1,	/* 0..3 resp. 0, 200, 400, 800ms */
	.audio_speed_up = 4,	/* 0..7 resp. 0, 50, 100, 150,
					200,250,400, 800ms */
	.audio_agc_ctrl = 1,	/* 0 .. 7: 0==no AGC, 7 very aggressive*/
	.audio_gain = 7,	/* 0..7: -12, -6,  0, 6
					12, 18, 24, 30 dB */
	.audio_source = 2,	/* 0..3: 0=curr33, 1=DCDC_FB
					2=GPIO1,  3=GPIO2 */
	.step_up_lowcur = true,
	.reset_on_i2c_shutdown = true,
	.caps_mounted_on_dcdc_feedback = 1,
	.cp_control = 0x10,
	.leds[0] = {
		.name = "lcd-backlight",
		.on_charge_pump = 0,
		.max_current_uA = 21000,
		.startup_current_uA = 21000,
		.use_dls = true,
	},
	.leds[1] = {
		.name = "led_2-not-connected",
		.on_charge_pump = 0,
		.max_current_uA = 0,
	},
	.leds[2] = {
		.name = "led_3-not-connected",
		.on_charge_pump = 0,
		.max_current_uA = 0,
	},
	.leds[2] = {
		.name = "led_4-not-connected",
		.on_charge_pump = 0,
		.max_current_uA = 0,
	},
	.leds[4] = {
		.name = "button-backlight-rgb1",
		.on_charge_pump = 0,
		.max_current_uA = 500,
		.startup_current_uA = 500,
	},
	.leds[5] = {
		.name = "led_6-not-connected",
		.on_charge_pump = 0,
		.max_current_uA = 0,
	},
	.leds[6] = {
		.name = "red",
		.on_charge_pump = 1,
		.max_current_uA = 6450,
	},
	.leds[7] = {
		.name = "green",
		.on_charge_pump = 1,
		.max_current_uA = 7950,
	},
	.leds[8] = {
		.name = "blue",
		.on_charge_pump = 1,
		.max_current_uA = 4500,
	},
	.leds[9] = {
		.name = "led_10-not-connected",
		.on_charge_pump = 0,
		.max_current_uA = 0,
	},
	.leds[10] = {
		.name = "led_11-not-connected",
		.on_charge_pump = 0,
		.max_current_uA = 0,
	},
	.leds[11] = {
		.name = "led_12-not-connected",
		.on_charge_pump = 0,
		.max_current_uA = 0,
	},
	.leds[12] = {
		.name = "led_13-not-connected",
		.on_charge_pump = 0,
		.max_current_uA = 0,
	},
};

#endif
