/* arch/arm/mach-msm/keypad-zeus.c
 *
 * Copyright (C) [2010] Sony Ericsson Mobile Communications AB.
 * Adapted for SEMC 2011 devices by Michael Bestas (mikeioannina@gmail.com)
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2, as
 * published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 */

#include <linux/mfd/pmic8058.h>
#include "keypad-semc.h"
/* zeus GPIO keypad/slider - start */
#include <linux/gpio.h>
#include <linux/gpio_event.h>
#include <linux/platform_device.h>
/* zeus GPIO keypad/slider - end */

static const unsigned int pm8xxx_keymap[] = {
	KEY(7, 0, KEY_VOLUMEUP),
	KEY(7, 1, KEY_VOLUMEDOWN),
	KEY(7, 2, BTN_SELECT),
	KEY(7, 3, KEY_ENTER),
};

static struct matrix_keymap_data pm8xxx_keymap_data = {
	.keymap_size	= ARRAY_SIZE(pm8xxx_keymap),
	.keymap		= pm8xxx_keymap,
};

struct pm8xxx_keypad_platform_data pm8xxx_keypad_data = {
	.input_name		= "pm8xxx-keypad",
	.input_phys_device	= "pm8xxx-keypad/input0",
	.num_rows		= 8,
	.num_cols		= 8,
	.rows_gpio_start	= PM8058_GPIO_PM_TO_SYS(8),
	.cols_gpio_start	= PM8058_GPIO_PM_TO_SYS(0),
	.debounce_ms		= 10,
	.scan_delay_ms		= 32,
	.row_hold_ns		= 91500,
	.wakeup			= 1,
	.keymap_data		= &pm8xxx_keymap_data,
};

/* zeus GPIO keypad - start */
#define KEY_L_GPIO		19
#define KEY_R_GPIO		88

#define KEY_A_GPIO		95
#define KEY_B_GPIO		96
#define KEY_C_4_GPIO		97
#define KEY_D_GPIO		98

#define KEY_CU_GPIO		99
#define KEY_CD_GPIO		100
#define KEY_CL_GPIO		101
#define KEY_CR_GPIO		102

#define PSN_KEY_GPIO		108

static struct gpio_event_direct_entry gpio_keypad_map[] = {
	{KEY_L_GPIO, BTN_TL},
	{KEY_R_GPIO, BTN_TR},
	{KEY_A_GPIO, BTN_Y},
	{KEY_B_GPIO, BTN_B},
	{KEY_C_4_GPIO, BTN_A},
	{KEY_D_GPIO, BTN_X},
	{KEY_CU_GPIO, KEY_RIGHT},
	{KEY_CD_GPIO, KEY_LEFT},
	{KEY_CL_GPIO, KEY_UP},
	{KEY_CR_GPIO, KEY_DOWN},
	{PSN_KEY_GPIO, KEY_MEDIA},
};

static struct gpio_event_input_info gpio_keypad_info = {
	.info.func = gpio_event_input_func,
	.flags = 0,
	.type = EV_KEY,
	.keymap = gpio_keypad_map,
	.keymap_size = ARRAY_SIZE(gpio_keypad_map),
	.debounce_time.tv64 = 10 * NSEC_PER_MSEC,
};

static struct gpio_event_info *keypad_info[] = {
	&gpio_keypad_info.info,
};

static int gpio_keypad_power(const struct gpio_event_platform_data *pdata, bool on)
{
	int i;
	int gpio;
	int mgp;
	int rc;

	if (!on) {
		for (i = 0; i < ARRAY_SIZE(gpio_keypad_map); ++i) {
			gpio = gpio_keypad_map[i].gpio;

			mgp = GPIO_CFG(gpio,
				       0,
				       GPIO_CFG_INPUT,
				       GPIO_CFG_PULL_DOWN,
				       GPIO_CFG_2MA);

			rc = gpio_tlmm_config(mgp, GPIO_CFG_DISABLE);
			if (rc)
				pr_err("%s: gpio_tlmm_config (gpio=%d), failed\n",
				       __func__, gpio_keypad_map[i].gpio);
		}
	} else {
		for (i = 0; i < ARRAY_SIZE(gpio_keypad_map); ++i) {
			gpio = gpio_keypad_map[i].gpio;

			mgp = GPIO_CFG(gpio,
				       0,
				       GPIO_CFG_INPUT,
				       GPIO_CFG_PULL_UP,
				       GPIO_CFG_2MA);

			rc = gpio_tlmm_config(mgp, GPIO_CFG_ENABLE);
			if (rc)
				pr_err("%s: gpio_tlmm_config (gpio=%d), failed\n",
				       __func__, gpio_keypad_map[i].gpio);
		}
	}

	return 0;
}

static struct gpio_event_platform_data gpio_keypad_data = {
	.name		= "gpio-keypad",
	.info		= keypad_info,
	.info_count	= ARRAY_SIZE(keypad_info),
	.power          = gpio_keypad_power,
};

struct platform_device gpio_keypad_device = {
	.name	= GPIO_EVENT_DEV_NAME,
	.id	= 0,
	.dev	= {
		.platform_data	= &gpio_keypad_data,
	},
};

static int __init gpio_keypad_device_init(void)
{
	gpio_keypad_power(&gpio_keypad_data,1);

	return platform_device_register(&gpio_keypad_device);
}

static void __exit gpio_keypad_device_exit(void)
{
	platform_device_unregister(&gpio_keypad_device);
}

module_init(gpio_keypad_device_init);
module_exit(gpio_keypad_device_exit);
/* zeus GPIO keypad - end */

/* zeus GPIO slider - start */
#define SLIDE_CLOSED_N_GPIO	180

static struct gpio_event_direct_entry gpio_switch_map[] = {
	{SLIDE_CLOSED_N_GPIO, SW_LID},
};

static struct gpio_event_input_info gpio_switch_info = {
	.info.func = gpio_event_input_func,
	.flags = 0,
	.type = EV_SW,
	.keymap = gpio_switch_map,
	.keymap_size = ARRAY_SIZE(gpio_switch_map),
	.info.no_suspend = true,
};

static struct gpio_event_info *slider_info[] = {
	&gpio_switch_info.info,
};

static struct gpio_event_platform_data gpio_slider_data = {
	.name		= "gpio-slider",
	.info		= slider_info,
	.info_count	= ARRAY_SIZE(slider_info),
};

struct platform_device gpio_slider_device = {
	.name	= GPIO_EVENT_DEV_NAME,
	.id	= 1,
	.dev	= {
		.platform_data	= &gpio_slider_data,
	},
};

static int __init gpio_slider_device_init(void)
{
	int i;
	int rc;
	int gpio;
	int mgp;

	for (i = 0; i < ARRAY_SIZE(gpio_switch_map); ++i) {
		gpio = gpio_switch_map[i].gpio;

		mgp = GPIO_CFG(gpio,
			       0,
			       GPIO_CFG_INPUT,
			       GPIO_CFG_NO_PULL,
			       GPIO_CFG_2MA);

		rc = gpio_tlmm_config(mgp, GPIO_CFG_ENABLE);
		if (rc)
			pr_err("%s: gpio_tlmm_config (gpio=%d), failed\n",
					__func__, gpio_switch_map[i].gpio);
	}

	return platform_device_register(&gpio_slider_device);
}

static void __exit gpio_slider_device_exit(void)
{
	platform_device_unregister(&gpio_slider_device);
}

module_init(gpio_slider_device_init);
module_exit(gpio_slider_device_exit);
/* zeus GPIO slider - end */
