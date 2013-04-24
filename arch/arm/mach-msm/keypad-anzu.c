/* /kernel/arch/arm/mach-msm/keypad-anzu.c
 *
 * Copyright (C) [2010] Sony Ericsson Mobile Communications AB.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2, as
 * published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 */

#include <linux/mfd/pmic8058.h>
#include "board-semc_mogami-keypad.h"

static const unsigned int keymap[] = {
	KEY(0, 0, KEY_BACK),
	KEY(0, 1, KEY_HOME),
	KEY(0, 2, KEY_MENU),
	KEY(0, 3, KEY_VOLUMEUP),
	KEY(0, 4, KEY_VOLUMEDOWN),
	KEY(0, 5, KEY_CAMERA),
	KEY(0, 6, KEY_CAMERA_FOCUS),
};

static struct matrix_keymap_data keymap_data = {
	.keymap_size	= ARRAY_SIZE(keymap),
	.keymap		= keymap,
};

static struct pm8xxx_keypad_platform_data surf_keypad_data = {
	.input_name		= PM8058_KEYPAD_DEV,
	.input_phys_device	= PM8058_KEYPAD_PHYS,
	.num_rows		= 1,
	.num_cols		= 7,
	.rows_gpio_start	= PM8058_GPIO_PM_TO_SYS(8),
	.cols_gpio_start	= PM8058_GPIO_PM_TO_SYS(0),
	.debounce_ms		= 10, //{8, 10},
	.scan_delay_ms		= 2,
	.row_hold_ns		= 122000,
	.wakeup			= 1,
	.keymap_data		= &keymap_data,
};

struct pm8xxx_keypad_platform_data *mogami_keypad_data(void)
{
	return &surf_keypad_data;
}
