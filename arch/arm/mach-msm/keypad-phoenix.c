/* arch/arm/mach-msm/keypad-phoenix.c
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

static const unsigned int pm8xxx_keymap[] = {
	KEY(7, 0, KEY_VOLUMEUP),
	KEY(7, 1, KEY_VOLUMEDOWN),
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

/* phoenix PMIC keypad - start */
#define BACK_GPIO		17
#define HOME_GPIO		18
#define MENU_GPIO		19
#define SEARCH_GPIO		26

#define NO_BATT_COV_N_GPIO	31

static struct pm_gpio pm_gpio_config = {
	.direction    = PM_GPIO_DIR_IN,
	.pull         = PM_GPIO_PULL_UP_31P5,
	.vin_sel      = PM8058_GPIO_VIN_S3,
	.out_strength = PM_GPIO_STRENGTH_LOW,
	.function     = PM_GPIO_FUNC_NORMAL,
	.inv_int_pol  = 0,
};

static struct keypad_pmic_key pmic_keypad_map[] = {
	{
		.code = KEY_BACK,
		.irq  = PM8058_GPIO_IRQ(PM8058_IRQ_BASE, BACK_GPIO - 1),
		.gpio = BACK_GPIO,
		.wake = 0,
		.debounce_time.tv64 = 10 * NSEC_PER_MSEC,
	},
	{
		.code = KEY_HOME,
		.irq  = PM8058_GPIO_IRQ(PM8058_IRQ_BASE, HOME_GPIO - 1),
		.gpio = HOME_GPIO,
		.wake = 1,
		.debounce_time.tv64 = 10 * NSEC_PER_MSEC,
	},
	{
		.code = KEY_MENU,
		.irq  = PM8058_GPIO_IRQ(PM8058_IRQ_BASE, MENU_GPIO - 1),
		.gpio = MENU_GPIO,
		.wake = 0,
		.debounce_time.tv64 = 10 * NSEC_PER_MSEC,
	},
	{
		.code = KEY_SEARCH,
		.irq  = PM8058_GPIO_IRQ(PM8058_IRQ_BASE, SEARCH_GPIO - 1),
		.gpio = SEARCH_GPIO,
		.wake = 0,
		.debounce_time.tv64 = 10 * NSEC_PER_MSEC,
	},
	{
		.code = KEY_VENDOR,
		.irq  = PM8058_GPIO_IRQ(PM8058_IRQ_BASE, NO_BATT_COV_N_GPIO - 1),
		.gpio = NO_BATT_COV_N_GPIO,
		.wake = 1,
		.debounce_time.tv64 = 10 * NSEC_PER_MSEC,
	},
};

struct keypad_pmic_platform_data pmic_keypad_data = {
	.keymap = pmic_keypad_map,
	.keymap_size = ARRAY_SIZE(pmic_keypad_map),
	.input_name = "pmic-keypad",
	.pm_gpio_config = &pm_gpio_config,
};
/* phoenix PMIC keypad - end */
