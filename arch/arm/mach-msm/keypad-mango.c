/* arch/arm/mach-msm/keypad-mango.c
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
	KEY(0, 1, KEY_CAMERA_FOCUS),        /* camera AF */
	KEY(0, 2, KEY_COMPOSE),         /* Symbol */
	KEY(0, 3, KEY_SPACE),
	KEY(0, 4, KEY_ENTER),
	KEY(0, 5, KEY_DOT),
	KEY(0, 6, KEY_UP),

	KEY(1, 0, KEY_HOME),
	KEY(1, 1, KEY_CAMERA),
	KEY(1, 3, KEY_A),
	KEY(1, 4, KEY_D),
	KEY(1, 5, KEY_APOSTROPHE),         /* camera AF */
	KEY(1, 6, KEY_DOWN),

	KEY(2, 1, KEY_VOLUMEDOWN),
	KEY(2, 3, KEY_B),
	KEY(2, 5, KEY_RIGHTBRACE),        /* ?/! */
	KEY(2, 7, KEY_RIGHT),

	KEY(3, 0, KEY_LEFTSHIFT),
	KEY(3, 1, KEY_VOLUMEUP),
	KEY(3, 2, KEY_F),
	KEY(3, 3, KEY_COMMA),
	KEY(3, 4, KEY_Z),
	KEY(3, 5, KEY_E),
	KEY(3, 7, KEY_LEFT),

	KEY(4, 1, KEY_Q),
	KEY(4, 3, KEY_R),
	KEY(4, 4, KEY_C),
	KEY(4, 5, KEY_T),
	KEY(4, 6, KEY_LEFTALT),
	KEY(4, 7, KEY_V),

	KEY(5, 1, KEY_G),
	KEY(5, 2, KEY_LEFTBRACE),        /* @/& */
	KEY(5, 4, KEY_Y),
	KEY(5, 5, KEY_H),
	KEY(5, 6, KEY_LANGUAGE),        /* language */
	KEY(5, 7, KEY_X),

	KEY(6, 1, KEY_U),
	KEY(6, 2, KEY_N),
	KEY(6, 3, KEY_J),
	KEY(6, 5, KEY_I),
	KEY(6, 6, KEY_K),
	KEY(6, 7, KEY_M),

	KEY(7, 0, KEY_W),
	KEY(7, 1, KEY_S),
	KEY(7, 2, KEY_O),
	KEY(7, 4, KEY_L),
	KEY(7, 6, KEY_BACKSPACE),
	KEY(7, 7, KEY_P),
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
	.row_hold_ns		= 122000,
	.wakeup			= 1,
	.keymap_data		= &pm8xxx_keymap_data,
};
