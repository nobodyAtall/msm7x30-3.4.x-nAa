/* /kernel/arch/arm/mach-msm/board-semc_mogami-keypad.h
 *
 * Copyright (C) 2010 Sony Ericsson Mobile Communications AB.
 * Adapted for SEMC 2011 devices by Vassilis Tsogkas (tsogkas@ceid.upatras.gr)
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2, as
 * published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 */

#ifndef _BOARD_SEMC_MOGAMI_KEYPAD_H
#define _BOARD_SEMC_MOGAMI_KEYPAD_H

#define PM8058_KEYPAD_DEV	"pm8058-keypad"
#define PM8058_KEYPAD_PHYS	"sys/bus/i2c/devices/6-0000"
#define PM8058_GPIO_PM_TO_SYS(pm_gpio)     (pm_gpio + NR_GPIO_IRQS)

struct pm8xxx_keypad_platform_data *mogami_keypad_data(void);

#endif
