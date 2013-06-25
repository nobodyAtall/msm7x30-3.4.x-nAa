/* arch/arm/mach-msm/charger-haida.c
 *
 * Copyright (C) 2010 Sony Ericsson Mobile Communications AB.
 * Copyright (C) 2013 LegacyXperia Project
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2, as
 * published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 */

#include <linux/battery_chargalg.h>

static struct battery_regulation_vs_temperature id_bat_reg = {
	/* Cold, Normal, Warm, Overheat */
	{5, 45,		55,	127},	/* battery temp */
	{0, 4200,	4000,	0},	/* charge volt */
	{0, USHRT_MAX,	400,	0},	/* charge curr */
};

struct device_data device_data = {
	.id_bat_reg = &id_bat_reg,
	.battery_capacity_mah = 1500,
	.maximum_charging_current_ma = 1050,
};
