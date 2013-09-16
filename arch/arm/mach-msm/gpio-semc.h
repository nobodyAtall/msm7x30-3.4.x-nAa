/* arm/mach-msm/gpio-semc.h
 *
 * Copyright (C) 2010 Sony Ericsson Mobile Communications AB.
 * Adapted for SEMC 2011 devices by Michael Bestas (mikeioannina@gmail.com)
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2, as
 * published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 */

#ifndef _GPIO_SEMC_H
#define _GPIO_SEMC_H

extern struct msm_gpio qsd_spi_gpio_config_data[];
extern int qsd_spi_gpio_config_data_size;

extern struct msm_gpio msm_i2c_gpios_io[];
extern struct msm_gpio msm_i2c_gpios_hw[];
extern int msm_i2c_gpios_hw_size;

extern struct msm_gpio qup_i2c_gpios_io[];
extern struct msm_gpio qup_i2c_gpios_hw[];
extern int qup_i2c_gpios_hw_size;

#endif /* _GPIO_SEMC_H */
