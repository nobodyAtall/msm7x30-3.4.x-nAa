/* /kernel/arch/arm/mach-msm/touch-iyohan.c
 *
 * Copyright (C) [2010-2011] Sony Ericsson Mobile Communications AB.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2, as
 * published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 */

#include <linux/input.h>
#include <linux/cyttsp.h>
#include "board-semc_mogami-touch.h"

struct cyttsp_platform_data cyttsp_data = {
	.wakeup = cyttsp_wakeup,
	.init = cyttsp_init,
	.mt_sync = input_mt_sync,
	/* TODO: max values should be retrieved from the firmware */
	.maxx = CONFIG_TOUCHSCREEN_CYTTSP_MAX_X,
	.maxy = CONFIG_TOUCHSCREEN_CYTTSP_MAX_Y,
	.maxz = CONFIG_TOUCHSCREEN_CYTTSP_MAX_Z,
	.flags = 0,
	.gen = CY_GEN3,
	.use_st = 0,
	.use_mt = 1,
	.use_trk_id = 0,
	.use_hndshk = 0,
	.use_timer = 0,
	.use_sleep = 1,
	.use_gestures = 0,
	.use_load_file = 1,
	.use_force_fw_update = 0,
	/* activate up groups */
	.gest_set = CY_GEST_KEEP_ASIS,
	/* set active distance */
	.act_dist = CY_ACT_DIST_01,
	/* change act_intrvl to customize the Active power state
	 * scanning/processing refresh interval for Operating mode
	 */
	.act_intrvl = CONFIG_TOUCHSCREEN_CYTTSP_ACT_INTRVL,
	/* change tch_tmout to customize the touch timeout for the
	 * Active power state for Operating mode
	 */
	.tch_tmout = CY_TCH_TMOUT_DFLT,
	/* change lp_intrvl to customize the Low Power power state
	 * scanning/processing refresh interval for Operating mode
	 */
	.lp_intrvl = CY_LP_INTRVL_DFLT,
	.name = CY_SPI_NAME,
	.irq_gpio = 42,
	.reset = cyttsp_xres,
	.idac_gain = 0,
};

