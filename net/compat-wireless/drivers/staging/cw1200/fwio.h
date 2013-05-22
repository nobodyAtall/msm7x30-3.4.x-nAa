/*
 * Firmware API for mac80211 ST-Ericsson CW1200 drivers
 *
 * Copyright (c) 2010, ST-Ericsson
 * Author: Dmitry Tarnyagin <dmitry.tarnyagin@stericsson.com>
 *
 * Based on:
 * ST-Ericsson UMAC CW1200 driver which is
 * Copyright (c) 2010, ST-Ericsson
 * Author: Ajitpal Singh <ajitpal.singh@stericsson.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef FWIO_H_INCLUDED
#define FWIO_H_INCLUDED

#define FIRMWARE_CUT22		("wsm_22.bin")
#define FIRMWARE_CUT20		("wsm_20.bin")
#define FIRMWARE_CUT11		("wsm_11.bin")
#define FIRMWARE_CUT10		("wsm_10.bin")
#define SDD_FILE_22		("sdd_22.bin")
#define SDD_FILE_20		("sdd_20.bin")
#define SDD_FILE_11		("sdd_11.bin")
#define SDD_FILE_10		("sdd_10.bin")

#define CW1200_HW_REV_CUT10	(10)
#define CW1200_HW_REV_CUT11	(11)
#define CW1200_HW_REV_CUT20	(20)
#define CW1200_HW_REV_CUT22	(22)

int cw1200_load_firmware(struct cw1200_common *priv);

#endif
