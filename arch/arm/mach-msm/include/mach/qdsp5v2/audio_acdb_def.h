/* Copyright (c) 2010 - 2011, The Linux Foundation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */
#ifndef _MACH_QDSP5_V2_AUDIO_ACDB_DEF_H
#define _MACH_QDSP5_V2_AUDIO_ACDB_DEF_H

/* Define ACDB device ID */
#define ACDB_ID_HANDSET_SPKR				1
#define ACDB_ID_HANDSET_MIC				2
#define ACDB_ID_HEADSET_MIC				3
#define ACDB_ID_HEADSET_SPKR_MONO			4
#define ACDB_ID_HEADSET_SPKR_STEREO			5
#define ACDB_ID_SPKR_PHONE_MIC				6
#define ACDB_ID_SPKR_PHONE_MONO				7
#define ACDB_ID_SPKR_PHONE_STEREO			8
#define ACDB_ID_BT_SCO_MIC				9
#define ACDB_ID_BT_SCO_SPKR				0x0A
#define ACDB_ID_BT_A2DP_SPKR				0x0B
#define ACDB_ID_BT_A2DP_TX				0x10
#define ACDB_ID_TTY_HEADSET_MIC				0x0C
#define ACDB_ID_TTY_HEADSET_SPKR			0x0D
#define ACDB_ID_HEADSET_MONO_PLUS_SPKR_MONO_RX		0x11
#define ACDB_ID_HEADSET_STEREO_PLUS_SPKR_STEREO_RX	0x14
#define ACDB_ID_FM_TX_LOOPBACK				0x17
#define ACDB_ID_FM_TX					0x18
#define ACDB_ID_LP_FM_SPKR_PHONE_STEREO_RX		0x19
#define ACDB_ID_LP_FM_HEADSET_SPKR_STEREO_RX		0x1A
#define ACDB_ID_I2S_RX					0x20
#define ACDB_ID_SPKR_PHONE_MIC_BROADSIDE		0x2B
#define ACDB_ID_HANDSET_MIC_BROADSIDE			0x2C
#define ACDB_ID_SPKR_PHONE_MIC_ENDFIRE			0x2D
#define ACDB_ID_HANDSET_MIC_ENDFIRE			0x2E
#define ACDB_ID_I2S_TX					0x30
#define ACDB_ID_HDMI					0x40
#define ACDB_ID_FM_RX					0x4F
#define ACDB_ID_HANDSET_SPKR_HAC			0x101
#define ACDB_ID_HEADPHONE_MIC				0x103
#define ACDB_ID_SPKR_PHONE_RINGTONE_MONO		0x107
#define ACDB_ID_SPKR_PHONE_RINGTONE_STEREO		0x108
#define ACDB_ID_BT_SCO_HFP_MIC				0x109
#define ACDB_ID_BT_SCO_HFP_SPKR				0x10A
#define ACDB_ID_BT_DSP_SCO_MIC				0x119
#define ACDB_ID_BT_DSP_SCO_SPKR				0x11A
#define ACDB_ID_HEADPHONE_MIC_ENDFIRE			0x12E
#define ACDB_ID_HANDSET_SPKR_SKT			0x201
#define ACDB_ID_HANDSET_MIC_SKT				0x202
#define ACDB_ID_SPKR_PHONE_XLOUD			0x208
#define ACDB_ID_HANDSET_MIC_ENDFIRE_SKT 		0x22E
#define ACDB_ID_HANDSET_SPKR_VZW			0x301
#define ACDB_ID_HANDSET_MIC_VZW				0x302
#define ACDB_ID_HANDSET_MIC_ENDFIRE_VZW		0x32E
/*Replace the max device ID,if any new device is added Specific to RTC only*/
#define ACDB_ID_MAX                                 ACDB_ID_HANDSET_MIC_ENDFIRE_VZW

/* ID used for virtual devices */
#define PSEUDO_ACDB_ID 					0xFFFF

#endif /* _MACH_QDSP5_V2_AUDIO_ACDB_DEF_H */
