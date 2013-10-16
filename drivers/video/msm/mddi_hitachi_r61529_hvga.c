/* drivers/video/msm/mddi_hitachi_r61529_hvga.c
 *
 * Copyright (C) 2010 Sony Ericsson Mobile Communications AB.
 *
 * Author: Johan Olson <johan.olson@sonyericsson.com>
 * Adapted for SEMC 2011 devices by Michael Bestas <mikeioannina@gmail.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2, as
 * published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 */

#include "msm_fb.h"
#include "mddihost.h"
#include "mddihosti.h"
#include <linux/fb.h>
#include <linux/delay.h>
#include <linux/mutex.h>
#include <generated/autoconf.h>
#include <linux/mddi_hitachi_r61529_hvga.h>

#if defined(CONFIG_MACH_SEMC_SATSUMA) || defined(CONFIG_MACH_SEMC_SMULTRON)
	#define REFRESH_RATE 6700
#elif defined(CONFIG_MACH_SEMC_MANGO)
	#define REFRESH_RATE 6200
#else
	#define REFRESH_RATE 6500
#endif

/* Internal version number */
#define MDDI_DRIVER_VERSION 0x0004

/* Driver_IC_ID value for display */
#define MDDI_HITACHI_DISPLAY_DRIVER_IC_ID 0x01

#define POWER_OFF 0
#define POWER_ON  1

/* Function protos */
static ssize_t show_driver_version(
	struct device *pdev,
	struct device_attribute *attr,
	char *buf);

static ssize_t show_dbc_ctrl(
	struct device *pdev,
	struct device_attribute *attr,
	char *buf);

static ssize_t store_dbc_ctrl(
	struct device *pdev,
	struct device_attribute *attr,
	const char *buf,
	size_t count);

/* Function Configuration */
#define DBC_OFF 0
#define DBC_ON	1

static int dbc_ctrl = DBC_ON;
module_param(dbc_ctrl, int, S_IRUSR | S_IWUSR | S_IRGRP | S_IROTH);
MODULE_PARM_DESC(dbc_ctrl, "Dynamic Backlight Control DBC_OFF = 0, DBC_ON = 1");

/* driver attributes */
static DEVICE_ATTR(display_driver_version, 0444, show_driver_version, NULL);
static DEVICE_ATTR(dbc_ctrl, 0644, show_dbc_ctrl, store_dbc_ctrl);

static DEFINE_MUTEX(mddi_mutex);

enum lcd_registers {
	LCD_REG_COLUMN_ADDRESS = 0x2a,
	LCD_REG_PAGE_ADDRESS = 0x2b,
	LCD_REG_DRIVER_IC_ID = 0xA1,
	LCD_REG_MODULE_ID = 0xA8,
	LCD_REG_REVISION_ID = 0xA8
};

enum mddi_hitachi_lcd_state {
	LCD_STATE_OFF,
	LCD_STATE_POWER_ON,
	LCD_STATE_DISPLAY_OFF,
	LCD_STATE_ON,
	LCD_STATE_SLEEP
};

struct coords {
	u16 x1;
	u16 x2;
	u16 y1;
	u16 y2;
};

struct panel_ids {
	u32 driver_ic_id;
	u32 module_id;
	u32 revision_id;
};

static struct msm_fb_panel_data hitachi_hvga_panel_data;

struct hitachi_record {
	struct hitachi_hvga_platform_data *pdata;
	struct mutex mddi_mutex;
	int power_ctrl;
	enum mddi_hitachi_lcd_state lcd_state;
	struct coords last_window;
	struct platform_device *pdev;
	struct panel_ids pid;
};

#define DBC_CONTROL_SIZE  20
/* maximum 30% setting*/
static u32 dbc_control_on_data[DBC_CONTROL_SIZE] = {
		0x00000001, 0x00000009, 0x00000009, 0x000000FF, 0X000000FF,
		0x000000E1, 0x000000E1, 0x0000000C, 0x00000018, 0X00000010,
		0x00000010, 0x00000037, 0x0000005A, 0x00000087, 0X000000BE,
		0x000000FF, 0x00000000, 0x00000000, 0x00000000, 0X00000000};

static u32 dbc_control_off_data[DBC_CONTROL_SIZE] = {
		0x00000000, 0x00000006, 0x00000006, 0x000000FF, 0X000000FF,
		0x000000ED, 0x000000ED, 0x0000000C, 0x00000018, 0X00000010,
		0x00000010, 0x00000037, 0x0000005A, 0x00000087, 0X000000BE,
		0x000000FF, 0x00000000, 0x00000000, 0x00000000, 0X00000000};

static void hitachi_lcd_dbc_on(void)
{
	if (dbc_ctrl) {
		/* Manufacture Command Access Protect */
		mddi_queue_register_write(0xB0, 0x04, TRUE, 0);

		/* Backlight control1 set */
		mddi_host_register_write_xl(0xB8, dbc_control_on_data,
				DBC_CONTROL_SIZE, TRUE, NULL, MDDI_HOST_PRIM);

		/* Backlight control2 set */
		mddi_host_register_write16(0xB9, 0x00000000, 0x000000F2,
				0x00000001, 0x00000008, 4,
				TRUE, NULL, MDDI_HOST_PRIM);

		/* Manufacture Command Access Protect */
		mddi_queue_register_write(0xB0, 0x03, TRUE, 0);
	}
}

static void hitachi_lcd_dbc_off(void)
{
	if (dbc_ctrl) {
		/* Manufacture Command Access Protect */
		mddi_queue_register_write(0xB0, 0x04, TRUE, 0);

		/* Backlight control1 set */
		mddi_host_register_write_xl(0xB8, dbc_control_off_data,
				DBC_CONTROL_SIZE, TRUE, NULL, MDDI_HOST_PRIM);

		/* Backlight control2 set */
		mddi_host_register_write16(0xB9, 0x00000000, 0x000000FF,
				0x00000002, 0x00000008, 4,
				TRUE, NULL, MDDI_HOST_PRIM);

		/* Manufacture Command Access Protect */
		mddi_queue_register_write(0xB0, 0x03, TRUE, 0);
	}
}

static void hitachi_lcd_window_address_set(enum lcd_registers reg,
						u16 start, u16 stop)
{
	uint32 para;

	para = start;
	para = (para << 16) | (start + stop);
	para = swab32(para);
	mddi_queue_register_write(reg, para, TRUE, 0);
}

static void hitachi_lcd_window_adjust(uint16 x1, uint16 x2,
					uint16 y1, uint16 y2)
{
	hitachi_lcd_window_address_set(LCD_REG_COLUMN_ADDRESS, x1, x2);
	hitachi_lcd_window_address_set(LCD_REG_PAGE_ADDRESS, y1, y2);

	/* Workaround: 0x3Ch at start of column bug */
	mddi_queue_register_write(0x3C, 0x00, TRUE, 0);
}

static void hitachi_lcd_enter_sleep(void)
{
	/* Set tear off */
	mddi_queue_register_write(0x34, 0x00, TRUE, 0);

	/* Enter sleep mode */
	mddi_queue_register_write(0x10, 0x00, TRUE, 0);
	mddi_wait(100);/* >90ms */
}

static void hitachi_lcd_exit_sleep(struct hitachi_record *rd)
{
	/* Set address mode */
	mddi_queue_register_write(0x36, 0x00, TRUE, 0);

	/* Set pixel format */
	mddi_queue_register_write(0x3A, 0x77, TRUE, 0);

	/* Exit sleep mode */
	mddi_queue_register_write(0x11, 0x00, TRUE, 0);

	mddi_wait(110);/* >108ms */

	/* Set tear on */
	mddi_queue_register_write(0x35, 0x00, TRUE, 0);
}

static void hitachi_lcd_display_on(void)
{
	/* Display on */
	mddi_queue_register_write(0x29, 0x00, TRUE, 0);
}

static void hitachi_lcd_display_off(void)
{
	/* Display off */
	mddi_queue_register_write(0x28, 0x00, TRUE, 0);
	mddi_wait(21); /* >20 ms */
}

static void hitachi_lcd_enter_deepstandby(void)
{
	mddi_queue_register_write(0xB0, 0x00, TRUE, 0);
	mddi_queue_register_write(0xB1, 0x01, TRUE, 0);
	mddi_wait(2); /* >1 ms */
}

static void hitachi_toggle_reset(struct hitachi_record *rd)
{
	if (rd->pdata->exit_deep_standby)
		rd->pdata->exit_deep_standby();
}

static void hitachi_lcd_exit_deepstandby(struct hitachi_record *rd)
{
	hitachi_toggle_reset(rd);
}

static void hitachi_power_on(struct hitachi_record *rd)
{
	if (rd->pdata->power_on)
		rd->pdata->power_on();
}

static void hitachi_power_off(struct hitachi_record *rd)
{
	if (rd->pdata->power_off)
		rd->pdata->power_off();
}

static struct hitachi_record *get_hitachi_record_from_mfd(
						struct platform_device *pdev)
{
	struct msm_fb_data_type *mfd;

	if (!pdev)
		return NULL;
	mfd = platform_get_drvdata(pdev);
	if (mfd == NULL)
		return NULL;
	if (mfd->key != MFD_KEY || mfd->panel_pdev == NULL)
		return NULL;

	return platform_get_drvdata(mfd->panel_pdev);
}

static int mddi_hitachi_ic_on_panel_off(struct platform_device *pdev)
{
	int ret = 0;
	struct hitachi_record *rd;

	rd = get_hitachi_record_from_mfd(pdev);
	if (!rd) {
		ret = -ENODEV;
		goto error;
	}

	mutex_lock(&rd->mddi_mutex);
	if (rd->power_ctrl) {
		switch (rd->lcd_state) {
		case LCD_STATE_OFF:
			hitachi_power_on(rd);
			rd->lcd_state = LCD_STATE_POWER_ON;
			break;

		case LCD_STATE_POWER_ON:
			hitachi_lcd_exit_sleep(rd);
			hitachi_lcd_dbc_on();
			rd->lcd_state = LCD_STATE_DISPLAY_OFF;
			break;

		case LCD_STATE_SLEEP:
			hitachi_lcd_exit_deepstandby(rd);
			hitachi_lcd_exit_sleep(rd);
			hitachi_lcd_dbc_on();
			rd->lcd_state = LCD_STATE_DISPLAY_OFF;
			break;

		default:
			break;
		}
	}
	mutex_unlock(&rd->mddi_mutex);
error:
	return ret;
}

static int mddi_hitachi_ic_on_panel_on(struct platform_device *pdev)
{
	int ret = 0;
	struct hitachi_record *rd;

	rd = get_hitachi_record_from_mfd(pdev);
	if (!rd) {
		ret = -ENODEV;
		goto error;
	}

	mutex_lock(&rd->mddi_mutex);
	if (rd->power_ctrl) {
		switch (rd->lcd_state) {
		case LCD_STATE_POWER_ON:
			hitachi_lcd_exit_sleep(rd);
			hitachi_lcd_dbc_on();
			hitachi_lcd_display_on();
			rd->lcd_state = LCD_STATE_ON;
			break;

		case LCD_STATE_SLEEP:
			hitachi_lcd_exit_deepstandby(rd);
			hitachi_lcd_exit_sleep(rd);
			hitachi_lcd_dbc_on();
			hitachi_lcd_display_on();
			rd->lcd_state = LCD_STATE_ON;
			break;

		case LCD_STATE_DISPLAY_OFF:
			hitachi_lcd_display_on();
			rd->lcd_state = LCD_STATE_ON;
			break;

		default:
			break;
		}
	}
	mutex_unlock(&rd->mddi_mutex);
error:
	return ret;
}

static int mddi_hitachi_ic_off_panel_off(struct platform_device *pdev)
{
	int ret = 0;
	struct hitachi_record *rd;

	rd = get_hitachi_record_from_mfd(pdev);
	if (!rd) {
		ret = -ENODEV;
		goto error;
	}

	mutex_lock(&rd->mddi_mutex);
	if (rd->power_ctrl) {
		switch (rd->lcd_state) {
		case LCD_STATE_POWER_ON:
			hitachi_power_off(rd);
			rd->lcd_state = LCD_STATE_OFF;
			break;

		case LCD_STATE_ON:
			hitachi_lcd_display_off();
			hitachi_lcd_dbc_off();
			hitachi_lcd_enter_sleep();
			hitachi_lcd_enter_deepstandby();
			rd->lcd_state = LCD_STATE_SLEEP;
			break;

		case LCD_STATE_SLEEP:
			hitachi_power_off(rd);
			rd->lcd_state = LCD_STATE_OFF;
			break;

		case LCD_STATE_DISPLAY_OFF:
			hitachi_lcd_dbc_off();
			hitachi_lcd_enter_sleep();
			hitachi_lcd_enter_deepstandby();
			rd->lcd_state = LCD_STATE_SLEEP;
			break;

		case LCD_STATE_OFF:
			break;

		default:
			break;
		}
	}
	mutex_unlock(&rd->mddi_mutex);
error:
	return 0;
}

static ssize_t show_driver_version(struct device *dev_p,
			struct device_attribute *attr,
			char *buf)
{
	return snprintf(buf, PAGE_SIZE, "R61529 Hitachi HVGA, drv ver:0x%x\n",
							MDDI_DRIVER_VERSION);
}

static ssize_t show_dbc_ctrl(struct device *dev_p,
			struct device_attribute *attr,
			char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%i\n", dbc_ctrl);
}

static ssize_t store_dbc_ctrl(struct device *dev_p,
			struct device_attribute *attr,
			const char *buf,
			size_t count)
{
	ssize_t ret;

	mutex_lock(&mddi_mutex);

	if (sscanf(buf, "%i", &ret) != 1) {
		pr_err("%s: mddi_hitachi_hvga: "
			"Invalid flag for dbc ctrl\n", __func__);
		ret = -EINVAL;
		goto unlock;
	}

	if (ret)
		dbc_ctrl = 1;
	else
		dbc_ctrl = 0;

	ret = strnlen(buf, count);

unlock:
	mutex_unlock(&mddi_mutex);
	return ret;
}

static void lcd_attribute_register(struct platform_device *pdev)
{
	int ret;

	ret = device_create_file(&pdev->dev, &dev_attr_display_driver_version);
	if (ret != 0)
		dev_err(&pdev->dev, "Failed to register display_driver_version"
						"attributes (%d)\n", ret);

	ret = device_create_file(&pdev->dev, &dev_attr_dbc_ctrl);
	if (ret != 0)
		dev_err(&pdev->dev, "Failed to register dbc_ctrl"
						"attributes (%d)\n", ret);
}

static int check_panel_ids(struct hitachi_record *rd)
{
	int ret = 0;

	mutex_lock(&rd->mddi_mutex);

	ret = mddi_host_register_read(LCD_REG_DRIVER_IC_ID,
					&rd->pid.driver_ic_id,
					1, MDDI_HOST_PRIM);
	if (ret < 0) {
		dev_err(&rd->pdev->dev, "%s: mddi_hitachi_hvga: "
			"Failed to read Display ID\n", __func__);
		ret = -ENODEV;
		goto error;
	}
	if ((rd->pid.driver_ic_id & 0xFF) !=
			MDDI_HITACHI_DISPLAY_DRIVER_IC_ID) {
		dev_err(&rd->pdev->dev, "%s: mddi_hitachi_hvga: "
			"Detected a non-Hitachi display\n", __func__);
		ret = -ENODEV;
		goto error;
	}

	ret = mddi_host_register_read(LCD_REG_MODULE_ID,
					&rd->pid.module_id,
					1, MDDI_HOST_PRIM);
	if (ret < 0)
		dev_err(&rd->pdev->dev, "%s: mddi_hitachi_hvga: "
			"Failed to read LCD_REG_MODULE_ID\n", __func__);

	ret = mddi_host_register_read(LCD_REG_REVISION_ID,
					&rd->pid.revision_id,
					1, MDDI_HOST_PRIM);
	if (ret < 0)
		dev_err(&rd->pdev->dev, "%s: mddi_hitachi_hvga: "
			"Failed to read LCD_REG_REVISION_ID\n", __func__);

	dev_info(&rd->pdev->dev, "Found display with module ID = 0x%x, "
			"revision ID = 0x%x, driver IC ID = 0x%x, "
			"driver ID = 0x%x\n",
			rd->pid.module_id & 0xFF,
			rd->pid.revision_id & 0xFF,
			rd->pid.driver_ic_id & 0xFF,
			MDDI_DRIVER_VERSION);

error:
	mutex_unlock(&rd->mddi_mutex);
	return ret;
}

static int mddi_hitachi_lcd_probe(struct platform_device *pdev)
{
	int ret = -ENODEV;
	struct hitachi_record *rd;
	struct msm_fb_panel_data *panel_data;

	if (!pdev) {
		dev_err(&pdev->dev, "%s: no platform_device\n", __func__);
		ret = -ENODEV;
		goto exit_point;
	}
	if (!pdev->dev.platform_data) {
		dev_err(&pdev->dev, "%s: no platform data\n", __func__);
		ret = -ENODEV;
		goto exit_point;
	}

	rd = kzalloc(sizeof(struct hitachi_record), GFP_KERNEL);
	if (rd == NULL) {
		ret = -ENOMEM;
		goto exit_point;
	}

	rd->pdev = pdev;
	rd->pdata = pdev->dev.platform_data;
	platform_set_drvdata(pdev, rd);
	mutex_init(&rd->mddi_mutex);

	panel_data = &hitachi_hvga_panel_data;

	if (!check_panel_ids(rd)) {
		rd->lcd_state = LCD_STATE_POWER_ON;
		rd->power_ctrl = POWER_ON;

		panel_data->on = mddi_hitachi_ic_on_panel_off;
		panel_data->controller_on_panel_on = mddi_hitachi_ic_on_panel_on;
		panel_data->off = mddi_hitachi_ic_off_panel_off;
		panel_data->window_adjust = hitachi_lcd_window_adjust;
		panel_data->power_on_panel_at_pan = 0;

		pdev->dev.platform_data = &hitachi_hvga_panel_data;

		/* adds mfd on driver_data */
		msm_fb_add_device(pdev);

		/* Add SYSFS to module */
		lcd_attribute_register(pdev);

		dev_info(&pdev->dev, "%s: Probe success!", __func__);
		ret = 0;
	} else {
		kfree(rd);
	}
exit_point:
	return ret;
}

static int __devexit mddi_hitachi_lcd_remove(struct platform_device *pdev)
{
	struct hitachi_record *rd;

	device_remove_file(&pdev->dev, &dev_attr_display_driver_version);
	device_remove_file(&pdev->dev, &dev_attr_dbc_ctrl);

	rd = platform_get_drvdata(pdev);
	if (rd)
		kfree(rd);
	return 0;
};

static struct platform_driver this_driver = {
	.probe  = mddi_hitachi_lcd_probe,
	.remove = __devexit_p(mddi_hitachi_lcd_remove),
	.driver = {
		.name = MDDI_HITACH_R61529_HVGA_NAME,
	},
};

static void __init msm_mddi_hitachi_hvga_display_device_init(void)
{
	struct msm_fb_panel_data *panel_data = &hitachi_hvga_panel_data;

	panel_data->panel_info.xres = 320;
	panel_data->panel_info.yres = 480;
	panel_data->panel_info.pdest = DISPLAY_1;
	panel_data->panel_info.type = MDDI_PANEL;
	panel_data->panel_info.mddi.vdopkt = MDDI_DEFAULT_PRIM_PIX_ATTR;
	panel_data->panel_info.wait_cycle = 0;
	panel_data->panel_info.bpp = 24;
	panel_data->panel_info.clk_rate = 192000000;
	panel_data->panel_info.clk_min = 190000000;
	panel_data->panel_info.clk_max = 200000000;
	panel_data->panel_info.fb_num = 2;
	panel_data->panel_info.bl_max = 4;
	panel_data->panel_info.bl_min = 1;
	panel_data->panel_info.width = 42;
	panel_data->panel_info.height = 63;

	panel_data->panel_info.lcd.vsync_enable = TRUE;
	panel_data->panel_info.lcd.refx100 = REFRESH_RATE;
	panel_data->panel_info.lcd.v_back_porch = 8;
	panel_data->panel_info.lcd.v_front_porch = 8;
	panel_data->panel_info.lcd.v_pulse_width = 0;
	panel_data->panel_info.lcd.hw_vsync_mode = TRUE;
	panel_data->panel_info.lcd.vsync_notifier_period = 0;
}

static int __init mddi_hitachi_lcd_init(void)
{
	msm_mddi_hitachi_hvga_display_device_init();
	return platform_driver_register(&this_driver);
}

static void __exit mddi_hitachi_lcd_exit(void)
{
	platform_driver_unregister(&this_driver);
}

MODULE_LICENSE("GPL");
MODULE_AUTHOR("johan.olson@sonyericsson.com");
MODULE_DESCRIPTION("Driver for Renesas R61529 with Hitachi HVGA panel");

module_init(mddi_hitachi_lcd_init);
module_exit(mddi_hitachi_lcd_exit);
