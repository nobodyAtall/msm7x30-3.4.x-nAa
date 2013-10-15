/* drivers/video/msm/mddi_auo_s6d05a1_hvga.c
 *
 * Copyright (C) 2011 Sony Ericsson Mobile Communications AB.
 *
 * Author: Macro Luo <macro.luo@sonyericsson.com>
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
#include <linux/mddi_auo_s6d05a1_hvga.h>

#define REFRESH_RATE 6500

/* Internal version number */
#define MDDI_DRIVER_VERSION 0x0003

/* Driver_IC_ID value for display */
#define MDDI_AUO_DISPLAY_DRIVER_IC_ID 0xF0

#define POWER_OFF 0
#define POWER_ON  1

/* Byte order,  word0=P4P3P2P1, little endian */
#define write_client_reg_nbr(__X, __Y0, __Y1, __Y2, __Y3, __NBR) \
  mddi_host_register_write16(__X, __Y0, __Y1, __Y2, __Y3, __NBR, \
			TRUE, NULL, MDDI_HOST_PRIM);

#define write_client_reg_xl(__X, __Y, __NBR) \
  mddi_host_register_write_xl(__X, __Y, __NBR, \
			TRUE, NULL, MDDI_HOST_PRIM);

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

static ssize_t show_dbc_mode_ctrl(
	struct device *dev_p,
	struct device_attribute *attr,
	char *buf);

static ssize_t store_dbc_mode_ctrl(
	struct device *dev_p,
	struct device_attribute *attr,
	const char *buf,
	size_t count);

/* Function Configuration */
#define DBC_OFF 0
#define DBC_ON	1

static int dbc_ctrl = DBC_ON;
module_param(dbc_ctrl, int, S_IRUSR | S_IWUSR | S_IRGRP | S_IROTH);
MODULE_PARM_DESC(dbc_ctrl, "Dynamic Backlight Control DBC_OFF = 0, DBC_ON = 1");

#define DBC_MODE_UI	1
#define DBC_MODE_IMAGE	2
#define DBC_MODE_VIDEO	3

static int dbc_mode = DBC_MODE_VIDEO;
module_param(dbc_mode, int, S_IRUSR | S_IWUSR | S_IRGRP | S_IROTH);
MODULE_PARM_DESC(dbc_ctrl, "Dynamic Backlight Mode DBC_MODE_UI = 1, DBC_MODE_IMAGE = 2, DBC_MODE_VIDEO = 3");

/* driver attributes */
static DEVICE_ATTR(display_driver_version, 0444, show_driver_version, NULL);
static DEVICE_ATTR(dbc_ctrl, 0644, show_dbc_ctrl, store_dbc_ctrl);
static DEVICE_ATTR(dbc_mode, 0644, show_dbc_mode_ctrl, store_dbc_mode_ctrl);

enum lcd_registers {
	LCD_REG_COLUMN_ADDRESS = 0x2a,
	LCD_REG_PAGE_ADDRESS = 0x2b,
	LCD_REG_DRIVER_IC_ID = 0xA1,
	LCD_REG_MODULE_ID = 0xDB,
	LCD_REG_REVISION_ID = 0xDC
};

enum mddi_auo_lcd_state {
	LCD_STATE_OFF,
	LCD_STATE_POWER_ON,
	LCD_STATE_DISPLAY_OFF,
	LCD_STATE_ON,
	LCD_STATE_SLEEP
};

struct panel_ids {
	u32 driver_ic_id;
	u32 module_id;
	u32 revision_id;
};

static struct msm_fb_panel_data auo_hvga_panel_data;

struct auo_record {
	struct auo_hvga_platform_data *pdata;
	struct mutex mddi_mutex;
	int power_ctrl;
	enum mddi_auo_lcd_state lcd_state;
	struct platform_device *pdev;
	struct panel_ids pid;
};

static void auo_lcd_dbc_on(void)
{
	if (dbc_ctrl) {
		/* Manual brightness */
		write_client_reg_nbr(0x51, 0x000000FF, 0, 0, 0, 1);

		/* Minimum Brightness */
		write_client_reg_nbr(0x5E, 0x00000000, 0, 0, 0, 1);

		/* Mobile Image Enhancement Mode */
		write_client_reg_nbr(0x55, 0x00000003, 0, 0, 0, 1);

		/* BL Control */
		write_client_reg_nbr(0x53, 0x00000024, 0, 0, 0, 1);
	}
}

static void auo_lcd_dbc_off(void)
{
	if (dbc_ctrl) {
		/* Mobile Image Enhancement Mode */
		write_client_reg_nbr(0x55, 0x00000000, 0, 0, 0, 1);

		/* BL Control */
		write_client_reg_nbr(0x53, 0x00000000, 0, 0, 0, 1);
	}
}

static void auo_lcd_window_address_set(enum lcd_registers reg,
						u16 start, u16 stop)
{
	uint32 para;

	para = start;
	para = (para << 16) | (start + stop);
	para = swab32(para);
	write_client_reg_nbr(reg, para, 0, 0, 0, 1);
}

static void auo_lcd_window_adjust(uint16 x1, uint16 x2,
					uint16 y1, uint16 y2)
{
	auo_lcd_window_address_set(LCD_REG_COLUMN_ADDRESS, x1, x2);
	auo_lcd_window_address_set(LCD_REG_PAGE_ADDRESS, y1, y2);

	/* Workaround: 0x3Ch at start of column bug */
	write_client_reg_nbr(0x3C, 0, 0, 0, 0, 1);
}

static void auo_lcd_enter_sleep(void)
{
	/* Enter sleep mode */
	write_client_reg_nbr(0x10, 0, 0, 0, 0, 1);
	mddi_wait(120); /* >120 ms */
}

static void auo_lcd_exit_sleep(struct auo_record *rd)
{
	/* Exit sleep mode */
	write_client_reg_nbr(0x11, 0x00000000, 0, 0, 0, 1);
	mddi_wait(120); /* >120 ms */
}

static void auo_lcd_display_on(void)
{
	/* Display on */
	write_client_reg_nbr(0x29, 0x00000000, 0, 0, 0, 1);
}

static void auo_lcd_display_off(void)
{
	/* Display off */
	write_client_reg_nbr(0x28, 0, 0, 0, 0, 1);
	mddi_wait(50); /* >50 ms */
}

static void auo_lcd_enter_deepstandby(void)
{
	/* PASSWD2 */
	write_client_reg_nbr(0xF1, 0x00005A5A, 0, 0, 0, 1);

	/* DSTB */
	write_client_reg_nbr(0xDF, 0x00000001, 0, 0, 0, 1);
}

static void auo_toggle_reset(struct auo_record *rd)
{
	if (rd->pdata->exit_deep_standby)
		rd->pdata->exit_deep_standby();
}

static void auo_lcd_exit_deepstandby(struct auo_record *rd)
{
	auo_toggle_reset(rd);
}

static void auo_power_on(struct auo_record *rd)
{
	if (rd->pdata->power_on)
		rd->pdata->power_on();
}

static void auo_power_off(struct auo_record *rd)
{
	if (rd->pdata->power_off)
		rd->pdata->power_off();
}

static struct auo_record *get_auo_record_from_mfd(
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

static int mddi_auo_ic_on_panel_off(struct platform_device *pdev)
{
	int ret = 0;
	struct auo_record *rd;

	rd = get_auo_record_from_mfd(pdev);
	if (!rd) {
		ret = -ENODEV;
		goto error;
	}

	mutex_lock(&rd->mddi_mutex);
	if (rd->power_ctrl) {
		switch (rd->lcd_state) {
		case LCD_STATE_OFF:
			auo_power_on(rd);
			rd->lcd_state = LCD_STATE_POWER_ON;
			break;

		case LCD_STATE_POWER_ON:
			auo_lcd_exit_sleep(rd);
			auo_lcd_dbc_on();
			rd->lcd_state = LCD_STATE_DISPLAY_OFF;
			break;

		case LCD_STATE_SLEEP:
			auo_lcd_exit_deepstandby(rd);
			auo_lcd_exit_sleep(rd);
			auo_lcd_dbc_on();
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

static int mddi_auo_ic_on_panel_on(struct platform_device *pdev)
{
	int ret = 0;
	struct auo_record *rd;

	rd = get_auo_record_from_mfd(pdev);
	if (!rd) {
		ret = -ENODEV;
		goto error;
	}

	mutex_lock(&rd->mddi_mutex);
	if (rd->power_ctrl) {
		switch (rd->lcd_state) {
		case LCD_STATE_POWER_ON:
			auo_lcd_exit_sleep(rd);
			auo_lcd_dbc_on();
			auo_lcd_display_on();
			rd->lcd_state = LCD_STATE_ON;
			break;

		case LCD_STATE_SLEEP:
			auo_lcd_exit_deepstandby(rd);
			auo_lcd_exit_sleep(rd);
			auo_lcd_dbc_on();
			auo_lcd_display_on();
			rd->lcd_state = LCD_STATE_ON;
			break;

		case LCD_STATE_DISPLAY_OFF:
			auo_lcd_display_on();
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

static int mddi_auo_ic_off_panel_off(struct platform_device *pdev)
{
	int ret = 0;
	struct auo_record *rd;

	rd = get_auo_record_from_mfd(pdev);
	if (!rd) {
		ret = -ENODEV;
		goto error;
	}

	mutex_lock(&rd->mddi_mutex);
	if (rd->power_ctrl) {
		switch (rd->lcd_state) {
		case LCD_STATE_POWER_ON:
			auo_power_off(rd);
			rd->lcd_state = LCD_STATE_OFF;
			break;

		case LCD_STATE_ON:
			auo_lcd_display_off();
			auo_lcd_dbc_off();
			auo_lcd_enter_sleep();
			auo_lcd_enter_deepstandby();
			rd->lcd_state = LCD_STATE_SLEEP;
			break;

		case LCD_STATE_SLEEP:
			auo_power_off(rd);
			rd->lcd_state = LCD_STATE_OFF;
			break;

		case LCD_STATE_DISPLAY_OFF:
			auo_lcd_dbc_off();
			auo_lcd_enter_sleep();
			auo_lcd_enter_deepstandby();
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
	return snprintf(buf, PAGE_SIZE, "s6d5a1 AUO HVGA, drv ver:0x%x\n",
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
	struct auo_record *rd;

	rd = kzalloc(sizeof(struct auo_record), GFP_KERNEL);
	if (rd == NULL) {
		ret = -ENOMEM;
		goto exit_point;
	}

	mutex_lock(&rd->mddi_mutex);

	if (sscanf(buf, "%i", &ret) != 1) {
		pr_err("%s: mddi_auo_hvga: "
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
	mutex_unlock(&rd->mddi_mutex);
exit_point:
	return ret;
}

static ssize_t show_dbc_mode_ctrl(struct device *dev_p,
				struct device_attribute *attr,
				char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%i\n", dbc_mode);
}

static ssize_t store_dbc_mode_ctrl(struct device *dev_p,
				struct device_attribute *attr,
				const char *buf,
				size_t count)
{
	ssize_t ret;
	struct auo_record *rd;

	rd = kzalloc(sizeof(struct auo_record), GFP_KERNEL);
	if (rd == NULL) {
		ret = -ENOMEM;
		goto exit_point;
	}

	mutex_lock(&rd->mddi_mutex);

	if (sscanf(buf, "%i", &ret) != 1) {
		pr_err("%s: mddi_auo_hvga: "
			"Invalid flag for dbc mode\n", __func__);
		ret = -EINVAL;
		goto unlock;
	}

	switch (ret) {
	case DBC_MODE_UI:
	case DBC_MODE_IMAGE:
	case DBC_MODE_VIDEO:
		dbc_mode = ret;
		break;
	default:
		pr_err("%s: mddi_auo_hvga: "
			"Invalid value for dbc mode\n", __func__);
		ret = -EINVAL;
		goto unlock;
	}

	if (rd->lcd_state != LCD_STATE_ON) {
		pr_err("%s: mddi_auo_hvga: LCD in sleep, "
			"not performing any dbc change\n", __func__);
		ret = -EINVAL;
		goto unlock;
	}

	/* Mobile Image Enhancement Mode */
	write_client_reg_nbr(0x55, dbc_mode, 0, 0, 0, 1);

	ret = strnlen(buf, count);

unlock:
	mutex_unlock(&rd->mddi_mutex);
exit_point:
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

	ret = device_create_file(&pdev->dev, &dev_attr_dbc_mode);
	if (ret != 0)
		dev_err(&pdev->dev, "Failed to register dbc_mode"
						"attributes (%d)\n", ret);
}

static int check_panel_ids(struct auo_record *rd)
{
	int ret = 0;

	mutex_lock(&rd->mddi_mutex);

	ret = mddi_host_register_read(LCD_REG_DRIVER_IC_ID,
					&rd->pid.driver_ic_id,
					1, MDDI_HOST_PRIM);
	if (ret < 0) {
		dev_err(&rd->pdev->dev, "%s: mddi_auo_hvga: "
			"Failed to read Display ID\n", __func__);
		ret = -ENODEV;
		goto error;
	}
	if ((rd->pid.driver_ic_id & 0xFF) !=
			MDDI_AUO_DISPLAY_DRIVER_IC_ID) {
		dev_err(&rd->pdev->dev, "%s: mddi_auo_hvga: "
			"Detected a non-AUO display\n", __func__);
		ret = -ENODEV;
		goto error;
	}

	ret = mddi_host_register_read(LCD_REG_MODULE_ID,
					&rd->pid.module_id,
					1, MDDI_HOST_PRIM);
	if (ret < 0)
		dev_err(&rd->pdev->dev, "%s: mddi_auo_hvga: "
			"Failed to read LCD_REG_MODULE_ID\n", __func__);

	ret = mddi_host_register_read(LCD_REG_REVISION_ID,
					&rd->pid.revision_id,
					1, MDDI_HOST_PRIM);
	if (ret < 0)
		dev_err(&rd->pdev->dev, "%s: mddi_auo_hvga: "
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

static int mddi_auo_lcd_probe(struct platform_device *pdev)
{
	int ret = -ENODEV;
	struct auo_record *rd;
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

	rd = kzalloc(sizeof(struct auo_record), GFP_KERNEL);
	if (rd == NULL) {
		ret = -ENOMEM;
		goto exit_point;
	}

	rd->pdev = pdev;
	rd->pdata = pdev->dev.platform_data;
	platform_set_drvdata(pdev, rd);
	mutex_init(&rd->mddi_mutex);

	panel_data = &auo_hvga_panel_data;

	if (!check_panel_ids(rd)) {
		rd->lcd_state = LCD_STATE_POWER_ON;
		rd->power_ctrl = POWER_ON;

		panel_data->on = mddi_auo_ic_on_panel_off;
		panel_data->controller_on_panel_on = mddi_auo_ic_on_panel_on;
		panel_data->off = mddi_auo_ic_off_panel_off;
		panel_data->window_adjust = auo_lcd_window_adjust;
		panel_data->power_on_panel_at_pan = 0;

		pdev->dev.platform_data = &auo_hvga_panel_data;

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

static int __devexit mddi_auo_lcd_remove(struct platform_device *pdev)
{
	struct auo_record *rd;

	device_remove_file(&pdev->dev, &dev_attr_display_driver_version);
	device_remove_file(&pdev->dev, &dev_attr_dbc_ctrl);
	device_remove_file(&pdev->dev, &dev_attr_dbc_mode);

	rd = platform_get_drvdata(pdev);
	if (rd)
		kfree(rd);
	return 0;
};

static struct platform_driver this_driver = {
	.probe  = mddi_auo_lcd_probe,
	.remove = __devexit_p(mddi_auo_lcd_remove),
	.driver = {
		.name = MDDI_AUO_S6D05A1_HVGA_NAME,
	},
};

static void __init msm_mddi_auo_hvga_display_device_init(void)
{
	struct msm_fb_panel_data *panel_data = &auo_hvga_panel_data;

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

static int __init mddi_auo_lcd_init(void)
{
	msm_mddi_auo_hvga_display_device_init();
	return platform_driver_register(&this_driver);
}

static void __exit mddi_auo_lcd_exit(void)
{
	platform_driver_unregister(&this_driver);
}

MODULE_LICENSE("GPL");
MODULE_AUTHOR("macro.luo@sonyericsson.com");
MODULE_DESCRIPTION("Driver for Samsung S6D05A1X01 with AUO HVGA panel");

module_init(mddi_auo_lcd_init);
module_exit(mddi_auo_lcd_exit);
