/* Copyright (c) 2009-2013, The Linux Foundation. All rights reserved.
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
 * Adapted for SEMC 2011 devices by Michael Bestas (mikeioannina@gmail.com)
 * Based on SEMC mogami board work by Vassilis Tsogkas (tsogkas@ceid.upatras.gr)
 */

#include <linux/kernel.h>
#include <linux/irq.h>
#include <linux/gpio.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/bootmem.h>
#include <linux/io.h>
#ifdef CONFIG_SPI_QSD
#include <linux/spi/spi.h>
#endif
#include <linux/msm_ssbi.h>
#include <linux/mfd/pmic8058.h>
#include <linux/mfd/marimba.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/power_supply.h>
#include <linux/msm_adc.h>
#include <linux/dma-mapping.h>
#include <linux/regulator/consumer.h>

#include <asm/system_info.h>
#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <asm/setup.h>

#include <mach/mpp.h>
#include <mach/board.h>
#include <mach/camera.h>
#include <mach/memory.h>
#include <mach/msm_iomap.h>
#include <mach/msm_hsusb.h>
#include <mach/msm_spi.h>
#include <mach/qdsp5v2/msm_lpa.h>
#include <mach/dma.h>
#include <linux/android_pmem.h>
#include <mach/pmic.h>
#include <mach/rpc_pmapp.h>
#include <mach/qdsp5v2/aux_pcm.h>
#include <mach/qdsp5v2/mi2s.h>
#include <mach/qdsp5v2/audio_dev_ctl.h>
#include <mach/rpc_server_handset.h>
#include <mach/socinfo.h>
#include <mach/msm_memtypes.h>

#include <asm/mach/mmc.h>
#include <asm/mach/flash.h>
#include <mach/vreg.h>
#include <linux/platform_data/qcom_crypto_device.h>

#include "devices.h"
#include "timer.h"
#ifdef CONFIG_USB_G_ANDROID
#include <linux/usb/android.h>
#include <mach/usbdiag.h>
#endif
#include "pm.h"
#include "pm-boot.h"
#include "spm.h"
#include "acpuclock.h"
#include "clock.h"
#include <mach/dal_axi.h>
#include <mach/msm_serial_hs.h>
#include <mach/qdsp5v2/mi2s.h>
#include <mach/qdsp5v2/audio_dev_ctl.h>
#include "smd_private.h"

#include "board-msm7x30-regulator.h"
#include "pm.h"

#include "gpio-semc.h"
#include "keypad-semc.h"

#ifdef CONFIG_SIMPLE_REMOTE_PLATFORM
#include <mach/simple_remote_msm7x30_pf.h>
#endif
#ifdef CONFIG_SEMC_RPC_SERVER_HANDSET
#include <mach/semc_rpc_server_handset.h>
#endif

#ifdef CONFIG_BATTERY_ZEUS
#include <linux/max17040.h>
#endif
#ifdef CONFIG_BATTERY_SEMC_ARCH
#include <mach/semc_battery_data.h>
#endif
#ifdef CONFIG_USB_MSM_OTG_72K
#include <mach/msm72k_otg.h>
#endif
#ifdef CONFIG_SEMC_CHARGER_USB_ARCH
#include <mach/semc_charger_usb.h>
#endif

#ifdef CONFIG_SENSORS_AKM8975
#include <linux/i2c/akm8975.h>
#endif
#ifdef CONFIG_INPUT_BMA150_NG
#include <linux/bma150_ng.h>
#endif
#ifdef CONFIG_INPUT_GP2A_SEMC
#include <linux/input/gp2ap002a00f_semc.h>
#endif

#ifdef CONFIG_LEDS_AS3676
#include <linux/leds-as3676.h>
#include "leds-semc.h"
#endif

#ifdef CONFIG_FB_MSM_MDDI_NOVATEK_FWVGA
#include <linux/hrtimer.h>
#include <mach/mddi_novatek_fwvga.h>
#endif

#ifdef CONFIG_TOUCHSCREEN_CY8CTMA300
#include <linux/spi/cypress_touch.h>
#endif

#ifdef CONFIG_JOYSTICK_SYNAPTICS
#include <linux/i2c/synaptics_touchpad.h>
#endif

#ifdef CONFIG_SENSORS_AKM8975
#define AKM8975_GPIO			92
#endif
#ifdef CONFIG_INPUT_BMA150_NG
#define BMA150_GPIO			51
#endif
#ifdef CONFIG_INPUT_GP2A_SEMC
#define GP2A_GPIO			20
#endif

#ifdef CONFIG_FB_MSM_MDDI_NOVATEK_FWVGA
#define NOVATEK_GPIO_RESET		157
#endif

#ifdef CONFIG_TOUCHSCREEN_CY8CTMA300
#define CYPRESS_TOUCH_GPIO_RESET	40
#define CYPRESS_TOUCH_GPIO_IRQ		42
#endif

#ifdef CONFIG_JOYSTICK_SYNAPTICS
#define SYNAPTICS_TOUCHPAD_GPIO		33
#endif

#define MSM_PMEM_SF_SIZE  0x2500000
#ifdef CONFIG_FB_MSM_TRIPLE_BUFFER
#define MSM_FB_PRIM_BUF_SIZE   (864 * 480 * 4 * 3) /* 4bpp * 3 Pages */
#else
#define MSM_FB_PRIM_BUF_SIZE   (864 * 480 * 4 * 2) /* 4bpp * 2 Pages */
#endif

#define MSM_FB_SIZE roundup(MSM_FB_PRIM_BUF_SIZE, 4096)

#define MSM_PMEM_ADSP_SIZE      0x1C00000
#define PMEM_KERNEL_EBI0_SIZE   0x600000

#define PMIC_GPIO_INT		27
#define PMIC_GPIO_SD_DET	22

#define PMIC_GPIO_SDC4_PWR_EN_N 24  /* PMIC GPIO Number 25 */

/* Macros assume PMIC GPIOs start at 0 */
#define PM8058_GPIO_PM_TO_SYS(pm_gpio)     (pm_gpio + NR_GPIO_IRQS)
#define PM8058_GPIO_SYS_TO_PM(sys_gpio)    (sys_gpio - NR_GPIO_IRQS)
#define PM8058_MPP_BASE			   PM8058_GPIO_PM_TO_SYS(PM8058_GPIOS)
#define PM8058_MPP_PM_TO_SYS(pm_gpio)	   (pm_gpio + PM8058_MPP_BASE)

#define DDR0_BANK_BASE PHYS_OFFSET
#define DDR0_BANK_SIZE 0X03C00000
#define DDR1_BANK_BASE 0x07400000
#define DDR1_BANK_SIZE 0x08C00000
#define DDR2_BANK_BASE 0X40000000
#define DDR2_BANK_SIZE 0X10000000

/* GPIO hardware device identification */
enum board_hwid {
	BOARD_HWID_UNK,
	BOARD_HWID_DP1,
	BOARD_HWID_SP1,
	BOARD_HWID_DBZ3,
	BOARD_HWID_SP1_5,
	BOARD_HWID_SP1_6,
	BOARD_HWID_DBZ3_1,
	BOARD_HWID_SP2,
	BOARD_HWID_DBZ3_2,
	BOARD_HWID_SP3,
	BOARD_HWID_AP1,
	BOARD_HWID_PQ,
};
static u8 board_hwid;

static unsigned int phys_add = DDR2_BANK_BASE;
unsigned long ebi1_phys_offset = DDR2_BANK_BASE;
EXPORT_SYMBOL(ebi1_phys_offset);

static int vreg_helper_on(const char *pzName, unsigned mv)
{
	struct vreg *reg = NULL;
	int rc = 0;

	reg = vreg_get(NULL, pzName);
	if (IS_ERR(reg)) {
		printk(KERN_ERR "Unable to resolve VREG name \"%s\"\n", pzName);
		return rc;
	}

	if (mv != (unsigned int)-1)
		rc = vreg_set_level(reg, mv);

	if (rc) {
		printk(KERN_ERR "Unable to set vreg \"%s\" level\n", pzName);
		return rc;
	}

	rc = vreg_enable(reg);
	if (rc) {
		printk(KERN_ERR "Unable to enable vreg \"%s\" level\n", pzName);
		return rc;
	}

	printk(KERN_INFO "Enabled VREG \"%s\" at %u mV\n", pzName, mv);
	return rc;
}

static void vreg_helper_off(const char *pzName)
{
	struct vreg *reg = NULL;
	int rc;

	reg = vreg_get(NULL, pzName);
	if (IS_ERR(reg)) {
		printk(KERN_ERR "Unable to resolve VREG name \"%s\"\n", pzName);
		return;
	}

	rc = vreg_disable(reg);
	if (rc) {
		printk(KERN_ERR "Unable to disable vreg \"%s\" level\n",
			pzName);
		return;
	}

	printk(KERN_INFO "Disabled VREG \"%s\"\n", pzName);
}

#ifdef CONFIG_SEMC_RPC_SERVER_HANDSET
static struct input_dev *input_dev_pwr_key = NULL;
static void msm_pmic_pwr_key_rpc_callback(uint32_t key, uint32_t event)
{
	if (!input_dev_pwr_key)
		return;
	switch (key) {
	case HS_PWR_K:
		key = KEY_POWER;
		break;
	case HS_END_K:
		key = KEY_END;
		break;
	default:
		return;
	}
	input_report_key(input_dev_pwr_key, key, event != HS_REL_K);
	input_sync(input_dev_pwr_key);
}

static int __init msm_pmic_pwr_key_init(void)
{
	input_dev_pwr_key = input_allocate_device();
	if (!input_dev_pwr_key) {
		printk(KERN_ERR "%s: Error, unable to alloc pwr key device\n",
			__func__);
		return -1;
	}
	input_dev_pwr_key->name = "msm_pmic_pwr_key";
	input_dev_pwr_key->phys = "semc_rpc_server_handset";
	input_set_capability(input_dev_pwr_key, EV_KEY, KEY_POWER);
	input_set_capability(input_dev_pwr_key, EV_KEY, KEY_END);
	if (input_register_device(input_dev_pwr_key)) {
		printk(KERN_ERR "%s: Error, unable to reg pwr key device\n",
			__func__);
		input_free_device(input_dev_pwr_key);
		return -1;
	}
	return 0;
}
module_init(msm_pmic_pwr_key_init);

/*
 * Add callbacks here. Every defined callback will receive
 * all events. The types are defined in the file
 * semc_rpc_server_handset.h
 */

static handset_cb_array_t semc_rpc_hs_callbacks = {
	&msm_pmic_pwr_key_rpc_callback,
#ifdef CONFIG_SIMPLE_REMOTE_PLATFORM
	&simple_remote_pf_button_handler,
#endif
};

static struct semc_handset_data semc_rpc_hs_data = {
	.callbacks = semc_rpc_hs_callbacks,
	.num_callbacks = ARRAY_SIZE(semc_rpc_hs_callbacks),
};

static struct platform_device semc_rpc_handset_device = {
	.name = SEMC_HANDSET_DRIVER_NAME,
	.id = -1,
	.dev = {
		.platform_data = &semc_rpc_hs_data,
	},
};
#endif

struct pm8xxx_gpio_init_info {
	unsigned			gpio;
	struct pm_gpio			config;
};

static int pm8058_gpios_init(void)
{
	int rc;

	struct pm8xxx_gpio_init_info sdc4_pwr_en = {
		PM8058_GPIO_PM_TO_SYS(PMIC_GPIO_SDC4_PWR_EN_N),
		{
			.direction      = PM_GPIO_DIR_OUT,
			.pull           = PM_GPIO_PULL_NO,
			.vin_sel        = PM8058_GPIO_VIN_L5,
			.function       = PM_GPIO_FUNC_NORMAL,
			.inv_int_pol    = 0,
			.out_strength   = PM_GPIO_STRENGTH_HIGH,
			.output_value   = 0,
		},
	};

	struct pm8xxx_gpio_init_info sdcc_det = {
		PM8058_GPIO_PM_TO_SYS(PMIC_GPIO_SD_DET - 1),
		{
			.direction      = PM_GPIO_DIR_IN,
			.pull           = PM_GPIO_PULL_NO,
			.vin_sel        = 2,
			.function       = PM_GPIO_FUNC_NORMAL,
			.inv_int_pol    = 0,
		},
	};

	rc = pm8xxx_gpio_config(sdcc_det.gpio, &sdcc_det.config);
	if (rc) {
		pr_err("%s PMIC_GPIO_SD_DET config failed\n", __func__);
		return rc;
	}

	/* SDC4 gpio_25 */
	rc = pm8xxx_gpio_config(sdc4_pwr_en.gpio, &sdc4_pwr_en.config);
	if (rc) {
		pr_err("%s PMIC_GPIO_SDC4_PWR_EN_N config failed: %d\n",
		       __func__, rc);
		return rc;
	}

	rc = gpio_request(sdc4_pwr_en.gpio, "sdc4_pwr_en");
	if (rc) {
		pr_err("PMIC_GPIO_SDC4_PWR_EN_N gpio_req failed: %d\n",
		       rc);
		return rc;
	}

	return 0;
}

/* Regulator API support */

#ifdef CONFIG_MSM_PROC_COMM_REGULATOR
static struct platform_device msm_proccomm_regulator_dev = {
	.name = PROCCOMM_REGULATOR_DEV_NAME,
	.id   = -1,
	.dev  = {
		.platform_data = &msm7x30_proccomm_regulator_data
	}
};
#endif

static struct pm8xxx_irq_platform_data pm8xxx_irq_pdata = {
	.irq_base		= PMIC8058_IRQ_BASE,
	.devirq			= MSM_GPIO_TO_INT(PMIC_GPIO_INT),
	.irq_trigger_flag       = IRQF_TRIGGER_LOW,
};

static struct pm8xxx_gpio_platform_data pm8xxx_gpio_pdata = {
	.gpio_base		= PM8058_GPIO_PM_TO_SYS(0),
};

static struct pm8xxx_mpp_platform_data pm8xxx_mpp_pdata = {
	.mpp_base	= PM8058_MPP_PM_TO_SYS(0),
};

static struct pm8xxx_vibrator_platform_data pm8xxx_vibrator_pdata = {
	.initial_vibrate_ms	= 0,
	.level_mV		= CONFIG_PMIC8XXX_VIBRATOR_VOLTAGE,
	.max_timeout_ms		= 15000,
};

static struct pm8xxx_misc_platform_data pm8xxx_misc_pdata = {
	.priority		= 0,
};

static struct pm8058_platform_data pm8058_7x30_data = {
	.irq_pdata		= &pm8xxx_irq_pdata,
	.gpio_pdata		= &pm8xxx_gpio_pdata,
	.mpp_pdata		= &pm8xxx_mpp_pdata,
	.vibrator_pdata		= &pm8xxx_vibrator_pdata,
	.misc_pdata		= &pm8xxx_misc_pdata,
};

#ifdef CONFIG_MSM_SSBI
static struct msm_ssbi_platform_data msm7x30_ssbi_pm8058_pdata = {
	.rsl_id = "D:PMIC_SSBI",
	.controller_type = MSM_SBI_CTRL_SSBI2,
	.slave	= {
		.name			= "pm8058-core",
		.platform_data		= &pm8058_7x30_data,
	},
};
#endif

#ifdef CONFIG_FB_MSM_MDDI_NOVATEK_FWVGA
static const struct panel_id *novatek_panels[] = {
#ifdef CONFIG_FB_MSM_MDDI_NOVATEK_TMD_MDP42
	&novatek_panel_id_tmd_mdp42_rev_c,
	&novatek_panel_id_tmd_mdp42_rev_d,
#endif
#ifdef CONFIG_FB_MSM_MDDI_NOVATEK_SHARP_LS040T8LX01
	&novatek_panel_id_sharp_ls040t8lx01_rev_c_x,	/* SP2.1 support */
	&novatek_panel_id_sharp_ls040t8lx01_rev_c,
	&novatek_panel_id_sharp_ls040t8lx01_rev_d,
#endif
};

struct novatek_i2c_pdata novatek_i2c_pdata = {
	.panels = novatek_panels,
};
#endif

#ifdef CONFIG_INPUT_BMA150_NG
static int bma150_gpio_setup(bool request)
{
	if (request)
		return gpio_request(BMA150_GPIO, "bma150_irq");
	else
		gpio_free(BMA150_GPIO);
	return 0;
}

struct bma150_platform_data bma150_ng_platform_data = {
	.gpio_setup = bma150_gpio_setup,
};
#endif

#ifdef CONFIG_JOYSTICK_SYNAPTICS
static struct msm_gpio synaptics_gpio_config_data[] = {
	{ GPIO_CFG(SYNAPTICS_TOUCHPAD_GPIO, 0, GPIO_CFG_INPUT, GPIO_CFG_PULL_UP, GPIO_CFG_2MA), "synaptics_touchpad_irq" },
};

static int synaptics_touchpad_gpio_setup(void)
{
	int rc;

	rc = msm_gpios_request_enable(synaptics_gpio_config_data,
		ARRAY_SIZE(synaptics_gpio_config_data));

	return rc;
}

static void synaptics_touchpad_gpio_teardown(void)
{
	msm_gpios_disable_free(synaptics_gpio_config_data,
		ARRAY_SIZE(synaptics_gpio_config_data));
}

static struct synaptics_touchpad_platform_data synaptics_touchpad_data = {
	.gpio_setup	= synaptics_touchpad_gpio_setup,
	.gpio_teardown	= synaptics_touchpad_gpio_teardown,
};
#endif

#ifdef CONFIG_INPUT_GP2A_SEMC
static struct msm_gpio gp2a_gpio_config_data[] = {
	{ GPIO_CFG(GP2A_GPIO, 0, GPIO_CFG_INPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA), "gp2a_vo" },
};

static int gp2a_gpio_setup(void)
{
	int rc;
	rc = msm_gpios_request_enable(gp2a_gpio_config_data,
		ARRAY_SIZE(gp2a_gpio_config_data));

	return rc;
}

static int gp2a_gpio_teardown(void)
{
	msm_gpios_disable_free(gp2a_gpio_config_data,
		ARRAY_SIZE(gp2a_gpio_config_data));

	return 0;
}

static struct gp2a_platform_data gp2a_platform_data = {
	.gpio = GP2A_GPIO,
	.wake = 1,
	.gpio_setup = gp2a_gpio_setup,
	.gpio_shutdown = gp2a_gpio_teardown,
};
#endif

static struct i2c_board_info msm_camera_boardinfo[] __initdata = {
#ifdef CONFIG_FB_MSM_MDDI_NOVATEK_FWVGA
	{
		I2C_BOARD_INFO(MDDI_NOVATEK_I2C_NAME, 0x98 >> 1),
		.type = MDDI_NOVATEK_I2C_NAME,
		.platform_data = &novatek_i2c_pdata,
	},
#endif
#ifdef CONFIG_JOYSTICK_SYNAPTICS
	{
		I2C_BOARD_INFO("synaptics_touchpad", 0x40 >> 1),
		.irq		= MSM_GPIO_TO_INT(SYNAPTICS_TOUCHPAD_GPIO),
		.platform_data	= &synaptics_touchpad_data,
	},
#endif
#ifdef CONFIG_INPUT_GP2A_SEMC
	{
		I2C_BOARD_INFO(GP2A_I2C_NAME, 0x88 >> 1),
		.irq		= MSM_GPIO_TO_INT(GP2A_GPIO),
		.platform_data	= &gp2a_platform_data
	},
#endif
#ifdef CONFIG_SEIX006 /* zeus camera */
	{
		I2C_BOARD_INFO("seix006", 0x1A),
		.type = "seix006"
	},
#endif /* CONFIG_SEIX006 */
#ifdef CONFIG_OVM7692 /* zeus vt camera */
	{
		I2C_BOARD_INFO("ovm7692", 0x3C),
		.type = "ovm7692"
	},
#endif /* CONFIG_OVM7692 */
#ifdef CONFIG_MT9V114 /* zeus vt camera version 2 */
	{
		I2C_BOARD_INFO("mt9v114", 0x3D),
		.type = "mt9v114"
	},
#endif /* CONFIG_MT9V114 */
};

#ifdef CONFIG_MSM_CAMERA
#define	CAM_STNDBY	143
static uint32_t camera_off_vcm_gpio_table[] = {
GPIO_CFG(1, 0, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA), /* VCM */
};

static uint32_t camera_off_gpio_table[] = {
	/* parallel CAMERA interfaces */
	/* RST */
	GPIO_CFG(0,  0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
	/* DAT4 */
	GPIO_CFG(4,  0, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA),
	/* DAT5 */
	GPIO_CFG(5,  0, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA),
	/* DAT6 */
	GPIO_CFG(6,  0, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA),
	/* DAT7 */
	GPIO_CFG(7,  0, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA),
	/* DAT8 */
	GPIO_CFG(8,  0, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA),
	/* DAT9 */
	GPIO_CFG(9,  0, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA),
	/* DAT10 */
	GPIO_CFG(10, 0, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA),
	/* DAT11 */
	GPIO_CFG(11, 0, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA),
	/* PCLK */
	GPIO_CFG(12, 0, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA),
	/* HSYNC_IN */
	GPIO_CFG(13, 0, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA),
	/* VSYNC_IN */
	GPIO_CFG(14, 0, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA),
	/* MCLK */
	GPIO_CFG(15, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_6MA),
	/* NIRQ */
	GPIO_CFG(21, 0, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA),
	/* VT VGA_PWDN */
	GPIO_CFG(31, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
	/* STB */
	GPIO_CFG(143, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
	/* LED_IND */
	GPIO_CFG(104, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
	/* LED_TORCH */
	GPIO_CFG(105, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
};

static uint32_t camera_on_vcm_gpio_table[] = {
GPIO_CFG(1, 0, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_2MA), /* VCM */
};

static uint32_t camera_on_gpio_table[] = {
	/* parallel CAMERA interfaces */
	/* RST */
	GPIO_CFG(0,  0, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA),
	/* DAT4 */
	GPIO_CFG(4,  1, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA),
	/* DAT5 */
	GPIO_CFG(5,  1, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA),
	/* DAT6 */
	GPIO_CFG(6,  1, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA),
	/* DAT7 */
	GPIO_CFG(7,  1, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA),
	/* DAT8 */
	GPIO_CFG(8,  1, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA),
	/* DAT9 */
	GPIO_CFG(9,  1, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA),
	/* DAT10 */
	GPIO_CFG(10, 1, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA),
	/* DAT11 */
	GPIO_CFG(11, 1, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA),
	/* PCLK */
	GPIO_CFG(12, 1, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA),
	/* HSYNC_IN */
	GPIO_CFG(13, 1, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA),
	/* VSYNC_IN */
	GPIO_CFG(14, 1, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA),
	/* MCLK */
	GPIO_CFG(15, 1, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_6MA),
	/* NIRQ */
	GPIO_CFG(21, 0, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA),
	/* VT VGA_PWDN */
	GPIO_CFG(31, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
	/* STB */
	GPIO_CFG(143, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
	/* LED_IND */
	GPIO_CFG(104, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
	/* LED_TORCH */
	GPIO_CFG(105, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
};

static void msm_camera_vreg_enable(void)
{
	vreg_helper_on("gp15", 1200); /* L22 */
	vreg_helper_on("lvsw1", 1800); /* LVS1 */
	vreg_helper_on("gp2", 2800); /* L11 */
}

static void msm_camera_vreg_disable(void)
{
	vreg_helper_off("gp2");
	vreg_helper_off("lvsw1");
	vreg_helper_off("gp15");
}

static void config_gpio_table(uint32_t *table, int len)
{
	int n, rc;
	for (n = 0; n < len; n++) {
		rc = gpio_tlmm_config(table[n], GPIO_CFG_ENABLE);
		if (rc) {
			pr_err("%s: gpio_tlmm_config(%#x)=%d\n",
				__func__, table[n], rc);
			break;
		}
	}
}
static int config_camera_on_gpios(void)
{
	gpio_request(31, "vtcam_rst");
	gpio_set_value(31, 1); // SCAM_RST_N
	gpio_free(31);
	config_gpio_table(camera_on_gpio_table,
		ARRAY_SIZE(camera_on_gpio_table));
	msm_camera_vreg_enable();

	config_gpio_table(camera_on_vcm_gpio_table,
		ARRAY_SIZE(camera_on_vcm_gpio_table));

	return 0;
}

static void config_camera_off_gpios(void)
{
	gpio_request(31, "vtcam_rst");
	gpio_set_value(31, 1); // SCAM_RST_N
	gpio_free(31);
	msm_camera_vreg_disable();
	config_gpio_table(camera_off_gpio_table,
		ARRAY_SIZE(camera_off_gpio_table));

	config_gpio_table(camera_off_vcm_gpio_table,
		ARRAY_SIZE(camera_off_vcm_gpio_table));
}

struct gpio_led as3685a_leds[] = {
	{
		.name = "indicator", /*MSM_CAMERA_LED_LOW*/
		.default_trigger = 0,
		.gpio = 104,
		.active_low = 0
	},
	{
		.name = "torch", /*MSM_CAMERA_LED_HIGH*/
		.default_trigger = 0,
		.gpio = 105,
		.active_low = 0
	}
};

struct gpio_led_platform_data as3685a_data = {
	.num_leds = ARRAY_SIZE(as3685a_leds),
	.leds = as3685a_leds
};

struct msm_camera_sensor_flash_src seix006_flash_src = {
	.flash_sr_type = MSM_CAMERA_FLASH_SRC_LED,
	._fsrc.gpio_led_src = &as3685a_data
};

struct resource msm_camera_resources[] = {
	{
		.start	= 0xA6000000,
		.end	= 0xA6000000 + SZ_1M - 1,
		.flags	= IORESOURCE_MEM,
	},
	{
		.start	= INT_VFE,
		.end	= INT_VFE,
		.flags	= IORESOURCE_IRQ,
	},
	{
		.flags  = IORESOURCE_DMA,
	}
};

struct msm_camera_device_platform_data msm_camera_device_data_seix006 = {
	.camera_gpio_on  = config_camera_on_gpios,
	.camera_gpio_off = config_camera_off_gpios,
	.ioext.mdcphy = MSM7X30_MDC_PHYS,
	.ioext.mdcsz  = MSM7X30_MDC_SIZE,
	.ioext.appphy = MSM7X30_CLK_CTL_PHYS,
	.ioext.appsz  = MSM7X30_CLK_CTL_SIZE,
	.ioext.camifpadphy = 0xAB000000,
	.ioext.camifpadsz  = 0x00000400,
	.ioext.csiphy = 0xA6100000,
	.ioext.csisz  = 0x00000400,
	.ioext.csiirq = INT_CSI,
	.ioclk.mclk_clk_rate = 19200000,
	.ioclk.vfe_clk_rate  = 122880000,
};

struct msm_camera_device_platform_data msm_camera_device_data_ovm7692 = {
	.camera_gpio_on  = config_camera_on_gpios,
	.camera_gpio_off = config_camera_off_gpios,
	.ioext.mdcphy = MSM7X30_MDC_PHYS,
	.ioext.mdcsz  = MSM7X30_MDC_SIZE,
	.ioext.appphy = MSM7X30_CLK_CTL_PHYS,
	.ioext.appsz  = MSM7X30_CLK_CTL_SIZE,
	.ioext.camifpadphy = 0xAB000000,
	.ioext.camifpadsz  = 0x00000400,
	.ioext.csiphy = 0xA6100000,
	.ioext.csisz  = 0x00000400,
	.ioext.csiirq = INT_CSI,
	.ioclk.mclk_clk_rate = 24000000,
	.ioclk.vfe_clk_rate  = 122880000,
};

struct msm_camera_device_platform_data msm_camera_device_data_mt9v114 = {
	.camera_gpio_on  = config_camera_on_gpios,
	.camera_gpio_off = config_camera_off_gpios,
	.ioext.mdcphy = MSM7X30_MDC_PHYS,
	.ioext.mdcsz  = MSM7X30_MDC_SIZE,
	.ioext.appphy = MSM7X30_CLK_CTL_PHYS,
	.ioext.appsz  = MSM7X30_CLK_CTL_SIZE,
	.ioext.camifpadphy = 0xAB000000,
	.ioext.camifpadsz  = 0x00000400,
	.ioext.csiphy = 0xA6100000,
	.ioext.csisz  = 0x00000400,
	.ioext.csiirq = INT_CSI,
	.ioclk.mclk_clk_rate = 19200000,
	.ioclk.vfe_clk_rate  = 122880000,
};

#ifdef CONFIG_SEIX006
static struct msm_camera_sensor_flash_data flash_seix006 = {
	.flash_type = MSM_CAMERA_FLASH_LED,
	.flash_src  = &seix006_flash_src
};
static struct msm_camera_sensor_info msm_camera_sensor_seix006_data = {
	.sensor_name    = "seix006",
	.sensor_reset   = 0,
	.sensor_pwd     = 143,
	.vcm_pwd        = 0,
	.pdata          = &msm_camera_device_data_seix006,
	.flash_data     = &flash_seix006,
	.resource       = msm_camera_resources,
	.num_resources  = ARRAY_SIZE(msm_camera_resources)
};

static struct platform_device msm_camera_sensor_seix006 = {
	.name      = "msm_camera_seix006",
	.dev       = {
		.platform_data = &msm_camera_sensor_seix006_data,
	},
};
#endif /*CONFIG_SEIX006 ZEUS 5MAF CAMERA*/

#ifdef CONFIG_OVM7692

static struct msm_camera_sensor_flash_data flash_ovm7692_data = {
	.flash_type = MSM_CAMERA_FLASH_NONE,
	.flash_src  = 0
};

static struct msm_camera_sensor_info msm_camera_sensor_ovm7692_data = {
	.sensor_name    = "ovm7692",
	.sensor_reset   = 0,
	.sensor_pwd     = 31,
	.vcm_pwd        = 0,
	.pdata          = &msm_camera_device_data_ovm7692,
	.flash_data     = &flash_ovm7692_data,
	.resource       = msm_camera_resources,
	.num_resources  = ARRAY_SIZE(msm_camera_resources)
};

static struct platform_device msm_camera_sensor_ovm7692 = {
	.name      = "msm_camera_ovm7692",
	.dev       = {
		.platform_data = &msm_camera_sensor_ovm7692_data,
	},
};
#endif /*CONFIG_OVM7692 ZEUS VGA VT CAMERA*/

#ifdef CONFIG_MT9V114

static struct msm_camera_sensor_flash_data flash_mt9v114_data = {
	.flash_type = MSM_CAMERA_FLASH_NONE,
	.flash_src  = 0
};

static struct msm_camera_sensor_info msm_camera_sensor_mt9v114_data = {
	.sensor_name    = "mt9v114",
	.sensor_reset   = 0,
	.sensor_pwd     = 31,
	.vcm_pwd        = 0,
	.pdata          = &msm_camera_device_data_mt9v114,
	.flash_data     = &flash_mt9v114_data,
	.resource       = msm_camera_resources,
	.num_resources  = ARRAY_SIZE(msm_camera_resources)
};

static struct platform_device msm_camera_sensor_mt9v114 = {
	.name      = "msm_camera_mt9v114",
	.dev       = {
		.platform_data = &msm_camera_sensor_mt9v114_data,
	},
};
#endif /*CONFIG_MT9V114 ZEUS VGA VT CAMERA*/

#ifdef CONFIG_MSM_VPE
static struct resource msm_vpe_resources[] = {
	{
		.start	= 0xAD200000,
		.end	= 0xAD200000 + SZ_1M - 1,
		.flags	= IORESOURCE_MEM,
	},
	{
		.start	= INT_VPE,
		.end	= INT_VPE,
		.flags	= IORESOURCE_IRQ,
	},
};

static struct platform_device msm_vpe_device = {
       .name = "msm_vpe",
       .id   = 0,
       .num_resources = ARRAY_SIZE(msm_vpe_resources),
       .resource = msm_vpe_resources,
};
#endif

#endif /*CONFIG_MSM_CAMERA*/

#ifdef CONFIG_MSM_GEMINI
static struct resource msm_gemini_resources[] = {
	{
		.start  = 0xA3A00000,
		.end    = 0xA3A00000 + 0x0150 - 1,
		.flags  = IORESOURCE_MEM,
	},
	{
		.start  = INT_JPEG,
		.end    = INT_JPEG,
		.flags  = IORESOURCE_IRQ,
	},
};

static struct platform_device msm_gemini_device = {
	.name           = "msm_gemini",
	.resource       = msm_gemini_resources,
	.num_resources  = ARRAY_SIZE(msm_gemini_resources),
};
#endif

#ifdef CONFIG_MSM7KV2_AUDIO
static uint32_t audio_pamp_gpio_config =
   GPIO_CFG(82, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA);

static uint32_t HAC_amp_gpio_config =
   GPIO_CFG(109, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA);

static int __init snddev_poweramp_gpio_init(void)
{
	int rc;

	pr_info("snddev_poweramp_gpio_init \n");
	rc = gpio_tlmm_config(audio_pamp_gpio_config, GPIO_CFG_ENABLE);
	if (rc) {
		printk(KERN_ERR
			"%s: gpio_tlmm_config(%#x)=%d\n",
			__func__, audio_pamp_gpio_config, rc);
	}

	/* Enabling HAC amplifier */
	rc = gpio_tlmm_config(HAC_amp_gpio_config, GPIO_CFG_ENABLE);
	if (rc) {
		printk(KERN_ERR
			"%s: gpio_tlmm_config(%#x)=%d\n",
			__func__, HAC_amp_gpio_config, rc);
	}

	return rc;
}

void msm_snddev_tx_route_config(void)
{
	pr_debug("%s()\n", __func__);
}

void msm_snddev_tx_route_deconfig(void)
{
	pr_debug("%s()\n", __func__);
}

void msm_snddev_hac_amp_on(void)
{
	gpio_set_value(109, 1);	/* enable HAC amp */
	pr_debug("%s: power on HAC amplifier\n", __func__);
}

void msm_snddev_hac_amp_off(void)
{
	gpio_set_value(109, 0);	/* disable HAC amp */
	pr_debug("%s: power off HAC amplifier\n", __func__);
}

void msm_snddev_poweramp_on(void)
{
	gpio_set_value(82, 1);	/* enable spkr poweramp */
	pr_debug("%s: power on amplifier\n", __func__);
}

void msm_snddev_poweramp_off(void)
{
	gpio_set_value(82, 0);	/* disable spkr poweramp */
	pr_debug("%s: power off amplifier\n", __func__);
}

static struct regulator_bulk_data snddev_regs[] = {
	{ .supply = "gp4", .min_uV = 2600000, .max_uV = 2600000 },
	{ .supply = "ncp", .min_uV = 1800000, .max_uV = 1800000 },
};

static int __init snddev_hsed_voltage_init(void)
{
	int rc;

	rc = regulator_bulk_get(NULL, ARRAY_SIZE(snddev_regs), snddev_regs);

	if (rc) {
		pr_err("%s: could not get regulators: %d\n", __func__, rc);
		goto out;
	}

	rc = regulator_bulk_set_voltage(ARRAY_SIZE(snddev_regs), snddev_regs);

	if (rc) {
		pr_err("%s: could not set regulator voltages: %d\n",
				__func__, rc);
		goto regs_free;
	}

	return 0;

regs_free:
	regulator_bulk_free(ARRAY_SIZE(snddev_regs), snddev_regs);
out:
	return rc;
}


void msm_snddev_hsed_voltage_on(void)
{
	int rc = regulator_bulk_enable(ARRAY_SIZE(snddev_regs), snddev_regs);

	if (rc)
		pr_err("%s: could not enable regulators: %d\n", __func__, rc);
}

void msm_snddev_hsed_voltage_off(void)
{
	int rc = regulator_bulk_disable(ARRAY_SIZE(snddev_regs), snddev_regs);

	if (rc) {
		pr_err("%s: could not disable regulators: %d\n", __func__, rc);
	}
}

static unsigned aux_pcm_gpio_on[] = {
	GPIO_CFG(138, 1, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),   /* PCM_DOUT */
	GPIO_CFG(139, 1, GPIO_CFG_INPUT,  GPIO_CFG_NO_PULL, GPIO_CFG_2MA),   /* PCM_DIN  */
	GPIO_CFG(140, 1, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),   /* PCM_SYNC */
	GPIO_CFG(141, 1, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),   /* PCM_CLK  */
};

static int __init aux_pcm_gpio_init(void)
{
	int pin, rc;

	pr_info("aux_pcm_gpio_init \n");
	for (pin = 0; pin < ARRAY_SIZE(aux_pcm_gpio_on); pin++) {
		rc = gpio_tlmm_config(aux_pcm_gpio_on[pin],
					GPIO_CFG_ENABLE);
		if (rc) {
			printk(KERN_ERR
				"%s: gpio_tlmm_config(%#x)=%d\n",
				__func__, aux_pcm_gpio_on[pin], rc);
		}
	}
	return rc;
}

static struct msm_gpio mi2s_clk_gpios[] = {
	{ GPIO_CFG(145, 1, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
	    "MI2S_SCLK"},
	{ GPIO_CFG(144, 1, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
	    "MI2S_WS"},
	{ GPIO_CFG(120, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
	    "MI2S_MCLK_A"},
};

static struct msm_gpio mi2s_rx_data_lines_gpios[] = {
	{ GPIO_CFG(121, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
	    "MI2S_DATA_SD0_A"},
	{ GPIO_CFG(122, 1, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
	    "MI2S_DATA_SD1_A"},
	{ GPIO_CFG(123, 1, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
	    "MI2S_DATA_SD2_A"},
	{ GPIO_CFG(146, 1, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
	    "MI2S_DATA_SD3"},
};

static struct msm_gpio mi2s_tx_data_lines_gpios[] = {
	{ GPIO_CFG(146, 1, GPIO_CFG_INPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
	    "MI2S_DATA_SD3"},
};

int mi2s_config_clk_gpio(void)
{
	int rc = 0;

	rc = msm_gpios_request_enable(mi2s_clk_gpios,
			ARRAY_SIZE(mi2s_clk_gpios));
	if (rc) {
		pr_err("%s: enable mi2s clk gpios  failed\n",
					__func__);
		return rc;
	}
	return 0;
}

int  mi2s_unconfig_data_gpio(u32 direction, u8 sd_line_mask)
{
	int i, rc = 0;
	sd_line_mask &= MI2S_SD_LINE_MASK;

	switch (direction) {
	case DIR_TX:
		msm_gpios_disable_free(mi2s_tx_data_lines_gpios, 1);
		break;
	case DIR_RX:
		i = 0;
		while (sd_line_mask) {
			if (sd_line_mask & 0x1)
				msm_gpios_disable_free(
					mi2s_rx_data_lines_gpios + i , 1);
			sd_line_mask = sd_line_mask >> 1;
			i++;
		}
		break;
	default:
		pr_err("%s: Invaild direction  direction = %u\n",
						__func__, direction);
		rc = -EINVAL;
		break;
	}
	return rc;
}

int mi2s_config_data_gpio(u32 direction, u8 sd_line_mask)
{
	int i , rc = 0;
	u8 sd_config_done_mask = 0;

	sd_line_mask &= MI2S_SD_LINE_MASK;

	switch (direction) {
	case DIR_TX:
		if ((sd_line_mask & MI2S_SD_0) || (sd_line_mask & MI2S_SD_1) ||
		   (sd_line_mask & MI2S_SD_2) || !(sd_line_mask & MI2S_SD_3)) {
			pr_err("%s: can not use SD0 or SD1 or SD2 for TX"
				".only can use SD3. sd_line_mask = 0x%x\n",
				__func__ , sd_line_mask);
			rc = -EINVAL;
		} else {
			rc = msm_gpios_request_enable(mi2s_tx_data_lines_gpios,
							 1);
			if (rc)
				pr_err("%s: enable mi2s gpios for TX failed\n",
					   __func__);
		}
		break;
	case DIR_RX:
		i = 0;
		while (sd_line_mask && (rc == 0)) {
			if (sd_line_mask & 0x1) {
				rc = msm_gpios_request_enable(
					mi2s_rx_data_lines_gpios + i , 1);
				if (rc) {
					pr_err("%s: enable mi2s gpios for"
					 "RX failed.  SD line = %s\n",
					 __func__,
					 (mi2s_rx_data_lines_gpios + i)->label);
					mi2s_unconfig_data_gpio(DIR_RX,
						sd_config_done_mask);
				} else
					sd_config_done_mask |= (1 << i);
			}
			sd_line_mask = sd_line_mask >> 1;
			i++;
		}
		break;
	default:
		pr_err("%s: Invaild direction  direction = %u\n",
			__func__, direction);
		rc = -EINVAL;
		break;
	}
	return rc;
}

int mi2s_unconfig_clk_gpio(void)
{
	msm_gpios_disable_free(mi2s_clk_gpios, ARRAY_SIZE(mi2s_clk_gpios));
	return 0;
}

#endif /* CONFIG_MSM7KV2_AUDIO */

static int __init buses_init(void)
{
	if (gpio_tlmm_config(GPIO_CFG(PMIC_GPIO_INT, 1, GPIO_CFG_INPUT,
				  GPIO_CFG_NO_PULL, GPIO_CFG_2MA), GPIO_CFG_ENABLE))
		pr_err("%s: gpio_tlmm_config (gpio=%d) failed\n",
		       __func__, PMIC_GPIO_INT);

	pm8058_7x30_data.keypad_pdata = &pm8xxx_keypad_data;
	pm8058_7x30_data.keypad_pmic_pdata = &pmic_keypad_data;

	return 0;
}


static struct regulator *vreg_marimba_1;
static struct regulator *vreg_marimba_2;

static unsigned int msm_marimba_setup_power(void)
{
	int rc;

	rc = regulator_enable(vreg_marimba_1);
	if (rc) {
		pr_err("%s: regulator_enable failed (%d)\n", __func__, rc);
		goto out;
	}

	rc = regulator_enable(vreg_marimba_2);
	if (rc) {
		pr_err("%s: regulator_enable failed (%d)\n", __func__, rc);
		goto disable_marimba_1;
	}

	return 0;

disable_marimba_1:
	regulator_disable(vreg_marimba_1);
out:
	return rc;
};

static void msm_marimba_shutdown_power(void)
{
	int rc;

	rc = regulator_disable(vreg_marimba_2);
	if (rc)
		pr_err("%s: regulator_disable failed (%d)\n", __func__, rc);

	rc = regulator_disable(vreg_marimba_1);
	if (rc)
		pr_err("%s: regulator_disable failed (%d)\n", __func__, rc);
};

/* Slave id address for FM/CDC/QMEMBIST
 * Values can be programmed using Marimba slave id 0
 * should there be a conflict with other I2C devices
 * */
#define MARIMBA_SLAVE_ID_FM_ADDR	0x2A
#define MARIMBA_SLAVE_ID_CDC_ADDR	0x77
#define MARIMBA_SLAVE_ID_QMEMBIST_ADDR	0X66

static struct regulator_bulk_data codec_regs[] = {
	{ .supply = "s4", .min_uV = 2200000, .max_uV = 2200000 },
};

static int __init msm_marimba_codec_init(void)
{
	int rc = regulator_bulk_get(NULL, ARRAY_SIZE(codec_regs), codec_regs);

	if (rc) {
		pr_err("%s: could not get regulators: %d\n", __func__, rc);
		goto out;
	}

	rc = regulator_bulk_set_voltage(ARRAY_SIZE(codec_regs), codec_regs);
	if (rc) {
		pr_err("%s: could not set regulator voltages: %d\n",
				__func__, rc);
		goto reg_free;
	}

	return rc;

reg_free:
	regulator_bulk_free(ARRAY_SIZE(codec_regs), codec_regs);
out:
	return rc;
}

static int msm_marimba_codec_power(int vreg_on)
{
	int rc = vreg_on ?
		regulator_bulk_enable(ARRAY_SIZE(codec_regs), codec_regs) :
		regulator_bulk_disable(ARRAY_SIZE(codec_regs), codec_regs);

	if (rc) {
		pr_err("%s: could not %sable regulators: %d",
				__func__, vreg_on ? "en" : "dis", rc);
		return rc;
	}

	return 0;
}

static struct marimba_codec_platform_data mariba_codec_pdata = {
	.marimba_codec_power =  msm_marimba_codec_power,
#ifdef CONFIG_MARIMBA_CODEC
	.snddev_profile_init = msm_snddev_init,
#endif
};

static struct marimba_platform_data marimba_pdata = {
	.slave_id[MARIMBA_SLAVE_ID_FM]       = MARIMBA_SLAVE_ID_FM_ADDR,
	.slave_id[MARIMBA_SLAVE_ID_CDC]	     = MARIMBA_SLAVE_ID_CDC_ADDR,
	.slave_id[MARIMBA_SLAVE_ID_QMEMBIST] = MARIMBA_SLAVE_ID_QMEMBIST_ADDR,
	.marimba_setup = msm_marimba_setup_power,
	.marimba_shutdown = msm_marimba_shutdown_power,
	.codec = &mariba_codec_pdata,
	.tsadc_ssbi_adap = MARIMBA_SSBI_ADAP,
};

static void __init msm7x30_init_marimba(void)
{
	int rc;

	struct regulator_bulk_data regs[] = {
		{ .supply = "s3",   .min_uV = 1800000, .max_uV = 1800000 },
		{ .supply = "gp16", .min_uV = 1200000, .max_uV = 1200000 },
		{ .supply = "usb2", .min_uV = 1800000, .max_uV = 1800000 },
	};

	rc = msm_marimba_codec_init();

	if (rc) {
		pr_err("%s: msm_marimba_codec_init failed (%d)\n",
				__func__, rc);
		return;
	}

	rc = regulator_bulk_get(NULL, ARRAY_SIZE(regs), regs);

	if (rc) {
		pr_err("%s: could not get regulators: %d\n", __func__, rc);
		return;
	}

	rc = regulator_bulk_set_voltage(ARRAY_SIZE(regs), regs);

	if (rc) {
		pr_err("%s: could not set voltages: %d\n", __func__, rc);
		regulator_bulk_free(ARRAY_SIZE(regs), regs);
		return;
	}

	vreg_marimba_1 = regs[0].consumer;
	vreg_marimba_2 = regs[1].consumer;
}

#ifdef CONFIG_MSM7KV2_AUDIO
static struct resource msm_aictl_resources[] = {
	{
		.name = "aictl",
		.start = 0xa5000100,
		.end = 0xa5000100,
		.flags = IORESOURCE_MEM,
	}
};

static struct resource msm_mi2s_resources[] = {
	{
		.name = "hdmi",
		.start = 0xac900000,
		.end = 0xac900038,
		.flags = IORESOURCE_MEM,
	},
	{
		.name = "codec_rx",
		.start = 0xac940040,
		.end = 0xac940078,
		.flags = IORESOURCE_MEM,
	},
	{
		.name = "codec_tx",
		.start = 0xac980080,
		.end = 0xac9800B8,
		.flags = IORESOURCE_MEM,
	}

};

static struct msm_lpa_platform_data lpa_pdata = {
	.obuf_hlb_size = 0x2BFF8,
	.dsp_proc_id = 0,
	.app_proc_id = 2,
	.nosb_config = {
		.llb_min_addr = 0,
		.llb_max_addr = 0x3ff8,
		.sb_min_addr = 0,
		.sb_max_addr = 0,
	},
	.sb_config = {
		.llb_min_addr = 0,
		.llb_max_addr = 0x37f8,
		.sb_min_addr = 0x3800,
		.sb_max_addr = 0x3ff8,
	}
};

static struct resource msm_lpa_resources[] = {
	{
		.name = "lpa",
		.start = 0xa5000000,
		.end = 0xa50000a0,
		.flags = IORESOURCE_MEM,
	}
};

static struct resource msm_aux_pcm_resources[] = {

	{
		.name = "aux_codec_reg_addr",
		.start = 0xac9c00c0,
		.end = 0xac9c00c8,
		.flags = IORESOURCE_MEM,
	},
	{
		.name   = "aux_pcm_dout",
		.start  = 138,
		.end    = 138,
		.flags  = IORESOURCE_IO,
	},
	{
		.name   = "aux_pcm_din",
		.start  = 139,
		.end    = 139,
		.flags  = IORESOURCE_IO,
	},
	{
		.name   = "aux_pcm_syncout",
		.start  = 140,
		.end    = 140,
		.flags  = IORESOURCE_IO,
	},
	{
		.name   = "aux_pcm_clkin_a",
		.start  = 141,
		.end    = 141,
		.flags  = IORESOURCE_IO,
	},
};

static struct platform_device msm_aux_pcm_device = {
	.name   = "msm_aux_pcm",
	.id     = 0,
	.num_resources  = ARRAY_SIZE(msm_aux_pcm_resources),
	.resource       = msm_aux_pcm_resources,
};

struct platform_device msm_aictl_device = {
	.name = "audio_interct",
	.id   = 0,
	.num_resources = ARRAY_SIZE(msm_aictl_resources),
	.resource = msm_aictl_resources,
};

struct platform_device msm_mi2s_device = {
	.name = "mi2s",
	.id   = 0,
	.num_resources = ARRAY_SIZE(msm_mi2s_resources),
	.resource = msm_mi2s_resources,
};

struct platform_device msm_lpa_device = {
	.name = "lpa",
	.id   = 0,
	.num_resources = ARRAY_SIZE(msm_lpa_resources),
	.resource = msm_lpa_resources,
	.dev		= {
		.platform_data = &lpa_pdata,
	},
};
#endif /* CONFIG_MSM7KV2_AUDIO */

#define DEC0_FORMAT ((1<<MSM_ADSP_CODEC_MP3)| \
	(1<<MSM_ADSP_CODEC_AAC)|(1<<MSM_ADSP_CODEC_WMA)| \
	(1<<MSM_ADSP_CODEC_WMAPRO)|(1<<MSM_ADSP_CODEC_AMRWB)| \
	(1<<MSM_ADSP_CODEC_AMRNB)|(1<<MSM_ADSP_CODEC_WAV)| \
	(1<<MSM_ADSP_CODEC_ADPCM)|(1<<MSM_ADSP_CODEC_YADPCM)| \
	(1<<MSM_ADSP_CODEC_EVRC)|(1<<MSM_ADSP_CODEC_QCELP))
#define DEC1_FORMAT ((1<<MSM_ADSP_CODEC_MP3)| \
	(1<<MSM_ADSP_CODEC_AAC)|(1<<MSM_ADSP_CODEC_WMA)| \
	(1<<MSM_ADSP_CODEC_WMAPRO)|(1<<MSM_ADSP_CODEC_AMRWB)| \
	(1<<MSM_ADSP_CODEC_AMRNB)|(1<<MSM_ADSP_CODEC_WAV)| \
	(1<<MSM_ADSP_CODEC_ADPCM)|(1<<MSM_ADSP_CODEC_YADPCM)| \
	(1<<MSM_ADSP_CODEC_EVRC)|(1<<MSM_ADSP_CODEC_QCELP))
 #define DEC2_FORMAT ((1<<MSM_ADSP_CODEC_MP3)| \
	(1<<MSM_ADSP_CODEC_AAC)|(1<<MSM_ADSP_CODEC_WMA)| \
	(1<<MSM_ADSP_CODEC_WMAPRO)|(1<<MSM_ADSP_CODEC_AMRWB)| \
	(1<<MSM_ADSP_CODEC_AMRNB)|(1<<MSM_ADSP_CODEC_WAV)| \
	(1<<MSM_ADSP_CODEC_ADPCM)|(1<<MSM_ADSP_CODEC_YADPCM)| \
	(1<<MSM_ADSP_CODEC_EVRC)|(1<<MSM_ADSP_CODEC_QCELP))
 #define DEC3_FORMAT ((1<<MSM_ADSP_CODEC_MP3)| \
	(1<<MSM_ADSP_CODEC_AAC)|(1<<MSM_ADSP_CODEC_WMA)| \
	(1<<MSM_ADSP_CODEC_WMAPRO)|(1<<MSM_ADSP_CODEC_AMRWB)| \
	(1<<MSM_ADSP_CODEC_AMRNB)|(1<<MSM_ADSP_CODEC_WAV)| \
	(1<<MSM_ADSP_CODEC_ADPCM)|(1<<MSM_ADSP_CODEC_YADPCM)| \
	(1<<MSM_ADSP_CODEC_EVRC)|(1<<MSM_ADSP_CODEC_QCELP))
#define DEC4_FORMAT (1<<MSM_ADSP_CODEC_MIDI)

static unsigned int dec_concurrency_table[] = {
	/* Audio LP */
	0,
	(DEC3_FORMAT|(1<<MSM_ADSP_MODE_NONTUNNEL)|(1<<MSM_ADSP_OP_DM)),
	(DEC2_FORMAT|(1<<MSM_ADSP_MODE_NONTUNNEL)|(1<<MSM_ADSP_OP_DM)),
	(DEC1_FORMAT|(1<<MSM_ADSP_MODE_NONTUNNEL)|(1<<MSM_ADSP_OP_DM)),
	(DEC0_FORMAT|(1<<MSM_ADSP_MODE_TUNNEL)|(1<<MSM_ADSP_MODE_LP)|
	(1<<MSM_ADSP_OP_DM)),

	/* Concurrency 1 */
	(DEC4_FORMAT),
	(DEC3_FORMAT|(1<<MSM_ADSP_MODE_TUNNEL)|(1<<MSM_ADSP_OP_DM)),
	(DEC2_FORMAT|(1<<MSM_ADSP_MODE_TUNNEL)|(1<<MSM_ADSP_OP_DM)),
	(DEC1_FORMAT|(1<<MSM_ADSP_MODE_TUNNEL)|(1<<MSM_ADSP_OP_DM)),
	(DEC0_FORMAT|(1<<MSM_ADSP_MODE_TUNNEL)|(1<<MSM_ADSP_OP_DM)),

	 /* Concurrency 2 */
	(DEC4_FORMAT),
	(DEC3_FORMAT|(1<<MSM_ADSP_MODE_TUNNEL)|(1<<MSM_ADSP_OP_DM)),
	(DEC2_FORMAT|(1<<MSM_ADSP_MODE_TUNNEL)|(1<<MSM_ADSP_OP_DM)),
	(DEC1_FORMAT|(1<<MSM_ADSP_MODE_TUNNEL)|(1<<MSM_ADSP_OP_DM)),
	(DEC0_FORMAT|(1<<MSM_ADSP_MODE_TUNNEL)|(1<<MSM_ADSP_OP_DM)),

	/* Concurrency 3 */
	(DEC4_FORMAT),
	(DEC3_FORMAT|(1<<MSM_ADSP_MODE_TUNNEL)|(1<<MSM_ADSP_OP_DM)),
	(DEC2_FORMAT|(1<<MSM_ADSP_MODE_TUNNEL)|(1<<MSM_ADSP_OP_DM)),
	(DEC1_FORMAT|(1<<MSM_ADSP_MODE_TUNNEL)|(1<<MSM_ADSP_OP_DM)),
	(DEC0_FORMAT|(1<<MSM_ADSP_MODE_NONTUNNEL)|(1<<MSM_ADSP_OP_DM)),

	/* Concurrency 4 */
	(DEC4_FORMAT),
	(DEC3_FORMAT|(1<<MSM_ADSP_MODE_TUNNEL)|(1<<MSM_ADSP_OP_DM)),
	(DEC2_FORMAT|(1<<MSM_ADSP_MODE_TUNNEL)|(1<<MSM_ADSP_OP_DM)),
	(DEC1_FORMAT|(1<<MSM_ADSP_MODE_NONTUNNEL)|(1<<MSM_ADSP_OP_DM)),
	(DEC0_FORMAT|(1<<MSM_ADSP_MODE_NONTUNNEL)|(1<<MSM_ADSP_OP_DM)),

	/* Concurrency 5 */
	(DEC4_FORMAT),
	(DEC3_FORMAT|(1<<MSM_ADSP_MODE_TUNNEL)|(1<<MSM_ADSP_OP_DM)),
	(DEC2_FORMAT|(1<<MSM_ADSP_MODE_NONTUNNEL)|(1<<MSM_ADSP_OP_DM)),
	(DEC1_FORMAT|(1<<MSM_ADSP_MODE_NONTUNNEL)|(1<<MSM_ADSP_OP_DM)),
	(DEC0_FORMAT|(1<<MSM_ADSP_MODE_NONTUNNEL)|(1<<MSM_ADSP_OP_DM)),

	/* Concurrency 6 */
	(DEC4_FORMAT),
	(DEC3_FORMAT|(1<<MSM_ADSP_MODE_NONTUNNEL)|(1<<MSM_ADSP_OP_DM)),
	(DEC2_FORMAT|(1<<MSM_ADSP_MODE_NONTUNNEL)|(1<<MSM_ADSP_OP_DM)),
	(DEC1_FORMAT|(1<<MSM_ADSP_MODE_NONTUNNEL)|(1<<MSM_ADSP_OP_DM)),
	(DEC0_FORMAT|(1<<MSM_ADSP_MODE_NONTUNNEL)|(1<<MSM_ADSP_OP_DM)),
};

#define DEC_INFO(name, queueid, decid, nr_codec) { .module_name = name, \
	.module_queueid = queueid, .module_decid = decid, \
	.nr_codec_support = nr_codec}

#define DEC_INSTANCE(max_instance_same, max_instance_diff) { \
	.max_instances_same_dec = max_instance_same, \
	.max_instances_diff_dec = max_instance_diff}

static struct msm_adspdec_info dec_info_list[] = {
	DEC_INFO("AUDPLAY4TASK", 17, 4, 1),  /* AudPlay4BitStreamCtrlQueue */
	DEC_INFO("AUDPLAY3TASK", 16, 3, 11),  /* AudPlay3BitStreamCtrlQueue */
	DEC_INFO("AUDPLAY2TASK", 15, 2, 11),  /* AudPlay2BitStreamCtrlQueue */
	DEC_INFO("AUDPLAY1TASK", 14, 1, 11),  /* AudPlay1BitStreamCtrlQueue */
	DEC_INFO("AUDPLAY0TASK", 13, 0, 11), /* AudPlay0BitStreamCtrlQueue */
};

static struct dec_instance_table dec_instance_list[][MSM_MAX_DEC_CNT] = {
	/* Non Turbo Mode */
	{
		DEC_INSTANCE(4, 3), /* WAV */
		DEC_INSTANCE(4, 3), /* ADPCM */
		DEC_INSTANCE(4, 2), /* MP3 */
		DEC_INSTANCE(0, 0), /* Real Audio */
		DEC_INSTANCE(4, 2), /* WMA */
		DEC_INSTANCE(3, 2), /* AAC */
		DEC_INSTANCE(0, 0), /* Reserved */
		DEC_INSTANCE(0, 0), /* MIDI */
		DEC_INSTANCE(4, 3), /* YADPCM */
		DEC_INSTANCE(4, 3), /* QCELP */
		DEC_INSTANCE(4, 3), /* AMRNB */
		DEC_INSTANCE(1, 1), /* AMRWB/WB+ */
		DEC_INSTANCE(4, 3), /* EVRC */
		DEC_INSTANCE(1, 1), /* WMAPRO */
	},
	/* Turbo Mode */
	{
		DEC_INSTANCE(4, 3), /* WAV */
		DEC_INSTANCE(4, 3), /* ADPCM */
		DEC_INSTANCE(4, 3), /* MP3 */
		DEC_INSTANCE(0, 0), /* Real Audio */
		DEC_INSTANCE(4, 3), /* WMA */
		DEC_INSTANCE(4, 3), /* AAC */
		DEC_INSTANCE(0, 0), /* Reserved */
		DEC_INSTANCE(0, 0), /* MIDI */
		DEC_INSTANCE(4, 3), /* YADPCM */
		DEC_INSTANCE(4, 3), /* QCELP */
		DEC_INSTANCE(4, 3), /* AMRNB */
		DEC_INSTANCE(2, 3), /* AMRWB/WB+ */
		DEC_INSTANCE(4, 3), /* EVRC */
		DEC_INSTANCE(1, 2), /* WMAPRO */
	},
};

static struct msm_adspdec_database msm_device_adspdec_database = {
	.num_dec = ARRAY_SIZE(dec_info_list),
	.num_concurrency_support = (ARRAY_SIZE(dec_concurrency_table) / \
					ARRAY_SIZE(dec_info_list)),
	.dec_concurrency_table = dec_concurrency_table,
	.dec_info_list = dec_info_list,
	.dec_instance_list = &dec_instance_list[0][0],
};

static struct platform_device msm_device_adspdec = {
	.name = "msm_adspdec",
	.id = -1,
	.dev    = {
		.platform_data = &msm_device_adspdec_database
	},
};

#ifdef CONFIG_USB_G_ANDROID
#define PID_MAGIC_ID		0x71432909
#define SERIAL_NUM_MAGIC_ID	0x61945374
#define SERIAL_NUMBER_LENGTH	127
#define DLOAD_USB_BASE_ADD	0x2A05F0C8

struct magic_num_struct {
	uint32_t pid;
	uint32_t serial_num;
};

struct dload_struct {
	uint32_t	reserved1;
	uint32_t	reserved2;
	uint32_t	reserved3;
	uint16_t	reserved4;
	uint16_t	pid;
	char		serial_number[SERIAL_NUMBER_LENGTH];
	uint16_t	reserved5;
	struct magic_num_struct
			magic_struct;
};

static int usb_diag_update_pid_and_serial_num(uint32_t pid, const char *snum)
{
	struct dload_struct __iomem *dload = 0;

	dload = ioremap(DLOAD_USB_BASE_ADD, sizeof(*dload));
	if (!dload) {
		pr_err("%s: cannot remap I/O memory region: %08x\n",
					__func__, DLOAD_USB_BASE_ADD);
		return -ENXIO;
	}

	pr_debug("%s: dload:%p pid:%x serial_num:%s\n",
				__func__, dload, pid, snum);
	/* update pid */
	dload->magic_struct.pid = PID_MAGIC_ID;
	dload->pid = pid;

	/* update serial number */
	dload->magic_struct.serial_num = 0;
	if (!snum)
		return 0;

	dload->magic_struct.serial_num = SERIAL_NUM_MAGIC_ID;
	strncpy(dload->serial_number, snum, SERIAL_NUMBER_LENGTH);
	dload->serial_number[SERIAL_NUMBER_LENGTH - 1] = '\0';

	iounmap(dload);

	return 0;
}

static struct android_usb_platform_data android_usb_pdata = {
	.update_pid_and_serial_num = usb_diag_update_pid_and_serial_num,
};

static struct platform_device android_usb_device = {
	.name	= "android_usb",
	.id		= -1,
	.dev		= {
		.platform_data = &android_usb_pdata,
	},
};
#endif

#ifdef CONFIG_FB_MSM_MDDI_NOVATEK_FWVGA
static void hr_usleep(int us)
{
	struct timespec req_time;
	long ret;

	req_time.tv_sec = us / 1000000;
	req_time.tv_nsec = (us % 1000000) * 1000;

	ret = hrtimer_nanosleep(&req_time, NULL, HRTIMER_MODE_REL,
							CLOCK_MONOTONIC);
	if (ret != 0)
		printk(KERN_ERR "%s: nanosleep failed, ret = %ld\n", __func__,
									ret);
}

static void hr_msleep(int ms)
{
	hr_usleep(1000 * ms);
}

static int novatek_power(int on)
{
	static int enabled_once;
	int rc = 0;

	if (on) {
		if (!enabled_once) {
			rc = vreg_helper_on("gp6", 2850);
			if (rc)
				goto out;
			rc = vreg_helper_on("gp9", 1800);
			if (rc) {
				vreg_helper_off("gp6");
				goto out;
			}
			hr_usleep(21); /* spec says > 20us */
			gpio_set_value(NOVATEK_GPIO_RESET, 1);
			hr_msleep(11); /* spec says > 11ms */
			enabled_once = 1;
		} else {
			gpio_set_value(NOVATEK_GPIO_RESET, 0);
			hr_msleep(4); /* spec says: > 4ms */
			gpio_set_value(NOVATEK_GPIO_RESET, 1);
			hr_msleep(11); /* spec says: > 10 ms */
		}
	}
	/* Do not do anything at power off */
out:
	return rc;
}

static struct novatek_fwvga_platform_data novatek_platform_data = {
	.power = novatek_power,
	.reset = NULL,
};

static struct platform_device novatek_device = {
	.name	= MDDI_NOVATEK_FWVGA_NAME,
	.id	= -1,
	.dev	= {
		.platform_data = &novatek_platform_data,
	}
};
#endif

#ifdef CONFIG_TOUCHSCREEN_CY8CTMA300
static struct cypress_callback *cy_callback;

static void cy_register_cb(struct cypress_callback *cy)
{
	cy_callback = cy;
}

static struct cypress_touch_platform_data cypress_touch_data = {
	.x_min		= 0,
	.x_max		= 479,
	.y_min		= 0,
	.y_max		= 853,
	.irq		= MSM_GPIO_TO_INT(CYPRESS_TOUCH_GPIO_IRQ),
	.gpio_irq_pin	= CYPRESS_TOUCH_GPIO_IRQ,
	.gpio_reset_pin	= CYPRESS_TOUCH_GPIO_RESET,
	.reset_polarity	= 1,
	.irq_polarity	= IRQF_TRIGGER_FALLING,
	.no_fw_update = 0,
	.register_cb	= cy_register_cb,
};

void charger_connected(int on)
{
	if (cy_callback && cy_callback->cb)
		cy_callback->cb(cy_callback, on);
}

static void cypress_touch_gpio_init(void)
{
	vreg_helper_on("gp13", 3000);

	/* Avoid writing firmward on SP3 */
	if (BOARD_HWID_SP3 == board_hwid)
		cypress_touch_data.no_fw_update = 1;
	else
		cypress_touch_data.no_fw_update = 0;

	gpio_request(CYPRESS_TOUCH_GPIO_RESET, "cy8_reset");
	/* intial reset */
	msleep(1);
	gpio_set_value(CYPRESS_TOUCH_GPIO_RESET,
			cypress_touch_data.reset_polarity);
	gpio_free(CYPRESS_TOUCH_GPIO_RESET);
}
#endif

#ifdef CONFIG_SEMC_CHARGER_USB_ARCH
static char *semc_chg_usb_supplied_to[] = {
	MAX17040_NAME,
};
#endif

#ifdef CONFIG_BATTERY_SEMC_ARCH
static char *semc_bdata_supplied_to[] = {
	MAX17040_NAME,
};

static struct semc_battery_platform_data semc_battery_platform_data = {
	.supplied_to = semc_bdata_supplied_to,
	.num_supplicants = ARRAY_SIZE(semc_bdata_supplied_to),
};

static struct platform_device bdata_driver = {
	.name = SEMC_BDATA_NAME,
	.id = -1,
	.dev = {
		.platform_data = &semc_battery_platform_data,
	},
};
#endif

#ifdef CONFIG_BATTERY_ZEUS
static struct max17040_platform_data max17040_platform_data = {
	.model_desc = {
		.ocv_test = { 0xD9, 0x80 },
		.soc_low = 0xF4,
		.soc_high = 0xF6,
		.model_data = {
			{
				0xA6, 0xA0, 0xB7, 0x50, 0xB8, 0xB0, 0xB8, 0xE0,
				0xB9, 0x30, 0xBB, 0x60, 0xBB, 0xF0, 0xBC, 0x40
			},
			{
				0xBC, 0xA0, 0xBD, 0x50, 0xBE, 0x20, 0xC0, 0x20,
				0xC3, 0xF0, 0xC6, 0xE0, 0xCB, 0x40, 0xCF, 0x80
			},
			{
				0x03, 0xA0, 0x1A, 0x80, 0xAD, 0x60, 0x43, 0x60,
				0x00, 0x40, 0x7E, 0x40, 0x0E, 0x80, 0x72, 0x00
			},
			{
				0x4C, 0x20, 0x3B, 0x40, 0x29, 0xE0, 0x1B, 0x00,
				0x1B, 0x20, 0x13, 0x60, 0x12, 0x40, 0x12, 0x40
			}
		},
		.exp = 1
	},
	.rcomp_data = {
		.rcomp0 = 0x55,
		.temp_co_hot = -1400,
		.temp_co_cold = -9725,
		.temp_div = 1000,
	},
	.chg_max_temp = 550,
	.chg_min_temp = 50,
};
#endif

#ifdef CONFIG_SENSORS_AKM8975
static struct msm_gpio akm8975_gpio_config_data[] = {
	{ GPIO_CFG(AKM8975_GPIO, 0, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN,
		GPIO_CFG_2MA), "akm8975_drdy_irq" },
};

static int akm8975_gpio_setup(void)
{
	return msm_gpios_request_enable(akm8975_gpio_config_data,
			ARRAY_SIZE(akm8975_gpio_config_data));
}

static void akm8975_gpio_shutdown(void)
{
	msm_gpios_disable_free(akm8975_gpio_config_data,
		ARRAY_SIZE(akm8975_gpio_config_data));

}

static struct akm8975_platform_data akm8975_platform_data = {
	.setup = akm8975_gpio_setup,
	.shutdown = akm8975_gpio_shutdown,
};
#endif

static struct i2c_board_info msm_i2c_board_info[] = {
#ifdef CONFIG_LEDS_AS3676
	{
		/* Config-spec is 8-bit = 0x80, src-code need 7-bit => 0x40 */
		I2C_BOARD_INFO("as3676", 0x80 >> 1),
		.platform_data = &as3676_platform_data,
	},
#endif
#ifdef CONFIG_BATTERY_ZEUS
	{
		I2C_BOARD_INFO(MAX17040_NAME, 0x6C >> 1),
		.platform_data = &max17040_platform_data,
	},
#endif
#ifdef CONFIG_INPUT_BMA150_NG
	{
		I2C_BOARD_INFO("bma150", 0x70 >> 1),
		.irq = MSM_GPIO_TO_INT(BMA150_GPIO),
		.platform_data = &bma150_ng_platform_data,
		.type = "bma150"
	},
#endif
#ifdef CONFIG_SENSORS_AKM8975
	{
		I2C_BOARD_INFO(AKM8975_I2C_NAME, 0x18 >> 1),
		.irq = MSM_GPIO_TO_INT(AKM8975_GPIO),
		.platform_data = &akm8975_platform_data,
	},
#endif
};

static struct spi_board_info msm_spi_board_info[] __initdata = {
#ifdef CONFIG_TOUCHSCREEN_CY8CTMA300
	{
		.modalias	= "cypress_touchscreen",
		.mode		= SPI_MODE_0,
		.platform_data	= &cypress_touch_data,
		.bus_num	= 0,
		.chip_select	= 0,
		.max_speed_hz	= 1 * 1000 * 1000,
	},
#endif
};

static struct i2c_board_info msm_marimba_board_info[] = {
	{
		I2C_BOARD_INFO("marimba", 0xc),
		.platform_data = &marimba_pdata,
	}
};


static struct msm_handset_platform_data hs_platform_data = {
	.hs_name = "7k_handset",
	.pwr_key_delay_ms = 500, /* 0 will disable end key */
};

static struct platform_device hs_device = {
	.name   = "msm-handset",
	.id     = -1,
	.dev    = {
		.platform_data = &hs_platform_data,
	},
};

static struct msm_pm_platform_data msm_pm_data[MSM_PM_SLEEP_MODE_NR] = {
	[MSM_PM_MODE(0, MSM_PM_SLEEP_MODE_POWER_COLLAPSE)] = {
		.idle_supported = 1,
		.suspend_supported = 1,
		.idle_enabled = 1,
		.suspend_enabled = 1,
		.latency = 8594,
		.residency = 23740,
	},
	[MSM_PM_MODE(0, MSM_PM_SLEEP_MODE_POWER_COLLAPSE_NO_XO_SHUTDOWN)] = {
		.idle_supported = 1,
		.suspend_supported = 1,
		.idle_enabled = 1,
		.suspend_enabled = 1,
		.latency = 4594,
		.residency = 23740,
	},
	[MSM_PM_MODE(0, MSM_PM_SLEEP_MODE_POWER_COLLAPSE_STANDALONE)] = {
#ifdef CONFIG_MSM_STANDALONE_POWER_COLLAPSE
		.idle_supported = 1,
		.suspend_supported = 1,
		.idle_enabled = 1,
		.suspend_enabled = 0,
#else /*CONFIG_MSM_STANDALONE_POWER_COLLAPSE*/
		.idle_supported = 0,
		.suspend_supported = 0,
		.idle_enabled = 0,
		.suspend_enabled = 0,
#endif /*CONFIG_MSM_STANDALONE_POWER_COLLAPSE*/
		.latency = 500,
		.residency = 6000,
	},
	[MSM_PM_MODE(0, MSM_PM_SLEEP_MODE_RAMP_DOWN_AND_WAIT_FOR_INTERRUPT)] = {
		.idle_supported = 1,
		.suspend_supported = 1,
		.idle_enabled = 0,
		.suspend_enabled = 1,
		.latency = 443,
		.residency = 1098,
	},
	[MSM_PM_MODE(0, MSM_PM_SLEEP_MODE_WAIT_FOR_INTERRUPT)] = {
		.idle_supported = 1,
		.suspend_supported = 1,
		.idle_enabled = 1,
		.suspend_enabled = 1,
		.latency = 2,
		.residency = 0,
	},
};

static struct msm_pm_boot_platform_data msm_pm_boot_pdata __initdata = {
	.mode = MSM_PM_BOOT_CONFIG_RESET_VECTOR_VIRT,
	.v_addr = (uint32_t *)PAGE_OFFSET,
};

static struct resource qsd_spi_resources[] = {
	{
		.name   = "spi_irq_in",
		.start	= INT_SPI_INPUT,
		.end	= INT_SPI_INPUT,
		.flags	= IORESOURCE_IRQ,
	},
	{
		.name   = "spi_irq_out",
		.start	= INT_SPI_OUTPUT,
		.end	= INT_SPI_OUTPUT,
		.flags	= IORESOURCE_IRQ,
	},
	{
		.name   = "spi_irq_err",
		.start	= INT_SPI_ERROR,
		.end	= INT_SPI_ERROR,
		.flags	= IORESOURCE_IRQ,
	},
	{
		.name   = "spi_base",
		.start	= 0xA8000000,
		.end	= 0xA8000000 + SZ_4K - 1,
		.flags	= IORESOURCE_MEM,
	},
	{
		.name   = "spidm_channels",
		.flags  = IORESOURCE_DMA,
	},
	{
		.name   = "spidm_crci",
		.flags  = IORESOURCE_DMA,
	},
};

#define AMDH0_BASE_PHYS		0xAC200000
#define ADMH0_GP_CTL		(ct_adm_base + 0x3D8)
static int msm_qsd_spi_dma_config(void)
{
	void __iomem *ct_adm_base = 0;
	u32 spi_mux = 0;
	int ret = 0;

	ct_adm_base = ioremap(AMDH0_BASE_PHYS, PAGE_SIZE);
	if (!ct_adm_base) {
		pr_err("%s: Could not remap %x\n", __func__, AMDH0_BASE_PHYS);
		return -ENOMEM;
	}

	spi_mux = (ioread32(ADMH0_GP_CTL) & (0x3 << 12)) >> 12;

	qsd_spi_resources[4].start  = DMOV_USB_CHAN;
	qsd_spi_resources[4].end    = DMOV_TSIF_CHAN;

	switch (spi_mux) {
	case (1):
		qsd_spi_resources[5].start  = DMOV_HSUART1_RX_CRCI;
		qsd_spi_resources[5].end    = DMOV_HSUART1_TX_CRCI;
		break;
	case (2):
		qsd_spi_resources[5].start  = DMOV_HSUART2_RX_CRCI;
		qsd_spi_resources[5].end    = DMOV_HSUART2_TX_CRCI;
		break;
	case (3):
		qsd_spi_resources[5].start  = DMOV_CE_OUT_CRCI;
		qsd_spi_resources[5].end    = DMOV_CE_IN_CRCI;
		break;
	default:
		ret = -ENOENT;
	}

	iounmap(ct_adm_base);

	return ret;
}

static struct platform_device qsd_device_spi = {
	.name		= "spi_qsd",
	.id		= 0,
	.num_resources	= ARRAY_SIZE(qsd_spi_resources),
	.resource	= qsd_spi_resources,
};

static int msm_qsd_spi_gpio_config(void)
{
	return msm_gpios_request_enable(qsd_spi_gpio_config_data,
					qsd_spi_gpio_config_data_size);
}

static void msm_qsd_spi_gpio_release(void)
{
	msm_gpios_disable_free(qsd_spi_gpio_config_data,
			       qsd_spi_gpio_config_data_size);
}

static struct msm_spi_platform_data qsd_spi_pdata = {
	.max_clock_speed = 26331429,
	.gpio_config  = msm_qsd_spi_gpio_config,
	.gpio_release = msm_qsd_spi_gpio_release,
	.dma_config = msm_qsd_spi_dma_config,
};

static void __init msm_qsd_spi_init(void)
{
	qsd_device_spi.dev.platform_data = &qsd_spi_pdata;
}

#ifdef CONFIG_USB_EHCI_MSM_72K
static void msm_hsusb_vbus_power(unsigned phy_info, int on)
{
}

static struct msm_usb_host_platform_data msm_usb_host_pdata = {
        .phy_info   = (USB_PHY_INTEGRATED | USB_PHY_MODEL_45NM),
        .vbus_power = msm_hsusb_vbus_power,
        .power_budget   = 300,
};
#endif

#ifdef CONFIG_USB_MSM_OTG_72K
static struct regulator *vreg_3p3;
static int msm_hsusb_ldo_init(int init)
{
	int def_vol = 3400000;

	if (init) {
		vreg_3p3 = regulator_get(NULL, "usb");
		if (IS_ERR(vreg_3p3))
			return PTR_ERR(vreg_3p3);
		regulator_set_voltage(vreg_3p3, def_vol, def_vol);
	} else
		regulator_put(vreg_3p3);

	return 0;
}

static int msm_hsusb_ldo_enable(int enable)
{
	static int ldo_status;

	if (!vreg_3p3 || IS_ERR(vreg_3p3))
		return -ENODEV;

	if (ldo_status == enable)
		return 0;

	ldo_status = enable;

	if (enable)
		return regulator_enable(vreg_3p3);
	else
		return regulator_disable(vreg_3p3);
}

static int msm_hsusb_ldo_set_voltage(int mV)
{
	static int cur_voltage;

	if (!vreg_3p3 || IS_ERR(vreg_3p3))
		return -ENODEV;

	if (cur_voltage == mV)
		return 0;

	cur_voltage = mV;

	pr_debug("%s: (%d)\n", __func__, mV);

	return regulator_set_voltage(vreg_3p3, mV*1000, mV*1000);
}
#endif

#ifndef CONFIG_USB_EHCI_MSM_72K
static int msm_hsusb_pmic_notif_init(void (*callback)(int online), int init);
#endif
static struct msm_otg_platform_data msm_otg_pdata = {
#ifndef CONFIG_USB_EHCI_MSM_72K
	.pmic_vbus_notif_init         = msm_hsusb_pmic_notif_init,
#else
	.vbus_power = msm_hsusb_vbus_power,
#endif
	.pemp_level		 = PRE_EMPHASIS_WITH_20_PERCENT,
	.cdr_autoreset		 = CDR_AUTO_RESET_DISABLE,
	.drv_ampl		 = HS_DRV_AMPLITUDE_DEFAULT,
	.se1_gating		 = SE1_GATING_DISABLE,
	.ldo_enable		 = msm_hsusb_ldo_enable,
	.ldo_init		 = msm_hsusb_ldo_init,
	.ldo_set_voltage	 = msm_hsusb_ldo_set_voltage,
#ifdef CONFIG_SEMC_CHARGER_USB_ARCH
	.chg_vbus_draw		 = semc_charger_usb_vbus_draw,
	.chg_connected		 = semc_charger_usb_connected,
	.chg_init		 = semc_charger_usb_init,
#endif
	.phy_can_powercollapse	 = 1,
};

#ifdef CONFIG_USB_GADGET
static struct msm_hsusb_gadget_platform_data msm_gadget_pdata = {
	.is_phy_status_timer_on = 1,
};
#endif
#ifndef CONFIG_USB_EHCI_MSM_72K
typedef void (*notify_vbus_state) (int);
notify_vbus_state notify_vbus_state_func_ptr;
int vbus_on_irq;
static irqreturn_t pmic_vbus_on_irq(int irq, void *data)
{
	pr_info("%s: vbus notification from pmic\n", __func__);

	(*notify_vbus_state_func_ptr) (1);

	return IRQ_HANDLED;
}
static int msm_hsusb_pmic_notif_init(void (*callback)(int online), int init)
{
	int ret;

	if (init) {
		if (!callback)
			return -ENODEV;

		notify_vbus_state_func_ptr = callback;
		vbus_on_irq = platform_get_irq_byname(&msm_device_otg,
			"vbus_on");
		if (vbus_on_irq <= 0) {
			pr_err("%s: unable to get vbus on irq\n", __func__);
			return -ENODEV;
		}

		ret = request_any_context_irq(vbus_on_irq, pmic_vbus_on_irq,
			IRQF_TRIGGER_RISING, "msm_otg_vbus_on", NULL);
		if (ret < 0) {
			pr_info("%s: request_irq for vbus_on"
				"interrupt failed\n", __func__);
			return ret;
		}
		msm_otg_pdata.pmic_vbus_irq = vbus_on_irq;
		return 0;
	} else {
		free_irq(vbus_on_irq, 0);
		notify_vbus_state_func_ptr = NULL;
		return 0;
	}
}
#endif

static struct android_pmem_platform_data android_pmem_pdata = {
	.name = "pmem",
	.allocator_type = PMEM_ALLOCATORTYPE_ALLORNOTHING,
	.cached = 1,
	.memory_type = MEMTYPE_EBI0,
};

static struct platform_device android_pmem_device = {
	.name = "android_pmem",
	.id = 0,
	.dev = { .platform_data = &android_pmem_pdata },
};

static struct msm_serial_hs_platform_data msm_uart_dm1_pdata = {
       .inject_rx_on_wakeup = 1,
       .rx_to_inject = 0xFD,
};

static struct resource msm_fb_resources[] = {
	{
		.flags  = IORESOURCE_DMA,
	}
};

static int msm_fb_detect_panel(const char *name)
{
	return -ENODEV;
}

static struct msm_fb_platform_data msm_fb_pdata = {
	.detect_client = msm_fb_detect_panel,
	.mddi_prescan = 1,
};

static struct platform_device msm_fb_device = {
	.name   = "msm_fb",
	.id     = 0,
	.num_resources  = ARRAY_SIZE(msm_fb_resources),
	.resource       = msm_fb_resources,
	.dev    = {
		.platform_data = &msm_fb_pdata,
	}
};

static struct platform_device msm_migrate_pages_device = {
	.name   = "msm_migrate_pages",
	.id     = -1,
};

static struct android_pmem_platform_data android_pmem_adsp_pdata = {
       .name = "pmem_adsp",
       .allocator_type = PMEM_ALLOCATORTYPE_BITMAP,
       .cached = 0,
	.memory_type = MEMTYPE_EBI0,
};

static struct platform_device android_pmem_adsp_device = {
       .name = "android_pmem",
       .id = 2,
       .dev = { .platform_data = &android_pmem_adsp_pdata },
};

#if defined(CONFIG_CRYPTO_DEV_QCRYPTO) || \
		defined(CONFIG_CRYPTO_DEV_QCRYPTO_MODULE) || \
		defined(CONFIG_CRYPTO_DEV_QCEDEV) || \
		defined(CONFIG_CRYPTO_DEV_QCEDEV_MODULE)

#define QCE_SIZE		0x10000
#define QCE_0_BASE		0xA8400000

#define QCE_HW_KEY_SUPPORT	1
#define QCE_SHA_HMAC_SUPPORT	0
#define QCE_SHARE_CE_RESOURCE	0
#define QCE_CE_SHARED		0

static struct resource qcrypto_resources[] = {
	[0] = {
		.start = QCE_0_BASE,
		.end = QCE_0_BASE + QCE_SIZE - 1,
		.flags = IORESOURCE_MEM,
	},
	[1] = {
		.name = "crypto_channels",
		.start = DMOV_CE_IN_CHAN,
		.end = DMOV_CE_OUT_CHAN,
		.flags = IORESOURCE_DMA,
	},
	[2] = {
		.name = "crypto_crci_in",
		.start = DMOV_CE_IN_CRCI,
		.end = DMOV_CE_IN_CRCI,
		.flags = IORESOURCE_DMA,
	},
	[3] = {
		.name = "crypto_crci_out",
		.start = DMOV_CE_OUT_CRCI,
		.end = DMOV_CE_OUT_CRCI,
		.flags = IORESOURCE_DMA,
	},
	[4] = {
		.name = "crypto_crci_hash",
		.start = DMOV_CE_HASH_CRCI,
		.end = DMOV_CE_HASH_CRCI,
		.flags = IORESOURCE_DMA,
	},
};

static struct resource qcedev_resources[] = {
	[0] = {
		.start = QCE_0_BASE,
		.end = QCE_0_BASE + QCE_SIZE - 1,
		.flags = IORESOURCE_MEM,
	},
	[1] = {
		.name = "crypto_channels",
		.start = DMOV_CE_IN_CHAN,
		.end = DMOV_CE_OUT_CHAN,
		.flags = IORESOURCE_DMA,
	},
	[2] = {
		.name = "crypto_crci_in",
		.start = DMOV_CE_IN_CRCI,
		.end = DMOV_CE_IN_CRCI,
		.flags = IORESOURCE_DMA,
	},
	[3] = {
		.name = "crypto_crci_out",
		.start = DMOV_CE_OUT_CRCI,
		.end = DMOV_CE_OUT_CRCI,
		.flags = IORESOURCE_DMA,
	},
	[4] = {
		.name = "crypto_crci_hash",
		.start = DMOV_CE_HASH_CRCI,
		.end = DMOV_CE_HASH_CRCI,
		.flags = IORESOURCE_DMA,
	},
};

#endif

#if defined(CONFIG_CRYPTO_DEV_QCRYPTO) || \
		defined(CONFIG_CRYPTO_DEV_QCRYPTO_MODULE)

static struct msm_ce_hw_support qcrypto_ce_hw_suppport = {
	.ce_shared = QCE_CE_SHARED,
	.shared_ce_resource = QCE_SHARE_CE_RESOURCE,
	.hw_key_support = QCE_HW_KEY_SUPPORT,
	.sha_hmac = QCE_SHA_HMAC_SUPPORT,
	/* Bus Scaling declaration*/
	.bus_scale_table = NULL,
};

static struct platform_device qcrypto_device = {
	.name		= "qcrypto",
	.id		= 0,
	.num_resources	= ARRAY_SIZE(qcrypto_resources),
	.resource	= qcrypto_resources,
	.dev		= {
		.coherent_dma_mask = DMA_BIT_MASK(32),
		.platform_data = &qcrypto_ce_hw_suppport,
	},
};
#endif

#if defined(CONFIG_CRYPTO_DEV_QCEDEV) || \
		defined(CONFIG_CRYPTO_DEV_QCEDEV_MODULE)

static struct msm_ce_hw_support qcedev_ce_hw_suppport = {
	.ce_shared = QCE_CE_SHARED,
	.shared_ce_resource = QCE_SHARE_CE_RESOURCE,
	.hw_key_support = QCE_HW_KEY_SUPPORT,
	.sha_hmac = QCE_SHA_HMAC_SUPPORT,
	/* Bus Scaling declaration*/
	.bus_scale_table = NULL,
};
static struct platform_device qcedev_device = {
	.name		= "qce",
	.id		= 0,
	.num_resources	= ARRAY_SIZE(qcedev_resources),
	.resource	= qcedev_resources,
	.dev		= {
		.coherent_dma_mask = DMA_BIT_MASK(32),
		.platform_data = &qcedev_ce_hw_suppport,
	},
};
#endif

static int display_power(int on)
{
	return 0;
}

static int msm_fb_mddi_sel_clk(u32 *clk_rate)
{
	*clk_rate *= 2;
	return 0;
}

static struct mddi_platform_data mddi_pdata = {
	.mddi_power_save = display_power,
	.mddi_sel_clk = msm_fb_mddi_sel_clk,
};

static struct msm_panel_common_pdata mdp_pdata = {
	.hw_revision_addr = 0xac001270,
	.gpio = 30,
	.mdp_max_clk = 192000000,
	.mdp_rev = MDP_REV_40,
	.mem_hid = MEMTYPE_EBI0,
};

static struct regulator *atv_s4, *atv_ldo9;

static int __init atv_dac_power_init(void)
{
	int rc;
	struct regulator_bulk_data regs[] = {
		{ .supply = "smps4", .min_uV = 2200000, .max_uV = 2200000 },
		{ .supply = "ldo9",  .min_uV = 2050000, .max_uV = 2050000 },
	};

	rc = regulator_bulk_get(NULL, ARRAY_SIZE(regs), regs);

	if (rc) {
		pr_err("%s: could not get regulators: %d\n", __func__, rc);
		goto bail;
	}

	rc = regulator_bulk_set_voltage(ARRAY_SIZE(regs), regs);

	if (rc) {
		pr_err("%s: could not set voltages: %d\n", __func__, rc);
		goto reg_free;
	}

	atv_s4   = regs[0].consumer;
	atv_ldo9 = regs[1].consumer;

reg_free:
	regulator_bulk_free(ARRAY_SIZE(regs), regs);
bail:
	return rc;
}

#ifdef CONFIG_SIMPLE_REMOTE_PLATFORM
#define PLUG_DET_ENA_PIN 80
#define PLUG_DET_READ_PIN 26
#define MODE_SWITCH_PIN -1

int simple_remote_pf_initialize_gpio(struct simple_remote_platform_data *data)
{
	int err = 0;
	int i;

	if (!data || -1 == data->headset_detect_enable_pin) {
		printk(KERN_ERR
		       "*** %s - Error: Invalid inparameter (GPIO Pins)."
		       " Aborting!\n", __func__);
		return -EIO;
	}

	err = gpio_request(data->headset_detect_enable_pin,
			   "Simple_remote_plug_detect_enable");
	if (err) {
		printk(KERN_CRIT "%s: Error %d - Request hs_detect_enable pin",
		       __func__, err);
		goto out;
	}

	err = gpio_direction_output(data->headset_detect_enable_pin, 1);
	if (err) {
		printk(KERN_CRIT "%s: Error %d - Set hs_detect_enable pin"
		       " as output high\n", __func__, err);
		goto out_hs_det_enable;
	}

	err = gpio_request(data->headset_detect_read_pin,
			   "Simple_remote_plug_detect_read");
	if (err) {
		printk(KERN_CRIT "%s - Error %d - Request hs-detect_read pin",
		       __func__, err);
		goto out_hs_det_enable;
	}

	err = gpio_direction_input(data->headset_detect_read_pin);
	if (err) {
		printk(KERN_CRIT "%s - Error %d - Set hs-detect pin as input\n",
		       __func__, err);
		goto out_hs_det_read;
	}

	if (BOARD_HWID_DBZ3 != board_hwid || BOARD_HWID_DBZ3_1 != board_hwid ||
	    BOARD_HWID_DBZ3 != board_hwid)
		data->invert_plug_det = 1;
	else
		data->invert_plug_det = 0;

	if (0 < data->headset_mode_switch_pin) {
		err = gpio_request(data->headset_mode_switch_pin,
				   "Simple_remote_headset_mode_switch");
		if (err) {
			printk(KERN_CRIT
			       "%s - Error %d - Request hs-mode_switch pin",
			       __func__, err);
			goto out_hs_det_read;
		}

		err = gpio_direction_output(data->headset_mode_switch_pin, 0);
		if (err) {
			printk(KERN_CRIT
			       "%s - Error %d - Set hs-mode_switch pin as "
			       "input\n", __func__, err);
			goto out_hs_mode_switch;
		}
	}

	for (i = 0; i < data->num_regs; i++) {
		data->regs[i].reg = vreg_get(NULL, data->regs[i].name);
		if (IS_ERR(data->regs[i].reg)) {
			printk(KERN_ERR "%s - Failed to find regulator %s\n",
			       __func__, data->regs[i].name);
			err = PTR_ERR(data->regs[i].reg);
			if (0 <= data->headset_mode_switch_pin)
				goto out_hs_mode_switch;
			else
				goto out_hs_det_read;
		}
	}

	return err;

out_hs_mode_switch:
	gpio_free(data->headset_mode_switch_pin);

out_hs_det_read:
	gpio_free(data->headset_detect_read_pin);

out_hs_det_enable:
	gpio_free(data->headset_detect_enable_pin);
out:
	return err;
}

void simple_remote_pf_deinitialize_gpio(
	struct simple_remote_platform_data *data)
{
	gpio_free(data->headset_detect_read_pin);
	gpio_free(data->headset_detect_enable_pin);
}

static struct simple_remote_platform_regulators regs[] =  {
	{
		.name = "ncp",
	},
	{
		.name = "s3",
	},
	{
		.name = "s2",
	},

};

static struct simple_remote_platform_data simple_remote_pf_data = {
	.headset_detect_enable_pin = PLUG_DET_ENA_PIN,
	.headset_detect_read_pin = PLUG_DET_READ_PIN,
	.headset_mode_switch_pin = MODE_SWITCH_PIN,
	.initialize = &simple_remote_pf_initialize_gpio,
	.deinitialize = &simple_remote_pf_deinitialize_gpio,

	.regs = regs,
	.num_regs = ARRAY_SIZE(regs),

	.controller = PM_HSED_CONTROLLER_1,
};

static struct platform_device simple_remote_pf_device = {
	.name = SIMPLE_REMOTE_PF_NAME,
	.dev = {
		.platform_data = &simple_remote_pf_data,
	},
};
#endif

static void __init msm_fb_add_devices(void)
{
	msm_fb_register_device("mdp", &mdp_pdata);
	msm_fb_register_device("pmdh", &mddi_pdata);
}

static char *msm_adc_device_names[] = {
	"XO_ADC",
};

static struct msm_adc_platform_data msm_adc_pdata = {
	.dev_names = msm_adc_device_names,
	.num_adc = ARRAY_SIZE(msm_adc_device_names),
};

static struct platform_device msm_adc_device = {
	.name   = "msm_adc",
	.id = -1,
	.dev = {
		.platform_data = &msm_adc_pdata,
	},
};

static struct platform_device *devices[] __initdata = {
#if defined(CONFIG_SERIAL_MSM) || defined(CONFIG_MSM_SERIAL_DEBUGGER)
	&msm_device_uart3,
#endif
#ifdef CONFIG_ANDROID_RAM_CONSOLE
	&ram_console_device,
#endif
#ifdef CONFIG_MSM_PROC_COMM_REGULATOR
	&msm_proccomm_regulator_dev,
#endif
	&asoc_msm_pcm,
	&asoc_msm_dai0,
	&asoc_msm_dai1,
#if defined (CONFIG_SND_MSM_MVS_DAI_SOC)
	&asoc_msm_mvs,
	&asoc_mvs_dai0,
	&asoc_mvs_dai1,
#endif
	&msm_device_smd,
	&msm_device_dmov,
	&msm_device_nand,
#ifdef CONFIG_USB_MSM_OTG_72K
	&msm_device_otg,
#ifdef CONFIG_USB_GADGET
	&msm_device_gadget_peripheral,
#endif
#endif
#ifdef CONFIG_USB_G_ANDROID
	&android_usb_device,
#endif
	&qsd_device_spi,

#ifdef CONFIG_MSM_SSBI
	&msm_device_ssbi_pmic1,
#endif
#ifdef CONFIG_I2C_SSBI
	&msm_device_ssbi7,
#endif
	&android_pmem_device,
	&msm_fb_device,
	&msm_migrate_pages_device,
#ifdef CONFIG_MSM_ROTATOR
	&msm_rotator_device,
#endif
	&android_pmem_adsp_device,
	&msm_device_i2c,
	&msm_device_i2c_2,
	&msm_device_uart_dm1,
#ifdef CONFIG_SEMC_RPC_SERVER_HANDSET
	&semc_rpc_handset_device,
#endif
	&hs_device,
#ifdef CONFIG_MSM7KV2_AUDIO
	&msm_aictl_device,
	&msm_mi2s_device,
	&msm_lpa_device,
	&msm_aux_pcm_device,
#endif
	&msm_device_adspdec,
	&qup_device_i2c,
	&msm_kgsl_3d0,
	&msm_kgsl_2d0,
#ifdef CONFIG_SEIX006
	&msm_camera_sensor_seix006,
#endif /* CONFIG_SEIX006 */
#ifdef CONFIG_OVM7692
	&msm_camera_sensor_ovm7692,
#endif /* CONFIG_OVM7692 */
#ifdef CONFIG_MT9V114
	&msm_camera_sensor_mt9v114,
#endif /* CONFIG_MT9V114 */
#ifdef CONFIG_MT9T013
	&msm_camera_sensor_mt9t013,
#endif
	&msm_device_vidc_720p,
#ifdef CONFIG_MSM_GEMINI
	&msm_gemini_device,
#endif
#ifdef CONFIG_MSM_VPE
	&msm_vpe_device,
#endif
#ifdef CONFIG_BATTERY_SEMC_ARCH
	&bdata_driver,
#endif
#ifdef CONFIG_SIMPLE_REMOTE_PLATFORM
	&simple_remote_pf_device,
#endif
#ifdef CONFIG_FB_MSM_MDDI_NOVATEK_FWVGA
	&novatek_device,
#endif

#if defined(CONFIG_CRYPTO_DEV_QCRYPTO) || \
		defined(CONFIG_CRYPTO_DEV_QCRYPTO_MODULE)
	&qcrypto_device,
#endif

#if defined(CONFIG_CRYPTO_DEV_QCEDEV) || \
		defined(CONFIG_CRYPTO_DEV_QCEDEV_MODULE)
	&qcedev_device,
#endif

	&msm_adc_device,
	&msm_ebi0_thermal,
	&msm_ebi1_thermal,
	&msm_adsp_device,
};

static void
msm_i2c_gpio_config(int adap_id, int config_type)
{
	struct msm_gpio *msm_i2c_table;

	/* Each adapter gets 2 lines from the table */
	if (adap_id > 0)
		return;
	if (config_type)
		msm_i2c_table = &msm_i2c_gpios_hw[adap_id*2];
	else
		msm_i2c_table = &msm_i2c_gpios_io[adap_id*2];
	msm_gpios_enable(msm_i2c_table, 2);

	vreg_helper_on("gp7", 1800);
}

static void
qup_i2c_gpio_config(int adap_id, int config_type)
{
	int rc = 0;
	struct msm_gpio *qup_i2c_table;
	/* Each adapter gets 2 lines from the table */
	if (adap_id != 4)
		return;
	if (config_type)
		qup_i2c_table = qup_i2c_gpios_hw;
	else
		qup_i2c_table = qup_i2c_gpios_io;
	rc = msm_gpios_enable(qup_i2c_table, 2);
	if (rc < 0)
		printk(KERN_ERR "QUP GPIO enable failed: %d\n", rc);
}

static struct msm_i2c_platform_data msm_i2c_pdata = {
	.clk_freq = 400000,
	.pri_clk = 70,
	.pri_dat = 71,
	.rmutex  = 1,
	.rsl_id = "D:I2C02000021",
	.msm_i2c_config_gpio = msm_i2c_gpio_config,
};

static void __init msm_device_i2c_init(void)
{
	if (msm_gpios_request(msm_i2c_gpios_hw, msm_i2c_gpios_hw_size))
		pr_err("failed to request I2C gpios\n");

	msm_device_i2c.dev.platform_data = &msm_i2c_pdata;
}

static struct msm_i2c_platform_data msm_i2c_2_pdata = {
	.clk_freq = 400000,
	.rmutex  = 0, // = 1,
	//.rsl_id = "D:I2C02000022",
	.msm_i2c_config_gpio = msm_i2c_gpio_config,
};

static void __init msm_device_i2c_2_init(void)
{
	msm_device_i2c_2.dev.platform_data = &msm_i2c_2_pdata;
}

static struct msm_i2c_platform_data qup_i2c_pdata = {
	.clk_freq = 400000,
	.msm_i2c_config_gpio = qup_i2c_gpio_config,
};

static void __init qup_device_i2c_init(void)
{
	if (msm_gpios_request(qup_i2c_gpios_hw, qup_i2c_gpios_hw_size))
		pr_err("failed to request I2C gpios\n");

	qup_device_i2c.dev.platform_data = &qup_i2c_pdata;
}

#ifdef CONFIG_I2C_SSBI
static struct msm_i2c_ssbi_platform_data msm_i2c_ssbi7_pdata = {
	.rsl_id = "D:CODEC_SSBI",
	.controller_type = MSM_SBI_CTRL_SSBI,
};
#endif

static void __init msm7x30_init_irq(void)
{
	msm_init_irq();
}

static struct msm_gpio msm_nand_ebi2_cfg_data[] = {
	{GPIO_CFG(86, 1, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_8MA), "ebi2_cs1"},
	{GPIO_CFG(115, 2, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_8MA), "ebi2_busy1"},
};

#if (defined(CONFIG_MMC_MSM_SDC3_SUPPORT)\
	|| defined(CONFIG_MMC_MSM_SDC4_SUPPORT))

struct sdcc_gpio {
	struct msm_gpio *cfg_data;
	uint32_t size;
	struct msm_gpio *sleep_cfg_data;
};

static struct msm_gpio sdc3_cfg_data[] = {
	{GPIO_CFG(110, 1, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_4MA),
								"sdc3_clk"},
	{GPIO_CFG(111, 1, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_4MA),
								"sdc3_cmd"},
	{GPIO_CFG(116, 1, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_4MA),
								"sdc3_dat_3"},
	{GPIO_CFG(117, 1, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_4MA),
								"sdc3_dat_2"},
	{GPIO_CFG(118, 1, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_4MA),
								"sdc3_dat_1"},
	{GPIO_CFG(119, 1, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_4MA),
								"sdc3_dat_0"},
};

static struct msm_gpio sdc3_sleep_cfg_data[] = {
	{GPIO_CFG(110, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_4MA),
								"sdc3_clk"},
	{GPIO_CFG(111, 0, GPIO_CFG_INPUT, GPIO_CFG_NO_PULL, GPIO_CFG_4MA),
								"sdc3_cmd"},
	{GPIO_CFG(116, 0, GPIO_CFG_INPUT, GPIO_CFG_NO_PULL, GPIO_CFG_4MA),
								"sdc3_dat_3"},
	{GPIO_CFG(117, 0, GPIO_CFG_INPUT, GPIO_CFG_NO_PULL, GPIO_CFG_4MA),
								"sdc3_dat_2"},
	{GPIO_CFG(118, 0, GPIO_CFG_INPUT, GPIO_CFG_NO_PULL, GPIO_CFG_4MA),
								"sdc3_dat_1"},
	{GPIO_CFG(119, 0, GPIO_CFG_INPUT, GPIO_CFG_NO_PULL, GPIO_CFG_4MA),
								"sdc3_dat_0"},
};

static struct msm_gpio sdc4_cfg_data[] = {
	{GPIO_CFG(58, 1, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_8MA),
								"sdc4_clk"},
	{GPIO_CFG(59, 1, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_8MA),
								"sdc4_cmd"},
	{GPIO_CFG(60, 1, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_8MA),
								"sdc4_dat_3"},
	{GPIO_CFG(61, 1, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_8MA),
								"sdc4_dat_2"},
	{GPIO_CFG(62, 1, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_8MA),
								"sdc4_dat_1"},
	{GPIO_CFG(63, 1, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_8MA),
								"sdc4_dat_0"},
};

static struct msm_gpio sdc4_sleep_cfg_data[] = {
	{GPIO_CFG(58, 0, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA),
								"sdc4_clk"},
	{GPIO_CFG(59, 0, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA),
								"sdc4_cmd"},
	{GPIO_CFG(60, 0, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA),
								"sdc4_dat_3"},
	{GPIO_CFG(61, 0, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA),
								"sdc4_dat_2"},
	{GPIO_CFG(62, 0, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA),
								"sdc4_dat_1"},
	{GPIO_CFG(63, 0, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA),
								"sdc4_dat_0"},
};

static struct sdcc_gpio sdcc_cfg_data[] = {
	{
		.cfg_data = NULL,
		.size = 0,
		.sleep_cfg_data = NULL,
	},
	{
		.cfg_data = NULL,
		.size = 0,
		.sleep_cfg_data = NULL,
	},
	{
		.cfg_data = sdc3_cfg_data,
		.size = ARRAY_SIZE(sdc3_cfg_data),
		.sleep_cfg_data = sdc3_sleep_cfg_data,
	},
	{
		.cfg_data = sdc4_cfg_data,
		.size = ARRAY_SIZE(sdc4_cfg_data),
		.sleep_cfg_data = sdc4_sleep_cfg_data,
	},
};

static struct regulator *sdcc_vreg_data[ARRAY_SIZE(sdcc_cfg_data)];

static unsigned long vreg_sts, gpio_sts;

static uint32_t msm_sdcc_setup_gpio(int dev_id, unsigned int enable)
{
	int rc = 0;
	struct sdcc_gpio *curr;

	curr = &sdcc_cfg_data[dev_id - 1];

	if (!(test_bit(dev_id, &gpio_sts)^enable))
		return rc;

	if (enable) {
		set_bit(dev_id, &gpio_sts);
		rc = msm_gpios_request_enable(curr->cfg_data, curr->size);
		if (rc)
			printk(KERN_ERR "%s: Failed to turn on GPIOs for slot %d\n",
				__func__,  dev_id);
	} else {
		clear_bit(dev_id, &gpio_sts);
		if (curr->sleep_cfg_data) {
			msm_gpios_enable(curr->sleep_cfg_data, curr->size);
			msm_gpios_free(curr->sleep_cfg_data, curr->size);
			if (dev_id == 4) {
				/*
				 * 200 milliseconds delay should be sufficient to allow
				 * microSD reaches zero voltage when uSD is power off.
				 */
				msleep(200);
			}
		} else {
			msm_gpios_disable_free(curr->cfg_data, curr->size);
		}
	}

	return rc;
}

static uint32_t msm_sdcc_setup_vreg(int dev_id, unsigned int enable)
{
	int rc = 0;
	struct regulator *curr = sdcc_vreg_data[dev_id - 1];
	static int enabled_once[] = {0, 0, 0, 0};

	if (test_bit(dev_id, &vreg_sts) == enable)
		return rc;

	if (dev_id == 4) {
		if (enable) {
			pr_debug("Enable Vdd dev_%d\n", dev_id);
			gpio_set_value_cansleep(
				PM8058_GPIO_PM_TO_SYS(PMIC_GPIO_SDC4_PWR_EN_N),
						0);
			set_bit(dev_id, &vreg_sts);
		} else {
			pr_debug("Disable Vdd dev_%d\n", dev_id);
			gpio_set_value_cansleep(
				PM8058_GPIO_PM_TO_SYS(PMIC_GPIO_SDC4_PWR_EN_N),
				1);
			clear_bit(dev_id, &vreg_sts);
		}
	}

	if (!enable || enabled_once[dev_id - 1])
			return 0;
	if (!curr)
		return -ENODEV;

	if (IS_ERR(curr))
		return PTR_ERR(curr);

	if (enable) {
		set_bit(dev_id, &vreg_sts);

		rc = regulator_enable(curr);
		if (rc)
			pr_err("%s: could not enable regulator: %d\n",
					__func__, rc);
		enabled_once[dev_id - 1] = 1;
	} else {
		clear_bit(dev_id, &vreg_sts);

		rc = regulator_disable(curr);
		if (rc)
			pr_err("%s: could not disable regulator: %d\n",
					__func__, rc);
	}
	return rc;
}

static uint32_t msm_sdcc_setup_power(struct device *dv, unsigned int vdd)
{
	int rc = 0;
	struct platform_device *pdev;

	pdev = container_of(dv, struct platform_device, dev);
	rc = msm_sdcc_setup_gpio(pdev->id, (vdd ? 1 : 0));
	if (rc)
		goto out;

	if (pdev->id == 4) /* S3 is always ON and cannot be disabled */
		rc = msm_sdcc_setup_vreg(pdev->id, (vdd ? 1 : 0));
out:
	return rc;
}

#endif

#ifdef CONFIG_MMC_MSM_SDC4_SUPPORT
static unsigned int msm7x30_sdcc_slot_status(struct device *dev)
{
	return (unsigned int)
		!gpio_get_value_cansleep(
			PM8058_GPIO_PM_TO_SYS(PMIC_GPIO_SD_DET - 1));
}
#endif

#ifdef CONFIG_MMC_MSM_SDC3_SUPPORT
static struct mmc_platform_data msm7x30_sdc3_data = {
	.ocr_mask	= MMC_VDD_165_195 | MMC_VDD_27_28 | MMC_VDD_28_29,
	.translate_vdd	= msm_sdcc_setup_power,
	.mmc_bus_width  = MMC_CAP_4_BIT_DATA,
	.sdiowakeup_irq = MSM_GPIO_TO_INT(118),
	.msmsdcc_fmin	= 144000,
	.msmsdcc_fmid	= 24576000,
	.msmsdcc_fmax	= 49152000,
	.nonremovable	= 1,
};
#endif

#ifdef CONFIG_MMC_MSM_SDC4_SUPPORT
static struct mmc_platform_data msm7x30_sdc4_data = {
	.ocr_mask	= MMC_VDD_27_28 | MMC_VDD_28_29,
	.translate_vdd	= msm_sdcc_setup_power,
	.mmc_bus_width  = MMC_CAP_4_BIT_DATA,
	.status      = msm7x30_sdcc_slot_status,
	.status_irq  = PM8058_GPIO_IRQ(PMIC8058_IRQ_BASE, PMIC_GPIO_SD_DET - 1),
	.irq_flags   = IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING,
	.msmsdcc_fmin	= 144000,
	.msmsdcc_fmid	= 24576000,
	.msmsdcc_fmax	= 49152000,
	.nonremovable	= 0,
};
#endif

static int mmc_regulator_init(int sdcc_no, const char *supply, int uV)
{
	int rc;

	BUG_ON(sdcc_no < 1 || sdcc_no > 4);

	sdcc_no--;

	sdcc_vreg_data[sdcc_no] = regulator_get(NULL, supply);

	if (IS_ERR(sdcc_vreg_data[sdcc_no])) {
		rc = PTR_ERR(sdcc_vreg_data[sdcc_no]);
		pr_err("%s: could not get regulator \"%s\": %d\n",
				__func__, supply, rc);
		goto out;
	}

	rc = regulator_set_voltage(sdcc_vreg_data[sdcc_no], uV, uV);

	if (rc) {
		pr_err("%s: could not set voltage for \"%s\" to %d uV: %d\n",
				__func__, supply, uV, rc);
		goto reg_free;
	}

	return rc;

reg_free:
	regulator_put(sdcc_vreg_data[sdcc_no]);
out:
	sdcc_vreg_data[sdcc_no] = NULL;
	return rc;
}

static void __init msm7x30_init_mmc(void)
{
#ifdef CONFIG_MMC_MSM_SDC3_SUPPORT
	if (mmc_regulator_init(3, "s3", 1800000))
		goto out3;

	msm_sdcc_setup_gpio(3, 1);
	msm_add_sdcc(3, &msm7x30_sdc3_data);
out3:
#endif
#ifdef CONFIG_MMC_MSM_SDC4_SUPPORT
	if (mmc_regulator_init(4, "mmc", 2850000))
		return;

	msm_add_sdcc(4, &msm7x30_sdc4_data);
#endif

}

/*
 * Attempts to identify the the hardware ID
 * of Zeus HW.
 */
static void __init zeus_detect_product(void)
{
	u8 hwid;
	const char *pzName[] = {
		[BOARD_HWID_UNK]    = "Unknown",
		[BOARD_HWID_DP1]    = "DP1",
		[BOARD_HWID_SP1]    = "SP1",
		[BOARD_HWID_DBZ3]   = "DBZ3",
		[BOARD_HWID_SP1_5]  = "SP1.5",
		[BOARD_HWID_SP1_6]  = "SP1.6",
		[BOARD_HWID_DBZ3_1] = "DBZ3.1",
		[BOARD_HWID_SP2]    = "SP2",
		[BOARD_HWID_DBZ3_2] = "DBZ3.2",
		[BOARD_HWID_SP3]    = "SP3",
		[BOARD_HWID_AP1]    = "AP1",
		[BOARD_HWID_PQ]     = "PQ",
	};

	int idmap[] = {
		BOARD_HWID_UNK,    BOARD_HWID_UNK,	/* 0,1 */
		BOARD_HWID_UNK,    BOARD_HWID_UNK,	/* 2,3 */
		BOARD_HWID_UNK,    BOARD_HWID_DBZ3_2,	/* 4,5 */
		BOARD_HWID_DBZ3_1, BOARD_HWID_DBZ3,	/* 6,7 */
		BOARD_HWID_DP1,    BOARD_HWID_SP1,	/* 8,9 */
		BOARD_HWID_SP1_5,  BOARD_HWID_SP1_6,	/* a,b */
		BOARD_HWID_SP2,    BOARD_HWID_SP3,	/* c,d */
		BOARD_HWID_AP1,    BOARD_HWID_PQ,	/* e,f */
	};

	/* Request GPIOs, and set the TLMM in a state where we can read them */
	gpio_request(43, "hwid_3");
	gpio_request(148, "hwid_2");
	gpio_request(149, "hwid_1");
	gpio_request(150, "hwid_0");
	gpio_request(38, "hwid_3_ex");
	gpio_tlmm_config(GPIO_CFG(43, 0,
			GPIO_CFG_INPUT, GPIO_CFG_PULL_UP,
			GPIO_CFG_2MA), GPIO_CFG_ENABLE);
	gpio_tlmm_config(GPIO_CFG(148, 0,
			GPIO_CFG_INPUT, GPIO_CFG_PULL_UP,
			GPIO_CFG_2MA), GPIO_CFG_ENABLE);
	gpio_tlmm_config(GPIO_CFG(149, 0,
			GPIO_CFG_INPUT, GPIO_CFG_PULL_UP,
			GPIO_CFG_2MA), GPIO_CFG_ENABLE);
	gpio_tlmm_config(GPIO_CFG(150, 0,
			GPIO_CFG_INPUT, GPIO_CFG_PULL_UP,
			GPIO_CFG_2MA), GPIO_CFG_ENABLE);
	/* GPIO 38 (NC) pin should be configured as no pull for DBZ3.0x */
	gpio_tlmm_config(GPIO_CFG(38, 0,
			GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL,
			GPIO_CFG_2MA), GPIO_CFG_ENABLE);

	hwid = 0;
	hwid |= (gpio_get_value(150) & 1) << 0;
	hwid |= (gpio_get_value(149) & 1) << 1;
	hwid |= (gpio_get_value(148) & 1) << 2;
	hwid |= (gpio_get_value(43) & 1) << 3;

	board_hwid = idmap[hwid];
	system_rev = board_hwid;

	printk(KERN_INFO "Zeus HWID: 0x%x (%s)\n", hwid, pzName[board_hwid]);

	/* Reconfigure the GPIOs so that we won't leak current */
	gpio_tlmm_config(GPIO_CFG(43, 0,
			GPIO_CFG_INPUT, GPIO_CFG_NO_PULL,
			GPIO_CFG_2MA), GPIO_CFG_DISABLE);
	gpio_tlmm_config(GPIO_CFG(148, 0,
			GPIO_CFG_INPUT, GPIO_CFG_NO_PULL,
			GPIO_CFG_2MA), GPIO_CFG_DISABLE);
	gpio_tlmm_config(GPIO_CFG(149, 0,
			GPIO_CFG_INPUT, GPIO_CFG_NO_PULL,
			GPIO_CFG_2MA), GPIO_CFG_DISABLE);
	gpio_tlmm_config(GPIO_CFG(150, 0,
			GPIO_CFG_INPUT, GPIO_CFG_NO_PULL,
			GPIO_CFG_2MA), GPIO_CFG_DISABLE);

	/* Free our allocations */
	gpio_free(43);
	gpio_free(148);
	gpio_free(149);
	gpio_free(150);
	gpio_free(38);
}

/*
 * Temporary place for hardware initialization until the devices in question
 * gets proper drivers
 */
static void __init zeus_temp_fixups(void)
{
	/* Power up cameras, but keeps both in RST */
	gpio_request(0, "maincam_rst");
	gpio_set_value(0, 0);	/* MCAM_RST_N */
	gpio_free(0);

	/* Tweak the NT3550 power */
	gpio_tlmm_config(GPIO_CFG(157, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
				GPIO_CFG_ENABLE);

	/* Since the sequencing for AKM & BMA needs to be L10 -> L8 */
	vreg_helper_on("gp4", 2850);	/* L10 */

	vreg_helper_on("wlan", 1800);	/* L13: touchpad VDIO */
	vreg_helper_on("gp10", 2800);	/* L16: touchpad */
}

static void __init msm7x30_init_nand(void)
{
	char *build_id;
	struct flash_platform_data *plat_data;

	build_id = socinfo_get_build_id();
	if (build_id == NULL) {
		pr_err("%s: Build ID not available from socinfo\n", __func__);
		return;
	}

	if (build_id[8] == 'C' &&
			!msm_gpios_request_enable(msm_nand_ebi2_cfg_data,
			ARRAY_SIZE(msm_nand_ebi2_cfg_data))) {
		plat_data = msm_device_nand.dev.platform_data;
		plat_data->interleave = 1;
		printk(KERN_INFO "%s: Interleave mode Build ID found\n",
			__func__);
	}
}

#ifdef CONFIG_SERIAL_MSM_CONSOLE
static struct msm_gpio uart3_config_data[] = {
	{ GPIO_CFG(53, 1, GPIO_CFG_INPUT,   GPIO_CFG_PULL_UP, GPIO_CFG_2MA), "UART3_Rx"},
	{ GPIO_CFG(54, 1, GPIO_CFG_OUTPUT,  GPIO_CFG_NO_PULL, GPIO_CFG_4MA), "UART3_Tx"},
};

static void msm7x30_init_uart3(void)
{
	msm_gpios_request_enable(uart3_config_data,
			ARRAY_SIZE(uart3_config_data));

}
#endif

static struct msm_spm_platform_data msm_spm_data __initdata = {
	.reg_base_addr = MSM_SAW0_BASE,

	.reg_init_values[MSM_SPM_REG_SAW_CFG] = 0x05,
	.reg_init_values[MSM_SPM_REG_SAW_SPM_CTL] = 0x18,
	.reg_init_values[MSM_SPM_REG_SAW_SPM_SLP_TMR_DLY] = 0x00006666,
	.reg_init_values[MSM_SPM_REG_SAW_SPM_WAKE_TMR_DLY] = 0xFF000666,

	.reg_init_values[MSM_SPM_REG_SAW_SLP_CLK_EN] = 0x01,
	.reg_init_values[MSM_SPM_REG_SAW_SLP_HSFS_PRECLMP_EN] = 0x03,
	.reg_init_values[MSM_SPM_REG_SAW_SLP_HSFS_POSTCLMP_EN] = 0x00,

	.reg_init_values[MSM_SPM_REG_SAW_SLP_CLMP_EN] = 0x01,
	.reg_init_values[MSM_SPM_REG_SAW_SLP_RST_EN] = 0x00,
	.reg_init_values[MSM_SPM_REG_SAW_SPM_MPM_CFG] = 0x00,

	.awake_vlevel = 0xF2,
	.retention_vlevel = 0xE0,
	.collapse_vlevel = 0x72,
	.retention_mid_vlevel = 0xE0,
	.collapse_mid_vlevel = 0xE0,

	.vctl_timeout_us = 50,
};

#ifdef CONFIG_INPUT_KEYRESET
#include <linux/keyreset.h>
/* keyreset platform device */
static int zeus_reset_keys_up[] = {
	KEY_VOLUMEDOWN,
	0
};

static struct keyreset_platform_data zeus_reset_keys_pdata = {
	.keys_up = zeus_reset_keys_up,
	.keys_down = {
		KEY_POWER,
		KEY_HOME,
		0
	},
};

struct platform_device zeus_reset_keys_device = {
	.name = KEYRESET_NAME,
	.dev    = {
		.platform_data = &zeus_reset_keys_pdata,
	},
};
#endif

static void __init msm7x30_init(void)
{
	unsigned smem_size;

	msm_clock_init(&msm7x30_clock_init_data);
#ifdef CONFIG_SERIAL_MSM_CONSOLE
	msm7x30_init_uart3();
#endif
	msm_spm_init(&msm_spm_data, 1);
	platform_device_register(&msm7x30_device_acpuclk);

#ifdef CONFIG_USB_MSM_OTG_72K
	msm_device_otg.dev.platform_data = &msm_otg_pdata;
#ifdef CONFIG_USB_GADGET
	msm_otg_pdata.swfi_latency =
 	msm_pm_data
 	[MSM_PM_SLEEP_MODE_RAMP_DOWN_AND_WAIT_FOR_INTERRUPT].latency;
	msm_device_gadget_peripheral.dev.platform_data = &msm_gadget_pdata;
#endif
#endif
	msm_uart_dm1_pdata.wakeup_irq = gpio_to_irq(136);
	msm_device_uart_dm1.dev.platform_data = &msm_uart_dm1_pdata;

	buses_init();

#ifdef CONFIG_MSM_SSBI
	msm_device_ssbi_pmic1.dev.platform_data =
				&msm7x30_ssbi_pm8058_pdata;
#endif

	platform_add_devices(msm_footswitch_devices,
			     msm_num_footswitch_devices);
	platform_add_devices(devices, ARRAY_SIZE(devices));
	zeus_temp_fixups();
#ifdef CONFIG_SEMC_CHARGER_USB_ARCH
	semc_chg_usb_set_supplicants(semc_chg_usb_supplied_to,
				  ARRAY_SIZE(semc_chg_usb_supplied_to));
#endif
#ifdef CONFIG_USB_EHCI_MSM_72K
	msm_add_host(0, &msm_usb_host_pdata);
#endif
	msm7x30_init_mmc();
	msm_qsd_spi_init();
	msm7x30_init_nand();
	atv_dac_power_init();
#ifdef CONFIG_INPUT_KEYRESET
	platform_device_register(&zeus_reset_keys_device);
#endif
	msm_fb_add_devices();
	msm_pm_set_platform_data(msm_pm_data, ARRAY_SIZE(msm_pm_data));
	BUG_ON(msm_pm_boot_init(&msm_pm_boot_pdata));
	msm_pm_register_irqs();
	msm_device_i2c_init();
	msm_device_i2c_2_init();
	qup_device_i2c_init();
	msm7x30_init_marimba();
#ifdef CONFIG_MSM7KV2_AUDIO
	snddev_poweramp_gpio_init();
	snddev_hsed_voltage_init();
	aux_pcm_gpio_init();
#endif
	zeus_detect_product();
#ifdef CONFIG_TOUCHSCREEN_CY8CTMA300
	cypress_touch_gpio_init();
#endif /* CONFIG_TOUCHSCREEN_CY8CTMA300 */

	i2c_register_board_info(0, msm_i2c_board_info,
			ARRAY_SIZE(msm_i2c_board_info));

	i2c_register_board_info(2, msm_marimba_board_info,
			ARRAY_SIZE(msm_marimba_board_info));

	i2c_register_board_info(4 /* QUP ID */, msm_camera_boardinfo,
				ARRAY_SIZE(msm_camera_boardinfo));

	spi_register_board_info(msm_spi_board_info,
				ARRAY_SIZE(msm_spi_board_info));

#ifdef CONFIG_I2C_SSBI
	msm_device_ssbi7.dev.platform_data = &msm_i2c_ssbi7_pdata;
#endif

	pm8058_gpios_init();

	boot_reason = *(unsigned int *)
		(smem_get_entry(SMEM_POWER_ON_STATUS_INFO, &smem_size));
	printk(KERN_NOTICE "Boot Reason = 0x%02x\n", boot_reason);
}

static unsigned pmem_sf_size = MSM_PMEM_SF_SIZE;
static int __init pmem_sf_size_setup(char *p)
{
	pmem_sf_size = memparse(p, NULL);
	return 0;
}
early_param("pmem_sf_size", pmem_sf_size_setup);

static unsigned fb_size;
static int __init fb_size_setup(char *p)
{
	fb_size = memparse(p, NULL);
	return 0;
}
early_param("fb_size", fb_size_setup);

static unsigned pmem_adsp_size = MSM_PMEM_ADSP_SIZE;
static int __init pmem_adsp_size_setup(char *p)
{
	pmem_adsp_size = memparse(p, NULL);
	return 0;
}
early_param("pmem_adsp_size", pmem_adsp_size_setup);

static unsigned pmem_kernel_ebi0_size = PMEM_KERNEL_EBI0_SIZE;
static int __init pmem_kernel_ebi0_size_setup(char *p)
{
	pmem_kernel_ebi0_size = memparse(p, NULL);
	return 0;
}
early_param("pmem_kernel_ebi0_size", pmem_kernel_ebi0_size_setup);

static struct memtype_reserve msm7x30_reserve_table[] __initdata = {
	[MEMTYPE_SMI] = {
	},
	[MEMTYPE_EBI0] = {
		.flags	=	MEMTYPE_FLAGS_1M_ALIGN,
	},
	[MEMTYPE_EBI1] = {
		.flags	=	MEMTYPE_FLAGS_1M_ALIGN,
	},
};

static void __init size_pmem_devices(void)
{
#ifdef CONFIG_ANDROID_PMEM
	android_pmem_adsp_pdata.size = pmem_adsp_size;
	android_pmem_pdata.size = pmem_sf_size;
#endif
}

static void __init reserve_memory_for(struct android_pmem_platform_data *p)
{
	msm7x30_reserve_table[p->memory_type].size += p->size;
}

static void __init reserve_pmem_memory(void)
{
#ifdef CONFIG_ANDROID_PMEM
	reserve_memory_for(&android_pmem_adsp_pdata);
	reserve_memory_for(&android_pmem_pdata);
	msm7x30_reserve_table[MEMTYPE_EBI0].size += pmem_kernel_ebi0_size;
#endif
}

static void __init msm7x30_calculate_reserve_sizes(void)
{
	size_pmem_devices();
	reserve_pmem_memory();
}

static int msm7x30_paddr_to_memtype(unsigned int paddr)
{
	if (paddr < phys_add)
		return MEMTYPE_EBI0;
	if (paddr >= phys_add && paddr < 0x80000000)
		return MEMTYPE_EBI1;
	return MEMTYPE_NONE;
}

static struct reserve_info msm7x30_reserve_info __initdata = {
	.memtype_reserve_table = msm7x30_reserve_table,
	.calculate_reserve_sizes = msm7x30_calculate_reserve_sizes,
	.paddr_to_memtype = msm7x30_paddr_to_memtype,
};

static void __init msm7x30_reserve(void)
{
	reserve_info = &msm7x30_reserve_info;
	msm_reserve();
#ifdef CONFIG_ANDROID_PERSISTENT_RAM
	add_persistent_ram();
#endif
}

static void __init msm7x30_allocate_memory_regions(void)
{
	void *addr;
	unsigned long size;

	size = fb_size ? : MSM_FB_SIZE;
	addr = alloc_bootmem_align(size, 0x1000);
	msm_fb_resources[0].start = __pa(addr);
	msm_fb_resources[0].end = msm_fb_resources[0].start + size - 1;
	pr_info("allocating %lu bytes at %p (%lx physical) for fb\n",
		size, addr, __pa(addr));
}

static void __init msm7x30_map_io(void)
{
	msm_shared_ram_phys = 0x00100000;
	msm_map_msm7x30_io();
	if (socinfo_init() < 0)
		printk(KERN_ERR "%s: socinfo_init() failed!\n",
		       __func__);
}

static void __init msm7x30_init_early(void)
{
	msm7x30_allocate_memory_regions();
}

static void __init msm7x30_fixup(struct tag *tags, char **cmdline,
				 struct meminfo *mi)
{
	mi->nr_banks = 3;
	mi->bank[0].start = DDR0_BANK_BASE;
	mi->bank[0].size = DDR0_BANK_SIZE;
	mi->bank[1].start = DDR1_BANK_BASE;
	mi->bank[1].size = DDR1_BANK_SIZE;
	mi->bank[2].start = DDR2_BANK_BASE;
	mi->bank[2].size = DDR2_BANK_SIZE;
}

MACHINE_START(SEMC_ZEUS, "zeus")
	.atag_offset = 0x100,
	.map_io = msm7x30_map_io,
	.reserve = msm7x30_reserve,
	.init_irq = msm7x30_init_irq,
	.init_machine = msm7x30_init,
	.timer = &msm_timer,
	.init_early = msm7x30_init_early,
	.handle_irq = vic_handle_irq,
	.fixup = msm7x30_fixup,
MACHINE_END
