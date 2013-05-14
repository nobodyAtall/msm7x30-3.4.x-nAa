/*
 * Copyright (C) 2013 Vassilis Tsogkas (tsogkas@ceid.upatras.gr)
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/delay.h>
#include <linux/device.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <asm/gpio.h>
#include <linux/slab.h>
#include <linux/skbuff.h>
#include <linux/ti_wilink_st.h>
#define WILINK_UART_DEV_NAME "/dev/ttyHS0"

static int bt_on;

static uint32_t bt_config_on_gpios[] = {
	GPIO_CFG(134, 1, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_4MA),
	GPIO_CFG(135, 1, GPIO_CFG_INPUT,  GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
	GPIO_CFG(136, 1, GPIO_CFG_INPUT,  GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
	GPIO_CFG(137, 1, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_4MA),
	GPIO_CFG(103, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
};

static uint32_t bt_config_off_gpios[] = {
	GPIO_CFG(134, 0, GPIO_CFG_INPUT,  GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
	GPIO_CFG(135, 0, GPIO_CFG_INPUT,  GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
	GPIO_CFG(136, 0, GPIO_CFG_INPUT,  GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
	GPIO_CFG(137, 0, GPIO_CFG_INPUT,  GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
	GPIO_CFG(103, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
};

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

static void bluetooth_init(void)
{
	config_gpio_table(bt_config_off_gpios, ARRAY_SIZE(bt_config_off_gpios));
	gpio_set_value(103, 0);
	bt_on = 0;
}

static int bluetooth_power(int on)
{
	if (on && !bt_on) {
		config_gpio_table(bt_config_on_gpios, ARRAY_SIZE(bt_config_on_gpios));
		gpio_set_value(103, 1);
	} else if (!on && bt_on) {
		config_gpio_table(bt_config_off_gpios, ARRAY_SIZE(bt_config_off_gpios));
		gpio_set_value(103, 0);
	}
	bt_on = on;
	return 0;
}

static int wilink_enable(struct kim_data_s *data)
{
	bluetooth_power(1);
	pr_info("%s\n", __func__);
	return 0;
}

static int wilink_disable(struct kim_data_s *data)
{
	bluetooth_power(0);
	pr_info("%s\n", __func__);
	return 0;
}

static int wilink_awake(struct kim_data_s *data)
{
	pr_info("%s\n", __func__);
	return 0;
}

static int wilink_asleep(struct kim_data_s *data)
{
	pr_info("%s\n", __func__);
	return 0;
}

int wilink_suspend(struct platform_device *pdev, pm_message_t state)
{
	pr_info("%s\n", __func__);
	return 0;
}

int wilink_resume(struct platform_device *pdev)
{
	pr_info("%s\n", __func__);
	return 0;
}

static struct ti_st_plat_data wilink_pdata = {
	.dev_name = WILINK_UART_DEV_NAME,
	.flow_cntrl = 1,
	.baud_rate = 3000000,
	.chip_enable = wilink_enable,
	.chip_disable = wilink_disable,
	.chip_awake = wilink_awake,
	.chip_asleep = wilink_asleep,
	.suspend = wilink_suspend,
	.resume = wilink_resume,
};

static struct platform_device btwilink_device = {
	.name = "btwilink",
	.id = -1,
};

/* wl127x BT, FM, GPS connectivity chip */
static struct platform_device wl1271_device = {
	.name   = "kim",
	.id     = -1,
	.dev.platform_data = &wilink_pdata,
};

static int __init mogami_bluedroid_init(void)
{
	bluetooth_init();
	platform_device_register(&wl1271_device);
	platform_device_register(&btwilink_device);
	return 0;
}

static void __exit mogami_bluedroid_exit(void)
{
	// TODO
	return;
}

module_init(mogami_bluedroid_init);
module_exit(mogami_bluedroid_exit);
MODULE_DESCRIPTION("mogami_bluedroid");
MODULE_AUTHOR("Vassilis Tsogkas <tsogkas@ceid.upatras.gr>");
MODULE_LICENSE("GPL");
