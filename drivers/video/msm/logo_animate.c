/* drivers/video/msm/logo.c
 *
 * Displays boot animation using a series of images in RLE 565 format
 *
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
#include <asm/uaccess.h>
#include <linux/module.h>
#include <linux/fb.h>
#include <linux/delay.h>
#include <linux/syscalls.h>

#define MAX_FRAMES 10000
#define MS_BETWEEN_FRAMES 50

int load_565rle_image(char *filename, bool bf_supported);
struct delayed_work rle_animate_work;

static void load_565rle_animate(struct work_struct *work)
{
	int i, ret = 0, bf_supported = 0;
	char filename [20];
	struct fb_info *info = registered_fb[0];
	set_fs(KERNEL_DS);
	printk(KERN_INFO "Starting kernel boot animation\n");
	for (i = 1; i < MAX_FRAMES; i++) {
		sprintf(filename, "/bootlogo/%d.rle", i);
		ret = load_565rle_image(filename, bf_supported);
		sys_unlink(filename);
		if (ret == -ENOENT)
			break;
		info->fbops->fb_open(info, 0);
		info->fbops->fb_pan_display(&info->var, info);
		msleep(MS_BETWEEN_FRAMES);
	}
}

static int __init logo_animate_init(void)
{
	INIT_DELAYED_WORK(&rle_animate_work, load_565rle_animate);
	schedule_delayed_work(&rle_animate_work, 5 * HZ);
	return 0;
}

static void __exit logo_animate_exit(void)
{
	return;
}

module_init(logo_animate_init);
module_exit(logo_animate_exit);
MODULE_DESCRIPTION("logo_animate");
MODULE_AUTHOR("Vassilis Tsogkas <tsogkas@ceid.upatras.gr>");
MODULE_LICENSE("GPL");
