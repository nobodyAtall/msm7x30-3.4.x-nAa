/* Copyright (c) 2012-2013, The Linux Foundation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/slab.h>
#include <linux/irq.h>
#include <asm/fiq.h>
#include <asm/unwind.h>
#include <asm/hardware/gic.h>
#include <asm/cacheflush.h>
#include <mach/irqs-8625.h>
#include <mach/socinfo.h>
#include <mach/fiq.h>
#include "msm_watchdog.h"

#define MODULE_NAME "MSM7K_FIQ"

struct msm_watchdog_dump msm_dump_cpu_ctx;
static int fiq_counter;
static int msm_fiq_no;
void *msm7k_fiq_stack;

/* Called from the FIQ asm handler */
void msm7k_fiq_handler(void)
{
	struct irq_data *d;
	struct irq_chip *c;
	struct pt_regs ctx_regs;
	unsigned long flags;

	pr_info("Fiq is received %s\n", __func__);
	fiq_counter++;

	local_irq_save(flags);
	d = irq_get_irq_data(msm_fiq_no);
	c = irq_data_get_irq_chip(d);
	c->irq_mask(d);

	if (cpu_is_msm8625() || cpu_is_msm8625q())
		/* Clear the IRQ from the ENABLE_SET */
		gic_clear_irq_pending(msm_fiq_no);
	else
		/* Clear the IRQ from the VIC_INT_CLEAR0*/
		c->irq_ack(d);

	ctx_regs.ARM_pc = msm_dump_cpu_ctx.fiq_r14;
	ctx_regs.ARM_lr = msm_dump_cpu_ctx.svc_r14;
	ctx_regs.ARM_sp = msm_dump_cpu_ctx.svc_r13;
	ctx_regs.ARM_fp = msm_dump_cpu_ctx.usr_r11;
	unwind_backtrace(&ctx_regs, current);
	arch_trigger_all_cpu_backtrace();
	local_irq_restore(flags);

	flush_cache_all();
	outer_flush_all();
	return;
}

struct fiq_handler msm7k_fh = {
	.name = MODULE_NAME,
};

static int __init msm_setup_fiq_handler(void)
{
	int ret = 0;

	claim_fiq(&msm7k_fh);
	set_fiq_handler(&msm7k_fiq_start, msm7k_fiq_length);
	msm7k_fiq_stack = (void *)__get_free_pages(GFP_KERNEL,
				THREAD_SIZE_ORDER);
	if (msm7k_fiq_stack == NULL) {
		pr_err("FIQ STACK SETUP IS NOT SUCCESSFUL\n");
		return -ENOMEM;
	}

	fiq_set_type(msm_fiq_no, IRQF_TRIGGER_RISING);
	if (cpu_is_msm8625() || cpu_is_msm8625q())
		gic_set_irq_secure(msm_fiq_no);
	else
		msm_fiq_select(msm_fiq_no);

	enable_irq(msm_fiq_no);
	pr_info("%s : MSM FIQ handler setup--done\n", __func__);
	return ret;
}

static int __init init7k_fiq(void)
{
	if (cpu_is_msm8625() || cpu_is_msm8625q())
		msm_fiq_no = MSM8625_INT_A9_M2A_2;
	else
		msm_fiq_no = INT_A9_M2A_2;

	if (msm_setup_fiq_handler())
		pr_err("MSM FIQ INIT FAILED\n");

	return 0;
}
fs_initcall(init7k_fiq);
