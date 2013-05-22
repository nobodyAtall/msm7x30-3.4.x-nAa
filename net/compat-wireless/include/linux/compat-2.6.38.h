#ifndef LINUX_26_38_COMPAT_H
#define LINUX_26_38_COMPAT_H

#include <linux/version.h>

#if (LINUX_VERSION_CODE < KERNEL_VERSION(2,6,38))

#include <linux/kernel.h>
#include <linux/skbuff.h>

/* rename member in struct mmc_host in include/linux/mmc/host.h */
#define max_segs	max_hw_segs


/* Exponentially weighted moving average (EWMA) */

/* For more documentation see lib/average.c */

struct ewma {
	unsigned long internal;
	unsigned long factor;
	unsigned long weight;
};

extern void ewma_init(struct ewma *avg, unsigned long factor,
		      unsigned long weight);

extern struct ewma *ewma_add(struct ewma *avg, unsigned long val);

/**
 * ewma_read() - Get average value
 * @avg: Average structure
 *
 * Returns the average value held in @avg.
 */
static inline unsigned long ewma_read(const struct ewma *avg)
{
	return DIV_ROUND_CLOSEST(avg->internal, avg->factor);
}

#define pr_warn pr_warning

static inline int skb_checksum_start_offset(const struct sk_buff *skb)
{
	return skb->csum_start - skb_headroom(skb);
}

#endif /* (LINUX_VERSION_CODE < KERNEL_VERSION(2,6,38)) */

#endif /* LINUX_26_38_COMPAT_H */
