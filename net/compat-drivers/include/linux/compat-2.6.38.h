#ifndef LINUX_26_38_COMPAT_H
#define LINUX_26_38_COMPAT_H

#include <linux/version.h>

#if (LINUX_VERSION_CODE < KERNEL_VERSION(2,6,38))

#include <linux/kernel.h>
#include <linux/skbuff.h>
#include <linux/etherdevice.h>
#include <net/sch_generic.h>

#define   PCI_MSIX_ENTRY_CTRL_MASKBIT  1

#define alloc_etherdev_mqs(sizeof_priv, tx_q, rx_q) alloc_etherdev_mq(sizeof_priv, tx_q)

/* MSI-X entry's format */
#define PCI_MSIX_ENTRY_SIZE            16
#define  PCI_MSIX_ENTRY_LOWER_ADDR     0
#define  PCI_MSIX_ENTRY_UPPER_ADDR     4
#define  PCI_MSIX_ENTRY_DATA           8
#define  PCI_MSIX_ENTRY_VECTOR_CTRL    12

#if (LINUX_VERSION_CODE > KERNEL_VERSION(2,6,30))
static inline void bstats_update(struct gnet_stats_basic_packed *bstats,
				 const struct sk_buff *skb)
{
	bstats->bytes += qdisc_pkt_len((struct sk_buff *) skb);
	bstats->packets += skb_is_gso(skb) ? skb_shinfo(skb)->gso_segs : 1;
}
static inline void qdisc_bstats_update(struct Qdisc *sch,
				       const struct sk_buff *skb)
{
	bstats_update(&sch->bstats, skb);
}
#else
/*
 * kernels <= 2.6.30 do not pass a const skb to qdisc_pkt_len, and
 * gnet_stats_basic_packed did not exist (see c1a8f1f1c8)
 */
static inline void bstats_update(struct gnet_stats_basic *bstats,
				 struct sk_buff *skb)
{
	bstats->bytes += qdisc_pkt_len(skb);
	bstats->packets += skb_is_gso(skb) ? skb_shinfo(skb)->gso_segs : 1;
}
static inline void qdisc_bstats_update(struct Qdisc *sch,
				       struct sk_buff *skb)
{
	bstats_update(&sch->bstats, skb);
}
#endif


/* rename member in struct mmc_host in include/linux/mmc/host.h */
#define max_segs	max_hw_segs


/* Exponentially weighted moving average (EWMA) */

/* For more documentation see lib/average.c */

struct ewma {
	unsigned long internal;
	unsigned long factor;
	unsigned long weight;
};

/* mask ewma_init as RHEL6 backports this */
#define ewma_init(a,b,c) compat_ewma_init(a,b,c)

extern void ewma_init(struct ewma *avg, unsigned long factor,
		      unsigned long weight);

/* mask ewma_add as RHEL6 backports this */
#define ewma_add(a,b) compat_ewma_add(a,b)

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
#define create_freezable_workqueue create_freezeable_workqueue

static inline int skb_checksum_start_offset(const struct sk_buff *skb)
{
	return skb->csum_start - skb_headroom(skb);
}

/* from include/linux/printk.h */ 
#define pr_emerg_once(fmt, ...)					\
	printk_once(KERN_EMERG pr_fmt(fmt), ##__VA_ARGS__)
#define pr_alert_once(fmt, ...)					\
	printk_once(KERN_ALERT pr_fmt(fmt), ##__VA_ARGS__)
#define pr_crit_once(fmt, ...)					\
	printk_once(KERN_CRIT pr_fmt(fmt), ##__VA_ARGS__)
#define pr_err_once(fmt, ...)					\
	printk_once(KERN_ERR pr_fmt(fmt), ##__VA_ARGS__)
#define pr_warn_once(fmt, ...)					\
	printk_once(KERN_WARNING pr_fmt(fmt), ##__VA_ARGS__)
#define pr_notice_once(fmt, ...)				\
	printk_once(KERN_NOTICE pr_fmt(fmt), ##__VA_ARGS__)
#define pr_info_once(fmt, ...)					\
	printk_once(KERN_INFO pr_fmt(fmt), ##__VA_ARGS__)
#define pr_cont_once(fmt, ...)					\
	printk_once(KERN_CONT pr_fmt(fmt), ##__VA_ARGS__)
#if defined(DEBUG)
#define pr_debug_once(fmt, ...)					\
	printk_once(KERN_DEBUG pr_fmt(fmt), ##__VA_ARGS__)
#else
#define pr_debug_once(fmt, ...)					\
	no_printk(KERN_DEBUG pr_fmt(fmt), ##__VA_ARGS__)
#endif

/* include/linux/netdevice.h */
#define alloc_netdev_mqs(sizeof_priv, name, setup, txqs, rxqs) \
	alloc_netdev_mq(sizeof_priv, name, setup, \
			max_t(unsigned int, txqs, rxqs))

#define ETH_P_LINK_CTL	0x886c		/* HPNA, wlan link local tunnel */

/**
 * is_unicast_ether_addr - Determine if the Ethernet address is unicast
 * @addr: Pointer to a six-byte array containing the Ethernet address
 *
 * Return true if the address is a unicast address.
 */
static inline int is_unicast_ether_addr(const u8 *addr)
{
	return !is_multicast_ether_addr(addr);
}

/* Backport of:
 *
 * commit 7ef88ad561457c0346355dfd1f53e503ddfde719
 * Author: Rusty Russell <rusty@rustcorp.com.au>
 * Date:   Mon Jan 24 14:45:10 2011 -0600
 *
 *     BUILD_BUG_ON: make it handle more cases
 */
#undef BUILD_BUG_ON
/**
 * BUILD_BUG_ON - break compile if a condition is true.
 * @condition: the condition which the compiler should know is false.
 *
 * If you have some code which relies on certain constants being equal, or
 * other compile-time-evaluated condition, you should use BUILD_BUG_ON to
 * detect if someone changes it.
 *
 * The implementation uses gcc's reluctance to create a negative array, but
 * gcc (as of 4.4) only emits that error for obvious cases (eg. not arguments
 * to inline functions).  So as a fallback we use the optimizer; if it can't
 * prove the condition is false, it will cause a link error on the undefined
 * "__build_bug_on_failed".  This error message can be harder to track down
 * though, hence the two different methods.
 */
#ifndef __OPTIMIZE__
#define BUILD_BUG_ON(condition) ((void)sizeof(char[1 - 2*!!(condition)]))
#else
extern int __build_bug_on_failed;
#define BUILD_BUG_ON(condition)					\
	do {							\
		((void)sizeof(char[1 - 2*!!(condition)]));	\
		if (condition) __build_bug_on_failed = 1;	\
	} while(0)
#endif

/* Backport of:
 *
 * commit e159489baa717dbae70f9903770a6a4990865887
 * Author: Tejun Heo <tj@kernel.org>
 * Date:   Sun Jan 9 23:32:15 2011 +0100
 *
 *     workqueue: relax lockdep annotation on flush_work()
 */
#ifdef CONFIG_DEBUG_LOCK_ALLOC
# ifdef CONFIG_PROVE_LOCKING
#  define lock_map_acquire_read(l)	lock_acquire(l, 0, 0, 2, 2, NULL, _THIS_IP_)
# else
#  define lock_map_acquire_read(l)	lock_acquire(l, 0, 0, 2, 1, NULL, _THIS_IP_)
# endif
#else
# define lock_map_acquire_read(l)		do { } while (0)
#endif

#endif /* (LINUX_VERSION_CODE < KERNEL_VERSION(2,6,38)) */

#endif /* LINUX_26_38_COMPAT_H */
