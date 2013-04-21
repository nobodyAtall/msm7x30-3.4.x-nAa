#ifndef LINUX_3_8_COMPAT_H
#define LINUX_3_8_COMPAT_H

#include <linux/version.h>

#if (LINUX_VERSION_CODE < KERNEL_VERSION(3,8,0))

#include <linux/hid.h>
#include <linux/netdevice.h>
#include <linux/efi.h>

#if (LINUX_VERSION_CODE < KERNEL_VERSION(3,7,8))
extern void netdev_set_default_ethtool_ops(struct net_device *dev,
					   const struct ethtool_ops *ops);
#endif

#define HID_BUS_ANY                            0xffff
#define HID_GROUP_ANY                          0x0000

#define  PCI_EXP_LNKCTL_ASPM_L0S  0x01 /* L0s Enable */
#define  PCI_EXP_LNKCTL_ASPM_L1   0x02 /* L1 Enable */

extern bool hid_ignore(struct hid_device *);

/* This backports:
 *
 * commit 4b20db3de8dab005b07c74161cb041db8c5ff3a7
 * Author: Thomas Hellstrom <thellstrom@vmware.com>
 * Date:   Tue Nov 6 11:31:49 2012 +0000
 *
 *	kref: Implement kref_get_unless_zero v3
 */
/**
 * kref_get_unless_zero - Increment refcount for object unless it is zero.
 * @kref: object.
 *
 * Return non-zero if the increment succeeded. Otherwise return 0.
 *
 * This function is intended to simplify locking around refcounting for
 * objects that can be looked up from a lookup structure, and which are
 * removed from that lookup structure in the object destructor.
 * Operations on such objects require at least a read lock around
 * lookup + kref_get, and a write lock around kref_put + remove from lookup
 * structure. Furthermore, RCU implementations become extremely tricky.
 * With a lookup followed by a kref_get_unless_zero *with return value check*
 * locking in the kref_put path can be deferred to the actual removal from
 * the lookup structure and RCU lookups become trivial.
 */
static inline int __must_check kref_get_unless_zero(struct kref *kref)
{
	return atomic_add_unless(&kref->refcount, 1, 0);
}

/* This backports:
 *
 * commit 83e68189745ad931c2afd45d8ee3303929233e7f
 * Author: Matt Fleming <matt.fleming@intel.com>
 * Date:   Wed Nov 14 09:42:35 2012 +0000
 *
 *     efi: Make 'efi_enabled' a function to query EFI facilities
 *
 */
/* check first if this was already backported */
#ifndef EFI_BOOT
/*
 * We play games with efi_enabled so that the compiler will, if
 * possible, remove EFI-related code altogether.
 */
#define EFI_BOOT		0	/* Were we booted from EFI? */
#define EFI_SYSTEM_TABLES	1	/* Can we use EFI system tables? */
#define EFI_CONFIG_TABLES	2	/* Can we use EFI config tables? */
#define EFI_RUNTIME_SERVICES	3	/* Can we use runtime services? */
#define EFI_MEMMAP		4	/* Can we use EFI memory map? */
#define EFI_64BIT		5	/* Is the firmware 64-bit? */

#ifdef CONFIG_EFI
# ifdef CONFIG_X86
static inline int compat_efi_enabled(int facility)
{
	switch (facility) {
	case EFI_BOOT:
		return efi_enabled;
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(3,4,0))
	case EFI_64BIT:
		return efi_64bit;
#endif
	default:
		printk(KERN_ERR "can not translate efi_enabled() to old values completly\n");
		return efi_enabled;
	}
}
# else
static inline int compat_efi_enabled(int facility)
{
	return 1;
}
# endif
#else
static inline int compat_efi_enabled(int facility)
{
	return 0;
}
#endif
#ifdef efi_enabled
#undef efi_enabled
#endif
#define efi_enabled(facility) compat_efi_enabled(facility)
#endif /* EFI_BOOT */

#endif /* (LINUX_VERSION_CODE < KERNEL_VERSION(3,8,0)) */

#endif /* LINUX_3_8_COMPAT_H */
