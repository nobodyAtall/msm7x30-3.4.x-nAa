#ifndef LINUX_3_0_COMPAT_H
#define LINUX_3_0_COMPAT_H

#include <linux/version.h>
#include <linux/sched.h>

#if (LINUX_VERSION_CODE < KERNEL_VERSION(3,0,0))

/*
 * since commit 1c5cae815d19ffe02bdfda1260949ef2b1806171
 * "net: call dev_alloc_name from register_netdevice" dev_alloc_name is
 * called automatically. This is not implemented in older kernel
 * versions so it will result in device wrong names.
 */
static inline int register_netdevice_name(struct net_device *dev)
{
	int err;

	if (strchr(dev->name, '%')) {
		err = dev_alloc_name(dev, dev->name);
		if (err < 0)
			return err;
	}

	return register_netdevice(dev);
}

#define register_netdevice(dev) register_netdevice_name(dev)

#endif /* (LINUX_VERSION_CODE < KERNEL_VERSION(3,0,0)) */

#endif /* LINUX_3_0_COMPAT_H */
