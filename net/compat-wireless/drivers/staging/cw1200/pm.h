/*
 * Mac80211 power management interface for ST-Ericsson CW1200 mac80211 drivers
 *
 * Copyright (c) 2011, ST-Ericsson
 * Author: Dmitry Tarnyagin <dmitry.tarnyagin@stericsson.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef PM_H_INCLUDED
#define PM_H_INCLUDED

#ifdef CONFIG_WAKELOCK
#include <linux/wakelock.h>
#endif

/* ******************************************************************** */
/* mac80211 API								*/

#ifdef CONFIG_PM

/* extern */  struct cw1200_common;
/* private */ struct cw1200_suspend_state;

struct cw1200_pm_state {
	struct cw1200_suspend_state *suspend_state;
#ifdef CONFIG_WAKELOCK
	struct wake_lock wakelock;
#else
	struct timer_list stay_awake;
#endif
	spinlock_t lock;
};

int cw1200_pm_init(struct cw1200_pm_state *pm,
		    struct cw1200_common *priv);
void cw1200_pm_deinit(struct cw1200_pm_state *pm);
void cw1200_pm_stay_awake(struct cw1200_pm_state *pm,
			  unsigned long tmo);
int cw1200_wow_suspend(struct ieee80211_hw *hw,
		       struct cfg80211_wowlan *wowlan);
int cw1200_wow_resume(struct ieee80211_hw *hw);

int cw1200_can_suspend(struct cw1200_common *priv);
#endif /* CONFIG_PM */

#endif
