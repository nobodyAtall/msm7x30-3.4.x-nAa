/*
 * Mac80211 STA API for ST-Ericsson CW1200 drivers
 *
 * Copyright (c) 2010, ST-Ericsson
 * Copyright (C) 2012 Sony Mobile Communications AB. All rights reserved.
 *
 * Author: Dmitry Tarnyagin <dmitry.tarnyagin@stericsson.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/vmalloc.h>
#include <linux/sched.h>
#include <linux/firmware.h>

#include "cw1200.h"
#include "sta.h"
#include "ap.h"
#include "fwio.h"
#include "bh.h"
#include "debug.h"

#if defined(CONFIG_CW1200_STA_DEBUG)
#define sta_printk pr_debug
#else
#define sta_printk(...)
#endif

static inline void __cw1200_free_event_queue(struct list_head *list)
{
	while (!list_empty(list)) {
		struct cw1200_wsm_event *event =
			list_first_entry(list, struct cw1200_wsm_event,
			link);
		list_del(&event->link);
		kfree(event);
	}
}

static inline void __cw1200_bf_configure(struct cw1200_common *priv)
{
	priv->bf_table.numOfIEs = __cpu_to_le32(3);
	priv->bf_table.entry[0].ieId = WLAN_EID_VENDOR_SPECIFIC;
	priv->bf_table.entry[0].actionFlags = WSM_BEACON_FILTER_IE_HAS_CHANGED |
					WSM_BEACON_FILTER_IE_NO_LONGER_PRESENT |
					WSM_BEACON_FILTER_IE_HAS_APPEARED;
	priv->bf_table.entry[0].oui[0] = 0x50;
	priv->bf_table.entry[0].oui[1] = 0x6F;
	priv->bf_table.entry[0].oui[2] = 0x9A;

	priv->bf_table.entry[1].ieId = WLAN_EID_ERP_INFO;
	priv->bf_table.entry[1].actionFlags = WSM_BEACON_FILTER_IE_HAS_CHANGED |
						WSM_BEACON_FILTER_IE_NO_LONGER_PRESENT |
						WSM_BEACON_FILTER_IE_HAS_APPEARED;

	priv->bf_table.entry[2].ieId = WLAN_EID_HT_INFORMATION;
	priv->bf_table.entry[2].actionFlags = WSM_BEACON_FILTER_IE_HAS_CHANGED |
						WSM_BEACON_FILTER_IE_NO_LONGER_PRESENT |
						WSM_BEACON_FILTER_IE_HAS_APPEARED;

	priv->bf_control.enabled = WSM_BEACON_FILTER_ENABLE;
}

/* ******************************************************************** */
/* STA API								*/

int cw1200_start(struct ieee80211_hw *dev)
{
	struct cw1200_common *priv = dev->priv;
	int ret;

	cw1200_pm_stay_awake(&priv->pm_state, HZ);

	ret = cw1200_core_start(priv);
	if (WARN_ON(ret))
		return ret;

	mutex_lock(&priv->conf_mutex);

	/* default EDCA */
	WSM_EDCA_SET(&priv->edca, 0, 0x0002, 0x0003, 0x0007, 47, 0xc8, false);
	WSM_EDCA_SET(&priv->edca, 1, 0x0002, 0x0007, 0x000f, 94, 0xc8, false);
	WSM_EDCA_SET(&priv->edca, 2, 0x0003, 0x000f, 0x03ff, 0, 0xc8, false);
	WSM_EDCA_SET(&priv->edca, 3, 0x0007, 0x000f, 0x03ff, 0, 0xc8, false);
	ret = wsm_set_edca_params(priv, &priv->edca);
	if (WARN_ON(ret))
		goto out;

	ret = cw1200_set_uapsd_param(priv, &priv->edca);
	if (WARN_ON(ret))
		goto out;

	memset(priv->bssid, ~0, ETH_ALEN);
	memcpy(priv->mac_addr, dev->wiphy->perm_addr, ETH_ALEN);
	priv->mode = NL80211_IFTYPE_MONITOR;
	priv->softled_state = 0;
	priv->wep_default_key_id = -1;

	priv->cqm_link_loss_count = 60;
	priv->cqm_beacon_loss_count = 20;

	priv->block_ack_enabled = false;
	/* Temporary configuration - beacon filter table */
	__cw1200_bf_configure(priv);

	ret = cw1200_setup_mac(priv);
	if (WARN_ON(ret))
		goto out;

	/* err = cw1200_set_leds(priv); */

out:
	mutex_unlock(&priv->conf_mutex);
	if (ret)
		cw1200_core_stop(priv);
	return ret;
}

void cw1200_stop(struct ieee80211_hw *dev)
{
	struct cw1200_common *priv = dev->priv;
	LIST_HEAD(list);
	int i;

	wsm_lock_tx(priv);

	while (down_trylock(&priv->scan.lock)) {
		/* Scan is in progress. Force it to stop. */
		priv->scan.req = NULL;
		schedule();
	}
	up(&priv->scan.lock);

	cancel_delayed_work_sync(&priv->scan.probe_work);
	cancel_delayed_work_sync(&priv->scan.timeout);
	cancel_delayed_work_sync(&priv->clear_recent_scan_work);
	cancel_delayed_work_sync(&priv->join_timeout);
	cancel_delayed_work_sync(&priv->bss_loss_work);
	cancel_delayed_work_sync(&priv->connection_loss_work);
	cancel_delayed_work_sync(&priv->link_id_gc_work);
	flush_workqueue(priv->workqueue);
	del_timer_sync(&priv->mcast_timeout);

	mutex_lock(&priv->conf_mutex);
	priv->mode = NL80211_IFTYPE_UNSPECIFIED;
	priv->listening = false;
	priv->setup_mac_done = false;
	priv->softled_state = 0;
	/* cw1200_set_leds(priv); */

	spin_lock(&priv->event_queue_lock);
	list_splice_init(&priv->event_queue, &list);
	spin_unlock(&priv->event_queue_lock);
	__cw1200_free_event_queue(&list);

	priv->delayed_link_loss = 0;
	spin_lock(&priv->bss_loss_lock);
	priv->bss_loss_status = CW1200_BSS_LOSS_NONE;
	priv->bss_loss_checking = 0;
	spin_unlock(&priv->bss_loss_lock);
	priv->join_status = CW1200_JOIN_STATUS_PASSIVE;
	priv->join_pending = false;

	for (i = 0; i < 4; i++)
		cw1200_queue_clear(&priv->tx_queue[i]);
	mutex_unlock(&priv->conf_mutex);

	/* HACK! */
	if (atomic_xchg(&priv->tx_lock, 1) != 1)
		sta_printk(KERN_DEBUG "[STA] TX is force-unlocked "
			"due to stop request.\n");

	wsm_unlock_tx(priv);
	atomic_xchg(&priv->tx_lock, 0); /* for recovery to work */
	cw1200_core_stop(priv);
}

int cw1200_add_interface(struct ieee80211_hw *dev,
			 struct ieee80211_vif *vif)
{
	int ret;
	struct cw1200_common *priv = dev->priv;
	/* __le32 auto_calibration_mode = __cpu_to_le32(1); */
	sta_printk(KERN_DEBUG "%s called, type: %d\n", __func__, vif->type);

	mutex_lock(&priv->conf_mutex);

	if (priv->mode != NL80211_IFTYPE_MONITOR) {
		mutex_unlock(&priv->conf_mutex);
		return -EOPNOTSUPP;
	}

	switch (vif->type) {
	case NL80211_IFTYPE_STATION:
	case NL80211_IFTYPE_ADHOC:
	case NL80211_IFTYPE_MESH_POINT:
	case NL80211_IFTYPE_AP:
		priv->mode = vif->type;
		break;
	default:
		mutex_unlock(&priv->conf_mutex);
		return -EOPNOTSUPP;
	}

	priv->vif = vif;
	memcpy(priv->mac_addr, vif->addr, ETH_ALEN);

	ret = WARN_ON(cw1200_setup_mac(priv));
	/* Enable auto-calibration */
	/* Exception in subsequent channel switch; disabled.
	WARN_ON(wsm_write_mib(priv, WSM_MIB_ID_SET_AUTO_CALIBRATION_MODE,
		&auto_calibration_mode, sizeof(auto_calibration_mode)));
	*/

	mutex_unlock(&priv->conf_mutex);
	return ret;
}

void cw1200_remove_interface(struct ieee80211_hw *dev,
			     struct ieee80211_vif *vif)
{
	struct cw1200_common *priv = dev->priv;
	struct wsm_reset reset = {
		.reset_statistics = true,
	};
	int i;

	sta_printk(KERN_DEBUG "%s called \n", __func__);
	mutex_lock(&priv->conf_mutex);
	wsm_lock_tx(priv);
	switch (priv->join_status) {
	case CW1200_JOIN_STATUS_JOINING:
	case CW1200_JOIN_STATUS_PRE_STA:
	case CW1200_JOIN_STATUS_STA:
		wsm_lock_tx(priv);
		if (queue_work(priv->workqueue, &priv->unjoin_work) <= 0)
			wsm_unlock_tx(priv);
		break;
	case CW1200_JOIN_STATUS_AP:
		for (i = 0; priv->link_id_map; ++i) {
			if (priv->link_id_map & BIT(i)) {
				reset.link_id = i;
				wsm_reset(priv, &reset);
				priv->link_id_map &= ~BIT(i);
			}
		}
		memset(priv->link_id_db, 0,
				sizeof(priv->link_id_db));
		priv->sta_asleep_mask = 0;
		priv->enable_beacon = false;
		priv->tx_multicast = false;
		priv->aid0_bit_set = false;
		priv->buffered_multicasts = false;
		priv->pspoll_mask = 0;
		reset.link_id = 0;
		wsm_reset(priv, &reset);
		break;
	case CW1200_JOIN_STATUS_MONITOR:
		cw1200_update_listening(priv, false);
		break;
	default:
		break;
	}
	priv->vif = NULL;
	priv->mode = NL80211_IFTYPE_MONITOR;
	memset(priv->mac_addr, 0, ETH_ALEN);
	memset(priv->bssid, 0, ETH_ALEN);
	memset(&priv->p2p_ps_modeinfo, 0, sizeof(priv->p2p_ps_modeinfo));
	cw1200_free_keys(priv);
	cw1200_setup_mac(priv);
	priv->listening = false;
	priv->join_status = CW1200_JOIN_STATUS_PASSIVE;
	if (!__cw1200_flush(priv, true))
		wsm_unlock_tx(priv);
	wsm_unlock_tx(priv);

	mutex_unlock(&priv->conf_mutex);
}

int cw1200_change_interface(struct ieee80211_hw *dev,
			    struct ieee80211_vif *vif,
			    enum nl80211_iftype new_type,
			    bool p2p)
{
	int ret = 0;
	sta_printk(KERN_DEBUG "%s called: new_type: %d (%d), "
		   "old_type: %d (%d)\n", __func__, new_type,
		   p2p, vif->type, vif->p2p);

	if (new_type != vif->type || vif->p2p != p2p) {
		cw1200_remove_interface(dev, vif);
		vif->type = new_type;
		vif->p2p = p2p;
		ret = cw1200_add_interface(dev, vif);
	}

	return ret;
}

int cw1200_config(struct ieee80211_hw *dev, u32 changed)
{
	int ret = 0;
	struct cw1200_common *priv = dev->priv;
	struct ieee80211_conf *conf = &dev->conf;

	down(&priv->scan.lock);
	mutex_lock(&priv->conf_mutex);
	/* TODO: IEEE80211_CONF_CHANGE_QOS */
	if (changed & IEEE80211_CONF_CHANGE_POWER) {
		priv->output_power = conf->power_level;
		sta_printk(KERN_DEBUG "[STA] TX power: %d\n",
				priv->output_power);
		WARN_ON(wsm_set_output_power(priv, priv->output_power * 10));
	}

	if ((changed & IEEE80211_CONF_CHANGE_CHANNEL) &&
			(priv->channel != conf->channel)) {
		struct ieee80211_channel *ch = conf->channel;
		struct wsm_switch_channel channel = {
			.newChannelNumber = ch->hw_value,
		};
		sta_printk(KERN_DEBUG "[STA] Freq %d (wsm ch: %d).\n",
			ch->center_freq, ch->hw_value);

		ret = WARN_ON(__cw1200_flush(priv, false));
		if (!ret) {
			ret = WARN_ON(wsm_switch_channel(priv, &channel));
			if (!ret) {
				ret = wait_event_timeout(
					priv->channel_switch_done,
					!priv->channel_switch_in_progress,
					3 * HZ);
				/* TODO: We should check also switch channel
				 * complete indication
				 */
				if (ret) {
					priv->channel = ch;
					ret = 0;
				} else
					ret = -ETIMEDOUT;
			} else
				wsm_unlock_tx(priv);
		}
	}

	if (changed & IEEE80211_CONF_CHANGE_PS) {
		if (!(conf->flags & IEEE80211_CONF_PS))
			priv->powersave_mode.pmMode = WSM_PSM_ACTIVE;
		else if (conf->dynamic_ps_timeout <= 0)
			priv->powersave_mode.pmMode = WSM_PSM_PS;
		else
			priv->powersave_mode.pmMode = WSM_PSM_FAST_PS;

		/* Firmware requires that value for this 1-byte field must
		 * be specified in units of 500us. Values above the 128ms
		 * threshold are not supported. */
		if (conf->dynamic_ps_timeout >= 0x80)
			priv->powersave_mode.fastPsmIdlePeriod = 0xFF;
		else
			priv->powersave_mode.fastPsmIdlePeriod =
					conf->dynamic_ps_timeout << 1;

		if (priv->join_status == CW1200_JOIN_STATUS_STA &&
				priv->bss_params.aid)
			cw1200_set_pm(priv, &priv->powersave_mode);
	}

#if defined(CONFIG_CW1200_USE_STE_EXTENSIONS)
	if (changed & IEEE80211_CONF_CHANGE_P2P_PS) {
		struct wsm_p2p_ps_modeinfo *modeinfo;
		modeinfo = &priv->p2p_ps_modeinfo;
		sta_printk(KERN_DEBUG "[STA] IEEE80211_CONF_CHANGE_P2P_PS\n");
		sta_printk(KERN_DEBUG "[STA] Legacy PS: %d for AID %d "
			"in %d mode.\n", conf->p2p_ps.legacy_ps,
			priv->bss_params.aid, priv->join_status);

		if (conf->p2p_ps.legacy_ps >= 0) {
			if (conf->p2p_ps.legacy_ps > 0)
				priv->powersave_mode.pmMode = WSM_PSM_PS;
			else
				priv->powersave_mode.pmMode = WSM_PSM_ACTIVE;

			if (priv->join_status == CW1200_JOIN_STATUS_STA)
				cw1200_set_pm(priv, &priv->powersave_mode);
		}

		sta_printk(KERN_DEBUG "[STA] CTWindow: %d\n",
			conf->p2p_ps.ctwindow);
		if (conf->p2p_ps.ctwindow >= 128)
			modeinfo->oppPsCTWindow = 127;
		else if (conf->p2p_ps.ctwindow >= 0)
			modeinfo->oppPsCTWindow = conf->p2p_ps.ctwindow;

		sta_printk(KERN_DEBUG "[STA] Opportunistic: %d\n",
			conf->p2p_ps.opp_ps);
		switch (conf->p2p_ps.opp_ps) {
		case 0:
			modeinfo->oppPsCTWindow &= ~(BIT(7));
			break;
		case 1:
			modeinfo->oppPsCTWindow |= BIT(7);
			break;
		default:
			break;
		}

		sta_printk(KERN_DEBUG "[STA] NOA: %d, %d, %d, %d\n",
			conf->p2p_ps.count,
			conf->p2p_ps.start,
			conf->p2p_ps.duration,
			conf->p2p_ps.interval);
		/* Notice of Absence */
		modeinfo->count = conf->p2p_ps.count;

		if (conf->p2p_ps.count) {
			/* In case P2P_GO we need some extra time to be sure
			 * we will update beacon/probe_resp IEs correctly */
#define NOA_DELAY_START_MS	300
			if (priv->join_status == CW1200_JOIN_STATUS_AP)
				modeinfo->startTime =
					__cpu_to_le32(conf->p2p_ps.start +
						      NOA_DELAY_START_MS);
			else
				modeinfo->startTime =
					__cpu_to_le32(conf->p2p_ps.start);
			modeinfo->duration =
				__cpu_to_le32(conf->p2p_ps.duration);
			modeinfo->interval =
				 __cpu_to_le32(conf->p2p_ps.interval);
			modeinfo->dtimCount = 1;
			modeinfo->reserved = 0;
		} else {
			modeinfo->dtimCount = 0;
			modeinfo->startTime = 0;
			modeinfo->reserved = 0;
			modeinfo->duration = 0;
			modeinfo->interval = 0;
		}

#if defined(CONFIG_CW1200_STA_DEBUG)
		print_hex_dump_bytes("p2p_set_ps_modeinfo: ",
				     DUMP_PREFIX_NONE,
				     (u8 *)modeinfo,
				     sizeof(*modeinfo));
#endif /* CONFIG_CW1200_STA_DEBUG */
		if (priv->join_status == CW1200_JOIN_STATUS_STA ||
		    priv->join_status == CW1200_JOIN_STATUS_AP) {
			WARN_ON(wsm_set_p2p_ps_modeinfo(priv, modeinfo));
		}

		/* Temporary solution while firmware don't support NOA change
		 * notification yet */
		cw1200_notify_noa(priv, CW1200_NOA_NOTIFICATION_DELAY);
	}
#endif /* CONFIG_CW1200_USE_STE_EXTENSIONS */

	if (changed & IEEE80211_CONF_CHANGE_MONITOR) {
		/* TBD: It looks like it's transparent
		 * there's a monitor interface present -- use this
		 * to determine for example whether to calculate
		 * timestamps for packets or not, do not use instead
		 * of filter flags! */
	}

	if (changed & IEEE80211_CONF_CHANGE_IDLE) {
		struct wsm_operational_mode mode = {
			.power_mode = wsm_power_mode_quiescent,
			.disableMoreFlagUsage = true,
		};

		wsm_lock_tx(priv);
		/* Disable p2p-dev mode forced by TX request */
		if ((priv->join_status == CW1200_JOIN_STATUS_MONITOR) &&
				(conf->flags & IEEE80211_CONF_IDLE) &&
				!priv->listening) {
			cw1200_disable_listening(priv);
			priv->join_status = CW1200_JOIN_STATUS_PASSIVE;
		}
		WARN_ON(wsm_set_operational_mode(priv, &mode));
		wsm_unlock_tx(priv);
	}

	if (changed & IEEE80211_CONF_CHANGE_RETRY_LIMITS) {
		sta_printk(KERN_DEBUG "[STA] Retry limits: %d (long), " \
			"%d (short).\n",
			conf->long_frame_max_tx_count,
			conf->short_frame_max_tx_count);
		spin_lock_bh(&priv->tx_policy_cache.lock);
		priv->long_frame_max_tx_count = conf->long_frame_max_tx_count;
		priv->short_frame_max_tx_count =
			(conf->short_frame_max_tx_count < 0x0F) ?
			conf->short_frame_max_tx_count : 0x0F;
		priv->hw->max_rate_tries = priv->short_frame_max_tx_count;
		spin_unlock_bh(&priv->tx_policy_cache.lock);
		/* TBD: I think we don't need tx_policy_force_upload().
		 * Outdated policies will leave cache in a normal way. */
		/* WARN_ON(tx_policy_force_upload(priv)); */
	}
	mutex_unlock(&priv->conf_mutex);
	up(&priv->scan.lock);
	return ret;
}

void cw1200_update_filtering(struct cw1200_common *priv)
{
	int ret;
	bool bssid_filtering = !priv->rx_filter.bssid;
	bool is_p2p = priv->vif && priv->vif->p2p;
	bool is_sta = priv->vif && NL80211_IFTYPE_STATION == priv->vif->type;
	static struct wsm_beacon_filter_control bf_disabled = {
		.enabled = 0,
		.bcn_count = 1,
	};
	static struct wsm_beacon_filter_table bf_table_auto = {
		.numOfIEs = __cpu_to_le32(2),
		.entry[0].ieId = WLAN_EID_VENDOR_SPECIFIC,
		.entry[0].actionFlags = WSM_BEACON_FILTER_IE_HAS_CHANGED |
					WSM_BEACON_FILTER_IE_NO_LONGER_PRESENT |
					WSM_BEACON_FILTER_IE_HAS_APPEARED,
		.entry[0].oui[0] = 0x50,
		.entry[0].oui[1] = 0x6F,
		.entry[0].oui[2] = 0x9A,

		.entry[1].ieId = WLAN_EID_HT_INFORMATION,
		.entry[1].actionFlags = WSM_BEACON_FILTER_IE_HAS_CHANGED |
					WSM_BEACON_FILTER_IE_NO_LONGER_PRESENT |
					WSM_BEACON_FILTER_IE_HAS_APPEARED,
	};
	static struct wsm_beacon_filter_control bf_auto = {
		.enabled = WSM_BEACON_FILTER_ENABLE |
			WSM_BEACON_FILTER_AUTO_ERP,
		.bcn_count = 1,
	};
	bf_auto.bcn_count = priv->bf_control.bcn_count;

	if (priv->join_status == CW1200_JOIN_STATUS_PASSIVE)
		return;
	else if (priv->join_status == CW1200_JOIN_STATUS_MONITOR)
		bssid_filtering = false;

	/*
	* When acting as p2p client being connected to p2p GO, in order to
	* receive frames from a different p2p device, turn off bssid filter.
	*
	* WARNING: FW dependency!
	* This can only be used with FW WSM371 and its successors.
	* In that FW version even with bssid filter turned off,
	* device will block most of the unwanted frames.
	*/
	if (is_p2p)
		bssid_filtering = false;

	ret = wsm_set_rx_filter(priv, &priv->rx_filter);
	if (!ret) {
		if (is_p2p || !is_sta)
			ret = wsm_set_beacon_filter_table(priv, &priv->bf_table);
		else
			ret = wsm_set_beacon_filter_table(priv, &bf_table_auto);
	}
	if (!ret) {
		if (priv->disable_beacon_filter)
			ret = wsm_beacon_filter_control(priv,
					&bf_disabled);
		else {
			if (is_p2p || !is_sta)
				ret = wsm_beacon_filter_control(priv,
						&priv->bf_control);
			else
				ret = wsm_beacon_filter_control(priv,
						&bf_auto);
		}
	}
	if (!ret)
		ret = wsm_set_bssid_filtering(priv, bssid_filtering);
	if (!ret)
		ret = wsm_set_multicast_filter(priv, &priv->multicast_filter);
	if (ret)
		wiphy_err(priv->hw->wiphy,
				"%s: Update filtering failed: %d.\n",
				__func__, ret);
	return;
}

void cw1200_update_filtering_work(struct work_struct *work)
{
	struct cw1200_common *priv =
		container_of(work, struct cw1200_common,
		update_filtering_work);

	cw1200_update_filtering(priv);
}

void cw1200_set_beacon_wakeup_period_work(struct work_struct *work)
{
	unsigned dtim_interval;
	struct cw1200_common *priv =
		container_of(work, struct cw1200_common,
		set_beacon_wakeup_period_work);

	dtim_interval = priv->beacon_int * priv->join_dtim_period >
			MAX_BEACON_SKIP_TIME_MS ? 1 : priv->join_dtim_period;

	WARN_ON(wsm_set_beacon_wakeup_period(priv,
		dtim_interval, 0));
}

u64 cw1200_prepare_multicast(struct ieee80211_hw *hw,
			     struct netdev_hw_addr_list *mc_list)
{
	static u8 broadcast_ipv6[ETH_ALEN] = {
		0x33, 0x33, 0x00, 0x00, 0x00, 0x01
	};
	static u8 broadcast_ipv4[ETH_ALEN] = {
		0x01, 0x00, 0x5e, 0x00, 0x00, 0x01
	};
	struct cw1200_common *priv = hw->priv;
	struct netdev_hw_addr *ha;
	int count = 0;

	/* Disable multicast filtering */
	priv->has_multicast_subscription = false;
	memset(&priv->multicast_filter, 0x00, sizeof(priv->multicast_filter));

	if (netdev_hw_addr_list_count(mc_list) > WSM_MAX_GRP_ADDRTABLE_ENTRIES)
		return 0;

	/* Enable if requested */
	netdev_hw_addr_list_for_each(ha, mc_list) {
		sta_printk(KERN_DEBUG "[STA] multicast: %pM\n", ha->addr);
		memcpy(&priv->multicast_filter.macAddress[count],
		       ha->addr, ETH_ALEN);
		if (memcmp(ha->addr, broadcast_ipv4, ETH_ALEN) &&
				memcmp(ha->addr, broadcast_ipv6, ETH_ALEN))
			priv->has_multicast_subscription = true;
		count++;
	}

	if (count) {
		priv->multicast_filter.enable = __cpu_to_le32(1);
		priv->multicast_filter.numOfAddresses = __cpu_to_le32(count);
	}

	return netdev_hw_addr_list_count(mc_list);
}

void cw1200_configure_filter(struct ieee80211_hw *dev,
			     unsigned int changed_flags,
			     unsigned int *total_flags,
			     u64 multicast)
{
	struct cw1200_common *priv = dev->priv;
	bool listening = !!(*total_flags &
			(FIF_PROMISC_IN_BSS |
			 FIF_OTHER_BSS |
			 FIF_BCN_PRBRESP_PROMISC |
			 FIF_PROBE_REQ));

	*total_flags &= FIF_PROMISC_IN_BSS |
			FIF_OTHER_BSS |
			FIF_FCSFAIL |
			FIF_BCN_PRBRESP_PROMISC |
			FIF_PROBE_REQ;

	down(&priv->scan.lock);
	mutex_lock(&priv->conf_mutex);

	priv->rx_filter.promiscuous = (*total_flags & FIF_PROMISC_IN_BSS)
			? 1 : 0;
	priv->rx_filter.bssid = (*total_flags & (FIF_OTHER_BSS |
			FIF_PROBE_REQ)) ? 1 : 0;
	priv->rx_filter.fcs = (*total_flags & FIF_FCSFAIL) ? 1 : 0;
	priv->bf_control.bcn_count = (*total_flags &
			(FIF_BCN_PRBRESP_PROMISC |
			 FIF_PROMISC_IN_BSS |
			 FIF_PROBE_REQ)) ? 1 : 0;
	if (priv->listening ^ listening) {
		priv->listening = listening;
		wsm_lock_tx(priv);
		cw1200_update_listening(priv, listening);
		wsm_unlock_tx(priv);
	}
	cw1200_update_filtering(priv);
	mutex_unlock(&priv->conf_mutex);
	up(&priv->scan.lock);
}

int cw1200_conf_tx(struct ieee80211_hw *dev, u16 queue,
		   const struct ieee80211_tx_queue_params *params)
{
	struct cw1200_common *priv = dev->priv;
	int ret = 0;
	/* To prevent re-applying PM request OID again and again*/
	bool old_uapsdFlags;

	mutex_lock(&priv->conf_mutex);

	if (queue < dev->queues) {
		old_uapsdFlags = priv->uapsd_info.uapsdFlags;

		WSM_TX_QUEUE_SET(&priv->tx_queue_params, queue, 0, 0, 0);
		ret = wsm_set_tx_queue_params(priv,
				&priv->tx_queue_params.params[queue], queue);
		if (ret) {
			ret = -EINVAL;
			goto out;
		}

		WSM_EDCA_SET(&priv->edca, queue, params->aifs,
			params->cw_min, params->cw_max, params->txop, 0xc8,
			params->uapsd);
		ret = wsm_set_edca_params(priv, &priv->edca);
		if (ret) {
			ret = -EINVAL;
			goto out;
		}

		if (priv->mode == NL80211_IFTYPE_STATION) {
			ret = cw1200_set_uapsd_param(priv, &priv->edca);
			if (!ret &&
				(priv->join_status == CW1200_JOIN_STATUS_STA) &&
				(old_uapsdFlags != priv->uapsd_info.uapsdFlags))
				cw1200_set_pm(priv, &priv->powersave_mode);
		}
	} else
		ret = -EINVAL;

out:
	mutex_unlock(&priv->conf_mutex);
	return ret;
}

int cw1200_get_stats(struct ieee80211_hw *dev,
		     struct ieee80211_low_level_stats *stats)
{
	struct cw1200_common *priv = dev->priv;

	memcpy(stats, &priv->stats, sizeof(*stats));
	return 0;
}

/*
int cw1200_get_tx_stats(struct ieee80211_hw *dev,
			struct ieee80211_tx_queue_stats *stats)
{
	int i;
	struct cw1200_common *priv = dev->priv;

	for (i = 0; i < dev->queues; ++i)
		cw1200_queue_get_stats(&priv->tx_queue[i], &stats[i]);

	return 0;
}
*/

int cw1200_set_pm(struct cw1200_common *priv, const struct wsm_set_pm *arg)
{
	struct wsm_set_pm pm = *arg;

	if (priv->uapsd_info.uapsdFlags != 0)
		pm.pmMode &= ~WSM_PSM_FAST_PS_FLAG;

	if (memcmp(&pm, &priv->firmware_ps_mode,
			sizeof(struct wsm_set_pm))) {
		priv->firmware_ps_mode = pm;
		return wsm_set_pm(priv, &pm);
	} else {
		return 0;
	}
}

int cw1200_set_key(struct ieee80211_hw *dev, enum set_key_cmd cmd,
		   struct ieee80211_vif *vif, struct ieee80211_sta *sta,
		   struct ieee80211_key_conf *key)
{
	int ret = -EOPNOTSUPP;
	struct cw1200_common *priv = dev->priv;

	mutex_lock(&priv->conf_mutex);

	if (cmd == SET_KEY) {
		u8 *peer_addr = NULL;
		int pairwise = (key->flags & IEEE80211_KEY_FLAG_PAIRWISE) ?
			1 : 0;
		int idx = cw1200_alloc_key(priv);
		struct wsm_add_key *wsm_key = &priv->keys[idx];

		if (idx < 0) {
			ret = -EINVAL;
			goto finally;
		}

		BUG_ON(pairwise && !sta);
		if (sta)
			peer_addr = sta->addr;

		switch (key->cipher) {
		case WLAN_CIPHER_SUITE_WEP40:
		case WLAN_CIPHER_SUITE_WEP104:
			if (key->keylen > 16) {
				cw1200_free_key(priv, idx);
				ret = -EINVAL;
				goto finally;
			}

			if (pairwise) {
				wsm_key->type = WSM_KEY_TYPE_WEP_PAIRWISE;
				memcpy(wsm_key->wepPairwiseKey.peerAddress,
					 peer_addr, ETH_ALEN);
				memcpy(wsm_key->wepPairwiseKey.keyData,
					&key->key[0], key->keylen);
				wsm_key->wepPairwiseKey.keyLength = key->keylen;
			} else {
				wsm_key->type = WSM_KEY_TYPE_WEP_DEFAULT;
				memcpy(wsm_key->wepGroupKey.keyData,
					&key->key[0], key->keylen);
				wsm_key->wepGroupKey.keyLength = key->keylen;
				wsm_key->wepGroupKey.keyId = key->keyidx;
			}
			break;
		case WLAN_CIPHER_SUITE_TKIP:
			if (pairwise) {
				wsm_key->type = WSM_KEY_TYPE_TKIP_PAIRWISE;
				memcpy(wsm_key->tkipPairwiseKey.peerAddress,
					peer_addr, ETH_ALEN);
				memcpy(wsm_key->tkipPairwiseKey.tkipKeyData,
					&key->key[0],  16);
				memcpy(wsm_key->tkipPairwiseKey.txMicKey,
					&key->key[16],  8);
				memcpy(wsm_key->tkipPairwiseKey.rxMicKey,
					&key->key[24],  8);
			} else {
				size_t mic_offset =
					(priv->mode == NL80211_IFTYPE_AP) ?
					16 : 24;
				wsm_key->type = WSM_KEY_TYPE_TKIP_GROUP;
				memcpy(wsm_key->tkipGroupKey.tkipKeyData,
					&key->key[0],  16);
				memcpy(wsm_key->tkipGroupKey.rxMicKey,
					&key->key[mic_offset],  8);

				/* TODO: Where can I find TKIP SEQ? */
				memset(wsm_key->tkipGroupKey.rxSeqCounter,
					0,		8);
				wsm_key->tkipGroupKey.keyId = key->keyidx;

				print_hex_dump_bytes("TKIP: ", DUMP_PREFIX_NONE,
					key->key, key->keylen);
			}
			break;
		case WLAN_CIPHER_SUITE_CCMP:
			if (pairwise) {
				wsm_key->type = WSM_KEY_TYPE_AES_PAIRWISE;
				memcpy(wsm_key->aesPairwiseKey.peerAddress,
					peer_addr, ETH_ALEN);
				memcpy(wsm_key->aesPairwiseKey.aesKeyData,
					&key->key[0],  16);
			} else {
				wsm_key->type = WSM_KEY_TYPE_AES_GROUP;
				memcpy(wsm_key->aesGroupKey.aesKeyData,
					&key->key[0],  16);
				/* TODO: Where can I find AES SEQ? */
				memset(wsm_key->aesGroupKey.rxSeqCounter,
					0,              8);
				wsm_key->aesGroupKey.keyId = key->keyidx;
			}
			break;
#ifdef CONFIG_CW1200_WAPI_SUPPORT
		case WLAN_CIPHER_SUITE_SMS4:
			if (pairwise) {
				wsm_key->type = WSM_KEY_TYPE_WAPI_PAIRWISE;
				memcpy(wsm_key->wapiPairwiseKey.peerAddress,
					peer_addr, ETH_ALEN);
				memcpy(wsm_key->wapiPairwiseKey.wapiKeyData,
					&key->key[0],  16);
				memcpy(wsm_key->wapiPairwiseKey.micKeyData,
					&key->key[16], 16);
				wsm_key->wapiPairwiseKey.keyId = key->keyidx;
			} else {
				wsm_key->type = WSM_KEY_TYPE_WAPI_GROUP;
				memcpy(wsm_key->wapiGroupKey.wapiKeyData,
					&key->key[0],  16);
				memcpy(wsm_key->wapiGroupKey.micKeyData,
					&key->key[16], 16);
				wsm_key->wapiGroupKey.keyId = key->keyidx;
			}
			break;
#endif /* CONFIG_CW1200_WAPI_SUPPORT */
		default:
			WARN_ON(1);
			cw1200_free_key(priv, idx);
			ret = -EOPNOTSUPP;
			goto finally;
		}
		ret = WARN_ON(wsm_add_key(priv, wsm_key));
		if (!ret)
			key->hw_key_idx = idx;
		else
			cw1200_free_key(priv, idx);
	} else if (cmd == DISABLE_KEY) {
		struct wsm_remove_key wsm_key = {
			.entryIndex = key->hw_key_idx,
		};

		if (wsm_key.entryIndex > WSM_KEY_MAX_INDEX) {
			ret = -EINVAL;
			goto finally;
		}

		cw1200_free_key(priv, wsm_key.entryIndex);
		ret = wsm_remove_key(priv, &wsm_key);
	} else {
		BUG_ON("Unsupported command");
	}

finally:
	mutex_unlock(&priv->conf_mutex);
	return ret;
}

void cw1200_wep_key_work(struct work_struct *work)
{
	struct cw1200_common *priv =
		container_of(work, struct cw1200_common, wep_key_work);
	u8 queueId = cw1200_queue_get_queue_id(priv->pending_frame_id);
	struct cw1200_queue *queue = &priv->tx_queue[queueId];
	__le32 wep_default_key_id = __cpu_to_le32(
		priv->wep_default_key_id);

	BUG_ON(queueId >= 4);

	sta_printk(KERN_DEBUG "[STA] Setting default WEP key: %d\n",
		priv->wep_default_key_id);
	wsm_flush_tx(priv);
	WARN_ON(wsm_write_mib(priv, WSM_MIB_ID_DOT11_WEP_DEFAULT_KEY_ID,
		&wep_default_key_id, sizeof(wep_default_key_id)));
	cw1200_queue_requeue(queue, priv->pending_frame_id);
	wsm_unlock_tx(priv);
}

int cw1200_set_rts_threshold(struct ieee80211_hw *hw, u32 value)
{
	int ret = 0;
	__le32 val32;
	struct cw1200_common *priv = hw->priv;

	if (value != (u32) -1)
		val32 = __cpu_to_le32(value);
	else
		val32 = 0; /* disabled */

	if (priv->mode == NL80211_IFTYPE_UNSPECIFIED) {
		/* device is down, can _not_ set threshold */
		ret = -ENODEV;
		goto out;
	}

	if (priv->rts_threshold == value)
		goto out;

	sta_printk(KERN_DEBUG "[STA] Setting RTS threshold: %d\n",
		priv->rts_threshold);

	/* mutex_lock(&priv->conf_mutex); */
	ret = WARN_ON(wsm_write_mib(priv, WSM_MIB_ID_DOT11_RTS_THRESHOLD,
		&val32, sizeof(val32)));
	if (!ret)
		priv->rts_threshold = value;
	/* mutex_unlock(&priv->conf_mutex); */

out:
	return ret;
}

int __cw1200_flush(struct cw1200_common *priv, bool drop)
{
	int i, ret;

	for (;;) {
		/* TODO: correct flush handling is required when dev_stop.
		 * Temporary workaround: 2s
		 */
		if (drop || priv->bh_error) {
			for (i = 0; i < 4; ++i)
				cw1200_queue_clear(&priv->tx_queue[i]);
		} else {
			ret = wait_event_timeout(
				priv->tx_queue_stats.wait_link_id_empty,
				cw1200_queue_stats_is_empty(
					&priv->tx_queue_stats, -1),
				2 * HZ);
		}

		if (!drop && unlikely(ret <= 0)) {
			ret = -ETIMEDOUT;
			break;
		} else {
			ret = 0;
		}

		wsm_lock_tx(priv);
		if (unlikely(!cw1200_queue_stats_is_empty(
				&priv->tx_queue_stats, -1))) {
			/* Highly unlekely: WSM requeued frames. */
			wsm_unlock_tx(priv);
			continue;
		}
		break;
	}
	return ret;
}

void cw1200_flush(struct ieee80211_hw *hw, bool drop)
{
	struct cw1200_common *priv = hw->priv;

	switch (priv->mode) {
	case NL80211_IFTYPE_MONITOR:
		drop = true;
		break;
	case NL80211_IFTYPE_AP:
		if (!priv->enable_beacon)
			drop = true;
		break;
	}

	if (!WARN_ON(__cw1200_flush(priv, drop)))
		wsm_unlock_tx(priv);

	return;
}

/* ******************************************************************** */
/* WSM callbacks							*/

void cw1200_channel_switch_cb(struct cw1200_common *priv)
{
	wsm_unlock_tx(priv);
}

void cw1200_free_event_queue(struct cw1200_common *priv)
{
	LIST_HEAD(list);

	spin_lock(&priv->event_queue_lock);
	list_splice_init(&priv->event_queue, &list);
	spin_unlock(&priv->event_queue_lock);

	__cw1200_free_event_queue(&list);
}

void cw1200_event_handler(struct work_struct *work)
{
	struct cw1200_common *priv =
		container_of(work, struct cw1200_common, event_handler);
	struct cw1200_wsm_event *event;
	LIST_HEAD(list);

	spin_lock(&priv->event_queue_lock);
	list_splice_init(&priv->event_queue, &list);
	spin_unlock(&priv->event_queue_lock);

	list_for_each_entry(event, &list, link) {
		switch (event->evt.eventId) {
		case WSM_EVENT_ERROR:
			/* I even don't know what is it about.. */
			STUB();
			break;
		case WSM_EVENT_BSS_LOST:
		{
			spin_lock(&priv->bss_loss_lock);
			if (priv->bss_loss_status > CW1200_BSS_LOSS_NONE) {
				spin_unlock(&priv->bss_loss_lock);
				break;
			}
			priv->bss_loss_status = CW1200_BSS_LOSS_CHECKING;
			spin_unlock(&priv->bss_loss_lock);

			sta_printk(KERN_DEBUG "[CQM] BSS lost.\n");
			cancel_delayed_work_sync(&priv->bss_loss_work);
			cancel_delayed_work_sync(&priv->connection_loss_work);
			if (!down_trylock(&priv->scan.lock)) {
				up(&priv->scan.lock);
				priv->delayed_link_loss = 0;
				queue_delayed_work(priv->workqueue,
						&priv->bss_loss_work, 0);
			} else {
				/* Scan is in progress. Delay reporting. */
				/* Scan complete will trigger bss_loss_work */
				priv->delayed_link_loss = 1;
				/* Also we're starting watchdog. */
				queue_delayed_work(priv->workqueue,
						&priv->bss_loss_work, 10 * HZ);
			}
			break;
		}
		case WSM_EVENT_BSS_REGAINED:
		{
			sta_printk(KERN_DEBUG "[CQM] BSS regained.\n");
			priv->delayed_link_loss = 0;
			spin_lock(&priv->bss_loss_lock);
			priv->bss_loss_status = CW1200_BSS_LOSS_NONE;
			priv->bss_loss_checking = 0;
			spin_unlock(&priv->bss_loss_lock);
			cancel_delayed_work_sync(&priv->bss_loss_work);
			cancel_delayed_work_sync(&priv->connection_loss_work);
			break;
		}
		case WSM_EVENT_RADAR_DETECTED:
			STUB();
			break;
		case WSM_EVENT_RCPI_RSSI:
		{
			/* RSSI: signed Q8.0, RCPI: unsigned Q7.1
			 * RSSI = RCPI / 2 - 110 */
			int rcpiRssi = (int)(event->evt.eventData & 0xFF);
			int cqm_evt;
			if (priv->cqm_use_rssi)
				rcpiRssi = (s8)rcpiRssi;
			else
				rcpiRssi =  rcpiRssi / 2 - 110;

			cqm_evt = (rcpiRssi <= priv->cqm_rssi_thold) ?
				NL80211_CQM_RSSI_THRESHOLD_EVENT_LOW :
				NL80211_CQM_RSSI_THRESHOLD_EVENT_HIGH;
			sta_printk(KERN_DEBUG "[CQM] RSSI event: %d", rcpiRssi);
			ieee80211_cqm_rssi_notify(priv->vif, cqm_evt,
								GFP_KERNEL);
			break;
		}
		case WSM_EVENT_BT_INACTIVE:
			STUB();
			break;
		case WSM_EVENT_BT_ACTIVE:
			STUB();
			break;
		}
	}
	__cw1200_free_event_queue(&list);
}

void cw1200_bss_loss_work(struct work_struct *work)
{
	struct cw1200_common *priv =
		container_of(work, struct cw1200_common, bss_loss_work.work);
	int timeout; /* in beacons */
	struct sk_buff *skb;

	timeout = priv->cqm_link_loss_count -
		priv->cqm_beacon_loss_count;

	/* Skip the confimration procedure in P2P case */
	if (priv->vif->p2p)
		goto report;

	spin_lock(&priv->bss_loss_lock);
	if (priv->bss_loss_status == CW1200_BSS_LOSS_CHECKING) {
		spin_unlock(&priv->bss_loss_lock);
		skb = ieee80211_nullfunc_get(priv->hw, priv->vif);
		if (!(WARN_ON(!skb))) {
			cw1200_tx(priv->hw, skb);
			/* Start watchdog -- if nullfunc TX doesn't fail
			 * in 1 sec, forward event to upper layers */
			queue_delayed_work(priv->workqueue,
					&priv->bss_loss_work, 1 * HZ);
		}
		return;
	} else if (priv->bss_loss_status == CW1200_BSS_LOSS_CONFIRMING) {
		priv->bss_loss_status = CW1200_BSS_LOSS_NONE;
		priv->bss_loss_checking = 0;
		spin_unlock(&priv->bss_loss_lock);
		return;
	}
	spin_unlock(&priv->bss_loss_lock);

report:
	if (priv->cqm_beacon_loss_count) {
		sta_printk(KERN_DEBUG "[CQM] Beacon loss.\n");
		if (timeout <= 0)
			timeout = 0;
#if defined(CONFIG_CW1200_USE_STE_EXTENSIONS)
		ieee80211_cqm_beacon_miss_notify(priv->vif, GFP_KERNEL);
#endif /* CONFIG_CW1200_USE_STE_EXTENSIONS */
	} else {
		timeout = 0;
	}

	cancel_delayed_work_sync(&priv->connection_loss_work);
	queue_delayed_work(priv->workqueue,
		&priv->connection_loss_work,
		timeout * HZ / 10);

	spin_lock(&priv->bss_loss_lock);
	priv->bss_loss_status = CW1200_BSS_LOSS_NONE;
	priv->bss_loss_checking = 0;
	spin_unlock(&priv->bss_loss_lock);
}

void cw1200_connection_loss_work(struct work_struct *work)
{
	struct cw1200_common *priv =
		container_of(work, struct cw1200_common,
				connection_loss_work.work);
	sta_printk(KERN_DEBUG "[CQM] Reporting connection loss.\n");
	ieee80211_connection_loss(priv->vif);
}

void cw1200_tx_failure_work(struct work_struct *work)
{
	struct cw1200_common *priv =
		container_of(work, struct cw1200_common, tx_failure_work);
	sta_printk(KERN_DEBUG "[CQM] Reporting TX failure.\n");
#if defined(CONFIG_CW1200_USE_STE_EXTENSIONS)
	ieee80211_cqm_tx_fail_notify(priv->vif, GFP_KERNEL);
#else /* CONFIG_CW1200_USE_STE_EXTENSIONS */
	(void)priv;
#endif /* CONFIG_CW1200_USE_STE_EXTENSIONS */
}

/* ******************************************************************** */
/* Internal API								*/



/*
* This function is called to Parse the SDD file
* to extract listen_interval and PTA related information.
*
* sdd is a TLV: u8 id, u8 len, u8 data[]
*
*/
static int cw1200_parse_SDD_file(struct cw1200_common *priv)
{
	#define SDD_PTA_CFG_ELT_ID 0xEB
	const u8 *p = priv->sdd->data;
	int ret = 0;

	priv->is_BT_Present = false;

	while (p + 2 <= priv->sdd->data + priv->sdd->size) {
		if (p + p[1] + 2 > priv->sdd->data + priv->sdd->size) {
			printk(KERN_WARNING "Malformed sdd structure\n");
			return -1;
		}

		switch (p[0]) {
		case SDD_PTA_CFG_ELT_ID:
		{
			u16 v;
			if (p[1] < 4) {
				printk(KERN_WARNING
				       "SDD_PTA_CFG_ELT_ID malformed\n");
				ret = -1;
				break;
			}

			v = le16_to_cpu(*((u16*)(p + 4)));
			priv->conf_listen_interval = (v >> 7) & 0x1F;
			priv->is_BT_Present = true;
			sta_printk(KERN_DEBUG "PTA element found (%04x).\n", v);
			sta_printk(KERN_DEBUG "Listen Interval %d\n",
						priv->conf_listen_interval);
		}
		break;

		default:
		break;
		}

		p += p[1] + 2;
	}

	if (priv->is_BT_Present == false) {
		sta_printk(KERN_DEBUG "PTA element NOT found.\n");
		priv->conf_listen_interval = 0;
	}
	return ret;

	#undef SDD_PTA_CFG_ELT_ID
}


int cw1200_setup_mac(struct cw1200_common *priv)
{
	int ret = 0;
	struct wsm_configuration cfg = {
		.dot11StationId = &priv->mac_addr[0],
	};

	/* NOTE: There is a bug in FW: it reports signal
	* as RSSI if RSSI subscription is enabled.
	* It's not enough to set WSM_RCPI_RSSI_USE_RSSI. */
	/* NOTE2: RSSI based reports have been switched to RCPI, since
	* FW has a bug and RSSI reported values are not stable,
	* what can leads to signal level oscilations in user-end applications */
	struct wsm_rcpi_rssi_threshold threshold = {
		.rssiRcpiMode = WSM_RCPI_RSSI_THRESHOLD_ENABLE |
		WSM_RCPI_RSSI_DONT_USE_UPPER |
		WSM_RCPI_RSSI_DONT_USE_LOWER,
		.rollingAverageCount = 16,
	};

	/* Remember the decission here to make sure, we will handle
	 * the RCPI/RSSI value correctly on WSM_EVENT_RCPI_RSS */
	if (threshold.rssiRcpiMode & WSM_RCPI_RSSI_USE_RSSI)
		priv->cqm_use_rssi = true;

	if (!priv->sdd) {
		const char *sdd_path = NULL;

		switch (priv->hw_revision) {
		case CW1200_HW_REV_CUT10:
			sdd_path = SDD_FILE_10;
			break;
		case CW1200_HW_REV_CUT11:
			sdd_path = SDD_FILE_11;
			break;
		case CW1200_HW_REV_CUT20:
			sdd_path = SDD_FILE_20;
			break;
		case CW1200_HW_REV_CUT22:
			sdd_path = SDD_FILE_22;
			break;
		default:
			BUG_ON(1);
		}

		ret = request_firmware(&priv->sdd,
				       sdd_path, priv->pdev);
		if (unlikely(ret)) {
			cw1200_dbg(CW1200_DBG_ERROR,
				"%s: can't load sdd file %s.\n",
				__func__, sdd_path);
			return ret;
		}
	}
	if (!priv->setup_mac_done) {
		cfg.dpdData = priv->sdd->data;
		cfg.dpdData_size = priv->sdd->size;
		ret = WARN_ON(wsm_configuration(priv, &cfg));
		/* Parse SDD file for PTA element */
		cw1200_parse_SDD_file(priv);

		if (ret)
			return ret;

		priv->setup_mac_done = true;
	}

	/* Configure RSSI/SCPI reporting as RSSI. */
	WARN_ON(wsm_set_rcpi_rssi_threshold(priv, &threshold));

	/* TODO: */
	switch (priv->mode) {
	case NL80211_IFTYPE_STATION:
	case NL80211_IFTYPE_MONITOR:
	case NL80211_IFTYPE_AP:
		break;
	case NL80211_IFTYPE_ADHOC:
	case NL80211_IFTYPE_MESH_POINT:
		/* TODO: Not verified yet. */
		STUB();
		break;
	}

	return 0;
}

void cw1200_offchannel_work(struct work_struct *work)
{
	struct cw1200_common *priv =
		container_of(work, struct cw1200_common, offchannel_work);
	u8 queueId = cw1200_queue_get_queue_id(priv->pending_frame_id);
	struct cw1200_queue *queue = &priv->tx_queue[queueId];

	BUG_ON(queueId >= 4);
	BUG_ON(!priv->channel);

	mutex_lock(&priv->conf_mutex);
	if (likely(!priv->join_status)) {
		wsm_flush_tx(priv);
		cw1200_update_listening(priv, true);
		cw1200_update_filtering(priv);
	}
	if (unlikely(!priv->join_status))
		cw1200_queue_remove(queue, priv->pending_frame_id);
	else
		cw1200_queue_requeue(queue, priv->pending_frame_id);
	mutex_unlock(&priv->conf_mutex);
	wsm_unlock_tx(priv);
}

static void cw1200_join_complete(struct cw1200_common *priv)
{
	sta_printk(KERN_DEBUG "[STA] Join complete.\n");

	mutex_lock(&priv->conf_mutex);
	priv->join_pending = false;
	if (priv->join_complete_status) {
		priv->join_status = CW1200_JOIN_STATUS_PASSIVE;
		memset(&priv->join_bssid[0], 0, sizeof(priv->join_bssid));
		wsm_lock_tx(priv);
		cw1200_update_listening(priv, priv->listening);
		wsm_unlock_tx(priv);
	} else {
		priv->join_status = CW1200_JOIN_STATUS_PRE_STA;
		/* Assoc timeout, not to leave device in full power. */
		queue_delayed_work(priv->workqueue,
			&priv->join_timeout, CW1200_AUTH_TIMEOUT);
		/* Due to beacon filtering it is possible that the
		* AP's beacon is not known for the mac80211 stack.
		* Disable filtering temporary to make sure the stack
		* receives at least one */
		priv->disable_beacon_filter = true;
		cw1200_update_filtering(priv);
	}
	mutex_unlock(&priv->conf_mutex);
}

void cw1200_join_complete_work(struct work_struct *work)
{
	struct cw1200_common *priv =
		container_of(work, struct cw1200_common, join_complete_work);
	cw1200_join_complete(priv);
}

void cw1200_join_complete_cb(struct cw1200_common *priv,
				struct wsm_join_complete *arg)
{
	sta_printk(KERN_DEBUG "[STA] cw1200_join_complete_cb called, "
			"status=%d.\n", arg->status);

	if (cancel_delayed_work(&priv->join_timeout)) {
		priv->join_complete_status = arg->status;
		queue_work(priv->workqueue, &priv->join_complete_work);
	}
}

void cw1200_join_work(struct work_struct *work)
{
	struct cw1200_common *priv =
		container_of(work, struct cw1200_common, join_work);
	u8 queueId = cw1200_queue_get_queue_id(priv->pending_frame_id);
	struct cw1200_queue *queue = &priv->tx_queue[queueId];
	const struct cw1200_txpriv *txpriv = NULL;
	struct sk_buff *skb = NULL;
	const struct wsm_tx *wsm;
	const struct ieee80211_hdr *frame;
	const u8 *bssid;
	struct cfg80211_bss *bss;
	const u8 *ssidie;
	const u8 *dtimie;
	const struct ieee80211_tim_ie *tim = NULL;
	struct wsm_protected_mgmt_policy mgmt_policy;

	BUG_ON(queueId >= 4);
	if (cw1200_queue_get_skb(queue, priv->pending_frame_id,
			&skb, &txpriv)) {
		wsm_unlock_tx(priv);
		return;
	}

	if (WARN_ON(delayed_work_pending(&priv->join_timeout))) {
		printk(KERN_ERR "[STA] Skipping join request - "
				"previous request yet to be completed\n");
		cw1200_queue_remove(queue, priv->pending_frame_id);
		wsm_unlock_tx(priv);
		priv->join_pending = false;
		return;
	}

	wsm = (struct wsm_tx *)&skb->data[0];
	frame = (struct ieee80211_hdr *)&skb->data[txpriv->offset];
	bssid = &frame->addr1[0]; /* AP SSID in a 802.11 frame */

	BUG_ON(!wsm);
	BUG_ON(!priv->channel);

	if (unlikely(priv->join_status)) {
		wsm_lock_tx(priv);
		cw1200_unjoin_work(&priv->unjoin_work);
	}

	bss = cfg80211_get_bss(priv->hw->wiphy, priv->channel,
			bssid, NULL, 0, 0, 0);
	if (!bss) {
		cw1200_queue_remove(queue, priv->pending_frame_id);
		wsm_unlock_tx(priv);
		priv->join_pending = false;
		return;
	}
	ssidie = cfg80211_find_ie(WLAN_EID_SSID,
		bss->information_elements,
		bss->len_information_elements);
	dtimie = cfg80211_find_ie(WLAN_EID_TIM,
		bss->information_elements,
		bss->len_information_elements);
	if (dtimie)
		tim = (struct ieee80211_tim_ie *)&dtimie[2];

	mutex_lock(&priv->conf_mutex);
	{
		struct wsm_join join = {
			.mode = (bss->capability & WLAN_CAPABILITY_IBSS) ?
				WSM_JOIN_MODE_IBSS : WSM_JOIN_MODE_BSS,
			.preambleType = WSM_JOIN_PREAMBLE_SHORT,
			.probeForJoin = 1,
			/* dtimPeriod will be updated after association */
			.dtimPeriod = 1,
			.beaconInterval = bss->beacon_interval,
			/* basicRateSet will be updated after association */
			.basicRateSet = 7,
		};

		/* Under the conf lock: check scan status and
		 * bail out if it is in progress. */
		if (unlikely(atomic_read(&priv->scan.in_progress))) {
			cw1200_queue_remove(queue, priv->pending_frame_id);
			mutex_unlock(&priv->conf_mutex);
			wsm_unlock_tx(priv);
			return;
		}

		/* BT Coex related changes */
		if (priv->is_BT_Present) {
			if (((priv->conf_listen_interval * 100) %
					bss->beacon_interval) == 0)
				priv->listen_interval =
					((priv->conf_listen_interval * 100) /
					bss->beacon_interval);
			else
				priv->listen_interval =
					((priv->conf_listen_interval * 100) /
					bss->beacon_interval + 1);
		}

		if (tim && tim->dtim_period > 1) {
			join.dtimPeriod = tim->dtim_period;
			priv->join_dtim_period = tim->dtim_period;
		}
		priv->beacon_int = bss->beacon_interval;
		sta_printk(KERN_DEBUG "[STA] Join DTIM: %d, interval: %d\n",
				join.dtimPeriod, priv->beacon_int);

		join.channelNumber = priv->channel->hw_value;
		join.band = (priv->channel->band == IEEE80211_BAND_5GHZ) ?
			WSM_PHY_BAND_5G : WSM_PHY_BAND_2_4G;

		memcpy(&join.bssid[0], bssid, sizeof(join.bssid));
		memcpy(&priv->join_bssid[0], bssid, sizeof(priv->join_bssid));

		if (ssidie) {
			join.ssidLength = ssidie[1];
			if (WARN_ON(join.ssidLength > sizeof(join.ssid)))
				join.ssidLength = sizeof(join.ssid);
			memcpy(&join.ssid[0], &ssidie[2], join.ssidLength);
		}

		if (priv->vif->p2p) {
			join.flags |= WSM_JOIN_FLAGS_P2P_GO;
			join.basicRateSet =
				cw1200_rate_mask_to_wsm(priv, 0xFF0);
		}

		/* Enable asynchronous join calls */
		join.flags |= WSM_JOIN_FLAGS_FORCE;
		join.flags |= WSM_JOIN_FLAGS_FORCE_WITH_COMPLETE_IND;

		wsm_flush_tx(priv);

		queue_delayed_work(priv->workqueue,
			&priv->join_timeout, CW1200_JOIN_TIMEOUT);
		/* Stay Awake for Join and Auth Timeouts and a bit more */
		cw1200_pm_stay_awake(&priv->pm_state,
			CW1200_JOIN_TIMEOUT + CW1200_AUTH_TIMEOUT);

		cw1200_update_listening(priv, false);
		/* BlockACK policy will be updated when assoc is done */
		WARN_ON(wsm_set_block_ack_policy(priv,
			0, priv->ba_tid_mask));

		mgmt_policy.protectedMgmtEnable = 0;
		mgmt_policy.unprotectedMgmtFramesAllowed = 1;
		mgmt_policy.encryptionForAuthFrame = 1;
		wsm_set_protected_mgmt_policy(priv, &mgmt_policy);

		if (wsm_join(priv, &join)) {
			sta_printk(KERN_ERR "[STA] cw1200_join_work: "
				"wsm_join request failed!\n");
			memset(&priv->join_bssid[0],
				0, sizeof(priv->join_bssid));
			cw1200_queue_remove(queue, priv->pending_frame_id);
			cancel_delayed_work(&priv->join_timeout);
			cw1200_update_listening(priv, priv->listening);
		} else {
			sta_printk(KERN_DEBUG "[STA] cw1200_join_work: "
					"wsm_join request send succesfully\n");
			priv->join_status = CW1200_JOIN_STATUS_JOINING;
			WARN_ON(cw1200_upload_keys(priv));
			cw1200_queue_requeue(queue, priv->pending_frame_id);
		}
	}
	mutex_unlock(&priv->conf_mutex);
	cfg80211_put_bss(bss);
	wsm_unlock_tx(priv);
}

void cw1200_join_timeout(struct work_struct *work)
{
	struct cw1200_common *priv =
		container_of(work, struct cw1200_common, join_timeout.work);
	sta_printk(KERN_DEBUG "[WSM] Issue unjoin command (TMO).\n");
	wsm_lock_tx(priv);
	ieee80211_connection_loss(priv->vif);
	cw1200_unjoin_work(&priv->unjoin_work);
}

void cw1200_unjoin_work(struct work_struct *work)
{
	struct cw1200_common *priv =
		container_of(work, struct cw1200_common, unjoin_work);

	struct wsm_reset reset = {
		.reset_statistics = true,
	};

	cancel_delayed_work(&priv->join_timeout);
	mutex_lock(&priv->conf_mutex);
	priv->join_pending = false;
	if (unlikely(atomic_read(&priv->scan.in_progress))) {
		if (priv->delayed_unjoin) {
			wiphy_dbg(priv->hw->wiphy,
				"%s: Delayed unjoin "
				"is already scheduled.\n",
				__func__);
			wsm_unlock_tx(priv);
		} else {
			priv->delayed_unjoin = true;
		}
		mutex_unlock(&priv->conf_mutex);
		return;
	}

	if (priv->join_status &&
			priv->join_status > CW1200_JOIN_STATUS_STA) {
		wiphy_err(priv->hw->wiphy,
				"%s: Unexpected: join status: %d\n",
				__func__, priv->join_status);
		BUG_ON(1);
	}
	if (priv->join_status) {
		cancel_work_sync(&priv->update_filtering_work);
		cancel_work_sync(&priv->set_beacon_wakeup_period_work);
		memset(&priv->join_bssid[0], 0, sizeof(priv->join_bssid));
		priv->join_status = CW1200_JOIN_STATUS_PASSIVE;

		/* Unjoin is a reset. */
		wsm_flush_tx(priv);
		WARN_ON(wsm_keep_alive_period(priv, 0));
		WARN_ON(wsm_reset(priv, &reset));
		WARN_ON(wsm_set_output_power(priv, priv->output_power * 10));
		priv->join_dtim_period = 0;
		WARN_ON(cw1200_setup_mac(priv));
		cw1200_free_event_queue(priv);
		cancel_work_sync(&priv->event_handler);
		cancel_delayed_work_sync(&priv->connection_loss_work);
		cw1200_update_listening(priv, priv->listening);
		WARN_ON(wsm_set_block_ack_policy(priv,
			0, priv->ba_tid_mask));
		priv->block_ack_enabled = false;
		priv->disable_beacon_filter = false;
		cw1200_update_filtering(priv);
		memset(&priv->association_mode, 0,
			sizeof(priv->association_mode));
		memset(&priv->bss_params, 0, sizeof(priv->bss_params));
		memset(&priv->firmware_ps_mode, 0,
			sizeof(priv->firmware_ps_mode));
		sta_printk(KERN_DEBUG "[STA] Unjoin.\n");
	}
	mutex_unlock(&priv->conf_mutex);
	wsm_unlock_tx(priv);
}

int cw1200_enable_listening(struct cw1200_common *priv)
{
	struct wsm_start start = {
		.mode = WSM_START_MODE_P2P_DEV,
		.band = WSM_PHY_BAND_2_4G,
		.beaconInterval = 100,
		.channelNumber = 1,
		.DTIMPeriod = 1,
		.probeDelay = 0,
		.basicRateSet = 0x0F,
	};

	if (priv->channel) {
		start.band = priv->channel->band == IEEE80211_BAND_5GHZ ?
			     WSM_PHY_BAND_5G : WSM_PHY_BAND_2_4G;
		start.channelNumber = priv->channel->hw_value;
	}

	return wsm_start(priv, &start);
}

int cw1200_disable_listening(struct cw1200_common *priv)
{
	int ret;
	struct wsm_reset reset = {
		.reset_statistics = true,
	};
	ret = wsm_reset(priv, &reset);
	return ret;
}

void cw1200_update_listening(struct cw1200_common *priv, bool enabled)
{
	if (enabled) {
		switch (priv->join_status) {
		case CW1200_JOIN_STATUS_PASSIVE:
			if (!WARN_ON(cw1200_enable_listening(priv)))
				priv->join_status = CW1200_JOIN_STATUS_MONITOR;
			WARN_ON(wsm_set_probe_responder(priv, true));
			break;
		default:
			break;
		}
	} else {
		switch (priv->join_status) {
		case CW1200_JOIN_STATUS_MONITOR:
			if (!WARN_ON(cw1200_disable_listening(priv)))
				priv->join_status = CW1200_JOIN_STATUS_PASSIVE;
			WARN_ON(wsm_set_probe_responder(priv, false));
		default:
			break;
		}
	}
}

int cw1200_set_uapsd_param(struct cw1200_common *priv,
				const struct wsm_edca_params *arg)
{
	int ret;
	u16 uapsdFlags = 0;

	/* Here's the mapping AC [queue, bit]
	VO [0,3], VI [1, 2], BE [2, 1], BK [3, 0]*/

	if (arg->params[0].uapsdEnable)
		uapsdFlags |= 1 << 3;

	if (arg->params[1].uapsdEnable)
		uapsdFlags |= 1 << 2;

	if (arg->params[2].uapsdEnable)
		uapsdFlags |= 1 << 1;

	if (arg->params[3].uapsdEnable)
		uapsdFlags |= 1;

	/* Currently pseudo U-APSD operation is not supported, so setting
	* MinAutoTriggerInterval, MaxAutoTriggerInterval and
	* AutoTriggerStep to 0 */

	priv->uapsd_info.uapsdFlags = cpu_to_le16(uapsdFlags);
	priv->uapsd_info.minAutoTriggerInterval = 0;
	priv->uapsd_info.maxAutoTriggerInterval = 0;
	priv->uapsd_info.autoTriggerStep = 0;

	ret = wsm_set_uapsd_info(priv, &priv->uapsd_info);
	return ret;
}

const u8 *cw1200_get_ie(u8 *start, size_t len, u8 ie)
{
	u8 *end, *pos;

	pos = start;
	if (pos == NULL)
		return NULL;
	end = pos + len;

	while (pos + 1 < end) {
		if (pos + 2 + pos[1] > end)
			break;
		if (pos[0] == ie)
			return pos;
		pos += 2 + pos[1];
	}

	return NULL;
}
