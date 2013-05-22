/*
 * Datapath implementation for ST-Ericsson CW1200 mac80211 drivers
 *
 * Copyright (c) 2010, ST-Ericsson
 * Author: Dmitry Tarnyagin <dmitry.tarnyagin@stericsson.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <net/mac80211.h>
#include <linux/etherdevice.h>
#include <linux/skbuff.h>

#include "cw1200.h"
#include "wsm.h"
#include "bh.h"
#include "ap.h"
#include "debug.h"
#include "sta.h"
#include "sbus.h"

#if defined(CONFIG_CW1200_TX_POLICY_DEBUG)
#define tx_policy_printk pr_debug
#else
#define tx_policy_printk(...)
#endif

#define CW1200_INVALID_RATE_ID (0xFF)

static int cw1200_handle_action_rx(struct cw1200_common *priv,
				   struct sk_buff *skb);
static const struct ieee80211_rate *
cw1200_get_tx_rate(const struct cw1200_common *priv,
		   const struct ieee80211_tx_rate *rate);

/* ******************************************************************** */
/* TX queue lock / unlock						*/

static inline void cw1200_tx_queues_lock(struct cw1200_common *priv)
{
	int i;
	for (i = 0; i < 4; ++i)
		cw1200_queue_lock(&priv->tx_queue[i]);
}

static inline void cw1200_tx_queues_unlock(struct cw1200_common *priv)
{
	int i;
	for (i = 0; i < 4; ++i)
		cw1200_queue_unlock(&priv->tx_queue[i]);
}

/* ******************************************************************** */
/* TX policy cache implementation					*/

static void tx_policy_dump(struct tx_policy *policy)
{
	tx_policy_printk(KERN_DEBUG "[TX policy] "
		"%.1X%.1X%.1X%.1X%.1X%.1X%.1X%.1X"
		"%.1X%.1X%.1X%.1X%.1X%.1X%.1X%.1X"
		"%.1X%.1X%.1X%.1X%.1X%.1X%.1X%.1X: %d\n",
		policy->raw[0] & 0x0F,  policy->raw[0] >> 4,
		policy->raw[1] & 0x0F,  policy->raw[1] >> 4,
		policy->raw[2] & 0x0F,  policy->raw[2] >> 4,
		policy->raw[3] & 0x0F,  policy->raw[3] >> 4,
		policy->raw[4] & 0x0F,  policy->raw[4] >> 4,
		policy->raw[5] & 0x0F,  policy->raw[5] >> 4,
		policy->raw[6] & 0x0F,  policy->raw[6] >> 4,
		policy->raw[7] & 0x0F,  policy->raw[7] >> 4,
		policy->raw[8] & 0x0F,  policy->raw[8] >> 4,
		policy->raw[9] & 0x0F,  policy->raw[9] >> 4,
		policy->raw[10] & 0x0F,  policy->raw[10] >> 4,
		policy->raw[11] & 0x0F,  policy->raw[11] >> 4,
		policy->defined);
}

static void tx_policy_build(const struct cw1200_common *priv,
	/* [out] */ struct tx_policy *policy,
	struct ieee80211_tx_rate *rates, size_t count)
{
	int i, j;
	unsigned limit = priv->short_frame_max_tx_count;
	unsigned total = 0;
	BUG_ON(rates[0].idx < 0);
	memset(policy, 0, sizeof(*policy));

	/* minstrel is buggy a little bit, so distille
	 * incoming rates first. */

	/* Sort rates in descending order. */
	for (i = 1; i < count; ++i) {
		if (rates[i].idx < 0) {
			count = i;
			break;
		}
		if (rates[i].idx > rates[i - 1].idx) {
			struct ieee80211_tx_rate tmp = rates[i - 1];
			rates[i - 1] = rates[i];
			rates[i] = tmp;
		}
	}

	/* Eliminate duplicates. */
	total = rates[0].count;
	for (i = 0, j = 1; j < count; ++j) {
		if (rates[j].idx == rates[i].idx) {
			rates[i].count += rates[j].count;
		} else if (rates[j].idx > rates[i].idx) {
			break;
		} else {
			++i;
			if (i != j)
				rates[i] = rates[j];
		}
		total += rates[j].count;
	}
	count = i + 1;

	/* Re-fill policy trying to keep every requested rate and with
	 * respect to the global max tx retransmission count. */
	if (limit < count)
		limit = count;
	if (total > limit) {
		for (i = 0; i < count; ++i) {
			int left = count - i - 1;
			if (rates[i].count > limit - left)
				rates[i].count = limit - left;
			limit -= rates[i].count;
		}
	}

	/* HACK!!! Device has problems (at least) switching from
	 * 54Mbps CTS to 1Mbps. This switch takes enormous amount
	 * of time (100-200 ms), leading to valuable throughput drop.
	 * As a workaround, additional g-rates are injected to the
	 * policy.
	 */
	if (count == 2 && !(rates[0].flags & IEEE80211_TX_RC_MCS) &&
			rates[0].idx > 4 && rates[0].count > 2 &&
			rates[1].idx < 2) {
		/* ">> 1" is an equivalent of "/ 2", but faster */
		int mid_rate = (rates[0].idx + 4) >> 1;

		/* Decrease number of retries for the initial rate */
		rates[0].count -= 2;

		if (mid_rate != 4) {
			/* Keep fallback rate at 1Mbps. */
			rates[3] = rates[1];

			/* Inject 1 transmission on lowest g-rate */
			rates[2].idx = 4;
			rates[2].count = 1;
			rates[2].flags = rates[1].flags;

			/* Inject 1 transmission on mid-rate */
			rates[1].idx = mid_rate;
			rates[1].count = 1;

			/* Fallback to 1 Mbps is a really bad thing,
			 * so let's try to increase probability of
			 * successful transmission on the lowest g rate
			 * even more */
			if (rates[0].count >= 3) {
				--rates[0].count;
				++rates[2].count;
			}

			/* Adjust amount of rates defined */
			count += 2;
		} else {
			/* Keep fallback rate at 1Mbps. */
			rates[2] = rates[1];

			/* Inject 2 transmissions on lowest g-rate */
			rates[1].idx = 4;
			rates[1].count = 2;

			/* Adjust amount of rates defined */
			count += 1;
		}
	}

	policy->defined = cw1200_get_tx_rate(priv, &rates[0])->hw_value + 1;

	for (i = 0; i < count; ++i) {
		register unsigned rateid, off, shift, retries;

		rateid = cw1200_get_tx_rate(priv, &rates[i])->hw_value;
		off = rateid >> 3;		/* eq. rateid / 8 */
		shift = (rateid & 0x07) << 2;	/* eq. (rateid % 8) * 4 */

		retries = rates[i].count;
		if (unlikely(retries > 0x0F))
			rates[i].count = retries = 0x0F;
		policy->tbl[off] |= __cpu_to_le32(retries << shift);
		policy->retry_count += retries;
	}

	tx_policy_printk(KERN_DEBUG "[TX policy] Policy (%d): " \
		"%d:%d, %d:%d, %d:%d, %d:%d, %d:%d\n",
		count,
		rates[0].idx, rates[0].count,
		rates[1].idx, rates[1].count,
		rates[2].idx, rates[2].count,
		rates[3].idx, rates[3].count,
		rates[4].idx, rates[4].count);
}

static inline bool tx_policy_is_equal(const struct tx_policy *wanted,
					const struct tx_policy *cached)
{
	size_t count = wanted->defined >> 1;
	if (wanted->defined > cached->defined)
		return false;
	if (count) {
		if (memcmp(wanted->raw, cached->raw, count))
			return false;
	}
	if (wanted->defined & 1) {
		if ((wanted->raw[count] & 0x0F) != (cached->raw[count] & 0x0F))
			return false;
	}
	return true;
}

static int tx_policy_find(struct tx_policy_cache *cache,
				const struct tx_policy *wanted)
{
	/* O(n) complexity. Not so good, but there's only 8 entries in
	 * the cache.
	 * Also lru helps to reduce search time. */
	struct tx_policy_cache_entry *it;
	/* First search for policy in "used" list */
	list_for_each_entry(it, &cache->used, link) {
		if (tx_policy_is_equal(wanted, &it->policy))
			return it - cache->cache;
	}
	/* Then - in "free list" */
	list_for_each_entry(it, &cache->free, link) {
		if (tx_policy_is_equal(wanted, &it->policy))
			return it - cache->cache;
	}
	return -1;
}

static inline void tx_policy_use(struct tx_policy_cache *cache,
				 struct tx_policy_cache_entry *entry)
{
	++entry->policy.usage_count;
	list_move(&entry->link, &cache->used);
}

static inline int tx_policy_release(struct tx_policy_cache *cache,
				    struct tx_policy_cache_entry *entry)
{
	int ret = --entry->policy.usage_count;
	if (!ret)
		list_move(&entry->link, &cache->free);
	return ret;
}

/* ******************************************************************** */
/* External TX policy cache API						*/

void tx_policy_init(struct cw1200_common *priv)
{
	struct tx_policy_cache *cache = &priv->tx_policy_cache;
	int i;

	memset(cache, 0, sizeof(*cache));

	spin_lock_init(&cache->lock);
	INIT_LIST_HEAD(&cache->used);
	INIT_LIST_HEAD(&cache->free);

	for (i = 0; i < TX_POLICY_CACHE_SIZE; ++i)
		list_add(&cache->cache[i].link, &cache->free);
}

void tx_policy_clean(struct cw1200_common *priv)
{
	int idx, locked;
	struct tx_policy_cache *cache = &priv->tx_policy_cache;
	struct tx_policy_cache_entry *entry;

	cw1200_tx_queues_lock(priv);
	spin_lock_bh(&cache->lock);
	locked = list_empty(&cache->free);

	for (idx = 0; idx < TX_POLICY_CACHE_SIZE; idx++) {
		entry = &cache->cache[idx];
		/* Policy usage count should be 0 at this time as all queues
		   should be empty */
		if (entry->policy.usage_count) {
			WARN_ON(1);
			entry->policy.usage_count = 0;
			list_move(&entry->link, &cache->free);
		}
		memset(&entry->policy, 0, sizeof(entry->policy));
	}
	if (locked)
		cw1200_tx_queues_unlock(priv);

	cw1200_tx_queues_unlock(priv);
	spin_unlock_bh(&cache->lock);
}

static int tx_policy_get(struct cw1200_common *priv,
		  struct ieee80211_tx_rate *rates,
		  size_t count, bool *renew)
{
	int idx;
	struct tx_policy_cache *cache = &priv->tx_policy_cache;
	struct tx_policy wanted;

	tx_policy_build(priv, &wanted, rates, count);

	spin_lock_bh(&cache->lock);
	if (WARN_ON_ONCE(list_empty(&cache->free))) {
		spin_unlock_bh(&cache->lock);
		return CW1200_INVALID_RATE_ID;
	}
	idx = tx_policy_find(cache, &wanted);
	if (idx >= 0) {
		tx_policy_printk(KERN_DEBUG "[TX policy] Used TX policy: %d\n",
					idx);
		*renew = false;
	} else {
		struct tx_policy_cache_entry *entry;
		*renew = true;
		/* If policy is not found create a new one
		 * using the oldest entry in "free" list */
		entry = list_entry(cache->free.prev,
			struct tx_policy_cache_entry, link);
		entry->policy = wanted;
		idx = entry - cache->cache;
		tx_policy_printk(KERN_DEBUG "[TX policy] New TX policy: %d\n",
					idx);
		tx_policy_dump(&entry->policy);
	}
	tx_policy_use(cache, &cache->cache[idx]);
	if (unlikely(list_empty(&cache->free))) {
		/* Lock TX queues. */
		cw1200_tx_queues_lock(priv);
	}
	spin_unlock_bh(&cache->lock);
	return idx;
}

static void tx_policy_put(struct cw1200_common *priv, int idx)
{
	int usage, locked;
	struct tx_policy_cache *cache = &priv->tx_policy_cache;

	spin_lock_bh(&cache->lock);
	locked = list_empty(&cache->free);
	usage = tx_policy_release(cache, &cache->cache[idx]);
	if (unlikely(locked) && !usage) {
		/* Unlock TX queues. */
		cw1200_tx_queues_unlock(priv);
	}
	spin_unlock_bh(&cache->lock);
}

/*
bool tx_policy_cache_full(struct cw1200_common *priv)
{
	bool ret;
	struct tx_policy_cache *cache = &priv->tx_policy_cache;
	spin_lock_bh(&cache->lock);
	ret = list_empty(&cache->free);
	spin_unlock_bh(&cache->lock);
	return ret;
}
*/

static int tx_policy_upload(struct cw1200_common *priv)
{
	struct tx_policy_cache *cache = &priv->tx_policy_cache;
	int i;
	struct wsm_set_tx_rate_retry_policy arg = {
		.hdr = {
			.numTxRatePolicies = 0,
		}
	};
	spin_lock_bh(&cache->lock);

	/* Upload only modified entries. */
	for (i = 0; i < TX_POLICY_CACHE_SIZE; ++i) {
		struct tx_policy *src = &cache->cache[i].policy;
		if (src->retry_count && !src->uploaded) {
			struct wsm_set_tx_rate_retry_policy_policy *dst =
				&arg.tbl[arg.hdr.numTxRatePolicies];
			dst->policyIndex = i;
			dst->shortRetryCount = priv->short_frame_max_tx_count;
			dst->longRetryCount = priv->long_frame_max_tx_count;

			/* BIT(2) - Terminate retries when Tx rate retry policy
			 *          finishes.
			 * BIT(3) - Count initial frame transmission as part of
			 *          rate retry counting but not as a retry
			 *          attempt */
			dst->policyFlags = BIT(2) | BIT(3);

			memcpy(dst->rateCountIndices, src->tbl,
					sizeof(dst->rateCountIndices));
			src->uploaded = 1;
			++arg.hdr.numTxRatePolicies;
		}
	}
	spin_unlock_bh(&cache->lock);
	cw1200_debug_tx_cache_miss(priv);
	tx_policy_printk(KERN_DEBUG "[TX policy] Upload %d policies\n",
				arg.hdr.numTxRatePolicies);
	return wsm_set_tx_rate_retry_policy(priv, &arg);
}

void tx_policy_upload_work(struct work_struct *work)
{
	struct cw1200_common *priv =
		container_of(work, struct cw1200_common, tx_policy_upload_work);

	tx_policy_printk(KERN_DEBUG "[TX] TX policy upload.\n");
	WARN_ON(tx_policy_upload(priv));

	wsm_unlock_tx(priv);
	cw1200_tx_queues_unlock(priv);
}

/* ******************************************************************** */
/* cw1200 TX implementation						*/

struct cw1200_txinfo {
	struct sk_buff *skb;
	unsigned queue;
	struct ieee80211_tx_info *tx_info;
	const struct ieee80211_rate *rate;
	struct ieee80211_hdr *hdr;
	size_t hdrlen;
	const u8 *da;
	struct cw1200_sta_priv *sta_priv;
	struct cw1200_txpriv txpriv;
};

u32 cw1200_rate_mask_to_wsm(struct cw1200_common *priv, u32 rates)
{
	u32 ret = 0;
	int i;
	for (i = 0; i < 32; ++i) {
		if (rates & BIT(i))
			ret |= BIT(priv->rates[i].hw_value);
	}
	return ret;
}

static const struct ieee80211_rate *
cw1200_get_tx_rate(const struct cw1200_common *priv,
		   const struct ieee80211_tx_rate *rate)
{
	if (rate->idx < 0)
		return NULL;
	if (rate->flags & IEEE80211_TX_RC_MCS)
		return &priv->mcs_rates[rate->idx];
	return &priv->hw->wiphy->bands[priv->channel->band]->
		bitrates[rate->idx];
}

static int
cw1200_tx_h_calc_link_ids(struct cw1200_common *priv,
			  struct cw1200_txinfo *t)
{

	if (likely(t->tx_info->control.sta && t->sta_priv->link_id))
		t->txpriv.raw_link_id =
				t->txpriv.link_id =
				t->sta_priv->link_id;
	else if (priv->mode != NL80211_IFTYPE_AP)
		t->txpriv.raw_link_id =
				t->txpriv.link_id = 0;
	else if (is_multicast_ether_addr(t->da)) {
		if (priv->enable_beacon) {
			t->txpriv.raw_link_id = 0;
			t->txpriv.link_id = CW1200_LINK_ID_AFTER_DTIM;
		} else {
			t->txpriv.raw_link_id = 0;
			t->txpriv.link_id = 0;
		}
	} else {
		t->txpriv.link_id = cw1200_find_link_id(priv, t->da);
		if (!t->txpriv.link_id)
			t->txpriv.link_id = cw1200_alloc_link_id(priv, t->da);
		if (!t->txpriv.link_id) {
			wiphy_err(priv->hw->wiphy,
				"%s: No more link IDs available.\n",
				__func__);
			return -ENOENT;
		}
		t->txpriv.raw_link_id = t->txpriv.link_id;
	}
	if (t->txpriv.raw_link_id)
		priv->link_id_db[t->txpriv.raw_link_id - 1].timestamp =
				jiffies;

#if defined(CONFIG_CW1200_USE_STE_EXTENSIONS)
	if (t->tx_info->control.sta &&
			(t->tx_info->control.sta->uapsd_queues & BIT(t->queue)))
		t->txpriv.link_id = CW1200_LINK_ID_UAPSD;
#endif /* CONFIG_CW1200_USE_STE_EXTENSIONS */
	return 0;
}

static void
cw1200_tx_h_pm(struct cw1200_common *priv,
	       struct cw1200_txinfo *t)
{
	if (unlikely(ieee80211_is_auth(t->hdr->frame_control))) {
		u32 mask = ~BIT(t->txpriv.raw_link_id);
		spin_lock_bh(&priv->ps_state_lock);
		priv->sta_asleep_mask &= mask;
		priv->pspoll_mask &= mask;
		spin_unlock_bh(&priv->ps_state_lock);
	}
}

static void
cw1200_tx_h_calc_tid(struct cw1200_common *priv,
		     struct cw1200_txinfo *t)
{
	if (ieee80211_is_data_qos(t->hdr->frame_control)) {
		u8 *qos = ieee80211_get_qos_ctl(t->hdr);
		t->txpriv.tid = qos[0] & IEEE80211_QOS_CTL_TID_MASK;
	} else if (ieee80211_is_data(t->hdr->frame_control)) {
		t->txpriv.tid = 0;
	}
}

/* IV/ICV injection. */
/* TODO: Quite unoptimal. It's better co modify mac80211
 * to reserve space for IV */
static int
cw1200_tx_h_crypt(struct cw1200_common *priv,
		  struct cw1200_txinfo *t)
{
	size_t iv_len;
	size_t icv_len;
	u8 *icv;
	u8 *newhdr;

	if (!t->tx_info->control.hw_key ||
	    !(t->hdr->frame_control &
	     __cpu_to_le32(IEEE80211_FCTL_PROTECTED)))
		return 0;

	iv_len = t->tx_info->control.hw_key->iv_len;
	icv_len = t->tx_info->control.hw_key->icv_len;

	if (t->tx_info->control.hw_key->cipher == WLAN_CIPHER_SUITE_TKIP)
		icv_len += 8; /* MIC */

	if ((skb_headroom(t->skb) + skb_tailroom(t->skb) <
			 iv_len + icv_len + WSM_TX_EXTRA_HEADROOM) ||
			(skb_headroom(t->skb) <
			 iv_len + WSM_TX_EXTRA_HEADROOM)) {
		wiphy_err(priv->hw->wiphy,
			"Bug: no space allocated for crypto headers.\n"
			"headroom: %d, tailroom: %d, "
			"req_headroom: %d, req_tailroom: %d\n"
			"Please fix it in cw1200_get_skb().\n",
			skb_headroom(t->skb), skb_tailroom(t->skb),
			iv_len + WSM_TX_EXTRA_HEADROOM, icv_len);
		return -ENOMEM;
	} else if (skb_tailroom(t->skb) < icv_len) {
		size_t offset = icv_len - skb_tailroom(t->skb);
		u8 *p;
		wiphy_warn(priv->hw->wiphy,
			"Slowpath: tailroom is not big enough. "
			"Req: %d, got: %d.\n",
			icv_len, skb_tailroom(t->skb));

		p = skb_push(t->skb, offset);
		memmove(p, &p[offset], t->skb->len - offset);
		skb_trim(t->skb, t->skb->len - offset);
	}

	newhdr = skb_push(t->skb, iv_len);
	memmove(newhdr, newhdr + iv_len, t->hdrlen);
	t->hdr = (struct ieee80211_hdr *) newhdr;
	t->hdrlen += iv_len;
	icv = skb_put(t->skb, icv_len);

	return 0;
}

static int
cw1200_tx_h_align(struct cw1200_common *priv,
		  struct cw1200_txinfo *t,
		  u8 *flags)
{
	size_t offset = (size_t)t->skb->data & 3;

	if (!offset)
		return 0;

	if (offset & 1) {
		wiphy_err(priv->hw->wiphy,
			"Bug: attempt to transmit a frame "
			"with wrong alignment: %d\n",
			offset);
		return -EINVAL;
	}

	if (skb_headroom(t->skb) < offset) {
		wiphy_err(priv->hw->wiphy,
			"Bug: no space allocated "
			"for DMA alignment.\n"
			"headroom: %d\n",
			skb_headroom(t->skb));
		return -ENOMEM;
	}
	skb_push(t->skb, offset);
	t->hdrlen += offset;
	t->txpriv.offset += offset;
	*flags |= WSM_TX_2BYTES_SHIFT;
	cw1200_debug_tx_align(priv);
	return 0;
}

static int
cw1200_tx_h_action(struct cw1200_common *priv,
		   struct cw1200_txinfo *t)
{
	struct ieee80211_mgmt *mgmt =
		(struct ieee80211_mgmt *)t->hdr;
	if (ieee80211_is_action(t->hdr->frame_control) &&
			mgmt->u.action.category == WLAN_CATEGORY_BACK)
		return 1;
	else
		return 0;
}

/* Add WSM header */
static struct wsm_tx *
cw1200_tx_h_wsm(struct cw1200_common *priv,
		struct cw1200_txinfo *t)
{
	struct wsm_tx *wsm;

	if (skb_headroom(t->skb) < sizeof(struct wsm_tx)) {
		wiphy_err(priv->hw->wiphy,
			"Bug: no space allocated "
			"for WSM header.\n"
			"headroom: %d\n",
			skb_headroom(t->skb));
		return NULL;
	}

	wsm = (struct wsm_tx *)skb_push(t->skb, sizeof(struct wsm_tx));
	t->txpriv.offset += sizeof(struct wsm_tx);
	memset(wsm, 0, sizeof(*wsm));
	wsm->hdr.len = __cpu_to_le16(t->skb->len);
	wsm->hdr.id = __cpu_to_le16(0x0004);
	wsm->queueId = wsm_queue_id_to_wsm(t->queue);
	return wsm;
}

/* BT Coex specific handling */
static void
cw1200_tx_h_bt(struct cw1200_common *priv,
	       struct cw1200_txinfo *t,
	       struct wsm_tx *wsm)
{
	u8 priority = 0;

	if (!priv->is_BT_Present)
		return;

	if (unlikely(ieee80211_is_nullfunc(t->hdr->frame_control)))
		priority = WSM_EPTA_PRIORITY_MGT;
	else if (ieee80211_is_data(t->hdr->frame_control)) {
		/* Skip LLC SNAP header (+6) */
		u8 *payload = &t->skb->data[t->hdrlen];
		u16 *ethertype = (u16 *) &payload[6];
		if (unlikely(*ethertype == __be16_to_cpu(ETH_P_PAE)))
			priority = WSM_EPTA_PRIORITY_EAPOL;
	} else if (unlikely(ieee80211_is_assoc_req(t->hdr->frame_control) ||
		ieee80211_is_reassoc_req(t->hdr->frame_control))) {
		struct ieee80211_mgmt *mgt_frame =
				(struct ieee80211_mgmt *)t->hdr;

		if (mgt_frame->u.assoc_req.listen_interval <
						priv->listen_interval) {
			txrx_printk(KERN_DEBUG
				"Modified Listen Interval to %d from %d\n",
				priv->listen_interval,
				mgt_frame->u.assoc_req.listen_interval);
			/* Replace listen interval derieved from
			 * the one read from SDD */
			mgt_frame->u.assoc_req.listen_interval =
				priv->listen_interval;
		}
	}

	if (likely(!priority)) {
		if (ieee80211_is_action(t->hdr->frame_control))
			priority = WSM_EPTA_PRIORITY_ACTION;
		else if (ieee80211_is_mgmt(t->hdr->frame_control))
			priority = WSM_EPTA_PRIORITY_MGT;
		else if ((wsm->queueId == WSM_QUEUE_VOICE))
			priority = WSM_EPTA_PRIORITY_VOICE;
		else if ((wsm->queueId == WSM_QUEUE_VIDEO))
			priority = WSM_EPTA_PRIORITY_VIDEO;
		else
			priority = WSM_EPTA_PRIORITY_DATA;
	}

	txrx_printk(KERN_DEBUG "[TX] EPTA priority %d.\n",
		priority);

	wsm->flags |= priority << 1;
}

static int
cw1200_tx_h_rate_policy(struct cw1200_common *priv,
			struct cw1200_txinfo *t,
			struct wsm_tx *wsm)
{
	bool tx_policy_renew = false;

	t->txpriv.rate_id = tx_policy_get(priv,
		t->tx_info->control.rates, IEEE80211_TX_MAX_RATES,
		&tx_policy_renew);
	if (t->txpriv.rate_id == CW1200_INVALID_RATE_ID)
		return -EFAULT;

	wsm->flags |= t->txpriv.rate_id << 4;

	t->rate = cw1200_get_tx_rate(priv,
		&t->tx_info->control.rates[0]),
	wsm->maxTxRate = t->rate->hw_value;
	if (t->rate->flags & IEEE80211_TX_RC_MCS) {
		if (cw1200_ht_greenfield(&priv->ht_info))
			wsm->htTxParameters |=
				__cpu_to_le32(WSM_HT_TX_GREENFIELD);
		else
			wsm->htTxParameters |=
				__cpu_to_le32(WSM_HT_TX_MIXED);
	}

	if (tx_policy_renew) {
		tx_policy_printk(KERN_DEBUG "[TX] TX policy renew.\n");
		/* It's not so optimal to stop TX queues every now and then.
		 * Maybe it's better to reimplement task scheduling with
		 * a counter. */
		/* cw1200_tx_queues_lock(priv); */
		/* Definetly better. TODO. */
		wsm_lock_tx_async(priv);
		cw1200_tx_queues_lock(priv);
		if (queue_work(priv->workqueue,
				&priv->tx_policy_upload_work) <= 0) {
			cw1200_tx_queues_unlock(priv);
			wsm_unlock_tx(priv);
		}
	}
	return 0;
}

static bool
cw1200_tx_h_pm_state(struct cw1200_common *priv,
		     struct cw1200_txinfo *t)
{
	int was_buffered = 1;

	if (t->txpriv.link_id == CW1200_LINK_ID_AFTER_DTIM &&
			!priv->buffered_multicasts) {
		priv->buffered_multicasts = true;
		if (priv->sta_asleep_mask)
			queue_work(priv->workqueue,
				&priv->multicast_start_work);
	}

	if (t->txpriv.raw_link_id && t->txpriv.tid < CW1200_MAX_TID)
		was_buffered = priv->link_id_db[t->txpriv.raw_link_id - 1]
				.buffered[t->txpriv.tid]++;

	return !was_buffered;
}

static int
cw1200_tx_h_skb_pad(struct cw1200_common *priv,
		    struct wsm_tx *wsm,
		    struct sk_buff *skb)
{
	size_t len = __le16_to_cpu(wsm->hdr.len);
	size_t padded_len = priv->sbus_ops->align_size(priv->sbus_priv, len);

	if (WARN_ON(skb_padto(skb, padded_len) != 0)) {
		return -EINVAL;
	}
	return 0;
}

/* ******************************************************************** */

void cw1200_tx(struct ieee80211_hw *dev, struct sk_buff *skb)
{
	struct cw1200_common *priv = dev->priv;
	struct cw1200_txinfo t = {
		.skb = skb,
		.queue = skb_get_queue_mapping(skb),
		.tx_info = IEEE80211_SKB_CB(skb),
		.hdr = (struct ieee80211_hdr *)skb->data,
		.txpriv.tid = CW1200_MAX_TID,
		.txpriv.rate_id = CW1200_INVALID_RATE_ID,
	};
	struct ieee80211_sta *sta;
	struct wsm_tx *wsm;
	bool tid_update = 0;
	u8 flags = 0;
	int ret;

	t.hdrlen = ieee80211_hdrlen(t.hdr->frame_control);
	t.da = ieee80211_get_DA(t.hdr);
	t.sta_priv =
		(struct cw1200_sta_priv *)&t.tx_info->control.sta->drv_priv;

	if (WARN_ON(t.queue >= 4))
		goto drop;

	ret = cw1200_tx_h_calc_link_ids(priv, &t);
	if (ret)
		goto drop;

	txrx_printk(KERN_DEBUG "[TX] TX %d bytes "
			"(queue: %d, link_id: %d (%d)).\n",
			skb->len, t.queue, t.txpriv.link_id,
			t.txpriv.raw_link_id);

	cw1200_tx_h_pm(priv, &t);
	cw1200_tx_h_calc_tid(priv, &t);
	ret = cw1200_tx_h_crypt(priv, &t);
	if (ret)
		goto drop;
	ret = cw1200_tx_h_align(priv, &t, &flags);
	if (ret)
		goto drop;
	ret = cw1200_tx_h_action(priv, &t);
	if (ret)
		goto drop;
	wsm = cw1200_tx_h_wsm(priv, &t);
	if (!wsm) {
		ret = -ENOMEM;
		goto drop;
	}
	wsm->flags |= flags;
	cw1200_tx_h_bt(priv, &t, wsm);
	ret = cw1200_tx_h_rate_policy(priv, &t, wsm);
	if (ret)
		goto drop;

	ret = cw1200_tx_h_skb_pad(priv, wsm, skb);
	if (ret)
		goto drop;

	rcu_read_lock();
	sta = rcu_dereference(t.tx_info->control.sta);

	spin_lock_bh(&priv->ps_state_lock);
	{
		tid_update = cw1200_tx_h_pm_state(priv, &t);
		BUG_ON(cw1200_queue_put(&priv->tx_queue[t.queue],
				t.skb, &t.txpriv));
	}
	spin_unlock_bh(&priv->ps_state_lock);

#if defined(CONFIG_CW1200_USE_STE_EXTENSIONS)
	if (tid_update && sta)
		ieee80211_sta_set_buffered(sta,
				t.txpriv.tid, true);
#endif /* CONFIG_CW1200_USE_STE_EXTENSIONS */

	rcu_read_unlock();

	cw1200_bh_wakeup(priv);

	return;

drop:
	cw1200_skb_dtor(priv, skb, &t.txpriv);
	return;
}

/* ******************************************************************** */

static int cw1200_handle_action_rx(struct cw1200_common *priv,
				   struct sk_buff *skb)
{
	struct ieee80211_mgmt *mgmt = (void *)skb->data;

	/* Filter block ACK negotiation: fully controlled by firmware */
	if (mgmt->u.action.category == WLAN_CATEGORY_BACK)
		return 1;

	return 0;
}

static int cw1200_handle_pspoll(struct cw1200_common *priv,
				struct sk_buff *skb)
{
	struct ieee80211_sta *sta;
	struct ieee80211_pspoll *pspoll =
		(struct ieee80211_pspoll *) skb->data;
	int link_id = 0;
	u32 pspoll_mask = 0;
	int drop = 1;
	int i;

	if (priv->join_status != CW1200_JOIN_STATUS_AP)
		goto done;
	if (memcmp(priv->vif->addr, pspoll->bssid, ETH_ALEN))
		goto done;

	rcu_read_lock();
	sta = ieee80211_find_sta(priv->vif, pspoll->ta);
	if (sta) {
		struct cw1200_sta_priv *sta_priv;
		sta_priv = (struct cw1200_sta_priv *)&sta->drv_priv;
		link_id = sta_priv->link_id;
		pspoll_mask = BIT(sta_priv->link_id);
	}
	rcu_read_unlock();
	if (!link_id)
		goto done;

	priv->pspoll_mask |= pspoll_mask;
	drop = 0;

	/* Do not report pspols if data for given link id is
	 * queued already. */
	for (i = 0; i < 4; ++i) {
		if (cw1200_queue_get_num_queued(
				&priv->tx_queue[i],
				pspoll_mask)) {
			cw1200_bh_wakeup(priv);
			drop = 1;
			break;
		}
	}
	txrx_printk(KERN_DEBUG "[RX] PSPOLL: %s\n", drop ? "local" : "fwd");
done:
	return drop;
}

/* ******************************************************************** */

void cw1200_tx_confirm_cb(struct cw1200_common *priv,
			  struct wsm_tx_confirm *arg)
{
	u8 queue_id = cw1200_queue_get_queue_id(arg->packetID);
	struct cw1200_queue *queue = &priv->tx_queue[queue_id];
	struct sk_buff *skb;
	const struct cw1200_txpriv *txpriv;

	txrx_printk(KERN_DEBUG "[TX] TX confirm: %d, %d.\n",
		arg->status, arg->ackFailures);

	if (unlikely(cw1200_itp_tx_running(priv)))
		return;

	if (unlikely(priv->mode == NL80211_IFTYPE_UNSPECIFIED)) {
		/* STA is stopped. */
		return;
	}

	if (WARN_ON(queue_id >= 4))
		return;

	if (arg->status)
		txrx_printk(KERN_DEBUG "TX failed: %d.\n",
				arg->status);

	if ((arg->status == WSM_REQUEUE) &&
	    (arg->flags & WSM_TX_STATUS_REQUEUE)) {
		/* "Requeue" means "implicit suspend" */
		struct wsm_suspend_resume suspend = {
			.link_id = arg->link_id,
			.stop = 1,
			.multicast = !arg->link_id,
		};
		cw1200_suspend_resume(priv, &suspend);
		wiphy_warn(priv->hw->wiphy, "Requeue for link_id %d (try %d)."
			" STAs asleep: 0x%.8X\n",
			arg->link_id,
			cw1200_queue_get_generation(arg->packetID) + 1,
			priv->sta_asleep_mask);
		WARN_ON(cw1200_queue_requeue(queue,
				arg->packetID));
		spin_lock_bh(&priv->ps_state_lock);
		if (!arg->link_id) {
			priv->buffered_multicasts = true;
			if (priv->sta_asleep_mask) {
				queue_work(priv->workqueue,
					&priv->multicast_start_work);
			}
		}
		spin_unlock_bh(&priv->ps_state_lock);
	} else if (!WARN_ON(cw1200_queue_get_skb(
			queue, arg->packetID, &skb, &txpriv))) {
		struct ieee80211_tx_info *tx = IEEE80211_SKB_CB(skb);
		int tx_count = arg->ackFailures;
		u8 ht_flags = 0;
		int i;

		if (cw1200_ht_greenfield(&priv->ht_info))
			ht_flags |= IEEE80211_TX_RC_GREEN_FIELD;

		if (likely(!arg->status)) {
			spin_lock(&priv->bss_loss_lock);
			if ((priv->bss_loss_status == CW1200_BSS_LOSS_CONFIRMING) &&
			    (priv->bss_loss_checking < BSS_LOSS_CHECKING_MAX_TRY)) {
				priv->bss_loss_status = CW1200_BSS_LOSS_CHECKING;
				priv->bss_loss_checking++;
				spin_unlock(&priv->bss_loss_lock);
				cancel_delayed_work(&priv->bss_loss_work);
				queue_delayed_work(priv->workqueue,
                                          &priv->bss_loss_work, 0);
			} else
				spin_unlock(&priv->bss_loss_lock);
			tx->flags |= IEEE80211_TX_STAT_ACK;
			priv->cqm_tx_failure_count = 0;
			++tx_count;
			cw1200_debug_txed(priv);
			if (arg->flags & WSM_TX_STATUS_AGGREGATION) {
				/* Do not report aggregation to mac80211:
				 * it confuses minstrel a lot. */
				/* tx->flags |= IEEE80211_TX_STAT_AMPDU; */
				cw1200_debug_txed_agg(priv);
			}
		} else {
			spin_lock(&priv->bss_loss_lock);
			if (priv->bss_loss_status ==
					CW1200_BSS_LOSS_CONFIRMING &&
					priv->bss_loss_confirm_id ==
					arg->packetID) {
				priv->bss_loss_status =
					CW1200_BSS_LOSS_CONFIRMED;
				priv->bss_loss_checking = 0;
				spin_unlock(&priv->bss_loss_lock);
				cancel_delayed_work(&priv->bss_loss_work);
				queue_delayed_work(priv->workqueue,
						&priv->bss_loss_work, 0);
			} else
				spin_unlock(&priv->bss_loss_lock);

			/* TODO: Update TX failure counters */
			if (unlikely(priv->cqm_tx_failure_thold &&
			     (++priv->cqm_tx_failure_count >
			      priv->cqm_tx_failure_thold))) {
				priv->cqm_tx_failure_thold = 0;
				queue_work(priv->workqueue,
						&priv->tx_failure_work);
			}
			if (tx_count)
				++tx_count;
		}

		for (i = 0; i < IEEE80211_TX_MAX_RATES; ++i) {
			if (tx->status.rates[i].count >= tx_count) {
				tx->status.rates[i].count = tx_count;
				break;
			}
			tx_count -= tx->status.rates[i].count;
			if (tx->status.rates[i].flags & IEEE80211_TX_RC_MCS)
				tx->status.rates[i].flags |= ht_flags;
		}

		for (++i; i < IEEE80211_TX_MAX_RATES; ++i) {
			tx->status.rates[i].count = 0;
			tx->status.rates[i].idx = -1;
		}


		cw1200_queue_remove(queue, arg->packetID);
	}
}

static void cw1200_notify_buffered_tx(struct cw1200_common *priv,
			       struct sk_buff *skb, int link_id, int tid)
{
#if defined(CONFIG_CW1200_USE_STE_EXTENSIONS)
	struct ieee80211_sta *sta;
	struct ieee80211_hdr *hdr;
	u8 *buffered;
	u8 still_buffered = 0;

	if (link_id && tid < CW1200_MAX_TID) {
		buffered = priv->link_id_db
				[link_id - 1].buffered;

		spin_lock_bh(&priv->ps_state_lock);
		if (!WARN_ON(!buffered[tid]))
			still_buffered = --buffered[tid];
		spin_unlock_bh(&priv->ps_state_lock);

		if (!still_buffered && tid < CW1200_MAX_TID) {
			hdr = (struct ieee80211_hdr *) skb->data;
			rcu_read_lock();
			sta = ieee80211_find_sta(priv->vif, hdr->addr1);
			if (sta)
				ieee80211_sta_set_buffered(sta, tid, false);
			rcu_read_unlock();
		}
	}
#endif /* CONFIG_CW1200_USE_STE_EXTENSIONS */
}

void cw1200_skb_dtor(struct cw1200_common *priv,
		     struct sk_buff *skb,
		     const struct cw1200_txpriv *txpriv)
{
	skb_pull(skb, txpriv->offset);
	if (txpriv->rate_id != CW1200_INVALID_RATE_ID) {
		cw1200_notify_buffered_tx(priv, skb,
				txpriv->raw_link_id, txpriv->tid);
		tx_policy_put(priv, txpriv->rate_id);
	}
	if (likely(!cw1200_is_itp(priv)))
		ieee80211_tx_status(priv->hw, skb);
}

void cw1200_rx_cb(struct cw1200_common *priv,
		  struct wsm_rx *arg,
		  struct sk_buff **skb_p)
{
	struct sk_buff *skb = *skb_p;
	struct ieee80211_rx_status *hdr = IEEE80211_SKB_RXCB(skb);
	struct ieee80211_hdr *frame = (struct ieee80211_hdr *)skb->data;
#if defined(CONFIG_CW1200_USE_STE_EXTENSIONS)
	struct ieee80211_mgmt *mgmt = (struct ieee80211_mgmt *)skb->data;
#endif
	struct cw1200_link_entry *entry = NULL;
	unsigned long grace_period;
	bool early_data = false;
	bool p2p = priv->vif && priv->vif->p2p;
	size_t hdrlen;

	hdr->flag = 0;

	if (unlikely(priv->mode == NL80211_IFTYPE_UNSPECIFIED)) {
		/* STA is stopped. */
		goto drop;
	}

	if (arg->link_id && arg->link_id <= CW1200_MAX_STA_IN_AP_MODE) {
		entry =	&priv->link_id_db[arg->link_id - 1];
		if (entry->status == CW1200_LINK_SOFT &&
				ieee80211_is_data(frame->frame_control))
			early_data = true;
		entry->timestamp = jiffies;
	}
#if defined(CONFIG_CW1200_USE_STE_EXTENSIONS)
	else if (p2p
			&& ieee80211_is_action(frame->frame_control)
			&& (mgmt->u.action.category == WLAN_CATEGORY_PUBLIC)) {
		txrx_printk(KERN_DEBUG "[RX] Going to MAP&RESET link ID\n");

		if (work_pending(&priv->linkid_reset_work))
			WARN_ON(1);

		memcpy(&priv->action_frame_sa[0],
				ieee80211_get_SA(frame), ETH_ALEN);
		priv->action_linkid = 0;
		schedule_work(&priv->linkid_reset_work);
	}

	if (arg->link_id && p2p
			&& ieee80211_is_action(frame->frame_control)
			&& (mgmt->u.action.category == WLAN_CATEGORY_PUBLIC)) {
		/* Link ID already exists for the ACTION frame.
		 * Reset and Remap */
		if (work_pending(&priv->linkid_reset_work))
			WARN_ON(1);
		memcpy(&priv->action_frame_sa[0],
				ieee80211_get_SA(frame), ETH_ALEN);
		priv->action_linkid = arg->link_id;
		schedule_work(&priv->linkid_reset_work);
	}
#endif
	if (unlikely(arg->status)) {
		if (arg->status == WSM_STATUS_MICFAILURE) {
			txrx_printk(KERN_DEBUG "[RX] MIC failure.\n");
			hdr->flag |= RX_FLAG_MMIC_ERROR;
		} else if (arg->status == WSM_STATUS_NO_KEY_FOUND) {
			txrx_printk(KERN_DEBUG "[RX] No key found.\n");
			goto drop;
		} else {
			txrx_printk(KERN_DEBUG "[RX] Receive failure: %d.\n",
				arg->status);
			goto drop;
		}
	}

	if (skb->len < sizeof(struct ieee80211_pspoll)) {
		wiphy_warn(priv->hw->wiphy, "Mailformed SDU rx'ed. "
				"Size is lesser than IEEE header.\n");
		goto drop;
	}

	if (unlikely(ieee80211_is_pspoll(frame->frame_control)))
		if (cw1200_handle_pspoll(priv, skb))
			goto drop;

	hdr->mactime = 0; /* Not supported by WSM */
	hdr->band = (arg->channelNumber > 14) ?
			IEEE80211_BAND_5GHZ : IEEE80211_BAND_2GHZ;
	hdr->freq = ieee80211_channel_to_frequency(
			arg->channelNumber,
			hdr->band);

	if (arg->rxedRate >= 14) {
		hdr->flag |= RX_FLAG_HT;
		hdr->rate_idx = arg->rxedRate - 14;
	} else if (arg->rxedRate >= 4) {
		hdr->rate_idx = arg->rxedRate - 2;
	} else {
		hdr->rate_idx = arg->rxedRate;
	}

	hdr->signal = (s8)arg->rcpiRssi;
	hdr->antenna = 0;

	hdrlen = ieee80211_hdrlen(frame->frame_control);

	if (WSM_RX_STATUS_ENCRYPTION(arg->flags)) {
		size_t iv_len = 0, icv_len = 0;

		hdr->flag |= RX_FLAG_DECRYPTED | RX_FLAG_IV_STRIPPED;

		/* Oops... There is no fast way to ask mac80211 about
		 * IV/ICV lengths. Even defineas are not exposed.*/
		switch (WSM_RX_STATUS_ENCRYPTION(arg->flags)) {
		case WSM_RX_STATUS_WEP:
			iv_len = 4 /* WEP_IV_LEN */;
			icv_len = 4 /* WEP_ICV_LEN */;
			break;
		case WSM_RX_STATUS_TKIP:
			iv_len = 8 /* TKIP_IV_LEN */;
			icv_len = 4 /* TKIP_ICV_LEN */
				+ 8 /*MICHAEL_MIC_LEN*/;
			hdr->flag |= RX_FLAG_MMIC_STRIPPED;
			break;
		case WSM_RX_STATUS_AES:
			iv_len = 8 /* CCMP_HDR_LEN */;
			icv_len = 8 /* CCMP_MIC_LEN */;
			break;
		case WSM_RX_STATUS_WAPI:
			iv_len = 18 /* WAPI_HDR_LEN */;
			icv_len = 16 /* WAPI_MIC_LEN */;
			break;
		default:
			WARN_ON("Unknown encryption type");
			goto drop;
		}

		/* Firmware strips ICV in case of MIC failure. */
		if (arg->status == WSM_STATUS_MICFAILURE)
			icv_len = 0;

		if (skb->len < hdrlen + iv_len + icv_len) {
			wiphy_warn(priv->hw->wiphy, "Mailformed SDU rx'ed. "
				"Size is lesser than crypto headers.\n");
			goto drop;
		}

		/* Remove IV, ICV and MIC */
		skb_trim(skb, skb->len - icv_len);
		memmove(skb->data + iv_len, skb->data, hdrlen);
		skb_pull(skb, iv_len);
	}

	cw1200_debug_rxed(priv);
	if (arg->flags & WSM_RX_STATUS_AGGREGATE)
		cw1200_debug_rxed_agg(priv);

	if (ieee80211_is_action(frame->frame_control) &&
			(arg->flags & WSM_RX_STATUS_ADDRESS1)) {
		if (cw1200_handle_action_rx(priv, skb))
			return;
	} else if (ieee80211_is_beacon(frame->frame_control) &&
			!arg->status &&
			!memcmp(ieee80211_get_SA(frame), priv->join_bssid,
				ETH_ALEN)) {
		const u8 *tim_ie;
		u8 *ies = ((struct ieee80211_mgmt *)
			  (skb->data))->u.beacon.variable;
		size_t ies_len = skb->len - (ies - (u8 *)(skb->data));

		tim_ie = cw1200_get_ie(ies, ies_len, WLAN_EID_TIM);
		if (tim_ie) {
			struct ieee80211_tim_ie *tim =
				(struct ieee80211_tim_ie *)&tim_ie[2];

			if (priv->join_dtim_period != tim->dtim_period) {
				priv->join_dtim_period = tim->dtim_period;
				queue_work(priv->workqueue,
					&priv->set_beacon_wakeup_period_work);
			}
		}
		if (priv->disable_beacon_filter) {
			priv->disable_beacon_filter = false;
			queue_work(priv->workqueue,
				&priv->update_filtering_work);
		}
	}

	/* Stay awake for 1sec. after frame is received to give
	 * userspace chance to react and acquire appropriate
	 * wakelock. */
	if (ieee80211_is_auth(frame->frame_control))
		grace_period = 5 * HZ;
	else if (ieee80211_is_deauth(frame->frame_control))
		grace_period = 5 * HZ;
	else
		grace_period = 1 * HZ;

	cw1200_pm_stay_awake(&priv->pm_state, grace_period);

	if (unlikely(cw1200_itp_rxed(priv, skb)))
		consume_skb(skb);
	else if (unlikely(early_data)) {
		spin_lock_bh(&priv->ps_state_lock);
		/* Double-check status with lock held */
		if (entry->status == CW1200_LINK_SOFT)
			skb_queue_tail(&entry->rx_queue, skb);
		else
			ieee80211_rx_irqsafe(priv->hw, skb);
		spin_unlock_bh(&priv->ps_state_lock);
	} else {
		ieee80211_rx_irqsafe(priv->hw, skb);
	}
	*skb_p = NULL;

	return;

drop:
	/* TODO: update failure counters */
	return;
}

/* ******************************************************************** */
/* Security								*/

int cw1200_alloc_key(struct cw1200_common *priv)
{
	int idx;

	idx = ffs(~priv->key_map) - 1;
	if (idx < 0 || idx > WSM_KEY_MAX_INDEX)
		return -1;

	priv->key_map |= BIT(idx);
	priv->keys[idx].entryIndex = idx;
	return idx;
}

void cw1200_free_key(struct cw1200_common *priv, int idx)
{
	BUG_ON(!(priv->key_map & BIT(idx)));
	memset(&priv->keys[idx], 0, sizeof(priv->keys[idx]));
	priv->key_map &= ~BIT(idx);
}

void cw1200_free_keys(struct cw1200_common *priv)
{
	memset(&priv->keys, 0, sizeof(priv->keys));
	priv->key_map = 0;
}

int cw1200_upload_keys(struct cw1200_common *priv)
{
	int idx, ret = 0;
	for (idx = 0; idx <= WSM_KEY_MAX_INDEX; ++idx)
		if (priv->key_map & BIT(idx)) {
			ret = wsm_add_key(priv, &priv->keys[idx]);
			if (ret < 0)
				break;
		}
	return ret;
}
#if defined(CONFIG_CW1200_USE_STE_EXTENSIONS)
/* Workaround for WFD test case 6.1.10 */
void cw1200_link_id_reset(struct work_struct *work)
{
	struct cw1200_common *priv =
		container_of(work, struct cw1200_common, linkid_reset_work);
	int temp_linkid;

	if (!priv->action_linkid) {
		/* In GO mode we can receive ACTION frames without a linkID */
		temp_linkid = cw1200_alloc_link_id(priv,
				&priv->action_frame_sa[0]);
		WARN_ON(!temp_linkid);
		if (temp_linkid) {
			/* Make sure we execute the WQ */
			flush_workqueue(priv->workqueue);
			/* Release the link ID */
			spin_lock_bh(&priv->ps_state_lock);
			priv->link_id_db[temp_linkid - 1].prev_status =
				priv->link_id_db[temp_linkid - 1].status;
			priv->link_id_db[temp_linkid - 1].status =
				CW1200_LINK_RESET;
			spin_unlock_bh(&priv->ps_state_lock);
			wsm_lock_tx_async(priv);
			if (queue_work(priv->workqueue,
					&priv->link_id_work) <= 0)
				wsm_unlock_tx(priv);
		}
	} else {
		spin_lock_bh(&priv->ps_state_lock);
		priv->link_id_db[priv->action_linkid - 1].prev_status =
			priv->link_id_db[priv->action_linkid - 1].status;
		priv->link_id_db[priv->action_linkid - 1].status =
			CW1200_LINK_RESET_REMAP;
		spin_unlock_bh(&priv->ps_state_lock);
		wsm_lock_tx_async(priv);
		if (queue_work(priv->workqueue, &priv->link_id_work) <= 0)
			wsm_unlock_tx(priv);
		flush_workqueue(priv->workqueue);
	}
}
#endif
