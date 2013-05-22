/*
 * DebugFS code for ST-Ericsson CW1200 mac80211 driver
 *
 * Copyright (c) 2011, ST-Ericsson
 * Author: Dmitry Tarnyagin <dmitry.tarnyagin@stericsson.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef CW1200_DEBUG_H_INCLUDED
#define CW1200_DEBUG_H_INCLUDED

#include "itp.h"

struct cw200_common;

#ifdef CONFIG_CW1200_DEBUGFS

struct cw1200_debug_priv {
	struct dentry *debugfs_phy;
	int tx;
	int tx_agg;
	int rx;
	int rx_agg;
	int tx_multi;
	int tx_multi_frames;
	int tx_cache_miss;
	int tx_align;
	int tx_ttl;
	int tx_burst;
	int rx_burst;
#ifdef CONFIG_CW1200_ITP
	struct cw1200_itp itp;
#endif /* CONFIG_CW1200_ITP */
};

int cw1200_debug_init(struct cw1200_common *priv);
void cw1200_debug_release(struct cw1200_common *priv);

static inline void cw1200_debug_txed(struct cw1200_common *priv)
{
	++priv->debug->tx;
}

static inline void cw1200_debug_txed_agg(struct cw1200_common *priv)
{
	++priv->debug->tx_agg;
}

static inline void cw1200_debug_txed_multi(struct cw1200_common *priv,
					   int count)
{
	++priv->debug->tx_multi;
	priv->debug->tx_multi_frames += count;
}

static inline void cw1200_debug_rxed(struct cw1200_common *priv)
{
	++priv->debug->rx;
}

static inline void cw1200_debug_rxed_agg(struct cw1200_common *priv)
{
	++priv->debug->rx_agg;
}

static inline void cw1200_debug_tx_cache_miss(struct cw1200_common *priv)
{
	++priv->debug->tx_cache_miss;
}

static inline void cw1200_debug_tx_align(struct cw1200_common *priv)
{
	++priv->debug->tx_align;
}

static inline void cw1200_debug_tx_ttl(struct cw1200_common *priv)
{
	++priv->debug->tx_ttl;
}

static inline void cw1200_debug_tx_burst(struct cw1200_common *priv)
{
	++priv->debug->tx_burst;
}

static inline void cw1200_debug_rx_burst(struct cw1200_common *priv)
{
	++priv->debug->rx_burst;
}

int cw1200_print_fw_version(struct cw1200_common *priv, u8 *buf, size_t len);

#else /* CONFIG_CW1200_DEBUGFS */

static inline int cw1200_debug_init(struct cw1200_common *priv)
{
	return 0;
}

static inline void cw1200_debug_release(struct cw1200_common *priv)
{
}

static inline void cw1200_debug_txed(struct cw1200_common *priv)
{
}

static inline void cw1200_debug_txed_agg(struct cw1200_common *priv)
{
}

static inline void cw1200_debug_txed_multi(struct cw1200_common *priv,
					   int count)
{
}

static inline void cw1200_debug_rxed(struct cw1200_common *priv)
{
}

static inline void cw1200_debug_rxed_agg(struct cw1200_common *priv)
{
}

static inline void cw1200_debug_tx_cache_miss(struct cw1200_common *priv)
{
}

static inline void cw1200_debug_tx_align(struct cw1200_common *priv)
{
}

static inline void cw1200_debug_tx_ttl(struct cw1200_common *priv)
{
}

static inline void cw1200_debug_tx_burst(struct cw1200_common *priv)
{
}

static inline void cw1200_debug_rx_burst(struct cw1200_common *priv)
{
}

int cw1200_print_fw_version(struct cw1200_common *priv, u8 *buf, size_t len)
{
}

#endif /* CONFIG_CW1200_DEBUGFS */

#endif /* CW1200_DEBUG_H_INCLUDED */
