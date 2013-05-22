/*
 * Device handling thread implementation for mac80211 ST-Ericsson CW1200 drivers
 *
 * Copyright (c) 2010, ST-Ericsson
 * Author: Dmitry Tarnyagin <dmitry.tarnyagin@stericsson.com>
 *
 * Based on:
 * ST-Ericsson UMAC CW1200 driver, which is
 * Copyright (c) 2010, ST-Ericsson
 * Author: Ajitpal Singh <ajitpal.singh@stericsson.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <net/mac80211.h>
#include <linux/kthread.h>

#include "cw1200.h"
#include "bh.h"
#include "hwio.h"
#include "wsm.h"
#include "sbus.h"
#include "debug.h"

#if defined(CONFIG_CW1200_BH_DEBUG)
#define bh_printk pr_debug
#else
#define bh_printk(...)
#endif

static int cw1200_bh(void *arg);

/* TODO: Verify these numbers with WSM specification. */
#define DOWNLOAD_BLOCK_SIZE_WR	(0x1000 - 4)
/* an SPI message cannot be bigger than (2"12-1)*2 bytes
 * "*2" to cvt to bytes */
#define MAX_SZ_RD_WR_BUFFERS	(DOWNLOAD_BLOCK_SIZE_WR*2)
#define PIGGYBACK_CTRL_REG	(2)
#define EFFECTIVE_BUF_SIZE	(MAX_SZ_RD_WR_BUFFERS - PIGGYBACK_CTRL_REG)

/* Suspend state privates */
enum cw1200_bh_pm_state {
	CW1200_BH_RESUMED = 0,
	CW1200_BH_SUSPEND,
	CW1200_BH_SUSPENDED,
	CW1200_BH_RESUME,
};

typedef int (*cw1200_wsm_handler)(struct cw1200_common *priv,
	u8 *data, size_t size);


int cw1200_register_bh(struct cw1200_common *priv)
{
	int err = 0;

	bh_printk(KERN_DEBUG "[BH] register.\n");
	atomic_set(&priv->bh_rx, 0);
	atomic_set(&priv->bh_tx, 0);
	atomic_set(&priv->bh_term, 0);
	atomic_set(&priv->bh_suspend, CW1200_BH_RESUMED);
	priv->buf_id_tx = 0;
	priv->buf_id_rx = 0;
	priv->bh_error = 0;
	priv->hw_bufs_used = 0;
	init_waitqueue_head(&priv->bh_wq);
	init_waitqueue_head(&priv->bh_evt_wq);

	err = !queue_work(priv->bh_workqueue, &priv->bh_work);
	WARN_ON(err);
	return err;
}

void cw1200_unregister_bh(struct cw1200_common *priv)
{
	bh_printk(KERN_DEBUG "[BH] unregister.\n");
	atomic_add(1, &priv->bh_term);
	wake_up(&priv->bh_wq);
	flush_workqueue(priv->bh_workqueue);
}

void cw1200_irq_handler(struct cw1200_common *priv)
{
	bh_printk(KERN_DEBUG "[BH] irq.\n");
	if (/* WARN_ON */(priv->bh_error))
		return;

	if (atomic_add_return(1, &priv->bh_rx) == 1)
		wake_up(&priv->bh_wq);
}

void cw1200_bh_wakeup(struct cw1200_common *priv)
{
	bh_printk(KERN_DEBUG "[BH] wakeup.\n");
	if (priv->bh_error) {
		cw1200_dbg(CW1200_DBG_ERROR, "[BH] wakeup failed (BH error)\n");
		return;
	}

	if (atomic_add_return(1, &priv->bh_tx) == 1)
		wake_up(&priv->bh_wq);
}

int cw1200_bh_suspend(struct cw1200_common *priv)
{
	bh_printk(KERN_DEBUG "[BH] suspend.\n");
	if (priv->bh_error) {
		wiphy_warn(priv->hw->wiphy, "BH error -- can't suspend\n");
		return -EINVAL;
	}

	atomic_set(&priv->bh_suspend, CW1200_BH_SUSPEND);
	wake_up(&priv->bh_wq);
	return wait_event_timeout(priv->bh_evt_wq, priv->bh_error ||
		(CW1200_BH_SUSPENDED == atomic_read(&priv->bh_suspend)),
		 1 * HZ) ? 0 : -ETIMEDOUT;
}

int cw1200_bh_resume(struct cw1200_common *priv)
{
	bh_printk(KERN_DEBUG "[BH] resume.\n");
	if (priv->bh_error) {
		wiphy_warn(priv->hw->wiphy, "BH error -- can't resume\n");
		return -EINVAL;
	}

	atomic_set(&priv->bh_suspend, CW1200_BH_RESUME);
	wake_up(&priv->bh_wq);
	return wait_event_timeout(priv->bh_evt_wq, priv->bh_error ||
		(CW1200_BH_RESUMED == atomic_read(&priv->bh_suspend)),
		1 * HZ) ? 0 : -ETIMEDOUT;
}

static inline void wsm_alloc_tx_buffer(struct cw1200_common *priv)
{
	++priv->hw_bufs_used;
}

int wsm_release_tx_buffer(struct cw1200_common *priv, int count)
{
	int ret = 0;
	int hw_bufs_used = priv->hw_bufs_used;

	priv->hw_bufs_used -= count;
	if (WARN_ON(priv->hw_bufs_used < 0))
		ret = -1;
	else if (hw_bufs_used >= priv->wsm_caps.numInpChBufs)
		ret = 1;
	if (!priv->hw_bufs_used)
		wake_up(&priv->bh_evt_wq);
	return ret;
}

static struct sk_buff *cw1200_get_skb(struct cw1200_common *priv, size_t len)
{
	struct sk_buff *skb;
	size_t alloc_len = (len > SDIO_BLOCK_SIZE) ? len : SDIO_BLOCK_SIZE;

	if (len > SDIO_BLOCK_SIZE || !priv->skb_cache) {
		skb = dev_alloc_skb(alloc_len
				+ WSM_TX_EXTRA_HEADROOM
				+ 8  /* TKIP IV */
				+ 12 /* TKIP ICV + MIC */
				- 2  /* Piggyback */);
		/* In AP mode RXed SKB can be looped back as a broadcast.
		 * Here we reserve enough space for headers. */
		if(skb)
			skb_reserve(skb, WSM_TX_EXTRA_HEADROOM
				+ 8 /* TKIP IV */
				- WSM_RX_EXTRA_HEADROOM);
		else
			printk(KERN_ERR
				"[BH] Failed to allocate skb in cw1200_get_skb.\n");
	} else {
		skb = priv->skb_cache;
		priv->skb_cache = NULL;
	}
	return skb;
}

static void cw1200_put_skb(struct cw1200_common *priv, struct sk_buff *skb)
{
	if (priv->skb_cache)
		dev_kfree_skb(skb);
	else
		priv->skb_cache = skb;
}

static int cw1200_bh_read_ctrl_reg(struct cw1200_common *priv,
					  u16 *ctrl_reg)
{
	int ret;

	ret = cw1200_reg_read_16(priv,
			ST90TDS_CONTROL_REG_ID, ctrl_reg);
	if (ret) {
		ret = cw1200_reg_read_16(priv,
				ST90TDS_CONTROL_REG_ID, ctrl_reg);
		if (ret)
			printk(KERN_ERR
				"[BH] Failed to read control register.\n");
	}

	return ret;
}

static int cw1200_device_wakeup(struct cw1200_common *priv)
{
	u16 ctrl_reg;
	int ret;

	bh_printk(KERN_DEBUG "[BH] Device wakeup.\n");

	/* To force the device to be always-on, the host sets WLAN_UP to 1 */
	ret = cw1200_reg_write_16(priv, ST90TDS_CONTROL_REG_ID,
			ST90TDS_CONT_WUP_BIT);
	if (WARN_ON(ret))
		return ret;

	ret = cw1200_bh_read_ctrl_reg(priv, &ctrl_reg);
	if (WARN_ON(ret))
		return ret;

	/* If the device returns WLAN_RDY as 1, the device is active and will
	 * remain active. */
	if (ctrl_reg & ST90TDS_CONT_RDY_BIT) {
		bh_printk(KERN_DEBUG "[BH] Device awake.\n");
		return 1;
	}

	return 0;
}

/* Must be called from BH thraed. */
void cw1200_enable_powersave(struct cw1200_common *priv,
			     bool enable)
{
	bh_printk(KERN_DEBUG "[BH] Powerave is %s.\n",
			enable ? "enabled" : "disabled");
	priv->powersave_enabled = enable;
}

void cw1200_bh_work(struct work_struct *work)
{
	struct cw1200_common *priv =
	container_of(work, struct cw1200_common, bh_work);
	bh_printk(KERN_DEBUG "[BH] %s enter\n", __func__);
	cw1200_bh(priv);
	bh_printk(KERN_DEBUG "[BH] %s exit\n", __func__);
}

static int cw1200_bh(void *arg)
{
	struct cw1200_common *priv = arg;
	struct sk_buff *skb_rx = NULL;
	size_t read_len = 0;
	int rx, tx, term, suspend;
	struct wsm_hdr *wsm;
	size_t wsm_len;
	int wsm_id;
	u8 wsm_seq;
	int rx_resync = 1;
	u16 ctrl_reg = 0;
	int tx_allowed;
	int pending_tx = 0;
	int tx_burst;
	int rx_burst = 0;
	long status;
#if defined(CONFIG_CW1200_WSM_DUMPS)
	size_t wsm_dump_max = -1;
#endif
	u32 dummy;

	for (;;) {
		if (!priv->hw_bufs_used
				&& priv->powersave_enabled
				&& !priv->device_can_sleep
				&& !atomic_read(&priv->recent_scan)) {
			status = 1 * HZ;
			bh_printk(KERN_DEBUG "[BH] Device wakedown. No data.\n");
			WARN_ON(cw1200_reg_write_16(priv,
						ST90TDS_CONTROL_REG_ID, 0));
			priv->device_can_sleep = true;
		} else if (priv->hw_bufs_used)
			/* Interrupt loss detection */
			status = 1 * HZ;
		else
			status = MAX_SCHEDULE_TIMEOUT;

		/* Dummy Read for SDIO retry mechanism*/
		if (((atomic_read(&priv->bh_rx) == 0) &&
				(atomic_read(&priv->bh_tx) == 0)))
			cw1200_reg_read(priv, ST90TDS_CONFIG_REG_ID,
					&dummy, sizeof(dummy));
#if defined(CONFIG_CW1200_WSM_DUMPS_SHORT)
		wsm_dump_max = priv->wsm_dump_max_size;
#endif /* CONFIG_CW1200_WSM_DUMPS_SHORT */

		bh_printk(KERN_DEBUG "[BH] waiting ...\n");
		status = wait_event_interruptible_timeout(priv->bh_wq, ({
				rx = atomic_xchg(&priv->bh_rx, 0);
				tx = atomic_xchg(&priv->bh_tx, 0);
				term = atomic_xchg(&priv->bh_term, 0);
				suspend = pending_tx ?
					0 : atomic_read(&priv->bh_suspend);
				(rx || tx || term || suspend || priv->bh_error);
			}), status);

		bh_printk(KERN_DEBUG "[BH] - rx: %d, tx: %d, term: %d, "
					"suspend: %d, status: %ld\n", rx, tx, term,
					suspend, status);

		if (status < 0 || term || priv->bh_error)
			break;

		if (!status && priv->hw_bufs_used) {
			unsigned long timestamp = jiffies;
			long timeout;
			bool pending = false;
			int i;

			wiphy_warn(priv->hw->wiphy, "Missed interrupt?\n");
			rx = 1;

			/* Get a timestamp of "oldest" frame */
			for (i = 0; i < 4; ++i)
				pending |= cw1200_queue_get_xmit_timestamp(
						&priv->tx_queue[i],
						&timestamp, priv->pending_frame_id);

			/* Check if frame transmission is timed out.
			 * Add an extra second with respect to possible
			 * interrupt loss. */
			timeout = timestamp +
					WSM_CMD_LAST_CHANCE_TIMEOUT +
					1 * HZ  -
					jiffies;

			/* And terminate BH tread if the frame is "stuck" */
			if (pending && timeout < 0) {
				wiphy_warn(priv->hw->wiphy,
					"Timeout waiting for TX confirm.\n");
				break;
			}

#if defined(CONFIG_CW1200_DUMP_ON_ERROR)
			BUG_ON(1);
#endif /* CONFIG_CW1200_DUMP_ON_ERROR */
		} else if (!status) {
			if (!priv->device_can_sleep
					&& !atomic_read(&priv->recent_scan)) {
				bh_printk(KERN_DEBUG "[BH] Device wakedown. Timeout.\n");
				WARN_ON(cw1200_reg_write_16(priv,
						ST90TDS_CONTROL_REG_ID, 0));
				priv->device_can_sleep = true;
			}
			continue;
		} else if (suspend) {
			bh_printk(KERN_DEBUG "[BH] Device suspend.\n");
			if (priv->powersave_enabled) {
				bh_printk(KERN_DEBUG "[BH] Device wakedown. Suspend.\n");
				WARN_ON(cw1200_reg_write_16(priv,
						ST90TDS_CONTROL_REG_ID, 0));
				priv->device_can_sleep = true;
			}

			atomic_set(&priv->bh_suspend, CW1200_BH_SUSPENDED);
			wake_up(&priv->bh_evt_wq);
			status = wait_event_interruptible(priv->bh_wq,
					CW1200_BH_RESUME == atomic_read(
						&priv->bh_suspend));
			if (status < 0) {
				wiphy_err(priv->hw->wiphy,
					"%s: Failed to wait for resume: %ld.\n",
					__func__, status);
				break;
			}
			bh_printk(KERN_DEBUG "[BH] Device resume.\n");
			atomic_set(&priv->bh_suspend, CW1200_BH_RESUMED);
			wake_up(&priv->bh_evt_wq);
			atomic_add(1, &priv->bh_rx);
			continue;
		}

		tx += pending_tx;
		pending_tx = 0;

		if (rx) {
			size_t alloc_len;
			u8 *data;

			if (WARN_ON(cw1200_bh_read_ctrl_reg(
					priv, &ctrl_reg)))
				break;
rx:
			read_len = (ctrl_reg & ST90TDS_CONT_NEXT_LEN_MASK) * 2;
			if (!read_len) {
				rx_burst = 0;
				goto tx;
			}

			if (WARN_ON((read_len < sizeof(struct wsm_hdr)) ||
					(read_len > EFFECTIVE_BUF_SIZE))) {
				printk(KERN_DEBUG "Invalid read len: %d",
					read_len);
				break;
			}

			/* Add SIZE of PIGGYBACK reg (CONTROL Reg)
			 * to the NEXT Message length + 2 Bytes for SKB */
			read_len = read_len + 2;

#if defined(CONFIG_CW1200_NON_POWER_OF_TWO_BLOCKSIZES)
			alloc_len = priv->sbus_ops->align_size(
					priv->sbus_priv, read_len);
#else /* CONFIG_CW1200_NON_POWER_OF_TWO_BLOCKSIZES */
			/* Platform's SDIO workaround */
			alloc_len = read_len & ~(SDIO_BLOCK_SIZE - 1);
			if (read_len & (SDIO_BLOCK_SIZE - 1))
				alloc_len += SDIO_BLOCK_SIZE;
#endif /* CONFIG_CW1200_NON_POWER_OF_TWO_BLOCKSIZES */

			/* Check if not exceeding CW1200 capabilities */
			if (WARN_ON_ONCE(alloc_len > EFFECTIVE_BUF_SIZE)) {
				printk(KERN_DEBUG "Read aligned len: %d\n",
					alloc_len);
			}

			skb_rx = cw1200_get_skb(priv, alloc_len);
			if (WARN_ON(!skb_rx))
				break;

			skb_trim(skb_rx, 0);
			skb_put(skb_rx, read_len);
			data = skb_rx->data;
			if (WARN_ON(!data))
				break;

			if (WARN_ON(cw1200_data_read(priv, data, alloc_len)))
				break;

			/* Piggyback */
			ctrl_reg = __le16_to_cpu(
				((__le16 *)data)[alloc_len / 2 - 1]);

			wsm = (struct wsm_hdr *)data;
			wsm_len = __le32_to_cpu(wsm->len);
			if (WARN_ON(wsm_len > read_len))
				break;

#if defined(CONFIG_CW1200_WSM_DUMPS)
			if (unlikely(priv->wsm_enable_wsm_dumps))
				print_hex_dump_bytes("<-- ",
					DUMP_PREFIX_NONE,
					data, min(wsm_len, wsm_dump_max));
#endif /* CONFIG_CW1200_WSM_DUMPS */

			wsm_id  = __le32_to_cpu(wsm->id) & 0xFFF;
			wsm_seq = (__le32_to_cpu(wsm->id) >> 13) & 7;

			skb_trim(skb_rx, wsm_len);

			if (unlikely(wsm_id == 0x0800)) {
				wsm_handle_exception(priv,
					 &data[sizeof(*wsm)],
					wsm_len - sizeof(*wsm));
				break;
			} else if (unlikely(!rx_resync)) {
				if (WARN_ON(wsm_seq != priv->wsm_rx_seq)) {
#if defined(CONFIG_CW1200_DUMP_ON_ERROR)
					BUG_ON(1);
#endif /* CONFIG_CW1200_DUMP_ON_ERROR */
					break;
				}
			}
			priv->wsm_rx_seq = (wsm_seq + 1) & 7;
			rx_resync = 0;

			if (wsm_id & 0x0400) {
				int rc = wsm_release_tx_buffer(priv, 1);
				if (WARN_ON(rc < 0))
					break;
				else if (rc > 0)
					tx = 1;
			}

			/* cw1200_wsm_rx takes care on SKB livetime */
			if (WARN_ON(wsm_handle_rx(priv, wsm_id, wsm, &skb_rx)))
				break;

			if (skb_rx) {
				cw1200_put_skb(priv, skb_rx);
				skb_rx = NULL;
			}

			read_len = 0;

			if (rx_burst) {
				cw1200_debug_rx_burst(priv);
				--rx_burst;
				goto rx;
			}
		}

tx:
		BUG_ON(priv->hw_bufs_used > priv->wsm_caps.numInpChBufs);
		tx_burst = priv->wsm_caps.numInpChBufs - priv->hw_bufs_used;
		tx_allowed = tx_burst > 0;

		if (tx && tx_allowed) {
			size_t tx_len;
			u8 *data;
			int ret;

			if (priv->device_can_sleep) {
				ret = cw1200_device_wakeup(priv);
				if (WARN_ON(ret < 0))
					break;
				else if (ret)
					priv->device_can_sleep = false;
				else {
					/* Wait for "awake" interrupt */
					pending_tx = tx;
					continue;
				}
			}

			wsm_alloc_tx_buffer(priv);
			ret = wsm_get_tx(priv, &data, &tx_len, &tx_burst);
			if (ret <= 0) {
				wsm_release_tx_buffer(priv, 1);
				if (WARN_ON(ret < 0))
					break;
			} else {
				wsm = (struct wsm_hdr *)data;
				BUG_ON(tx_len < sizeof(*wsm));
				BUG_ON(__le32_to_cpu(wsm->len) != tx_len);

#if 0 /* count is not implemented */
				if (ret > 1)
					atomic_add(1, &priv->bh_tx);
#else
				atomic_add(1, &priv->bh_tx);
#endif


#if defined(CONFIG_CW1200_NON_POWER_OF_TWO_BLOCKSIZES)
				tx_len = priv->sbus_ops->align_size(
						priv->sbus_priv, tx_len);
#else /* CONFIG_CW1200_NON_POWER_OF_TWO_BLOCKSIZES */
				/* HACK!!! Platform limitation.
				* It is also supported by upper layer:
				* there is always enough space at the
				* end of the buffer. */
				if (tx_len & (SDIO_BLOCK_SIZE - 1)) {
					tx_len &= ~(SDIO_BLOCK_SIZE - 1);
					tx_len += SDIO_BLOCK_SIZE;
				}
#endif /* CONFIG_CW1200_NON_POWER_OF_TWO_BLOCKSIZES */

				/* Check if not exceeding CW1200
				    capabilities */
				if (WARN_ON_ONCE(
				    tx_len > EFFECTIVE_BUF_SIZE)) {
					printk(KERN_DEBUG "Write aligned len:"
					" %d\n", tx_len);
				}

				wsm->id &= __cpu_to_le32(
						~WSM_TX_SEQ(WSM_TX_SEQ_MAX));
				wsm->id |= cpu_to_le32(
						WSM_TX_SEQ(priv->wsm_tx_seq));

				if (WARN_ON(cw1200_data_write(priv,
				    data, tx_len))) {
					wsm_release_tx_buffer(priv, 1);
					break;
				}

#if defined(CONFIG_CW1200_WSM_DUMPS)
				if (unlikely(priv->wsm_enable_wsm_dumps))
					print_hex_dump_bytes("--> ",
						DUMP_PREFIX_NONE,
						data,
						min(__le32_to_cpu(wsm->len),
						 wsm_dump_max));
#endif /* CONFIG_CW1200_WSM_DUMPS */

				wsm_txed(priv, data);
				priv->wsm_tx_seq = (priv->wsm_tx_seq + 1) &
						WSM_TX_SEQ_MAX;

				if (tx_burst > 1) {
					cw1200_debug_tx_burst(priv);
					++rx_burst;
					goto tx;
				}
			}
		}

		if (ctrl_reg & ST90TDS_CONT_NEXT_LEN_MASK)
			goto rx;
	}

	if (skb_rx) {
		cw1200_put_skb(priv, skb_rx);
		skb_rx = NULL;
	}


	if (!term) {
		cw1200_dbg(CW1200_DBG_ERROR, "[BH] Fatal error, exitting.\n");
#if defined(CONFIG_CW1200_DUMP_ON_ERROR)
		BUG_ON(1);
#endif /* CONFIG_CW1200_DUMP_ON_ERROR */
		priv->bh_error = 1;
#if defined(CONFIG_CW1200_USE_STE_EXTENSIONS)
		cw1200_dbg(CW1200_DBG_ERROR, "[BH] Call hang_notify\n");
		ieee80211_driver_hang_notify(priv->hw->wiphy, GFP_KERNEL);
		cw1200_pm_stay_awake(&priv->pm_state, 3*HZ);
#endif
		/* TODO: schedule_work(recovery) */
#ifndef HAS_PUT_TASK_STRUCT
		/* The only reason of having this stupid code here is
		 * that __put_task_struct is not exported by kernel. */
		for (;;) {
			int status = wait_event_interruptible(priv->bh_wq, ({
				term = atomic_xchg(&priv->bh_term, 0);
				(term);
				}));

			if (status || term)
				break;
		}
#endif
	}
	return 0;
}
