// SPDX-License-Identifier: ISC
/*
 * Copyright (C) 2016 Felix Fietkau <nbd@nbd.name>
 */

#include <linux/dma-mapping.h>
#include "mt76.h"
#include "dma.h"

#if defined CONFIG_NET_MEDIATEK_SOC_WED

#define Q_READ(_q, _field) ({						\
	u32 _offset = offsetof(struct mt76_queue_regs, _field);		\
	u32 _val;							\
	if ((_q)->flags & MT_QFLAG_WED)					\
		_val = mtk_wed_device_reg_read((_q)->wed,		\
					       ((_q)->wed_regs +	\
					        _offset));		\
	else								\
		_val = readl(&(_q)->regs->_field);			\
	_val;								\
})

#define Q_WRITE(_q, _field, _val)	do {				\
	u32 _offset = offsetof(struct mt76_queue_regs, _field);		\
	if ((_q)->flags & MT_QFLAG_WED)					\
		mtk_wed_device_reg_write((_q)->wed,			\
					 ((_q)->wed_regs + _offset),	\
					 _val);				\
	else								\
		writel(_val, &(_q)->regs->_field);			\
} while (0)

#else

#define Q_READ(_q, _field)		readl(&(_q)->regs->_field)
#define Q_WRITE(_q, _field, _val)	writel(_val, &(_q)->regs->_field)

#endif

static struct mt76_txwi_cache *
mt76_alloc_rxwi(struct mt76_dev *dev)
{
	struct mt76_txwi_cache *t;

	t = kzalloc(L1_CACHE_ALIGN(sizeof(*t)), GFP_ATOMIC);
	if (!t)
		return NULL;

	t->ptr = NULL;
	return t;
}

static struct mt76_txwi_cache *
__mt76_get_rxwi(struct mt76_dev *dev)
{
	struct mt76_txwi_cache *t = NULL;

	spin_lock_bh(&dev->wed_lock);
	if (!list_empty(&dev->rxwi_cache)) {
		t = list_first_entry(&dev->rxwi_cache, struct mt76_txwi_cache,
				     list);
		list_del(&t->list);
	}
	spin_unlock_bh(&dev->wed_lock);

	return t;
}

struct mt76_txwi_cache *
mt76_get_rxwi(struct mt76_dev *dev)
{
	struct mt76_txwi_cache *t = __mt76_get_rxwi(dev);

	if (t)
		return t;

	return mt76_alloc_rxwi(dev);
}
EXPORT_SYMBOL_GPL(mt76_get_rxwi);

void
mt76_put_txwi(struct mt76_dev *dev, struct mt76_txwi_cache *t)
{
	if (!t)
		return;

	spin_lock(&dev->lock);
	list_add(&t->list, &dev->txwi_cache);
	spin_unlock(&dev->lock);
}
EXPORT_SYMBOL_GPL(mt76_put_txwi);

void
mt76_put_rxwi(struct mt76_dev *dev, struct mt76_txwi_cache *t)
{
	if (!t)
		return;

	spin_lock_bh(&dev->wed_lock);
	list_add(&t->list, &dev->rxwi_cache);
	spin_unlock_bh(&dev->wed_lock);
}
EXPORT_SYMBOL_GPL(mt76_put_rxwi);

static void
mt76_dma_sync_idx(struct mt76_dev *dev, struct mt76_queue *q)
{
	Q_WRITE(q, desc_base, q->desc_dma);
	if (q->flags & MT_QFLAG_WED_RRO_EN)
		Q_WRITE(q, ring_size, MT_DMA_RRO_EN | q->ndesc);
	else
		Q_WRITE(q, ring_size, q->ndesc);
	q->head = Q_READ(q, dma_idx);
	q->tail = q->head;
}

static void
__mt76_dma_queue_reset(struct mt76_dev *dev, struct mt76_queue *q,
		       bool reset_idx)
{
	if (!q || !q->ndesc)
		return;

	if (!mt76_queue_is_wed_rro_ind(q)) {
		int i;

		/* clear descriptors */
		for (i = 0; i < q->ndesc; i++)
			q->desc[i].ctrl = cpu_to_le32(MT_DMA_CTL_DMA_DONE);
	}

	if (reset_idx) {
		Q_WRITE(q, cpu_idx, 0);
		Q_WRITE(q, dma_idx, 0);
	}
	mt76_dma_sync_idx(dev, q);
}

static void
mt76_dma_queue_reset(struct mt76_dev *dev, struct mt76_queue *q)
{
	__mt76_dma_queue_reset(dev, q, true);
}

static int
mt76_dma_add_rx_buf(struct mt76_dev *dev, struct mt76_queue *q,
		    struct mt76_queue_buf *buf, void *data)
{
	struct mt76_queue_entry *entry = &q->entry[q->head];
	struct mt76_txwi_cache *txwi = NULL;
	struct mt76_desc *desc;
	int idx = q->head;
	u32 buf1 = 0, ctrl;
	int rx_token;

	if (mt76_queue_is_wed_rro_ind(q)) {
		struct mt76_wed_rro_desc *rro_desc;

		rro_desc = (struct mt76_wed_rro_desc *)q->desc;
		data = &rro_desc[q->head];
		goto done;
	}

	desc = &q->desc[q->head];
	ctrl = FIELD_PREP(MT_DMA_CTL_SD_LEN0, buf[0].len);
#ifdef CONFIG_ARCH_DMA_ADDR_T_64BIT
	buf1 = FIELD_PREP(MT_DMA_CTL_SDP0_H, buf->addr >> 32);
#endif

	if (mt76_queue_is_wed_rx(q)) {
		txwi = mt76_get_rxwi(dev);
		if (!txwi)
			return -ENOMEM;

		rx_token = mt76_rx_token_consume(dev, data, txwi, buf->addr);
		if (rx_token < 0) {
			mt76_put_rxwi(dev, txwi);
			return -ENOMEM;
		}

		buf1 |= FIELD_PREP(MT_DMA_CTL_TOKEN, rx_token);
		ctrl |= MT_DMA_CTL_TO_HOST;
	}

	WRITE_ONCE(desc->buf0, cpu_to_le32(buf->addr));
	WRITE_ONCE(desc->buf1, cpu_to_le32(buf1));
	WRITE_ONCE(desc->ctrl, cpu_to_le32(ctrl));
	WRITE_ONCE(desc->info, 0);

done:
	entry->dma_addr[0] = buf->addr;
	entry->dma_len[0] = buf->len;
	entry->txwi = txwi;
	entry->buf = data;
	entry->wcid = 0xffff;
	entry->skip_buf1 = true;
	q->head = (q->head + 1) % q->ndesc;
	q->queued++;

	return idx;
}

static void
mt76_dma_kick_queue(struct mt76_dev *dev, struct mt76_queue *q)
{
	wmb();
	Q_WRITE(q, cpu_idx, q->head);
}

static int
mt76_dma_rx_fill(struct mt76_dev *dev, struct mt76_queue *q,
		 bool allow_direct)
{
	int len = SKB_WITH_OVERHEAD(q->buf_size);
	int frames = 0;

	if (!q->ndesc)
		return 0;

	spin_lock_bh(&q->lock);

	while (q->queued < q->ndesc - 1) {
		struct mt76_queue_buf qbuf = {};
		enum dma_data_direction dir;
		dma_addr_t addr;
		int offset;
		void *buf = NULL;

		if (mt76_queue_is_wed_rro_ind(q))
			goto done;

		buf = mt76_get_page_pool_buf(q, &offset, q->buf_size);
		if (!buf)
			break;

		addr = page_pool_get_dma_addr(virt_to_head_page(buf)) + offset;
		dir = page_pool_get_dma_dir(q->page_pool);
		dma_sync_single_for_device(dev->dma_dev, addr, len, dir);

		qbuf.addr = addr + q->buf_offset;
done:
		qbuf.len = len - q->buf_offset;
		qbuf.skip_unmap = false;
		if (mt76_dma_add_rx_buf(dev, q, &qbuf, buf) < 0) {
			mt76_put_page_pool_buf(buf, allow_direct);
			break;
		}
		frames++;
	}

	if (frames || mt76_queue_is_wed_rx(q))
		mt76_dma_kick_queue(dev, q);

	spin_unlock_bh(&q->lock);

	return frames;
}

int mt76_dma_wed_setup(struct mt76_dev *dev, struct mt76_queue *q, bool reset)
{
#ifdef CONFIG_NET_MEDIATEK_SOC_WED
	int ret = 0, type, ring;
	u16 flags;

	if (!q || !q->ndesc)
		return -EINVAL;

	flags = q->flags;
	if (!q->wed || !mtk_wed_device_active(q->wed))
		q->flags &= ~MT_QFLAG_WED;

	if (!(q->flags & MT_QFLAG_WED))
		return 0;

	type = FIELD_GET(MT_QFLAG_WED_TYPE, q->flags);
	ring = FIELD_GET(MT_QFLAG_WED_RING, q->flags);

	switch (type) {
	case MT76_WED_Q_TX:
		ret = mtk_wed_device_tx_ring_setup(q->wed, ring, q->regs,
						   reset);
		if (!ret)
			q->wed_regs = q->wed->tx_ring[ring].reg_base;
		break;
	case MT76_WED_Q_TXFREE:
		/* WED txfree queue needs ring to be initialized before setup */
		q->flags = 0;
		mt76_dma_queue_reset(dev, q);
		mt76_dma_rx_fill(dev, q, false);

		ret = mtk_wed_device_txfree_ring_setup(q->wed, q->regs);
		if (!ret)
			q->wed_regs = q->wed->txfree_ring.reg_base;
		break;
	case MT76_WED_Q_RX:
		ret = mtk_wed_device_rx_ring_setup(q->wed, ring, q->regs,
						   reset);
		if (!ret)
			q->wed_regs = q->wed->rx_ring[ring].reg_base;
		break;
	case MT76_WED_RRO_Q_DATA:
		q->flags &= ~MT_QFLAG_WED;
		__mt76_dma_queue_reset(dev, q, false);
		mtk_wed_device_rro_rx_ring_setup(q->wed, ring, q->regs);
		q->head = q->ndesc - 1;
		q->queued = q->head;
		break;
	case MT76_WED_RRO_Q_MSDU_PG:
		q->flags &= ~MT_QFLAG_WED;
		__mt76_dma_queue_reset(dev, q, false);
		mtk_wed_device_msdu_pg_rx_ring_setup(q->wed, ring, q->regs);
		q->head = q->ndesc - 1;
		q->queued = q->head;
		break;
	case MT76_WED_RRO_Q_IND:
		q->flags &= ~MT_QFLAG_WED;
		mt76_dma_queue_reset(dev, q);
		mt76_dma_rx_fill(dev, q, false);
		mtk_wed_device_ind_rx_ring_setup(q->wed, q->regs);
		break;
	default:
		ret = -EINVAL;
		break;
	}
	q->flags = flags;

	return ret;
#else
	return 0;
#endif
}
EXPORT_SYMBOL_GPL(mt76_dma_wed_setup);
