// SPDX-License-Identifier: ISC
/*
 * Copyright (C) 2018 Felix Fietkau <nbd@nbd.name>
 */
#include "mt76.h"

static unsigned long mt76_aggr_tid_to_timeo(u8 tidno)
{
	/* Currently voice traffic (AC_VO) always runs without aggregation,
	 * no special handling is needed. AC_BE/AC_BK use tids 0-3. Just check
	 * for non AC_BK/AC_BE and set smaller timeout for it. */
	return HZ / (tidno >= 4 ? 25 : 10);
}

static void
mt76_aggr_release(struct mt76_rx_tid *tid, struct sk_buff_head *frames, int idx)
{
	struct sk_buff *skb;

	// TODO: ieee80211_sn_inc must be implemented on Windows
	tid->head = ieee80211_sn_inc(tid->head);

	skb = tid->reorder_buf[idx];
	if (!skb)
		return;

	tid->reorder_buf[idx] = NULL;
	tid->nframes--;

	// TODO: __skb_queue_tail must be implemented on Windows
	__skb_queue_tail(frames, skb);
}

static void
mt76_rx_aggr_release_frames(struct mt76_rx_tid *tid,
			    struct sk_buff_head *frames,
			    u16 head)
{
	int idx;

	// TODO: ieee80211_sn_less must be implemented on Windows
	while (ieee80211_sn_less(tid->head, head)) {
		idx = tid->head % tid->size;
		mt76_aggr_release(tid, frames, idx);
	}
}

static void
mt76_rx_aggr_release_head(struct mt76_rx_tid *tid, struct sk_buff_head *frames)
{
	int idx = tid->head % tid->size;

	while (tid->reorder_buf[idx]) {
		mt76_aggr_release(tid, frames, idx);
		idx = tid->head % tid->size;
	}
}

static void
mt76_rx_aggr_check_release(struct mt76_rx_tid *tid, struct sk_buff_head *frames)
{
	struct mt76_rx_status *status;
	struct sk_buff *skb;
	int start, idx, nframes;

	if (!tid->nframes)
		return;

	mt76_rx_aggr_release_head(tid, frames);

	start = tid->head % tid->size;
	nframes = tid->nframes;

	for (idx = (tid->head + 1) % tid->size;
	     idx != start && nframes;
	     idx = (idx + 1) % tid->size) {
		skb = tid->reorder_buf[idx];
		if (!skb)
			continue;

		nframes--;
		status = (struct mt76_rx_status *)skb->cb;
		// TODO: time_after32 must be implemented on Windows
		if (!time_after32(jiffies,
				  status->reorder_time +
				  mt76_aggr_tid_to_timeo(tid->num)))
			continue;

		mt76_rx_aggr_release_frames(tid, frames, status->seqno);
	}

	mt76_rx_aggr_release_head(tid, frames);
}

static void
mt76_rx_aggr_reorder_work(struct work_struct *work)
{
	// TODO: container_of must be implemented on Windows
	struct mt76_rx_tid *tid = container_of(work, struct mt76_rx_tid,
					       reorder_work.work);
	struct mt76_dev *dev = tid->dev;
	struct sk_buff_head frames;
	int nframes;

	// TODO: __skb_queue_head_init must be implemented on Windows
	__skb_queue_head_init(&frames);

	// TODO: local_bh_disable must be implemented on Windows
	local_bh_disable();
	// TODO: rcu_read_lock must be implemented on Windows
	rcu_read_lock();

	// TODO: spin_lock must be implemented on Windows
	spin_lock(&tid->lock);
	mt76_rx_aggr_check_release(tid, &frames);
	nframes = tid->nframes;
	// TODO: spin_unlock must be implemented on Windows
	spin_unlock(&tid->lock);

	if (nframes)
		// TODO: ieee80211_queue_delayed_work must be implemented on Windows
		ieee80211_queue_delayed_work(tid->dev->hw, &tid->reorder_work,
					     mt76_aggr_tid_to_timeo(tid->num));
	mt76_rx_complete(dev, &frames, NULL);

	// TODO: rcu_read_unlock must be implemented on Windows
	rcu_read_unlock();
	// TODO: local_bh_enable must be implemented on Windows
	local_bh_enable();
}

static void
mt76_rx_aggr_check_ctl(struct sk_buff *skb, struct sk_buff_head *frames)
{
	struct mt76_rx_status *status = (struct mt76_rx_status *)skb->cb;
	struct ieee80211_bar *bar = mt76_skb_get_hdr(skb);
	struct mt76_wcid *wcid = status->wcid;
	struct mt76_rx_tid *tid;
	u8 tidno = status->qos_ctl & IEEE80211_QOS_CTL_TID_MASK;
	u16 seqno;

	// TODO: ieee80211_is_ctl must be implemented on Windows
	if (!ieee80211_is_ctl(bar->frame_control))
		return;

	// TODO: ieee80211_is_back_req must be implemented on Windows
	if (!ieee80211_is_back_req(bar->frame_control))
		return;

	// TODO: le16_to_cpu must be implemented on Windows
	status->qos_ctl = tidno = le16_to_cpu(bar->control) >> 12;
	// TODO: IEEE80211_SEQ_TO_SN must be implemented on Windows
	seqno = IEEE80211_SEQ_TO_SN(le16_to_cpu(bar->start_seq_num));
	// TODO: rcu_dereference must be implemented on Windows
	tid = rcu_dereference(wcid->aggr[tidno]);
	if (!tid)
		return;

	// TODO: spin_lock_bh must be implemented on Windows
	spin_lock_bh(&tid->lock);
	if (!tid->stopped) {
		mt76_rx_aggr_release_frames(tid, frames, seqno);
		mt76_rx_aggr_release_head(tid, frames);
	}
	// TODO: spin_unlock_bh must be implemented on Windows
	spin_unlock_bh(&tid->lock);
}

void mt76_rx_aggr_reorder(struct sk_buff *skb, struct sk_buff_head *frames)
{
	struct mt76_rx_status *status = (struct mt76_rx_status *)skb->cb;
	struct mt76_wcid *wcid = status->wcid;
	struct ieee80211_sta *sta;
	struct mt76_rx_tid *tid;
	bool sn_less;
	u16 seqno, head, size, idx;
	u8 tidno = status->qos_ctl & IEEE80211_QOS_CTL_TID_MASK;
	u8 ackp;

	// TODO: __skb_queue_tail must be implemented on Windows
	__skb_queue_tail(frames, skb);

	// TODO: wcid_to_sta must be implemented on Windows
	sta = wcid_to_sta(wcid);
	if (!sta)
		return;

	if (!status->aggr) {
		if (!(status->flag & RX_FLAG_8023))
			mt76_rx_aggr_check_ctl(skb, frames);
		return;
	}

	/* not part of a BA session */
	ackp = status->qos_ctl & IEEE80211_QOS_CTL_ACK_POLICY_MASK;
	if (ackp == IEEE80211_QOS_CTL_ACK_POLICY_NOACK)
		return;

	// TODO: rcu_dereference must be implemented on Windows
	tid = rcu_dereference(wcid->aggr[tidno]);
	if (!tid)
		return;

	status->flag |= RX_FLAG_DUP_VALIDATED;
	// TODO: spin_lock_bh must be implemented on Windows
	spin_lock_bh(&tid->lock);

	if (tid->stopped)
		goto out;

	head = tid->head;
	seqno = status->seqno;
	size = tid->size;
	// TODO: ieee80211_sn_less must be implemented on Windows
	sn_less = ieee80211_sn_less(seqno, head);

	if (!tid->started) {
		if (sn_less)
			goto out;

		tid->started = true;
	}

	if (sn_less) {
		// TODO: __skb_unlink must be implemented on Windows
		__skb_unlink(skb, frames);
		// TODO: dev_kfree_skb must be implemented on Windows
		dev_kfree_skb(skb);
		goto out;
	}

	if (seqno == head) {
		// TODO: ieee80211_sn_inc must be implemented on Windows
		tid->head = ieee80211_sn_inc(head);
		if (tid->nframes)
			mt76_rx_aggr_release_head(tid, frames);
		goto out;
	}

	// TODO: __skb_unlink must be implemented on Windows
	__skb_unlink(skb, frames);

	/*
	 * Frame sequence number exceeds buffering window, free up some space
	 * by releasing previous frames
	 */
	if (!ieee80211_sn_less(seqno, head + size)) {
		// TODO: ieee80211_sn_inc must be implemented on Windows
		// TODO: ieee80211_sn_sub must be implemented on Windows
		head = ieee80211_sn_inc(ieee80211_sn_sub(seqno, size));
		mt76_rx_aggr_release_frames(tid, frames, head);
	}

	idx = seqno % size;

	/* Discard if the current slot is already in use */
	if (tid->reorder_buf[idx]) {
		// TODO: dev_kfree_skb must be implemented on Windows
		dev_kfree_skb(skb);
		goto out;
	}

	status->reorder_time = jiffies;
	tid->reorder_buf[idx] = skb;
	tid->nframes++;
	mt76_rx_aggr_release_head(tid, frames);

	// TODO: ieee80211_queue_delayed_work must be implemented on Windows
	ieee80211_queue_delayed_work(tid->dev->hw, &tid->reorder_work,
				     mt76_aggr_tid_to_timeo(tid->num));

out:
	// TODO: spin_unlock_bh must be implemented on Windows
	spin_unlock_bh(&tid->lock);
}

int mt76_rx_aggr_start(struct mt76_dev *dev, struct mt76_wcid *wcid, u8 tidno,
		       u16 ssn, u16 size)
{
	struct mt76_rx_tid *tid;

	mt76_rx_aggr_stop(dev, wcid, tidno);

	// TODO: kzalloc must be implemented on Windows
	tid = kzalloc(struct_size(tid, reorder_buf, size), GFP_KERNEL);
	if (!tid)
		return -ENOMEM;

	tid->dev = dev;
	tid->head = ssn;
	tid->size = size;
	tid->num = tidno;
	// TODO: INIT_DELAYED_WORK must be implemented on Windows
	INIT_DELAYED_WORK(&tid->reorder_work, mt76_rx_aggr_reorder_work);
	// TODO: spin_lock_init must be implemented on Windows
	spin_lock_init(&tid->lock);

	// TODO: rcu_assign_pointer must be implemented on Windows
	rcu_assign_pointer(wcid->aggr[tidno], tid);

	return 0;
}
EXPORT_SYMBOL_GPL(mt76_rx_aggr_start);

static void mt76_rx_aggr_shutdown(struct mt76_dev *dev, struct mt76_rx_tid *tid)
{
	u16 size = tid->size;
	int i;

	// TODO: spin_lock_bh must be implemented on Windows
	spin_lock_bh(&tid->lock);

	tid->stopped = true;
	for (i = 0; tid->nframes && i < size; i++) {
		struct sk_buff *skb = tid->reorder_buf[i];

		if (!skb)
			continue;

		tid->reorder_buf[i] = NULL;
		tid->nframes--;
		// TODO: dev_kfree_skb must be implemented on Windows
		dev_kfree_skb(skb);
	}

	// TODO: spin_unlock_bh must be implemented on Windows
	spin_unlock_bh(&tid->lock);

	// TODO: cancel_delayed_work_sync must be implemented on Windows
	cancel_delayed_work_sync(&tid->reorder_work);
}

void mt76_rx_aggr_stop(struct mt76_dev *dev, struct mt76_wcid *wcid, u8 tidno)
{
	struct mt76_rx_tid *tid = NULL;

	// TODO: rcu_replace_pointer must be implemented on Windows
	tid = rcu_replace_pointer(wcid->aggr[tidno], tid,
		// TODO: lockdep_is_held must be implemented on Windows
				  lockdep_is_held(&dev->mutex));
	if (tid) {
		mt76_rx_aggr_shutdown(dev, tid);
		// TODO: kfree_rcu must be implemented on Windows
		kfree_rcu(tid, rcu_head);
	}
}
EXPORT_SYMBOL_GPL(mt76_rx_aggr_stop);
