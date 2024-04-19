// SPDX-License-Identifier: ISC
/*
 * Copyright (C) 2016 Felix Fietkau <nbd@nbd.name>
 */

#include <linux/module.h>
#include "mt76.h"

bool __mt76_poll(struct mt76_dev *dev, u32 offset, u32 mask, u32 val,
		 int timeout)
{
	u32 cur;

	timeout /= 10;
	do {
		cur = __mt76_rr(dev, offset) & mask;
		if (cur == val)
			return true;

		udelay(10);
	} while (timeout-- > 0);

	return false;
}
EXPORT_SYMBOL_GPL(__mt76_poll);

bool ____mt76_poll_msec(struct mt76_dev *dev, u32 offset, u32 mask, u32 val,
			int timeout, int tick)
{
	u32 cur;

	timeout /= tick;
	do {
		cur = __mt76_rr(dev, offset) & mask;
		if (cur == val)
			return true;

		usleep_range(1000 * tick, 2000 * tick);
	} while (timeout-- > 0);

	return false;
}
EXPORT_SYMBOL_GPL(____mt76_poll_msec);

int mt76_wcid_alloc(u32 *mask, int size)
{
	int i, idx = 0, cur;

	for (i = 0; i < DIV_ROUND_UP(size, 32); i++) {
		idx = ffs(~mask[i]);
		if (!idx)
			continue;

		idx--;
		cur = i * 32 + idx;
		if (cur >= size)
			break;

		mask[i] |= BIT(idx);
		return cur;
	}

	return -1;
}
EXPORT_SYMBOL_GPL(mt76_wcid_alloc);

int __mt76_worker_fn(void *ptr)
{
	struct mt76_worker *w = ptr;

	while (!kthread_should_stop()) {
		set_current_state(TASK_INTERRUPTIBLE);

		if (kthread_should_park()) {
			kthread_parkme();
			continue;
		}

		if (!test_and_clear_bit(MT76_WORKER_SCHEDULED, &w->state)) {
			schedule();
			continue;
		}

		set_bit(MT76_WORKER_RUNNING, &w->state);
		set_current_state(TASK_RUNNING);
		w->fn(w);
		cond_resched();
		clear_bit(MT76_WORKER_RUNNING, &w->state);
	}

	return 0;
}
EXPORT_SYMBOL_GPL(__mt76_worker_fn);

MODULE_DESCRIPTION("MediaTek MT76x helpers");
MODULE_LICENSE("Dual BSD/GPL");
