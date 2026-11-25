/* SPDX-License-Identifier: GPL-2.0-only */
#ifndef _LINUX_SOC_ESP_RMT_H
#define _LINUX_SOC_ESP_RMT_H

#include <linux/types.h>

struct device;
struct esp_rmt_channel;

/* One 48-word hardware block, including the mandatory EOF symbol. */
#define ESP_RMT_TX_MAX_SYMBOLS 47

struct esp_rmt_channel *devm_esp_rmt_tx_get(struct device *dev, u32 channel);
int esp_rmt_tx(struct esp_rmt_channel *chan, unsigned long tick_hz,
	       const u32 *symbols, size_t count);

#endif
