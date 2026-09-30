/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Shared between pinctrl-esp and gpio-esp32s31.
 *
 * Copyright (C) 2026 Espressif Systems (Shanghai) Co. Ltd.
 */

#ifndef __SOC_ESPRESSIF_ESP_GPIO_H
#define __SOC_ESPRESSIF_ESP_GPIO_H

#include <linux/spinlock_types.h>

/*
 * Serialises read-modify-write of GPIO_PINn_REG, which holds both the
 * open-drain bit pinctrl-esp writes and the INT_TYPE/INT_ENA fields
 * gpio-esp32s31 writes.
 */
extern raw_spinlock_t esp_gpio_pin_reg_lock;

#endif /* __SOC_ESPRESSIF_ESP_GPIO_H */
