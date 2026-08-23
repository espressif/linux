/* SPDX-License-Identifier: (GPL-2.0 OR MIT) */
/* Copyright (c) 2026 Espressif Systems (Shanghai) Co. Ltd. */

#ifndef ESP32S31_RESET_H
#define ESP32S31_RESET_H

/*
 * Ids are dense from zero and index the provider's descriptor table directly.
 * New resets append at the next free id: they are devicetree ABI, so an id may
 * not be renumbered or reused for a different reset once a dtb references it.
 */

/* IO_MUX reset (HP_SYS_CLKRST IOMUX_CTRL0) */
#define ESP_S31_RST_IOMUX	0
/* I2C0 reset (HP_SYS_CLKRST I2C0_CTRL0) */
#define ESP_S31_RST_I2C0	1
/* I2C1 reset (HP_SYS_CLKRST I2C1_CTRL0) */
#define ESP_S31_RST_I2C1	2

#define ESP_S31_RST_NUM		3

#endif /* ESP32S31_RESET_H */
