/* SPDX-License-Identifier: (GPL-2.0 OR MIT) */
/* Copyright (c) 2026 Espressif Systems (Shanghai) Co. Ltd. */

#ifndef ESP32S31_CLOCK_H
#define ESP32S31_CLOCK_H

/*
 * Ids are dense from zero and index the provider's descriptor table directly.
 * New clocks append at the next free id: they are devicetree ABI, so an id may
 * not be renumbered or reused for a different clock once a dtb references it.
 */

/* IO_MUX APB bus clock (HP_SYS_CLKRST IOMUX_CTRL0) */
#define ESP_S31_CLK_IOMUX_APB	0
/* IO_MUX function clock (HP_SYS_CLKRST IOMUX_CTRL0) */
#define ESP_S31_CLK_IOMUX	1
/* I2C0 APB bus clock (HP_SYS_CLKRST I2C0_CTRL0) */
#define ESP_S31_CLK_I2C0_APB	2
/* I2C0 module clock, source mux and divider (HP_SYS_CLKRST I2C0_CTRL0) */
#define ESP_S31_CLK_I2C0	3
/* I2C1 APB bus clock (HP_SYS_CLKRST I2C1_CTRL0) */
#define ESP_S31_CLK_I2C1_APB	4
/* I2C1 module clock, source mux and divider (HP_SYS_CLKRST I2C1_CTRL0) */
#define ESP_S31_CLK_I2C1	5
/* EMAC system clock (HP_SYS_CLKRST EMAC_CTRL0) */
#define ESP_S31_CLK_EMAC_SYS	6
/* EMAC RGMII reference clock, source mux, gate and divider (CNNT_SYS HP_EMAC_REF_CTRL) */
#define ESP_S31_CLK_EMAC_REF	7
/* USB OTG-HS APB and system clocks (HP_SYS_CLKRST USB_OTGHS_CTRL0) */
#define ESP_S31_CLK_USB_OTGHS_BUS	8
/* USB OTG-HS PHY reference clock (CNNT_SYS USB_OTG20_CTRL) */
#define ESP_S31_CLK_USB_OTGHS_PHYREF	9
/* EMAC PTP reference clock (CNNT_SYS HP_EMAC_PTP_CTRL) */
#define ESP_S31_CLK_EMAC_PTP	10

#define ESP_S31_CLK_NUM		11

#endif /* ESP32S31_CLOCK_H */
