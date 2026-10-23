/* SPDX-License-Identifier: GPL-2.0-or-later */
#ifndef _ESP32_UART_H
#define _ESP32_UART_H

#include <linux/types.h>

struct esp32_uart_baud {
	u32 div;
	u32 frag;
	u32 sclk_div;
};

int esp32_uart_calc_baud(u32 uartclk, u32 baud, u32 clkdiv_mask, bool has_clkconf,
			 struct esp32_uart_baud *out);

#endif /* _ESP32_UART_H */
