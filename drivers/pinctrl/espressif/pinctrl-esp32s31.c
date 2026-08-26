// SPDX-License-Identifier: GPL-2.0
/*
 * ESP32-S31 SoC data for the Espressif pinctrl driver.
 *
 * Copyright (C) 2026 Espressif Systems (Shanghai) Co. Ltd.
 *
 * Every pad has one IO_MUX register selecting one of eight functions, plus the
 * pad settings (input buffer, pulls, drive strength) in the same word. Function
 * 1 is the GPIO matrix, a crossbar that reaches any peripheral signal from any
 * pad, so most functions here are a matrix routing rather than a direct mux.
 */

#include <linux/errno.h>
#include <dt-bindings/pinctrl/esp32s31-iomux.h>
#include "pinctrl-esp.h"

/*
 * 54 user-available HP GPIO pins: 0-25, 33-40, 42-61. GPIO 26-32 are
 * dedicated to external flash, GPIO 29/41/62 are reserved (not bonded out).
 * pin_count must still equal the highest GPIO number + 1 so that per-pin
 * register addressing (pin * stride) works for every valid pin.
 */
#define S31_PIN_COUNT			62

/*
 * IO_MUX pad register: one per pad, in GPIO order, from the start of the
 * window. The field positions differ from the other SoCs in this family --
 * they are not interchangeable.
 */
#define S31_MUX_MODE_MASK		(0x7U << 12)
#define S31_FUN_DRV_M			(0x3U << 10)
#define S31_FUN_IE			BIT(9)
#define S31_FUN_PU			BIT(8)
#define S31_FUN_PD			BIT(7)

/* Registers in the GPIO matrix window, which the GPIO controller owns. */
#define S31_GPIO_PIN_REG_BASE		0xF4
#define S31_GPIO_PIN_PAD_DRIVER		BIT(2)

#define S31_MATRIX_IN_SEL_CFG_OFFSET	0x2F4
#define S31_MATRIX_OUT_SEL_CFG_OFFSET	0xAF4
#define S31_MATRIX_SIG_IN_SEL		BIT(9)

/* In the USB-Serial/JTAG block, reached through its syscon. */
#define S31_USB_JTAG_CONF0		0x18
#define S31_USB_JTAG_PAD_ENABLE		BIT(14)
#define S31_USB_PHY0_DM_GPIO		33
#define S31_USB_PHY0_DP_GPIO		34

/*
 * Only user-available pins are registered with pinctrl. GPIO 26-32 (flash),
 * 29 (reserved), 41 (reserved) and 62 (reserved) are omitted.
 */
static const struct pinctrl_pin_desc esp32s31_pins[] = {
	ESP_PIN_DESC(0),  ESP_PIN_DESC(1),  ESP_PIN_DESC(2),  ESP_PIN_DESC(3),
	ESP_PIN_DESC(4),  ESP_PIN_DESC(5),  ESP_PIN_DESC(6),  ESP_PIN_DESC(7),
	ESP_PIN_DESC(8),  ESP_PIN_DESC(9),  ESP_PIN_DESC(10), ESP_PIN_DESC(11),
	ESP_PIN_DESC(12), ESP_PIN_DESC(13), ESP_PIN_DESC(14), ESP_PIN_DESC(15),
	ESP_PIN_DESC(16), ESP_PIN_DESC(17), ESP_PIN_DESC(18), ESP_PIN_DESC(19),
	ESP_PIN_DESC(20), ESP_PIN_DESC(21), ESP_PIN_DESC(22), ESP_PIN_DESC(23),
	ESP_PIN_DESC(24), ESP_PIN_DESC(25),
	/* GPIO 26-32: flash-dedicated, omitted */
	ESP_PIN_DESC(33), ESP_PIN_DESC(34), ESP_PIN_DESC(35),
	ESP_PIN_DESC(36), ESP_PIN_DESC(37), ESP_PIN_DESC(38), ESP_PIN_DESC(39),
	ESP_PIN_DESC(40),
	/* GPIO 41: reserved (not bonded), omitted */
	ESP_PIN_DESC(42), ESP_PIN_DESC(43),
	ESP_PIN_DESC(44), ESP_PIN_DESC(45), ESP_PIN_DESC(46), ESP_PIN_DESC(47),
	ESP_PIN_DESC(48), ESP_PIN_DESC(49), ESP_PIN_DESC(50), ESP_PIN_DESC(51),
	ESP_PIN_DESC(52), ESP_PIN_DESC(53), ESP_PIN_DESC(54), ESP_PIN_DESC(55),
	ESP_PIN_DESC(56), ESP_PIN_DESC(57), ESP_PIN_DESC(58), ESP_PIN_DESC(59),
	ESP_PIN_DESC(60), ESP_PIN_DESC(61),
};

/*
 * The GPIO matrix window is owned by the GPIO controller. A gpio-ranges
 * dependency means the GPIO driver waits for pinctrl, so pinctrl cannot also
 * wait for the GPIO driver's regmap. The window is mapped directly without
 * claiming, so both drivers can reach their respective register subsets.
 * Concurrent RMW on GPIO_PINn_REG is serialized by esp_gpio_pin_reg_lock.
 */
static const struct esp_region esp32s31_regions[] = {
	{ .id = ESP_REGMAP_MUX,    .dt_name = "iomux" },
	{ .id = ESP_REGMAP_MATRIX, .dt_name = "matrix", .shared = true },
	/* Without it GPIO33/34 are left to USB-Serial/JTAG. */
	{ .id = ESP_REGMAP_USB_JTAG, .syscon = "esp,usb-jtag", .optional = true },
};

static const struct esp_mux_layer esp32s31_layers[ESP_FIELD_MAX] = {
	[ESP_FIELD_FUNC]       = { ESP_REGMAP_MUX, 0, 4, S31_MUX_MODE_MASK },
	[ESP_FIELD_IE]         = { ESP_REGMAP_MUX, 0, 4, S31_FUN_IE },
	[ESP_FIELD_PULL_UP]    = { ESP_REGMAP_MUX, 0, 4, S31_FUN_PU },
	[ESP_FIELD_PULL_DOWN]  = { ESP_REGMAP_MUX, 0, 4, S31_FUN_PD },
	[ESP_FIELD_DRIVE]      = { ESP_REGMAP_MUX, 0, 4, S31_FUN_DRV_M },
	[ESP_FIELD_OPEN_DRAIN] = { ESP_REGMAP_MATRIX, S31_GPIO_PIN_REG_BASE, 4,
				   S31_GPIO_PIN_PAD_DRIVER },
};

/* FUN_DRV register encoding to milliamps (TRM §1.20.2, IO_MUX_GPIOn_FUN_DRV). */
static const unsigned int esp32s31_drv_mA[ESP_DRV_LEVELS_MAX] = { 5, 10, 20, 40 };

/*
 * Function table, indexed by the function id a pinmux cell carries. Which pad
 * carries a signal is a board fact and stays in the dts; only the routing a
 * function needs is described here.
 */
static const struct esp_pin_function esp32s31_functions[] = {
	[ESP_FUNC_S31_I2C0_SCL_ID] = { GPIO_FUNC_GPIO, I2C0_SCL_PAD_IN_IDX,
				       I2C0_SCL_PAD_OUT_IDX },
	[ESP_FUNC_S31_I2C0_SDA_ID] = { GPIO_FUNC_GPIO, I2C0_SDA_PAD_IN_IDX,
				       I2C0_SDA_PAD_OUT_IDX },
	[ESP_FUNC_S31_I2C1_SCL_ID] = { GPIO_FUNC_GPIO, I2C1_SCL_PAD_IN_IDX,
				       I2C1_SCL_PAD_OUT_IDX },
	[ESP_FUNC_S31_I2C1_SDA_ID] = { GPIO_FUNC_GPIO, I2C1_SDA_PAD_IN_IDX,
				       I2C1_SDA_PAD_OUT_IDX },
};

static int esp_s31_matrix_set_in(struct esp_pinctrl *epctl, u32 gpio, u32 signal)
{
	if (!epctl->regs[ESP_REGMAP_MATRIX])
		return -ENODEV;

	esp_region_write(epctl, ESP_REGMAP_MATRIX,
			 S31_MATRIX_IN_SEL_CFG_OFFSET + signal * 4,
			 gpio | S31_MATRIX_SIG_IN_SEL);
	dev_dbg(epctl->dev, "s31 matrix-in: signal %u <- gpio %u\n", signal, gpio);

	return 0;
}

static int esp_s31_matrix_set_out(struct esp_pinctrl *epctl, u32 gpio, u32 signal)
{
	if (!epctl->regs[ESP_REGMAP_MATRIX])
		return -ENODEV;

	esp_region_write(epctl, ESP_REGMAP_MATRIX,
			 S31_MATRIX_OUT_SEL_CFG_OFFSET + gpio * 4, signal);
	dev_dbg(epctl->dev, "s31 matrix-out: gpio %u <- signal %u\n", gpio, signal);

	return 0;
}

/*
 * GPIO33/34 are the USB PHY's D-/D+, held by USB-Serial/JTAG while its pad
 * enable is set. Nothing sets it again, so muxing either pad loses the port
 * until reset.
 */
static const struct esp_pad_claim esp32s31_pad_claims[] = {
	{ S31_USB_PHY0_DM_GPIO, S31_USB_PHY0_DP_GPIO, ESP_REGMAP_USB_JTAG,
	  S31_USB_JTAG_CONF0, S31_USB_JTAG_PAD_ENABLE },
};

/*
 * A pad reaches gpiolib through the matrix like any other signal: mux mode 1
 * takes it to the crossbar, and output signal SIG_GPIO_OUT_IDX means "drive
 * this pad from GPIO_OUT" rather than from a peripheral.
 */
static int esp_s31_gpio_request(struct esp_pinctrl *epctl, unsigned int pin)
{
	esp_set_mux(epctl, pin, GPIO_FUNC_GPIO);

	return esp_s31_matrix_set_out(epctl, pin, SIG_GPIO_OUT_IDX);
}

const struct esp_pinctrl_soc esp32s31_soc = {
	.name		= "esp32s31",
	.pins		= esp32s31_pins,
	.npins		= ARRAY_SIZE(esp32s31_pins),
	.pin_count	= S31_PIN_COUNT,
	.regions	= esp32s31_regions,
	.nregions	= ARRAY_SIZE(esp32s31_regions),
	.layers		= esp32s31_layers,
	.functions	= esp32s31_functions,
	.nfunctions	= ARRAY_SIZE(esp32s31_functions),
	.drv_mA		= esp32s31_drv_mA,
	.ndrv_levels	= ARRAY_SIZE(esp32s31_drv_mA),
	.matrix_set_in	= esp_s31_matrix_set_in,
	.matrix_set_out	= esp_s31_matrix_set_out,
	.gpio_request	= esp_s31_gpio_request,
	.pad_claims	= esp32s31_pad_claims,
	.npad_claims	= ARRAY_SIZE(esp32s31_pad_claims),
};
