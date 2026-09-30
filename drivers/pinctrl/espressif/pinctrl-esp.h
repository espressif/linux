/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Shared definitions for the Espressif pinctrl driver family.
 *
 * The driver is split into a SoC-independent core (pinctrl-esp-core.c) plus one
 * file per SoC, which describes that SoC's pads, register regions, iomux field
 * layout and function table as data.
 */

#ifndef __PINCTRL_ESP_H
#define __PINCTRL_ESP_H

#include <linux/bitops.h>
#include <linux/device.h>
#include <linux/io.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/spinlock.h>
#include <linux/pinctrl/pinctrl.h>
#include <linux/pinctrl/pinconf.h>
#include <linux/pinctrl/pinmux.h>

#include "../core.h"
#include "../pinmux.h"

#define ESP_PIN_DESC(n)			PINCTRL_PIN(n, "GPIO" #n)

struct esp_pin_cfg {
	unsigned int	pin;
	u32		func;		/* index into esp_pinctrl_soc.functions[] */
};

/*
 * pinmux binding: DT cells pack a pin id and a function id into one u32 via the
 * ESP_PINMUX(pin, func) macro in the per-SoC dt-bindings header. The function
 * id indexes esp_pinctrl_soc.functions[].
 */
#define ESP_PINMUX_PIN_SHIFT		16
#define ESP_PINMUX_GET_PIN(v)		((v) >> ESP_PINMUX_PIN_SHIFT)
#define ESP_PINMUX_GET_FUNC(v)		((v) & 0xffffU)

struct esp_pinctrl;
struct regmap;

/*
 * enum esp_regmap_id - the register regions a SoC may expose. A SoC declares
 * the subset it uses in esp_pinctrl_soc.regions; register layers address a
 * region by this id.
 */
enum esp_regmap_id {
	ESP_REGMAP_MUX = 0,	/* iomux */
	ESP_REGMAP_MATRIX,	/* gpio matrix */
	ESP_REGMAP_USB_JTAG,	/* USB-Serial/JTAG, a syscon */
	ESP_REGMAP_MAX,
};

/*
 * struct esp_region - how the core reaches one register region: an MMIO mapping
 * by reg-name or by platform reg index when unnamed, or, for a block another
 * driver owns, the syscon a phandle property points at.
 */
struct esp_region {
	enum esp_regmap_id	id;
	const char		*dt_name;	/* reg-names entry, if named */
	unsigned int		reg_index;	/* else platform reg index */
	const char		*syscon;	/* phandle property, instead of reg */
	bool			optional;	/* an absent region is not an error */
	bool			shared;		/* map without claiming: another driver owns it */
};

enum esp_pin_field {
	ESP_FIELD_FUNC = 0,	/* primary mux / function selector */
	ESP_FIELD_IE,		/* input enable */
	ESP_FIELD_PULL_UP,
	ESP_FIELD_PULL_DOWN,
	ESP_FIELD_DRIVE,
	ESP_FIELD_OPEN_DRAIN,
	ESP_FIELD_MAX,
};

struct esp_mux_layer {
	enum esp_regmap_id	map;
	u32			reg_base;	/* offset of the first covered pin */
	u32			reg_stride;	/* bytes per pin; 0 treated as 4 */
	u32			mask;		/* field mask within the register */
	unsigned int		first_pin;	/* lowest pin this layer covers */
};

struct esp_pin_function {
	s16	top_mux;	/* primary direct-mux selector (ESP_FIELD_FUNC) */
	s16	matrix_in;	/* GPIO-matrix input signal, or -1 */
	s16	matrix_out;	/* GPIO-matrix output signal, or -1 */
};

#define ESP_FUNC_NONE			(-1)

/*
 * struct esp_pad_claim - pads another block holds while @mask is set in its
 * register @reg, reached through syscon region @map. Selecting any IO_MUX
 * function on one of them clears @mask first.
 */
struct esp_pad_claim {
	unsigned int		first_pin;
	unsigned int		last_pin;
	enum esp_regmap_id	map;
	u32			reg;
	u32			mask;
};

#define ESP_DRV_LEVELS_MAX		4

struct esp_pinctrl_soc {
	const char			*name;
	const struct pinctrl_pin_desc	*pins;
	unsigned int			npins;
	unsigned int			pin_count;

	const struct esp_region		*regions;
	unsigned int			nregions;

	/* iomux layers, indexed by enum esp_pin_field (mask 0 = unused). */
	const struct esp_mux_layer	*layers;
	const struct esp_pin_function	*functions;
	unsigned int			nfunctions;

	/*
	 * Drive-strength translation: register encoding (index) to milliamps.
	 * NULL means the driver writes the DT argument raw (no translation).
	 */
	const unsigned int		*drv_mA;
	unsigned int			ndrv_levels;

	/* Matrix routing (SoC-specific). */
	int  (*matrix_set_in)(struct esp_pinctrl *epctl, u32 gpio, u32 signal);
	int  (*matrix_set_out)(struct esp_pinctrl *epctl, u32 gpio, u32 signal);

	/*
	 * Take a pad to simple GPIO for gpiolib. Optional: without it the core
	 * selects mux mode 0, which is what a SoC whose pad reaches GPIO
	 * directly needs. A SoC that reaches GPIO through the matrix supplies
	 * this instead, to select the matrix and route the pad's output.
	 */
	int  (*gpio_request)(struct esp_pinctrl *epctl, unsigned int pin);

	const struct esp_pad_claim	*pad_claims;
	unsigned int			npad_claims;
};

struct esp_pinctrl {
	struct device			*dev;
	struct pinctrl_dev		*pctl;
	const struct esp_pinctrl_soc	*soc;

	void __iomem			*regs[ESP_REGMAP_MAX];
	struct regmap			*syscons[ESP_REGMAP_MAX];

	raw_spinlock_t			lock;
};

/* Core helpers shared with the per-SoC files. */
void esp_reg_update(void __iomem *reg, u32 mask, u32 value,
		    raw_spinlock_t *lock);

void esp_region_write(struct esp_pinctrl *epctl, enum esp_regmap_id id,
		      u32 off, u32 val);
void esp_region_update(struct esp_pinctrl *epctl, enum esp_regmap_id id,
		       u32 off, u32 mask, u32 val);
void esp_set_mux(struct esp_pinctrl *epctl, unsigned int pin, u32 mode);

/* Per-SoC descriptors, referenced by the core of_match table. */
extern const struct esp_pinctrl_soc esp32s31_soc;

#endif /* __PINCTRL_ESP_H */
