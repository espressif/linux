// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Espressif ESP32-S31 clock provider
 *
 * Copyright (C) 2026 Espressif Systems (Shanghai) Co. Ltd.
 *
 */

#include <linux/bitops.h>
#include <linux/clk-provider.h>
#include <linux/device.h>
#include <linux/err.h>
#include <linux/math64.h>
#include <linux/mfd/syscon.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>
#include <linux/slab.h>

#include <dt-bindings/clock/esp32s31.h>

/* Peripheral control registers in HP_SYS_CLKRST */
#define ESP32S31_HP_CLKRST_EMAC_CTRL0	0x0c8
#define ESP32S31_HP_CLKRST_I2C0_CTRL0	0x0dc
#define ESP32S31_HP_CLKRST_I2C1_CTRL0	0x0e0
#define ESP32S31_HP_CLKRST_IOMUX_CTRL0	0x15c
#define ESP32S31_HP_CLKRST_PCNT_CTRL0	0x138
#define ESP32S31_HP_CLKRST_SDIO_HOST_CTRL0	0x0c0

/* Peripheral control registers in CNNT_SYS */
#define ESP32S31_CNNT_HP_EMAC_REF_CTRL	0x040
#define ESP32S31_CNNT_HP_EMAC_PTP_CTRL	0x054

/* CPU clock tree registers in HP_SYS_CLKRST */
#define ESP32S31_HP_CLKRST_SOC_CLK_SEL		0x000
#define ESP32S31_HP_CLKRST_CPU_FREQ_CTRL0	0x004
#define ESP32S31_HP_CLKRST_SYS_FREQ_CTRL0	0x00c
#define ESP32S31_HP_CLKRST_APB_FREQ_CTRL0	0x010

#define ESP32S31_SOC_CLK_SEL			GENMASK(1, 0)
#define ESP32S31_CPU_DIV_NUM			GENMASK(7, 0)
#define ESP32S31_CPU_DIV_NUMERATOR		GENMASK(10, 8)
#define ESP32S31_CPU_DIV_DENOMINATOR		GENMASK(13, 11)
#define ESP32S31_SYS_DIV_NUM			GENMASK(7, 0)
#define ESP32S31_SYS_DIV_NUMERATOR		GENMASK(10, 8)
#define ESP32S31_SYS_DIV_DENOMINATOR		GENMASK(13, 11)
#define ESP32S31_APB_DIV_NUM			GENMASK(7, 0)
#define ESP32S31_APB_DIV_NUMERATOR		GENMASK(10, 8)
#define ESP32S31_APB_DIV_DENOMINATOR		GENMASK(13, 11)

/* CPLL divider register in LP_AON_CLKRST */
#define ESP32S31_LP_CLKRST_CPLL_DIV_CFG		0x04c
#define ESP32S31_CPLL_DIV_REF			GENMASK(3, 0)
#define ESP32S31_CPLL_DIV_FB			GENMASK(11, 4)
#define ESP32S31_LP_CLKRST_MPLL_DIV_CFG		0x054
#define ESP32S31_MPLL_DIV_REF			GENMASK(2, 0)
#define ESP32S31_MPLL_DIV_FB			GENMASK(7, 3)

/* Nominal uncalibrated rate from SOC_CLK_RC_FAST_FREQ_APPROX. */
#define ESP32S31_RC_FAST_RATE			17500000
/* Firmware-owned fixed-frequency output derived from BBPLL. */
#define ESP32S31_PLL_F240M_RATE			240000000

/*
 * Field layout shared by the IO_MUX and both I2C control registers, and named
 * for them because it does not generalise: UART0 puts its bus gate in bit 1 and
 * its module gate in bit 7.
 */
#define ESP32S31_IOMUX_I2C_APB_CLK_EN		BIT(0)
#define ESP32S31_IOMUX_I2C_CLK_SRC_SEL		BIT(3)
#define ESP32S31_IOMUX_I2C_CLK_EN		BIT(4)
#define ESP32S31_IOMUX_I2C_CLK_DIV_NUM		GENMASK(12, 5)
#define ESP32S31_I2C_CLK_DIV_FRAC		GENMASK(28, 13)

/* EMAC_CTRL0 and HP_EMAC_REF_CTRL, which share neither layout */
#define ESP32S31_EMAC_SYS_CLK_EN	BIT(0)
#define ESP32S31_EMAC_REF_CLK_SEL	GENMASK(1, 0)
#define ESP32S31_EMAC_REF_CLK_EN	BIT(2)
#define ESP32S31_EMAC_REF_CLK_DIV	GENMASK(15, 8)

/*
 * HP_EMAC_PTP_CTRL. Bit 1 selects the PTP reference source (0 = xtal,
 * 1 = 80 MHz internal); only xtal is modelled, so the select stays at its
 * reset default and the mux is not described.
 */
#define ESP32S31_EMAC_PTP_REF_CLK_EN	BIT(0)

#define ESP32S31_PCNT0_APB_CLK_EN	BIT(0)
#define ESP32S31_PCNT1_APB_CLK_EN	BIT(3)

/* HP_SYS_CLKRST SDIO host clock controls */
#define ESP32S31_SDMMC_SYS_CLK_EN		BIT(0)

/* HP_SYS_CLKRST USB_OTGHS_CTRL0 */
#define ESP32S31_HP_CLKRST_USB_OTGHS_CTRL0	0x0ac
#define ESP32S31_USB_OTGHS_APB_CLK_EN		BIT(0)
#define ESP32S31_USB_OTGHS_SYS_CLK_EN		BIT(1)

/* CNNT_SYS USB_OTG20_CTRL */
#define ESP32S31_CNNT_USB_OTG20_CTRL		0x030
#define ESP32S31_USB_OTG20_PHYREF_CLK_EN	BIT(27)

/*
 * Register windows, indexing the local regmap array. HP_SYS_CLKRST comes from
 * the parent syscon and CNNT_SYS from the esp,cnnt-sys phandle.
 */
#define ESP32S31_CLK_MAP_HP_SYS_CLKRST	0
#define ESP32S31_CLK_MAP_CNNT_SYS	1
#define ESP32S31_CLK_NUM_MAPS		2

#define ESP32S31_CLK_MAX_PARENTS	2
#define ESP32S31_CLK_PARENT_FW		U16_MAX

struct esp_clk_desc {
	const char *name;
	u8 map_idx;
	u32 reg;
	u32 gate;
	u32 div;
	u32 frac;
	u32 mux;
	const char * const *parents;
	const u16 *parent_ids;
	u8 num_parents;
};

static u32 esp_clk_field_get(u32 val, u32 mask)
{
	return (val & mask) >> __ffs(mask);
}

static u32 esp_clk_field_prep(u32 mask, u32 val)
{
	return (val << __ffs(mask)) & mask;
}

struct esp_clk {
	struct clk_hw hw;
	struct regmap *map;
	const struct esp_clk_desc *desc;
};

#define to_esp_clk(_hw) container_of(_hw, struct esp_clk, hw)

enum esp_root_clk_type {
	ESP32S31_ROOT_CPLL,
	ESP32S31_ROOT_MPLL,
	ESP32S31_ROOT_HP,
	ESP32S31_ROOT_CPU,
	ESP32S31_ROOT_SYS,
	ESP32S31_ROOT_APB,
};

struct esp_root_clk {
	struct clk_hw hw;
	struct regmap *hp_sys_clkrst;
	struct regmap *lp_aon_clkrst;
	enum esp_root_clk_type type;
};

#define to_esp_root_clk(_hw) container_of(_hw, struct esp_root_clk, hw)

static u8 esp_root_clk_get_parent(struct clk_hw *hw)
{
	struct esp_root_clk *root_clk = to_esp_root_clk(hw);
	unsigned int val;

	if (regmap_read(root_clk->hp_sys_clkrst,
			ESP32S31_HP_CLKRST_SOC_CLK_SEL, &val))
		return 0;

	return esp_clk_field_get(val, ESP32S31_SOC_CLK_SEL);
}

static unsigned long esp_root_fractional_rate(unsigned long parent_rate,
					      u32 val, u32 div_mask,
					      u32 numerator_mask,
					      u32 denominator_mask)
{
	u32 divider = esp_clk_field_get(val, div_mask) + 1;
	u32 numerator = esp_clk_field_get(val, numerator_mask);
	u32 denominator = esp_clk_field_get(val, denominator_mask);

	if (!denominator) {
		denominator = 1;
		numerator = 0;
	}

	return div64_u64((u64)parent_rate * denominator,
			 (u64)divider * denominator + numerator);
}

static unsigned long esp_root_clk_recalc_rate(struct clk_hw *hw,
					      unsigned long parent_rate)
{
	struct esp_root_clk *root_clk = to_esp_root_clk(hw);
	u32 divider, numerator;
	unsigned int val;
	u64 rate;

	switch (root_clk->type) {
	case ESP32S31_ROOT_CPLL:
		if (regmap_read(root_clk->lp_aon_clkrst,
				ESP32S31_LP_CLKRST_CPLL_DIV_CFG, &val))
			return 0;

		divider = esp_clk_field_get(val, ESP32S31_CPLL_DIV_REF);
		numerator = esp_clk_field_get(val, ESP32S31_CPLL_DIV_FB);
		if (!divider || !numerator)
			return 0;

		rate = mul_u64_u32_div(parent_rate, numerator, divider);
		break;

	case ESP32S31_ROOT_MPLL:
		if (regmap_read(root_clk->lp_aon_clkrst,
				ESP32S31_LP_CLKRST_MPLL_DIV_CFG, &val))
			return 0;

		divider = esp_clk_field_get(val, ESP32S31_MPLL_DIV_REF) + 1;
		numerator = esp_clk_field_get(val, ESP32S31_MPLL_DIV_FB) + 1;
		rate = mul_u64_u32_div(parent_rate, numerator, divider);
		break;

	case ESP32S31_ROOT_HP:
		rate = parent_rate;
		break;

	case ESP32S31_ROOT_CPU:
		if (regmap_read(root_clk->hp_sys_clkrst,
				ESP32S31_HP_CLKRST_CPU_FREQ_CTRL0, &val))
			return 0;

		rate = esp_root_fractional_rate(parent_rate, val,
						ESP32S31_CPU_DIV_NUM,
						ESP32S31_CPU_DIV_NUMERATOR,
						ESP32S31_CPU_DIV_DENOMINATOR);
		break;

	case ESP32S31_ROOT_SYS:
		if (regmap_read(root_clk->hp_sys_clkrst,
				ESP32S31_HP_CLKRST_SYS_FREQ_CTRL0, &val))
			return 0;
		rate = esp_root_fractional_rate(parent_rate, val,
						ESP32S31_SYS_DIV_NUM,
						ESP32S31_SYS_DIV_NUMERATOR,
						ESP32S31_SYS_DIV_DENOMINATOR);
		break;

	case ESP32S31_ROOT_APB:
		if (regmap_read(root_clk->hp_sys_clkrst,
				ESP32S31_HP_CLKRST_APB_FREQ_CTRL0, &val))
			return 0;
		rate = esp_root_fractional_rate(parent_rate, val,
						ESP32S31_APB_DIV_NUM,
						ESP32S31_APB_DIV_NUMERATOR,
						ESP32S31_APB_DIV_DENOMINATOR);
		break;

	default:
		WARN_ON_ONCE(1);
		return 0;
	}

	return min_t(u64, rate, (u64)~0UL);
}

static const struct clk_ops esp_root_clk_ops = {
	.get_parent = esp_root_clk_get_parent,
	.recalc_rate = esp_root_clk_recalc_rate,
};

static const struct clk_ops esp_root_div_clk_ops = {
	.recalc_rate = esp_root_clk_recalc_rate,
};

static int esp_clk_prepare(struct clk_hw *hw)
{
	struct esp_clk *esp_clk = to_esp_clk(hw);

	return regmap_update_bits(esp_clk->map, esp_clk->desc->reg,
				  esp_clk->desc->gate, esp_clk->desc->gate);
}

static void esp_clk_unprepare(struct clk_hw *hw)
{
	struct esp_clk *esp_clk = to_esp_clk(hw);

	regmap_update_bits(esp_clk->map, esp_clk->desc->reg,
			   esp_clk->desc->gate, 0);
}

/* The field holds div - 1, the encoding clk-divider calls the default one. */
static u8 esp_clk_div_width(const struct esp_clk_desc *desc)
{
	return fls(desc->div >> __ffs(desc->div));
}

static unsigned long esp_clk_recalc_rate(struct clk_hw *hw,
					 unsigned long parent_rate)
{
	struct esp_clk *esp_clk = to_esp_clk(hw);
	unsigned int val;

	if (regmap_read(esp_clk->map, esp_clk->desc->reg, &val))
		return 0;

	return divider_recalc_rate(hw, parent_rate,
				   esp_clk_field_get(val, esp_clk->desc->div),
				   NULL, 0,
				   esp_clk_div_width(esp_clk->desc));
}

/*
 * No CLK_SET_RATE_PARENT, so the divider is chosen against the current parent
 * alone: a parent change is a separate, explicit clk_set_parent(). A rate of 0
 * asks for the largest divider.
 */
static int esp_clk_determine_rate(struct clk_hw *hw,
				  struct clk_rate_request *req)
{
	struct esp_clk *esp_clk = to_esp_clk(hw);

	return divider_determine_rate(hw, req, NULL,
				      esp_clk_div_width(esp_clk->desc), 0);
}

static int esp_clk_set_rate(struct clk_hw *hw, unsigned long rate,
			    unsigned long parent_rate)
{
	struct esp_clk *esp_clk = to_esp_clk(hw);
	int val;

	val = divider_get_val(rate, parent_rate, NULL,
			      esp_clk_div_width(esp_clk->desc), 0);
	if (val < 0)
		return val;

	return regmap_update_bits(esp_clk->map, esp_clk->desc->reg,
				  esp_clk->desc->div | esp_clk->desc->frac,
				  esp_clk_field_prep(esp_clk->desc->div, val));
}

static u8 esp_clk_get_parent(struct clk_hw *hw)
{
	struct esp_clk *esp_clk = to_esp_clk(hw);
	unsigned int val;
	u8 index;

	if (regmap_read(esp_clk->map, esp_clk->desc->reg, &val))
		return 0;

	index = esp_clk_field_get(val, esp_clk->desc->mux);

	return index < esp_clk->desc->num_parents ? index : 0;
}

static int esp_clk_set_parent(struct clk_hw *hw, u8 index)
{
	struct esp_clk *esp_clk = to_esp_clk(hw);

	return regmap_update_bits(esp_clk->map, esp_clk->desc->reg,
				  esp_clk->desc->mux,
				  esp_clk_field_prep(esp_clk->desc->mux,
						     index));
}

/*
 * Deliberately no .is_prepared: the IO_MUX gates come out of reset enabled, and
 * reporting that state would let clk_disable_unused() gate the pad logic of
 * every peripheral whose driver has not claimed its clock yet.
 */
static const struct clk_ops esp_clk_ops_gate = {
	.prepare = esp_clk_prepare,
	.unprepare = esp_clk_unprepare,
};

static const struct clk_ops esp_clk_ops_gate_div = {
	.prepare = esp_clk_prepare,
	.unprepare = esp_clk_unprepare,
	.recalc_rate = esp_clk_recalc_rate,
	.determine_rate = esp_clk_determine_rate,
	.set_rate = esp_clk_set_rate,
};

static const struct clk_ops esp_clk_ops_mux_gate_div = {
	.prepare = esp_clk_prepare,
	.unprepare = esp_clk_unprepare,
	.recalc_rate = esp_clk_recalc_rate,
	.determine_rate = esp_clk_determine_rate,
	.set_rate = esp_clk_set_rate,
	.get_parent = esp_clk_get_parent,
	.set_parent = esp_clk_set_parent,
};

/*
 * Firmware parents are named against the provider node's "clock-names".
 * parent_ids replaces an entry with another output of this provider. Both are
 * listed in CLK_SRC_SEL encoding order.
 *
 * IMPORTANT: Update `ESP32S31_CLK_MAX_PARENTS` in this file to always be more
 * (or equal to) than the maximum number of parents.
 * Currently the value is 2.
 */
static const char * const esp32s31_i2c_parents[] = { "xtal", "fosc" };
static const char * const esp32s31_emac_sys_parents[] = { "xtal" };
static const char * const esp32s31_emac_ref_parents[] = { "mpll" };
static const char * const esp32s31_emac_ptp_parents[] = { "xtal" };
static const char * const esp32s31_usb_otghs_bus_parents[] = { "xtal" };
static const char * const esp32s31_usb_otghs_phyref_parents[] = { "xtal" };
static const char * const esp32s31_apb_parents[] = { "apb" };

static const u16 esp32s31_mpll_parent_ids[] = {
	ESP_S31_CLK_MPLL,
};

static const struct esp_clk_desc esp32s31_clk_descs[] = {
	[ESP_S31_CLK_IOMUX_APB] = {
		.name = "iomux_apb",
		.reg = ESP32S31_HP_CLKRST_IOMUX_CTRL0,
		.gate = ESP32S31_IOMUX_I2C_APB_CLK_EN,
	},
	/*
	 * Gate only. The register's source select and divider are left at their
	 * reset values and not modelled, since nothing needs this clock's rate.
	 */
	[ESP_S31_CLK_IOMUX] = {
		.name = "iomux",
		.reg = ESP32S31_HP_CLKRST_IOMUX_CTRL0,
		.gate = ESP32S31_IOMUX_I2C_CLK_EN,
	},
	[ESP_S31_CLK_I2C0_APB] = {
		.name = "i2c0_apb",
		.reg = ESP32S31_HP_CLKRST_I2C0_CTRL0,
		.gate = ESP32S31_IOMUX_I2C_APB_CLK_EN,
	},
	[ESP_S31_CLK_I2C0] = {
		.name = "i2c0",
		.reg = ESP32S31_HP_CLKRST_I2C0_CTRL0,
		.gate = ESP32S31_IOMUX_I2C_CLK_EN,
		.div = ESP32S31_IOMUX_I2C_CLK_DIV_NUM,
		.frac = ESP32S31_I2C_CLK_DIV_FRAC,
		.mux = ESP32S31_IOMUX_I2C_CLK_SRC_SEL,
		.parents = esp32s31_i2c_parents,
		.num_parents = ARRAY_SIZE(esp32s31_i2c_parents),
	},
	[ESP_S31_CLK_I2C1_APB] = {
		.name = "i2c1_apb",
		.reg = ESP32S31_HP_CLKRST_I2C1_CTRL0,
		.gate = ESP32S31_IOMUX_I2C_APB_CLK_EN,
	},
	[ESP_S31_CLK_I2C1] = {
		.name = "i2c1",
		.reg = ESP32S31_HP_CLKRST_I2C1_CTRL0,
		.gate = ESP32S31_IOMUX_I2C_CLK_EN,
		.div = ESP32S31_IOMUX_I2C_CLK_DIV_NUM,
		.frac = ESP32S31_I2C_CLK_DIV_FRAC,
		.mux = ESP32S31_IOMUX_I2C_CLK_SRC_SEL,
		.parents = esp32s31_i2c_parents,
		.num_parents = ARRAY_SIZE(esp32s31_i2c_parents),
	},
	[ESP_S31_CLK_EMAC_SYS] = {
		.name = "emac_sys",
		.reg = ESP32S31_HP_CLKRST_EMAC_CTRL0,
		.gate = ESP32S31_EMAC_SYS_CLK_EN,
		.parents = esp32s31_emac_sys_parents,
		.num_parents = ARRAY_SIZE(esp32s31_emac_sys_parents),
	},
	[ESP_S31_CLK_EMAC_REF] = {
		.name = "emac_ref",
		.map_idx = ESP32S31_CLK_MAP_CNNT_SYS,
		.reg = ESP32S31_CNNT_HP_EMAC_REF_CTRL,
		.gate = ESP32S31_EMAC_REF_CLK_EN,
		.div = ESP32S31_EMAC_REF_CLK_DIV,
		.mux = ESP32S31_EMAC_REF_CLK_SEL,
		.parents = esp32s31_emac_ref_parents,
		.parent_ids = esp32s31_mpll_parent_ids,
		.num_parents = ARRAY_SIZE(esp32s31_emac_ref_parents),
	},
	[ESP_S31_CLK_USB_OTGHS_BUS] = {
		.name = "usb_otghs_bus",
		.reg = ESP32S31_HP_CLKRST_USB_OTGHS_CTRL0,
		.gate = ESP32S31_USB_OTGHS_APB_CLK_EN |
			ESP32S31_USB_OTGHS_SYS_CLK_EN,
		.parents = esp32s31_usb_otghs_bus_parents,
		.num_parents = ARRAY_SIZE(esp32s31_usb_otghs_bus_parents),
	},
	[ESP_S31_CLK_USB_OTGHS_PHYREF] = {
		.name = "usb_otghs_phyref",
		.map_idx = ESP32S31_CLK_MAP_CNNT_SYS,
		.reg = ESP32S31_CNNT_USB_OTG20_CTRL,
		.gate = ESP32S31_USB_OTG20_PHYREF_CLK_EN,
		.parents = esp32s31_usb_otghs_phyref_parents,
		.num_parents = ARRAY_SIZE(esp32s31_usb_otghs_phyref_parents),
	},
	[ESP_S31_CLK_EMAC_PTP] = {
		.name = "emac_ptp",
		.map_idx = ESP32S31_CLK_MAP_CNNT_SYS,
		.reg = ESP32S31_CNNT_HP_EMAC_PTP_CTRL,
		.gate = ESP32S31_EMAC_PTP_REF_CLK_EN,
		.parents = esp32s31_emac_ptp_parents,
		.num_parents = ARRAY_SIZE(esp32s31_emac_ptp_parents),
	},
	[ESP_S31_CLK_PCNT0_APB] = {
		.name = "pcnt0_apb",
		.reg = ESP32S31_HP_CLKRST_PCNT_CTRL0,
		.gate = ESP32S31_PCNT0_APB_CLK_EN,
		.parents = esp32s31_apb_parents,
		.num_parents = ARRAY_SIZE(esp32s31_apb_parents),
	},
	[ESP_S31_CLK_PCNT1_APB] = {
		.name = "pcnt1_apb",
		.reg = ESP32S31_HP_CLKRST_PCNT_CTRL0,
		.gate = ESP32S31_PCNT1_APB_CLK_EN,
		.parents = esp32s31_apb_parents,
		.num_parents = ARRAY_SIZE(esp32s31_apb_parents),
	},
	[ESP_S31_CLK_SDMMC_BIU] = {
		.name = "sdmmc_biu",
		.reg = ESP32S31_HP_CLKRST_SDIO_HOST_CTRL0,
		.gate = ESP32S31_SDMMC_SYS_CLK_EN,
	},
};

static const struct clk_ops *esp_clk_pick_ops(const struct esp_clk_desc *desc)
{
	if (desc->mux)
		return &esp_clk_ops_mux_gate_div;

	if (desc->div)
		return &esp_clk_ops_gate_div;

	return &esp_clk_ops_gate;
}

static int esp_clk_register_root(struct device *dev,
				 struct clk_hw_onecell_data *clk_data,
				 struct regmap *hp_sys_clkrst,
				 struct regmap *lp_aon_clkrst,
				 unsigned int id, const char *name,
				 enum esp_root_clk_type type,
				 const u16 *parent_ids,
				 const char * const *parent_names,
				 unsigned int num_parents)
{
	struct clk_parent_data *parent_data;
	struct clk_init_data init = { };
	struct esp_root_clk *root_clk;
	unsigned int i;
	int ret;

	root_clk = devm_kzalloc(dev, sizeof(*root_clk), GFP_KERNEL);
	if (!root_clk)
		return -ENOMEM;

	parent_data = devm_kcalloc(dev, num_parents, sizeof(*parent_data), GFP_KERNEL);
	if (!parent_data)
		return -ENOMEM;

	for (i = 0; i < num_parents; i++) {
		if (parent_ids[i] == ESP32S31_CLK_PARENT_FW)
			parent_data[i].fw_name = parent_names[i];
		else
			parent_data[i].hw = clk_data->hws[parent_ids[i]];
	}

	init.name = name;
	init.ops = type == ESP32S31_ROOT_HP ? &esp_root_clk_ops :
						    &esp_root_div_clk_ops;
	init.parent_data = parent_data;
	init.num_parents = num_parents;
	init.flags = CLK_GET_RATE_NOCACHE;

	root_clk->hp_sys_clkrst = hp_sys_clkrst;
	root_clk->lp_aon_clkrst = lp_aon_clkrst;
	root_clk->type = type;
	root_clk->hw.init = &init;

	ret = devm_clk_hw_register(dev, &root_clk->hw);
	if (ret)
		return dev_err_probe(dev, ret, "failed to register %s\n",
				     name);

	clk_data->hws[id] = &root_clk->hw;

	return 0;
}

static int esp_clk_register_roots(struct device *dev,
				  struct clk_hw_onecell_data *clk_data,
				  struct regmap *hp_sys_clkrst,
				  struct regmap *lp_aon_clkrst)
{
	static const char * const xtal_parent_name[] = { "xtal" };
	static const char * const hp_parent_names[] = {
		"xtal", NULL, NULL, NULL,
	};
	static const u16 cpll_parent[] = {
		ESP32S31_CLK_PARENT_FW,
	};
	static const u16 hp_parents[] = {
		ESP32S31_CLK_PARENT_FW,
		ESP_S31_CLK_CPLL,
		ESP_S31_CLK_RC_FAST,
		ESP_S31_CLK_PLL_F240M,
	};
	static const u16 cpu_parent[] = { ESP_S31_CLK_HP_ROOT };
	static const u16 cpu_child_parent[] = { ESP_S31_CLK_CPU };
	static const u16 sys_parent[] = { ESP_S31_CLK_SYS };
	struct clk_hw *hw;
	int ret;

	ret = esp_clk_register_root(dev, clk_data, hp_sys_clkrst,
				    lp_aon_clkrst, ESP_S31_CLK_CPLL, "cpll",
				    ESP32S31_ROOT_CPLL, cpll_parent,
				    xtal_parent_name, ARRAY_SIZE(cpll_parent));
	if (ret)
		return ret;

	ret = esp_clk_register_root(dev, clk_data, hp_sys_clkrst,
				    lp_aon_clkrst, ESP_S31_CLK_MPLL, "mpll",
				    ESP32S31_ROOT_MPLL, cpll_parent,
				    xtal_parent_name, ARRAY_SIZE(cpll_parent));
	if (ret)
		return ret;

	hw = devm_clk_hw_register_fixed_rate(dev, "rc_fast", NULL, 0,
					     ESP32S31_RC_FAST_RATE);
	if (IS_ERR(hw))
		return dev_err_probe(dev, PTR_ERR(hw),
				     "failed to register rc_fast\n");
	clk_data->hws[ESP_S31_CLK_RC_FAST] = hw;

	hw = devm_clk_hw_register_fixed_rate(dev, "pll_f240m", NULL, 0,
					     ESP32S31_PLL_F240M_RATE);
	if (IS_ERR(hw))
		return dev_err_probe(dev, PTR_ERR(hw),
				     "failed to register pll_f240m\n");
	clk_data->hws[ESP_S31_CLK_PLL_F240M] = hw;

	ret = esp_clk_register_root(dev, clk_data, hp_sys_clkrst,
				    lp_aon_clkrst, ESP_S31_CLK_HP_ROOT,
				    "hp_root", ESP32S31_ROOT_HP, hp_parents,
				    hp_parent_names, ARRAY_SIZE(hp_parents));
	if (ret)
		return ret;

	ret = esp_clk_register_root(dev, clk_data, hp_sys_clkrst,
				    lp_aon_clkrst, ESP_S31_CLK_CPU, "cpu",
				    ESP32S31_ROOT_CPU, cpu_parent, NULL,
				    ARRAY_SIZE(cpu_parent));
	if (ret)
		return ret;

	ret = esp_clk_register_root(dev, clk_data, hp_sys_clkrst,
				    lp_aon_clkrst, ESP_S31_CLK_SYS, "sys",
				    ESP32S31_ROOT_SYS, cpu_child_parent, NULL,
				    ARRAY_SIZE(cpu_child_parent));
	if (ret)
		return ret;

	return esp_clk_register_root(dev, clk_data, hp_sys_clkrst,
				     lp_aon_clkrst, ESP_S31_CLK_APB, "apb",
				     ESP32S31_ROOT_APB, sys_parent, NULL,
				     ARRAY_SIZE(sys_parent));
}

static int esp_clk_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	const struct esp_clk_desc *descs;
	struct clk_hw_onecell_data *clk_data;
	struct regmap *maps[ESP32S31_CLK_NUM_MAPS];
	struct regmap *lp_aon_clkrst;
	struct regmap *map;
	unsigned int i, j;
	int ret;

	descs = of_device_get_match_data(dev);
	if (!descs || !dev->parent)
		return -ENODEV;

	map = syscon_node_to_regmap(dev->parent->of_node);
	if (IS_ERR(map))
		return dev_err_probe(dev, PTR_ERR(map),
				     "failed to get parent syscon regmap\n");
	maps[ESP32S31_CLK_MAP_HP_SYS_CLKRST] = map;

	map = syscon_regmap_lookup_by_phandle(dev->of_node,
					      "esp,cnnt-sys");
	if (IS_ERR(map))
		return dev_err_probe(dev, PTR_ERR(map),
				     "failed to get cnnt-sys regmap\n");
	maps[ESP32S31_CLK_MAP_CNNT_SYS] = map;

	lp_aon_clkrst = syscon_regmap_lookup_by_phandle(dev->of_node,
							"esp,lp-aon-clkrst");
	if (IS_ERR(lp_aon_clkrst))
		return dev_err_probe(dev, PTR_ERR(lp_aon_clkrst),
				     "failed to get lp-aon-clkrst regmap\n");

	clk_data = devm_kzalloc(dev, struct_size(clk_data, hws, ESP_S31_CLK_NUM),
				GFP_KERNEL);
	if (!clk_data)
		return -ENOMEM;
	clk_data->num = ESP_S31_CLK_NUM;

	ret = esp_clk_register_roots(dev, clk_data,
				     maps[ESP32S31_CLK_MAP_HP_SYS_CLKRST],
				     lp_aon_clkrst);
	if (ret)
		return ret;

	for (i = 0; i < ARRAY_SIZE(esp32s31_clk_descs); i++) {
		const struct esp_clk_desc *desc = &descs[i];
		struct clk_parent_data parent_data[ESP32S31_CLK_MAX_PARENTS] = { };
		struct clk_init_data init = { };
		struct esp_clk *esp_clk;

		esp_clk = devm_kzalloc(dev, sizeof(*esp_clk), GFP_KERNEL);
		if (!esp_clk)
			return -ENOMEM;

		esp_clk->map = maps[desc->map_idx];
		esp_clk->desc = desc;

		for (j = 0; j < desc->num_parents; j++) {
			if (desc->parent_ids &&
			    desc->parent_ids[j] != ESP32S31_CLK_PARENT_FW)
				parent_data[j].hw =
					clk_data->hws[desc->parent_ids[j]];
			else
				parent_data[j].fw_name = desc->parents[j];
		}

		init.name = desc->name;
		init.ops = esp_clk_pick_ops(desc);
		init.parent_data = desc->num_parents ? parent_data : NULL;
		init.num_parents = desc->num_parents;
		if (desc->div)
			init.flags = CLK_GET_RATE_NOCACHE;

		esp_clk->hw.init = &init;

		ret = devm_clk_hw_register(dev, &esp_clk->hw);
		if (ret)
			return dev_err_probe(dev, ret, "failed to register %s\n",
					     desc->name);

		clk_data->hws[i] = &esp_clk->hw;
	}

	ret = devm_of_clk_add_hw_provider(dev, of_clk_hw_onecell_get, clk_data);
	if (ret)
		return dev_err_probe(dev, ret, "failed to add clk provider\n");

	dev_info(dev, "esp clk provider registered (%u clocks)\n",
		 ESP_S31_CLK_NUM);

	return 0;
}

static const struct of_device_id esp_clk_of_match[] = {
	{ .compatible = "esp,esp32s31-clk", .data = esp32s31_clk_descs },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, esp_clk_of_match);

static struct platform_driver esp_clk_driver = {
	.probe	= esp_clk_probe,
	.driver	= {
		.name		= "esp32s31-clk",
		.of_match_table	= esp_clk_of_match,
	},
};
module_platform_driver(esp_clk_driver);

MODULE_AUTHOR("Espressif Systems");
MODULE_DESCRIPTION("Espressif ESP32-S31 clock provider");
MODULE_LICENSE("GPL");
