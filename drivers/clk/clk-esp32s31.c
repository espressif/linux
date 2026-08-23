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

/* Peripheral control registers in CNNT_SYS */
#define ESP32S31_CNNT_HP_EMAC_REF_CTRL	0x040
#define ESP32S31_CNNT_HP_EMAC_PTP_CTRL	0x054

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

struct esp_clk_desc {
	const char *name;
	u8 map_idx;
	u32 reg;
	u32 gate;
	u32 div;
	u32 frac;
	u32 mux;
	const char * const *parents;
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

static int esp_clk_prepare(struct clk_hw *hw)
{
	struct esp_clk *c = to_esp_clk(hw);

	return regmap_update_bits(c->map, c->desc->reg, c->desc->gate,
				  c->desc->gate);
}

static void esp_clk_unprepare(struct clk_hw *hw)
{
	struct esp_clk *c = to_esp_clk(hw);

	regmap_update_bits(c->map, c->desc->reg, c->desc->gate, 0);
}

/* The field holds div - 1, the encoding clk-divider calls the default one. */
static u8 esp_clk_div_width(const struct esp_clk_desc *d)
{
	return fls(d->div >> __ffs(d->div));
}

static unsigned long esp_clk_recalc_rate(struct clk_hw *hw,
					 unsigned long parent_rate)
{
	struct esp_clk *c = to_esp_clk(hw);
	unsigned int val;

	if (regmap_read(c->map, c->desc->reg, &val))
		return 0;

	return divider_recalc_rate(hw, parent_rate,
				   esp_clk_field_get(val, c->desc->div), NULL, 0,
				   esp_clk_div_width(c->desc));
}

/*
 * No CLK_SET_RATE_PARENT, so the divider is chosen against the current parent
 * alone: a parent change is a separate, explicit clk_set_parent(). A rate of 0
 * asks for the largest divider.
 */
static int esp_clk_determine_rate(struct clk_hw *hw,
				  struct clk_rate_request *req)
{
	struct esp_clk *c = to_esp_clk(hw);

	return divider_determine_rate(hw, req, NULL, esp_clk_div_width(c->desc),
				      0);
}

static int esp_clk_set_rate(struct clk_hw *hw, unsigned long rate,
			    unsigned long parent_rate)
{
	struct esp_clk *c = to_esp_clk(hw);
	int val;

	val = divider_get_val(rate, parent_rate, NULL,
			      esp_clk_div_width(c->desc), 0);
	if (val < 0)
		return val;

	return regmap_update_bits(c->map, c->desc->reg,
				  c->desc->div | c->desc->frac,
				  esp_clk_field_prep(c->desc->div, val));
}

static u8 esp_clk_get_parent(struct clk_hw *hw)
{
	struct esp_clk *c = to_esp_clk(hw);
	unsigned int val;
	u8 index;

	if (regmap_read(c->map, c->desc->reg, &val))
		return 0;

	index = esp_clk_field_get(val, c->desc->mux);

	return index < c->desc->num_parents ? index : 0;
}

static int esp_clk_set_parent(struct clk_hw *hw, u8 index)
{
	struct esp_clk *c = to_esp_clk(hw);

	return regmap_update_bits(c->map, c->desc->reg, c->desc->mux,
				  esp_clk_field_prep(c->desc->mux, index));
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
 * Parents are named against the provider node's "clock-names" and listed in
 * CLK_SRC_SEL encoding order
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

static const struct esp_clk_desc esp32s31_clk_descs[ESP_S31_CLK_NUM] = {
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
};

static const struct clk_ops *esp_clk_pick_ops(const struct esp_clk_desc *d)
{
	if (d->mux)
		return &esp_clk_ops_mux_gate_div;

	if (d->div)
		return &esp_clk_ops_gate_div;

	return &esp_clk_ops_gate;
}

static int esp_clk_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	const struct esp_clk_desc *descs;
	struct clk_hw_onecell_data *clk_data;
	struct regmap *maps[ESP32S31_CLK_NUM_MAPS];
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

	clk_data = devm_kzalloc(dev, struct_size(clk_data, hws, ESP_S31_CLK_NUM),
				GFP_KERNEL);
	if (!clk_data)
		return -ENOMEM;
	clk_data->num = ESP_S31_CLK_NUM;

	for (i = 0; i < ESP_S31_CLK_NUM; i++) {
		const struct esp_clk_desc *d = &descs[i];
		struct clk_parent_data pdata[ESP32S31_CLK_MAX_PARENTS] = { };
		struct clk_init_data init = { };
		struct esp_clk *c;

		c = devm_kzalloc(dev, sizeof(*c), GFP_KERNEL);
		if (!c)
			return -ENOMEM;

		c->map = maps[d->map_idx];
		c->desc = d;

		for (j = 0; j < d->num_parents; j++)
			pdata[j].fw_name = d->parents[j];

		init.name = d->name;
		init.ops = esp_clk_pick_ops(d);
		init.parent_data = d->num_parents ? pdata : NULL;
		init.num_parents = d->num_parents;
		if (d->div)
			init.flags = CLK_GET_RATE_NOCACHE;

		c->hw.init = &init;

		ret = devm_clk_hw_register(dev, &c->hw);
		if (ret)
			return dev_err_probe(dev, ret, "failed to register %s\n",
					     d->name);

		clk_data->hws[i] = &c->hw;
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
