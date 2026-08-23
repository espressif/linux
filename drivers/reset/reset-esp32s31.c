// SPDX-License-Identifier: GPL-2.0
/*
 * Espressif ESP32-S31 reset controller
 *
 * Copyright (C) 2026 Espressif Systems (Shanghai) Co. Ltd.
 *
 */

#include <linux/bitops.h>
#include <linux/mfd/syscon.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>
#include <linux/reset-controller.h>

#include <dt-bindings/reset/esp32s31.h>

/* Peripheral control registers in HP_SYS_CLKRST */
#define ESP32S31_HP_CLKRST_IOMUX_CTRL0	0x15c
#define ESP32S31_HP_CLKRST_I2C0_CTRL0	0x0dc
#define ESP32S31_HP_CLKRST_I2C1_CTRL0	0x0e0

/*
 * The reset bit shared between IO_MUX and I2C control registers.
 */
#define ESP32S31_IOMUX_I2C_RST_EN	BIT(1)

struct esp_s31_rst_desc {
	u32 reg;
	u32 mask;
};

static const struct esp_s31_rst_desc esp_s31_rst_descs[ESP_S31_RST_NUM] = {
	[ESP_S31_RST_IOMUX] = {
		.reg = ESP32S31_HP_CLKRST_IOMUX_CTRL0,
		.mask = ESP32S31_IOMUX_I2C_RST_EN,
	},
	[ESP_S31_RST_I2C0] = {
		.reg = ESP32S31_HP_CLKRST_I2C0_CTRL0,
		.mask = ESP32S31_IOMUX_I2C_RST_EN,
	},
	[ESP_S31_RST_I2C1] = {
		.reg = ESP32S31_HP_CLKRST_I2C1_CTRL0,
		.mask = ESP32S31_IOMUX_I2C_RST_EN,
	},
};

struct esp_s31_reset {
	struct reset_controller_dev rcdev;
	struct regmap *map;
};

static struct esp_s31_reset *to_esp_s31_reset(struct reset_controller_dev *rcdev)
{
	return container_of(rcdev, struct esp_s31_reset, rcdev);
}

static int esp_s31_reset_assert(struct reset_controller_dev *rcdev,
				unsigned long id)
{
	struct esp_s31_reset *rst = to_esp_s31_reset(rcdev);
	const struct esp_s31_rst_desc *d;

	if (id >= ESP_S31_RST_NUM)
		return -EINVAL;

	d = &esp_s31_rst_descs[id];

	return regmap_update_bits(rst->map, d->reg, d->mask, d->mask);
}

static int esp_s31_reset_deassert(struct reset_controller_dev *rcdev,
				  unsigned long id)
{
	struct esp_s31_reset *rst = to_esp_s31_reset(rcdev);
	const struct esp_s31_rst_desc *d;

	if (id >= ESP_S31_RST_NUM)
		return -EINVAL;

	d = &esp_s31_rst_descs[id];

	return regmap_update_bits(rst->map, d->reg, d->mask, 0);
}

static int esp_s31_reset_status(struct reset_controller_dev *rcdev,
				unsigned long id)
{
	struct esp_s31_reset *rst = to_esp_s31_reset(rcdev);
	const struct esp_s31_rst_desc *d;
	unsigned int val;
	int ret;

	if (id >= ESP_S31_RST_NUM)
		return -EINVAL;

	d = &esp_s31_rst_descs[id];

	ret = regmap_read(rst->map, d->reg, &val);
	if (ret)
		return ret;

	return !!(val & d->mask);
}

static const struct reset_control_ops esp_s31_reset_ops = {
	.assert = esp_s31_reset_assert,
	.deassert = esp_s31_reset_deassert,
	.status = esp_s31_reset_status,
};

static int esp_s31_reset_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct esp_s31_reset *rst;
	int ret;

	if (!dev->parent)
		return -ENODEV;

	rst = devm_kzalloc(dev, sizeof(*rst), GFP_KERNEL);
	if (!rst)
		return -ENOMEM;

	rst->map = syscon_node_to_regmap(dev->parent->of_node);
	if (IS_ERR(rst->map))
		return dev_err_probe(dev, PTR_ERR(rst->map),
				     "failed to get parent syscon regmap\n");

	rst->rcdev.ops = &esp_s31_reset_ops;
	rst->rcdev.owner = THIS_MODULE;
	rst->rcdev.dev = dev;
	rst->rcdev.of_node = dev->of_node;
	rst->rcdev.nr_resets = ESP_S31_RST_NUM;
	rst->rcdev.of_reset_n_cells = 1;

	ret = devm_reset_controller_register(dev, &rst->rcdev);
	if (ret)
		return dev_err_probe(dev, ret,
				     "failed to register reset controller\n");

	dev_info(dev, "ESP32-S31 reset provider registered\n");

	return 0;
}

static const struct of_device_id esp_s31_reset_of_match[] = {
	{ .compatible = "esp,esp32s31-reset" },
	{ }
};
MODULE_DEVICE_TABLE(of, esp_s31_reset_of_match);

static struct platform_driver esp_s31_reset_driver = {
	.probe = esp_s31_reset_probe,
	.driver = {
		.name = "esp32s31-reset",
		.of_match_table = esp_s31_reset_of_match,
	},
};
module_platform_driver(esp_s31_reset_driver);

MODULE_AUTHOR("Espressif Systems");
MODULE_DESCRIPTION("Espressif ESP32-S31 reset controller");
MODULE_LICENSE("GPL");
