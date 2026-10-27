// SPDX-License-Identifier: GPL-2.0
/*
 * ESP32-S31 DesignWare Mobile Storage Host Controller glue
 *
 * Copyright (C) 2026 Espressif Systems (Shanghai) Co. Ltd.
 */

#include <linux/bitfield.h>
#include <linux/clk.h>
#include <linux/mfd/syscon.h>
#include <linux/mmc/host.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>

#include "dw_mmc.h"
#include "dw_mmc-pltfm.h"

#define ESP32S31_SDIO_HOST_CTRL0		0x0c0
#define ESP32S31_SDIO_HS_MODE			BIT(1)
#define ESP32S31_SDIO_LS_CLK_SRC_SEL		BIT(2)
#define ESP32S31_SDIO_LS_CLK_EN			BIT(3)

#define ESP32S31_SDIO_HOST_FUNC_CTRL0		0x0c4
#define ESP32S31_SDIO_LS_CLK_EDGE_CFG_UPDATE	BIT(0)
#define ESP32S31_SDIO_LS_CLK_EDGE_L		GENMASK(4, 1)
#define ESP32S31_SDIO_LS_CLK_EDGE_H		GENMASK(8, 5)
#define ESP32S31_SDIO_LS_CLK_EDGE_N		GENMASK(12, 9)
#define ESP32S31_SDIO_LS_SLF_CLK_EDGE_SEL	GENMASK(15, 13)
#define ESP32S31_SDIO_LS_DRV_CLK_EDGE_SEL	GENMASK(18, 16)
#define ESP32S31_SDIO_LS_SAM_CLK_EDGE_SEL	GENMASK(21, 19)
#define ESP32S31_SDIO_LS_SLF_CLK_EN		BIT(22)
#define ESP32S31_SDIO_LS_DRV_CLK_EN		BIT(23)
#define ESP32S31_SDIO_LS_SAM_CLK_EN		BIT(24)
#define ESP32S31_SDIO_LS_DRV_CLK_EDGE_DEFAULT	1

#define ESP32S31_SDMMC_MEM_LP_CTRL		0x010
#define ESP32S31_SDMMC_MEM_LP_EN		BIT(2)
#define ESP32S31_SDMMC_MEM_LP_FORCE_CTRL	BIT(3)

struct esp32s31_sdmmc {
	struct regmap *clkrst;
	struct regmap *cnnt_sys;
	struct clk *func_clk_src;
};

static int dw_mci_esp32s31_parse_dt(struct dw_mci *host)
{
	struct device_node *np = host->dev->of_node;
	struct esp32s31_sdmmc *priv;

	priv = devm_kzalloc(host->dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	priv->clkrst = syscon_regmap_lookup_by_phandle(np, "esp,clkrst");
	if (IS_ERR(priv->clkrst))
		return dev_err_probe(host->dev, PTR_ERR(priv->clkrst),
				     "failed to get clock syscon\n");

	priv->cnnt_sys = syscon_regmap_lookup_by_phandle(np, "esp,cnnt-sys");
	if (IS_ERR(priv->cnnt_sys))
		return dev_err_probe(host->dev, PTR_ERR(priv->cnnt_sys),
				     "failed to get connectivity syscon\n");

	priv->func_clk_src = devm_clk_get(host->dev, "func_clk_src");
	if (IS_ERR(priv->func_clk_src))
		return dev_err_probe(host->dev, PTR_ERR(priv->func_clk_src),
				     "failed to get functional clock source\n");

	host->priv = priv;

	return 0;
}

static void dw_mci_esp32s31_disable_func_clk(void *data)
{
	struct esp32s31_sdmmc *priv = data;

	regmap_clear_bits(priv->clkrst, ESP32S31_SDIO_HOST_FUNC_CTRL0,
			  ESP32S31_SDIO_LS_SLF_CLK_EN |
			  ESP32S31_SDIO_LS_DRV_CLK_EN |
			  ESP32S31_SDIO_LS_SAM_CLK_EN);
	regmap_clear_bits(priv->clkrst, ESP32S31_SDIO_HOST_CTRL0,
			  ESP32S31_SDIO_LS_CLK_EN);
	clk_disable_unprepare(priv->func_clk_src);
}

static void dw_mci_esp32s31_power_release(void *data)
{
	struct esp32s31_sdmmc *priv = data;

	regmap_update_bits(priv->cnnt_sys, ESP32S31_SDMMC_MEM_LP_CTRL,
			   ESP32S31_SDMMC_MEM_LP_FORCE_CTRL |
			   ESP32S31_SDMMC_MEM_LP_EN, 0);
}

static int dw_mci_esp32s31_init(struct dw_mci *host)
{
	struct esp32s31_sdmmc *priv = host->priv;
	unsigned long src_rate;
	u32 clock_mask, clock_val, div;
	int ret;

	src_rate = clk_get_rate(priv->func_clk_src);
	if (!src_rate || !host->bus_hz)
		return dev_err_probe(host->dev, -EINVAL,
				     "invalid functional clock rate\n");

	div = DIV_ROUND_UP(src_rate, host->bus_hz);
	if (div > FIELD_MAX(ESP32S31_SDIO_LS_CLK_EDGE_L) + 1)
		return dev_err_probe(host->dev, -EINVAL,
				     "functional clock divider %u is too large\n",
				     div);

	host->bus_hz = src_rate / div;

	ret = regmap_update_bits(priv->cnnt_sys,
				 ESP32S31_SDMMC_MEM_LP_CTRL,
				 ESP32S31_SDMMC_MEM_LP_FORCE_CTRL |
				 ESP32S31_SDMMC_MEM_LP_EN,
				 ESP32S31_SDMMC_MEM_LP_FORCE_CTRL);
	if (ret)
		return ret;

	ret = devm_add_action_or_reset(host->dev,
				       dw_mci_esp32s31_power_release, priv);
	if (ret)
		return ret;

	ret = clk_prepare_enable(priv->func_clk_src);
	if (ret)
		return ret;

	ret = devm_add_action_or_reset(host->dev,
				       dw_mci_esp32s31_disable_func_clk, priv);
	if (ret)
		return ret;

	/* MPLL is source selector 0; the divider and phases are IP-specific. */
	ret = regmap_update_bits(priv->clkrst, ESP32S31_SDIO_HOST_CTRL0,
				 ESP32S31_SDIO_HS_MODE |
				 ESP32S31_SDIO_LS_CLK_SRC_SEL |
				 ESP32S31_SDIO_LS_CLK_EN,
				 (div == 1 ? ESP32S31_SDIO_HS_MODE : 0) |
				 ESP32S31_SDIO_LS_CLK_EN);
	if (ret)
		return ret;

	clock_mask = ESP32S31_SDIO_LS_CLK_EDGE_L |
		     ESP32S31_SDIO_LS_CLK_EDGE_H |
		     ESP32S31_SDIO_LS_CLK_EDGE_N |
		     ESP32S31_SDIO_LS_SLF_CLK_EDGE_SEL |
		     ESP32S31_SDIO_LS_DRV_CLK_EDGE_SEL |
		     ESP32S31_SDIO_LS_SAM_CLK_EDGE_SEL |
		     ESP32S31_SDIO_LS_SLF_CLK_EN |
		     ESP32S31_SDIO_LS_DRV_CLK_EN |
		     ESP32S31_SDIO_LS_SAM_CLK_EN;
	clock_val = FIELD_PREP(ESP32S31_SDIO_LS_CLK_EDGE_L, div - 1) |
		    FIELD_PREP(ESP32S31_SDIO_LS_CLK_EDGE_H,
			       div > 1 ? div / 2 - 1 : 0) |
		    FIELD_PREP(ESP32S31_SDIO_LS_CLK_EDGE_N, div - 1) |
		    FIELD_PREP(ESP32S31_SDIO_LS_DRV_CLK_EDGE_SEL,
			       ESP32S31_SDIO_LS_DRV_CLK_EDGE_DEFAULT) |
		    ESP32S31_SDIO_LS_SLF_CLK_EN |
		    ESP32S31_SDIO_LS_DRV_CLK_EN |
		    ESP32S31_SDIO_LS_SAM_CLK_EN;
	ret = regmap_update_bits(priv->clkrst,
				 ESP32S31_SDIO_HOST_FUNC_CTRL0,
				 clock_mask, clock_val);
	if (ret)
		return ret;

	ret = regmap_set_bits(priv->clkrst, ESP32S31_SDIO_HOST_FUNC_CTRL0,
			      ESP32S31_SDIO_LS_CLK_EDGE_CFG_UPDATE);
	if (ret)
		return ret;

	return regmap_clear_bits(priv->clkrst,
				 ESP32S31_SDIO_HOST_FUNC_CTRL0,
				 ESP32S31_SDIO_LS_CLK_EDGE_CFG_UPDATE);
}

static const struct dw_mci_drv_data esp32s31_data = {
	.init = dw_mci_esp32s31_init,
	.parse_dt = dw_mci_esp32s31_parse_dt,
};

static const struct of_device_id dw_mci_esp32s31_match[] = {
	{ .compatible = "esp,esp32s31-dw-mshc" },
	{ }
};
MODULE_DEVICE_TABLE(of, dw_mci_esp32s31_match);

static int dw_mci_esp32s31_probe(struct platform_device *pdev)
{
	return dw_mci_pltfm_register(pdev, &esp32s31_data);
}

static struct platform_driver dw_mci_esp32s31_driver = {
	.probe = dw_mci_esp32s31_probe,
	.remove = dw_mci_pltfm_remove,
	.driver = {
		.name = "dwmmc_esp32s31",
		.probe_type = PROBE_PREFER_ASYNCHRONOUS,
		.of_match_table = dw_mci_esp32s31_match,
	},
};
module_platform_driver(dw_mci_esp32s31_driver);

MODULE_DESCRIPTION("ESP32-S31 DesignWare Mobile Storage Host Controller glue");
MODULE_LICENSE("GPL");
