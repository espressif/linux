// SPDX-License-Identifier: GPL-2.0
/* Espressif ESP32-S31 DWMAC glue.
 *
 * Copyright (C) 2026, Espressif Systems (Shanghai) CO LTD
 */

#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/mfd/syscon.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>
#include <linux/stmmac.h>

#include "stmmac.h"
#include "stmmac_platform.h"

/*
 * DMA ring depths, shrunk from the stmmac defaults (512/512).
 * Higher numbers do not translate to higher throughput.
 */
#define ESP32S31_DMA_RX_SIZE	64
#define ESP32S31_DMA_TX_SIZE	128

#define CNNT_HP_EMAC_RMII_PAD_CTRL	0x00000044U
#define EMAC_RMII_PAD_CLK_EN		BIT(1)
#define EMAC_RMII_PAD_CLK_INV_EN	BIT(2)
#define CNNT_HP_EMAC_RMII_CTRL		0x00000048U
#define EMAC_RMII_CLK_SEL		BIT(0)
#define EMAC_RMII_CLK_EN		BIT(1)
#define EMAC_RMII_PAD_OUT_CLK		BIT(2)
#define CNNT_HP_EMAC_RX_CTRL		0x0000004cU
#define EMAC_RX_PAD_CLK_EN		BIT(0)
#define EMAC_RX_PAD_CLK_INV_EN		BIT(1)
#define EMAC_RX_CLK_SEL			BIT(2)
#define EMAC_RX_180_CLK_EN		BIT(3)
#define CNNT_HP_EMAC_TX_CTRL		0x00000050U
#define EMAC_TX_PAD_CLK_EN		BIT(0)
#define EMAC_TX_CLK_SEL			BIT(2)
#define EMAC_TX_180_CLK_EN		BIT(3)
#define CNNT_GMAC_CTRL0			0x00000060U
#define GMAC_PHY_INTF_SEL_MASK		GENMASK(4, 2)
#define GMAC_PHY_INTF_SEL_RGMII		BIT(2)

struct esp32s31_dwmac {
	struct regmap	*cnnt_sys;
	struct clk	*clk_ref;
};

static int esp32s31_dwmac_init(struct platform_device *pdev, void *bsp_priv)
{
	struct esp32s31_dwmac *dwmac = bsp_priv;

	regmap_update_bits(dwmac->cnnt_sys, CNNT_GMAC_CTRL0,
			   GMAC_PHY_INTF_SEL_MASK, GMAC_PHY_INTF_SEL_RGMII);

	regmap_update_bits(dwmac->cnnt_sys, CNNT_HP_EMAC_RMII_PAD_CTRL,
			   EMAC_RMII_PAD_CLK_EN | EMAC_RMII_PAD_CLK_INV_EN, 0);

	regmap_update_bits(dwmac->cnnt_sys, CNNT_HP_EMAC_RMII_CTRL,
			   EMAC_RMII_CLK_SEL | EMAC_RMII_CLK_EN |
			   EMAC_RMII_PAD_OUT_CLK, EMAC_RMII_PAD_OUT_CLK);

	regmap_update_bits(dwmac->cnnt_sys, CNNT_HP_EMAC_RX_CTRL,
			   EMAC_RX_PAD_CLK_INV_EN | EMAC_RX_PAD_CLK_EN |
			   EMAC_RX_CLK_SEL | EMAC_RX_180_CLK_EN,
			   EMAC_RX_PAD_CLK_EN | EMAC_RX_CLK_SEL |
			   EMAC_RX_180_CLK_EN);

	regmap_update_bits(dwmac->cnnt_sys, CNNT_HP_EMAC_TX_CTRL,
			   EMAC_TX_PAD_CLK_EN | EMAC_TX_CLK_SEL |
			   EMAC_TX_180_CLK_EN, EMAC_TX_180_CLK_EN);

	return 0;
}

static int esp32s31_dwmac_probe(struct platform_device *pdev)
{
	struct plat_stmmacenet_data *plat_dat;
	struct stmmac_resources stmmac_res;
	struct esp32s31_dwmac *dwmac;
	struct stmmac_priv *priv;
	int rc;

	rc = stmmac_get_platform_resources(pdev, &stmmac_res);
	if (rc)
		return rc;

	plat_dat = devm_stmmac_probe_config_dt(pdev, stmmac_res.mac);
	if (IS_ERR(plat_dat))
		return PTR_ERR(plat_dat);

	dwmac = devm_kzalloc(&pdev->dev, sizeof(*dwmac), GFP_KERNEL);
	if (!dwmac)
		return -ENOMEM;

	dwmac->cnnt_sys = syscon_regmap_lookup_by_phandle(pdev->dev.of_node,
							  "esp,cnnt-sys");
	if (IS_ERR(dwmac->cnnt_sys))
		return dev_err_probe(&pdev->dev, PTR_ERR(dwmac->cnnt_sys),
				     "esp,cnnt-sys syscon\n");

	dwmac->clk_ref = devm_clk_get_enabled(&pdev->dev, "ref");
	if (IS_ERR(dwmac->clk_ref))
		return dev_err_probe(&pdev->dev, PTR_ERR(dwmac->clk_ref),
				     "ref clock\n");

	/* snps,dwmac-3.70a does not imply enhanced descriptors, and
	 * stmmac_dwmac1_quirks() picks the descriptor ops before
	 * stmmac_hw_init() can overwrite it from dma_cap.
	 */
	plat_dat->enh_desc = 1;
	plat_dat->bsp_priv = dwmac;
	plat_dat->init = esp32s31_dwmac_init;

	/* The core already knows how to drive an RGMII TX clock per speed. */
	plat_dat->clk_tx_i = dwmac->clk_ref;
	plat_dat->set_clk_tx_rate = stmmac_set_clk_tx_rate;

	rc = devm_stmmac_pltfr_probe(pdev, plat_dat, &stmmac_res);
	if (rc)
		return rc;

	/* Seed the ring sizes before the first open: stmmac_setup_dma_desc()
	 * uses priv->dma_conf.dma_{rx,tx}_size when non-zero, else the
	 * DMA_DEFAULT_{RX,TX}_SIZE. A later `ethtool -G eth0 rx/tx N` overrides.
	 */
	priv = netdev_priv(platform_get_drvdata(pdev));
	priv->dma_conf.dma_rx_size = ESP32S31_DMA_RX_SIZE;
	priv->dma_conf.dma_tx_size = ESP32S31_DMA_TX_SIZE;

	return 0;
}

static const struct of_device_id esp32s31_dwmac_match[] = {
	{ .compatible = "esp,esp32s31-dwmac" },
	{ }
};
MODULE_DEVICE_TABLE(of, esp32s31_dwmac_match);

static struct platform_driver esp32s31_dwmac_driver = {
	.probe = esp32s31_dwmac_probe,
	.driver = {
		.name = "dwmac-esp32s31",
		.pm = &stmmac_pltfr_pm_ops,
		.of_match_table = esp32s31_dwmac_match,
	},
};
module_platform_driver(esp32s31_dwmac_driver);

MODULE_DESCRIPTION("Espressif ESP32-S31 DWMAC glue");
MODULE_LICENSE("GPL");
