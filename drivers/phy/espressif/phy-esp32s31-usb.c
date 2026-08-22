// SPDX-License-Identifier: GPL-2.0
/*
 * Espressif ESP32-S31 USB 2.0 OTG-HS PHY
 *
 * Copyright (C) 2026 Espressif Systems (Shanghai) Co. Ltd.
 */

#include <linux/bitfield.h>
#include <linux/bitops.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/io.h>
#include <linux/mfd/syscon.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/phy/phy.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>
#include <linux/usb/of.h>
#include <linux/usb/otg.h>

/* HP_ALIVE_SYS USB_OTGHS_CTRL */
#define ESP_USB_ALIVE_OTGHS_CTRL	0x0b0
#define ESP_USB_PHY_PLL_FORCE_EN	BIT(0)
#define ESP_USB_PHY_PLL_EN		BIT(1)
#define ESP_USB_PHY_SUSPENDM_FORCE_EN	BIT(2)
#define ESP_USB_PHY_SUSPENDM		BIT(3)
#define ESP_USB_PHY_OTG_SUSPENDM	BIT(7)
#define ESP_USB_PHY_REFCLK_MODE		BIT(8)

/* HP_ALIVE_SYS USB_CTRL */
#define ESP_USB_ALIVE_USB_CTRL		0x024
#define ESP_USB_PHY_DM_PULLDOWN		BIT(2)
#define ESP_USB_PHY_DP_PULLDOWN		BIT(3)

/* CNNT_SYS USB_OTG20_CTRL */
#define ESP_USB_CNNT_OTG20_CTRL		0x030
#define ESP_USB_PHYREF_CLK_SRC_SEL	GENMASK(26, 25)
#define ESP_USB_PHYREF_CLK_SRC_12M	0

/*
 * FC register 6 of the PHY's own window: low-speed support. The rest of
 * the file is analog tuning this driver leaves at its reset value.
 */
#define ESP_USB_UTMI_FC_06		0x018
#define ESP_USB_UTMI_LS_PAR_EN		BIT(0)
#define ESP_USB_UTMI_LS_KPALV_EN	BIT(3)

/* The PHY PLL needs a moment to lock before the controller resets itself. */
#define ESP_USB_PHY_PLL_LOCK_US		50

struct esp_usb_phy {
	void __iomem *utmi;
	struct regmap *alive_sys;
	struct regmap *cnnt_sys;
	struct clk *ref;
	enum usb_dr_mode dr_mode;
};

static void esp_usb_phy_utmi_set(struct esp_usb_phy *priv, u32 reg, u32 mask)
{
	writel(readl(priv->utmi + reg) | mask, priv->utmi + reg);
}

static int esp_usb_phy_power_on(struct phy *phy)
{
	struct esp_usb_phy *priv = phy_get_drvdata(phy);
	u32 mask;
	int ret;

	/*
	 * Pick the 12 MHz reference on both sides that describe it. These are
	 * the power-on values; writing them keeps the source select and the
	 * PHY's refclk mode from being able to disagree.
	 */
	ret = regmap_update_bits(priv->cnnt_sys, ESP_USB_CNNT_OTG20_CTRL,
				 ESP_USB_PHYREF_CLK_SRC_SEL,
				 FIELD_PREP(ESP_USB_PHYREF_CLK_SRC_SEL,
					    ESP_USB_PHYREF_CLK_SRC_12M));
	if (ret)
		return ret;

	ret = regmap_update_bits(priv->alive_sys, ESP_USB_ALIVE_OTGHS_CTRL,
				 ESP_USB_PHY_REFCLK_MODE,
				 ESP_USB_PHY_REFCLK_MODE);
	if (ret)
		return ret;

	ret = clk_prepare_enable(priv->ref);
	if (ret)
		return ret;

	/*
	 * Out of reset the PHY is force-suspended with its PLL under hardware
	 * control, so take over both: suspendm is active low, hence set to
	 * leave suspend.
	 */
	mask = ESP_USB_PHY_PLL_FORCE_EN | ESP_USB_PHY_PLL_EN |
	       ESP_USB_PHY_SUSPENDM_FORCE_EN | ESP_USB_PHY_SUSPENDM |
	       ESP_USB_PHY_OTG_SUSPENDM;
	ret = regmap_update_bits(priv->alive_sys, ESP_USB_ALIVE_OTGHS_CTRL,
				 mask, mask);
	if (ret)
		goto err_clk;

	udelay(ESP_USB_PHY_PLL_LOCK_US);

	/* D+/D- pulled down for the host idle line state. */
	mask = ESP_USB_PHY_DP_PULLDOWN | ESP_USB_PHY_DM_PULLDOWN;
	ret = regmap_update_bits(priv->alive_sys, ESP_USB_ALIVE_USB_CTRL, mask,
				 priv->dr_mode == USB_DR_MODE_HOST ? mask : 0);
	if (ret)
		goto err_clk;

	esp_usb_phy_utmi_set(priv, ESP_USB_UTMI_FC_06,
			     ESP_USB_UTMI_LS_PAR_EN | ESP_USB_UTMI_LS_KPALV_EN);

	return 0;

err_clk:
	clk_disable_unprepare(priv->ref);

	return ret;
}

static int esp_usb_phy_power_off(struct phy *phy)
{
	struct esp_usb_phy *priv = phy_get_drvdata(phy);

	/* Re-suspend, but keep the force enables so the state stays ours. */
	regmap_update_bits(priv->alive_sys, ESP_USB_ALIVE_OTGHS_CTRL,
			   ESP_USB_PHY_SUSPENDM | ESP_USB_PHY_OTG_SUSPENDM |
			   ESP_USB_PHY_PLL_EN, 0);

	regmap_update_bits(priv->alive_sys, ESP_USB_ALIVE_USB_CTRL,
			   ESP_USB_PHY_DP_PULLDOWN | ESP_USB_PHY_DM_PULLDOWN, 0);

	clk_disable_unprepare(priv->ref);

	return 0;
}

static const struct phy_ops esp_usb_phy_ops = {
	.power_on = esp_usb_phy_power_on,
	.power_off = esp_usb_phy_power_off,
	.owner = THIS_MODULE,
};

static int esp_usb_phy_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct phy_provider *provider;
	struct esp_usb_phy *priv;
	struct phy *phy;

	priv = devm_kzalloc(dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	priv->dr_mode = of_usb_get_dr_mode_by_phy(dev->of_node, -1);
	if (priv->dr_mode == USB_DR_MODE_UNKNOWN)
		priv->dr_mode = USB_DR_MODE_HOST;

	priv->utmi = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(priv->utmi))
		return PTR_ERR(priv->utmi);

	priv->alive_sys = syscon_regmap_lookup_by_phandle(dev->of_node,
							 "esp,hp-alive-sys");
	if (IS_ERR(priv->alive_sys))
		return dev_err_probe(dev, PTR_ERR(priv->alive_sys),
				     "failed to get hp-alive-sys regmap\n");

	priv->cnnt_sys = syscon_regmap_lookup_by_phandle(dev->of_node,
							 "esp,cnnt-sys");
	if (IS_ERR(priv->cnnt_sys))
		return dev_err_probe(dev, PTR_ERR(priv->cnnt_sys),
				     "failed to get cnnt-sys regmap\n");

	priv->ref = devm_clk_get(dev, "ref");
	if (IS_ERR(priv->ref))
		return dev_err_probe(dev, PTR_ERR(priv->ref),
				     "failed to get reference clock\n");

	phy = devm_phy_create(dev, NULL, &esp_usb_phy_ops);
	if (IS_ERR(phy))
		return dev_err_probe(dev, PTR_ERR(phy),
				     "failed to create phy\n");

	phy_set_drvdata(phy, priv);

	provider = devm_of_phy_provider_register(dev, of_phy_simple_xlate);

	return PTR_ERR_OR_ZERO(provider);
}

static const struct of_device_id esp_usb_phy_of_match[] = {
	{ .compatible = "esp,esp32s31-usb-phy" },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, esp_usb_phy_of_match);

static struct platform_driver esp_usb_phy_driver = {
	.probe	= esp_usb_phy_probe,
	.driver	= {
		.name		= "esp32s31-usb-phy",
		.of_match_table	= esp_usb_phy_of_match,
	},
};
module_platform_driver(esp_usb_phy_driver);

MODULE_AUTHOR("Espressif Systems");
MODULE_DESCRIPTION("Espressif ESP32-S31 USB 2.0 PHY");
MODULE_LICENSE("GPL");
