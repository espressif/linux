// SPDX-License-Identifier: GPL-2.0-only
/*
 * Espressif ESP32-S31 four-channel RMT TX controller.
 *
 * Author: Glauber Franco <gnfranco@gmail.com>
 */
#include <linux/bitfield.h>
#include <linux/clk.h>
#include <linux/completion.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/jiffies.h>
#include <linux/math64.h>
#include <linux/io.h>
#include <linux/mfd/syscon.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/of_platform.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>
#include <linux/reset.h>
#include <linux/soc/espressif/rmt.h>
#include <linux/spinlock.h>

#define RMT_CHANNELS 4
#define RMT_WORDS 48
#define RMT_CONF(n) (0x20 + 4 * (n))
#define RMT_STATUS(n) (0x50 + 4 * (n))
#define RMT_RAW 0x70
#define RMT_INT_ST 0x74
#define RMT_INT_EN 0x78
#define RMT_INT_CLR 0x7c
#define RMT_SYS 0xc0
#define RMT_REF_RST 0xc8
#define RMT_RAM(n) (0x800 + 4 * RMT_WORDS * (n))
#define RMT_TX_START BIT(0)
#define RMT_RD_RST BIT(1)
#define RMT_APB_RST BIT(2)
#define RMT_IDLE_EN BIT(6)
#define RMT_STOP BIT(7)
#define RMT_DIV GENMASK(15, 8)
#define RMT_MEM_SIZE GENMASK(19, 16)
#define RMT_UPDATE BIT(24)
#define RMT_END(n) BIT(n)
#define RMT_ERR(n) BIT(4 + (n))
#define RMT_IRQ_MASK(n) (RMT_END(n) | RMT_ERR(n))
#define RMT_MEM_LP 0x204
#define RMT_LP_MASK (BIT(2) | BIT(3))

struct esp_rmt;
struct esp_rmt_channel {
	struct esp_rmt *rmt;
	struct mutex tx_lock;
	struct completion done;
	unsigned int id;
	int result;
	bool claimed;
	bool active;
};

struct esp_rmt {
	struct device *dev;
	void __iomem *base;
	struct clk *clk;
	struct regmap *hp;
	struct mutex claim_lock;
	spinlock_t irq_lock;
	struct esp_rmt_channel channels[RMT_CHANNELS];
	int irq;
	u32 old_lp;
};

static irqreturn_t esp_rmt_irq(int irq, void *data)
{
	struct esp_rmt *rmt = data;
	unsigned long flags;
	u32 status;
	unsigned int i;

	spin_lock_irqsave(&rmt->irq_lock, flags);
	status = readl(rmt->base + RMT_INT_ST) & GENMASK(7, 0);
	if (!status) {
		spin_unlock_irqrestore(&rmt->irq_lock, flags);
		return IRQ_NONE;
	}
	writel(status, rmt->base + RMT_INT_CLR);
	for (i = 0; i < RMT_CHANNELS; i++) {
		struct esp_rmt_channel *ch = &rmt->channels[i];

		if (!(status & RMT_IRQ_MASK(i)) || !ch->active)
			continue;
		ch->result = status & RMT_ERR(i) ? -EIO : 0;
		ch->active = false;
		complete(&ch->done);
	}
	spin_unlock_irqrestore(&rmt->irq_lock, flags);
	return IRQ_HANDLED;
}

static void esp_rmt_release(void *data)
{
	struct esp_rmt_channel *ch = data;

	mutex_lock(&ch->rmt->claim_lock);
	ch->claimed = false;
	mutex_unlock(&ch->rmt->claim_lock);
}

struct esp_rmt_channel *devm_esp_rmt_tx_get(struct device *dev, u32 channel)
{
	struct esp_rmt *rmt;
	struct esp_rmt_channel *ch;
	int ret;

	if (!dev->parent || !of_device_is_compatible(dev->parent->of_node,
						     "esp,esp32s31-rmt"))
		return ERR_PTR(-ENODEV);
	rmt = dev_get_drvdata(dev->parent);
	if (!rmt)
		return ERR_PTR(-EPROBE_DEFER);
	if (channel >= RMT_CHANNELS)
		return ERR_PTR(-EINVAL);
	ch = &rmt->channels[channel];
	mutex_lock(&rmt->claim_lock);
	if (ch->claimed) {
		mutex_unlock(&rmt->claim_lock);
		return ERR_PTR(-EBUSY);
	}
	ch->claimed = true;
	mutex_unlock(&rmt->claim_lock);
	ret = devm_add_action_or_reset(dev, esp_rmt_release, ch);
	return ret ? ERR_PTR(ret) : ch;
}
EXPORT_SYMBOL_GPL(devm_esp_rmt_tx_get);

int esp_rmt_tx(struct esp_rmt_channel *ch, unsigned long tick_hz,
	       const u32 *symbols, size_t count)
{
	struct esp_rmt *rmt = ch->rmt;
	void __iomem *base = rmt->base;
	unsigned int id = ch->id;
	unsigned long flags, rate = clk_get_rate(rmt->clk);
	u32 conf, mask = RMT_IRQ_MASK(id), enable, div;
	unsigned int i;
	u64 ticks = 0;
	unsigned long timeout;
	int ret;

	if (!symbols || !tick_hz || !count || count > ESP_RMT_TX_MAX_SYMBOLS ||
	    rate % tick_hz || rate / tick_hz > 256 || rate < tick_hz)
		return -EINVAL;
	for (i = 0; i < count; i++) {
		u32 d0 = FIELD_GET(GENMASK(14, 0), symbols[i]);
		u32 d1 = FIELD_GET(GENMASK(30, 16), symbols[i]);

		if (!d0 || !d1)
			return -EINVAL;
		ticks += d0 + d1;
	}
	/* Allow the full encoded waveform duration plus scheduling margin. */
	timeout = msecs_to_jiffies(DIV_ROUND_UP_ULL(ticks * 1000, tick_hz) + 100);
	div = (rate / tick_hz) & 0xff;
	conf = RMT_IDLE_EN | FIELD_PREP(RMT_DIV, div) |
		FIELD_PREP(RMT_MEM_SIZE, 1);
	mutex_lock(&ch->tx_lock);
	reinit_completion(&ch->done);
	writel(conf | RMT_RD_RST | RMT_APB_RST, base + RMT_CONF(id));
	writel(conf, base + RMT_CONF(id));
	for (i = 0; i < count; i++)
		writel(symbols[i], base + RMT_RAM(id) + 4 * i);
	writel(0, base + RMT_RAM(id) + 4 * count);
	spin_lock_irqsave(&rmt->irq_lock, flags);
	writel(mask, base + RMT_INT_CLR);
	ch->active = true;
	ch->result = -EINPROGRESS;
	enable = readl(base + RMT_INT_EN);
	writel(enable | mask, base + RMT_INT_EN);
	writel(conf | RMT_UPDATE, base + RMT_CONF(id));
	writel(conf | RMT_TX_START, base + RMT_CONF(id));
	spin_unlock_irqrestore(&rmt->irq_lock, flags);
	if (!wait_for_completion_timeout(&ch->done, timeout))
		ret = -ETIMEDOUT;
	else
		ret = ch->result;
	spin_lock_irqsave(&rmt->irq_lock, flags);
	ch->active = false;
	writel(readl(base + RMT_INT_EN) & ~mask, base + RMT_INT_EN);
	if (ret) {
		dev_err_ratelimited(rmt->dev, "TX%u failed %d raw=%08x status=%08x\n",
				    id, ret, readl(base + RMT_RAW),
				    readl(base + RMT_STATUS(id)));
		writel(conf | RMT_STOP | RMT_UPDATE, base + RMT_CONF(id));
	}
	writel(mask, base + RMT_INT_CLR);
	spin_unlock_irqrestore(&rmt->irq_lock, flags);
	mutex_unlock(&ch->tx_lock);
	return ret;
}
EXPORT_SYMBOL_GPL(esp_rmt_tx);

static void esp_rmt_restore(void *data)
{
	struct esp_rmt *rmt = data;

	regmap_update_bits(rmt->hp, RMT_MEM_LP, RMT_LP_MASK, rmt->old_lp);
}

static int esp_rmt_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct esp_rmt *rmt;
	struct clk *apb;
	struct reset_control *rst;
	unsigned int i;
	int ret;

	rmt = devm_kzalloc(dev, sizeof(*rmt), GFP_KERNEL);
	if (!rmt)
		return -ENOMEM;
	rmt->dev = dev;
	mutex_init(&rmt->claim_lock);
	spin_lock_init(&rmt->irq_lock);
	for (i = 0; i < RMT_CHANNELS; i++) {
		rmt->channels[i].rmt = rmt;
		rmt->channels[i].id = i;
		mutex_init(&rmt->channels[i].tx_lock);
		init_completion(&rmt->channels[i].done);
	}
	rmt->base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(rmt->base))
		return PTR_ERR(rmt->base);
	apb = devm_clk_get_enabled(dev, "apb");
	if (IS_ERR(apb))
		return dev_err_probe(dev, PTR_ERR(apb), "APB clock\n");
	rmt->clk = devm_clk_get_enabled(dev, "rmt");
	if (IS_ERR(rmt->clk))
		return dev_err_probe(dev, PTR_ERR(rmt->clk), "RMT clock\n");
	rst = devm_reset_control_get_exclusive(dev, NULL);
	if (IS_ERR(rst))
		return dev_err_probe(dev, PTR_ERR(rst), "reset\n");
	ret = reset_control_assert(rst);
	if (ret)
		return ret;
	udelay(2);
	ret = reset_control_deassert(rst);
	if (ret)
		return ret;
	rmt->hp = syscon_regmap_lookup_by_phandle(dev->of_node, "esp,hp-system");
	if (IS_ERR(rmt->hp))
		return PTR_ERR(rmt->hp);
	ret = regmap_read(rmt->hp, RMT_MEM_LP, &rmt->old_lp);
	if (ret)
		return ret;
	ret = devm_add_action_or_reset(dev, esp_rmt_restore, rmt);
	if (ret)
		return ret;
	ret = regmap_update_bits(rmt->hp, RMT_MEM_LP, RMT_LP_MASK, BIT(3));
	if (ret)
		return ret;
	writel(readl(rmt->base + RMT_SYS) | BIT(0) | BIT(1), rmt->base + RMT_SYS);
	writel(0, rmt->base + RMT_INT_EN);
	writel(GENMASK(7, 0), rmt->base + RMT_INT_CLR);
	writel(GENMASK(3, 0), rmt->base + RMT_REF_RST);
	rmt->irq = platform_get_irq(pdev, 0);
	if (rmt->irq < 0)
		return rmt->irq;
	ret = devm_request_irq(dev, rmt->irq, esp_rmt_irq, 0, dev_name(dev), rmt);
	if (ret)
		return dev_err_probe(dev, ret, "RMT IRQ\n");
	platform_set_drvdata(pdev, rmt);
	ret = devm_of_platform_populate(dev);
	if (ret)
		return ret;
	dev_info(dev, "RMT TX: %u channels, IRQ %d, clock %lu Hz\n",
		 RMT_CHANNELS, rmt->irq, clk_get_rate(rmt->clk));
	return 0;
}

static const struct of_device_id esp_rmt_of_match[] = {
	{ .compatible = "esp,esp32s31-rmt" }, { }
};
MODULE_DEVICE_TABLE(of, esp_rmt_of_match);
static struct platform_driver esp_rmt_driver = {
	.probe = esp_rmt_probe,
	.driver = { .name = "esp32s31-rmt", .of_match_table = esp_rmt_of_match },
};
module_platform_driver(esp_rmt_driver);
MODULE_AUTHOR("Glauber Franco <gnfranco@gmail.com>");
MODULE_DESCRIPTION("Espressif ESP32-S31 four-channel RMT TX controller");
MODULE_LICENSE("GPL");
