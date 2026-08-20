// SPDX-License-Identifier: GPL-2.0
/*
 * Espressif ESP32-S31 GPIO controller
 *
 * Copyright (C) 2026 Espressif Systems (Shanghai) Co. Ltd.
 *
 * The pads are grouped in banks of 32, each bank with its own set of
 * level/enable/status registers and its own parent interrupt. Pad properties
 * -- the input buffer in particular -- live in the IO_MUX window owned by
 * pinctrl-esp, so direction changes go through the pinctrl gpio-range.
 */

#include <linux/bitops.h>
#include <linux/device.h>
#include <linux/errno.h>
#include <linux/gpio/driver.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/minmax.h>
#include <linux/mod_devicetable.h>
#include <linux/pinctrl/consumer.h>
#include <linux/platform_device.h>
#include <linux/property.h>
#include <linux/regmap.h>
#include <linux/slab.h>
#include <linux/soc/espressif/esp-gpio.h>
#include <linux/spinlock.h>

#define ESP_GPIO_PER_BANK	32
#define ESP_GPIO_NR_BANKS	2

#define ESP_GPIO_BANK(pin)		((pin) / ESP_GPIO_PER_BANK)
#define ESP_GPIO_PIN_IN_BANK(pin)	((pin) % ESP_GPIO_PER_BANK)
#define ESP_GPIO_GLOBAL(bank, p)	((bank) * ESP_GPIO_PER_BANK + (p))

/* GPIO_PINn_REG fields. */
#define ESP_GPIO_PIN_INT_TYPE_SHIFT	7
#define ESP_GPIO_PIN_INT_TYPE_MASK	(0x7 << ESP_GPIO_PIN_INT_TYPE_SHIFT)
#define ESP_GPIO_PIN_INT_ENA_SHIFT	13
#define ESP_GPIO_PIN_INT_ENA_BIT(bank)	BIT(ESP_GPIO_PIN_INT_ENA_SHIFT + (bank))

/* GPIO_PINn_REG.INT_TYPE encoding (3 bits). */
enum esp_gpio_int_type {
	ESP_GPIO_INT_TYPE_DISABLE	= 0,
	ESP_GPIO_INT_TYPE_POSEDGE	= 1,
	ESP_GPIO_INT_TYPE_NEGEDGE	= 2,
	ESP_GPIO_INT_TYPE_ANYEDGE	= 3,
	ESP_GPIO_INT_TYPE_LOW_LEVEL	= 4,
	ESP_GPIO_INT_TYPE_HIGH_LEVEL	= 5,
};

/* Per-bank register offsets within the GPIO core region. */
struct esp_gpio_reglayout {
	const u16 *out;		/* output level, readable */
	const u16 *out_set;	/* output level set   (W1TS) */
	const u16 *out_clr;	/* output level clear (W1TC) */
	const u16 *enable;	/* output enable (direction), readable */
	const u16 *en_set;	/* output enable set   (W1TS) */
	const u16 *en_clr;	/* output enable clear (W1TC) */
	const u16 *in;		/* input data */
	const u16 *status_clr;	/* interrupt status clear (W1TC) */
	const u16 *int_status;	/* interrupt status (read) */
	u16 pin_reg_base;	/* GPIO_PIN0_REG, stride 4 */
};

static const u16 esp_s31_out[ESP_GPIO_NR_BANKS]	= { 0x04, 0x10 };
static const u16 esp_s31_out_set[ESP_GPIO_NR_BANKS]	= { 0x08, 0x14 };
static const u16 esp_s31_out_clr[ESP_GPIO_NR_BANKS]	= { 0x0C, 0x18 };
static const u16 esp_s31_enable[ESP_GPIO_NR_BANKS]	= { 0x34, 0x40 };
static const u16 esp_s31_en_set[ESP_GPIO_NR_BANKS]	= { 0x38, 0x44 };
static const u16 esp_s31_en_clr[ESP_GPIO_NR_BANKS]	= { 0x3C, 0x48 };
static const u16 esp_s31_in[ESP_GPIO_NR_BANKS]		= { 0x64, 0x68 };
static const u16 esp_s31_status_clr[ESP_GPIO_NR_BANKS]	= { 0x7C, 0x88 };
/* Diagonal of GPIO_INT_<bank>_<bank>_REG: bank <n> routes to parent IRQ <n>. */
static const u16 esp_s31_int_status[ESP_GPIO_NR_BANKS]	= { 0xA4, 0xB8 };

static const struct esp_gpio_reglayout esp_gpio_s31_regs = {
	.out		= esp_s31_out,
	.out_set	= esp_s31_out_set,
	.out_clr	= esp_s31_out_clr,
	.enable		= esp_s31_enable,
	.en_set		= esp_s31_en_set,
	.en_clr		= esp_s31_en_clr,
	.in		= esp_s31_in,
	.status_clr	= esp_s31_status_clr,
	.int_status	= esp_s31_int_status,
	.pin_reg_base	= 0xF4,
};

struct esp_gpio_variant {
	const struct esp_gpio_reglayout *regs;
	unsigned int max_pins;
};

static const struct esp_gpio_variant esp_gpio_s31_variant = {
	.regs		= &esp_gpio_s31_regs,
	.max_pins	= 63,
};

struct esp_gpio {
	struct gpio_chip	gc;
	struct regmap		*regs;
	raw_spinlock_t		lock;
	const struct esp_gpio_variant *variant;
	unsigned int		nr_parent_irqs;
};

#define ESP_GPIO_PIN_REG(chip, pin)	\
	((chip)->variant->regs->pin_reg_base + (pin) * 4)

/* ---------- low-level helpers ---------- */

static inline bool esp_gpio_offset_valid(struct esp_gpio *chip, unsigned int offset)
{
	return offset < chip->variant->max_pins;
}

static unsigned int esp_gpio_bank_width(struct esp_gpio *chip, unsigned int bank)
{
	unsigned int base = bank * ESP_GPIO_PER_BANK;

	if (base >= chip->variant->max_pins)
		return 0;

	return min_t(unsigned int, ESP_GPIO_PER_BANK,
		     chip->variant->max_pins - base);
}

static int esp_gpio_set(struct gpio_chip *gc, unsigned int offset, int value)
{
	struct esp_gpio *chip = gpiochip_get_data(gc);
	const struct esp_gpio_reglayout *r = chip->variant->regs;
	unsigned int bank = ESP_GPIO_BANK(offset);
	unsigned int pin  = ESP_GPIO_PIN_IN_BANK(offset);

	if (!esp_gpio_offset_valid(chip, offset))
		return -EINVAL;

	return regmap_write(chip->regs,
			    value ? r->out_set[bank] : r->out_clr[bank],
			    BIT(pin));
}

static int esp_gpio_get(struct gpio_chip *gc, unsigned int offset)
{
	struct esp_gpio *chip = gpiochip_get_data(gc);
	const struct esp_gpio_reglayout *r = chip->variant->regs;
	unsigned int bank = ESP_GPIO_BANK(offset);
	unsigned int pin  = ESP_GPIO_PIN_IN_BANK(offset);
	unsigned int val, reg;
	int ret;

	if (!esp_gpio_offset_valid(chip, offset))
		return -EINVAL;

	ret = regmap_read(chip->regs, r->enable[bank], &val);
	if (ret)
		return ret;

	/* An output's input buffer is off, so report the level it drives. */
	reg = (val & BIT(pin)) ? r->out[bank] : r->in[bank];

	ret = regmap_read(chip->regs, reg, &val);
	if (ret) {
		dev_err(gc->parent, "failed to read GPIO level[%u]: %d\n", bank, ret);
		return ret;
	}

	return !!(val & BIT(pin));
}

static int esp_gpio_direction_input(struct gpio_chip *gc, unsigned int offset)
{
	struct esp_gpio *chip = gpiochip_get_data(gc);
	const struct esp_gpio_reglayout *r = chip->variant->regs;
	unsigned int bank = ESP_GPIO_BANK(offset);
	unsigned int pin  = ESP_GPIO_PIN_IN_BANK(offset);
	int ret;

	if (!esp_gpio_offset_valid(chip, offset))
		return -EINVAL;

	/* Stop driving the pad. */
	ret = regmap_write(chip->regs, r->en_clr[bank], BIT(pin));
	if (ret)
		return ret;

	/*
	 * The pad input buffer (direct_mux fun_ie) lives in the pinctrl
	 * region, so enable it through pinctrl to make GPIO_IN sample the pad.
	 */
	return pinctrl_gpio_direction_input(&chip->gc, offset);
}

static int esp_gpio_direction_output(struct gpio_chip *gc,
				     unsigned int offset, int value)
{
	struct esp_gpio *chip = gpiochip_get_data(gc);
	const struct esp_gpio_reglayout *r = chip->variant->regs;
	unsigned int bank = ESP_GPIO_BANK(offset);
	unsigned int pin  = ESP_GPIO_PIN_IN_BANK(offset);
	int ret;

	if (!esp_gpio_offset_valid(chip, offset))
		return -EINVAL;

	/* Programme the level before enabling the driver to avoid a glitch. */
	ret = esp_gpio_set(gc, offset, value);
	if (ret)
		return ret;

	ret = regmap_write(chip->regs, r->en_set[bank], BIT(pin));
	if (ret)
		return ret;

	return pinctrl_gpio_direction_output(&chip->gc, offset);
}

static int esp_gpio_get_direction(struct gpio_chip *gc, unsigned int offset)
{
	struct esp_gpio *chip = gpiochip_get_data(gc);
	const struct esp_gpio_reglayout *r = chip->variant->regs;
	unsigned int bank = ESP_GPIO_BANK(offset);
	unsigned int pin  = ESP_GPIO_PIN_IN_BANK(offset);
	unsigned int val;
	int ret;

	if (!esp_gpio_offset_valid(chip, offset))
		return -EINVAL;

	ret = regmap_read(chip->regs, r->enable[bank], &val);
	if (ret)
		return ret;

	return (val & BIT(pin)) ? GPIO_LINE_DIRECTION_OUT
				: GPIO_LINE_DIRECTION_IN;
}

/* ---------- irq_chip ---------- */

static void esp_gpio_irq_ack(struct irq_data *d)
{
	struct esp_gpio *chip = gpiochip_get_data(irq_data_get_irq_chip_data(d));
	const struct esp_gpio_reglayout *r = chip->variant->regs;
	unsigned int hwirq = irqd_to_hwirq(d);
	unsigned int bank = ESP_GPIO_BANK(hwirq);
	unsigned int pin  = ESP_GPIO_PIN_IN_BANK(hwirq);
	unsigned long flags;

	raw_spin_lock_irqsave(&chip->lock, flags);
	regmap_write(chip->regs, r->status_clr[bank], BIT(pin));
	raw_spin_unlock_irqrestore(&chip->lock, flags);
}

static void esp_gpio_irq_mask(struct irq_data *d)
{
	struct esp_gpio *chip = gpiochip_get_data(irq_data_get_irq_chip_data(d));
	unsigned int hwirq = irqd_to_hwirq(d);
	unsigned int bank = ESP_GPIO_BANK(hwirq);
	unsigned long flags;

	raw_spin_lock_irqsave(&esp_gpio_pin_reg_lock, flags);
	regmap_update_bits(chip->regs, ESP_GPIO_PIN_REG(chip, hwirq),
			   ESP_GPIO_PIN_INT_ENA_BIT(bank), 0);
	raw_spin_unlock_irqrestore(&esp_gpio_pin_reg_lock, flags);

	gpiochip_disable_irq(&chip->gc, hwirq);
}

static void esp_gpio_irq_unmask(struct irq_data *d)
{
	struct esp_gpio *chip = gpiochip_get_data(irq_data_get_irq_chip_data(d));
	unsigned int hwirq = irqd_to_hwirq(d);
	unsigned int bank = ESP_GPIO_BANK(hwirq);
	unsigned long flags;

	gpiochip_enable_irq(&chip->gc, hwirq);

	raw_spin_lock_irqsave(&esp_gpio_pin_reg_lock, flags);
	regmap_update_bits(chip->regs, ESP_GPIO_PIN_REG(chip, hwirq),
			   ESP_GPIO_PIN_INT_ENA_BIT(bank),
			   ESP_GPIO_PIN_INT_ENA_BIT(bank));
	raw_spin_unlock_irqrestore(&esp_gpio_pin_reg_lock, flags);
}

static int esp_gpio_irq_set_type(struct irq_data *d, unsigned int type)
{
	struct esp_gpio *chip = gpiochip_get_data(irq_data_get_irq_chip_data(d));
	const struct esp_gpio_reglayout *r = chip->variant->regs;
	unsigned int hwirq = irqd_to_hwirq(d);
	unsigned int bank = ESP_GPIO_BANK(hwirq);
	unsigned int pin  = ESP_GPIO_PIN_IN_BANK(hwirq);
	irq_flow_handler_t handler;
	unsigned int trig;
	unsigned long flags;

	switch (type) {
	case IRQ_TYPE_EDGE_RISING:
		trig = ESP_GPIO_INT_TYPE_POSEDGE;
		handler = handle_edge_irq;
		break;
	case IRQ_TYPE_EDGE_FALLING:
		trig = ESP_GPIO_INT_TYPE_NEGEDGE;
		handler = handle_edge_irq;
		break;
	case IRQ_TYPE_EDGE_BOTH:
		trig = ESP_GPIO_INT_TYPE_ANYEDGE;
		handler = handle_edge_irq;
		break;
	case IRQ_TYPE_LEVEL_HIGH:
		trig = ESP_GPIO_INT_TYPE_HIGH_LEVEL;
		handler = handle_level_irq;
		break;
	case IRQ_TYPE_LEVEL_LOW:
		trig = ESP_GPIO_INT_TYPE_LOW_LEVEL;
		handler = handle_level_irq;
		break;
	default:
		return -EINVAL;
	}

	raw_spin_lock_irqsave(&esp_gpio_pin_reg_lock, flags);

	/* IRQCHIP_SET_TYPE_MASKED: the core masks around this call. */
	regmap_write(chip->regs, r->status_clr[bank], BIT(pin));
	regmap_update_bits(chip->regs, ESP_GPIO_PIN_REG(chip, hwirq),
			   ESP_GPIO_PIN_INT_TYPE_MASK,
			   trig << ESP_GPIO_PIN_INT_TYPE_SHIFT);

	raw_spin_unlock_irqrestore(&esp_gpio_pin_reg_lock, flags);

	irq_set_handler_locked(d, handler);
	return 0;
}

/*
 * A line used only as an interrupt is never requested through gpiolib, so
 * nothing else turns its input buffer on. Its mux needs no help: pads reset to
 * the GPIO function, and a pad a pin group has muxed elsewhere is not free to
 * use as an interrupt anyway.
 */
static int esp_gpio_irq_reqres(struct irq_data *d)
{
	struct gpio_chip *gc = irq_data_get_irq_chip_data(d);
	unsigned int hwirq = irqd_to_hwirq(d);
	int ret;

	ret = gpiochip_reqres_irq(gc, hwirq);
	if (ret)
		return ret;

	ret = pinctrl_gpio_direction_input(gc, hwirq);
	if (ret)
		gpiochip_relres_irq(gc, hwirq);

	return ret;
}

static const struct irq_chip esp_gpio_irq_chip = {
	.name			= "esp-gpio",
	.irq_ack		= esp_gpio_irq_ack,
	.irq_mask		= esp_gpio_irq_mask,
	.irq_unmask		= esp_gpio_irq_unmask,
	.irq_set_type		= esp_gpio_irq_set_type,
	.irq_request_resources	= esp_gpio_irq_reqres,
	.irq_release_resources	= gpiochip_irq_relres,
	.flags			= IRQCHIP_IMMUTABLE | IRQCHIP_SET_TYPE_MASKED,
};

/* ---------- chained parent IRQ handler ---------- */

static void esp_gpio_irq_handler(struct irq_desc *desc)
{
	struct esp_gpio *chip = gpiochip_get_data(irq_desc_get_handler_data(desc));
	const struct esp_gpio_reglayout *r = chip->variant->regs;
	struct irq_chip *parent = irq_desc_get_chip(desc);
	struct irq_domain *dom = chip->gc.irq.domain;
	unsigned int parent_irq = irq_desc_get_irq(desc);
	unsigned int bank = UINT_MAX;
	unsigned int width, i;
	unsigned long status;
	unsigned int status32;
	int ret;

	/* Map parent irq back to bank index by DT order. */
	for (i = 0; i < chip->gc.irq.num_parents; i++) {
		if (chip->gc.irq.parents[i] == parent_irq) {
			bank = i;
			break;
		}
	}
	if (bank >= ESP_GPIO_NR_BANKS) {
		dev_err(chip->gc.parent, "spurious parent IRQ %u\n", parent_irq);
		return;
	}

	chained_irq_enter(parent, desc);

	ret = regmap_read(chip->regs, r->int_status[bank], &status32);
	if (ret) {
		dev_err(chip->gc.parent,
			"failed to read GPIO int status[%u]: %d\n", bank, ret);
		goto out;
	}

	width = esp_gpio_bank_width(chip, bank);
	if (!width)
		goto out;

	status = status32 & GENMASK(width - 1, 0);

	/* The flow handlers ack each line, which clears its status bit. */
	for_each_set_bit(i, &status, width)
		generic_handle_domain_irq(dom, ESP_GPIO_GLOBAL(bank, i));

out:
	chained_irq_exit(parent, desc);
}

/* ---------- probe ---------- */

static const struct regmap_config esp_gpio_regmap_config = {
	.reg_bits		= 32,
	.reg_stride		= 4,
	.val_bits		= 32,
	.fast_io		= true,
};

static int esp_gpio_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct esp_gpio *chip;
	struct gpio_irq_chip *girq;
	void __iomem *base;
	unsigned int nbanks;
	u32 ngpio;
	int ret, i;

	chip = devm_kzalloc(dev, sizeof(*chip), GFP_KERNEL);
	if (!chip)
		return -ENOMEM;

	chip->variant = device_get_match_data(dev);
	if (!chip->variant)
		return -ENODEV;

	raw_spin_lock_init(&chip->lock);

	base = devm_platform_ioremap_resource_byname(pdev, "gpio");
	if (IS_ERR(base))
		return dev_err_probe(dev, PTR_ERR(base),
				     "failed to map gpio region\n");

	chip->regs = devm_regmap_init_mmio(dev, base, &esp_gpio_regmap_config);
	if (IS_ERR(chip->regs))
		return dev_err_probe(dev, PTR_ERR(chip->regs),
				     "failed to init gpio regmap\n");

	ret = device_property_read_u32(dev, "ngpios", &ngpio);
	if (ret)
		return dev_err_probe(dev, ret, "missing ngpios property\n");

	if (!ngpio || ngpio > chip->variant->max_pins)
		return dev_err_probe(dev, -EINVAL,
				     "invalid ngpios=%u (max %u)\n",
				     ngpio, chip->variant->max_pins);

	nbanks = DIV_ROUND_UP(ngpio, ESP_GPIO_PER_BANK);

	chip->gc.label		  = dev_name(dev);
	chip->gc.parent		  = dev;
	chip->gc.owner		  = THIS_MODULE;
	chip->gc.base		  = -1;
	chip->gc.ngpio		  = ngpio;
	chip->gc.set		  = esp_gpio_set;
	chip->gc.get		  = esp_gpio_get;
	chip->gc.direction_input  = esp_gpio_direction_input;
	chip->gc.direction_output = esp_gpio_direction_output;
	chip->gc.can_sleep	  = false;

	/*
	 * gpiochip_generic_request / _free go through pinctrl_gpio_request()
	 * when the gpio node carries a 'gpio-ranges' pointing at pinctrl-esp,
	 * which is how we hand the pin off to / from the pinctrl side.
	 */
	chip->gc.request	= gpiochip_generic_request;
	chip->gc.free		= gpiochip_generic_free;

	chip->gc.get_direction = esp_gpio_get_direction;

	/* IRQ wiring: one parent IRQ per populated bank, same order as DT. */
	girq = &chip->gc.irq;
	gpio_irq_chip_set_chip(girq, &esp_gpio_irq_chip);

	girq->parents = devm_kcalloc(dev, ESP_GPIO_NR_BANKS,
				     sizeof(*girq->parents), GFP_KERNEL);
	if (!girq->parents)
		return -ENOMEM;

	for (i = 0; i < ESP_GPIO_NR_BANKS; i++) {
		int virq = platform_get_irq_optional(pdev, i);

		if (virq == -ENXIO)
			break;
		if (virq < 0)
			return virq;
		girq->parents[i] = virq;
	}
	girq->num_parents = i;

	if (!girq->num_parents) {
		dev_warn(dev, "no parent IRQs, GPIO interrupts disabled\n");
		girq->parents = NULL;
	} else if (girq->num_parents != nbanks) {
		/* Each populated bank needs its own parent IRQ. */
		return dev_err_probe(dev, -EINVAL,
				     "need %u parent IRQs for %u GPIOs, got %u\n",
				     nbanks, ngpio, girq->num_parents);
	}
	chip->nr_parent_irqs = girq->num_parents;

	girq->default_type   = IRQ_TYPE_NONE;
	girq->handler	     = handle_bad_irq;
	girq->parent_handler = esp_gpio_irq_handler;

	platform_set_drvdata(pdev, chip);

	return devm_gpiochip_add_data(dev, &chip->gc, chip);
}

static const struct of_device_id esp_gpio_of_match[] = {
	{ .compatible = "esp,esp32s31-gpio", .data = &esp_gpio_s31_variant },
	{ }
};
MODULE_DEVICE_TABLE(of, esp_gpio_of_match);

static struct platform_driver esp_gpio_driver = {
	.probe  = esp_gpio_probe,
	.driver = {
		.name		= "esp-gpio",
		.of_match_table	= esp_gpio_of_match,
	},
};
module_platform_driver(esp_gpio_driver);

MODULE_AUTHOR("Espressif Systems");
MODULE_DESCRIPTION("Espressif ESP32-S31 GPIO driver");
MODULE_LICENSE("GPL");
