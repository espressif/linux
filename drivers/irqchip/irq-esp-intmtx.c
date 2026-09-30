// SPDX-License-Identifier: GPL-2.0
/*
 * Espressif ESP32-S31 INTMTX — peripheral-source -> CLIC-slot routing.
 *
 * The chip's peripherals each drive a fixed "ETS source" line into the
 * INTMTX. The matrix is a flat 32-bit register per source: writing the
 * 6-bit CLIC slot number maps that source onto that slot; writing 0
 * disables routing. The CLIC then delivers interrupts on the chosen
 * slot to S-mode as a regular scause exccode, where irq-esp-clic
 * picks them up.
 *
 * This driver hangs underneath irq-esp-clic in the irq-domain
 * hierarchy. DT peripherals reference INTMTX with their ETS source
 * number + trigger type; alloc picks a free CLIC slot from a bitmap
 * (slots 16-31 — limited by riscv-intc's 32-hwirq cap on RV32),
 * programs the matrix, and forwards the request to the CLIC layer
 * with the slot number as parent hwirq. mask/unmask/set_type pass
 * through to the parent (CLIC) chip; INTMTX itself has no per-source
 * gate.
 *
 * Core0 view only — the chip has a parallel Core1 INTMTX at
 * 0x20585800, wired up later when SMP comes online.
 */

#include <linux/bitmap.h>
#include <linux/io.h>
#include <linux/irq.h>
#include <linux/irqchip.h>
#include <linux/irqdomain.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/slab.h>
#include <linux/spinlock.h>

/* Per-source 32-bit register, low 6 bits = target CLIC slot. */
#define INTMTX_SRC_REG(src)	((src) * 4U)
#define INTMTX_SLOT_MASK	0x3FU

/* Linux can use CLIC slots 16-31 today (irq-esp-clic limit). */
#define CLIC_SLOT_FIRST		16U
#define CLIC_SLOT_LAST		31U
#define CLIC_SLOT_COUNT		(CLIC_SLOT_LAST - CLIC_SLOT_FIRST + 1U)

/* Largest ETS source we'll accept from DT. The matrix has ~100 sources
 * on the chip; cap at 127 here defensively — anything beyond
 * INTMTX_MAX_SRC writes off the documented window.
 */
#define INTMTX_MAX_SRC		127U

struct esp_intmtx {
	void __iomem		*base;
	raw_spinlock_t		lock;
	DECLARE_BITMAP(slots_in_use, CLIC_SLOT_COUNT);
	u8			src_to_slot[INTMTX_MAX_SRC + 1];
};

static struct esp_intmtx *intmtx;

static int esp_intmtx_alloc_slot(struct esp_intmtx *priv)
{
	int bit;

	bit = find_first_zero_bit(priv->slots_in_use, CLIC_SLOT_COUNT);
	if (bit >= CLIC_SLOT_COUNT)
		return -ENOSPC;

	set_bit(bit, priv->slots_in_use);
	return CLIC_SLOT_FIRST + bit;
}

static void esp_intmtx_free_slot(struct esp_intmtx *priv, unsigned int slot)
{
	if (slot < CLIC_SLOT_FIRST || slot > CLIC_SLOT_LAST)
		return;
	clear_bit(slot - CLIC_SLOT_FIRST, priv->slots_in_use);
}

static struct irq_chip esp_intmtx_chip = {
	.name		= "esp-intmtx",
	.irq_mask	= irq_chip_mask_parent,
	.irq_unmask	= irq_chip_unmask_parent,
	.irq_set_type	= irq_chip_set_type_parent,
	.flags		= IRQCHIP_SKIP_SET_WAKE,
};

static int esp_intmtx_domain_alloc(struct irq_domain *domain,
				   unsigned int virq, unsigned int nr_irqs,
				   void *arg)
{
	struct irq_fwspec *fwspec = arg;
	struct irq_fwspec parent_fwspec;
	struct esp_intmtx *priv = domain->host_data;
	irq_hw_number_t src;
	unsigned int type;
	unsigned long flags;
	int slot, ret;

	if (nr_irqs != 1)
		return -EINVAL;

	ret = irq_domain_translate_twocell(domain, fwspec, &src, &type);
	if (ret)
		return ret;

	if (src > INTMTX_MAX_SRC)
		return -EINVAL;

	raw_spin_lock_irqsave(&priv->lock, flags);
	if (priv->src_to_slot[src]) {
		ret = -EBUSY;
		goto out;
	}

	slot = esp_intmtx_alloc_slot(priv);
	if (slot < 0) {
		ret = slot;
		goto out;
	}

	writel_relaxed((u32)slot & INTMTX_SLOT_MASK,
		       priv->base + INTMTX_SRC_REG(src));
	priv->src_to_slot[src] = (u8)slot;
	raw_spin_unlock_irqrestore(&priv->lock, flags);

	irq_domain_set_info(domain, virq, src, &esp_intmtx_chip,
			    priv, handle_level_irq, NULL, NULL);

	parent_fwspec.fwnode = domain->parent->fwnode;
	parent_fwspec.param_count = 2;
	parent_fwspec.param[0] = slot;
	parent_fwspec.param[1] = type;

	ret = irq_domain_alloc_irqs_parent(domain, virq, 1, &parent_fwspec);
	if (ret) {
		raw_spin_lock_irqsave(&priv->lock, flags);
		writel_relaxed(0, priv->base + INTMTX_SRC_REG(src));
		priv->src_to_slot[src] = 0;
		esp_intmtx_free_slot(priv, slot);
		raw_spin_unlock_irqrestore(&priv->lock, flags);
		return ret;
	}

	return 0;

out:
	raw_spin_unlock_irqrestore(&priv->lock, flags);
	return ret;
}

static void esp_intmtx_domain_free(struct irq_domain *domain,
				   unsigned int virq, unsigned int nr_irqs)
{
	struct esp_intmtx *priv = domain->host_data;
	struct irq_data *d = irq_domain_get_irq_data(domain, virq);
	unsigned long flags;
	unsigned int src, slot;

	if (!d)
		return;

	src = d->hwirq;
	if (src > INTMTX_MAX_SRC)
		return;

	raw_spin_lock_irqsave(&priv->lock, flags);
	slot = priv->src_to_slot[src];
	writel_relaxed(0, priv->base + INTMTX_SRC_REG(src));
	priv->src_to_slot[src] = 0;
	esp_intmtx_free_slot(priv, slot);
	raw_spin_unlock_irqrestore(&priv->lock, flags);

	irq_domain_free_irqs_common(domain, virq, nr_irqs);
}

static const struct irq_domain_ops esp_intmtx_domain_ops = {
	.translate	= irq_domain_translate_twocell,
	.alloc		= esp_intmtx_domain_alloc,
	.free		= esp_intmtx_domain_free,
};

static int __init esp_intmtx_init(struct device_node *node,
				  struct device_node *parent)
{
	struct irq_domain *parent_domain, *domain;
	struct esp_intmtx *priv;
	unsigned int src;

	if (intmtx) {
		pr_err("esp-intmtx: already initialised\n");
		return -EEXIST;
	}

	parent_domain = irq_find_host(parent);
	if (!parent_domain) {
		pr_err("esp-intmtx: %pOF: parent CLIC domain missing\n", node);
		return -ENXIO;
	}

	priv = kzalloc(sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	priv->base = of_iomap(node, 0);
	if (!priv->base) {
		pr_err("esp-intmtx: %pOF: failed to map regs\n", node);
		kfree(priv);
		return -ENXIO;
	}
	raw_spin_lock_init(&priv->lock);

	/* Clear every source's slot routing so we start from a known
	 * state — anything OpenSBI or the chip ROM left in place is
	 * unmapped here.
	 */
	for (src = 0; src <= INTMTX_MAX_SRC; src++)
		writel_relaxed(0, priv->base + INTMTX_SRC_REG(src));

	intmtx = priv;

	domain = irq_domain_create_hierarchy(parent_domain, 0,
					     INTMTX_MAX_SRC + 1,
					     of_fwnode_handle(node),
					     &esp_intmtx_domain_ops, priv);
	if (!domain) {
		iounmap(priv->base);
		kfree(priv);
		intmtx = NULL;
		return -ENOMEM;
	}

	pr_info("esp-intmtx: %u sources, parent %pOF; %u CLIC slots available\n",
		INTMTX_MAX_SRC + 1, parent, CLIC_SLOT_COUNT);
	return 0;
}

IRQCHIP_DECLARE(esp32s31_intmtx, "esp,esp32s31-intmtx", esp_intmtx_init);
