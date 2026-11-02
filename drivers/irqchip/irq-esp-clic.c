// SPDX-License-Identifier: GPL-2.0
/*
 * Espressif ESP32-S31 CLIC irqchip — S-mode view of the chip's CLIC.
 */

#include <linux/io.h>
#include <linux/irq.h>
#include <linux/irqchip.h>
#include <linux/irqchip/chained_irq.h>
#include <linux/irqdomain.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/slab.h>
#include <linux/smp.h>

/* Supervisor bank per-source byte layout (mirror of M-bank in OpenSBI):
 *   +0 clicintip   pending bit
 *   +1 clicintie   interrupt enable
 *   +2 clicintattr [7:6] mode, [2:1] trigger, [0] SHV
 *   +3 clicintctl  priority/level
 */
#define CLIC_CTRL_OFF		0x1000U

#define CLIC_INTIP_OFF(s)	(CLIC_CTRL_OFF + (s) * 4U + 0U)
#define CLIC_INTIE_OFF(s)	(CLIC_CTRL_OFF + (s) * 4U + 1U)
#define CLIC_INTATTR_OFF(s)	(CLIC_CTRL_OFF + (s) * 4U + 2U)
#define CLIC_INTCTL_OFF(s)	(CLIC_CTRL_OFF + (s) * 4U + 3U)

#define CLIC_ATTR_TRIG_MASK	0x6U
#define CLIC_ATTR_TRIG_LEVEL	0x0U

/*
 * OpenSBI leaves cliccfg.snlbits at 0, so every S-mode source decodes to the
 * same interrupt level and clicintctl only orders simultaneously pending
 * sources. All of them get the same value: no interrupt preempts another.
 */
#define CLIC_CTL_MAX_PRIO	0xE0U

#define CLIC_EXT_FIRST		16U
#define CLIC_EXT_LAST		47U
#define CLIC_EXT_COUNT		(CLIC_EXT_LAST - CLIC_EXT_FIRST + 1U)

/* clicintattr with mode=01 (S-mode) and trigger=level. */
#define CLIC_ATTR_S_LEVEL	0x40U

struct esp_clic {
	void __iomem		*base;
	struct irq_domain	*domain;
	struct irq_domain	*parent_domain;
	unsigned int		intc_virqs[CLIC_EXT_COUNT];
#ifdef CONFIG_SMP
	/* The peer core's bank, through this core's Other aperture. */
	void __iomem		*peer_base;
#endif
};

static struct esp_clic *clic;

static inline unsigned int slot_to_idx(unsigned int slot)
{
	return slot - CLIC_EXT_FIRST;
}

static void __iomem *esp_clic_bank_for(struct esp_clic *priv, unsigned int cpu)
{
#ifdef CONFIG_SMP
	if (cpu != raw_smp_processor_id())
		return priv->peer_base;
#endif
	return priv->base;
}

static unsigned int esp_clic_target_cpu(struct irq_data *d)
{
	return cpumask_first(irq_data_get_effective_affinity_mask(d));
}

/*
 * Called with the irq descriptor lock held and interrupts off, so the bank
 * resolved for the irq's current target CPU cannot go stale before the
 * write below.
 */
static void esp_clic_set_ie(struct irq_data *d, bool enable)
{
	void __iomem *bank = esp_clic_bank_for(clic, esp_clic_target_cpu(d));

	writeb_relaxed(enable ? 1 : 0, bank + CLIC_INTIE_OFF(d->hwirq));
}

static void esp_clic_mask(struct irq_data *d)
{
	esp_clic_set_ie(d, false);
}

static void esp_clic_unmask(struct irq_data *d)
{
	esp_clic_set_ie(d, true);
}

/* Same locking guarantee as esp_clic_set_ie() above. */
static int esp_clic_set_type(struct irq_data *d, unsigned int type)
{
	void __iomem *bank;
	u8 attr;

	if (type != IRQ_TYPE_LEVEL_HIGH)
		return -EINVAL;

	bank = esp_clic_bank_for(clic, esp_clic_target_cpu(d));
	attr = readb_relaxed(bank + CLIC_INTATTR_OFF(d->hwirq));
	attr = (attr & ~CLIC_ATTR_TRIG_MASK) | CLIC_ATTR_TRIG_LEVEL;
	writeb_relaxed(attr, bank + CLIC_INTATTR_OFF(d->hwirq));
	irq_set_handler_locked(d, handle_level_irq);
	return 0;
}

#ifdef CONFIG_SMP
static int esp_clic_set_affinity(struct irq_data *d,
				 const struct cpumask *mask_val, bool force)
{
	void __iomem *bank;
	unsigned int cpu, old;
	u8 attr;

	cpu = force ? cpumask_first(mask_val) :
		cpumask_first_and(mask_val, cpu_online_mask);
	if (cpu >= nr_cpu_ids)
		return -EINVAL;

	old = esp_clic_target_cpu(d);
	if (old == cpu)
		return IRQ_SET_MASK_OK_DONE;

	writeb_relaxed(0, esp_clic_bank_for(clic, old) +
			  CLIC_INTIP_OFF(d->hwirq));

	irq_data_update_effective_affinity(d, cpumask_of(cpu));

	bank = esp_clic_bank_for(clic, cpu);
	attr = readb_relaxed(bank + CLIC_INTATTR_OFF(d->hwirq));
	attr = (attr & ~CLIC_ATTR_TRIG_MASK) | CLIC_ATTR_TRIG_LEVEL;
	writeb_relaxed(attr, bank + CLIC_INTATTR_OFF(d->hwirq));
	writeb_relaxed(CLIC_CTL_MAX_PRIO, bank + CLIC_INTCTL_OFF(d->hwirq));
	writeb_relaxed(0, bank + CLIC_INTIP_OFF(d->hwirq));

	return IRQ_SET_MASK_OK_DONE;
}
#endif /* CONFIG_SMP */

static struct irq_chip esp_clic_chip = {
	.name		= "esp-clic",
	.irq_mask	= esp_clic_mask,
	.irq_unmask	= esp_clic_unmask,
	.irq_set_type	= esp_clic_set_type,
#ifdef CONFIG_SMP
	.irq_set_affinity = esp_clic_set_affinity,
#endif
	.flags		= IRQCHIP_SKIP_SET_WAKE,
};

static void esp_clic_percpu_set_ie(struct irq_data *d, bool enable)
{
	writeb_relaxed(enable ? 1 : 0, clic->base + CLIC_INTIE_OFF(d->hwirq));
}

static void esp_clic_percpu_mask(struct irq_data *d)
{
	esp_clic_percpu_set_ie(d, false);
}

static void esp_clic_percpu_unmask(struct irq_data *d)
{
	esp_clic_percpu_set_ie(d, true);
}

static int esp_clic_percpu_set_type(struct irq_data *d, unsigned int type)
{
	if (type != IRQ_TYPE_LEVEL_HIGH)
		return -EINVAL;

	writeb_relaxed(CLIC_ATTR_S_LEVEL,
		       clic->base + CLIC_INTATTR_OFF(d->hwirq));
	writeb_relaxed(CLIC_CTL_MAX_PRIO,
		       clic->base + CLIC_INTCTL_OFF(d->hwirq));
	return 0;
}

static struct irq_chip esp_clic_percpu_chip = {
	.name		= "esp-clic-percpu",
	.irq_mask	= esp_clic_percpu_mask,
	.irq_unmask	= esp_clic_percpu_unmask,
	.irq_set_type	= esp_clic_percpu_set_type,
};

/*
 * Chained handler — runs in interrupt context when scause lands on a
 * CLIC slot we own. The intc-layer descriptor's hwirq is the slot ID,
 * which is also our CLIC domain's hwirq, so a straight forward.
 */
static void esp_clic_chained_handler(struct irq_desc *desc)
{
	struct irq_chip *chip = irq_desc_get_chip(desc);
	unsigned int slot = irq_desc_get_irq_data(desc)->hwirq;

	chained_irq_enter(chip, desc);
	generic_handle_domain_irq(clic->domain, slot);
	chained_irq_exit(chip, desc);
}

static int esp_clic_install_chained(struct esp_clic *priv, unsigned int slot)
{
	unsigned int idx = slot_to_idx(slot);
	unsigned int intc_virq;

	/* Kept across a domain free, so a re-used slot finds its handler here. */
	if (priv->intc_virqs[idx])
		return 0;

	intc_virq = irq_create_mapping(priv->parent_domain, slot);
	if (!intc_virq) {
		pr_err("esp-clic: failed to map intc IRQ for slot %u\n", slot);
		return -ENXIO;
	}

	irq_set_chained_handler_and_data(intc_virq, esp_clic_chained_handler,
					 priv);
	priv->intc_virqs[idx] = intc_virq;
	return 0;
}

static int esp_clic_domain_alloc(struct irq_domain *domain, unsigned int virq,
				 unsigned int nr_irqs, void *arg)
{
	struct irq_fwspec *fwspec = arg;
	struct esp_clic *priv = domain->host_data;
	irq_hw_number_t hwirq;
	unsigned int type;
	int i, ret;

	ret = irq_domain_translate_twocell(domain, fwspec, &hwirq, &type);
	if (ret)
		return ret;

	if (hwirq < CLIC_EXT_FIRST || hwirq + nr_irqs - 1 > CLIC_EXT_LAST)
		return -EINVAL;

	for (i = 0; i < nr_irqs; i++) {
		irq_hw_number_t src = hwirq + i;
		void __iomem *bank;

		if (irq_is_percpu_devid(virq + i)) {
			irq_domain_set_info(domain, virq + i, src,
					    &esp_clic_percpu_chip, priv,
					    handle_percpu_devid_irq, NULL,
					    NULL);

			ret = esp_clic_install_chained(priv, src);
			if (ret)
				return ret;

			continue;
		}

		/* Set the effective affinity to CPU0 by default */
		irq_data_update_effective_affinity(irq_get_irq_data(virq + i),
						   cpumask_of(0));

		/* Preemption off so the Self/Other choice cannot go stale. */
		preempt_disable();
		bank = esp_clic_bank_for(priv, 0);

		/*
		 * Clear any pending bit latched by ROM/OpenSBI before this slot
		 * gets a handler, so the first unmask doesn't take a spurious
		 * interrupt.
		 */
		writeb_relaxed(0, bank + CLIC_INTIP_OFF(src));
		writeb_relaxed(CLIC_CTL_MAX_PRIO, bank + CLIC_INTCTL_OFF(src));
		preempt_enable();

		irq_domain_set_info(domain, virq + i, src, &esp_clic_chip,
				    priv, handle_level_irq, NULL, NULL);

		ret = esp_clic_install_chained(priv, src);
		if (ret)
			return ret;
	}

	return 0;
}

static const struct irq_domain_ops esp_clic_domain_ops = {
	.translate	= irq_domain_translate_twocell,
	.alloc		= esp_clic_domain_alloc,
	.free		= irq_domain_free_irqs_common,
};

static int __init esp_clic_init(struct device_node *node,
				struct device_node *parent)
{
	struct irq_domain *parent_domain, *domain;
	struct esp_clic *priv;
	int ret;

	if (clic) {
		pr_err("esp-clic: already initialised\n");
		return -EEXIST;
	}

	parent_domain = irq_find_host(parent);
	if (!parent_domain) {
		pr_err("esp-clic: %pOF: parent intc domain missing\n", node);
		return -ENXIO;
	}

	priv = kzalloc(sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	priv->base = of_iomap(node, 0);
	if (!priv->base) {
		pr_err("esp-clic: %pOF: failed to map regs\n", node);
		ret = -ENXIO;
		goto err_free;
	}
	priv->parent_domain = parent_domain;

#ifdef CONFIG_SMP
	priv->peer_base = of_iomap(node, 1);
	if (!priv->peer_base && num_possible_cpus() > 1) {
		pr_err("esp-clic: %pOF: failed to map peer bank (reg 1)\n", node);
		ret = -ENXIO;
		goto err_unmap;
	}
#endif

	domain = irq_domain_create_linear(of_fwnode_handle(node),
					  CLIC_EXT_LAST + 1,
					  &esp_clic_domain_ops, priv);
	if (!domain) {
		ret = -ENOMEM;
		goto err_unmap;
	}

	priv->domain = domain;
	clic = priv;

	pr_info("esp-clic: %u external slots (IDs %u-%u), chained on %pOF\n",
		CLIC_EXT_COUNT, CLIC_EXT_FIRST, CLIC_EXT_LAST, parent);
	return 0;

err_unmap:
#ifdef CONFIG_SMP
	if (priv->peer_base)
		iounmap(priv->peer_base);
#endif
	iounmap(priv->base);
err_free:
	kfree(priv);
	return ret;
}

IRQCHIP_DECLARE(esp32s31_clic, "esp,esp32s31-clic", esp_clic_init);
