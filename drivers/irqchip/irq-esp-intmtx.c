// SPDX-License-Identifier: GPL-2.0
/*
 * Espressif ESP32-S31 INTMTX — peripheral-source -> CLIC-slot routing.
 */

#include <linux/bitmap.h>
#include <linux/cpumask.h>
#include <linux/io.h>
#include <linux/irq.h>
#include <linux/irqchip.h>
#include <linux/irqdomain.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/slab.h>
#include <linux/smp.h>
#include <linux/spinlock.h>

/* Per-source 32-bit register, low 6 bits = target CLIC slot. */
#define INTMTX_SRC_REG(src)	((src) * 4U)
#define INTMTX_SLOT_MASK	0x3FU

/* One window per core; the window a source is routed in picks its target core. */
#define INTMTX_WINDOW_SIZE	0x800U
#define INTMTX_WINDOW(cpu)	((cpu) * INTMTX_WINDOW_SIZE)

#define CLIC_SLOT_FIRST		16U
#define CLIC_SLOT_LAST		47U
#define CLIC_SLOT_COUNT		(CLIC_SLOT_LAST - CLIC_SLOT_FIRST + 1U)

#define INTMTX_NR_CORES		2U

/*
 * A per-core source has one instance per hart, each routed in that hart's own
 * window. Consumers name the hart-0 instance, sources[0].
 */
struct esp_intmtx_percpu_src {
	unsigned int sources[INTMTX_NR_CORES];
};

static const struct esp_intmtx_percpu_src esp_intmtx_percpu_srcs[] = {
	{ .sources = { 33U, 34U } },	/* systimer TARGETn alarm */
	{ .sources = { 65U, 66U } },	/* CPU_INT_FROM_CPU_n doorbell (IPIs) */
};

/*
 * Largest ETS source we'll accept from DT. The matrix has ~100 sources
 * on the chip; cap at 127 here defensively — anything beyond
 * INTMTX_MAX_SRC writes off the documented window.
 */
#define INTMTX_MAX_SRC		127U

struct esp_intmtx {
	void __iomem		*base;
	raw_spinlock_t		lock;
	unsigned int		nr_windows;
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

#ifdef CONFIG_SMP
/*
 * This is single-target routing hardware feeding a level source: only one
 * window may hold a source's slot at a time, and the receiving bank must be
 * ready before the source is pointed at it. Unroute the old window, let the
 * CLIC layer reprogram the new bank's attr/ctl (irq_chip_set_affinity_parent,
 * which also moves the effective-affinity record we both read), then route
 * the new window -- never the reverse order. The mask/unmask bracketing the
 * whole move mirrors __irq_move_irq(): masked while no window is correctly
 * configured, unmasked again only once the new one is. That bracket is also
 * what makes the relaxed writes below safe without barriers between the INTMTX
 * and CLIC register files: clicintie is 0 for the whole sequence, so no
 * ordering between a route and a bank write can expose a deliverable source.
 *
 * irqd_irq_masked() rather than __irq_move_irq()'s irqd_irq_disabled() is the
 * right predicate here: what has to be restored is the hardware enable this
 * function cleared, and that is exactly what the masked state records.
 */
static int esp_intmtx_set_affinity(struct irq_data *d,
				   const struct cpumask *mask_val, bool force)
{
	struct esp_intmtx *priv = irq_data_get_irq_chip_data(d);
	irq_hw_number_t src = d->hwirq;
	unsigned long flags;
	unsigned int cpu, old;
	bool was_masked;
	int ret;

	cpu = force ? cpumask_first(mask_val) :
		      cpumask_first_and(mask_val, cpu_online_mask);
	if (cpu >= nr_cpu_ids)
		return -EINVAL;

	old = cpumask_first(irq_data_get_effective_affinity_mask(d));
	if (old == cpu)
		return IRQ_SET_MASK_OK_DONE;

	was_masked = irqd_irq_masked(d);
	if (!was_masked)
		irq_chip_mask_parent(d);

	raw_spin_lock_irqsave(&priv->lock, flags);
	writel_relaxed(0, priv->base + INTMTX_WINDOW(old) + INTMTX_SRC_REG(src));
	raw_spin_unlock_irqrestore(&priv->lock, flags);

	ret = irq_chip_set_affinity_parent(d, cpumask_of(cpu), force);
	if (ret < 0) {
		raw_spin_lock_irqsave(&priv->lock, flags);
		writel_relaxed(priv->src_to_slot[src],
			       priv->base + INTMTX_WINDOW(old) + INTMTX_SRC_REG(src));
		raw_spin_unlock_irqrestore(&priv->lock, flags);
		if (!was_masked)
			irq_chip_unmask_parent(d);
		return ret;
	}

	raw_spin_lock_irqsave(&priv->lock, flags);
	writel_relaxed(priv->src_to_slot[src],
		       priv->base + INTMTX_WINDOW(cpu) + INTMTX_SRC_REG(src));
	raw_spin_unlock_irqrestore(&priv->lock, flags);

	if (!was_masked)
		irq_chip_unmask_parent(d);

	return IRQ_SET_MASK_OK_DONE;
}
#endif /* CONFIG_SMP */

static const struct esp_intmtx_percpu_src *
esp_intmtx_find_percpu_src(irq_hw_number_t src)
{
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(esp_intmtx_percpu_srcs); i++)
		if (esp_intmtx_percpu_srcs[i].sources[0] == src)
			return &esp_intmtx_percpu_srcs[i];

	return NULL;
}

static void esp_intmtx_percpu_route(struct irq_data *d, bool on)
{
	struct esp_intmtx *priv = irq_data_get_irq_chip_data(d);
	unsigned int cpu = smp_processor_id();
	unsigned int src = esp_intmtx_find_percpu_src(d->hwirq)->sources[cpu];
	unsigned long flags;

	raw_spin_lock_irqsave(&priv->lock, flags);
	writel_relaxed(on ? priv->src_to_slot[d->hwirq] : 0,
		       priv->base + INTMTX_WINDOW(cpu) + INTMTX_SRC_REG(src));
	raw_spin_unlock_irqrestore(&priv->lock, flags);
}

static void esp_intmtx_percpu_enable(struct irq_data *d)
{
	esp_intmtx_percpu_route(d, true);
	irq_chip_enable_parent(d);
}

static void esp_intmtx_percpu_disable(struct irq_data *d)
{
	irq_chip_disable_parent(d);
	esp_intmtx_percpu_route(d, false);
}

static struct irq_chip esp_intmtx_percpu_chip = {
	.name		= "esp-intmtx-percpu",
	.irq_enable	= esp_intmtx_percpu_enable,
	.irq_disable	= esp_intmtx_percpu_disable,
	.irq_mask	= irq_chip_mask_parent,
	.irq_unmask	= irq_chip_unmask_parent,
	.irq_set_type	= irq_chip_set_type_parent,
	.flags		= IRQCHIP_SKIP_SET_WAKE,
};

/*
 * Every hart routes its own instance of the source into the same slot, so
 * the slot is taken from the pool shared by all banks.
 */
static int esp_intmtx_alloc_percpu(struct irq_domain *domain, unsigned int virq,
				   irq_hw_number_t src, unsigned int type)
{
	struct esp_intmtx *priv = domain->host_data;
	struct irq_fwspec parent_fwspec;
	unsigned long flags;
	int slot, ret;

	raw_spin_lock_irqsave(&priv->lock, flags);
	if (priv->src_to_slot[src]) {
		raw_spin_unlock_irqrestore(&priv->lock, flags);
		return -EBUSY;
	}
	slot = esp_intmtx_alloc_slot(priv);
	if (slot >= 0)
		priv->src_to_slot[src] = (u8)slot;
	raw_spin_unlock_irqrestore(&priv->lock, flags);
	if (slot < 0)
		return slot;

	ret = irq_set_percpu_devid(virq);
	if (ret)
		goto err_free;

	irq_domain_set_info(domain, virq, src, &esp_intmtx_percpu_chip,
			    priv, handle_percpu_devid_irq, NULL, NULL);

	parent_fwspec.fwnode = domain->parent->fwnode;
	parent_fwspec.param_count = 2;
	parent_fwspec.param[0] = slot;
	parent_fwspec.param[1] = type;

	ret = irq_domain_alloc_irqs_parent(domain, virq, 1, &parent_fwspec);
	if (!ret)
		return 0;

err_free:
	raw_spin_lock_irqsave(&priv->lock, flags);
	priv->src_to_slot[src] = 0;
	esp_intmtx_free_slot(priv, slot);
	raw_spin_unlock_irqrestore(&priv->lock, flags);
	return ret;
}

static struct irq_chip esp_intmtx_chip = {
	.name		= "esp-intmtx",
	.irq_mask	= irq_chip_mask_parent,
	.irq_unmask	= irq_chip_unmask_parent,
	.irq_set_type	= irq_chip_set_type_parent,
#ifdef CONFIG_SMP
	.irq_set_affinity = esp_intmtx_set_affinity,
#endif
	.flags		= IRQCHIP_SKIP_SET_WAKE | IRQCHIP_AFFINITY_PRE_STARTUP,
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

	if (esp_intmtx_find_percpu_src(src))
		return esp_intmtx_alloc_percpu(domain, virq, src, type);

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
		       priv->base + INTMTX_WINDOW(0) + INTMTX_SRC_REG(src));
	priv->src_to_slot[src] = (u8)slot;
	raw_spin_unlock_irqrestore(&priv->lock, flags);

	irq_domain_set_info(domain, virq, src, &esp_intmtx_chip,
			    priv, handle_level_irq, NULL, NULL);

	irq_data_update_effective_affinity(irq_get_irq_data(virq), cpumask_of(0));

	parent_fwspec.fwnode = domain->parent->fwnode;
	parent_fwspec.param_count = 2;
	parent_fwspec.param[0] = slot;
	parent_fwspec.param[1] = type;

	ret = irq_domain_alloc_irqs_parent(domain, virq, 1, &parent_fwspec);
	if (ret) {
		raw_spin_lock_irqsave(&priv->lock, flags);
		writel_relaxed(0, priv->base + INTMTX_WINDOW(0) + INTMTX_SRC_REG(src));
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
	unsigned int src, slot, cpu;

	if (!d)
		return;

	src = d->hwirq;
	if (src > INTMTX_MAX_SRC)
		return;

	/* Routing lives wherever this irq's effective affinity currently points. */
	cpu = cpumask_first(irq_data_get_effective_affinity_mask(d));

	raw_spin_lock_irqsave(&priv->lock, flags);
	slot = priv->src_to_slot[src];
	/* Per-core routes are per hart and removed as each hart disables them. */
	if (!esp_intmtx_find_percpu_src(src))
		writel_relaxed(0, priv->base + INTMTX_WINDOW(cpu) + INTMTX_SRC_REG(src));
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
	struct resource res;
	unsigned int src, win;
	int ret;

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

	ret = of_address_to_resource(node, 0, &res);
	if (ret) {
		pr_err("esp-intmtx: %pOF: no reg\n", node);
		kfree(priv);
		return ret;
	}

	priv->base = ioremap(res.start, resource_size(&res));
	if (!priv->base) {
		pr_err("esp-intmtx: %pOF: failed to map regs\n", node);
		kfree(priv);
		return -ENXIO;
	}

	/* Every CPU needs its own window to route its interrupts in. */
	priv->nr_windows = num_possible_cpus();
	if (priv->nr_windows > INTMTX_NR_CORES ||
	    resource_size(&res) < priv->nr_windows * INTMTX_WINDOW_SIZE) {
		pr_err("esp-intmtx: %pOF: %pR has no window for each of %u CPUs\n",
		       node, &res, priv->nr_windows);
		iounmap(priv->base);
		kfree(priv);
		return -EINVAL;
	}

	raw_spin_lock_init(&priv->lock);

	for (win = 0; win < priv->nr_windows; win++)
		for (src = 0; src <= INTMTX_MAX_SRC; src++)
			writel_relaxed(0, priv->base + INTMTX_WINDOW(win) +
				      INTMTX_SRC_REG(src));

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

	pr_info("esp-intmtx: %u sources, parent %pOF; %u CLIC slots available, %u window%s\n",
		INTMTX_MAX_SRC + 1, parent, CLIC_SLOT_COUNT,
		priv->nr_windows, priv->nr_windows == 1 ? "" : "s");
	return 0;
}

IRQCHIP_DECLARE(esp32s31_intmtx, "esp,esp32s31-intmtx", esp_intmtx_init);
