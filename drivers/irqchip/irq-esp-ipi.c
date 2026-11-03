// SPDX-License-Identifier: GPL-2.0
/*
 * Espressif inter-processor interrupts.
 *
 */

#define pr_fmt(fmt) KBUILD_MODNAME ": " fmt

#include <linux/cpu.h>
#include <linux/cpuhotplug.h>
#include <linux/io.h>
#include <linux/irq.h>
#include <linux/irqchip.h>
#include <linux/irqchip/chained_irq.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <linux/smp.h>

#define ESP_IPI_DOORBELL_REG(cpu)	((cpu) * 4U)
#define ESP_IPI_DOORBELL_BIT		BIT(0)

static struct {
	void __iomem	*doorbell;
	unsigned int	parent_virq;
} esp_ipi;

static void esp_ipi_send(unsigned int cpu)
{
	writel(ESP_IPI_DOORBELL_BIT,
	       esp_ipi.doorbell + ESP_IPI_DOORBELL_REG(cpu));
}

static void esp_ipi_clear(unsigned int cpu)
{
	writel(0, esp_ipi.doorbell + ESP_IPI_DOORBELL_REG(cpu));
}

static void esp_ipi_handle(struct irq_desc *desc)
{
	struct irq_chip *chip = irq_desc_get_chip(desc);

	chained_irq_enter(chip, desc);

	/*
	 * The doorbell is a level source with no write-1-to-clear, so clear it
	 * before processing. Clearing afterwards would race a sender that
	 * raised it while we were inside ipi_mux_process() and silently drop
	 * that IPI.
	 */
	esp_ipi_clear(smp_processor_id());
	ipi_mux_process();

	chained_irq_exit(chip, desc);
}

/*
 * enable_percpu_irq() on the starting hart is what routes its doorbell into
 * its own INTMTX window and programs the slot in its own CLIC bank, both
 * through the irq hierarchy; passing the trigger type is what gets the bank's
 * attributes reapplied there.
 */
static int esp_ipi_starting_cpu(unsigned int cpu)
{
	enable_percpu_irq(esp_ipi.parent_virq,
			  irq_get_trigger_type(esp_ipi.parent_virq));
	return 0;
}

/* Disabling on the dying hart also unroutes its doorbell, through intmtx. */
static int esp_ipi_dying_cpu(unsigned int cpu)
{
	esp_ipi_clear(cpu);
	disable_percpu_irq(esp_ipi.parent_virq);
	return 0;
}

static int __init esp_ipi_init(struct device_node *node,
			       struct device_node *parent)
{
	int mux;

	esp_ipi.doorbell = of_iomap(node, 0);
	if (!esp_ipi.doorbell) {
		pr_err("%pOF: failed to map the doorbells\n", node);
		return -ENXIO;
	}

	esp_ipi.parent_virq = irq_of_parse_and_map(node, 0);
	if (!esp_ipi.parent_virq) {
		pr_err("%pOF: no doorbell interrupt\n", node);
		goto err_unmap;
	}
	if (!irq_is_percpu_devid(esp_ipi.parent_virq)) {
		pr_err("%pOF: doorbell interrupt is not per-core\n", node);
		goto err_dispose;
	}

	mux = ipi_mux_create(BITS_PER_BYTE, esp_ipi_send);
	if (mux <= 0) {
		pr_err("unable to create muxed IPIs\n");
		goto err_dispose;
	}

	irq_set_chained_handler(esp_ipi.parent_virq, esp_ipi_handle);

	cpuhp_setup_state(CPUHP_AP_IRQ_ESP_IPI_STARTING,
			  "irqchip/esp-ipi:starting", esp_ipi_starting_cpu,
			  esp_ipi_dying_cpu);

	riscv_ipi_set_virq_range(mux, BITS_PER_BYTE);
	pr_info("IPIs on the CPU_FROM_CPU doorbells, irq %u\n",
		esp_ipi.parent_virq);
	return 0;

err_dispose:
	irq_dispose_mapping(esp_ipi.parent_virq);
err_unmap:
	esp_ipi.parent_virq = 0;
	iounmap(esp_ipi.doorbell);
	esp_ipi.doorbell = NULL;
	return -ENXIO;
}

IRQCHIP_DECLARE(esp32s31_ipi, "esp,esp32s31-ipi", esp_ipi_init);
