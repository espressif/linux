// SPDX-License-Identifier: GPL-2.0
/*
 * ESP32-S31 systimer clocksource + clockevent.
 *
 * The systimer is an always-on, XTAL-fed counter (40 MHz / 2.5 = 16 MHz),
 * independent of CPU_CLK and free-running through WFI -- unlike the chip's
 * CPU-clocked CLINT MTIME that backs the SBI timer. OpenSBI enables the
 * peripheral clock (s31_systimer_clk_init); Linux owns the counter (UNIT0)
 * and a one-shot alarm (COMP0/TARGET0) here, so timekeeping and ticks need no
 * SBI ecall and no rdtime trap. The alarm IRQ arrives as a normal peripheral
 * interrupt via INTMTX -> CLIC (ETS source 33).
 *
 * 52-bit counter (20-bit HI + 32-bit LO); a snapshot is latched by pulsing
 * UNIT0_OP.UPDATE and waiting for VALID before reading HI/LO.
 */

#include <linux/clk.h>
#include <linux/clockchips.h>
#include <linux/clocksource.h>
#include <linux/interrupt.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <linux/sched_clock.h>

#define ST_CONF			0x00	/* b30 UNIT0 work_en, b27/28 stall, b24 TARGET0 work_en */
#define  ST_CONF_UNIT0_WORK_EN	BIT(30)
#define  ST_CONF_TARGET0_WORK_EN BIT(24)
#define ST_UNIT0_OP		0x04	/* b30 UPDATE, b29 VALID */
#define  ST_UNIT0_UPDATE	BIT(30)
#define  ST_UNIT0_VALID		BIT(29)
#define ST_TARGET0_HI		0x1c	/* bits [19:0] */
#define ST_TARGET0_LO		0x20
#define ST_TARGET0_CONF		0x34	/* b31 unit_sel(0=UNIT0), b30 period_mode(0=oneshot) */
#define  ST_TARGET0_UNIT_SEL	BIT(31)
#define  ST_TARGET0_PERIOD_MODE	BIT(30)
#define ST_UNIT0_VALUE_HI	0x40	/* bits [19:0] */
#define ST_UNIT0_VALUE_LO	0x44
#define ST_COMP0_LOAD		0x50	/* b0 load */
#define ST_INT_ENA		0x64	/* b0 TARGET0 */
#define ST_INT_CLR		0x6c	/* b0 TARGET0 */

#define ST_HI_MASK		0xfffffU
#define ST_COUNTER_BITS		52

static struct esp_systimer {
	void __iomem		*base;
	struct clock_event_device ce;
} systimer;

static u64 systimer_count(void)
{
	void __iomem *b = systimer.base;
	unsigned long flags;
	int guard = 100000;
	u32 hi, lo;

	/* Snapshot is a shared latch (UNIT0_OP); serialise against IRQs. */
	local_irq_save(flags);
	writel(readl(b + ST_UNIT0_OP) | ST_UNIT0_UPDATE, b + ST_UNIT0_OP);
	while (!(readl(b + ST_UNIT0_OP) & ST_UNIT0_VALID) && --guard)
		cpu_relax();
	hi = readl(b + ST_UNIT0_VALUE_HI) & ST_HI_MASK;
	lo = readl(b + ST_UNIT0_VALUE_LO);
	local_irq_restore(flags);

	return ((u64)hi << 32) | lo;
}

static u64 systimer_cs_read(struct clocksource *cs)
{
	return systimer_count();
}

static u64 notrace systimer_sched_read(void)
{
	return systimer_count();
}

static struct clocksource systimer_cs = {
	.name	= "esp32s31-systimer",
	.rating	= 450,	/* beat riscv_clocksource (400) so reads skip the rdtime trap */
	.read	= systimer_cs_read,
	.mask	= CLOCKSOURCE_MASK(ST_COUNTER_BITS),
	.flags	= CLOCK_SOURCE_IS_CONTINUOUS,
};

static int systimer_set_next_event(unsigned long delta, struct clock_event_device *ce)
{
	void __iomem *b = systimer.base;
	u64 tgt = systimer_count() + delta;

	writel(readl(b + ST_CONF) & ~ST_CONF_TARGET0_WORK_EN, b + ST_CONF);
	writel((u32)(tgt >> 32) & ST_HI_MASK, b + ST_TARGET0_HI);
	writel((u32)tgt, b + ST_TARGET0_LO);
	writel(readl(b + ST_TARGET0_CONF) & ~(ST_TARGET0_UNIT_SEL | ST_TARGET0_PERIOD_MODE),
	       b + ST_TARGET0_CONF);
	writel(BIT(0), b + ST_COMP0_LOAD);
	writel(readl(b + ST_CONF) | ST_CONF_TARGET0_WORK_EN, b + ST_CONF);
	return 0;
}

static int systimer_shutdown(struct clock_event_device *ce)
{
	void __iomem *b = systimer.base;

	writel(readl(b + ST_CONF) & ~ST_CONF_TARGET0_WORK_EN, b + ST_CONF);
	writel(readl(b + ST_INT_ENA) & ~BIT(0), b + ST_INT_ENA);
	writel(BIT(0), b + ST_INT_CLR);
	return 0;
}

static int systimer_set_oneshot(struct clock_event_device *ce)
{
	writel(readl(systimer.base + ST_INT_ENA) | BIT(0), systimer.base + ST_INT_ENA);
	return 0;
}

static irqreturn_t systimer_isr(int irq, void *dev_id)
{
	struct clock_event_device *ce = dev_id;
	void __iomem *b = systimer.base;

	/* One-shot: disarm before clearing so the level IRQ can't re-assert. */
	writel(readl(b + ST_CONF) & ~ST_CONF_TARGET0_WORK_EN, b + ST_CONF);
	writel(BIT(0), b + ST_INT_CLR);
	ce->event_handler(ce);
	return IRQ_HANDLED;
}

static int __init systimer_init(struct device_node *np)
{
	u32 rate = 16000000;
	int irq, ret;

	systimer.base = of_iomap(np, 0);
	if (!systimer.base)
		return -ENXIO;

	of_property_read_u32(np, "clock-frequency", &rate);

	/* UNIT0 free-running (OpenSBI already clocked + un-stalled it). */
	writel(readl(systimer.base + ST_CONF) | ST_CONF_UNIT0_WORK_EN,
	       systimer.base + ST_CONF);

	/*
	 * Register the counter-based clocksource + sched_clock FIRST: they need no
	 * IRQ, and timer-riscv.c gates off the riscv sched_clock when this driver
	 * is built -- so a later clockevent/IRQ failure must not leave the kernel
	 * with no clocksource/sched_clock. Once these are up we never tear them
	 * back down (sched_clock would dereference a freed mapping).
	 */
	ret = clocksource_register_hz(&systimer_cs, rate);
	if (ret) {
		iounmap(systimer.base);
		return ret;
	}
	sched_clock_register(systimer_sched_read, ST_COUNTER_BITS, rate);

	irq = irq_of_parse_and_map(np, 0);
	if (!irq) {
		pr_err("esp32s31-systimer: no IRQ; clockevent unavailable\n");
		return -EINVAL;
	}

	systimer.ce.name		= "esp32s31-systimer";
	systimer.ce.features		= CLOCK_EVT_FEAT_ONESHOT;
	/* Above riscv_timer's max rating (450 with Sstc): the systimer clockevent
	 * MUST win, or the kernel falls back to the CLINT clockevent whose MTIME
	 * freezes in WFI (FORCE_ON is intentionally not set) -> idle hang.
	 */
	systimer.ce.rating		= 500;
	systimer.ce.set_next_event	= systimer_set_next_event;
	systimer.ce.set_state_shutdown	= systimer_shutdown;
	systimer.ce.set_state_oneshot	= systimer_set_oneshot;
	systimer.ce.cpumask		= cpu_possible_mask;
	systimer.ce.irq			= irq;

	ret = request_irq(irq, systimer_isr, IRQF_TIMER | IRQF_IRQPOLL,
			  "esp32s31-systimer", &systimer.ce);
	if (ret) {
		pr_err("esp32s31-systimer: request_irq %d failed (%d)\n", irq, ret);
		return ret;
	}

	/* min_delta well above the snapshot read+program latency so a one-shot
	 * deadline can't be programmed already in the past.
	 */
	clockevents_config_and_register(&systimer.ce, rate, 0x10, 0x7fffffff);
	pr_info("esp32s31-systimer: %u Hz clocksource+clockevent, irq %d\n", rate, irq);
	return 0;
}

TIMER_OF_DECLARE(esp32s31_systimer, "esp,esp32s31-systimer", systimer_init);
