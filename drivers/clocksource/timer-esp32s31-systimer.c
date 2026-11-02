// SPDX-License-Identifier: GPL-2.0
/*
 * ESP32-S31 systimer clocksource + clockevent.
 */

#include <linux/clk.h>
#include <linux/clockchips.h>
#include <linux/clocksource.h>
#include <linux/cpu.h>
#include <linux/cpuhotplug.h>
#include <linux/interrupt.h>
#include <linux/irqchip.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <linux/sched_clock.h>

/* b30 UNIT0 work_en, b27/28 stall, b24/b23/b22 TARGET0/1/2 work_en */
#define ST_CONF					0x00
#define ST_CONF_UNIT0_WORK_EN			BIT(30)
#define ST_CONF_TARGET_WORK_EN(n)		BIT(24 - (n))
#define ST_UNIT0_OP				0x04	/* b30 UPDATE, b29 VALID */
#define ST_UNIT0_UPDATE				BIT(30)
#define ST_UNIT0_VALID				BIT(29)
/* Comparator n: target value HI (bits [19:0]) and LO */
#define ST_TARGET_HI(n)				(0x1c + (n) * 8)
#define ST_TARGET_LO(n)				(0x20 + (n) * 8)
/* b31 unit_sel(0=UNIT0), b30 period_mode(0=oneshot) */
#define ST_TARGET_CONF(n)			(0x34 + (n) * 4)
#define ST_TARGET_UNIT_SEL			BIT(31)
#define ST_TARGET_PERIOD_MODE			BIT(30)
#define ST_UNIT0_VALUE_HI			0x40	/* bits [19:0] */
#define ST_UNIT0_VALUE_LO			0x44
#define ST_COMP_LOAD(n)				(0x50 + (n) * 4)	/* b0 load */
#define ST_INT_ENA				0x64	/* bit n: TARGETn */
#define ST_INT_CLR				0x6c	/* bit n: TARGETn */
#define ST_INT_TARGET(n)			BIT(n)

#define ST_NR_COMPARATORS			3
#define ST_HI_MASK				0xfffffU
#define ST_COUNTER_BITS				52

struct systimer_cpu {
	struct clock_event_device ce;
	unsigned int comp;
};

static struct {
	void __iomem		*base;
	u32			rate;
	int			virq;
} systimer;

static DEFINE_PER_CPU(struct systimer_cpu, systimer_cpu);

/*
 * ST_CONF, ST_INT_ENA and the UNIT0 snapshot latch are single registers shared
 * by all comparators, so every read-modify-write of them and every counter
 * snapshot has to be serialised across the CPUs driving the block.
 */
static DEFINE_RAW_SPINLOCK(systimer_lock);

static u64 notrace systimer_count(void)
{
	static u64 last;
	void __iomem *b = systimer.base;
	unsigned long flags;
	int guard = 100000;
	u32 hi, lo;
	u64 now;

	/* Snapshot is a shared latch (UNIT0_OP): serialise against both IRQs and
	 * the other CPUs.
	 */
	raw_spin_lock_irqsave(&systimer_lock, flags);
	writel(readl(b + ST_UNIT0_OP) | ST_UNIT0_UPDATE, b + ST_UNIT0_OP);
	while (!(readl(b + ST_UNIT0_OP) & ST_UNIT0_VALID) && --guard)
		cpu_relax();
	if (guard) {
		hi = readl(b + ST_UNIT0_VALUE_HI) & ST_HI_MASK;
		lo = readl(b + ST_UNIT0_VALUE_LO);
		now = ((u64)hi << 32) | lo;
		last = now;
	} else {
		/* The VALUE registers hold an older snapshot, or none. */
		now = last;
	}
	raw_spin_unlock_irqrestore(&systimer_lock, flags);

	/* After the unlock: printk timestamps come from here. */
	WARN_ONCE(!guard, "esp32s31-systimer: counter snapshot timed out\n");
	return now;
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

/* Arm comparator n to fire delta ticks from now. The caller must not hold
 * systimer_lock (systimer_count() takes it).
 */
static void systimer_arm(unsigned int n, unsigned long delta)
{
	void __iomem *b = systimer.base;
	u64 tgt = systimer_count() + delta;
	unsigned long flags;

	raw_spin_lock_irqsave(&systimer_lock, flags);
	writel(readl(b + ST_CONF) & ~ST_CONF_TARGET_WORK_EN(n), b + ST_CONF);
	writel((u32)(tgt >> 32) & ST_HI_MASK, b + ST_TARGET_HI(n));
	writel((u32)tgt, b + ST_TARGET_LO(n));
	writel(readl(b + ST_TARGET_CONF(n)) &
	       ~(ST_TARGET_UNIT_SEL | ST_TARGET_PERIOD_MODE),
	       b + ST_TARGET_CONF(n));
	writel(BIT(0), b + ST_COMP_LOAD(n));
	writel(readl(b + ST_CONF) | ST_CONF_TARGET_WORK_EN(n), b + ST_CONF);
	raw_spin_unlock_irqrestore(&systimer_lock, flags);
}

static void systimer_disarm(unsigned int n, bool disable)
{
	void __iomem *b = systimer.base;
	unsigned long flags;

	raw_spin_lock_irqsave(&systimer_lock, flags);
	writel(readl(b + ST_CONF) & ~ST_CONF_TARGET_WORK_EN(n), b + ST_CONF);
	if (disable) {
		writel(readl(b + ST_INT_ENA) & ~ST_INT_TARGET(n), b + ST_INT_ENA);
		writel(ST_INT_TARGET(n), b + ST_INT_CLR);
	}
	raw_spin_unlock_irqrestore(&systimer_lock, flags);
}

/* Unmask comparator n's interrupt in the shared ST_INT_ENA. */
static void systimer_int_enable(unsigned int n)
{
	unsigned long flags;

	raw_spin_lock_irqsave(&systimer_lock, flags);
	writel(readl(systimer.base + ST_INT_ENA) | ST_INT_TARGET(n),
	       systimer.base + ST_INT_ENA);
	raw_spin_unlock_irqrestore(&systimer_lock, flags);
}

static unsigned int systimer_comp(struct clock_event_device *ce)
{
	return container_of(ce, struct systimer_cpu, ce)->comp;
}

static int systimer_set_next_event(unsigned long delta,
				   struct clock_event_device *ce)
{
	systimer_arm(systimer_comp(ce), delta);
	return 0;
}

static int systimer_shutdown(struct clock_event_device *ce)
{
	systimer_disarm(systimer_comp(ce), true);
	return 0;
}

static int systimer_set_oneshot(struct clock_event_device *ce)
{
	systimer_int_enable(systimer_comp(ce));
	return 0;
}

/* dev_id is this CPU's systimer_cpu, from handle_percpu_devid_irq(). */
static irqreturn_t systimer_isr(int irq, void *dev_id)
{
	struct systimer_cpu *sc = dev_id;

	/* One-shot: disarm before clearing so the level IRQ can't re-assert. */
	systimer_disarm(sc->comp, false);
	writel(ST_INT_TARGET(sc->comp), systimer.base + ST_INT_CLR);
	sc->ce.event_handler(&sc->ce);
	return IRQ_HANDLED;
}

static void systimer_setup_clockevent(unsigned int cpu)
{
	struct systimer_cpu *sc = per_cpu_ptr(&systimer_cpu, cpu);

	sc->comp = cpu;

	systimer_disarm(cpu, true);

	sc->ce.name			= "esp32s31-systimer";
	sc->ce.features			= CLOCK_EVT_FEAT_ONESHOT;
	/*
	 * Above riscv_timer's max rating (450 with Sstc): the systimer clockevent
	 * MUST win, or the kernel falls back to the CLINT clockevent whose MTIME
	 * freezes in WFI (FORCE_ON is intentionally not set) -> idle hang.
	 */
	sc->ce.rating			= 500;
	sc->ce.set_next_event		= systimer_set_next_event;
	sc->ce.set_state_shutdown	= systimer_shutdown;
	sc->ce.set_state_oneshot	= systimer_set_oneshot;
	sc->ce.cpumask			= cpumask_of(cpu);
	sc->ce.irq			= systimer.virq;

	/*
	 * The snapshot and programming sequence in systimer_arm() take longer
	 * than min_delta, so a short deadline can already be behind the counter
	 * when the comparator is enabled. A comparator whose target is already
	 * behind the counter fires at once, so the event is late rather than
	 * lost.
	 */
	clockevents_config_and_register(&sc->ce, systimer.rate, 0x10, 0x7fffffff);
}

static int systimer_starting_cpu(unsigned int cpu)
{
	systimer_setup_clockevent(cpu);
	enable_percpu_irq(systimer.virq, irq_get_trigger_type(systimer.virq));
	return 0;
}

static int systimer_dying_cpu(unsigned int cpu)
{
	systimer_disarm(cpu, true);
	disable_percpu_irq(systimer.virq);
	return 0;
}

static int __init systimer_init(struct device_node *np)
{
	u32 rate = 16000000;
	int ret;

	/* Each CPU's clockevent is a comparator of its own. */
	if (num_possible_cpus() > ST_NR_COMPARATORS) {
		pr_err("esp32s31-systimer: %u CPUs but only %u comparators\n",
		       num_possible_cpus(), ST_NR_COMPARATORS);
		return -EINVAL;
	}

	systimer.base = of_iomap(np, 0);
	if (!systimer.base)
		return -ENXIO;

	of_property_read_u32(np, "clock-frequency", &rate);
	systimer.rate = rate;

	/* UNIT0 free-running (OpenSBI already clocked + un-stalled it). */
	writel(readl(systimer.base + ST_CONF) | ST_CONF_UNIT0_WORK_EN,
	       systimer.base + ST_CONF);

	/*
	 * The alarm is one per-core interrupt: the dts names the hart-0
	 * comparator's source on intmtx, which routes hart n's instance in
	 * hart n's own window onto one CLIC slot that irq-esp-clic
	 * programs in each hart's bank. Nothing here touches an
	 * interrupt-controller register.
	 */
	systimer.virq = irq_of_parse_and_map(np, 0);
	if (!systimer.virq || !irq_is_percpu_devid(systimer.virq)) {
		pr_err("esp32s31-systimer: no per-core alarm interrupt\n");
		ret = -EINVAL;
		goto err_dispose;
	}

	ret = request_percpu_irq(systimer.virq, systimer_isr,
				 "esp32s31-systimer", &systimer_cpu);
	if (ret) {
		pr_err("esp32s31-systimer: percpu irq %d failed (%d)\n",
		       systimer.virq, ret);
		goto err_dispose;
	}

	/* Runs systimer_starting_cpu() for the boot CPU right here. */
	ret = cpuhp_setup_state(CPUHP_AP_ESP_SYSTIMER_STARTING,
				"clockevents/esp32s31-systimer:starting",
				systimer_starting_cpu, systimer_dying_cpu);
	if (ret < 0) {
		pr_err("esp32s31-systimer: cpuhp state failed (%d)\n", ret);
		goto err_free_irq;
	}

	/* Last: sched_clock cannot be unregistered, so nothing may unmap after it. */
	ret = clocksource_register_hz(&systimer_cs, rate);
	if (ret)
		return ret;
	sched_clock_register(systimer_sched_read, ST_COUNTER_BITS, rate);

	pr_info("esp32s31-systimer: %u Hz clocksource+clockevent, irq %d\n",
		rate, systimer.virq);
	return 0;

err_free_irq:
	free_percpu_irq(systimer.virq, &systimer_cpu);
err_dispose:
	irq_dispose_mapping(systimer.virq);
	iounmap(systimer.base);
	return ret;
}

TIMER_OF_DECLARE(esp32s31_systimer, "esp,esp32s31-systimer", systimer_init);
