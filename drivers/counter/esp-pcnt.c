// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Espressif pulse counter (Generic Counter)
 *
 * Copyright (C) 2026 Espressif Systems (Shanghai) Co. Ltd.
 *
 * One DT node is one instance, four 16-bit units. H_LIM/L_LIM stay at
 * full range; the hardware clears a unit to zero when it reaches either
 * limit, and the ISR adds the corresponding limit to the software
 * accumulator to extend the count to 64 bits. A missed IRQ loses those
 * wraps. The count is exposed as u64, so negative totals use the usual
 * two's complement representation; count_write accepts the same encoding.
 *
 * Watch events are not supported. The hardware threshold comparators see
 * only the 16-bit raw counter, while the exported count is extended to
 * 64 bits in software. A software-domain threshold would have to be
 * reprogrammed after every wrap and count write and still could not cover
 * arbitrary 64-bit values; exposing raw hardware thresholds would not
 * match the exported count.
 */

#include <linux/bitfield.h>
#include <linux/bitops.h>
#include <linux/bits.h>
#include <linux/build_bug.h>
#include <linux/clk.h>
#include <linux/counter.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/limits.h>
#include <linux/math.h>
#include <linux/math64.h>
#include <linux/mod_devicetable.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/reset.h>
#include <linux/spinlock.h>
#include <linux/time64.h>

#define ESP_PCNT_UNITS			4
#define ESP_PCNT_SIGS_PER_UNIT		2
#define ESP_PCNT_H_LIM			S16_MAX
#define ESP_PCNT_L_LIM			S16_MIN

#define ESP_PCNT_SIG_A			0
#define ESP_PCNT_SIG_B			1

struct esp_pcnt_unit_conf_regs {
	u32 conf0;
	u32 conf1;
	u32 conf2;
	u32 conf3;
};

struct esp_pcnt_regs {
	struct esp_pcnt_unit_conf_regs conf[ESP_PCNT_UNITS];
	u32 cnt[ESP_PCNT_UNITS];
	u32 int_raw;
	u32 int_st;
	u32 int_ena;
	u32 int_clr;
	u32 status[ESP_PCNT_UNITS];
	u32 ctrl;
};

#define ESP_PCNT_FILTER_THRES		GENMASK(9, 0)
#define ESP_PCNT_FILTER_MAX		FIELD_MAX(ESP_PCNT_FILTER_THRES)
#define ESP_PCNT_FILTER_EN		BIT(10)
#define ESP_PCNT_THR_ZERO_EN		BIT(11)
#define ESP_PCNT_THR_H_LIM_EN		BIT(12)
#define ESP_PCNT_THR_L_LIM_EN		BIT(13)
#define ESP_PCNT_THR_THRES0_EN		BIT(14)
#define ESP_PCNT_THR_THRES1_EN		BIT(15)
#define ESP_PCNT_CH0_NEG		GENMASK(17, 16)
#define ESP_PCNT_CH0_POS		GENMASK(19, 18)
#define ESP_PCNT_CH0_HCTRL		GENMASK(21, 20)
#define ESP_PCNT_CH0_LCTRL		GENMASK(23, 22)
#define ESP_PCNT_CH1_NEG		GENMASK(25, 24)
#define ESP_PCNT_CH1_POS		GENMASK(27, 26)
#define ESP_PCNT_CH1_HCTRL		GENMASK(29, 28)
#define ESP_PCNT_CH1_LCTRL		GENMASK(31, 30)
#define ESP_PCNT_THR_EN			GENMASK(15, 11)
#define ESP_PCNT_CHAN_MODE		GENMASK(31, 16)
#define ESP_PCNT_CONF2_H_LIM		GENMASK(15, 0)
#define ESP_PCNT_CONF2_L_LIM		GENMASK(31, 16)
#define ESP_PCNT_PULSE_CNT		GENMASK(15, 0)

#define ESP_PCNT_ST_L_LIM		BIT(4)
#define ESP_PCNT_ST_H_LIM		BIT(5)

#define ESP_PCNT_CTRL_RST(n)		BIT(2 * (n))
#define ESP_PCNT_CTRL_PAUSE(n)		BIT(2 * (n) + 1)
#define ESP_PCNT_CTRL_CLK_EN		BIT(16)
/* Even bits are per-unit reset, odd bits are per-unit pause. */
#define ESP_PCNT_CTRL_RST_MASK		(GENMASK(ESP_PCNT_UNITS * 2 - 1, 0) & 0x55555555)
#define ESP_PCNT_CTRL_PAUSE_MASK	(GENMASK(ESP_PCNT_UNITS * 2 - 1, 0) & 0xaaaaaaaa)
#define ESP_PCNT_INT_MASK		GENMASK(ESP_PCNT_UNITS - 1, 0)

enum esp_pcnt_edge_action {
	ESP_PCNT_EDGE_HOLD = 0,
	ESP_PCNT_EDGE_INC,
	ESP_PCNT_EDGE_DEC,
};

enum esp_pcnt_level_action {
	ESP_PCNT_LEVEL_KEEP = 0,
	ESP_PCNT_LEVEL_INV,
	ESP_PCNT_LEVEL_HOLD,
};

struct esp_pcnt_unit {
	s64 accum;
	enum counter_function function;
	bool enabled;
	u16 filter_thres;
};

struct esp_pcnt {
	struct esp_pcnt_regs __iomem *regs;
	struct clk *clk;
	struct reset_control *rst;
	spinlock_t lock; /* accum and MMIO */
	struct esp_pcnt_unit units[ESP_PCNT_UNITS];
	struct counter_signal signals[ESP_PCNT_UNITS * ESP_PCNT_SIGS_PER_UNIT];
	struct counter_synapse synapses[ESP_PCNT_UNITS * ESP_PCNT_SIGS_PER_UNIT];
	struct counter_count counts[ESP_PCNT_UNITS];
};

static void esp_pcnt_ctrl_update(struct esp_pcnt *pcnt, u32 mask, u32 val)
{
	u32 ctrl = readl(&pcnt->regs->ctrl);

	ctrl = (ctrl & ~mask) | (val & mask);
	writel(ctrl, &pcnt->regs->ctrl);
}

static s16 esp_pcnt_read_cnt(struct esp_pcnt *pcnt, unsigned int unit)
{
	return sign_extend32(readl(&pcnt->regs->cnt[unit]) & ESP_PCNT_PULSE_CNT,
			     15);
}

static void esp_pcnt_clear_cnt(struct esp_pcnt *pcnt, unsigned int unit)
{
	esp_pcnt_ctrl_update(pcnt, ESP_PCNT_CTRL_RST(unit),
			     ESP_PCNT_CTRL_RST(unit));
	esp_pcnt_ctrl_update(pcnt, ESP_PCNT_CTRL_RST(unit), 0);
}

static void esp_pcnt_apply_filter(struct esp_pcnt *pcnt, unsigned int unit)
{
	struct esp_pcnt_unit *u = &pcnt->units[unit];
	u32 conf0 = readl(&pcnt->regs->conf[unit].conf0);

	conf0 &= ~(ESP_PCNT_FILTER_THRES | ESP_PCNT_FILTER_EN);
	if (u->filter_thres) {
		conf0 |= FIELD_PREP(ESP_PCNT_FILTER_THRES, u->filter_thres);
		conf0 |= ESP_PCNT_FILTER_EN;
	}
	writel(conf0, &pcnt->regs->conf[unit].conf0);
}

static void esp_pcnt_apply_function(struct esp_pcnt *pcnt, unsigned int unit)
{
	u32 conf0 = readl(&pcnt->regs->conf[unit].conf0);
	u32 modes;

	conf0 &= ~ESP_PCNT_CHAN_MODE;

	if (pcnt->units[unit].function == COUNTER_FUNCTION_QUADRATURE_X4) {
		/* A = CH0 signal / CH1 control; B is the reverse. Invert on low. */
		modes = FIELD_PREP(ESP_PCNT_CH0_POS, ESP_PCNT_EDGE_DEC) |
			FIELD_PREP(ESP_PCNT_CH0_NEG, ESP_PCNT_EDGE_INC) |
			FIELD_PREP(ESP_PCNT_CH0_HCTRL, ESP_PCNT_LEVEL_KEEP) |
			FIELD_PREP(ESP_PCNT_CH0_LCTRL, ESP_PCNT_LEVEL_INV) |
			FIELD_PREP(ESP_PCNT_CH1_POS, ESP_PCNT_EDGE_INC) |
			FIELD_PREP(ESP_PCNT_CH1_NEG, ESP_PCNT_EDGE_DEC) |
			FIELD_PREP(ESP_PCNT_CH1_HCTRL, ESP_PCNT_LEVEL_KEEP) |
			FIELD_PREP(ESP_PCNT_CH1_LCTRL, ESP_PCNT_LEVEL_INV);
	} else {
		modes = FIELD_PREP(ESP_PCNT_CH0_POS, ESP_PCNT_EDGE_INC) |
			FIELD_PREP(ESP_PCNT_CH0_NEG, ESP_PCNT_EDGE_HOLD) |
			FIELD_PREP(ESP_PCNT_CH0_HCTRL, ESP_PCNT_LEVEL_KEEP) |
			FIELD_PREP(ESP_PCNT_CH0_LCTRL, ESP_PCNT_LEVEL_KEEP);
	}

	writel(conf0 | modes, &pcnt->regs->conf[unit].conf0);
}

static void esp_pcnt_init_unit(struct esp_pcnt *pcnt, unsigned int unit)
{
	struct esp_pcnt_unit *u = &pcnt->units[unit];
	u32 conf0;

	u->function = COUNTER_FUNCTION_INCREASE;
	u->filter_thres = 0;
	u->enabled = false;
	u->accum = 0;

	writel(FIELD_PREP_CONST(ESP_PCNT_CONF2_H_LIM, ESP_PCNT_H_LIM) |
	       FIELD_PREP_CONST(ESP_PCNT_CONF2_L_LIM, (u16)ESP_PCNT_L_LIM),
	       &pcnt->regs->conf[unit].conf2);

	conf0 = readl(&pcnt->regs->conf[unit].conf0);
	conf0 &= ~ESP_PCNT_THR_EN;
	conf0 |= ESP_PCNT_THR_H_LIM_EN | ESP_PCNT_THR_L_LIM_EN;
	writel(conf0, &pcnt->regs->conf[unit].conf0);

	esp_pcnt_apply_filter(pcnt, unit);
	esp_pcnt_apply_function(pcnt, unit);
	esp_pcnt_clear_cnt(pcnt, unit);
}

static s64 esp_pcnt_pending_delta(struct esp_pcnt *pcnt, unsigned int unit,
				  s16 hw)
{
	u32 st;

	if (!(readl(&pcnt->regs->int_st) & BIT(unit)))
		return 0;

	st = readl(&pcnt->regs->status[unit]);
	/*
	 * The limit event may have been latched after the cnt snapshot but
	 * before this status read. If cnt is still on the pre-reset side of
	 * the midpoint, the snapshot does not include the reset; leave it
	 * for the ISR.
	 */
	if ((st & ESP_PCNT_ST_L_LIM) && hw >= ESP_PCNT_L_LIM / 2)
		return ESP_PCNT_L_LIM;
	if ((st & ESP_PCNT_ST_H_LIM) && hw <= ESP_PCNT_H_LIM / 2)
		return ESP_PCNT_H_LIM;
	return 0;
}

static irqreturn_t esp_pcnt_isr(int irq, void *dev_id)
{
	struct counter_device *counter = dev_id;
	struct esp_pcnt *pcnt = counter_priv(counter);
	u32 pending;
	u32 st[ESP_PCNT_UNITS];
	unsigned int unit;

	scoped_guard(spinlock, &pcnt->lock) {
		pending = readl(&pcnt->regs->int_st);
		if (!pending)
			return IRQ_NONE;

		for (unit = 0; unit < ESP_PCNT_UNITS; unit++) {
			if (pending & BIT(unit))
				st[unit] = readl(&pcnt->regs->status[unit]);
		}

		writel(pending, &pcnt->regs->int_clr);

		for (unit = 0; unit < ESP_PCNT_UNITS; unit++) {
			if (!(pending & BIT(unit)))
				continue;
			if (st[unit] & ESP_PCNT_ST_L_LIM)
				pcnt->units[unit].accum += ESP_PCNT_L_LIM;
			else if (st[unit] & ESP_PCNT_ST_H_LIM)
				pcnt->units[unit].accum += ESP_PCNT_H_LIM;
		}
	}

	return IRQ_HANDLED;
}

static int esp_pcnt_count_read(struct counter_device *counter,
			       struct counter_count *count, u64 *val)
{
	struct esp_pcnt *pcnt = counter_priv(counter);
	unsigned int unit = count->id;
	s16 hw;
	s64 total;

	scoped_guard(spinlock_irqsave, &pcnt->lock) {
		hw = esp_pcnt_read_cnt(pcnt, unit);
		total = pcnt->units[unit].accum + hw +
			esp_pcnt_pending_delta(pcnt, unit, hw);
	}
	*val = (u64)total;

	return 0;
}

static int esp_pcnt_count_write(struct counter_device *counter,
				struct counter_count *count, u64 val)
{
	struct esp_pcnt *pcnt = counter_priv(counter);
	unsigned int unit = count->id;

	scoped_guard(spinlock_irqsave, &pcnt->lock) {
		if (pcnt->units[unit].enabled)
			return -EBUSY;
		/*
		 * Pulses between the accum store and the HW clear are dropped.
		 * Clear a latched limit so the next read/ISR does not add
		 * ±H/L_LIM on top of the new accum.
		 */
		pcnt->units[unit].accum = (s64)val;
		esp_pcnt_clear_cnt(pcnt, unit);
		writel(BIT(unit), &pcnt->regs->int_clr);
	}

	return 0;
}

static int esp_pcnt_function_read(struct counter_device *counter,
				  struct counter_count *count,
				  enum counter_function *function)
{
	struct esp_pcnt *pcnt = counter_priv(counter);

	scoped_guard(spinlock_irqsave, &pcnt->lock) {
		*function = pcnt->units[count->id].function;
	}

	return 0;
}

static int esp_pcnt_function_write(struct counter_device *counter,
				   struct counter_count *count,
				   enum counter_function function)
{
	struct esp_pcnt *pcnt = counter_priv(counter);
	unsigned int unit = count->id;

	if (function != COUNTER_FUNCTION_INCREASE &&
	    function != COUNTER_FUNCTION_QUADRATURE_X4)
		return -EINVAL;

	scoped_guard(spinlock_irqsave, &pcnt->lock) {
		if (pcnt->units[unit].enabled)
			return -EBUSY;
		pcnt->units[unit].function = function;
		esp_pcnt_apply_function(pcnt, unit);
	}

	return 0;
}

static int esp_pcnt_action_read(struct counter_device *counter,
				struct counter_count *count,
				struct counter_synapse *synapse,
				enum counter_synapse_action *action)
{
	struct esp_pcnt *pcnt = counter_priv(counter);
	unsigned int unit = count->id;
	unsigned int which = synapse->signal->id - unit * ESP_PCNT_SIGS_PER_UNIT;

	scoped_guard(spinlock_irqsave, &pcnt->lock) {
		if (pcnt->units[unit].function == COUNTER_FUNCTION_QUADRATURE_X4)
			*action = COUNTER_SYNAPSE_ACTION_BOTH_EDGES;
		else
			*action = (which == ESP_PCNT_SIG_A) ?
				COUNTER_SYNAPSE_ACTION_RISING_EDGE :
				COUNTER_SYNAPSE_ACTION_NONE;
	}
	return 0;
}

static int esp_pcnt_enable_read(struct counter_device *counter,
				struct counter_count *count, u8 *enable)
{
	struct esp_pcnt *pcnt = counter_priv(counter);

	scoped_guard(spinlock_irqsave, &pcnt->lock) {
		*enable = pcnt->units[count->id].enabled;
	}

	return 0;
}

static int esp_pcnt_enable_write(struct counter_device *counter,
				 struct counter_count *count, u8 enable)
{
	struct esp_pcnt *pcnt = counter_priv(counter);
	unsigned int unit = count->id;

	scoped_guard(spinlock_irqsave, &pcnt->lock) {
		pcnt->units[unit].enabled = enable;
		esp_pcnt_ctrl_update(pcnt, ESP_PCNT_CTRL_PAUSE(unit),
				     enable ? 0 : ESP_PCNT_CTRL_PAUSE(unit));
	}

	return 0;
}

static int esp_pcnt_apb_hz(struct esp_pcnt *pcnt, unsigned long *rate)
{
	*rate = clk_get_rate(pcnt->clk);
	return *rate ? 0 : -EOPNOTSUPP;
}

static int esp_pcnt_ns_to_cycles(struct esp_pcnt *pcnt, u64 ns, u16 *cycles)
{
	unsigned long rate;
	int ret;

	if (!ns) {
		*cycles = 0;
		return 0;
	}

	ret = esp_pcnt_apb_hz(pcnt, &rate);
	if (ret)
		return ret;
	/* Cap first so DIV_ROUND_UP_ULL cannot wrap and program 0. */
	if (ns > div_u64((u64)ESP_PCNT_FILTER_MAX * NSEC_PER_SEC, rate))
		return -ERANGE;

	/* ABI: pulses shorter or equal to the requested ns are ignored. */
	*cycles = DIV_ROUND_UP_ULL(ns * rate, NSEC_PER_SEC);
	return 0;
}

static int esp_pcnt_cycles_to_ns(struct esp_pcnt *pcnt, u16 cycles, u64 *ns)
{
	unsigned long rate;
	int ret;

	if (!cycles) {
		*ns = 0;
		return 0;
	}

	ret = esp_pcnt_apb_hz(pcnt, &rate);
	if (ret)
		return ret;

	/*
	 * Report the floor: write rounds up, so reporting a larger value would
	 * claim a longer filter window than is actually programmed. At APB rates
	 * this also keeps a read/write round-trip on the same cycle count.
	 */
	*ns = div_u64((u64)cycles * NSEC_PER_SEC, rate);
	return 0;
}

static int esp_pcnt_spike_filter_read(struct counter_device *counter,
				      struct counter_count *count, u64 *val)
{
	struct esp_pcnt *pcnt = counter_priv(counter);
	u16 cycles;

	scoped_guard(spinlock_irqsave, &pcnt->lock) {
		cycles = pcnt->units[count->id].filter_thres;
	}

	return esp_pcnt_cycles_to_ns(pcnt, cycles, val);
}

static int esp_pcnt_spike_filter_write(struct counter_device *counter,
				       struct counter_count *count, u64 val)
{
	struct esp_pcnt *pcnt = counter_priv(counter);
	unsigned int unit = count->id;
	u16 cycles;
	int ret;

	ret = esp_pcnt_ns_to_cycles(pcnt, val, &cycles);
	if (ret)
		return ret;

	scoped_guard(spinlock_irqsave, &pcnt->lock) {
		if (pcnt->units[unit].enabled)
			return -EBUSY;
		pcnt->units[unit].filter_thres = cycles;
		esp_pcnt_apply_filter(pcnt, unit);
	}

	return 0;
}

static const char *const esp_pcnt_unit_names[] = {
	"Unit 0", "Unit 1", "Unit 2", "Unit 3",
};

static_assert(ARRAY_SIZE(esp_pcnt_unit_names) == ESP_PCNT_UNITS);

static const char *const esp_pcnt_signal_names[] = {
	"Unit 0 A", "Unit 0 B", "Unit 1 A", "Unit 1 B",
	"Unit 2 A", "Unit 2 B", "Unit 3 A", "Unit 3 B",
};

static_assert(ARRAY_SIZE(esp_pcnt_signal_names) ==
	      ESP_PCNT_UNITS * ESP_PCNT_SIGS_PER_UNIT);

static const enum counter_function esp_pcnt_functions[] = {
	COUNTER_FUNCTION_INCREASE,
	COUNTER_FUNCTION_QUADRATURE_X4,
};

static const enum counter_synapse_action esp_pcnt_actions[] = {
	COUNTER_SYNAPSE_ACTION_NONE,
	COUNTER_SYNAPSE_ACTION_RISING_EDGE,
	COUNTER_SYNAPSE_ACTION_BOTH_EDGES,
};

static struct counter_comp esp_pcnt_count_ext[] = {
	COUNTER_COMP_ENABLE(esp_pcnt_enable_read, esp_pcnt_enable_write),
	COUNTER_COMP_COUNT_U64("spike_filter_ns", esp_pcnt_spike_filter_read,
			       esp_pcnt_spike_filter_write),
};

static int esp_pcnt_watch_validate(struct counter_device *counter,
				   const struct counter_watch *watch)
{
	/* Watch events are not supported; see the file header. */
	return -EOPNOTSUPP;
}

static const struct counter_ops esp_pcnt_ops = {
	.count_read	= esp_pcnt_count_read,
	.count_write	= esp_pcnt_count_write,
	.function_read	= esp_pcnt_function_read,
	.function_write	= esp_pcnt_function_write,
	.action_read	= esp_pcnt_action_read,
	.watch_validate	= esp_pcnt_watch_validate,
};

static void esp_pcnt_assert_reset(void *data)
{
	reset_control_assert(data);
}

static void esp_pcnt_quiesce(void *data)
{
	struct esp_pcnt *pcnt = data;

	/* Stop the interrupt source before devres frees the IRQ. */
	scoped_guard(spinlock_irqsave, &pcnt->lock) {
		writel(0, &pcnt->regs->int_ena);
		writel(ESP_PCNT_INT_MASK, &pcnt->regs->int_clr);
	}
}

static int esp_pcnt_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct counter_device *counter;
	struct esp_pcnt *pcnt;
	unsigned int unit;
	int irq, ret;

	counter = devm_counter_alloc(dev, sizeof(*pcnt));
	if (!counter)
		return -ENOMEM;

	pcnt = counter_priv(counter);
	spin_lock_init(&pcnt->lock);

	pcnt->regs = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(pcnt->regs))
		return PTR_ERR(pcnt->regs);

	pcnt->clk = devm_clk_get_enabled(dev, "apb");
	if (IS_ERR(pcnt->clk))
		return dev_err_probe(dev, PTR_ERR(pcnt->clk),
				     "failed to get apb clock\n");
	if (!clk_get_rate(pcnt->clk))
		return dev_err_probe(dev, -EOPNOTSUPP,
				     "apb clock has no rate\n");

	pcnt->rst = devm_reset_control_get_exclusive(dev, NULL);
	if (IS_ERR(pcnt->rst))
		return dev_err_probe(dev, PTR_ERR(pcnt->rst),
				     "failed to get reset\n");

	/* This reset controller has no .reset callback. */
	ret = reset_control_assert(pcnt->rst);
	if (ret)
		return dev_err_probe(dev, ret, "failed to assert PCNT reset\n");
	udelay(1);
	ret = reset_control_deassert(pcnt->rst);
	if (ret)
		return dev_err_probe(dev, ret, "failed to deassert PCNT reset\n");

	ret = devm_add_action_or_reset(dev, esp_pcnt_assert_reset, pcnt->rst);
	if (ret)
		return ret;

	/* CTRL.clk_en must be set before any other access. */
	writel(ESP_PCNT_CTRL_CLK_EN | ESP_PCNT_CTRL_RST_MASK |
	       ESP_PCNT_CTRL_PAUSE_MASK, &pcnt->regs->ctrl);
	writel(ESP_PCNT_CTRL_CLK_EN | ESP_PCNT_CTRL_PAUSE_MASK,
	       &pcnt->regs->ctrl);

	for (unit = 0; unit < ESP_PCNT_UNITS; unit++)
		esp_pcnt_init_unit(pcnt, unit);

	for (unit = 0; unit < ESP_PCNT_UNITS; unit++) {
		unsigned int a = unit * ESP_PCNT_SIGS_PER_UNIT;
		unsigned int b = a + 1;

		pcnt->signals[a].id = a;
		pcnt->signals[a].name = esp_pcnt_signal_names[a];
		pcnt->signals[b].id = b;
		pcnt->signals[b].name = esp_pcnt_signal_names[b];

		pcnt->synapses[a].actions_list = esp_pcnt_actions;
		pcnt->synapses[a].num_actions = ARRAY_SIZE(esp_pcnt_actions);
		pcnt->synapses[a].signal = &pcnt->signals[a];
		pcnt->synapses[b].actions_list = esp_pcnt_actions;
		pcnt->synapses[b].num_actions = ARRAY_SIZE(esp_pcnt_actions);
		pcnt->synapses[b].signal = &pcnt->signals[b];

		pcnt->counts[unit].id = unit;
		pcnt->counts[unit].name = esp_pcnt_unit_names[unit];
		pcnt->counts[unit].functions_list = esp_pcnt_functions;
		pcnt->counts[unit].num_functions = ARRAY_SIZE(esp_pcnt_functions);
		pcnt->counts[unit].synapses = &pcnt->synapses[a];
		pcnt->counts[unit].num_synapses = ESP_PCNT_SIGS_PER_UNIT;
		pcnt->counts[unit].ext = esp_pcnt_count_ext;
		pcnt->counts[unit].num_ext = ARRAY_SIZE(esp_pcnt_count_ext);
	}

	counter->name = dev_name(dev);
	counter->parent = dev;
	counter->ops = &esp_pcnt_ops;
	counter->signals = pcnt->signals;
	counter->num_signals = ESP_PCNT_UNITS * ESP_PCNT_SIGS_PER_UNIT;
	counter->counts = pcnt->counts;
	counter->num_counts = ESP_PCNT_UNITS;

	irq = platform_get_irq(pdev, 0);
	if (irq < 0)
		return irq;

	ret = devm_request_irq(dev, irq, esp_pcnt_isr, 0, dev_name(dev),
			       counter);
	if (ret)
		return dev_err_probe(dev, ret, "failed to request IRQ %d\n",
				     irq);

	writel(ESP_PCNT_INT_MASK, &pcnt->regs->int_clr);
	writel(ESP_PCNT_INT_MASK, &pcnt->regs->int_ena);

	ret = devm_counter_add(dev, counter);
	if (ret)
		return dev_err_probe(dev, ret, "failed to add counter\n");

	ret = devm_add_action_or_reset(dev, esp_pcnt_quiesce, pcnt);
	if (ret)
		return ret;

	return 0;
}

static const struct of_device_id esp_pcnt_of_match[] = {
	{ .compatible = "esp,esp32s31-pcnt" },
	{ }
};
MODULE_DEVICE_TABLE(of, esp_pcnt_of_match);

static struct platform_driver esp_pcnt_driver = {
	.probe = esp_pcnt_probe,
	.driver = {
		.name = "esp-pcnt",
		.of_match_table = esp_pcnt_of_match,
	},
};
module_platform_driver(esp_pcnt_driver);

MODULE_AUTHOR("Mao Shengrong <maoshengrong@espressif.com>");
MODULE_DESCRIPTION("Espressif pulse counter");
MODULE_LICENSE("GPL");
MODULE_IMPORT_NS("COUNTER");
