// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Espressif ESP32-S31 I2C master controller driver
 *
 * Copyright (C) 2026 Espressif Systems (Shanghai) Co. Ltd.
 *
 */

#include <linux/bitfield.h>
#include <linux/bitops.h>
#include <linux/clk.h>
#include <linux/completion.h>
#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/reset.h>
#include <linux/string.h>

/* ========== Register offsets ========== */
#define ESP_I2C_SCL_LOW_PERIOD		0x00
#define ESP_I2C_CTR			0x04
#define ESP_I2C_SR			0x08
#define ESP_I2C_TO			0x0C
#define ESP_I2C_FIFO_ST			0x14
#define ESP_I2C_FIFO_CONF		0x18
#define ESP_I2C_DATA			0x1C
#define ESP_I2C_INT_RAW			0x20
#define ESP_I2C_INT_CLR			0x24
#define ESP_I2C_INT_ENA			0x28
#define ESP_I2C_INT_STATUS		0x2C
#define ESP_I2C_SDA_HOLD		0x30
#define ESP_I2C_SDA_SAMPLE		0x34
#define ESP_I2C_SCL_HIGH_PERIOD		0x38
#define ESP_I2C_SCL_START_HOLD		0x40
#define ESP_I2C_SCL_RSTART_SETUP	0x44
#define ESP_I2C_SCL_STOP_HOLD		0x48
#define ESP_I2C_SCL_STOP_SETUP		0x4C
#define ESP_I2C_FILTER_CFG		0x50
#define ESP_I2C_COMD(n)			(0x58 + (n) * 4)
#define ESP_I2C_SCL_ST_TO		0x78
#define ESP_I2C_SCL_MAIN_ST_TO		0x7C
#define ESP_I2C_DATE			0xFC

/* ========== CTR register bits ========== */
#define ESP_I2C_CTR_SDA_FORCE_OUT	BIT(0)
#define ESP_I2C_CTR_SCL_FORCE_OUT	BIT(1)
#define ESP_I2C_CTR_SAMPLE_SCL_LVL	BIT(2)
#define ESP_I2C_CTR_RX_FULL_ACK_LVL	BIT(3)
#define ESP_I2C_CTR_MS_MODE		BIT(4)
#define ESP_I2C_CTR_TRANS_START		BIT(5)
#define ESP_I2C_CTR_CLK_EN		BIT(8)
#define ESP_I2C_CTR_ARBITRATION_EN	BIT(9)
#define ESP_I2C_CTR_FSM_RST		BIT(10)
#define ESP_I2C_CTR_CONF_UPDATE		BIT(11)

/* ========== SR register bits ========== */
#define ESP_I2C_SR_BUS_BUSY		BIT(4)

/* ========== FIFO_CONF register bits ========== */
#define ESP_I2C_FIFO_CONF_RX_RST	BIT(12)
#define ESP_I2C_FIFO_CONF_TX_RST	BIT(13)
#define ESP_I2C_FIFO_CONF_FIFO_PRT_EN	BIT(14)

/* ========== TO register bits ========== */
#define ESP_I2C_TO_VAL_MASK		GENMASK(4, 0)
#define ESP_I2C_TO_EN			BIT(5)

/* ========== Filter config bits ========== */
#define ESP_I2C_FILTER_SCL_THRES	GENMASK(3, 0)
#define ESP_I2C_FILTER_SDA_THRES	GENMASK(7, 4)
#define ESP_I2C_FILTER_SCL_EN		BIT(8)
#define ESP_I2C_FILTER_SDA_EN		BIT(9)
#define ESP_I2C_FILTER_THRES_DEFAULT_TICK	7

/* ========== Interrupt bits ========== */
#define ESP_I2C_INT_END_DETECT		BIT(3)
#define ESP_I2C_INT_ARB_LOST		BIT(5)
#define ESP_I2C_INT_TRANS_COMPLETE	BIT(7)
#define ESP_I2C_INT_TIME_OUT		BIT(8)
#define ESP_I2C_INT_NACK		BIT(10)
#define ESP_I2C_INT_SCL_ST_TO		BIT(13)
#define ESP_I2C_INT_SCL_MAIN_ST_TO	BIT(14)
#define ESP_I2C_INT_ALL			0x7FFFF

/* Master error interrupts */
#define ESP_I2C_INT_ERR_MASK		(ESP_I2C_INT_NACK |		\
					 ESP_I2C_INT_TIME_OUT |		\
					 ESP_I2C_INT_ARB_LOST |		\
					 ESP_I2C_INT_SCL_ST_TO |	\
					 ESP_I2C_INT_SCL_MAIN_ST_TO)

/* ========== Command register ========== */
#define ESP_I2C_CMD_DONE		BIT(31)
#define ESP_I2C_CMD_OP_S		11
#define ESP_I2C_CMD_ACK_EN		BIT(8)
#define ESP_I2C_CMD_ACK_EXP		BIT(9)
#define ESP_I2C_CMD_ACK_VAL		BIT(10)

/* Hardware command opcodes */
#define ESP_I2C_OP_WRITE		1
#define ESP_I2C_OP_STOP			2
#define ESP_I2C_OP_READ			3
#define ESP_I2C_OP_END			4
#define ESP_I2C_OP_RSTART		6

/* ========== Constants ========== */
#define ESP_I2C_FIFO_LEN		32
#define ESP_I2C_CMD_MAX			8
#define ESP_I2C_TIMEOUT_MS		1000
/*
 * SCL_ST_TO and SCL_MAIN_ST_TO: an FSM state held for 2^(val - 1) module clock
 * cycles raises the matching interrupt. 23 is the largest value the hardware
 * accepts, about 105 ms at 40 MHz.
 */
#define ESP_I2C_SCL_ST_TO_MAX		23

#define ESP_I2C_CTR_BASE		(ESP_I2C_CTR_MS_MODE |		\
					 ESP_I2C_CTR_CLK_EN |		\
					 ESP_I2C_CTR_ARBITRATION_EN)

/* ========== Driver private data ========== */
struct esp_i2c {
	struct i2c_adapter	adap;
	struct device		*dev;
	void __iomem		*base;
	int			irq;
	u32			bus_freq;
	u32			src_clk_freq;
	struct completion	xfer_done;
	u32			irq_status;
	u32			addr_cmds;	/* commands writing an address */
	bool			addr_nack;	/* last NACK was on an address */
};

/* ========== Register access helpers ========== */
static inline u32 esp_i2c_read(struct esp_i2c *i2c, u32 reg)
{
	return readl(i2c->base + reg);
}

static inline void esp_i2c_write(struct esp_i2c *i2c, u32 reg, u32 val)
{
	writel(val, i2c->base + reg);
}

/* ========== FIFO operations ========== */
static void esp_i2c_reset_fifo(struct esp_i2c *i2c)
{
	u32 val = esp_i2c_read(i2c, ESP_I2C_FIFO_CONF);

	val |= ESP_I2C_FIFO_CONF_TX_RST | ESP_I2C_FIFO_CONF_RX_RST;
	esp_i2c_write(i2c, ESP_I2C_FIFO_CONF, val);

	val &= ~(ESP_I2C_FIFO_CONF_TX_RST | ESP_I2C_FIFO_CONF_RX_RST);
	esp_i2c_write(i2c, ESP_I2C_FIFO_CONF, val);
}

static void esp_i2c_write_txfifo(struct esp_i2c *i2c, const u8 *buf, int len)
{
	int i;

	for (i = 0; i < len; i++)
		esp_i2c_write(i2c, ESP_I2C_DATA, buf[i]);
}

static void esp_i2c_read_rxfifo(struct esp_i2c *i2c, u8 *buf, int len)
{
	int i;

	for (i = 0; i < len; i++)
		buf[i] = esp_i2c_read(i2c, ESP_I2C_DATA) & 0xFF;
}

/* ========== Command register programming ========== */
static void esp_i2c_set_cmd(struct esp_i2c *i2c, int idx, u32 opcode,
			     u8 byte_num, bool ack_en, bool ack_nack)
{
	u32 cmd;

	cmd = (opcode << ESP_I2C_CMD_OP_S) | (byte_num & 0xFF);
	if (ack_en)
		cmd |= ESP_I2C_CMD_ACK_EN;
	if (ack_nack)
		cmd |= ESP_I2C_CMD_ACK_VAL;

	esp_i2c_write(i2c, ESP_I2C_COMD(idx), cmd);
}

/*
 * An address byte always gets a WRITE of its own, recorded so a NACK on it can
 * be told from a NACK on data. Every batch that writes an address goes through
 * here; esp_i2c_start() clears the record once the batch has run.
 */
static void esp_i2c_set_addr_cmd(struct esp_i2c *i2c, int idx, u8 addr_byte)
{
	esp_i2c_write_txfifo(i2c, &addr_byte, 1);
	esp_i2c_set_cmd(i2c, idx, ESP_I2C_OP_WRITE, 1, true, false);
	i2c->addr_cmds |= BIT(idx);
}

/* ========== Hardware control ========== */
static void esp_i2c_conf_update(struct esp_i2c *i2c)
{
	esp_i2c_write(i2c, ESP_I2C_CTR,
		      ESP_I2C_CTR_BASE | ESP_I2C_CTR_CONF_UPDATE);
}

static void esp_i2c_start_hw(struct esp_i2c *i2c)
{
	esp_i2c_write(i2c, ESP_I2C_CTR,
		      ESP_I2C_CTR_BASE | ESP_I2C_CTR_TRANS_START);
}

static void esp_i2c_hw_reset(struct esp_i2c *i2c)
{
	esp_i2c_write(i2c, ESP_I2C_CTR,
		      ESP_I2C_CTR_BASE | ESP_I2C_CTR_FSM_RST);

	/* Reset FIFOs */
	esp_i2c_reset_fifo(i2c);

	/* Clear all interrupts */
	esp_i2c_write(i2c, ESP_I2C_INT_CLR, ESP_I2C_INT_ALL);
	esp_i2c_write(i2c, ESP_I2C_INT_ENA, 0);
}

/* ========== Bus timing ========== */

/*
 * SCL_HIGH_PERIOD register layout:
 *   [8:0]  scl_high_period
 *   [15:9] scl_wait_high_period
 *
 * The actual SCL high time = scl_high_period + scl_wait_high_period.
 * scl_wait_high is a window for SCL to be pulled high by external
 * pull-ups before the controller starts counting scl_high_period.
 */
#define ESP_I2C_SCL_HIGH_MASK		0x1FF
#define ESP_I2C_SCL_WAIT_HIGH_S		9
#define ESP_I2C_SCL_WAIT_HIGH_MASK	0x7F

/*
 * Calculate and set bus timing
 * i2c_ll_master_cal_bus_clk() / i2c_ll_master_set_bus_timing().
 * Key points:
 *   half_cycle = sclk_freq / bus_freq / 2
 *
 *   scl_low       = half_cycle - 1
 *   scl_wait_high = (bus_freq >= 80kHz) ? half_cycle/2 - 2 : half_cycle/4
 *   scl_high      = half_cycle - scl_wait_high
 *   sda_hold      = half_cycle / 4 - 1
 *   sda_sample    = half_cycle / 2 - 1
 *   setup         = half_cycle - 1
 *   hold          = half_cycle - 1
 *   tout          = ceil(log2(5 * half_cycle)) + 2
 */
static int esp_i2c_set_bus_timing(struct esp_i2c *i2c)
{
	u32 sclk_freq = i2c->src_clk_freq;
	u32 bus_freq = i2c->bus_freq;
	u32 half_cycle;
	u32 scl_low, scl_high, scl_wait_high;
	u32 sda_hold, sda_sample;
	u32 setup, hold;
	u32 tout;
	u32 scl_high_reg;

	half_cycle = sclk_freq / bus_freq / 2;

	/* SCL timing */
	scl_low = half_cycle - 1;

	/*
	 * scl_wait_high: time window for SCL to rise before counting.
	 * For >= 80kHz: use half_cycle/2 - 2 (faster rise expected).
	 * For < 80kHz:  use half_cycle/4 (slower, more conservative).
	 */
	if (bus_freq >= 80000)
		scl_wait_high = half_cycle / 2 - 2;
	else
		scl_wait_high = half_cycle / 4;

	scl_high = half_cycle - scl_wait_high;

	/*
	 * The module clock's divider saturates, so a bus frequency low enough
	 * to outrun it leaves the periods too wide for their register fields.
	 * Masking them would silently clock the bus at some other rate.
	 */
	if (scl_low > ESP_I2C_SCL_HIGH_MASK ||
	    scl_high > ESP_I2C_SCL_HIGH_MASK ||
	    scl_wait_high > ESP_I2C_SCL_WAIT_HIGH_MASK) {
		dev_err(i2c->dev,
			"bus frequency %u too low for a %u Hz module clock\n",
			bus_freq, sclk_freq);
		return -EINVAL;
	}

	/* SDA timing */
	sda_hold = half_cycle / 4 - 1;
	sda_sample = half_cycle / 2 - 1;

	/* START/STOP setup and hold */
	setup = half_cycle - 1;
	hold = half_cycle - 1;

	/* Timeout: ~10 bus cycles → log2(5 * half_cycle) + 2 */
	tout = fls(5 * half_cycle) + 2;

	/* SCL_LOW_PERIOD register */
	esp_i2c_write(i2c, ESP_I2C_SCL_LOW_PERIOD, scl_low);

	/*
	 * SCL_HIGH_PERIOD register: pack scl_high and scl_wait_high
	 * into a single 32-bit write.
	 */
	scl_high_reg = (scl_high & ESP_I2C_SCL_HIGH_MASK) |
		       ((scl_wait_high & ESP_I2C_SCL_WAIT_HIGH_MASK)
			<< ESP_I2C_SCL_WAIT_HIGH_S);
	esp_i2c_write(i2c, ESP_I2C_SCL_HIGH_PERIOD, scl_high_reg);

	/* SDA hold / sample */
	esp_i2c_write(i2c, ESP_I2C_SDA_HOLD, sda_hold);
	esp_i2c_write(i2c, ESP_I2C_SDA_SAMPLE, sda_sample);

	/* START condition: rstart_setup, start_hold */
	esp_i2c_write(i2c, ESP_I2C_SCL_RSTART_SETUP, setup);
	esp_i2c_write(i2c, ESP_I2C_SCL_START_HOLD, hold);

	/* STOP condition: stop_setup, stop_hold */
	esp_i2c_write(i2c, ESP_I2C_SCL_STOP_SETUP, setup);
	esp_i2c_write(i2c, ESP_I2C_SCL_STOP_HOLD, hold);

	/* Timeout: clamp to 5-bit field, actual period = 2^(tout) i2c_sclk cycles */
	tout = min_t(u32, tout, ESP_I2C_TO_VAL_MASK);
	esp_i2c_write(i2c, ESP_I2C_TO,
		      (tout & ESP_I2C_TO_VAL_MASK) | ESP_I2C_TO_EN);

	esp_i2c_conf_update(i2c);

	dev_dbg(i2c->dev,
		"timing: sclk=%u half=%u scl_low=%u scl_high=%u wait_high=%u sda_hold=%u sda_sample=%u tout=%u\n",
		sclk_freq, half_cycle, scl_low, scl_high, scl_wait_high,
		sda_hold, sda_sample, tout);

	return 0;
}

/* ========== Hardware initialization ========== */
static int esp_i2c_hw_init(struct esp_i2c *i2c)
{
	int ret;

	/* Reset FSM */
	esp_i2c_write(i2c, ESP_I2C_CTR, ESP_I2C_CTR_FSM_RST);

	/*
	 * Configure controller:
	 *  - Master mode
	 *    Controller actively drives both high and low.
	 *    No dependency on external pull-up for SCL rise time.
	 *  - Enable internal clock gating
	 *  - Enable arbitration detection
	 */
	esp_i2c_write(i2c, ESP_I2C_CTR, ESP_I2C_CTR_BASE);

	/* Enable FIFO mode with protection */
	esp_i2c_write(i2c, ESP_I2C_FIFO_CONF, ESP_I2C_FIFO_CONF_FIFO_PRT_EN);

	/* Enable SCL and SDA glitch filters; both share one register here. */
	esp_i2c_write(i2c, ESP_I2C_FILTER_CFG,
		      FIELD_PREP(ESP_I2C_FILTER_SCL_THRES,
				 ESP_I2C_FILTER_THRES_DEFAULT_TICK) |
		      FIELD_PREP(ESP_I2C_FILTER_SDA_THRES,
				 ESP_I2C_FILTER_THRES_DEFAULT_TICK) |
		      ESP_I2C_FILTER_SCL_EN | ESP_I2C_FILTER_SDA_EN);

	/* Set bus timing (includes timeout configuration) */
	ret = esp_i2c_set_bus_timing(i2c);
	if (ret)
		return ret;

	/* SCL state machine timeouts (for bus-stuck detection) */
	esp_i2c_write(i2c, ESP_I2C_SCL_ST_TO, ESP_I2C_SCL_ST_TO_MAX);
	esp_i2c_write(i2c, ESP_I2C_SCL_MAIN_ST_TO, ESP_I2C_SCL_ST_TO_MAX);

	/* Disable all interrupts and clear pending */
	esp_i2c_write(i2c, ESP_I2C_INT_ENA, 0);
	esp_i2c_write(i2c, ESP_I2C_INT_CLR, ESP_I2C_INT_ALL);

	/* Sync configuration to hardware */
	esp_i2c_conf_update(i2c);

	return 0;
}

/* ========== Interrupt handler ========== */

/*
 * On NACK, immediately start a STOP condition from ISR context. This overlaps
 * STOP execution with the scheduler wake-up latency, so by the time the thread
 * resumes, STOP is already done. Not on ARB_LOST: the bus belongs to the
 * master that won it.
 */
static inline void esp_i2c_isr_send_stop_cmd(struct esp_i2c *i2c)
{
	esp_i2c_set_cmd(i2c, 0, ESP_I2C_OP_STOP, 0, false, false);
	esp_i2c_conf_update(i2c);
	esp_i2c_start_hw(i2c);
}

/*
 * The controller marks the NACKed command done and runs nothing after it, so
 * the NACKed command is the one before the first command not done.
 */
static bool esp_i2c_nack_on_addr(struct esp_i2c *i2c)
{
	int next;

	for (next = 0; next < ESP_I2C_CMD_MAX; next++) {
		if (!(esp_i2c_read(i2c, ESP_I2C_COMD(next)) & ESP_I2C_CMD_DONE))
			break;
	}

	return next && (i2c->addr_cmds & BIT(next - 1));
}

static irqreturn_t esp_i2c_isr(int irq, void *dev_id)
{
	struct esp_i2c *i2c = dev_id;
	u32 status;

	status = esp_i2c_read(i2c, ESP_I2C_INT_STATUS);
	if (!status)
		return IRQ_NONE;

	esp_i2c_write(i2c, ESP_I2C_INT_CLR, status);
	esp_i2c_write(i2c, ESP_I2C_INT_ENA, 0);

	if ((status & ESP_I2C_INT_NACK) && !(status & ESP_I2C_INT_ARB_LOST)) {
		/*
		 * Classify first: the STOP below is written into command 0,
		 * which clears that command's done bit.
		 */
		i2c->addr_nack = esp_i2c_nack_on_addr(i2c);
		esp_i2c_isr_send_stop_cmd(i2c);
	}

	i2c->irq_status = status;
	complete(&i2c->xfer_done);

	return IRQ_HANDLED;
}

/* ========== Bus STOP and release ========== */

/*
 * Wait for a STOP that was already started (from ISR context) to complete.
 */
static int esp_i2c_wait_stop_done(struct esp_i2c *i2c)
{
	unsigned long timeout = jiffies + msecs_to_jiffies(10);

	while (esp_i2c_read(i2c, ESP_I2C_SR) & ESP_I2C_SR_BUS_BUSY) {
		if (time_after(jiffies, timeout)) {
			dev_err(i2c->dev, "wait for bus idle timed out\n");
			return -ETIMEDOUT;
		}
		usleep_range(100, 200);
	}

	return 0;
}

/*
 * Send a STOP condition on the I2C bus using polling.
 */
static int esp_i2c_send_stop(struct esp_i2c *i2c)
{
	esp_i2c_write(i2c, ESP_I2C_INT_ENA, 0);
	esp_i2c_write(i2c, ESP_I2C_INT_CLR, ESP_I2C_INT_ALL);

	esp_i2c_set_cmd(i2c, 0, ESP_I2C_OP_STOP, 0, false, false);
	esp_i2c_conf_update(i2c);
	esp_i2c_start_hw(i2c);

	return esp_i2c_wait_stop_done(i2c);
}

static int esp_i2c_start(struct esp_i2c *i2c)
{
	u32 int_mask;
	unsigned long time_left;

	int_mask = ESP_I2C_INT_END_DETECT | ESP_I2C_INT_TRANS_COMPLETE |
		  ESP_I2C_INT_ERR_MASK;

	reinit_completion(&i2c->xfer_done);
	i2c->irq_status = 0;
	i2c->addr_nack = false;

	esp_i2c_write(i2c, ESP_I2C_INT_CLR, ESP_I2C_INT_ALL);
	esp_i2c_write(i2c, ESP_I2C_INT_ENA, int_mask);
	esp_i2c_conf_update(i2c);
	esp_i2c_start_hw(i2c);

	time_left = wait_for_completion_timeout(&i2c->xfer_done,
						msecs_to_jiffies(ESP_I2C_TIMEOUT_MS));

	esp_i2c_write(i2c, ESP_I2C_INT_ENA, 0);
	esp_i2c_write(i2c, ESP_I2C_INT_CLR, ESP_I2C_INT_ALL);
	i2c->addr_cmds = 0;

	if (!time_left) {
		dev_err(i2c->dev, "transfer timeout\n");
		esp_i2c_hw_reset(i2c);
		return -ETIMEDOUT;
	}

	if (i2c->irq_status & ESP_I2C_INT_ARB_LOST) {
		dev_dbg(i2c->dev, "arbitration lost\n");
		esp_i2c_hw_reset(i2c);
		return -EAGAIN;
	}

	if (i2c->irq_status & ESP_I2C_INT_NACK) {
		dev_dbg(i2c->dev, "%s NACK\n",
			i2c->addr_nack ? "address" : "data");
		esp_i2c_wait_stop_done(i2c);
		/*
		 * Reset the FSM even once the bus is idle: left alone after a
		 * NACK that cut a WRITE short, it NACKs every later transfer
		 * without sending a byte.
		 */
		esp_i2c_hw_reset(i2c);
		return i2c->addr_nack ? -ENXIO : -EIO;
	}

	if (i2c->irq_status & (ESP_I2C_INT_TIME_OUT | ESP_I2C_INT_SCL_ST_TO |
			       ESP_I2C_INT_SCL_MAIN_ST_TO)) {
		dev_err(i2c->dev, "hardware timeout (status %#x)\n",
			i2c->irq_status);
		esp_i2c_hw_reset(i2c);
		return -ETIMEDOUT;
	}

	return 0;
}

/*
 * Execute an I2C write message.
 *
 * All command sequences end with END. After the last segment
 * of the last message, STOP is sent via polling in process context.
 *
 * For short writes (addr + data <= FIFO_LEN):
 *   CMD0: RSTART
 *   CMD1: WRITE(addr_byte, ack_en=1)
 *   CMD2: WRITE(data, ack_en=1)   [if len > 0]
 *   CMD3: END
 *
 * For long writes (segmented):
 *   First segment:
 *     CMD0: RSTART
 *     CMD1: WRITE(addr_byte, ack_en=1)
 *     CMD2: WRITE(data_chunk, ack_en=1)
 *     CMD3: END
 *   Subsequent segments:
 *     CMD0: WRITE(data_chunk, ack_en=1)
 *     CMD1: END
 */
static int esp_i2c_do_write(struct esp_i2c *i2c, struct i2c_msg *msg, bool last)
{
	u8 addr_byte = i2c_8bit_addr_from_msg(msg);
	int remaining = msg->len;
	int pos = 0;
	bool first = true;
	int ret;

	do {
		int cmd_idx = 0;
		int fifo_avail = ESP_I2C_FIFO_LEN;
		int chunk_len;

		esp_i2c_reset_fifo(i2c);

		if (first) {
			esp_i2c_set_cmd(i2c, cmd_idx++, ESP_I2C_OP_RSTART,
					0, false, false);
			esp_i2c_set_addr_cmd(i2c, cmd_idx++, addr_byte);
			fifo_avail--;
			first = false;
		}

		chunk_len = min(remaining, fifo_avail);

		if (chunk_len > 0) {
			esp_i2c_write_txfifo(i2c, msg->buf + pos, chunk_len);
			esp_i2c_set_cmd(i2c, cmd_idx++, ESP_I2C_OP_WRITE,
					chunk_len, true, false);
		}

		remaining -= chunk_len;

		esp_i2c_set_cmd(i2c, cmd_idx++, ESP_I2C_OP_END,
				0, false, false);

		ret = esp_i2c_start(i2c);
		if (ret)
			return ret;

		pos += chunk_len;
	} while (remaining > 0);

	if (last) {
		ret = esp_i2c_send_stop(i2c);
		if (ret)
			return ret;
	}

	return 0;
}

/*
 * Execute an I2C read message.
 *
 * All command sequences end with END. After the last segment
 * of the last message, STOP is sent via polling in process context
 * (after draining RX FIFO to preserve data).
 *
 * For short reads (data <= FIFO_LEN):
 *   CMD0: RSTART
 *   CMD1: WRITE(addr_byte, ack_en=1)
 *   CMD2: READ(len-1, ack_val=0)  [ACK, if len > 1]
 *   CMD3: READ(1, ack_val=1)      [NACK for last byte]
 *   CMD4: END
 *
 * For long reads (segmented):
 *   First segment:
 *     CMD0: RSTART
 *     CMD1: WRITE(addr_byte, ack_en=1)
 *     CMD2: READ(FIFO_LEN, ack_val=0)
 *     CMD3: END
 *   Subsequent segments:
 *     CMD0: READ(chunk, ack_val=0 or 1 for last byte)
 *     CMD1: END
 */
static int esp_i2c_do_read(struct esp_i2c *i2c, struct i2c_msg *msg, bool last)
{
	u8 addr_byte = i2c_8bit_addr_from_msg(msg);
	int remaining = msg->len;
	int pos = 0;
	bool first = true;
	int ret;

	do {
		int cmd_idx = 0;
		int chunk_len;
		bool is_last_chunk;

		esp_i2c_reset_fifo(i2c);

		if (first) {
			esp_i2c_set_cmd(i2c, cmd_idx++, ESP_I2C_OP_RSTART,
					0, false, false);
			esp_i2c_set_addr_cmd(i2c, cmd_idx++, addr_byte);
			first = false;
		}

		if (remaining == 0) {
			esp_i2c_set_cmd(i2c, cmd_idx++, ESP_I2C_OP_END,
					0, false, false);
			ret = esp_i2c_start(i2c);
			if (ret)
				return ret;
			if (last)
				return esp_i2c_send_stop(i2c);
			return 0;
		}

		chunk_len = min(remaining, ESP_I2C_FIFO_LEN);
		is_last_chunk = (chunk_len == remaining);

		if (is_last_chunk) {
			if (chunk_len > 1)
				esp_i2c_set_cmd(i2c, cmd_idx++,
						ESP_I2C_OP_READ,
						chunk_len - 1, false, false);
			esp_i2c_set_cmd(i2c, cmd_idx++, ESP_I2C_OP_READ,
					1, false, true);
		} else {
			esp_i2c_set_cmd(i2c, cmd_idx++, ESP_I2C_OP_READ,
					chunk_len, false, false);
		}

		esp_i2c_set_cmd(i2c, cmd_idx++, ESP_I2C_OP_END,
				0, false, false);

		ret = esp_i2c_start(i2c);
		if (ret)
			return ret;

		/* Drain RX FIFO before sending STOP */
		esp_i2c_read_rxfifo(i2c, msg->buf + pos, chunk_len);

		pos += chunk_len;
		remaining -= chunk_len;
	} while (remaining > 0);

	if (last) {
		ret = esp_i2c_send_stop(i2c);
		if (ret)
			return ret;
	}

	return 0;
}

static int esp_i2c_xfer_combined(struct esp_i2c *i2c,
				 struct i2c_msg *msgs, int num)
{
	int i, cmd_idx = 0, ret;
	int tx_len = 0, cmds_needed = 0;
	int read_idx = -1;
	int first_rx_chunk = 0;
	bool need_more_rx, full_read;

	/* --- Pre-flight: check if all messages fit in one batch --- */
	for (i = 0; i < num; i++) {
		bool is_read = (msgs[i].flags & I2C_M_RD);

		cmds_needed++;		/* RSTART */

		if (is_read) {
			if (read_idx >= 0)
				return 0;	/* >1 read, use fallback */
			read_idx = i;
			tx_len++;		/* address byte */
			cmds_needed++;		/* WRITE(addr) */

			/*
			 * The rest of a read longer than the FIFO is fetched
			 * after this batch, which is only in order when no
			 * message follows it.
			 */
			if (i != num - 1 && msgs[i].len > ESP_I2C_FIFO_LEN)
				return 0;

			first_rx_chunk = min_t(int, msgs[i].len,
					       ESP_I2C_FIFO_LEN);

			if (first_rx_chunk < msgs[i].len)
				cmds_needed++;
			else if (msgs[i].len)
				cmds_needed += (msgs[i].len > 1) ? 2 : 1;
		} else {
			tx_len += 1 + msgs[i].len;	/* addr + data */
			cmds_needed++;			/* WRITE(addr) */
			if (msgs[i].len)
				cmds_needed++;		/* WRITE(data) */
		}
	}
	cmds_needed++;	/* END */

	if (cmds_needed > ESP_I2C_CMD_MAX || tx_len > ESP_I2C_FIFO_LEN)
		return 0;	/* doesn't fit, use fallback */

	/* --- Program all messages into CMD registers --- */
	esp_i2c_reset_fifo(i2c);

	for (i = 0; i < num; i++) {
		struct i2c_msg *msg = &msgs[i];
		u8 addr_byte = i2c_8bit_addr_from_msg(msg);
		bool is_read = (msg->flags & I2C_M_RD);

		esp_i2c_set_cmd(i2c, cmd_idx++, ESP_I2C_OP_RSTART,
				0, false, false);

		esp_i2c_set_addr_cmd(i2c, cmd_idx++, addr_byte);

		if (is_read) {
			if (msg->len == 0)
				continue;

			full_read = (first_rx_chunk == msg->len);

			if (full_read) {
				if (first_rx_chunk > 1)
					esp_i2c_set_cmd(i2c, cmd_idx++,
							ESP_I2C_OP_READ,
							first_rx_chunk - 1,
							false, false);
				esp_i2c_set_cmd(i2c, cmd_idx++,
						ESP_I2C_OP_READ, 1,
						false, true);
			} else {
				esp_i2c_set_cmd(i2c, cmd_idx++,
						ESP_I2C_OP_READ,
						first_rx_chunk,
						false, false);
			}
		} else if (msg->len > 0) {
			esp_i2c_write_txfifo(i2c, msg->buf, msg->len);
			esp_i2c_set_cmd(i2c, cmd_idx++, ESP_I2C_OP_WRITE,
					msg->len, true, false);
		}
	}

	esp_i2c_set_cmd(i2c, cmd_idx++, ESP_I2C_OP_END, 0, false, false);

	/* --- Execute first batch --- */
	need_more_rx = (read_idx >= 0 &&
			first_rx_chunk < msgs[read_idx].len);

	ret = esp_i2c_start(i2c);
	if (ret)
		return ret;

	/* Drain RX FIFO for the first read chunk */
	if (read_idx >= 0 && first_rx_chunk > 0)
		esp_i2c_read_rxfifo(i2c, msgs[read_idx].buf, first_rx_chunk);

	if (!need_more_rx) {
		ret = esp_i2c_send_stop(i2c);
		if (ret)
			return ret;
		return num;
	}

	/* --- Continue reading remaining data in FIFO-sized segments --- */
	if (read_idx >= 0) {
		struct i2c_msg *rmsg = &msgs[read_idx];
		int remaining = rmsg->len - first_rx_chunk;
		int pos = first_rx_chunk;

		while (remaining > 0) {
			int chunk = min(remaining, ESP_I2C_FIFO_LEN);
			bool is_final = (chunk == remaining);

			cmd_idx = 0;
			esp_i2c_reset_fifo(i2c);

			if (is_final) {
				if (chunk > 1)
					esp_i2c_set_cmd(i2c, cmd_idx++,
							ESP_I2C_OP_READ,
							chunk - 1,
							false, false);
				esp_i2c_set_cmd(i2c, cmd_idx++,
						ESP_I2C_OP_READ, 1,
						false, true);
			} else {
				esp_i2c_set_cmd(i2c, cmd_idx++,
						ESP_I2C_OP_READ, chunk,
						false, false);
			}

			esp_i2c_set_cmd(i2c, cmd_idx++, ESP_I2C_OP_END,
					0, false, false);

			ret = esp_i2c_start(i2c);
			if (ret)
				return ret;

			esp_i2c_read_rxfifo(i2c, rmsg->buf + pos, chunk);

			if (is_final) {
				ret = esp_i2c_send_stop(i2c);
				if (ret)
					return ret;
			}

			pos += chunk;
			remaining -= chunk;
		}
	}

	return num;
}

/* ========== I2C adapter operations ========== */
static int esp_i2c_xfer(struct i2c_adapter *adap, struct i2c_msg *msgs,
			 int num)
{
	struct esp_i2c *i2c = i2c_get_adapdata(adap);
	int i, ret;

	for (i = 0; i < num; i++) {
		if (msgs[i].flags & I2C_M_TEN)
			return -EOPNOTSUPP;
	}

	/* Try combined transaction (zero gap between messages) */
	ret = esp_i2c_xfer_combined(i2c, msgs, num);
	if (ret != 0)
		return ret;

	/* Fallback: per-message processing (large writes, multi-read, etc.) */
	for (i = 0; i < num; i++) {
		bool is_last = (i == num - 1);

		if (msgs[i].flags & I2C_M_RD)
			ret = esp_i2c_do_read(i2c, &msgs[i], is_last);
		else
			ret = esp_i2c_do_write(i2c, &msgs[i], is_last);

		if (ret)
			return ret;
	}

	return num;
}

static u32 esp_i2c_func(struct i2c_adapter *adap)
{
	return I2C_FUNC_I2C | I2C_FUNC_SMBUS_EMUL;
}

static const struct i2c_algorithm esp_i2c_algo = {
	.master_xfer	= esp_i2c_xfer,
	.functionality	= esp_i2c_func,
};

/* ========== Platform driver ========== */
static void esp_i2c_assert_reset(void *data)
{
	struct reset_control *rst = data;

	reset_control_assert(rst);
}

static int esp_i2c_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	unsigned long parent_rate;
	struct reset_control *rst;
	struct clk *core_clk;
	struct esp_i2c *i2c;
	unsigned int clkm_div;
	struct clk *clk;
	int ret;

	i2c = devm_kzalloc(dev, sizeof(*i2c), GFP_KERNEL);
	if (!i2c)
		return -ENOMEM;

	i2c->dev = dev;
	platform_set_drvdata(pdev, i2c);

	/* Map registers */
	i2c->base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(i2c->base))
		return PTR_ERR(i2c->base);

	/*
	 * The APB gate has to be on before any register access, and both it and
	 * the module clock live in HP_SYS_CLKRST rather than in the window
	 * above. devm disables them again if probe fails.
	 */
	clk = devm_clk_get_enabled(dev, "apb");
	if (IS_ERR(clk))
		return dev_err_probe(dev, PTR_ERR(clk),
				     "failed to get apb clock\n");

	core_clk = devm_clk_get_enabled(dev, "core");
	if (IS_ERR(core_clk))
		return dev_err_probe(dev, PTR_ERR(core_clk),
				     "failed to get core clock\n");

	rst = devm_reset_control_get_exclusive(dev, NULL);
	if (IS_ERR(rst))
		return dev_err_probe(dev, PTR_ERR(rst), "failed to get reset\n");

	ret = reset_control_deassert(rst);
	if (ret)
		return dev_err_probe(dev, ret, "failed to deassert reset\n");

	ret = devm_add_action_or_reset(dev, esp_i2c_assert_reset, rst);
	if (ret)
		return ret;

	/* Get bus frequency from device tree (default: 100kHz) */
	ret = device_property_read_u32(dev, "clock-frequency", &i2c->bus_freq);
	if (ret)
		i2c->bus_freq = I2C_MAX_STANDARD_MODE_FREQ;

	if (!i2c->bus_freq || i2c->bus_freq > I2C_MAX_FAST_MODE_FREQ)
		return dev_err_probe(dev, -EINVAL,
				     "bus frequency %u out of range, fast mode is the maximum\n",
				     i2c->bus_freq);

	parent_rate = clk_get_rate(clk_get_parent(core_clk));
	if (!parent_rate)
		return dev_err_probe(dev, -EINVAL,
				     "core clock parent rate is zero\n");

	clkm_div = parent_rate / (i2c->bus_freq * 1024) + 1;

	ret = clk_set_rate(core_clk, parent_rate / clkm_div);
	if (ret)
		return dev_err_probe(dev, ret, "failed to set core clock rate\n");

	i2c->src_clk_freq = clk_get_rate(core_clk);
	if (!i2c->src_clk_freq)
		return dev_err_probe(dev, -EINVAL, "core clock rate is zero\n");

	/* Get interrupt */
	i2c->irq = platform_get_irq(pdev, 0);
	if (i2c->irq < 0)
		return i2c->irq;

	init_completion(&i2c->xfer_done);

	/* Initialize hardware */
	ret = esp_i2c_hw_init(i2c);
	if (ret)
		return ret;

	ret = devm_request_irq(dev, i2c->irq, esp_i2c_isr, 0, dev_name(dev), i2c);
	if (ret)
		return dev_err_probe(dev, ret, "failed to request IRQ %d\n",
				     i2c->irq);

	/* Setup I2C adapter */
	i2c->adap.owner = THIS_MODULE;
	i2c->adap.algo = &esp_i2c_algo;
	i2c->adap.dev.parent = dev;
	i2c->adap.dev.of_node = dev->of_node;
	i2c->adap.nr = -1;
	strscpy(i2c->adap.name, "Espressif ESP I2C adapter",
		sizeof(i2c->adap.name));
	i2c_set_adapdata(&i2c->adap, i2c);

	ret = i2c_add_numbered_adapter(&i2c->adap);
	if (ret)
		return dev_err_probe(dev, ret, "failed to add I2C adapter\n");

	dev_info(dev, "ESP I2C adapter registered at %p, %u Hz, IRQ %d\n",
		 i2c->base, i2c->bus_freq, i2c->irq);

	return 0;
}

static void esp_i2c_remove(struct platform_device *pdev)
{
	struct esp_i2c *i2c = platform_get_drvdata(pdev);

	i2c_del_adapter(&i2c->adap);

	/* Disable interrupts and reset hardware */
	esp_i2c_write(i2c, ESP_I2C_INT_ENA, 0);
	esp_i2c_write(i2c, ESP_I2C_INT_CLR, ESP_I2C_INT_ALL);
	synchronize_irq(i2c->irq);
}

static const struct of_device_id esp_i2c_of_match[] = {
	{ .compatible = "esp,esp32s31-i2c" },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, esp_i2c_of_match);

static struct platform_driver esp_i2c_driver = {
	.probe	= esp_i2c_probe,
	.remove	= esp_i2c_remove,
	.driver	= {
		.name		= "esp-i2c",
		.of_match_table	= esp_i2c_of_match,
	},
};
module_platform_driver(esp_i2c_driver);

MODULE_AUTHOR("Espressif Systems");
MODULE_DESCRIPTION("Espressif ESP32-S31 I2C master controller driver");
MODULE_LICENSE("GPL");
