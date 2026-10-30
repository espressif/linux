// SPDX-License-Identifier: GPL-2.0-or-later

#include <kunit/test.h>
#include <linux/bits.h>
#include <linux/errno.h>

#include "esp32_uart.h"

/* Hardware divider fields: ESP32 is 20 bits, ESP32-S3 and later are 12 bits. */
static const u32 esp32_clkdiv_mask = GENMASK(19, 0);
static const u32 esp32s3_clkdiv_mask = GENMASK(11, 0);

static void esp32_uart_115200_from_40m(struct kunit *test)
{
	struct esp32_uart_baud baud = { };
	int ret;

	ret = esp32_uart_calc_baud(40000000, 115200, esp32s3_clkdiv_mask,
				   true, &baud);
	KUNIT_EXPECT_EQ(test, 0, ret);
	KUNIT_EXPECT_EQ(test, 347, baud.div);
	KUNIT_EXPECT_EQ(test, 3, baud.frag);
	KUNIT_EXPECT_EQ(test, 0, baud.sclk_div);
}

static void esp32_uart_exact_115200(struct kunit *test)
{
	struct esp32_uart_baud baud = { };
	int ret;

	ret = esp32_uart_calc_baud(1843200, 115200, esp32_clkdiv_mask,
				   false, &baud);
	KUNIT_EXPECT_EQ(test, 0, ret);
	KUNIT_EXPECT_EQ(test, 16, baud.div);
	KUNIT_EXPECT_EQ(test, 0, baud.frag);
	KUNIT_EXPECT_EQ(test, 0, baud.sclk_div);
}

static void esp32_uart_fractional_48m(struct kunit *test)
{
	struct esp32_uart_baud baud = { };
	int ret;

	ret = esp32_uart_calc_baud(48000000, 115200, esp32s3_clkdiv_mask,
				   true, &baud);
	KUNIT_EXPECT_EQ(test, 0, ret);
	KUNIT_EXPECT_EQ(test, 416, baud.div);
	KUNIT_EXPECT_EQ(test, 10, baud.frag);
	KUNIT_EXPECT_EQ(test, 0, baud.sclk_div);
}

static void esp32_uart_source_divisor_product_exceeds_u32(struct kunit *test)
{
	struct esp32_uart_baud baud = { };
	/* First baud whose product with the 12-bit divider exceeds U32_MAX. */
	const u32 baud_rate = 1048833;

	KUNIT_EXPECT_EQ(test, 0,
			esp32_uart_calc_baud(80000000, baud_rate,
					     esp32s3_clkdiv_mask, true, &baud));
	KUNIT_EXPECT_EQ(test, 76, baud.div);
	KUNIT_EXPECT_EQ(test, 4, baud.frag);
	KUNIT_EXPECT_EQ(test, 0, baud.sclk_div);
}

static void esp32_uart_no_clkconf_matches_s3_when_div_fits(struct kunit *test)
{
	struct esp32_uart_baud with_clk = { };
	struct esp32_uart_baud without_clk = { };

	KUNIT_EXPECT_EQ(test, 0,
			esp32_uart_calc_baud(40000000, 115200, esp32s3_clkdiv_mask,
					     true, &with_clk));
	KUNIT_EXPECT_EQ(test, 0,
			esp32_uart_calc_baud(40000000, 115200, esp32s3_clkdiv_mask,
					     false, &without_clk));
	KUNIT_EXPECT_EQ(test, with_clk.div, without_clk.div);
	KUNIT_EXPECT_EQ(test, with_clk.frag, without_clk.frag);
	KUNIT_EXPECT_EQ(test, 0, without_clk.sclk_div);
}

static void esp32_uart_min_divider(struct kunit *test)
{
	struct esp32_uart_baud baud = { };

	KUNIT_EXPECT_EQ(test, 0,
			esp32_uart_calc_baud(40000000, 40000000, esp32s3_clkdiv_mask,
					     false, &baud));
	KUNIT_EXPECT_EQ(test, 1, baud.div);
	KUNIT_EXPECT_EQ(test, 0, baud.frag);
}

static void esp32_uart_max_divider_without_prescaler(struct kunit *test)
{
	struct esp32_uart_baud baud = { };
	const u32 baud_rate = 9600;
	const u32 uartclk = esp32s3_clkdiv_mask * baud_rate;

	KUNIT_EXPECT_EQ(test, 0,
			esp32_uart_calc_baud(uartclk, baud_rate, esp32s3_clkdiv_mask,
					     true, &baud));
	KUNIT_EXPECT_EQ(test, esp32s3_clkdiv_mask, baud.div);
	KUNIT_EXPECT_EQ(test, 0, baud.frag);
	KUNIT_EXPECT_EQ(test, 0, baud.sclk_div);
}

static void esp32_uart_divider_above_mask_engages_prescaler(struct kunit *test)
{
	struct esp32_uart_baud baud = { };
	const u32 baud_rate = 9600;
	const u32 uartclk = (esp32s3_clkdiv_mask + 1) * baud_rate;

	KUNIT_EXPECT_EQ(test, 0,
			esp32_uart_calc_baud(uartclk, baud_rate, esp32s3_clkdiv_mask,
					     true, &baud));
	KUNIT_EXPECT_EQ(test, 1, baud.sclk_div);
	KUNIT_EXPECT_EQ(test, 2048, baud.div);
	KUNIT_EXPECT_EQ(test, 0, baud.frag);
}

static void esp32_uart_max_source_divider(struct kunit *test)
{
	struct esp32_uart_baud baud = { };
	const u32 source_div = 256;
	const u32 baud_rate = 100;
	const u32 uartclk = source_div * esp32s3_clkdiv_mask * baud_rate;

	KUNIT_EXPECT_EQ(test, 0,
			esp32_uart_calc_baud(uartclk, baud_rate, esp32s3_clkdiv_mask,
					     true, &baud));
	KUNIT_EXPECT_EQ(test, source_div - 1, baud.sclk_div);
	KUNIT_EXPECT_EQ(test, esp32s3_clkdiv_mask, baud.div);
	KUNIT_EXPECT_EQ(test, 0, baud.frag);
}

static void esp32_uart_source_divider_too_wide(struct kunit *test)
{
	const u32 baud_rate = 100;
	const u32 uartclk = 257 * esp32s3_clkdiv_mask * baud_rate;
	struct esp32_uart_baud baud = { };

	KUNIT_EXPECT_EQ(test, -ERANGE,
			esp32_uart_calc_baud(uartclk, baud_rate, esp32s3_clkdiv_mask,
					     true, &baud));
}

static void esp32_uart_unrepresentable_without_clkconf(struct kunit *test)
{
	const u32 baud_rate = 9600;
	const u32 uartclk = (esp32s3_clkdiv_mask + 1) * baud_rate;
	struct esp32_uart_baud baud = { };

	KUNIT_EXPECT_EQ(test, -ERANGE,
			esp32_uart_calc_baud(uartclk, baud_rate, esp32s3_clkdiv_mask,
					     false, &baud));
}

static void esp32_uart_zero_baud(struct kunit *test)
{
	struct esp32_uart_baud baud = { };

	KUNIT_EXPECT_EQ(test, -EINVAL,
			esp32_uart_calc_baud(40000000, 0, esp32s3_clkdiv_mask,
					     true, &baud));
}

static void esp32_uart_baud_above_clock(struct kunit *test)
{
	struct esp32_uart_baud baud = { };

	KUNIT_EXPECT_EQ(test, -ERANGE,
			esp32_uart_calc_baud(40000000, 40000001, esp32s3_clkdiv_mask,
					     true, &baud));
}

static struct kunit_case esp32_uart_cases[] = {
	KUNIT_CASE(esp32_uart_115200_from_40m),
	KUNIT_CASE(esp32_uart_exact_115200),
	KUNIT_CASE(esp32_uart_fractional_48m),
	KUNIT_CASE(esp32_uart_source_divisor_product_exceeds_u32),
	KUNIT_CASE(esp32_uart_no_clkconf_matches_s3_when_div_fits),
	KUNIT_CASE(esp32_uart_min_divider),
	KUNIT_CASE(esp32_uart_max_divider_without_prescaler),
	KUNIT_CASE(esp32_uart_divider_above_mask_engages_prescaler),
	KUNIT_CASE(esp32_uart_max_source_divider),
	KUNIT_CASE(esp32_uart_source_divider_too_wide),
	KUNIT_CASE(esp32_uart_unrepresentable_without_clkconf),
	KUNIT_CASE(esp32_uart_zero_baud),
	KUNIT_CASE(esp32_uart_baud_above_clock),
	{}
};

static struct kunit_suite esp32_uart_suite = {
	.name = "esp32_uart_baud",
	.test_cases = esp32_uart_cases,
};

kunit_test_suite(esp32_uart_suite);

MODULE_DESCRIPTION("KUnit tests for the ESP32 UART baud divider");
MODULE_LICENSE("GPL");
