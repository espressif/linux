// SPDX-License-Identifier: GPL-2.0-only
/*
 * WS2812 LED using the ESP32-S31 RMT TX controller.
 *
 * Author: Glauber Franco <gnfranco@gmail.com>
 */
#include <linux/led-class-multicolor.h>
#include <linux/mod_devicetable.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/soc/espressif/rmt.h>
#include <dt-bindings/leds/common.h>

#define WS2812_TICK_HZ 10000000

struct s31_rgb {
	struct esp_rmt_channel *channel;
	struct led_classdev_mc mc;
	struct mc_subled colors[3];
	struct mutex lock;
};

static int s31_send(struct s31_rgb *rgb, u32 grb)
{
	u32 symbols[25];
	int i;

	for (i = 0; i < 24; i++) {
		bool one = grb & BIT(23 - i);
		u32 hi = one ? 8 : 4, lo = one ? 5 : 9;

		symbols[i] = hi | BIT(15) | (lo << 16);
	}
	/* 100 us low latch. Controller appends EOF. */
	symbols[24] = 500 | (500 << 16);
	return esp_rmt_tx(rgb->channel, WS2812_TICK_HZ, symbols,
			  ARRAY_SIZE(symbols));
}

static int s31_set(struct led_classdev *cdev, enum led_brightness brightness)
{
	struct s31_rgb *rgb = container_of(lcdev_to_mccdev(cdev), struct s31_rgb, mc);
	u32 grb;
	int ret;

	mutex_lock(&rgb->lock);
	led_mc_calc_color_components(&rgb->mc, brightness);
	grb = (rgb->colors[1].brightness << 16) |
	      (rgb->colors[0].brightness << 8) | rgb->colors[2].brightness;
	ret = s31_send(rgb, grb);
	mutex_unlock(&rgb->lock);
	return ret;
}

static int s31_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct s31_rgb *rgb;
	u32 channel;
	int ret;

	ret = of_property_read_u32(dev->of_node, "reg", &channel);
	if (ret)
		return ret;
	rgb = devm_kzalloc(dev, sizeof(*rgb), GFP_KERNEL);
	if (!rgb)
		return -ENOMEM;
	mutex_init(&rgb->lock);
	rgb->channel = devm_esp_rmt_tx_get(dev, channel);
	if (IS_ERR(rgb->channel))
		return dev_err_probe(dev, PTR_ERR(rgb->channel), "RMT channel\n");
	ret = s31_send(rgb, 0);
	if (ret)
		return dev_err_probe(dev, ret, "initial RMT transfer failed\n");
	rgb->colors[0].color_index = LED_COLOR_ID_RED;
	rgb->colors[1].color_index = LED_COLOR_ID_GREEN;
	rgb->colors[2].color_index = LED_COLOR_ID_BLUE;
	rgb->colors[0].intensity = rgb->colors[1].intensity = rgb->colors[2].intensity = 255;
	rgb->mc.num_colors = 3;
	rgb->mc.subled_info = rgb->colors;
	if (channel == 0) {
		rgb->mc.led_cdev.name = "s31:rgb:status";
	} else {
		rgb->mc.led_cdev.name = devm_kasprintf(dev, GFP_KERNEL,
						     "s31:rgb:status-%u", channel);
		if (!rgb->mc.led_cdev.name)
			return -ENOMEM;
	}
	rgb->mc.led_cdev.max_brightness = 255;
	rgb->mc.led_cdev.brightness_set_blocking = s31_set;
	ret = devm_led_classdev_multicolor_register(dev, &rgb->mc);
	if (!ret)
		dev_info(dev, "WS2812 RMT TX%u ready (IRQ completion)\n", channel);
	return ret;
}

static const struct of_device_id s31_of_match[] = {
	{ .compatible = "esp,esp32s31-rmt-ws2812" }, { }
};
MODULE_DEVICE_TABLE(of, s31_of_match);
static struct platform_driver s31_driver = {
	.probe = s31_probe,
	.driver = { .name = "esp32s31-rmt-ws2812", .of_match_table = s31_of_match },
};
module_platform_driver(s31_driver);
MODULE_AUTHOR("Glauber Franco <gnfranco@gmail.com>");
MODULE_DESCRIPTION("WS2812 LED using the ESP32-S31 RMT TX controller");
MODULE_LICENSE("GPL");
