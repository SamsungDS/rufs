// SPDX-License-Identifier: GPL-2.0

#include <linux/gpio/consumer.h>

struct gpio_desc *
rust_helper_devm_gpiod_get_optional_output_low(struct device *dev,
						const char *name)
{
	return devm_gpiod_get_optional(dev, name, GPIOD_OUT_LOW);
}

int rust_helper_gpiod_set_value_cansleep(struct gpio_desc *desc, int value)
{
	return gpiod_set_value_cansleep(desc, value);
}
