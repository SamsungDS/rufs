// SPDX-License-Identifier: GPL-2.0

#include <linux/gpio/consumer.h>

struct gpio_desc *
rust_helper_devm_gpiod_get_optional_output(struct device *dev,
					    const char *name, bool value)
{
	return devm_gpiod_get_optional(dev, name,
				       value ? GPIOD_OUT_HIGH : GPIOD_OUT_LOW);
}

int rust_helper_gpiod_set_value_cansleep(struct gpio_desc *desc, int value)
{
	return gpiod_set_value_cansleep(desc, value);
}
