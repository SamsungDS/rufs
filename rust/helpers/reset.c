// SPDX-License-Identifier: GPL-2.0

#include <linux/reset.h>

struct reset_control *
rust_helper_devm_reset_control_get_optional_exclusive(struct device *dev,
						       const char *id)
{
	return devm_reset_control_get_optional_exclusive(dev, id);
}
