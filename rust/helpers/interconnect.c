// SPDX-License-Identifier: GPL-2.0

#include <linux/interconnect.h>

struct icc_path *rust_helper_devm_of_icc_get(struct device *dev,
					      const char *name)
{
	return devm_of_icc_get(dev, name);
}

int rust_helper_icc_set_bw(struct icc_path *path, u32 avg_bw, u32 peak_bw)
{
	return icc_set_bw(path, avg_bw, peak_bw);
}
