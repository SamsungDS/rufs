// SPDX-License-Identifier: GPL-2.0

#include <linux/phy/phy.h>

struct phy *rust_helper_devm_phy_get(struct device *dev, const char *name)
{
	return devm_phy_get(dev, name);
}

int rust_helper_phy_init(struct phy *phy)
{
	return phy_init(phy);
}

int rust_helper_phy_exit(struct phy *phy)
{
	return phy_exit(phy);
}

int rust_helper_phy_set_ufs_mode(struct phy *phy, bool rate_b, int gear)
{
	enum phy_mode mode = rate_b ? PHY_MODE_UFS_HS_B : PHY_MODE_UFS_HS_A;

	return phy_set_mode_ext(phy, mode, gear);
}

int rust_helper_phy_power_on(struct phy *phy)
{
	return phy_power_on(phy);
}

int rust_helper_phy_power_off(struct phy *phy)
{
	return phy_power_off(phy);
}

int rust_helper_phy_calibrate(struct phy *phy)
{
	return phy_calibrate(phy);
}
