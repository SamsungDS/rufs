# SPDX-License-Identifier: GPL-2.0

CONTEXT_ANALYSIS := y

obj-$(CONFIG_SCSI_UFSHCD)		+= ufshcd-core.o
ufshcd-core-y				+= ufs-sysfs.o ufs-mcq.o ufs-txeq.o
ifeq ($(CONFIG_SCSI_UFSHCD_STANDALONE_ANALYSIS),y)
ufshcd-core-y				+= ufshcd-standalone.o
else
ufshcd-core-y				+= ufshcd.o
endif
ufshcd-core-$(CONFIG_RPMB)		+= ufs-rpmb.o
ufshcd-core-$(CONFIG_DEBUG_FS)		+= ufs-debugfs.o
ufshcd-core-$(CONFIG_SCSI_UFS_BSG)	+= ufs_bsg.o
ufshcd-core-$(CONFIG_SCSI_UFS_CRYPTO)	+= ufshcd-crypto.o
ufshcd-core-$(CONFIG_SCSI_UFS_FAULT_INJECTION) += ufs-fault-injection.o
ufshcd-core-$(CONFIG_SCSI_UFS_HWMON)	+= ufs-hwmon.o
