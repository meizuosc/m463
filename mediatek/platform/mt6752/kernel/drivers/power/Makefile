# 2014-9-29, by Yin ShunQing, remove tps6128x i2c driver
ifeq ($(MEIZU_M71),yes)
obj-$(CONFIG_MTK_PMIC)          += pmic.o upmu_common.o pmic_auxadc.o pmic_chr_type_det.o pmic_otg_type_det.o mt6311.o
else
obj-$(CONFIG_MTK_PMIC)          += pmic.o upmu_common.o pmic_auxadc.o pmic_chr_type_det.o pmic_otg_type_det.o tps6128x.o mt6311.o
endif


ifeq ($(MTK_PMIC_DVT_SUPPORT),yes)
     obj-$(CONFIG_MTK_PMIC)     += pmic_dvt.o
endif

obj-$(CONFIG_MTK_SMART_BATTERY) += battery_meter_hal.o

ifeq ($(MTK_BQ24158_SUPPORT),yes)
     obj-$(CONFIG_MTK_SMART_BATTERY) += bq24158.o charging_hw_bq24158.o
else
	ifeq ($(MTK_BQ24296_SUPPORT),yes)
		 obj-$(CONFIG_MTK_SMART_BATTERY) += bq24296.o charging_hw_bq24296.o
	else

		ifeq ($(MTK_BQ24261_SUPPORT),yes)
			obj-$(CONFIG_MTK_SMART_BATTERY) += bq24261.o charging_hw_bq24261.o
		else
			ifeq ($(MTK_SWCHR_SUPPORT),yes)
				obj-$(CONFIG_MTK_SMART_BATTERY) += charging_hw_mt_sw_charger.o
			else
				ifeq ($(MTK_NCP1854_SUPPORT),yes)
		           obj-$(CONFIG_MTK_SMART_BATTERY) += ncp1854.o charging_hw_ncp1854.o
			   else
		           obj-$(CONFIG_MTK_SMART_BATTERY) += charging_hw_pmic.o
			   endif
			endif
		endif
	endif
endif
