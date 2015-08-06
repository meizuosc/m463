ifeq ($(CONFIG_MTK_FPGA),y)
obj-y	:= mt_gpio_fpga.o
else
obj-y	:= mt_gpio_base.o 
endif
obj-y	+= mt_gpio_ext.o mt_gpio_affix.o mt_gpio_debug.o

# 2014-9-29 by Yin ShunQing, remove mt6306 i2c driver
ifneq ($(MEIZU_M71),yes)
obj-y	+= dcl_sim_gpio.o
endif
