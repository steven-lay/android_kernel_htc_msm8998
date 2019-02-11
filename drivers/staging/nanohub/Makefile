#
# Makefile for nanohub
#

obj-$(CONFIG_NANOHUB) += nanohub.o
nanohub-y := main.o comms.o
nanohub-$(CONFIG_NANOHUB_SPI) += spi.o
nanohub-$(CONFIG_NANOHUB_I2C) += i2c.o
ifdef CONFIG_NANOHUB_STM32L4
nanohub-y += bl_l4.o
else
nanohub-y += bl.o
endif
