#
# Makefile for miscellaneous I2C chip drivers.
#
# Do not add new drivers to this directory! It is DEPRECATED.
#
# Device drivers are better grouped according to the functionality they
# implement rather than to the bus they are connected to. In particular:
# * Hardware monitoring chip drivers go to drivers/hwmon
# * RTC chip drivers go to drivers/rtc
# * I/O expander drivers go to drivers/gpio
#
#proximity-objs := cm3629.o

#NFC ++
ifeq ($(findstring JAPAN,$(PRIVATE_SKU_NAME)),JAPAN)
obj-$(CONFIG_SENSORS_NFC_CXD224X) += cxd224x-i2c.o
endif
#NFC --
