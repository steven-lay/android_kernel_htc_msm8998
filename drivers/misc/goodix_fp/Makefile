ifeq ($(KERNELRELEASE),)

KDIR ?= /lib/modules/$(shell uname -r)/build

PWD := $(shell pwd)

modules:
	$(MAKE) -C $(KDIR) M=$(PWD) modules

modules_install:
	$(MAKE) -C $(KDIR) M=$(PWD) modules_install

clean:
	rm -rf *.o *~ core .depend .*.cmd *.ko *.mod.c .tmp_versions Module.symvers modules.order

else

gf_drv_tee-objs := gf_spi.o platform.o netlink.o
obj-$(CONFIG_GOODIX_FP)  += gf_drv_tee.o

endif
