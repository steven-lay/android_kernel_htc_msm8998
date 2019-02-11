# To distinguish T/inhouse project for smlog mechanism
# Modem_BSP++
ifneq (,$(filter google Google, $(PRODUCT_MANUFACTURER)))
ccflags-$(CONFIG_UIO_MSM_SHAREDMEM)+= -DIS_T_PROJECT=1
else
ccflags-$(CONFIG_UIO_MSM_SHAREDMEM)+= -DIS_T_PROJECT=0
endif
# Modem_BSP--

obj-$(CONFIG_UIO_MSM_SHAREDMEM) := \
	msm_sharedmem.o \
	remote_filesystem_access_v01.o \
	sharedmem_qmi.o \
	runtime_embedded_log.o \
