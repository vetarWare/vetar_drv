# This is useful if cross-compiling. Taken from kernel Makefile (CC changed)
#AS      =$(CROSS_COMPILE)as
#LD      =$(CROSS_COMPILE)ld
#CC      =$(CROSS_COMPILE)gcc
#CPP     =$(CC) -E
#AR      =$(CROSS_COMPILE)ar
#NM      =$(CROSS_COMPILE)nm
#STRIP   =$(CROSS_COMPILE)strip
#OBJCOPY =$(CROSS_COMPILE)objcopy
#OBJDUMP =$(CROSS_COMPILE)objdump


KERNELVER ?= `uname -r`
KERNELDIR ?= /lib/modules/$(KERNELVER)/build

ifneq ($(KERNELRELEASE),)
	obj-m	:= vetar.o
	ccflags-y := -I/root/vetar_drv/vmebridge/driver
	vetar-objs := vetar-drv.o
else
	KERNELDIR ?= /lib/modules/$(shell uname -r)/build
	PWD       := $(shell pwd)

//KBUILD_EXTRA_SYMBOLS = $(M)/Module.symvers.vme
//KBUILD_EXTRA_SYMBOLS = $(M)/Module.symvers

all: modules

modules:
	$(MAKE) -C $(KERNELDIR) M=$(PWD)

install:
	$(MAKE) -C $(KERNELDIR) M=$(PWD) modules_install

endif

clean:
	rm -rf *.o *~ core .depend .*.cmd *.ko *.mod.c .tmp_versions

