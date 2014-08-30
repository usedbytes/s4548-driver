MODULES := s4548.o

#guest architecture
ARCH := arm

CROSS_COMPILE := arm-none-linux-gnueabi-
obj-m := $(MODULES)

#path of the arm compiled kernel
ROOTDIR := /home/kernelcode/raspberrypi/kernel/linux

MAKEARCH := $(MAKE) ARCH=$(ARCH) CROSS_COMPILE=$(CROSS_COMPILE)

all: modules
modules:
	$(MAKEARCH) -C $(ROOTDIR) M=${shell pwd} modules

clean:
	$(MAKEARCH) -C $(ROOTDIR) M=${shell pwd} clean
