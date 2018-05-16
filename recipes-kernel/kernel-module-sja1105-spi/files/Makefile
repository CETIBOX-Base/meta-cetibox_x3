KERNELDIR ?= /lib/modules/`uname -r`/build
PWD := $(shell pwd)

obj-m += sja1105_spi.o

sja1105_spi-objs := switch-spi.o

default:
	$(MAKE) -C $(KERNELDIR) M=$(PWD) modules

install: default
	$(MAKE) -C $(KERNELDIR) M=$(PWD) modules_install

all: default

clean:
	$(MAKE) -C $(KERNELDIR) M=$(PWD) clean

help:
	@echo 'Building switch abstraction layer as kernel module for Linux kernel.'
	@echo ''
	@echo '  make            - build the productive module'
	@echo '  make clean      - remove generated files in module directory'
	@echo ''
