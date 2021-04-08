obj-m += sht3x.o


all:
	@$(MAKE) -C $(KERNEL_DIR)  ARCH=$(ARCH) CROSS_COMPILE=$(CROSS_COMPILE) M=$(shell pwd) modules

clean:
	@$(MAKE) -C $(KERNEL_DIR) M=$(shell pwd) CROSS_COMPILE=$(CROSS_COMPILE) clean
