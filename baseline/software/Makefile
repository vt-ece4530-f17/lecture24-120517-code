ALT_DEVICE_FAMILY ?= soc_cv_av
SOCEDS_ROOT ?= c:/intelFPGA_lite/17.0/embedded
HWLIBS_ROOT = $(SOCEDS_ROOT)/ip/altera/hps/altera_hps/hwlib
CROSS_COMPILE = arm-linux-gnueabihf-
CROSS_CFLAGS = -g -Wall \
		-D$(ALT_DEVICE_FAMILY) \
		-I../conversion		\
		-I$(HWLIBS_ROOT)/include/$(ALT_DEVICE_FAMILY) \
		-I$(HWLIBS_ROOT)/include/
CROSS_LDFLAGS = -g -Wall -lm
CROSS_CC = $(CROSS_COMPILE)gcc
CROSS_OBJDUMP= $(CROSS_COMPILE)objdump
CROSS_SIZE= $(CROSS_COMPILE)size
FUNC_CC = gcc -lm -DNODE1SOC

build: correlate shiftdemo

correlate: correlate.c
	$(CROSS_CC) $(CROSS_CFLAGS) $(CROSS_LDFLAGS) $^ -o $@
	$(CROSS_OBJDUMP) -D correlate > correlate.lst
	$(CROSS_SIZE) correlate
	$(FUNC_CC) $^ -o correlate_x86

shiftdemo: shiftdemo.c
	$(CROSS_CC) $(CROSS_CFLAGS) $(CROSS_LDFLAGS) $^ -o $@
	$(CROSS_OBJDUMP) -D shiftdemo > shiftdemo.lst
	$(CROSS_SIZE) shiftdemo

%.o : %.c
	$(CROSS_CC) $(CROSS_CFLAGS) -c $< -o $@

.PHONY: clean

clean:
	rm -f correlate shiftdemo correlate_x86 *.lst wave*.txt *.a *.o *~
