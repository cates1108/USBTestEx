

include ../../Makefile.project
include ../../Makefile.toolchain

CC = gcc
KERNELDIR = $(TOP_DIR)/../kernel/linux

TARGET_EXE = usbtest
SRCS_LIST = usbtest.c

all: $(TARGET_EXE)
	
install:
ifndef CONFIG_INSTALLDIR
	$(error [CHWUU] CONFIG_INSTALLDIR not define!!)
endif
	cp -arf $(TARGET_EXE) $(CONFIG_INSTALLDIR)/tmp
	
include ../../rules.mak
