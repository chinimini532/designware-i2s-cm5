# ===== Makefile: build designware-i2s.ko from dwc-i2s_NGT.c + designware_pcm.c =====
# Usage (host cross-compile):
#   export KDIR=/home/chinimini/linux
#   export ARCH=arm64
#   export CROSS_COMPILE=aarch64-linux-gnu-
#   make
#
# Usage (native on CM5):
#   export KDIR=/lib/modules/$(uname -r)/build
#   make

KDIR ?= /home/chinimini/linux
ARCH ?= arm64
CROSS_COMPILE ?= aarch64-linux-gnu-

# Build one module from two objects
obj-m += designware-i2s-NGT.o
designware-i2s-NGT-objs := dwc-i2s-ngt.o  dwc-pcm.o

# Detect the kernel *source* dir even if kernel was built with O=<outdir>
# Pure Make conditionals (portable with /bin/sh)
ifeq ($(wildcard $(KDIR)/source/.),)
  KSRCDIR := $(KDIR)
else
  KSRCDIR := $(KDIR)/source
endif

# Add the path where local.h lives in the kernel tree
ifneq ($(wildcard $(KSRCDIR)/sound/soc/dwc/local.h),)
  ccflags-y += -I$(KSRCDIR)/sound/soc/dwc
else
  $(warning Could not find $(KSRCDIR)/sound/soc/dwc/local.h. \
Set KDIR to your kernel build dir; e.g. export KDIR=/home/chinimini/linux)
endif

.PHONY: all clean modules_install

all:
	$(MAKE) -C $(KDIR) ARCH=$(ARCH) CROSS_COMPILE=$(CROSS_COMPILE) \
		M=$(PWD) modules

modules_install:
	@if [ -z "$(DESTDIR)" ]; then \
	  echo "ERROR: Set DESTDIR=/absolute/path/to/mounted/rootfs"; exit 1; \
	fi
	$(MAKE) -sC $(KDIR) ARCH=$(ARCH) CROSS_COMPILE=$(CROSS_COMPILE) \
		M=$(PWD) modules_install INSTALL_MOD_PATH=$(DESTDIR)
	@KREL="$$( $(MAKE) -sC $(KDIR) ARCH=$(ARCH) CROSS_COMPILE=$(CROSS_COMPILE) kernelrelease )"; \
	echo "Installed under $(DESTDIR)/lib/modules/$$KREL/"

clean:
	$(MAKE) -C $(KDIR) ARCH=$(ARCH) CROSS_COMPILE=$(CROSS_COMPILE) \
		M=$(PWD) clean
