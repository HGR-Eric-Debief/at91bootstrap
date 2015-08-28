#
# Default config file is in $(TOPDIR)/board/$(BOARD_NAME)/*_defconfig
# First, run xxx_defconfig
# Then, `make menuconfig' if needed
#

# Do not:
# o  use make's built-in rules and variables
#    (this increases performance and avoids hard-to-debug behaviour);
# o  print "Entering directory ...";
MAKEFLAGS += -rR --no-print-directory

TOPDIR=$(shell pwd)

CONFIG_CONFIG_IN=Config.in
CONFIG_DEFCONFIG=.defconfig
CONFIG=config

CONFIG_SHELL=$(shell which bash)
ifeq ($(CONFIG_SHELL),)
$(error GNU Bash is needed to build Bootstrap!)
endif

BINDIR:=$(TOPDIR)/binaries


DATE := $(shell date --rfc-3339=seconds)
REVISION := alpha5
SCMINFO := $(shell ($(TOPDIR)/host-utilities/setlocalversion $(TOPDIR)))

ifeq ($(SCMINFO),)
-include $(TOPDIR)/scminfo.mk
SCMINFO=$(RECORD_SCMINFO)
endif

TARBALL_NAME=AT91Bootstrap-$(VERSION)$(REV)$(SCMINFO).tar.gz
TARBALL_DIR=../tarball_dir
TARBALL_PREFIX=at91bootstrap-$(VERSION)$(REV)/

# Use 'make V=1' for the verbose mode
ifneq ("$(origin V)", "command line")
Q=@
export Q
endif

noconfig_targets:= menuconfig defconfig $(CONFIG) oldconfig savedefconfig

# Check first if we want to configure at91bootstrap
#
ifeq ($(filter $(noconfig_targets),$(MAKECMDGOALS)),)
-include .config
endif

include	host-utilities/host.mk

ifeq ($(HAVE_DOT_CONFIG),)

all: menuconfig

# configuration
# ---------------------------------------------------------------------------

HOSTCFLAGS=$(CFLAGS_FOR_BUILD)
export HOSTCFLAGS

$(CONFIG)/conf:
	@mkdir -p $(CONFIG)/at91bootstrap-config
	@$(MAKE) CC="$(HOSTCC)" -C $(CONFIG) conf
	-@if [ ! -f .config ]; then \
		cp $(CONFIG_DEFCONFIG) .config; \
	fi

$(CONFIG)/mconf:
	@mkdir -p $(CONFIG)/at91bootstrap-config
	@$(MAKE) CC="$(HOSTCC)" -C $(CONFIG) conf mconf
	-@if [ ! -f .config ]; then \
		cp $(CONFIG_DEFCONFIG) .config; \
	fi

menuconfig: $(CONFIG)/mconf
	@mkdir -p $(CONFIG)/at91bootstrap-config
	@if ! KCONFIG_AUTOCONFIG=$(CONFIG)/at91bootstrap-config/auto.conf \
		KCONFIG_AUTOHEADER=$(CONFIG)/at91bootstrap-config/autoconf.h \
		$(CONFIG)/mconf $(CONFIG_CONFIG_IN); then \
		test -f .config.cmd || rm -f .config; \
	fi

$(CONFIG): $(CONFIG)/conf
	@mkdir -p $(CONFIG)/at91bootstrap-config
	@KCONFIG_AUTOCONFIG=$(CONFIG)/at91bootstrap-config/auto.conf \
		KCONFIG_AUTOHEADER=$(CONFIG)/at91bootstrap-config/autoconf.h \
		$(CONFIG)/conf $(CONFIG_CONFIG_IN)

oldconfig: $(CONFIG)/conf
	@mkdir -p $(CONFIG)/at91bootstrap-config
	@KCONFIG_AUTOCONFIG=$(CONFIG)/at91bootstrap-config/auto.conf \
		KCONFIG_AUTOHEADER=$(CONFIG)/at91bootstrap-config/autoconf.h \
		$(CONFIG)/conf --oldconfig $(CONFIG_CONFIG_IN)

defconfig: $(CONFIG)/conf
	@mkdir -p $(CONFIG)/at91bootstrap-config
	@KCONFIG_AUTOCONFIG=$(CONFIG)/at91bootstrap-config/auto.conf \
		KCONFIG_AUTOHEADER=$(CONFIG)/at91bootstrap-config/autoconf.h \
		$(CONFIG)/conf --defconfig=.config $(CONFIG_CONFIG_IN)

savedefconfig: $(CONFIG)/conf
	@mkdir -p $(CONFIG)/at91bootstrap-config
	@KCONFIG_AUTOCONFIG=$(CONFIG)/at91bootstrap-config/auto.conf \
		KCONFIG_AUTOHEADER=$(CONFIG)/at91bootstrap-config/autoconf.h \
		$(CONFIG)/conf --savedefconfig=defconfig $(CONFIG_CONFIG_IN)

else #  Have DOT Config

HOSTARCH := $(shell uname -m | sed -e s/arm.*/arm/)

AS=$(CROSS_COMPILE)gcc
CC=$(CROSS_COMPILE)gcc
LD=$(CROSS_COMPILE)ld
NM= $(CROSS_COMPILE)nm
SIZE=$(CROSS_COMPILE)size
OBJCOPY=$(CROSS_COMPILE)objcopy
OBJDUMP=$(CROSS_COMPILE)objdump

PROJECT := $(strip $(subst ",,$(CONFIG_PROJECT)))
IMG_ADDRESS := $(strip $(subst ",,$(CONFIG_IMG_ADDRESS)))
IMG_SIZE := $(strip $(subst ",,$(CONFIG_IMG_SIZE)))
JUMP_ADDR := $(strip $(subst ",,$(CONFIG_JUMP_ADDR)))
OF_OFFSET := $(strip $(subst ",,$(CONFIG_OF_OFFSET)))
OF_ADDRESS := $(strip $(subst ",,$(CONFIG_OF_ADDRESS)))
BOOTSTRAP_MAXSIZE := $(strip $(subst ",,$(CONFIG_BOOTSTRAP_MAXSIZE)))
MEMORY := $(strip $(subst ",,$(CONFIG_MEMORY)))
IMAGE_NAME:= $(strip $(subst ",,$(CONFIG_IMAGE_NAME)))
CARD_SUFFIX := $(strip $(subst ",,$(CONFIG_CARD_SUFFIX)))
MEM_BANK := $(strip $(subst ",,$(CONFIG_MEM_BANK)))
MEM_SIZE := $(strip $(subst ",,$(CONFIG_MEM_SIZE)))
LINUX_KERNEL_ARG_STRING := $(strip $(subst ",,$(CONFIG_LINUX_KERNEL_ARG_STRING)))

# Board definitions
BOARDNAME:=$(strip $(subst ",,$(CONFIG_BOARDNAME)))

MACH_TYPE:=$(strip $(subst ",,$(CONFIG_MACH_TYPE)))
LINK_ADDR:=$(strip $(subst ",,$(CONFIG_LINK_ADDR)))
DATA_SECTION_ADDR:=$(strip $(subst ",,$(CONFIG_DATA_SECTION_ADDR)))
TOP_OF_MEMORY:=$(strip $(subst ",,$(CONFIG_TOP_OF_MEMORY)))

# CRYSTAL is UNUSED
CRYSTAL:=$(strip $(subst ",,$(CONFIG_CRYSTAL)))

# driver definitions
SPI_CLK:=$(strip $(subst ",,$(CONFIG_SPI_CLK)))
QSPI_CLK:=$(strip $(subst ",,$(CONFIG_QSPI_CLK)))
SPI_BOOT:=$(strip $(subst ",,$(CONFIG_SPI_BOOT)))

#Add name decoration according to the BUILD destination.

ifeq ($(CONFIG_DATAFLASH_LOAD_WITH_DMA),y)
REVISION :=$(REVISION)-FASTLOAD
endif

ifeq ($(CONFIG_BUILD_RELEASE),)
REVISION :=$(REVISION)-DEBUG
endif

ifeq ($(CONFIG_WITH_MMU),y)
REVISION :=$(REVISION)-MMU
endif

ifeq ($(CONFIG_WITH_CACHE),y)
REVISION :=$(REVISION)-CACHE
endif

ifeq ($(CONFIG_DEBUG_3RD_STAGE),y)
REVISION :=$(REVISION)-DBG_3RD
endif

ifeq ($(CONFIG_CHECK_APPLICATION_LOAD),y)
REVISION :=$(REVISION)-CHECKED_APPLI
endif

#Add RAM type in name, TODO : to be refactored : set it in boards' directories.
ifeq ($(CONFIG_ONLY_INTERNAL_RAM),y)
REVISION :=$(REVISION)-SRAM
endif

ifeq ($(CONFIG_DDR2),y)
REVISION :=$(REVISION)-DDR2
endif

#LP-DDR1 : only known chips
ifeq ($(CONFIG_LPDDR1),y)

ifeq ($(CONFIG_RAM_CHIP_IS43LR32400),y)
REVISION :=$(REVISION)-IS43LR32400
endif

ifeq ($(CONFIG_RAM_CHIP_IS43LR32800),y)
REVISION :=$(REVISION)-IS43LR32800
endif

ifeq ($(CONFIG_RAM_CHIP_W948D2),y)
REVISION :=$(REVISION)-WD948D2
endif

endif

ifeq ($(CONFIG_LPDDR2),y)
REVISION :=$(REVISION)-LPDDR2
endif


ifeq ($(CONFIG_EXTERNAL_RAM_TEST),y)
REVISION := $(REVISION)-CHECK

ifeq ($(CONFIG_EXTERNAL_RAM_TEST_INFINITE),y)
REVISION := $(REVISION)-INFINITE
endif

endif


ifeq ($(REVISION),)
REV:=
else
REV:=-$(strip $(subst ",,$(REVISION)))
endif

ifeq ($(CONFIG_OF_LIBFDT), y)
BLOB:=-dt
else
BLOB:=
endif

#Default value
TARGET_NAME := spinning

ifeq ($(CONFIG_LOAD_LINUX), y)
TARGET_NAME:=linux-$(subst I,i,$(IMAGE_NAME))
endif

ifeq ($(CONFIG_LOAD_ANDROID), y)
TARGET_NAME:=android-$(subst I,i,$(IMAGE_NAME))
endif

ifeq ($(CONFIG_LOAD_UBOOT), y)
TARGET_NAME:=$(subst -,,$(basename $(IMAGE_NAME)))
endif

ifeq ($(CONFIG_LOAD_64KB), y)
TARGET_NAME:=$(basename $(IMAGE_NAME))
endif

ifeq ($(CONFIG_LOAD_1MB), y)
TARGET_NAME:=$(basename $(IMAGE_NAME))
endif

ifeq ($(CONFIG_LOAD_4MB), y)
TARGET_NAME:=$(basename $(IMAGE_NAME))
endif

BOOT_NAME=$(BOARDNAME)-$(PROJECT)$(CARD_SUFFIX)boot-$(TARGET_NAME)$(BLOB)-$(VERSION)$(REV)
AT91BOOTSTRAP:=$(BINDIR)/$(BOOT_NAME).bin

ifeq ($(IMAGE),)
IMAGE=$(BOOT_NAME).bin
endif

ifeq ($(SYMLINK),)
SYMLINK=at91bootstrap.bin
SYMLINK_ELF=at91bootstrap.elf
endif

ifeq ($(SYMLINK_BOOT),)
SYMLINK_BOOT=boot.bin
endif

COBJS-y:= $(TOPDIR)/main.o $(TOPDIR)/board/$(BOARDNAME)/$(BOARDNAME).o
SOBJS-y := $(TOPDIR)/crt0_gnu.o


include	lib/lib.mk
include	driver/driver.mk
include	fs/src/fat.mk

#$(SOBJS-y:.o=.S)

INCL=board/$(BOARDNAME)
GC_SECTIONS=--gc-sections

NOSTDINC_FLAGS=-nostdinc -isystem $(shell $(CC) -print-file-name=include)

CPPFLAGS=$(NOSTDINC_FLAGS) -ffunction-sections -Wall \
	-fno-stack-protector -fno-common \
	-I$(INCL) -Iinclude -Ifs/include -I$(TOPDIR)/config/at91bootstrap-config \
	-DAT91BOOTSTRAP_VERSION=\"$(VERSION)$(REV)\" -DCOMPILE_TIME="\"$(DATE)\"" \
	-DBOARD_NAME=\"$(BOARDNAME)\"

ASFLAGS=-Wall -I$(INCL) -Iinclude

## Release BUILD
ifeq ($(CONFIG_BUILD_RELEASE),y)
## debug Release
CPPFLAGS += -Os
ASFLAGS += -Os
#Fake RELEASE => DEBUG from startup.
#CPPFLAGS += -g -Os
#ASFLAGS += -g -Os
else
CPPFLAGS += -g -g3 -O0
ASFLAGS += -g -O0
endif

include	toplevel_cpp.mk
include	board/board_cpp.mk
include	driver/driver_cpp.mk

#Must be defined once all include hase been done as new source files can be added in included MAKE rules file.
SRCS:= $(COBJS-y:.o=.c)
OBJS:= $(SOBJS-y) $(COBJS-y)

ifeq ($(CONFIG_ENTER_NWD), y)
link_script:=elf32-littlearm-tz.lds
else
link_script:=elf32-littlearm.lds
endif

# Linker flags.
#Thumb mode or DEBUG build (-O0) need some libgcc support functions.
#  -Wl,...:     tell GCC to pass this to linker.
#    -Map:      create map file
#    --cref:    add cross reference to map file
#  -lc 	   : 	tells the linker to tie in newlib
#  -lgcc   : 	tells the linker to tie in libgcc
# -L$(shell dirname `$(CC) --print-libgcc-file-name`) : got the libgcc file path.
LDFLAGS+=-nostartfiles -Map=$(BINDIR)/$(BOOT_NAME).map --cref -static
LDFLAGS+=-T $(link_script) $(GC_SECTIONS) -Ttext $(LINK_ADDR)
LDFLAGS+= -L$(shell dirname `$(CC) --print-libgcc-file-name`)
LIBS+=-lgcc

ifneq ($(DATA_SECTION_ADDR),)
LDFLAGS+=-Tdata $(DATA_SECTION_ADDR)
endif


gccversion := $(shell expr `$(CC) -dumpversion`)

ifdef YYY   # For other utils
ifeq ($(CC),gcc) 
TARGETS=no-cross-compiler
else
TARGETS=$(AT91BOOTSTRAP) host-utilities .config filesize
endif
endif

TARGETS=$(AT91BOOTSTRAP)

PHONY:=all

all: CheckCrossCompile PrintFlags $(AT91BOOTSTRAP) ChkFileSize

CheckCrossCompile:
	@( if [ "$(HOSTARCH)" != "arm" ]; then \
		if [ "x$(CROSS_COMPILE)" = "x" ]; then \
			echo "error: Environment variable "CROSS_COMPILE" must be defined!"; \
			exit 2; \
		fi \
	fi )

PrintFlags:
ifeq ($(CONFIG_BUILD_RELEASE),y)
	@echo " !! --- RELEASE BUILD --- !! " && echo
else
	@echo " !! --- DEBUG BUILD --- !! " && echo
endif
ifeq ($(CONFIG_DEBUG_3RD_STAGE),y)
	@echo " !! --- WAIT FOR DEBUGGER ONCE THIRD STAGE LOADED --- !!"
endif
	@echo Driver config
	@echo ========
	@echo SPI_CLCK : $(SPI_CLK)
	@echo SPI_BOOT : $(SPI_BOOT) && echo
	@echo CC
	@echo ========
	@echo $(CC) $(gccversion)&& echo
	@echo as FLAGS
	@echo ========
	@echo $(ASFLAGS) && echo
	@echo gcc FLAGS
	@echo =========
	@echo $(CPPFLAGS) && echo
	@echo ld FLAGS
	@echo ========
	@echo $(LDFLAGS) && echo
	@echo ======
	@echo Sources : $(SRCS) && echo
	@echo ======
	@echo Boot name : $(BOOT_NAME)
	
$(AT91BOOTSTRAP): $(OBJS)
	$(if $(wildcard $(BINDIR)),,mkdir -p $(BINDIR))
	@echo "  LD        "$(BOOT_NAME).elf
	@$(LD) $(LDFLAGS) -n -o $(BINDIR)/$(BOOT_NAME).elf $(OBJS) $(LIBS)
#	@$(OBJCOPY) --strip-debug --strip-unneeded $(BINDIR)/$(BOOT_NAME).elf -O binary $(BINDIR)/$(BOOT_NAME).bin
	@$(OBJCOPY) --strip-all $(BINDIR)/$(BOOT_NAME).elf -O binary $@
	@ln -sf $(BOOT_NAME).bin ${BINDIR}/${SYMLINK}
	@ln -sf $(BOOT_NAME).bin ${BINDIR}/${SYMLINK_BOOT}
	@ln -sf $(BOOT_NAME).elf ${BINDIR}/${SYMLINK_ELF}

%.o : %.c .config
	@echo "  CC        "$<
	@$(CC) $(CPPFLAGS) -c -o $@ $<

%.o : %.S .config
	@echo "  AS        "$<
	@$(AS) $(ASFLAGS)  -c -o $@  $<

PHONY+= bootstrap

rebuild: clean all

ChkFileSize: $(AT91BOOTSTRAP)
	@( fsize=`./scripts/get_sram_size.sh $(BINDIR)/$(BOOT_NAME).map`; \
	  if [ $$? -ne 0 ] ; then \
		rm $(BINDIR)/*.bin ;\
		rm ${BINDIR}/*.elf; \
		rm ${BINDIR}/*.map; \
		exit 3; \
	  fi ; \
	  echo "Size of $(BOOT_NAME).bin is $$fsize bytes"; \
	  if [ "$$fsize" -gt "$(BOOTSTRAP_MAXSIZE)" ] ; then \
		echo "[Failed***] It's too big to fit into SRAM area. the support maximum size is $(BOOTSTRAP_MAXSIZE)"; \
		rm $(BINDIR)/*.bin ;\
		rm ${BINDIR}/*.elf; \
		rm ${BINDIR}/*.map; \
		exit 2;\
	  else \
	  	echo "[Succeeded] It's OK; it fits into SRAM area ($(BOOTSTRAP_MAXSIZE) bytes)"; \
		stack_space=`expr $(BOOTSTRAP_MAXSIZE) - $$fsize`; \
		echo "[Attention] The space left for stack is $$stack_space bytes"; \
	  fi )
endif  # HAVE_DOT_CONFIG

PHONY+= rebuild

%_defconfig:
	@(conf_file=`find ./board -name $@`; \
	if [ "$$conf_file"x != "x" ]; then \
		cp $$conf_file .config; \
	else \
		echo "Error: *** Cannot find file: $@"; \
		exit 2; \
	fi )
	@$(MAKE) defconfig

update:
	cp .config board/$(BOARDNAME)/$(BOARDNAME)_defconfig

no-cross-compiler:
	@echo
	@echo
	@echo
	@echo "	You should consider using a cross compiler for ARM !"
	@echo "	I.E: CROSS_COMPILE should contains something useful."
	@echo

debug:
	@echo CONFIG=$(CONFIG)
	@echo AS=$(AS)
	@echo CROSS_COMPILE=$(CROSS_COMPILE)

PHONY+=update no-cross-compiler debug

distrib: mrproper
	$(Q)find . -type f \( -name .depend \
		-o -name '*.srec' \
		-o -name '*.elf' \
		-o -name '*.map' \
		-o -name '*.o' \
		-o -name '*~' \) \
		-print0 \
		| xargs -0 rm -f
	$(Q)rm -fr result
	$(Q)rm -fr build
	$(Q)rm -fr ..make.deps.tmp
	$(Q)rm -fr config/conf

config-clean:
	@echo "  CLEAN        "configuration files!
	$(Q)make -C config distclean
	$(Q)rm -fr config/at91bootstrap-config
	$(Q)rm -f  config/.depend

clean:
	@echo "  CLEAN        "obj and misc files!
	$(Q)find . -type f \( -name .depend \
		-o -name '*.srec' \
		-o -name '*.o' \
		-o -name '*~' \
		-o -name '*.S.txt' \) \
		-print0 \
		| xargs -0 rm -f

distclean: clean config-clean
#	rm -fr $(BINDIR)
	$(Q)rm -fr .config .config.cmd .config.old
	$(Q)rm -fr .auto.deps
	$(Q)rm -f .installed
	$(Q)rm -f ..*.tmp
	$(Q)rm -f .configured

mrproper: distclean
	@echo "  CLEAN        "binary files!
	$(Q)rm -fr $(BINDIR)
	$(Q)rm -fr log

PHONY+=distrib config-clean clean distclean mrproper

tarball:
	@echo "Tar the source code to ${TARBALL_NAME}"
	$(Q)mkdir -p ${TARBALL_DIR}
	$(Q)git archive --prefix=${TARBALL_PREFIX} HEAD | gzip > ${TARBALL_DIR}/${TARBALL_NAME}
	$(Q)echo "RECORD_SCMINFO=${SCMINFO}" > ${TARBALL_DIR}/scminfo.mk
	$(Q)cd ${TARBALL_DIR}; tar -xzf ${TARBALL_NAME}
	$(Q)cd ${TARBALL_DIR}; cp scminfo.mk ${TARBALL_PREFIX}
	$(Q)cd ${TARBALL_DIR}; tar czf ${TARBALL_NAME} ${TARBALL_PREFIX}
	$(Q)cp ${TARBALL_DIR}/${TARBALL_NAME} .
	$(Q)rm -rf ${TARBALL_DIR}

PHONY+=tarball

disassembly: all
	$(OBJDUMP) -DS $(BINDIR)/$(BOOT_NAME).elf > $(BINDIR)/$(BOOT_NAME).S.txt

PHONY += disassembly

.PHONY: $(PHONY)
