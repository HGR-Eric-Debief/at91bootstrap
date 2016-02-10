CPPFLAGS += -DCONFIG_SAMA5D2_PROTO888A
ASFLAGS += -DCONFIG_SAMA5D2_PROTO888A

BOARD_DRIVERS_SRC = $(TOPDIR)/board/$(BOARDNAME)/driver
BOARD_INC_PATH = $(TOPDIR)/board/$(BOARDNAME)/include

#Add the board's dedicated include directory.
CPPFLAGS += -I$(BOARD_INC_PATH)


#Add MMU support
COBJS-$(CONFIG_WITH_MMU) += $(BOARD_DRIVERS_SRC)/cp15.o
SOBJS-$(CONFIG_WITH_MMU) += $(BOARD_DRIVERS_SRC)/cp15_asm_gcc.o
COBJS-$(CONFIG_WITH_MMU) += $(BOARD_DRIVERS_SRC)/mmu.o
COBJS-$(CONFIG_WITH_MMU) += $(BOARD_DRIVERS_SRC)/board_memories.o

ifeq ($(CONFIG_WITH_MMU),y)
CPPFLAGS += -I$(BOARD_DRIVERS_SRC)
endif


#Add the memory lowlevel init code according to the actual type : only LP-DDR1 !
COBJS-$(CONFIG_LPDDR1) += $(BOARD_DRIVERS_SRC)/lpddr1.o
COBJS-$(CONFIG_RAM_CHIP_W948D2) += $(BOARD_DRIVERS_SRC)/wd948d2.o
