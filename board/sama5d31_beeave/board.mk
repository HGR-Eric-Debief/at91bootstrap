$(shell $(CC) --target-help > tmp.file)
gcc_cortexa5=$(shell grep cortex-a5 tmp.file)

ifeq (, $(findstring cortex-a5,$(gcc_cortexa5)))
CPPFLAGS += -DCONFIG_SAMA5D31_BEEAVE

ASFLAGS += \
	-DCONFIG_SAMA5D31_BEEAVE
else
CPPFLAGS += \
	-DCONFIG_SAMA5D31_BEEAVE \
	-mtune=cortex-a5

ASFLAGS += \
	-DCONFIG_SAMA5D31_BEEAVE
	-mcpu=cortex-a5
endif

$(shell rm tmp.file)

BOARD_DRIVERS_SRC = $(TOPDIR)/board/$(BOARDNAME)/driver
BOARD_INC_PATH = $(TOPDIR)/board/$(BOARDNAME)/include

#Add the board's dedicated include directory.
CPPFLAGS += -I$(BOARD_INC_PATH)

#Add MMU support
COBJS-$(CONFIG_WITH_MMU) += $(BOARD_DRIVERS_SRC)/CP15.o
SOBJS-$(CONFIG_WITH_MMU) += $(BOARD_DRIVERS_SRC)/cp15_asm_gcc.o
COBJS-$(CONFIG_WITH_MMU) += $(BOARD_DRIVERS_SRC)/mmu.o

ifeq ($(CONFIG_WITH_MMU),y)
CPPFLAGS += -I$(BOARD_DRIVERS_SRC)
endif

#Add DMA support file
COBJS-$(CONFIG_DATAFLASH_LOAD_WITH_DMA) += $(BOARD_DRIVERS_SRC)/dma_dev.o
COBJS-$(CONFIG_DATAFLASH_LOAD_WITH_DMA) += $(BOARD_DRIVERS_SRC)/dmad.o
COBJS-$(CONFIG_DATAFLASH_LOAD_WITH_DMA) += $(BOARD_DRIVERS_SRC)/at91_dma.o
COBJS-$(CONFIG_DATAFLASH_LOAD_WITH_DMA) += $(BOARD_DRIVERS_SRC)/dma_hardware_interface.o


#Add the memory lowlevel init code according to the actual type : only LP-DDR1 !
COBJS-$(CONFIG_LPDDR1) += $(BOARD_DRIVERS_SRC)/lpddr1.o
COBJS-$(CONFIG_RAM_CHIP_IS43LR32800) += $(BOARD_DRIVERS_SRC)/is43lr32800.o
COBJS-$(CONFIG_RAM_CHIP_W948D2) += $(BOARD_DRIVERS_SRC)/wd948d2.o
