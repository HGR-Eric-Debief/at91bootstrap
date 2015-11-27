CPPFLAGS += -DCONFIG_SAMA5D2_PROTO888A
ASFLAGS += -DCONFIG_SAMA5D2_PROTO888A

#Add MMU support
COBJS-$(CONFIG_WITH_MMU) += $(TOPDIR)/board/$(BOARDNAME)/driver/cp15.o
SOBJS-$(CONFIG_WITH_MMU) += $(TOPDIR)/board/$(BOARDNAME)/driver/cp15_asm_gcc.o
COBJS-$(CONFIG_WITH_MMU) += $(TOPDIR)/board/$(BOARDNAME)/driver/mmu.o
COBJS-$(CONFIG_WITH_MMU) += $(TOPDIR)/board/$(BOARDNAME)/driver/board_memories.o

ifeq ($(CONFIG_WITH_MMU),y)
CPPFLAGS += -I$(TOPDIR)/board/$(BOARDNAME)/driver
endif

#Add DMA support file
COBJS-$(CONFIG_DATAFLASH_LOAD_WITH_DMA) += $(TOPDIR)/board/$(BOARDNAME)/dma_hardware_interface.o


#Add the memory lowlevel init code according to the actual type : only LP-DDR1 !
COBJS-$(CONFIG_LPDDR1) += $(TOPDIR)/board/$(BOARDNAME)/lpddr1.o
COBJS-$(CONFIG_RAM_CHIP_W948D2) += $(TOPDIR)/board/$(BOARDNAME)/wd948d2.o
