CPPFLAGS += -DCONFIG_SAMA5D3XEK
ASFLAGS += -DCONFIG_SAMA5D3XEK


#Add MMU support helpers
COBJS-$(CONFIG_WITH_MMU) += $(TOPDIR)/board/$(BOARDNAME)/CP15.o

#Add Support assembler file
SOBJS-$(CONFIG_WITH_MMU) += $(TOPDIR)/board/$(BOARDNAME)/cp15_asm_gcc.o

#Add DMA support file
COBJS-$(CONFIG_DATAFLASH_LOAD_WITH_DMA) += $(TOPDIR)/board/$(BOARDNAME)/dma_hardware_interface.o