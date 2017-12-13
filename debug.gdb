target remote :2331
#monitor halt
file binaries/at91bootstrap.elf
load
break main
#break crt0_gnu.S:163
#break check_32bit_Ram_2Phases
#break check_32bit_Ram_2Phases:CHECK
#break check_32bit_Ram_2Phases:ERROR
break main:LOAD_IMAGE_BEGIN
break main:LOAD_IMAGE_END
#break load_image_done
#break load_dataflash
#break df_send_command_PIO
#break spi_flash_loadimage
#break spi_flash_loadimage:ENDLESS_LOOP
#break at91_spi0_hw_init
#break at91_spi_init
#break pio4_set_periph
#break ddramc_init
#break hw_init
#break dataflash_probe_chip
##DMA support
#break dataflash_read_array_DMA
#break df_send_command_DMA
#break DMA_DEV_OpenSPIIOStream
#break DMA_DEV_SPICommandResponse
#break DMA_DEV_SPICommandResponse:CONFIG
#break DMA_DEV_SPICommandResponse:START
#break DMA_DEV_QSPICommandResponse
#break DMA_DEV_QSPICommandResponse:CONFIG
#break DMA_DEV_QSPICommandResponse:START
#break xdmad_configure_transfer
#break xdmad_handler
#b DMA_MEM_copy
## DMA context print function, channel 0,1
define print_xdma_ctxt
echo TX(Channel 0) \n
echo CSA, CDA, CIM, (CIS), CNDA, CNDC, CUBC, CC \n
 p/x *0xF0010060
 p/x *0xF0010064
 p/x *0xF0010058
 # p/x *0xF001005C
 p/x *0xF0010068
 p/x *0xF001006C 
 p/x *0xF0010070 
 p/x *0xF0010078 
echo RX(Channel 1) \n
echo CSA, CDA, CIM, (CIS), CNDA, CNDC, CUBC, CC \n
 p/x *0xF00100A0
 p/x *0xF00100A4
 p/x *0xF0010098 
 # p/x *0xF001009C
 p/x *0xF00100A8
 p/x *0xF00100AC
 p/x *0xF00100B0
 p/x *0xF00100B8
end
#QSPI/DMA debug support
#b qspi_flash_loadimage
#b qspi_flash_read_jedec_id
#b qspi_flash_read_image
#b qspi_send_command
#b qspi_init
#b qspi_send_command:BEFORE_DATA_OP if data->size > 10
#b qspi_init_raw
#b qspi_send_command_raw
#b qspi_send_command_dma
#b qspi_send_command_raw_dma
#b qspi_flash_read_image_raw_dma
#b qspi_flash_read_image_dma
#b qspi_flash_set_quad_mode
#b qspi_flash_set_driver_strength
#b qspi_flash_read_image_smm
#b qspi_flash_read_image_smm_dma

# FUNCTION To load the application (RELEASE) from disk into the RAM
define load_rel_appli
 restore /home/eric/Projets/ModuleVideo/EclipseWorkspaces_Mars/CARIROutdoor/BeeAVEApp/AppRel-358B/BeeAVEApp.bin binary 0x20000000 0 300*1024
 file /home/eric/Projets/ModuleVideo/EclipseWorkspaces_Mars/CARIROutdoor/BeeAVEApp/AppRel-358B/BeeAVEApp.elf
 b main
 b APP::BSP_init
end

# FUNCTION To load the application (DEBUG) from disk into the RAM
define load_dbg_appli
 restore /home/eric/Projets/ModuleVideo/EclipseWorkspaces_Mars/CARIROutdoor/BeeAVEApp/AppDbg-358B/BeeAVEApp.bin binary 0x20000000 0 300*1024
 file /home/eric/Projets/ModuleVideo/EclipseWorkspaces_Mars/CARIROutdoor/BeeAVEApp/AppDbg-358B/BeeAVEApp.elf
 b main
 b APP::BSP_init
 b QF_undef
#b QK_irq
#b QF_pAbort
 b QF_dAbort
 b Q_onAssert
 b QF_except
 b BSP_onCPUException
end