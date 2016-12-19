/* ----------------------------------------------------------------------------
 *         ATMEL Microcontroller Software Support
 * ----------------------------------------------------------------------------
 * Copyright (c) 2015, Atmel Corporation
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * - Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the disclaimer below.
 *
 * Atmel's name may not be used to endorse or promote products derived from
 * this software without specific prior written permission.
 *
 * DISCLAIMER: THIS SOFTWARE IS PROVIDED BY ATMEL "AS IS" AND ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT ARE
 * DISCLAIMED. IN NO EVENT SHALL ATMEL BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
 * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
 * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
#include "common.h"
#include "board.h"
#include "qspi.h"
#include "string.h"
#include "debug.h"

//select the right QSPI access function  according to the QSPI access mode (static polymorphism)
//Single access mode.
#if defined CONFIG_SPI_FLASH_SINGLE_QSPI_MODE

#if defined CONFIG_QSPI_FLASH_SMM
#if defined CONFIG_DATAFLASH_LOAD_WITH_DMA 
#define qspi_flash_loadimage_in_single_mode_smm_dma qspi_flash_loadimage
#define FLASH_ACCESS_DEFINED
#else
#define qspi_flash_loadimage_in_single_mode_smm qspi_flash_loadimage
#define FLASH_ACCESS_DEFINED
#endif

#else
//No SMM
#if defined CONFIG_DATAFLASH_LOAD_WITH_DMA
#define qspi_flash_loadimage_in_single_mode_raw_dma qspi_flash_loadimage
#define FLASH_ACCESS_DEFINED
#else
#define qspi_flash_loadimage_in_single_mode_raw qspi_flash_loadimage
#define FLASH_ACCESS_DEFINED
#endif

#endif 

#endif /*Single access mode */

// Dual mode
#if defined CONFIG_SPI_FLASH_DUAL_QSPI_MODE && CONFIG_DATAFLASH_LOAD_WITH_DMA && defined CONFIG_QSPI_FLASH_SMM
#define qspi_flash_loadimage_in_dual_mode_smm_dma qspi_flash_loadimage
#define FLASH_ACCESS_DEFINED
#endif /* Dual mode */

#if defined CONFIG_SPI_FLASH_QUAD_QSPI_MODE && defined CONFIG_QSPI_FLASH_SMM
#if defined CONFIG_DATAFLASH_LOAD_WITH_DMA
#define qspi_flash_loadimage_in_quad_mode_smm_dma qspi_flash_loadimage
#define FLASH_ACCESS_DEFINED
#else
#define qspi_flash_loadimage_in_quad_mode_smm qspi_flash_loadimage
#define FLASH_ACCESS_DEFINED
#endif


#endif /* Quad mode */


//Check if a load service is defined
#if !defined FLASH_ACCESS_DEFINED
#error NO supported QSPI Flash access mode defined.
#endif

/*
 * SPI Extended Mode Commands (GENERIC)
 */
#define CMD_EXTENDED_IO_SLOW_SINGLE_READ 0x03
#define CMD_EXTENDED_IO_FAST_SINGLE_READ 0x0B
#define CMD_DUAL_OUTPUT_FAST_READ        0x3B
#define CMD_QUAD_OUTPUT_FAST_READ        0x6B


/*
 * Winond SPI Flash commands
 */
#define CMD_WINBOND_READ_SR1 0x05
#define CMD_WINBOND_READ_SR2 0x35
#define CMD_WINBOND_WRITE_STATUS_REGS 0x01
// //Below : for W25Q16F
//#define CMD_WINBOND_WRITE_SR2 0x31
//#define CMD_WINBOND_READ_SR3 0x15
//#define CMD_WINBOND_WRITE_SR3 0x11

// QE bit 2 in Status Reg 2.
#define WINBOND_BIT_QE (0x1 << 1)

// Busy bit ins SR1
#define WINBOND_BIT_BUSY (0x1 << 0)

/*
 * QSPI Flash Commands (Micron N25Q128A)
 */
#define	CMD_READ_ID				0x9f
#define	CMD_READ_EN_VOLATILE_CONFIG_REG		0x65
#define	CMD_WRITE_EN_VOLATILE_CONFIG_REG	0x61
#define	CMD_QUAD_IO_FAST_READ			0xeb

#define	CMD_WRITE_ENABLE			0x06
#define CMD_VOLATILE_WRITE_ENABLE 0x50
#define	CMD_WRITE_DISABLE			0x04

#define	CMD_READ_STATUS_REG			0x05
#define	CMD_WRITE_STATUS_REG			0x01

/* Enhanced Volatile Configuration Register Bit Definitions */
#define	EN_VOL_CONFIG_DUAL_IO		(0x1 << 6)
#define	EN_VOL_CONFIG_QUAD_IO		(0x1 << 7)

/* Status Register Bit Definitions */
#define	STATUS_WRITE_READY		(0x0 << 0)
#define	STATUS_WRITE_BUSY		(0x1 << 0)
#define	STATUS_WRITE_ENABLE_CLEAR	(0x0 << 1)
#define	STATUS_WRITE_ENABLE_SET		(0x1 << 1)

#define	QSPI_BUFF_LEN		20

static unsigned int qspi_buff[QSPI_BUFF_LEN];

static qspi_frame_t	qspi_frame;
static qspi_data_t	qspi_data;

static spi_protocols_t spi_mode = extended;

static void qspi_init_frame(qspi_frame_t *frame)
{
	memset((char *)frame, 0, sizeof(*frame));
}

static void qspi_init_data_buff(qspi_data_t *data, unsigned int* buff)
{
	memset((char *)buff, 0, QSPI_BUFF_LEN * sizeof(unsigned int));
	data->buffer = buff;
}

static unsigned int qspi_flash_read_status_reg(unsigned char readCmd)
{
	qspi_frame_t *frame = &qspi_frame;
	qspi_data_t *data = &qspi_data;

	qspi_init_frame(frame);
	frame->instruction = readCmd;
	frame->tansfer_type = read;
	frame->protocol = spi_mode;

	qspi_init_data_buff(data, qspi_buff);
	data->size = 1;
	data->direction = DATA_DIR_READ;

	qspi_send_command(frame, data);

	return data->buffer[0];
}

static unsigned int qspi_flash_write_status_regs(unsigned char sr1Value, unsigned char sr2Value)
{
  qspi_frame_t *frame = &qspi_frame;
  qspi_data_t *data = &qspi_data;
  unsigned int waitLoop = 2000;
  unsigned char status = 0;

  qspi_init_frame(frame);
  frame->instruction = CMD_WINBOND_WRITE_STATUS_REGS;
  frame->tansfer_type = write;
  frame->protocol = spi_mode;

  qspi_init_data_buff(data, qspi_buff);
  data->size = 2;
  data->buffer[0] = (((unsigned int)sr2Value) << 8) | sr1Value; //buffer is an array of INTEGER !!
  data->direction = DATA_DIR_WRITE;

  qspi_send_command(frame, data);
  
  //Wait write done
  do 
  {
    status = qspi_flash_read_status_reg(CMD_WINBOND_READ_SR1);
  } while ((status & WINBOND_BIT_BUSY) && waitLoop--);

  return status & WINBOND_BIT_BUSY;
}


static int qspi_flash_enable_write(unsigned char writeEnableCmd)
{
	qspi_frame_t *frame = &qspi_frame;
	unsigned char status;

	qspi_init_frame(frame);
	frame->instruction = writeEnableCmd;
	frame->tansfer_type = read;
	frame->protocol = spi_mode;

	qspi_send_command(frame, 0);

	status = qspi_flash_read_status_reg(CMD_WINBOND_READ_SR1);
	if (status & STATUS_WRITE_ENABLE_SET)
		return 0;
	else
		return -1;
}

static int qspi_flash_set_quad_mode(unsigned int enable)
{
	unsigned char sr1Value;
	unsigned char sr2Value;
	unsigned char check_bits;
	int ret = 0;

 //Do the change only if needed.
 sr2Value = qspi_flash_read_status_reg(CMD_WINBOND_READ_SR2);
 if (((enable && (sr2Value & WINBOND_BIT_QE)) || (!enable && !(sr2Value & WINBOND_BIT_QE))))
 {
   dbg_very_loud("QSPI quad mode : nothing to do\n");
   return ret; //EXIT HERE
 }
   dbg_very_loud("QSPI quad mode : change required\n");
 
 ret = qspi_flash_enable_write(CMD_WRITE_ENABLE);
 if (ret)
    return -1;
    
 sr1Value = qspi_flash_read_status_reg(CMD_WINBOND_READ_SR1);
 
	if (enable)
		sr2Value|= WINBOND_BIT_QE;
	else
		sr2Value &= ~WINBOND_BIT_QE;
  
	if (qspi_flash_write_status_regs(sr1Value,sr2Value))
  {
    //MUST NEVER HAPPENS !! check wait loop counter value.
    asm("BKPT");
  }
  
	sr1Value = qspi_flash_read_status_reg(CMD_WINBOND_READ_SR1);
  sr2Value = qspi_flash_read_status_reg(CMD_WINBOND_READ_SR2);
  
	check_bits = (sr2Value & WINBOND_BIT_QE) ? 0x01 : 0x00;

	return (enable ^ check_bits);
}

static void qspi_flash_read_jedec_id(void)
{
	qspi_frame_t *frame = &qspi_frame;
	qspi_data_t *data = &qspi_data;

	qspi_init_frame(frame);
	frame->instruction = CMD_READ_ID;
	frame->tansfer_type = read;
	frame->protocol = spi_mode;

	qspi_init_data_buff(data, qspi_buff);
	data->size = 3;
	data->direction = DATA_DIR_READ;

	qspi_send_command(frame, data);

	dbg_info("QSPI Flash: Manufacturer and Device ID: %d %d %d\n",
				data->buffer[0] & 0xff,
				(data->buffer[0] >> 8) & 0xff,
				(data->buffer[0] >> 16) & 0xff);
}
//************************************
static int qspi_flash_read_image_smm(struct image_info *image, spi_protocols_t access_protocol, unsigned int read_instruction_code, unsigned int wait_cycles)
{
	qspi_frame_t *frame = &qspi_frame;
	qspi_data_t *data = &qspi_data;

	qspi_init_frame(frame);
	frame->instruction = read_instruction_code;
	frame->tansfer_type = read_memory;
	frame->has_address = 1;
	frame->address = image->offset;
	frame->continue_read = 0;
	frame->dummy_cycles = wait_cycles;
	frame->protocol = access_protocol;

	data->buffer = (unsigned int *)image->dest;
	data->size = image->length;
	data->direction = DATA_DIR_READ;

	return qspi_send_command(frame, data);
}
//*************************************
//Read image in Serial Memory Mode + DMA
static int qspi_flash_read_image_smm_dma (struct image_info *image, 
                                      spi_protocols_t access_protocol, 
                                      unsigned int read_instruction_code, 
                                      unsigned int wait_cycles)
{
  qspi_frame_t *frame = &qspi_frame;
  qspi_data_t *data = &qspi_data;

  qspi_init_frame(frame);
  frame->instruction = read_instruction_code;
  frame->tansfer_type = read_memory;
  frame->has_address = 1;
  frame->address = image->offset;
  frame->continue_read = 0;
  frame->dummy_cycles = wait_cycles;
  frame->protocol = access_protocol;

  data->buffer = (unsigned int *)image->dest;
  data->size = image->length;
  data->direction = DATA_DIR_READ;

  return qspi_send_command_dma(frame, data);
}
//Read image in RAW mode
static int qspi_flash_read_image_raw(struct image_info *image, spi_protocols_t access_protocol, unsigned int read_instruction_code, unsigned int wait_cycles)
{
  qspi_frame_t *frame = &qspi_frame;
  qspi_data_t *data = &qspi_data;

  qspi_init_frame(frame);
  frame->instruction = read_instruction_code;
  frame->tansfer_type = read_memory;
  frame->has_address = 1;
  frame->address = image->offset;
  frame->continue_read = 0;
  frame->dummy_cycles = wait_cycles;
  frame->protocol = access_protocol;

  data->buffer = (unsigned int *)image->dest;
  data->size = image->length;
  data->direction = DATA_DIR_READ;

  return qspi_send_command_raw(frame, data);
}
//************************ SPI command/Respond with DMA ****
//Read image in RAW mode
static int qspi_flash_read_image_raw_dma(struct image_info *image, spi_protocols_t access_protocol, unsigned int read_instruction_code, unsigned int wait_cycles)
{
  qspi_frame_t *frame = &qspi_frame;
  qspi_data_t *data = &qspi_data;

  qspi_init_frame(frame);
  frame->instruction = read_instruction_code;
  frame->tansfer_type = read_memory;
  frame->has_address = 1;
  frame->address = image->offset;
  frame->continue_read = 0;
  frame->dummy_cycles = wait_cycles;
  frame->cmd_len = 1 + 3 + frame->dummy_cycles;  //CMD(1 byte) Addr(3B) + any dummy bytes

  data->buffer = (unsigned int *)image->dest;
  data->size = image->length;

  return qspi_send_command_raw_dma(frame, data);
}



/*
 * Load the image through QSPI bus in single access mode.
 */
int qspi_flash_loadimage_in_single_mode_smm(struct image_info *image)
{
  int ret;

  at91_qspi_hw_init();

  qspi_init(CONFIG_SYS_QSPI_CLOCK, CONFIG_SYS_QSPI_MODE);

  qspi_flash_read_jedec_id();

  //HYP the flash is in single access mode by default.
  dbg_info("QSPI Flash: Access in Single/Extended SPI mode\n");

  dbg_info("QSPI Flash: Copy %d bytes from %d to %d\n",
      image->length, image->offset, image->dest);

  //ret = qspi_flash_read_image_smm(image, extended, CMD_EXTENDED_IO_SLOW_SINGLE_READ, 0);
  ret = qspi_flash_read_image_smm(image, extended, CMD_EXTENDED_IO_FAST_SINGLE_READ, 1);
  
  //Disable the QSPI unit.
 dbg_loud("QSPI Flash: DeInit\n");
 qspi_deinit();
 at91_qspi_hw_deinit();

  return ret ? -1 : 0;
}
/*
 * Load the image through QSPI bus in single access Serial Memory mode + DMA
 */
int qspi_flash_loadimage_in_single_mode_smm_dma(struct image_info *image)
{
  int ret;

  at91_qspi_hw_init();

  qspi_init(CONFIG_SYS_QSPI_CLOCK, CONFIG_SYS_QSPI_MODE);

  qspi_flash_read_jedec_id();

  //HYP the flash is in single access mode by default.
  dbg_info("QSPI Flash: SerialMemory/DMA Access in Single/Extended SPI mode\n");

  dbg_info("QSPI Flash: Copy %d bytes from %d to %d\n",
      image->length, image->offset, image->dest);
  //ret = qspi_flash_read_image_smm_dma(image, extended, CMD_EXTENDED_IO_SLOW_SINGLE_READ, 0);
  ret = qspi_flash_read_image_smm_dma(image, extended, CMD_EXTENDED_IO_FAST_SINGLE_READ, 1);
  //Disable the QSPI unit.
 dbg_loud("QSPI Flash: DeInit\n");
 qspi_deinit();
 at91_qspi_hw_deinit();

  return ret ? -1 : 0;
}
/*
 * Load the image through QSPI bus in DUAL access Serial Memory mode + DMA
 */
int qspi_flash_loadimage_in_dual_mode_smm_dma(struct image_info *image)
{
  int ret;

  at91_qspi_hw_init();

  qspi_init(CONFIG_SYS_QSPI_CLOCK, CONFIG_SYS_QSPI_MODE);

  qspi_flash_read_jedec_id();

  //HYP the flash is in single access mode by default.
  dbg_info("QSPI Flash: SerialMemory/DMA Access in Single/Extended SPI mode\n");

  dbg_info("QSPI Flash: Copy %d bytes from %d to %d\n",
      image->length, image->offset, image->dest);

  ret = qspi_flash_read_image_smm_dma(image, dual_output, CMD_DUAL_OUTPUT_FAST_READ, 1);
 
  //Disable the QSPI unit.
 dbg_loud("QSPI Flash: DeInit\n");
 qspi_deinit();
 at91_qspi_hw_deinit();

  return ret ? -1 : 0;
}
//Load in single RAW mode
int qspi_flash_loadimage_in_single_mode_raw(struct image_info *image)
{
  int ret;

  at91_qspi_hw_init();

  qspi_init(CONFIG_SYS_QSPI_CLOCK, CONFIG_SYS_QSPI_MODE);

  qspi_flash_read_jedec_id();
  
  //Reinit in raw for further operations
  qspi_init_raw(CONFIG_SYS_QSPI_CLOCK, CONFIG_SYS_QSPI_MODE);

  //HYP the flash is in single access mode by default.
  dbg_info("QSPI Flash: Access in Single/Extended SPI mode\n");

  dbg_info("QSPI Flash: Copy %d bytes from %d to %d\n",
      image->length, image->offset, image->dest);

  ret = qspi_flash_read_image_raw(image, extended, CMD_EXTENDED_IO_FAST_SINGLE_READ, 1);
 //Disable the QSPI unit.
 dbg_loud("QSPI Flash: DeInit\n");
 qspi_deinit();
 at91_qspi_hw_deinit();

  return ret ? -1 : 0;
}
//Load in single RAW mode + DMA.
int qspi_flash_loadimage_in_single_mode_raw_dma(struct image_info *image)
{
  int ret;

  at91_qspi_hw_init();

  qspi_init(CONFIG_SYS_QSPI_CLOCK, CONFIG_SYS_QSPI_MODE);

  qspi_flash_read_jedec_id();
  
  //Reinit in raw for further operations
  qspi_init_raw(CONFIG_SYS_QSPI_CLOCK, CONFIG_SYS_QSPI_MODE);

  //HYP the flash is in single access mode by default.
  dbg_info("QSPI Flash: Access in Single/Extended SPI mode with DMA\n");

  dbg_info("QSPI Flash: Copy %d bytes from %d to %d\n",
      image->length, image->offset, image->dest);

  ret = qspi_flash_read_image_raw_dma(image, extended, CMD_EXTENDED_IO_FAST_SINGLE_READ, 1);
  
 //Disable the QSPI unit.
 dbg_loud("QSPI Flash: DeInit\n");
 qspi_deinit();
 at91_qspi_hw_deinit();


  return ret ? -1 : 0;
}

int qspi_flash_loadimage_in_quad_mode_smm(struct image_info *image)
{
	int ret;
  const unsigned int dummyCycleQty = 8;
  
	at91_qspi_hw_init();

	qspi_init(AT91C_QSPI_CLK, SPI_MODE3);

	qspi_flash_read_jedec_id();

	ret = qspi_flash_set_quad_mode(0x01);
	if (ret)
	{
    dbg_error("QSPI Flash:  Can't set the quad mode !!\n");
    goto EXIT_POINT;
  }
  
 
	dbg_loud("QSPI Flash: Switch to Quad SPI mode\n");

	dbg_info("QSPI Flash: Copy %d bytes from %d to %d\n",
			image->length, image->offset, image->dest);

	ret = qspi_flash_read_image_smm(image, quad_output, CMD_QUAD_OUTPUT_FAST_READ, dummyCycleQty);
  
  #if 0
  ret = qspi_flash_set_quad_mode(0);
  if (ret)
    return -1;
  dbg_loud("QSPI Flash: Switch to Extended SPI mode\n");
#endif

EXIT_POINT:
 //Disable the QSPI unit.
 dbg_loud("QSPI Flash: DeInit\n");
 qspi_deinit();
 at91_qspi_hw_deinit();
 
	return ret ? -1 : 0;
}
//Load the flash in quad mode (DMA)
int qspi_flash_loadimage_in_quad_mode_smm_dma(struct image_info *image)
{
  int ret;
  const unsigned int dummyCycleQty = 8;

  at91_qspi_hw_init();

  qspi_init(AT91C_QSPI_CLK, SPI_MODE3);

  qspi_flash_read_jedec_id();
  

#if 1
  ret = qspi_flash_set_quad_mode(0x01);
  if (ret)
  {
    dbg_error("QSPI Flash:  Can't set the quad mode !!\n");
    goto EXIT_POINT;
  }
  dbg_loud("QSPI Flash: Switch to Quad SPI mode\n");
#endif
  

  dbg_info("QSPI Flash: Copy %d bytes from %d to %d\n",
      image->length, image->offset, image->dest);

  ret = qspi_flash_read_image_smm_dma(image, quad_output, CMD_QUAD_OUTPUT_FAST_READ, dummyCycleQty);
  
#if 0
  ret = qspi_flash_set_quad_mode(0);
  if (ret)
    return -1;
  dbg_loud("QSPI Flash: Switch to Extended SPI mode\n");
#endif
EXIT_POINT:
 //Disable the QSPI unit.
 dbg_loud("QSPI Flash: DeInit\n");
 qspi_deinit();
 at91_qspi_hw_deinit();

  return ret ? -1 : 0;
}
