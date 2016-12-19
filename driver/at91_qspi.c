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
#include "hardware.h"
#include "board.h"
#include "qspi.h"
#include "pmc.h"
#include "div.h"
#include "string.h"
#include "arch/at91_qspi.h"
#include "dma_dev.h"
#include "debug.h"

#ifndef CONFIG_SYS_BASE_QSPI
#define CONFIG_SYS_BASE_QSPI		AT91C_BASE_QSPI0
#endif

#ifndef CONFIG_SYS_BASE_QSPI_MEM
#define	CONFIG_SYS_BASE_QSPI_MEM	AT91C_BASE_QSPI0_MEM
#endif

static inline unsigned int qspi_get_base(void)
{
	return CONFIG_SYS_BASE_QSPI;
}

static inline unsigned char *qspi_memory_base(void)
{
	return (unsigned char *)CONFIG_SYS_BASE_QSPI_MEM;
}

static inline unsigned int qspi_readl(unsigned int reg)
{
	return readl(qspi_get_base() + reg);
}

static inline void qspi_writel(unsigned int reg, unsigned int value)
{
	writel(value, qspi_get_base() + reg);
}

static int qspi_clock_init(unsigned int clock, unsigned int mode)
{
	unsigned int scbr;
	unsigned int reg;
	
	if (clock == 0)
		scbr = 1;
	else
		scbr = div(at91_get_ahb_clock(), clock);

	reg = QSPI_SCR_SCBR_(scbr);
	if (mode == SPI_MODE1)
		reg |= QSPI_SCR_CPHA;
	else if (mode == SPI_MODE2)
		reg |= QSPI_SCR_CPOL;
	else if (mode == SPI_MODE3)
		reg |= (QSPI_SCR_CPOL | QSPI_SCR_CPHA);

	qspi_writel(QSPI_SCR, reg);

	return 0;
}

int qspi_init(unsigned int clock, unsigned int mode)
{
	unsigned int config;

	qspi_writel(QSPI_CR, QSPI_CR_QSPIDIS);
	qspi_writel(QSPI_CR, QSPI_CR_SWRST);

	qspi_clock_init(clock, mode);

	config = QSPI_MR_SMM_MEMORY | QSPI_MR_DLYCS; 
	qspi_writel(QSPI_MR, config);

	qspi_writel(QSPI_CR, QSPI_CR_QSPIEN);

	return 0;
}

int qspi_deinit(void)
{
  qspi_writel(QSPI_CR, QSPI_CR_QSPIDIS);
  return 0;
}

int qspi_send_command(qspi_frame_t *frame, qspi_data_t *data)
{
	unsigned int instruction = 0;
	unsigned int config = 0;
	unsigned char *membuff;

	if (frame->protocol == extended)
    config |= QSPI_IFR_WIDTH_SINGLE_BIT_SPI;
  else  if (frame->protocol == dual_output)
    config |= QSPI_IFR_WIDTH_DUAL_OUTPUT;
  else  if (frame->protocol == dual)
    config |= QSPI_IFR_WIDTH_DUAL_CMD;
  else  if (frame->protocol == quad_output)
    config |= QSPI_IFR_WIDTH_QUAD_OUTPUT;
  else  if (frame->protocol == quad)
    config |= QSPI_IFR_WIDTH_QUAD_CMD;
  else
  {
    dbg_error(" qspi_send_command() : %d : unknown protocol !!\n",frame->protocol);
    asm("BKPT"); //Not a known protocol
  }

	if (frame->instruction) {
		config |= QSPI_IFR_INSTEN;
		instruction |= QSPI_ICR_INST_(frame->instruction);
	}

	if (frame->has_address) {
		config |= QSPI_IFR_ADDREN;
		if ((!data))
			qspi_writel(QSPI_IAR, frame->address);
	}

	if (data)
		config |= QSPI_IFR_DATAEN;

	if (frame->option) {
		config |= QSPI_IFR_OPTEN;
		instruction |= QSPI_ICR_OPT_(frame->option);
	}

	if (frame->option_len == 1)
		config |= QSPI_IFR_OPTL_1BIT;
	else if (frame->option_len == 2)
		config |= QSPI_IFR_OPTL_2BIT;
	else if (frame->option_len == 4)
		config |= QSPI_IFR_OPTL_4BIT;
	else if (frame->option_len == 8)
		config |= QSPI_IFR_OPTL_8BIT;

	if (frame->address_len)
		config |= QSPI_IFR_ADDRL_32_BIT;

	if (frame->tansfer_type == read)
		config |= QSPI_IFR_TFRTYPE_READ;
	else if (frame->tansfer_type == read_memory)
		config |= QSPI_IFR_TFRTYPE_READ_MEMORY;
	else if (frame->tansfer_type == write)
		config |= QSPI_IFR_TFRTYPE_WRITE;
	else if (frame->tansfer_type == write_memory)
		config |= QSPI_IFR_TFRTYPE_WRITE_MEMORY;

	if (frame->continue_read)
		config |= QSPI_IFR_CRM;

	config |= QSPI_IFR_NBDUM_(frame->dummy_cycles);

	qspi_writel(QSPI_ICR, instruction);
	qspi_writel(QSPI_IFR, config);

	qspi_readl(QSPI_IFR);	/* To synchronize system bus access */
  BEFORE_DATA_OP:
	if (data) {
		membuff = qspi_memory_base() + frame->address;
		if (data->direction == DATA_DIR_READ)
      memcpy((unsigned char *)data->buffer, membuff, data->size);
		else if (data->direction == DATA_DIR_WRITE)
      memcpy(membuff, (unsigned char *)data->buffer, data->size);
	}

	qspi_writel(QSPI_CR, QSPI_CR_LASTXFER);

	while (!(qspi_readl(QSPI_SR) & QSPI_SR_INSTRE))
		;

	return 0;
}
//************************************************************
int qspi_send_command_dma(qspi_frame_t *frame, qspi_data_t *data)
{
  unsigned int instruction = 0;
  unsigned int config = 0;
  unsigned char *membuff;
#if 0
  static unsigned char testDest[1024];
  static unsigned char testSource[1024];
  
  //For test
  memset(testSource,0xED, 1024);
  DMA_MEM_copy(testDest, testSource, 1024);
#endif

  if (frame->protocol == extended)
    config |= QSPI_IFR_WIDTH_SINGLE_BIT_SPI;
  else  if (frame->protocol == dual_output)
    config |= QSPI_IFR_WIDTH_DUAL_OUTPUT;
  else  if (frame->protocol == dual)
    config |= QSPI_IFR_WIDTH_DUAL_CMD;
  else  if (frame->protocol == quad_output)
    config |= QSPI_IFR_WIDTH_QUAD_OUTPUT;
  else  if (frame->protocol == quad)
    config |= QSPI_IFR_WIDTH_QUAD_CMD;
  else
  {
    dbg_error(" qspi_send_command() : %d : unknown protocol !!\n",frame->protocol);
    asm("BKPT"); //Not a known protocol
  }

  if (frame->instruction) {
    config |= QSPI_IFR_INSTEN;
    instruction |= QSPI_ICR_INST_(frame->instruction);
  }

  if (frame->has_address) {
    config |= QSPI_IFR_ADDREN;
    if ((!data))
      qspi_writel(QSPI_IAR, frame->address);
  }

  if (data)
    config |= QSPI_IFR_DATAEN;

  if (frame->option) {
    config |= QSPI_IFR_OPTEN;
    instruction |= QSPI_ICR_OPT_(frame->option);
  }

  if (frame->option_len == 1)
    config |= QSPI_IFR_OPTL_1BIT;
  else if (frame->option_len == 2)
    config |= QSPI_IFR_OPTL_2BIT;
  else if (frame->option_len == 4)
    config |= QSPI_IFR_OPTL_4BIT;
  else if (frame->option_len == 8)
    config |= QSPI_IFR_OPTL_8BIT;

  if (frame->address_len)
    config |= QSPI_IFR_ADDRL_32_BIT;

  if (frame->tansfer_type == read)
    config |= QSPI_IFR_TFRTYPE_READ;
  else if (frame->tansfer_type == read_memory)
    config |= QSPI_IFR_TFRTYPE_READ_MEMORY;
  else if (frame->tansfer_type == write)
    config |= QSPI_IFR_TFRTYPE_WRITE;
  else if (frame->tansfer_type == write_memory)
    config |= QSPI_IFR_TFRTYPE_WRITE_MEMORY;

  if (frame->continue_read)
    config |= QSPI_IFR_CRM;

  config |= QSPI_IFR_NBDUM_(frame->dummy_cycles);

  qspi_writel(QSPI_ICR, instruction);
  qspi_writel(QSPI_IFR, config);

  qspi_readl(QSPI_IFR); /* To synchronize system bus access */
  BEFORE_DATA_OP:
  if (data) {
    membuff = qspi_memory_base() + frame->address;
    if (data->direction == DATA_DIR_READ)
      DMA_MEM_copy(data->buffer, membuff, data->size);
    else if (data->direction == DATA_DIR_WRITE)
      DMA_MEM_copy(membuff, data->buffer, data->size);
  }

  qspi_writel(QSPI_CR, QSPI_CR_LASTXFER);

  while (!(qspi_readl(QSPI_SR) & QSPI_SR_INSTRE))
    ;

  return 0;
}
//************************************************************
//SPI Mode named RAW here
int qspi_init_raw(unsigned int clock, unsigned int mode)
{
  unsigned int config;

  qspi_writel(QSPI_CR, QSPI_CR_QSPIDIS);
  qspi_writel(QSPI_CR, QSPI_CR_SWRST);

  qspi_clock_init(clock, mode);

  config = QSPI_MR_SMM_SPI | QSPI_MR_CSMODE_LASTXFER | QSPI_MR_WDRBT_ENABLED | QSPI_MR_NBBITS_8_BIT ; 
  qspi_writel(QSPI_MR, config);

  qspi_writel(QSPI_CR, QSPI_CR_QSPIEN);

  return 0;
}
//************************************************************
int qspi_init_dual_raw(unsigned int clock, unsigned int mode)
{
  unsigned int config;

  qspi_writel(QSPI_CR, QSPI_CR_QSPIDIS);
  qspi_writel(QSPI_CR, QSPI_CR_SWRST);

  qspi_clock_init(clock, mode);

  config = QSPI_MR_SMM_SPI | QSPI_MR_CSMODE_LASTXFER | QSPI_MR_WDRBT_ENABLED | QSPI_MR_NBBITS_8_BIT ; 
  qspi_writel(QSPI_MR, config);

  qspi_writel(QSPI_CR, QSPI_CR_QSPIEN);

  return 0;
}
//************************************************************
int qspi_send_command_raw(qspi_frame_t *frame, qspi_data_t *data)
{
  unsigned int config = 0;
  volatile unsigned char dummy = 0;
  unsigned char addr[3] = {(frame->address)>>16,(frame->address)>>8, (frame->address) };
  unsigned char* destBuffer = (unsigned char*)data->buffer;
  
  //Simply run the QSPI bus in a send command/read response way skipping dummy cycles.
  //Command : instruction + address if any
  qspi_writel(QSPI_TDR, frame->instruction);
  while (!(qspi_readl(QSPI_SR) & QSPI_SR_TDRE));//Wait send done
  while (!(qspi_readl(QSPI_SR) & QSPI_SR_RDRF));
  dummy = qspi_readl(QSPI_RDR);
  //24 bits address for the moment
  qspi_writel(QSPI_TDR, addr[0]);
  while (!(qspi_readl(QSPI_SR) & QSPI_SR_TDRE));//Wait send done
  while (!(qspi_readl(QSPI_SR) & QSPI_SR_RDRF));
  dummy = qspi_readl(QSPI_RDR);
  
  qspi_writel(QSPI_TDR, addr[1]);
  while (!(qspi_readl(QSPI_SR) & QSPI_SR_TDRE));//Wait send done
  while (!(qspi_readl(QSPI_SR) & QSPI_SR_RDRF));
  dummy = qspi_readl(QSPI_RDR);
  
  qspi_writel(QSPI_TDR, addr[2]);
  while (!(qspi_readl(QSPI_SR) & QSPI_SR_TDRE));//Wait send done
  while (!(qspi_readl(QSPI_SR) & QSPI_SR_RDRF));
  dummy = qspi_readl(QSPI_RDR);

  //option & Dummy if needed.
  if (frame->option)
    while (frame->option_len--) 
    {
      qspi_writel(QSPI_TDR, 0xFF);
      while (!(qspi_readl(QSPI_SR) & QSPI_SR_TDRE));//Wait send done
      while (!(qspi_readl(QSPI_SR) & QSPI_SR_RDRF));
      dummy = qspi_readl(QSPI_RDR);
    }
    while (frame->dummy_cycles--) 
    {
      qspi_writel(QSPI_TDR, 0xFF);
      while (!(qspi_readl(QSPI_SR) & QSPI_SR_TDRE));//Wait send done
      while (!(qspi_readl(QSPI_SR) & QSPI_SR_RDRF));
      dummy = qspi_readl(QSPI_RDR);
    }
  //Data
    while (data->size--) 
    {
      qspi_writel(QSPI_TDR, *destBuffer);
      while (!(qspi_readl(QSPI_SR) & QSPI_SR_TDRE));//Wait send done
      while (!(qspi_readl(QSPI_SR) & QSPI_SR_RDRF));
      *destBuffer++ = qspi_readl(QSPI_RDR);
    }

  while (!(qspi_readl(QSPI_SR) & QSPI_SR_TXEMPTY));//Wait send fully done
  qspi_writel(QSPI_CR, QSPI_CR_LASTXFER);

  return 0;
}
// DMA
int qspi_send_command_raw_dma(qspi_frame_t *frame, qspi_data_t *data)
{
  unsigned int config = 0;
  volatile unsigned char dummy = 0;
  unsigned char addr[3] = {frame->address>>16, frame->address>>8, frame->address };
  unsigned char cmd[5];
  int ret = 0;
  //Fill the command
  cmd[0] = frame->instruction;
  cmd[1] = frame->address >> 16;
  cmd[2] = frame->address >> 8;
  cmd[3] = frame->address;
  cmd[4] = 0x55;//Dummy byte : bin pattern
  
  DMA_DEV_IOStream_t readStream;
  
  DMA_DEV_OpenSPIIOStream(&readStream, CONFIG_SYS_BASE_QSPI,  CONFIG_SYS_ID_QSPI);
  
  ret = DMA_DEV_QSPICommandResponse(&readStream,cmd, frame->cmd_len, data->buffer, data->size);
  qspi_writel(QSPI_CR, QSPI_CR_LASTXFER);
  
  DMA_DEV_CloseSPIIOStream(&readStream);
  return ret;

}
//***********************************************************************
int at91_qspi_oisync (int isActive)
{
  unsigned int reg;
  reg = qspi_readl(QSPI_MR);
  if (isActive)
    reg |= QSPI_MR_WDRBT;
  else
    reg &= ~QSPI_MR_WDRBT;
  qspi_writel(QSPI_MR, reg);
  return 0;
}
//************************************************************************
