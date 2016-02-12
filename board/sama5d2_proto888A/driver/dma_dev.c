/**
 * @file spi_flash_dma.c
 * @brief This file contains the SPI Flash access with DMA help.
 *
 *  Created on: 21 juin 2013
 *      Author: eric
 */
/* ----------------------------------------------------------------------------
 *         SAM Software Package License
 * ----------------------------------------------------------------------------
 * Copyright (c) 2013, HAGER Security
 * Copyright (c) 2012, ATMEL
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
 * DISCLAIMER: THIS SOFTWARE IS PROVIDED BY HAGER Security / ATMEL "AS IS" AND
 * ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT ARE
 * DISCLAIMED. IN NO EVENT SHALL ATMEL BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
 * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
 * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * ----------------------------------------------------------------------------
 */

#include <stdbool.h>
#include "common.h"
#include "hardware.h"
#include "dma_dev.h"
#include "arch/at91_spi.h"
#include "driver/xdmad.h"
#include "spi.h"
#include "gpio.h"
#include "usart.h"
#include "debug.h"

//! DMA_DEV_WITH_DMA_IRQ Will be set when IRQ support will be needed and set (IRQ stack in startup code)
#define NO_DMA_DEV_WITH_DMA_IRQ

//***********************************************************
//! Driver initialization flag.
static bool IsDriverInitDone = false;

#ifdef DMA_DEV_WITH_DMA_IRQ
//! Last Rx operation DMA status
static unsigned int sRxLastStatus = 0;

//! Last Tx operation DMA status
static unsigned int sTxLastStatus = 0;

/** reception done flag*/
//TODO : check the flag is only set when everything is done (even on multi-transfers conditions)
volatile bool recvDone0 = false;
#endif

#ifdef DMA_DEV_WITH_DMA_IRQ
//************* ISR DEBUG SUPPORT *****
/* The ISR debug pin through PortA pin 0*/
static const unsigned int isrPinId = 0;

/* The rx ISR debug pin through PortA pin 2*/
static const unsigned int rxIsrPinId = 2;

/* The tx ISR debug pin through PortA pin 2*/
static const unsigned int txIsrPinId = 4;
//***********************************************************
#if defined(DMA_DEV_WITH_DMA_IRQ)
#error NOT YET Implemented
/**
 * ISR for DMA interrupt
 * @note Not yet implemented
 */
static void ISR_DMA(void)
{
    //pio_set_value(isrPinId, 0);

    XDMAD_Handler();

    //pio_set_value(isrPinId, 1);
}
#endif /*DMA_DEV_WITH_DMA_IRQ*/
//***********************************************************
/**
 *  \brief Callback function for DMA0 receiving.
 */
static void _DmaRxCallback0(struct _xdmad_channel *channel, void *arg)
  {
#if defined (CONFIG_WITH_CACHE)
    //Force a cache re-load
    CP15_invalidate_dcache_for_dma((unsigned int)RES_BUFFER, ((unsigned int)RES_BUFFER) + RAM_BUFFER_SIZE);
#endif
    //TODO Check the channel Status 
    sRxLastStatus = 0;
    //Set the PIN according to status
    /*
    if (dmaStatus == XDMAD_OK)
      pio_set_value(rxIsrPinId, 0);
    else
      pio_set_value(rxIsrPinId, 1);
    */
    recvDone0 = true;
  }
//***********************************************************
static void _DmaTxCallback0(struct _xdmad_channel *channel, void *arg)
  {
#if defined (CONFIG_WITH_CACHE)
    //Force a cache re-load
    CP15_invalidate_dcache_for_dma((unsigned int)CMD_BUFFER, ((unsigned int)(CMD_BUFFER)) + RAM_BUFFER_SIZE);
#endif
    //TODO Check the channel Status 
    sTxLastStatus = 0;
    //Set the PIN according to status
    /*
    if (dmaStatus == DMAD_OK)
    pio_set_value(txIsrPinId, 0);
    else
    pio_set_value(txIsrPinId, 1);
    */
  }
//***********************************************************
#endif /*DMA_DEV_WITH_DMA_IRQ*/

unsigned int
DMA_DEV_OpenSPIIOStream(DMA_DEV_IOStream_t* const stream, const unsigned int spi_ctrlr_base_addr, const unsigned int spi_id)
{
  unsigned int dwCfg;
  unsigned char iController/*, iChannel*/;
  unsigned int dmadStatus = 0;
  /* Driver initialization if needed*/
  if (!IsDriverInitDone)
  {

#ifdef DMA_DEV_WITH_DMA_IRQ
      /* Debug Pin set*/
      pio_set_gpio_output(isrPinId,0);
      pio_set_gpio_output(txIsrPinId,0);
      pio_set_gpio_output(rxIsrPinId,0);
      xdmad_initialize(0);
#else
      //Polling mode
      xdmad_initialize();
#endif /*DMA_DEV_WITH_DMA_IRQ*/
      IsDriverInitDone = true;
  }

  /* Allocate TWO DMA channels for SPI */
  stream->TxChannel = xdmad_allocate_channel(XDMAD_PERIPH_MEMORY, spi_id);
  stream->RxChannel = xdmad_allocate_channel(spi_id, XDMAD_PERIPH_MEMORY);
  if (stream->TxChannel == NULL || stream->RxChannel == NULL)
    {
      dbg_log(DEBUG_ERROR,"DMA channel allocation error\n");
      return 1;
    }
/* Prepare DMA channels */
  xdmad_prepare_channel(stream->RxChannel);
  xdmad_prepare_channel(stream->TxChannel);
  
#ifdef DMA_DEV_WITH_DMA_IRQ
  /* Set RX callback */
  dmadStatus = xdmad_set_callback(stream->RxChannel, _DmaRxCallback0, 0);
  //TX Callback
  dmadStatus = xdmad_set_callback(stream->TxChannel, (DmadTransferCallback)_DmaTxCallback0, 0);
#endif
  stream->spi_ctrlr_base_addr = spi_ctrlr_base_addr;
  return 0;
}
//*******************************************************************
void
DMA_DEV_CloseSPIIOStream(DMA_DEV_IOStream_t* const stream)
{
  unsigned int dmaResult = 0;
  dmaResult = xdmad_stop_transfer(stream->RxChannel);
  dmaResult = xdmad_free_channel(stream->RxChannel);

  dmaResult = xdmad_stop_transfer(stream->TxChannel);
  dmaResult = xdmad_free_channel(stream->TxChannel);
}
//*******************************************************************
unsigned int
DMA_DEV_SPICommandResponse(const DMA_DEV_IOStream_t* const stream, void* toSend, const unsigned int sendLength, void* toRecv,
    const unsigned int recvLength)
{
  /** 
   * Dual transfer descriptors : an array with
   * - [0] : Command phase
   * - [1] : Response phase
   * On for each way (Send / Receive).
   * Use the View2 to set the special configuration according each phase.
   */
  struct _xdmad_desc_view2 tdSendStream[2];
  struct _xdmad_desc_view2 tdReceiveStream[2];

  unsigned char junkByte = 0; //Used to handle the unused send/received byte in inactive phase of each transfer.
  
  unsigned int dmaResult = 0;

#ifdef DMA_DEV_WITH_DMA_IRQ
  recvDone0 = 0;
#endif

  dbg_log(DEBUG_INFO,"%s() : send %d B, receive %d B\n", __FUNCTION__, sendLength, recvLength);
  
  //Activate IO sync on SPI
  at91_spi_oisync(1);

  //Force stop transfer on the channel if in use : Not always Ok once actually ended.
  if (!xdmad_is_transfer_done(stream->TxChannel))
    xdmad_stop_transfer(stream->TxChannel);

  if (!xdmad_is_transfer_done(stream->RxChannel))
    xdmad_stop_transfer(stream->RxChannel);
  CONFIG:
  dbg_log(DEBUG_LOUD,"============= SEND Command ==============\n");
  //First phase : send the command
  tdSendStream[0].next_desc = &tdSendStream[1];
  tdSendStream[0].ublock_size = XDMA_UBC_NVIEW_NDV3 | XDMA_UBC_NDE_FETCH_EN | XDMA_UBC_NSEN_UPDATED |XDMA_UBC_NDEN_UPDATED | sendLength;
  tdSendStream[0].src_addr = toSend;
  tdSendStream[0].dest_addr = (void*)(stream->spi_ctrlr_base_addr + SPI_TDR);
  tdSendStream[0].cfg.uint32_value = XDMAC_CC_PERID(xdmad_get_dest_txitf(stream->TxChannel))
    | XDMAC_CC_TYPE_PER_TRAN
    | XDMAC_CC_DSYNC_MEM2PER
    | XDMAC_CC_MEMSET_NORMAL_MODE
    | XDMAC_CC_CSIZE_CHK_1
    | XDMAC_CC_DWIDTH_BYTE
    | XDMAC_CC_DIF_AHB_IF1
    | XDMAC_CC_SIF_AHB_IF0
    | XDMAC_CC_DAM_FIXED_AM
    | XDMAC_CC_SAM_INCREMENTED_AM;
    
  //First phase : just dump the received byte
  tdReceiveStream[0].next_desc = &tdReceiveStream[1];
  tdReceiveStream[0].ublock_size = XDMA_UBC_NVIEW_NDV3 | XDMA_UBC_NDE_FETCH_EN | XDMA_UBC_NSEN_UPDATED |XDMA_UBC_NDEN_UPDATED | sendLength;
  tdReceiveStream[0].src_addr =(void*)(stream->spi_ctrlr_base_addr + SPI_RDR);
  tdReceiveStream[0].dest_addr = &junkByte;
  tdReceiveStream[0].cfg.uint32_value = XDMAC_CC_PERID(xdmad_get_src_rxitf(stream->RxChannel))
    | XDMAC_CC_TYPE_PER_TRAN
    | XDMAC_CC_DSYNC_PER2MEM
    | XDMAC_CC_MEMSET_NORMAL_MODE
    | XDMAC_CC_CSIZE_CHK_1
    | XDMAC_CC_DWIDTH_BYTE
    | XDMAC_CC_DIF_AHB_IF0
    | XDMAC_CC_SIF_AHB_IF1
    | XDMAC_CC_SAM_FIXED_AM
    | XDMAC_CC_DAM_FIXED_AM;
    
  dbg_log(DEBUG_LOUD,"============= RECEIVE Response ==============\n");
  //Second phase : just drive the bus to get the response.
  tdSendStream[1].next_desc = 0;
  tdSendStream[1].src_addr= &junkByte;
  tdSendStream[1].dest_addr = (void*)(stream->spi_ctrlr_base_addr + SPI_TDR);
  tdSendStream[1].ublock_size = XDMA_UBC_NVIEW_NDV3 | XDMA_UBC_NDE_FETCH_EN | XDMA_UBC_NSEN_UPDATED |XDMA_UBC_NDEN_UPDATED |recvLength;
  tdSendStream[1].cfg.uint32_value = XDMAC_CC_PERID(xdmad_get_dest_txitf(stream->TxChannel))
    | XDMAC_CC_TYPE_PER_TRAN
    | XDMAC_CC_DSYNC_MEM2PER
    | XDMAC_CC_MEMSET_NORMAL_MODE
    | XDMAC_CC_CSIZE_CHK_1
    | XDMAC_CC_DWIDTH_BYTE
    | XDMAC_CC_DIF_AHB_IF1
    | XDMAC_CC_SIF_AHB_IF0
    | XDMAC_CC_DAM_FIXED_AM
    | XDMAC_CC_SAM_FIXED_AM;
  //Second Phase : Actually receive the data
  tdReceiveStream[1].next_desc = 0;
  tdReceiveStream[1].ublock_size = XDMA_UBC_NVIEW_NDV3 | XDMA_UBC_NDE_FETCH_EN | XDMA_UBC_NSEN_UPDATED |XDMA_UBC_NDEN_UPDATED | recvLength;
  tdReceiveStream[1].src_addr = (void*)(stream->spi_ctrlr_base_addr + SPI_RDR);
  tdReceiveStream[1].dest_addr = toRecv;
  tdReceiveStream[1].cfg.uint32_value = XDMAC_CC_PERID(xdmad_get_src_rxitf(stream->RxChannel))
    | XDMAC_CC_TYPE_PER_TRAN
    | XDMAC_CC_DSYNC_PER2MEM
    | XDMAC_CC_MEMSET_NORMAL_MODE
    | XDMAC_CC_CSIZE_CHK_1
    | XDMAC_CC_DWIDTH_BYTE
    | XDMAC_CC_DIF_AHB_IF0
    | XDMAC_CC_SIF_AHB_IF1
    | XDMAC_CC_SAM_FIXED_AM
    | XDMAC_CC_DAM_INCREMENTED_AM;
    
  //The common configuration
  struct _xdmad_cfg transfer_config = {
    .ublock_size = 0,
    .block_size = 0, //Only one block
    .data_stride = 0, //no data stride
    .src_ublock_stride = 0,
    .dest_ublock_stride = 0,
    .src_addr = 0,
    .dest_addr = 0
    
  };
  // == Prepare the transfers
  
  //First check cache consistency
#if defined (CONFIG_WITH_CACHE)
  cp15_flush_dcache_for_dma((unsigned int)tdReceiveStream, (unsigned int)tdReceiveStream + sizeof(tdReceiveStream));  
  cp15_flush_dcache_for_dma((unsigned int)tSendStream, (unsigned int)tSendStream + sizeof(tSendStream));

  cp15_flush_dcache_for_dma((unsigned int)toSend, ((unsigned int)(toSend + sendLength)));
  cp15_flush_dcache_for_dma((unsigned int)toRecv, ((unsigned int)(toRecv + recvLength)));
#endif

  //TX channel
  dmaResult = xdmad_configure_transfer(stream->TxChannel, &transfer_config, XDMAC_CNDC_NDVIEW_NDV2 |
    XDMAC_CNDC_NDE_DSCR_FETCH_EN |
    XDMAC_CNDC_NDSUP_SRC_PARAMS_UPDATED |
    XDMAC_CNDC_NDDUP_DST_PARAMS_UPDATED, tdSendStream);

  dbg_log(DEBUG_VERY_LOUD, "DBG:TX : xdmad_configure_transfer()=>%d\n",dmaResult);
  if (dmaResult != XDMAD_OK)
    {
      goto EXIT_POINT;
    }
  //RX channel
  dmaResult = xdmad_configure_transfer(stream->RxChannel, &transfer_config, XDMAC_CNDC_NDVIEW_NDV2 |
    XDMAC_CNDC_NDE_DSCR_FETCH_EN |
    XDMAC_CNDC_NDSUP_SRC_PARAMS_UPDATED |
    XDMAC_CNDC_NDDUP_DST_PARAMS_UPDATED, tdReceiveStream);
  
  dbg_log(DEBUG_VERY_LOUD, "DBG : RX xdmad_configure_transfer()=>%d\n", dmaResult);
  if (dmaResult  != XDMAD_OK)
    {
      goto EXIT_POINT;
    }

  //Start !!
  START:
  dmaResult = xdmad_start_transfer(stream->RxChannel);
  dbg_log(DEBUG_VERY_LOUD, "DBG : RX xdmad_start_transfer()=>%d\n", dmaResult );
  if (dmaResult  != XDMAD_OK)
    goto EXIT_POINT;

  dmaResult = xdmad_start_transfer(stream->TxChannel);
  dbg_log(DEBUG_VERY_LOUD, "DBG, TX : xdmad_start_transfer()=>%d\n", dmaResult );
  if (dmaResult != XDMAD_OK)
    {
      xdmad_stop_transfer(stream->RxChannel);
      goto EXIT_POINT;
    }
  WAIT:
  //Wait the end of the transfer : operation is synchronous. Use RX  && TX (need at91_spi_oisync activated !!)
  //Only one channel get a state change !! ??
  //while (!(xdmad_is_transfer_done(stream->RxChannel) || xdmad_is_transfer_done(stream->TxChannel)))
  //while (!xdmad_is_transfer_done(stream->RxChannel))
  while (!xdmad_is_transfer_done(stream->TxChannel))
        xdmad_poll();
  DONE:
  xdmad_stop_transfer(stream->RxChannel);
  xdmad_stop_transfer(stream->TxChannel);

//Could be optionnal as already done before start above.
#if defined (CONFIG_WITH_CACHE)
//Force a fresh copy of the transfered AREA into the cache. note : not actually needed as a flush is done before.
  cp15_invalidate_dcache_for_dma((unsigned int)&tdReceiveStream, (unsigned int)&tdReceiveStream + sizeof(tdReceiveStream));
  cp15_invalidate_dcache_for_dma((unsigned int)&tdSendStream, (unsigned int)&tdSendStream + sizeof(tdSendStream));
  cp15_invalidate_dcache_for_dma((unsigned int)toSend, ((unsigned int)(toSend + sendLength)));
  cp15_invalidate_dcache_for_dma((unsigned int)toRecv, ((unsigned int)(toRecv + recvLength)));
#endif

  EXIT_POINT: at91_spi_oisync(0);
  return dmaResult;
}
//************************************************************
bool
DMA_DEV_IsTransferInProgress(const DMA_DEV_IOStream_t* const stream)
{
  return (stream->RxChannel != NULL && !xdmad_is_transfer_done(stream->RxChannel) ) ||
         (stream->TxChannel != NULL && !xdmad_is_transfer_done(stream->TxChannel));
}
//*************************************************************
unsigned int
DMA_DEV_GetDmaTransactionMaxSize(void)
{
  //The SAMA5D2x DMA DMA transaction (one micro-block and one block) size is of 2^24 bytes
  return 0xFFFFFF;
}
//**************************************************************