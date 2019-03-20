/* ----------------------------------------------------------------------------
 *         SAM Software Package License
 * ----------------------------------------------------------------------------
 * Copyright (c) 2013, HAGER Security
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
 * DISCLAIMER: THIS SOFTWARE IS PROVIDED BY HAGER Security "AS IS" AND ANY EXPRESS OR
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

#ifndef SPI_DEV_DMA_H_
#define SPI_DEV_DMA_H_

#include <stdbool.h>


#if defined (SAMA5D2)
struct _xdmad_channel;
#endif

//*********************** Types **********************
/**
 * The DMA stream descriptor Rx and Tx channels for SPI operation.
 * The inner structure depends of the target
 */
typedef struct _DMA_DEV_IOStream
{
  unsigned int spi_ctrlr_base_addr; //!< BAse addresse of the SPI Controller.
#if defined(SAMAD3X)
  unsigned int RxChannel; //!< RX channel of the stream.
  unsigned int TxChannel; //!< TX channel of the stream.
#elif defined (SAMA5D2)
  struct _xdmad_channel* RxChannel; //!< RX channel of the stream.
  struct _xdmad_channel* TxChannel; //!< TX channel of the stream.
#else
#error unknown target
#endif
} DMA_DEV_IOStream_t;

//! @return the max size of a DMA transaction
unsigned int
DMA_DEV_GetDmaTransactionMaxSize(void);

/**
 * Initialize the DMA for the given SPI ctrlr : Open the IOStream.
 * @param stream [OUT] : The DMA/Device stream to open.
 * @param spi_ctrlr_base_addr [IN]  : the base address of the used SPI Ctrlr.
 * @param spi_id [IN] : Device id for the SPI controller.
 * @return
 *  - 0 : OK
 *  - 1 : Stream allocation error.
 */
unsigned int
DMA_DEV_OpenSPIIOStream(DMA_DEV_IOStream_t* const stream,
    const unsigned int spi_ctrlr_base_addr, const unsigned int spi_id);

/**
 * This function will FORCE close the given SPI stream
 * @param stream [IN/OUT] : The stream going to be closed.
 */
void
DMA_DEV_CloseSPIIOStream(DMA_DEV_IOStream_t* const stream);

#if defined CONFIG_SPI
/**
 * This function will handle the Command Response transfer in a SPI. The bus is activated for sendLength + sendRecv bytes.
 * It will use a DMA multi-transfer feature if needed.
 * @param stream [IN] The DMA stream used to do the transaction
 * @param toSend pointer to the first byte to send
 * @param sendLength length (in bytes) of the toSend
 * @param toRecv Pointer to the location where to store the received bytes.
 * @param recvLength Length in bytes of the received bytes
 * @return Error code translated from eDmadStatus in dmad.h
 */
#endif /* CONFIG_SPI */

unsigned int
DMA_DEV_SPICommandResponse(const DMA_DEV_IOStream_t* const stream, void* toSend,
    const unsigned int sendLength, void* toRecv, const unsigned int recvLength);
/**
 * This function check if a transfer is in progress.
 * @param stream [IN] The stream on which a transfer status will be check (I/O transfer).
 * @return  the transfer status
 *  - true : a transfer is in progress
 *  - false : No transfer in progress.
 */
bool
DMA_DEV_IsTransferInProgress(const DMA_DEV_IOStream_t* const stream);
#if defined CONFIG_QSPI
/**
 * This function will handle the Command Response transfer in a QSPI. The bus is activated for sendLength + sendRecv bytes.
 * It will use a DMA multi-transfer feature if needed.
 * @param stream [IN] The DMA stream used to do the transaction
 * @param toSend pointer to the first byte to send
 * @param sendLength length (in bytes) of the toSend
 * @param toRecv Pointer to the location where to store the received bytes.
 * @param recvLength Length in bytes of the received bytes
 * @return Error code translated from eDmadStatus in dmad.h
 */
unsigned int 
DMA_DEV_QSPICommandResponse( const DMA_DEV_IOStream_t* const stream, void* toSend, const unsigned int sendLength, void* toRecv,
    const unsigned int recvLength);
#endif /*CONFIG_QSPI*/

unsigned int
/**
 * This function will do a memory copy with the DMA. Return when done.
 * @param dest [IN] The address of the destination area
 * @param source [IN] The address of the source area
 * @param len [IN] The length (in bytes) of the source area.
 */
DMA_MEM_copy(void* dest, void* source, unsigned int len);
#endif /*SPI_DEV_DMA_H_*/