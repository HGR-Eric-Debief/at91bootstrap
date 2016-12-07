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
#ifndef	__QSPI_H__
#define	__QSPI_H__

#include "spi.h"

typedef	enum {
	read = 0,
	read_memory,
	write,
	write_memory,
} tansfer_type_t;

typedef enum {
	extended = 0, //!< Single bit I/O
  dual_output, //!< Single bit input, dual output
  dual_io, //!< Single cmd, dual I/O
	dual, //!< Dual input/output
	quad_output, //!< Single input, Quad Output
  quad_io, //!< Single cmd, quad I/O
  quad, //!< Quad input/output
} spi_protocols_t;

typedef struct qspi_frame {
	unsigned int instruction;
  unsigned int cmd_len; //!< Used in raw access mode. Len = instruction length + address lenght + any dummy byte
	unsigned int option;
	unsigned int option_len;
	unsigned int has_address;
	unsigned int address;
	unsigned int address_len;
	unsigned int continue_read;
	unsigned int dummy_cycles;
	spi_protocols_t protocol;
	tansfer_type_t tansfer_type;
} qspi_frame_t;

#define	DATA_DIR_READ		0x11
#define	DATA_DIR_WRITE		0x22

typedef struct qspi_data {
	unsigned int* buffer;
	unsigned int size;
	unsigned int direction;
} qspi_data_t;

//Serial Memory Mode
int qspi_init(unsigned int clock, unsigned int mode);
int qspi_send_command(qspi_frame_t *frame, qspi_data_t *data);
int qspi_send_command_dma(qspi_frame_t *frame, qspi_data_t *data);

//SPI Mode
int qspi_init_raw(unsigned int clock, unsigned int mode);
int qspi_init_dual_raw(unsigned int clock, unsigned int mode);
int qspi_send_command_raw(qspi_frame_t *frame, qspi_data_t *data);
int qspi_send_command_raw_dma(qspi_frame_t *frame, qspi_data_t *data);

/**
 * This function drives the the SPI controller with the Wait Data Read Before Transfer mode.
 * If active, this will synchronize the Output with the Input, see SPI controller datasheet.
 * @note this is mandatory for DMA multi-buffers transfers, otherwise data loss can occur at high speeds (above 30MHz).
 * @param isActive [IN] boolean
 *  - 0 : disable the  Wait Data Read Before Transfer mode.
 *  - other : enable the Wait Data Read Before Transfer mode.
 */
extern int at91_qspi_oisync (int isActive);



#endif
