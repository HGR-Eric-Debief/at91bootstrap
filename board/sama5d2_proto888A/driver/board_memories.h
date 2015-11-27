/* ----------------------------------------------------------------------------
 *         SAM Software Package License
 * ----------------------------------------------------------------------------
 * Copyright (c) 2013, Atmel Corporation
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
 * ----------------------------------------------------------------------------
 */

/**
 * \file
 *
 * Interface for memories configuration on board.
 *
 */

#ifndef BOARD_MEMORIES_H
#define BOARD_MEMORIES_H

/*----------------------------------------------------------------------------
 *        Headers
 *----------------------------------------------------------------------------*/


#ifdef __cplusplus
extern "C"
{
#endif

/*----------------------------------------------------------------------------
 *        Exported Types
 *----------------------------------------------------------------------------*/

/**
 * This structure give the size of each of the external memories EBIi and DDRCS
 * @note all the sizes are in MEGA BYTES !!!
 * @todo the external memory descriptor should be more than a size. It could have all the parameters for fine tuning.
 */
struct ExtMemDescriptor
{
  unsigned int EBICS0Size;
  unsigned int EBICS1Size;
  unsigned int EBICS2Size;
  unsigned int EBICS3Size;
  unsigned int DDRCSSize;
};

/*----------------------------------------------------------------------------
 *        Functions
 *----------------------------------------------------------------------------*/

/**
 * \brief Setup TLB for the board according to the given descriptor
 */
extern void board_setup_tlb_with_desc(struct ExtMemDescriptor* pDesc, unsigned int *tlb);


#ifdef __cplusplus
}
#endif


#endif  /* BOARD_MEMORIES_H */
