/* ----------------------------------------------------------------------------
 *         SAM Software Package License
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
 * ----------------------------------------------------------------------------
 */

#ifndef _MMU_
#define _MMU_

#ifdef __cplusplus
extern "C" {
#endif

/*----------------------------------------------------------------------------
 *        Headers
 *----------------------------------------------------------------------------*/

#include "cp15.h"

#include "board_memories.h"

/*----------------------------------------------------------------------------
 *        Exported definitions
 *----------------------------------------------------------------------------*/

#define TTB_TYPE(x)                ((x) << 0)
#define TTB_TYPE_SECT              TTB_TYPE(2)

#define TTB_SECT_B(x)              ((x) << 2)
#define TTB_SECT_WRITE_THROUGH     TTB_SECT_B(0)
#define TTB_SECT_WRITE_BACK        TTB_SECT_B(1)

#define TTB_SECT_C(x)              ((x) << 3)
#define TTB_SECT_NON_CACHEABLE     TTB_SECT_C(0)
#define TTB_SECT_CACHEABLE         TTB_SECT_C(1)

#define TTB_SECT_XN(x)             ((x) << 4)
#define TTB_SECT_EXEC              TTB_SECT_XN(0)
#define TTB_SECT_EXEC_NEVER        TTB_SECT_XN(1)

#define TTB_SECT_DOMAIN(x)         ((x) << 5)

#define TTB_SECT_AP(x)             ((x) << 10)
#define TTB_SECT_APX(x)            ((x) << 15)
#define TTB_SECT_AP_PRIV_ONLY      (TTB_SECT_APX(0) | TTB_SECT_AP(1))
#define TTB_SECT_AP_NO_USER_WRITE  (TTB_SECT_APX(0) | TTB_SECT_AP(2))
#define TTB_SECT_AP_FULL_ACCESS    (TTB_SECT_APX(0) | TTB_SECT_AP(3))
#define TTB_SECT_AP_PRIV_READ_ONLY (TTB_SECT_APX(1) | TTB_SECT_AP(1))
#define TTB_SECT_AP_READ_ONLY      (TTB_SECT_APX(1) | TTB_SECT_AP(2))

#define TTB_SECT_TEX(x)            ((x) << 12)
#define TTB_SECT_STRONGLY_ORDERED  (TTB_SECT_TEX(0) | TTB_SECT_NON_CACHEABLE | TTB_SECT_WRITE_THROUGH)
#define TTB_SECT_SHAREABLE_DEVICE  (TTB_SECT_TEX(0) | TTB_SECT_NON_CACHEABLE | TTB_SECT_WRITE_BACK)
#define TTB_SECT_CACHEABLE_WT  (TTB_SECT_TEX(0) | TTB_SECT_CACHEABLE | TTB_SECT_WRITE_THROUGH)
#define TTB_SECT_CACHEABLE_WB  (TTB_SECT_TEX(0) | TTB_SECT_CACHEABLE | TTB_SECT_WRITE_BACK)

#define TTB_SECT_ADDR(x)           ((x) & 0xFFF00000)


/*----------------------------------------------------------------------------
 *        Exported functions
 *----------------------------------------------------------------------------*/
//*************** LOCAL functions declarations **************
/**
 * @brief This function will set the given translation Table into the MMU.
 * @param pTB [IN] The translation table pointer. (short descriptor). No actual Translation.
 */
extern void
mmu_set(unsigned int* pTB);

/**
 * \brief Initializes a MMU memory mapping, no translation activated according to the size of each external memories areas (EBI, DDRAM)
 * @param pDesc [IN] : eternal memory descriptor
 * \param pTB  [IN,OUT] : Address of the translation table.
 * @todo The external memories sizes should be checked !!
 * @deprecated replaced by board_setup_tlb_with_desc() since SoftPack 1.1
 */
extern void
mmu_tb_initialize(struct ExtMemDescriptor* pDesc, unsigned int *pTB);

/**
 * \brief Initializes the memory translation table & set the MMU with it.
 * \param pTB  Address of the translation table.
 * @deprecated use MMU_TB_Initialize() and MMU_Set() instead
 */
extern void
mmu_initialize(void);

#ifdef __cplusplus
}
#endif


#endif  /* #ifndef _MMU_ */
