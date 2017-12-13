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

/**
 * \file
 *
 * Implementation of memories configuration on board.
 *
 */


/*----------------------------------------------------------------------------
 *        Headers
 *----------------------------------------------------------------------------*/

#include "board_memories.h"

#include "mmu.h"
#include "cp15.h"


/*----------------------------------------------------------------------------
 *        Local functions
 *----------------------------------------------------------------------------*/

/*----------------------------------------------------------------------------
 *        Exported functions
 *----------------------------------------------------------------------------*/
void board_setup_tlb_with_desc(struct ExtMemDescriptor* pDesc,unsigned int *tlb)
{
  unsigned int addr;

  /* TODO: some peripherals are configured TTB_SECT_STRONGLY_ORDERED
     instead of TTB_SECT_SHAREABLE_DEVICE because their drivers have to
     be verified for correct operation when write-back is enabled */

  /* Reset table entries */
  for (addr = 0; addr < 4096; addr++)
      tlb[addr] = 0;

  /* 0x00000000: ROM */
  tlb[0x000] = TTB_SECT_ADDR(0x00000000)
             | TTB_SECT_AP_READ_ONLY
             | TTB_SECT_DOMAIN(0xf)
             | TTB_SECT_EXEC
             | TTB_SECT_CACHEABLE_WB
             | TTB_TYPE_SECT;

  /* 0x00100000: NFC SRAM */
  tlb[0x001] = TTB_SECT_ADDR(0x00100000)
             | TTB_SECT_AP_FULL_ACCESS
             | TTB_SECT_DOMAIN(0xf)
             | TTB_SECT_EXEC
             | TTB_SECT_SHAREABLE_DEVICE
             | TTB_TYPE_SECT;

  /* 0x00200000: SRAM */
  tlb[0x002] = TTB_SECT_ADDR(0x00200000)
             | TTB_SECT_AP_FULL_ACCESS
             | TTB_SECT_DOMAIN(0xf)
             | TTB_SECT_EXEC
             | TTB_SECT_STRONGLY_ORDERED
             | TTB_TYPE_SECT;

  /* 0x00300000: UDPHS (RAM) */
  tlb[0x003] = TTB_SECT_ADDR(0x00300000)
             | TTB_SECT_AP_FULL_ACCESS
             | TTB_SECT_DOMAIN(0xf)
             | TTB_SECT_EXEC_NEVER
             | TTB_SECT_SHAREABLE_DEVICE
             | TTB_TYPE_SECT;

  /* 0x00400000: UHPHS (OHCI) */
  tlb[0x004] = TTB_SECT_ADDR(0x00400000)
             | TTB_SECT_AP_FULL_ACCESS
             | TTB_SECT_DOMAIN(0xf)
             | TTB_SECT_EXEC_NEVER
             | TTB_SECT_SHAREABLE_DEVICE
             | TTB_TYPE_SECT;

  /* 0x00500000: UDPHS (EHCI) */
  tlb[0x005] = TTB_SECT_ADDR(0x00500000)
             | TTB_SECT_AP_FULL_ACCESS
             | TTB_SECT_DOMAIN(0xf)
             | TTB_SECT_EXEC_NEVER
             | TTB_SECT_SHAREABLE_DEVICE
             | TTB_TYPE_SECT;

  /* 0x00600000: AXIMX */
  tlb[0x006] = TTB_SECT_ADDR(0x00600000)
             | TTB_SECT_AP_FULL_ACCESS
             | TTB_SECT_DOMAIN(0xf)
             | TTB_SECT_EXEC_NEVER
             | TTB_SECT_SHAREABLE_DEVICE
             | TTB_TYPE_SECT;

  /* 0x00700000: DAP */
  tlb[0x007] = TTB_SECT_ADDR(0x00700000)
             | TTB_SECT_AP_FULL_ACCESS
             | TTB_SECT_DOMAIN(0xf)
             | TTB_SECT_EXEC_NEVER
             | TTB_SECT_SHAREABLE_DEVICE
             | TTB_TYPE_SECT;

  /* 0x00a00000: L2CC */
  tlb[0x00a] = TTB_SECT_ADDR(0x00a00000)
             | TTB_SECT_AP_FULL_ACCESS
             | TTB_SECT_DOMAIN(0xf)
             | TTB_SECT_EXEC_NEVER
             | TTB_SECT_SHAREABLE_DEVICE
             | TTB_TYPE_SECT;
  tlb[0x00b] = TTB_SECT_ADDR(0x00b00000)
             | TTB_SECT_AP_FULL_ACCESS
             | TTB_SECT_DOMAIN(0xf)
             | TTB_SECT_EXEC_NEVER
             | TTB_SECT_SHAREABLE_DEVICE
             | TTB_TYPE_SECT;

  /* 0x10000000: EBI Chip Select 0 */
  for (addr = 0x100; addr < (0x100 + pDesc->EBICS0Size); addr++)
      tlb[addr] = TTB_SECT_ADDR(addr << 20)
                    | TTB_SECT_AP_FULL_ACCESS
                    | TTB_SECT_DOMAIN(0xf)
                    | TTB_SECT_EXEC_NEVER
                    | TTB_SECT_STRONGLY_ORDERED
                    | TTB_TYPE_SECT;
#if 0
  /* 0x20000000: DDR Chip Select */
  /* (64MB cacheable, 448MB strongly ordered) */
  for (addr = 0x200; addr < 0x240; addr++)
      tlb[addr] = TTB_SECT_ADDR(addr << 20)
                    | TTB_SECT_AP_FULL_ACCESS
                    | TTB_SECT_DOMAIN(0xf)
                    | TTB_SECT_EXEC
                    | TTB_SECT_CACHEABLE_WB
                    | TTB_TYPE_SECT;

  for (addr = 0x240; addr < 0x400; addr++)
      tlb[addr] = TTB_SECT_ADDR(addr << 20)
                    | TTB_SECT_AP_FULL_ACCESS
                    | TTB_SECT_DOMAIN(0xf)
                    | TTB_SECT_EXEC
                    | TTB_SECT_STRONGLY_ORDERED
                    | TTB_TYPE_SECT;
#else
  //full RAM is cacheable
  for (addr = 0x200; addr < (0x200 + pDesc->DDRCSSize); addr++)
        tlb[addr] = TTB_SECT_ADDR(addr << 20)
                        | TTB_SECT_AP_FULL_ACCESS
                        | TTB_SECT_DOMAIN(0xf)
                        | TTB_SECT_EXEC
                        | TTB_SECT_CACHEABLE_WB
                        | TTB_TYPE_SECT;

#endif
  /* 0x40000000: DDR AESB Chip Select */
  for (addr = 0x400; addr < 0x600; addr++)
      tlb[addr] = TTB_SECT_ADDR(addr << 20)
                    | TTB_SECT_AP_FULL_ACCESS
                    | TTB_SECT_DOMAIN(0xf)
                    | TTB_SECT_EXEC
                    | TTB_SECT_CACHEABLE_WB
                    | TTB_TYPE_SECT;

  /* 0x60000000: EBI Chip Select 1 */
  for (addr = 0x600; addr < (0x600 + pDesc->EBICS1Size); addr++)
      tlb[addr] = TTB_SECT_ADDR(addr << 20)
                    | TTB_SECT_AP_FULL_ACCESS
                    | TTB_SECT_DOMAIN(0xf)
                    | TTB_SECT_EXEC_NEVER
                    | TTB_SECT_STRONGLY_ORDERED
                    | TTB_TYPE_SECT;

  /* 0x70000000: EBI Chip Select 2 */
  for (addr = 0x700; addr < (0x700 + pDesc->EBICS2Size); addr++)
      tlb[addr] = TTB_SECT_ADDR(addr << 20)
                    | TTB_SECT_AP_FULL_ACCESS
                    | TTB_SECT_DOMAIN(0xf)
                    | TTB_SECT_EXEC_NEVER
                    | TTB_SECT_STRONGLY_ORDERED
                    | TTB_TYPE_SECT;

  /* 0x80000000: EBI Chip Select 3 */
  for (addr = 0x800; addr < (0x800 + pDesc->EBICS3Size); addr++)
      tlb[addr] = TTB_SECT_ADDR(addr << 20)
                    | TTB_SECT_AP_FULL_ACCESS
                    | TTB_SECT_DOMAIN(0xf)
                    | TTB_SECT_EXEC_NEVER
                    | TTB_SECT_STRONGLY_ORDERED
                    | TTB_TYPE_SECT;

  /* 0x90000000: QSPI0/1 AESB MEM */
  for (addr = 0x900; addr < 0xa00; addr++)
      tlb[addr] = TTB_SECT_ADDR(addr << 20)
                    | TTB_SECT_AP_FULL_ACCESS
                    | TTB_SECT_DOMAIN(0xf)
                    | TTB_SECT_EXEC
                    | TTB_SECT_STRONGLY_ORDERED
                    | TTB_TYPE_SECT;

  /* 0xa0000000: SDMMC0 */
  for (addr = 0xa00; addr < 0xb00; addr++)
      tlb[addr] = TTB_SECT_ADDR(addr << 20)
                    | TTB_SECT_AP_FULL_ACCESS
                    | TTB_SECT_DOMAIN(0xf)
                    | TTB_SECT_EXEC_NEVER
                    //| TTB_SECT_SHAREABLE_DEVICE
                    | TTB_SECT_STRONGLY_ORDERED
                    | TTB_TYPE_SECT;

  /* 0xb0000000: SDMMC1 */
  for (addr = 0xb00; addr < 0xc00; addr++)
      tlb[addr] = TTB_SECT_ADDR(addr << 20)
                    | TTB_SECT_AP_FULL_ACCESS
                    | TTB_SECT_DOMAIN(0xf)
                    | TTB_SECT_EXEC_NEVER
                    //| TTB_SECT_SHAREABLE_DEVICE
                    | TTB_SECT_STRONGLY_ORDERED
                    | TTB_TYPE_SECT;

  /* 0xc0000000: NFC Command Register */
  for (addr = 0xc00; addr < 0xd00; addr++)
      tlb[addr] = TTB_SECT_ADDR(addr << 20)
                    | TTB_SECT_AP_FULL_ACCESS
                    | TTB_SECT_DOMAIN(0xf)
                    | TTB_SECT_EXEC_NEVER
                    //| TTB_SECT_SHAREABLE_DEVICE
                    | TTB_SECT_STRONGLY_ORDERED
                    | TTB_TYPE_SECT;

  /* 0xd0000000: QSPI0/1 MEM */
  for (addr = 0xd00; addr < 0xe00; addr++)
      tlb[addr] = TTB_SECT_ADDR(addr << 20)
                    | TTB_SECT_AP_FULL_ACCESS
                    | TTB_SECT_DOMAIN(0xf)
                    | TTB_SECT_EXEC
                    | TTB_SECT_STRONGLY_ORDERED
                    | TTB_TYPE_SECT;

  /* 0xf0000000: Internal Peripherals */
  tlb[0xf00] = TTB_SECT_ADDR(0xf0000000)
             | TTB_SECT_AP_FULL_ACCESS
             | TTB_SECT_DOMAIN(0xf)
             | TTB_SECT_EXEC
             | TTB_SECT_STRONGLY_ORDERED
             | TTB_TYPE_SECT;

  /* 0xf8000000: Internal Peripherals */
  tlb[0xf80] = TTB_SECT_ADDR(0xf8000000)
             | TTB_SECT_AP_FULL_ACCESS
             | TTB_SECT_DOMAIN(0xf)
             | TTB_SECT_EXEC
             | TTB_SECT_STRONGLY_ORDERED
             | TTB_TYPE_SECT;

  /* 0xfc000000: Internal Peripherals */
  tlb[0xfc0] = TTB_SECT_ADDR(0xfc000000)
             | TTB_SECT_AP_FULL_ACCESS
             | TTB_SECT_DOMAIN(0xf)
             | TTB_SECT_EXEC
             | TTB_SECT_STRONGLY_ORDERED
             | TTB_TYPE_SECT;

}
