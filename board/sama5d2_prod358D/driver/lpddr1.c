/* ----------------------------------------------------------------------------
 *         ATMEL Microcontroller Software Support
 * ----------------------------------------------------------------------------
 * Copyright (c) 2014, HAGER Security.

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
#include "hardware.h"
#include "pmc.h"
#include "usart.h"
#include "debug.h"
#include "ddramc.h"
#include "spi.h"
#include "gpio.h"
#include "slowclk.h"
#include "timer.h"
#include "watchdog.h"
#include "string.h"

#include "arch/at91_pmc.h"
#include "arch/at91_rstc.h"
#include "arch/sama5_smc.h"
#include "arch/at91_pio.h"
#include "arch/at91_ddrsdrc.h"
#include "arch/at91_sfr.h"
#include "ddramc.h"

//*************** MEMORY CHIP functions initialization prototypes  ******
//  (implemented on dedicated memory support file)
extern void ddram_chip_config(struct ddramc_register *ddramc_config);

//*****************************************************************
//! This function aims at LPDDR1 initialization.
void ddramc_init(void)
{
  struct ddramc_register ddramc_reg;
  unsigned int reg;

  //Set the DDR_DQ and DDR_DQS to always On in SFR, see DS 33.4.1
  // Force Buffers "always on" FDQIEN & FSQSIEN as written in the DS.
  writel(AT91C_DDRCFG_FDQSIEN | AT91C_DDRCFG_FDQIEN, SFR_DDRCFG + AT91C_BASE_SFR);
  
  ddram_chip_config(&ddramc_reg);

  /* enable MPDDRC/DDR chip clocks */
  pmc_enable_periph_clock(AT91C_ID_MPDDRC);
  pmc_enable_system_clock(AT91C_PMC_DDR);
  
  /* MPDDRC I/O Calibration Register with DDRCK=166MHz, before MPDDRC configuration */
#if 1
  reg = readl(AT91C_BASE_MPDDRC + MPDDRC_IO_CALIBR);
  reg &= ~AT91C_MPDDRC_RDIV;
  reg |= AT91C_MPDDRC_RDIV_DDR2_RZQ_50;
  reg &= ~AT91C_MPDDRC_TZQIO;
  reg |= AT91C_MPDDRC_TZQIO_(100);
  reg |= AT91C_MPDDRC_ENABLE_CALIB;
#else
  reg = 0;
  reg |= AT91C_MPDDRC_RDIV_DDR2_RZQ_50;
  reg |= AT91C_MPDDRC_TZQIO_(100);
  reg |= AT91C_MPDDRC_ENABLE_CALIB;
#endif
  writel(reg, (AT91C_BASE_MPDDRC + MPDDRC_IO_CALIBR));

  writel(AT91C_MPDDRC_RD_DATA_PATH_ONE_CYCLES,
      (AT91C_BASE_MPDDRC + MPDDRC_RD_DATA_PATH));
  
  /* LP-DDRAM1 Controller initialization */
  ddram_initialize(AT91C_BASE_MPDDRC, AT91C_BASE_DDRCS, &ddramc_reg);
  
  ddramc_dump_regs(AT91C_BASE_MPDDRC);
  
  //DDR_DQS/DDR_DQ controlled by the MPDDRC
  writel(0, SFR_DDRCFG + AT91C_BASE_SFR);
}
