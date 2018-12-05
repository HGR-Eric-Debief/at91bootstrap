/* ----------------------------------------------------------------------------
 *         ATMEL Microcontroller Software Support
 * ----------------------------------------------------------------------------
 * Copyright (c) 2007, Stelian Pop <stelian.pop@leadtechdesign.com>
 * Copyright (c) 2007 Lead Tech Design <www.leadtechdesign.com>
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
#include "arch/at91_ddrsdrc.h"
#include "debug.h"
#include "ddramc.h"
#include "timer.h"

/* write DDRC register */
static void write_ddramc(unsigned int address,
			unsigned int offset,
			const unsigned int value)
{
	writel(value, (address + offset));
}

/* read DDRC registers */
static unsigned int read_ddramc(unsigned int address, unsigned int offset)
{
	return readl(address + offset);
}


#if defined (CONFIG_LPDDR2)

static int ddramc_decodtype_is_seq(unsigned int ddramc_cr)
{
#if defined(AT91SAM9X5) || defined(AT91SAM9N12) || defined(SAMA5D3X) \
	|| defined(SAMA5D4)
	if (ddramc_cr & AT91C_DDRC2_DECOD_INTERLEAVED)
		return 0;
#endif
	return 1;
}

int ddram_initialize(unsigned int base_address,
			unsigned int ram_address,
			struct ddramc_register *ddramc_config)
{
	unsigned int cr;

	write_ddramc(base_address, MPDDRC_LPDDR2_LPR,
				ddramc_config->lpddr2_lpr);

	/*
	 * Step 1: Program the memory device type into
	 * the Memory Device Register
	 */
	write_ddramc(base_address, HDDRSDRC2_MDR, ddramc_config->mdr);

	/*
	 * Step 2: Program the feature of Low-power DDR2-SDRAM device into
	 * the Timing Register, and into the Configuration Register
	 */
	write_ddramc(base_address, HDDRSDRC2_CR, ddramc_config->cr);

	write_ddramc(base_address, HDDRSDRC2_T0PR, ddramc_config->t0pr);
	write_ddramc(base_address, HDDRSDRC2_T1PR, ddramc_config->t1pr);
	write_ddramc(base_address, HDDRSDRC2_T2PR, ddramc_config->t2pr);

	/*
	 * Step 3: An NOP command is issued to the Low-power DDR2-SDRAM
	 * Program the NOP command into the Mode Register, the application must
	 * set the MODE field to 1 in the Mode Register. Perform a write access
	 * to any Low-power DDR2-SDRAM address to acknowledge this command.
	 * Now, clocks which drive Low-power DDR2-SDRAM device is enabled.
	 */
	write_ddramc(base_address, HDDRSDRC2_MR, AT91C_DDRC2_MODE_NOP_CMD);
	*((unsigned volatile int *)ram_address) = 0;

	/*
	 * A minimum pause of 100 ns must be reserved
	 * to precede any single toggle
	 */
	udelay(1);

	/*
	 * Step 4:  An NOP command is issued to the Low-power DDR2-SDRAM.
	 * Now, CKE is drive high.
	 */
	write_ddramc(base_address, HDDRSDRC2_MR, AT91C_DDRC2_MODE_NOP_CMD);
	*((unsigned volatile int *)ram_address) = 0;

	/* A minimum pause of 200us must be satisfied before Reset Command*/
	udelay(200);

	/*
	 * Step 5: A reset command is issued to the Low-power DDR2-SDRAM.
	 * Now, the reset command is issued.
	 */
	write_ddramc(base_address, HDDRSDRC2_MR,
			AT91C_DDRC2_MRS(63) | AT91C_DDRC2_MODE_LPDDR2_CMD);
	*((unsigned volatile int *)ram_address) = 0;

	/* A minimum pause of 1 us must be satisfied befor any commands */
	udelay(1);

	/*
	 * Step 6: A Mode Register Read command is issued to the Low-power
	 * DDR2-SDRAM. Now, the Mode Register Read command is issued.
	 */
	write_ddramc(base_address, HDDRSDRC2_MR,
			AT91C_DDRC2_MRS(0) | AT91C_DDRC2_MODE_LPDDR2_CMD);
	*((unsigned volatile int *)ram_address) = 0;

	/* A minimum pause of 10 us must be satified before any commands */
	udelay(10);

	/*
	 * Step 7: a calibration command is issued to the Low-power DDR2-SDRAM.
	 * Now, the ZQ Calibration command is issued.
	 */
	cr = read_ddramc(base_address, HDDRSDRC2_CR);
	cr &= ~AT91C_DDRC2_ZQ;
	cr |= AT91C_DDRC2_ZQ_RESET;
	write_ddramc(base_address, HDDRSDRC2_CR, cr);

	write_ddramc(base_address, HDDRSDRC2_MR,
			AT91C_DDRC2_MRS(10) | AT91C_DDRC2_MODE_LPDDR2_CMD);
	*((unsigned volatile int *)ram_address) = 0;
	udelay(1);

	cr = read_ddramc(base_address, HDDRSDRC2_CR);
	cr &= ~AT91C_DDRC2_ZQ;
	cr |= AT91C_DDRC2_ZQ_SHORT;
	write_ddramc(base_address, HDDRSDRC2_CR, cr);

	/*
	 * Step 8: A Mode Register Write command is issued to the Low-power
	 * DDR2-SDRAM. Now, the Mode Register Write command is issued.
	 */
	write_ddramc(base_address, HDDRSDRC2_MR,
			AT91C_DDRC2_MRS(1) | AT91C_DDRC2_MODE_LPDDR2_CMD);
	*((unsigned volatile int *)ram_address) = 0;
	udelay(1);

	/*
	 * Step 9: A Mode Register Write command is issued to the Low-power
	 * DDR2-SDRAM. Now, the Mode Register Write command is issued.
	 */
	write_ddramc(base_address, HDDRSDRC2_MR,
			AT91C_DDRC2_MRS(2) | AT91C_DDRC2_MODE_LPDDR2_CMD);
	*((unsigned volatile int *)ram_address) = 0;
	udelay(1);

	/*
	 * Step 10: A Mode Register Write command is issued to the Low-power
	 * DDR2-SDRAM. Now, the Mode Register Write command is issued.
	 */
	write_ddramc(base_address, HDDRSDRC2_MR,
			AT91C_DDRC2_MRS(3) | AT91C_DDRC2_MODE_LPDDR2_CMD);
	*((unsigned volatile int *)ram_address) = 0;
	udelay(1);

	/*
	 * Step 11: A Mode Register Write command is issued to the Low-power
	 * DDR2-SDRAM. Now, the Mode Register Write command is issued.
	 */
	write_ddramc(base_address, HDDRSDRC2_MR,
			AT91C_DDRC2_MRS(16) | AT91C_DDRC2_MODE_LPDDR2_CMD);
	*((unsigned volatile int *)ram_address) = 0;
	udelay(1);

	/*
	 * Step 12: Write the refresh rate into the COUNT field in
	 * the Refresh Timer register.
	 */
	write_ddramc(base_address, HDDRSDRC2_RTR, ddramc_config->rtr);

	write_ddramc(base_address, HDDRSDRC2_MR, AT91C_DDRC2_MODE_NORMAL_CMD);
	*((unsigned volatile int *)ram_address) = 0;
	udelay(1);

	/* Launch short ZQ calibration */
	cr = read_ddramc(base_address, HDDRSDRC2_CR);
	cr &= ~AT91C_DDRC2_ZQ;
	cr |= AT91C_DDRC2_ZQ_SHORT;
	write_ddramc(base_address, HDDRSDRC2_CR, cr);
	*((unsigned volatile int *)ram_address) = 0;

	write_ddramc(base_address, MPDDRC_LPDDR2_TIM_CAL,
						ddramc_config->tim_calr);

	return 0;
}

void ddramc_dump_regs(unsigned int base_address)
{
#if (BOOTSTRAP_DEBUG_LEVEL >= DEBUG_LOUD)
	unsigned int size = 0x160;

	dbg_info("\nDump DDRAMC Registers:\n");
	dbg_hexdump((unsigned char *)base_address, size, DUMP_WIDTH_BIT_32);
#endif
}
#else /*defined (CONFIG_LPDDR2)*/
#error Wrong memory type, here for LP-DDR2.
#endif /*defined (CONFIG_LPDDR2)*/
