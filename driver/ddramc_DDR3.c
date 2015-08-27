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

static int ddramc_decodtype_is_seq(unsigned int ddramc_cr)
{
#if defined(AT91SAM9X5) || defined(AT91SAM9N12) || defined(SAMA5D3X) \
	|| defined(SAMA5D4)
	if (ddramc_cr & AT91C_DDRC2_DECOD_INTERLEAVED)
		return 0;
#endif
	return 1;
}

#if defined(CONFIG_DDR3)

int ddram_initialize(unsigned int base_address,
			unsigned int ram_address,
			struct ddramc_register *ddramc_config)
{
	unsigned int ba_offset;

	/* Compute BA[] offset according to CR configuration */
	ba_offset = (ddramc_config->cr & AT91C_DDRC2_NC) + 9;
	if (!(ddramc_config->cr & AT91C_DDRC2_DECOD_INTERLEAVED))
		ba_offset += ((ddramc_config->cr & AT91C_DDRC2_NR) >> 2) + 11;

	ba_offset += (ddramc_config->mdr & AT91C_DDRC2_DBW) ? 1 : 2;

	dbg_very_loud(" ba_offset = %x ...\n", ba_offset);

	/*
	 * Step 1: Program the memory device type in the MPDDRC Memory Device Register
	 */
	write_ddramc(base_address, HDDRSDRC2_MDR, ddramc_config->mdr);

	/*
	 * Step 2: Program features of the DDR3-SDRAM device in the MPDDRC
	 * Configuration Register and in the MPDDRC Timing Parameter 0 Register
	 * /MPDDRC Timing Parameter 1 Register
	 */
	write_ddramc(base_address, HDDRSDRC2_CR, ddramc_config->cr);

	write_ddramc(base_address, HDDRSDRC2_T0PR, ddramc_config->t0pr);
	write_ddramc(base_address, HDDRSDRC2_T1PR, ddramc_config->t1pr);
	write_ddramc(base_address, HDDRSDRC2_T2PR, ddramc_config->t2pr);

	/*
	 * Step 3: A NOP command is issued to the DDR3-SRAM.
	 * Program the NOP command in the MPDDRC Mode Register (MPDDRC_MR).
	 * The application must write a one to the MODE field in the MPDDRC_MR
	 * Perform a write access to any DDR3-SDRAM address to acknowledge this command.
	 * The clock which drive the DDR3-SDRAM device are now enabled.
	 */
	write_ddramc(base_address, HDDRSDRC2_MR, AT91C_DDRC2_MODE_NOP_CMD);
	*((unsigned volatile int *)ram_address) = 0;

	/*
	 * Step 4: A pause of at least 500us must be observed before a single toggle.
	 */
	 udelay(500);

	/*
	 * Step 5: A NOP command is issued to the DDR3-SDRAM
	 * Program the NOP command in the MPDDRC_MR.
	 * The application must write a one to the MODE field in the MPDDRC_MR.
	 * Perform a write access to any DDR3-SDRAM address to acknowledge this command.
	 * CKE is now driven high.
	 */
	write_ddramc(base_address, HDDRSDRC2_MR, AT91C_DDRC2_MODE_NOP_CMD);
	*((unsigned volatile int *)ram_address) = 0;

	/*
	 * Step 6: An Extended Mode Register Set (EMRS2) cycle is issued to choose
	 * between commercial or high temperature operations. The application must
	 * write a five to the MODE field in the MPDDRC_MR and perform a write
	 * access to the DDR3-SDRAM to acknowledge this command.
	 * The write address must be chosen so that signal BA[2] is set to 0,
	 * BA[1] is set to 1 and signal BA[0] is set to 0.
	 */
	write_ddramc(base_address, HDDRSDRC2_MR, AT91C_DDRC2_MODE_EXT_LMR_CMD);
	*((unsigned int *)(ram_address + (0x2 << ba_offset))) = 0;

	/*
	 * Step 7: An Extended Mode Register Set (EMRS3) cycle is issued to set
	 * the Extended Mode Register to 0. The application must write a five
	 * to the MODE field in the MPDDRC_MR and perform a write access to the
	 * DDR3-SDRAM to acknowledge this command. The write address must be
	 * chosen so that signal BA[2] is set to 0, BA[1] is set to 1 and signal
	 * BA[0] is set to 1.
	 */
	write_ddramc(base_address, HDDRSDRC2_MR, AT91C_DDRC2_MODE_EXT_LMR_CMD);
	*((unsigned int *)(ram_address + (0x3 << ba_offset))) = 0;

	/*
	 * Step 8: An Extended Mode Register Set (EMRS1) cycle is issued to
	 * disable and to program O.D.S. (Output Driver Strength).
	 * The application must write a five to the MODE field in the MPDDRC_MR
	 * and perform a write access to the DDR3-SDRAM to acknowledge this command.
	 * The write address must be chosen so that signal BA[2:1] is set to 0
	 * and signal BA[0] is set to 1.
	 */
	write_ddramc(base_address, HDDRSDRC2_MR, AT91C_DDRC2_MODE_EXT_LMR_CMD);
	*((unsigned int *)(ram_address + (0x1 << ba_offset))) = 0;

	/*
	 * Step 9: Write a one to the DLL bit (enable DLL reset) in the MPDDRC
	 * Configuration Register (MPDDRC_CR)
	 */
#if 0
	cr = read_ddramc(base_address, HDDRSDRC2_CR);
	write_ddramc(base_address, HDDRSDRC2_CR, cr | AT91C_DDRC2_DLL_RESET_ENABLED);
#endif

	/*
	 * Step 10: A Mode Register Set (MRS) cycle is issued to reset DLL.
	 * The application must write a three to the MODE field in the MPDDRC_MR
	 * and perform a write access to the DDR3-SDRAM to acknowledge this command.
	 * The write address must be chosen so that signals BA[2:0] are set to 0
	 */
	write_ddramc(base_address, HDDRSDRC2_MR, AT91C_DDRC2_MODE_LMR_CMD);
	*((unsigned int *)ram_address) = 0;

	udelay(50);

	/*
	 * Step 11: A Calibration command (MRS) is issued to calibrate RTT and
	 * RON values for the Process Voltage Temperature (PVT).
	 * The application must write a six to the MODE field in the MPDDRC_MR
	 * and perform a write access to the DDR3-SDRAM to acknowledge this command.
	 * The write address must be chosen so that signals BA[2:0] are set to 0.
	 */
	write_ddramc(base_address, HDDRSDRC2_MR, AT91C_DDRC2_MODE_DEEP_CMD);
	*((unsigned int *)ram_address) = 0;

	/*
	 * Step 12: A Normal Mode command is provided.
	 * Program the Normal mode in the MPDDRC_MR and perform a write access
	 * to any DDR3-SDRAM address to acknowledge this command.
	 */
	write_ddramc(base_address, HDDRSDRC2_MR, AT91C_DDRC2_MODE_NORMAL_CMD);
	*((unsigned int *)ram_address) = 0;

	/*
	 * Step 13: Perform a write access to any DDR3-SDRAM address.
	 */
	*((unsigned int *)ram_address) = 0;

	/*
	 * Step 14: Write the refresh rate into the COUNT field in the MPDDRC
	 * Refresh Timer Register (MPDDRC_RTR):
	 * refresh rate = delay between refresh cycles.
	 * The DDR3-SDRAM device requires a refresh every 7.81 us.
	 */
	write_ddramc(base_address, HDDRSDRC2_RTR, ddramc_config->rtr);

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
#else /*defined(CONFIG_DDR3)*/
#error Wrong memory type, here for DDR3.
#endif /*defined(CONFIG_DDR3)*/
