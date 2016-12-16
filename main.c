/* ----------------------------------------------------------------------------
 *         ATMEL Microcontroller Software Support
 * ----------------------------------------------------------------------------
 * Copyright (c) 2006, Atmel Corporation
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

#include "common.h"
#include "hardware.h"
#include "board.h"
#include "usart.h"
#include "debug.h"
#include "slowclk.h"
#include "dataflash.h"
#include "nandflash.h"
#include "sdcard.h"
#include "flash.h"
#include "string.h"
#include "board_hw_info.h"
#include "tz_utils.h"
#include "pm.h"
#include "act8865.h"
#include "sfr_aicredir.h"
#include "rstc.h"

#ifdef CONFIG_EXTERNAL_RAM_TEST
#include "ddram_utils.h"
#ifdef CONFIG_WITH_CACHE
#include "cp15.h"
#endif /*CONFIG_WITH_CACHE*/
#endif /*CONFIG_EXTERNAL_RAM_TEST*/

//TEST DDR selfrefresh
#include "arch/at91_pmc.h"


void (*sdcard_set_of_name)(char *) = NULL;

static void display_banner (void)
{
  //Banner from configuration.
  usart_puts(BANNER);

#if defined(CONFIG_UPLOAD_3RD_STAGE)
  static const char* const version = BOARD_NAME" Bootstrap - 3rd stage uploaded through DEBUG PROBE";
#elif defined(CONFIG_ONLY_INTERNAL_RAM)
  static const char* const version = BOARD_NAME" Bootstrap - ONLY internal RAM activated";
#else
  static const char* const version = BOARD_NAME" Bootstrap";
#endif

  char *ver_num = " "AT91BOOTSTRAP_VERSION" ("COMPILE_TIME")";

//CPU / BUS clock message
#if defined( CONFIG_CPU_CLK_498MHZ)
	static const char* const core_clock_msg = " CLOCKS: CPU: 498MHz, ";
#elif defined (CONFIG_CPU_CLK_400MHZ)
	static const char* const core_clock_msg = " CLOCKS: CPU: 400MHz, ";
#elif defined (CONFIG_CPU_CLK_528MHZ)
	static const char* const core_clock_msg = " CLOCKS: CPU: 528MHz, ";
#else
	static const char* const core_clock_msg = "CLOCKS: CPU: UNKNOWN, ";
#endif
#if defined (CONFIG_BUS_SPEED_176MHZ)
  static const char* const bus_clock_msg = "Bus: 176MHz";
#elif defined (CONFIG_BUS_SPEED_166MHZ)
  static const char* const bus_clock_msg = "Bus: 166MHz";
#elif defined (CONFIG_BUS_SPEED_133MHZ)
  static const char* const bus_clock_msg = "Bus: 133MHz";  
#else
  static const char* const bus_clock_msg = "Bus: UNKNOWN";
#endif
	usart_puts("\n");
	usart_puts(version);
	usart_puts(ver_num);
	usart_puts(core_clock_msg);
  usart_puts(bus_clock_msg);
	usart_puts("\n");
}
//**************************************************************************
int main(void)
{
	struct image_info image;
	int ret = 0;
  
#ifdef CONFIG_HW_INIT
  hw_init();
#endif

#if !(defined(CONFIG_ONLY_INTERNAL_RAM) || defined(CONFIG_UPLOAD_3RD_STAGE))
  
 
#endif  /* defined(CONFIG_ONLY_INTERNAL_RAM) || defined(CONFIG_UPLOAD_3RD_STAGE)*/
  
#if defined(CONFIG_SCLK)
#if !defined(CONFIG_SAMA5D4)
	slowclk_enable_osc32();
#endif
#endif

#ifdef CONFIG_HW_DISPLAY_BANNER
  display_banner();
#endif

#ifdef CONFIG_REDIRECT_ALL_INTS_AIC
	redirect_interrupts_to_nsaic();
#endif
  
#ifdef CONFIG_LOAD_HW_INFO
	load_board_hw_info();
#endif
//===================================================
#ifdef CONFIG_EXTERNAL_RAM_TEST
#warning RAM TEST
  
/**
 * @note : simply calling the do_external_ram_tests() function is currently LETHAL !!
 * The CP15_WriteControl() call in CP15_DisableDcache() raises an exception, but WHY ?? 
 * Stack looks good and LR too until called, once done, PC == LR !!
 * Nevertheless, it's ok when called "INLINE" as below.
 */
#if 1
#ifdef CONFIG_WITH_CACHE
  //disable the CACHE first.
  cp15_disable_icache();
  cp15_disable_dcache();
#endif
  
#ifdef CONFIG_EXTERNAL_RAM_TEST_INFINITE
#warning INFINITE RAM TEST
//@Note code below for the SAMA5D2 softpack support code.
for(;;)
{
#endif /*CONFIG_EXTERNAL_RAM_TEST_INFINITE*/

  usart_puts("##############################\n");
  
#ifdef CONFIG_EXTERNAL_RAM_TEST_WITHOUT_BURST  
  dbg_log(DEBUG_INFO,"Base memory access\n");
  do_external_ram_tests_step();  
#endif /*CONFIG_EXTERNAL_RAM_TEST_WITHOUT_BURST*/
  
#ifdef CONFIG_EXTERNAL_RAM_TEST_WITH_BURST
//Now test WITH CACHE Activated
  dbg_log(DEBUG_INFO,"WITH Memory BURST\n");

  cp15_enable_icache();
  cp15_enable_dcache();
  
  do_external_ram_tests_step();  
  
  cp15_disable_icache();
  cp15_disable_dcache();
#endif /*CONFIG_EXTERNAL_RAM_TEST_WITH_BURST*/

  dbg_log(DEBUG_INFO,"----------------------------\n");
  
#ifdef CONFIG_EXTERNAL_RAM_TEST_INFINITE
}
#endif /*CONFIG_EXTERNAL_RAM_TEST_INFINITE*/

//===================================================
#ifdef CONFIG_WITH_CACHE
  //Enable the CACHE if needed
  cp15_enable_icache();
  cp15_enable_dcache();
#endif /*CONFIG_WITH_CACHE*/
#else 
  //! @note Currently LETHAL, see the note above.
 do_external_ram_tests();
#endif  
  
#endif /*CONFIG_EXTERNAL_RAM_TEST*/
//===================================================
 
#ifdef CONFIG_PM
	at91_board_pm();
#endif

#ifdef CONFIG_DISABLE_ACT8865_I2C
	act8865_workaround();
#endif
  
#if defined(CONFIG_TEST_FIRMWARE_LOAD_LOOP)
  unsigned int retryLoop = 40000000;
  while (retryLoop--)
  {
#endif /*CONFIG_TEST_FIRMWARE_LOAD_LOOP*/
   
    LOAD_IMAGE_BEGIN:
    init_load_image(&image);
    ret = load_image(&image);
    
    LOAD_IMAGE_END:
    load_image_done(&image, ret);
    
#if defined(CONFIG_TEST_FIRMWARE_LOAD_LOOP)
    if (ret)
    {
      dbg_log(DEBUG_ERROR,"Load failed : %x, pausing...\r\n", ret);
      //Pause to let time to do the log.
      udelay(20000);
      udelay(20000);
      udelay(20000);
      udelay(20000);
      udelay(20000);
      asm("WFI");
    }
  }
  //EDF Test, reset if Ok.
  if (ret == 0)
    cpu_reset();
#endif /*CONFIG_TEST_FIRMWARE_LOAD_LOOP*/

#ifdef CONFIG_SCLK
	slowclk_switch_osc32();
#endif
  
dbg_error("FW upload done, let's start\n");
//asm("BKPT");
#if defined(CONFIG_ENTER_NWD)
	switch_normal_world();
	/* point never reached with TZ support */
#endif

return JUMP_ADDR;
}
//****************************************************
//Will just display a running symbol (use the ABI convention for parameters
void displayWaitDbg(unsigned int dbgdscr, unsigned int cpsr)
{
  dbg_log(DEBUG_INFO,"DBGDSCR:%b ; CPSR:0x%b\n", dbgdscr, cpsr);
  dbg_log(DEBUG_INFO,"Entering HALT Debug Mode, Waiting for the DEBUGGER ...\n");
}
//****************************************************
//Failure message
void displayFailedMsg()
{
  dbg_log(DEBUG_ERROR,"CPU not HALTED !\n");
}
//****************************************************
//Add the rolling prompt
static const char* ROLLING_PROMPT[] = {"|\r","/\r","-\r","\\\r"};

//! This function is called from the startup code if nothing is loaded.
void displayRollingPromptStep()
{
  static unsigned int loopIdx = 0;
  usart_puts(ROLLING_PROMPT[loopIdx++&0x3]);
}
//****************************************************

