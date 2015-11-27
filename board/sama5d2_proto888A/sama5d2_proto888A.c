/* ----------------------------------------------------------------------------
 *         HAGER Security Prot888A software support
 * ----------------------------------------------------------------------------
 * Copyright (c) 2015, HAGER Security
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
#include "pmc.h"
#include "usart.h"
#include "debug.h"
#include "ddramc.h"
#include "gpio.h"
#include "timer.h"
#include "watchdog.h"
#include "string.h"

#include "arch/at91_pmc.h"
#include "arch/at91_rstc.h"
#include "arch/at91_pio.h"
#include "sama5d2_proto888A.h"

#include "l2cc.h"
#include "act8865.h"
#include "twi.h"
#include "arch/tz_matrix.h"
#include "matrix.h"

#if defined(CONFIG_WITH_MMU)
#include "cp15.h"
#include "mmu.h"
#include "board_memories.h"
#endif


//*************** MEMORY functions initialization prototypes ******

#ifdef CONFIG_LPDDR1
  /* Initialize MPDDR Controller for LPDDR1*/
void  ddramc_init(void);
#elif !defined (CONFIG_ONLY_INTERNAL_RAM)
#error BeeAve module only support LP-DDR1 RAM.
#endif

#if defined(CONFIG_BEEAVE_WITH_DEBUG_LEDS)
//! This function will initialize the DEBUG LEDS of the BeeAve module : set them GREEN:NONE:ORANGE to see Bootstrap running.
// Note : 0 : ON, 1 : OFF
void debug_leds_init(void)
{
#ifdef CONFIG_BEEAVE_DEBUG_LEDS_ON
  const struct pio_desc debug_led_pio[] =
    {
      { "GREEN_LED_1", AT91C_PIN_PD(17), 0, PIO_DEFAULT, PIO_OUTPUT },
      { "RED_LED_1", AT91C_PIN_PD(18), 1, PIO_DEFAULT, PIO_OUTPUT },
      { "GREEN_LED_2", AT91C_PIN_PD(23), 0, PIO_DEFAULT, PIO_OUTPUT },
      { "RED_LED_2", AT91C_PIN_PD(24), 0, PIO_DEFAULT, PIO_OUTPUT },
      { "GREEN_LED_3", AT91C_PIN_PD(19), 0, PIO_DEFAULT, PIO_OUTPUT },
      { "RED_LED_3", AT91C_PIN_PD(20), 0, PIO_DEFAULT, PIO_OUTPUT },
      PIO_DESCRIPTION_END };
#endif /*CONFIG_BEEAVE_DEBUG_LEDS_ON*/

#ifdef CONFIG_BEEAVE_DEBUG_LEDS_OFF
#warning DEBUG LED OFF.
const struct pio_desc debug_led_pio[] =
    {
      { "GREEN_LED_1", AT91C_PIN_PD(17), 0, PIO_DEFAULT, PIO_INPUT },
      { "RED_LED_1", AT91C_PIN_PD(18), 0, PIO_DEFAULT, PIO_INPUT },
      { "GREEN_LED_2", AT91C_PIN_PD(23), 0, PIO_DEFAULT, PIO_INPUT },
      { "RED_LED_2", AT91C_PIN_PD(24), 0, PIO_DEFAULT, PIO_INPUT },
      { "GREEN_LED_3", AT91C_PIN_PD(19), 0, PIO_DEFAULT, PIO_INPUT },
      { "RED_LED_3", AT91C_PIN_PD(20), 0, PIO_DEFAULT, PIO_INPUT },
      PIO_DESCRIPTION_END };
#endif /*CONFIG_BEEAVE_DEBUG_LEDS_OFF*/

  pmc_enable_periph_clock(AT91C_ID_PIOD);
  pio_configure(debug_led_pio);
}
#else
//Nothing to do
#define debug_leds_init() 
#endif /*#if defined(CONFIG_BEEAVE_WITH_DEBUG_LEDS)*/
//*********************************************************
//! This function will init the LIGHTING LED (WHITE) of the Proto888A module : leave it OFF.
void lighting_led_init(void)
{
  const struct pio_desc lighting_led_pio[] =
    {
      { "LED_SCENE", AT91C_PIN_PD(5), 0, PIO_DEFAULT, PIO_OUTPUT },
      PIO_DESCRIPTION_END };

  pmc_enable_periph_clock(AT91C_ID_PIOD);
  pio_configure(lighting_led_pio);
}
//*********************************************************
static void at91_dbgu_hw_init(void)
{
	const struct pio_desc dbgu_pins[] = {
		{"URXD1", AT91C_PIN_PD(2), 0, PIO_DEFAULT, PIO_PERIPH_A},
		{"UTXD1", AT91C_PIN_PD(3), 0, PIO_DEFAULT, PIO_PERIPH_A},
		PIO_DESCRIPTION_END,
	};

	pio_configure(dbgu_pins);
	pmc_sam9x5_enable_periph_clk(AT91C_ID_UART1);
}

static void initialize_dbgu(void)
{
	unsigned int baudrate = 57600;

	at91_dbgu_hw_init();

	if (pmc_check_mck_h32mxdiv())
		usart_init(BAUDRATE(MASTER_CLOCK / 2, baudrate));
	else
		usart_init(BAUDRATE(MASTER_CLOCK, baudrate));
}

#if defined(CONFIG_MATRIX)
static int matrix_configure_slave(void)
{
	unsigned int ddr_port;
	unsigned int ssr_setting, sasplit_setting, srtop_setting;

	/*
	 * Matrix 0 (H64MX)
	 */

	/*
	 * 0: Bridge from H64MX to AXIMX
	 * (Internal ROM, Crypto Library, PKCC RAM): Always Secured
	 */

	/* 1: H64MX Peripheral Bridge */

	/* 2 ~ 9 DDR2 Port1 ~ 7: Non-Secure */
	srtop_setting = MATRIX_SRTOP(0, MATRIX_SRTOP_VALUE_128M);
	sasplit_setting = (MATRIX_SASPLIT(0, MATRIX_SASPLIT_VALUE_128M)
				| MATRIX_SASPLIT(1, MATRIX_SASPLIT_VALUE_128M)
				| MATRIX_SASPLIT(2, MATRIX_SASPLIT_VALUE_128M)
				| MATRIX_SASPLIT(3, MATRIX_SASPLIT_VALUE_128M));
	ssr_setting = (MATRIX_LANSECH_NS(0)
			| MATRIX_LANSECH_NS(1)
			| MATRIX_LANSECH_NS(2)
			| MATRIX_LANSECH_NS(3)
			| MATRIX_RDNSECH_NS(0)
			| MATRIX_RDNSECH_NS(1)
			| MATRIX_RDNSECH_NS(2)
			| MATRIX_RDNSECH_NS(3)
			| MATRIX_WRNSECH_NS(0)
			| MATRIX_WRNSECH_NS(1)
			| MATRIX_WRNSECH_NS(2)
			| MATRIX_WRNSECH_NS(3));
	/* DDR port 0 not used from NWd */
	for (ddr_port = 1; ddr_port < 8; ddr_port++) {
		matrix_configure_slave_security(AT91C_BASE_MATRIX64,
					(H64MX_SLAVE_DDR2_PORT_0 + ddr_port),
					srtop_setting,
					sasplit_setting,
					ssr_setting);
	}

	/*
	 * 10: Internal SRAM 128K
	 * TOP0 is set to 128K
	 * SPLIT0 is set to 64K
	 * LANSECH0 is set to 0, the low area of region 0 is the Securable one
	 * RDNSECH0 is set to 0, region 0 Securable area is secured for reads.
	 * WRNSECH0 is set to 0, region 0 Securable area is secured for writes
	 */
	srtop_setting = MATRIX_SRTOP(0, MATRIX_SRTOP_VALUE_128K);
	sasplit_setting = MATRIX_SASPLIT(0, MATRIX_SASPLIT_VALUE_64K);
	ssr_setting = (MATRIX_LANSECH_S(0)
			| MATRIX_RDNSECH_S(0)
			| MATRIX_WRNSECH_S(0));
	matrix_configure_slave_security(AT91C_BASE_MATRIX64,
					H64MX_SLAVE_INTERNAL_SRAM,
					srtop_setting,
					sasplit_setting,
					ssr_setting);

	/* 11:  Internal SRAM 128K (Cache L2) */
	/* 12:  QSPI0 */
	/* 13:  QSPI1 */
	/* 14:  AESB */

	/*
	 * Matrix 1 (H32MX)
	 */

	/* 0: Bridge from H32MX to H64MX: Not Secured */

	/* 1: H32MX Peripheral Bridge 0: Not Secured */

	/* 2: H32MX Peripheral Bridge 1: Not Secured */

	/*
	 * 3: External Bus Interface
	 * EBI CS0 Memory(256M) ----> Slave Region 0, 1
	 * EBI CS1 Memory(256M) ----> Slave Region 2, 3
	 * EBI CS2 Memory(256M) ----> Slave Region 4, 5
	 * EBI CS3 Memory(128M) ----> Slave Region 6
	 * NFC Command Registers(128M) -->Slave Region 7
	 *
	 * NANDFlash(EBI CS3) --> Slave Region 6: Non-Secure
	 */
	srtop_setting =	MATRIX_SRTOP(6, MATRIX_SRTOP_VALUE_128M);
	srtop_setting |= MATRIX_SRTOP(7, MATRIX_SRTOP_VALUE_128M);
	sasplit_setting = MATRIX_SASPLIT(6, MATRIX_SASPLIT_VALUE_128M);
	sasplit_setting |= MATRIX_SASPLIT(7, MATRIX_SASPLIT_VALUE_128M);
	ssr_setting = (MATRIX_LANSECH_NS(6)
			| MATRIX_RDNSECH_NS(6)
			| MATRIX_WRNSECH_NS(6));
	ssr_setting |= (MATRIX_LANSECH_NS(7)
			| MATRIX_RDNSECH_NS(7)
			| MATRIX_WRNSECH_NS(7));
	matrix_configure_slave_security(AT91C_BASE_MATRIX32,
					H32MX_EXTERNAL_EBI,
					srtop_setting,
					sasplit_setting,
					ssr_setting);

	/* 4: NFC SRAM (4K): Non-Secure */
	srtop_setting = MATRIX_SRTOP(0, MATRIX_SRTOP_VALUE_8K);
	sasplit_setting = MATRIX_SASPLIT(0, MATRIX_SASPLIT_VALUE_8K);
	ssr_setting = (MATRIX_LANSECH_NS(0)
			| MATRIX_RDNSECH_NS(0)
			| MATRIX_WRNSECH_NS(0));
	matrix_configure_slave_security(AT91C_BASE_MATRIX32,
					H32MX_NFC_SRAM,
					srtop_setting,
					sasplit_setting,
					ssr_setting);

	/* 5:
	 * USB Device High Speed Dual Port RAM (DPR): 1M
	 * USB Host OHCI registers: 1M
	 * USB Host EHCI registers: 1M
	 */
	srtop_setting = (MATRIX_SRTOP(0, MATRIX_SRTOP_VALUE_1M)
			| MATRIX_SRTOP(1, MATRIX_SRTOP_VALUE_1M)
			| MATRIX_SRTOP(2, MATRIX_SRTOP_VALUE_1M));
	sasplit_setting = (MATRIX_SASPLIT(0, MATRIX_SASPLIT_VALUE_1M)
			| MATRIX_SASPLIT(1, MATRIX_SASPLIT_VALUE_1M)
			| MATRIX_SASPLIT(2, MATRIX_SASPLIT_VALUE_1M));
	ssr_setting = (MATRIX_LANSECH_NS(0)
			| MATRIX_LANSECH_NS(1)
			| MATRIX_LANSECH_NS(2)
			| MATRIX_RDNSECH_NS(0)
			| MATRIX_RDNSECH_NS(1)
			| MATRIX_RDNSECH_NS(2)
			| MATRIX_WRNSECH_NS(0)
			| MATRIX_WRNSECH_NS(1)
			| MATRIX_WRNSECH_NS(2));
	matrix_configure_slave_security(AT91C_BASE_MATRIX32,
					H32MX_USB,
					srtop_setting,
					sasplit_setting,
					ssr_setting);

	return 0;
}

static unsigned int security_ps_peri_id[] = {
	0,
};

static int matrix_config_periheral(void)
{
	unsigned int *peri_id = security_ps_peri_id;
	unsigned int array_size = sizeof(security_ps_peri_id) / sizeof(unsigned int);
	int ret;

	ret = matrix_configure_peri_security(peri_id, array_size);
	if (ret)
		return -1;

	return 0;
}

static int matrix_init(void)
{
	int ret;

	matrix_write_disable(AT91C_BASE_MATRIX64);
	matrix_write_disable(AT91C_BASE_MATRIX32);

	ret = matrix_configure_slave();
	if (ret)
		return -1;

	ret = matrix_config_periheral();
	if (ret)
		return -1;

	return 0;
}
#endif	/* #if defined(CONFIG_MATRIX) */

#if defined(CONFIG_WITH_MMU)
unsigned int* MEMORY_TRANSLATION_TABLE_BASE = (unsigned int*) MEMORY_TRANSLATION_TABLE_BASE_ADDR;
#endif

#ifdef CONFIG_HW_INIT
void hw_init(void)
{
  #if defined(CONFIG_WITH_MMU)
  struct ExtMemDescriptor memDescriptor =
    {
      CONFIG_EBI_0_SIZE,
      CONFIG_EBI_1_SIZE,
      CONFIG_EBI_2_SIZE,
      CONFIG_EBI_3_SIZE,
      CONFIG_RAM_SIZE};
#endif

	/* Disable watchdog */
	at91_disable_wdt();

	/*
	 * while coming from the ROM code, we run on PLLA @ 396 MHz / 132 MHz
	 * so we need to slow down and configure MCKR accordingly.
	 * This is why we have a special flavor of the switching function.
	 */

	/* Switch PCK/MCK on Main Clock output */
	pmc_cfg_mck_down(BOARD_PRESCALER_MAIN_CLOCK);

	/* Configure PLLA */
	pmc_cfg_plla(PLLA_SETTINGS);

	/* Initialize PLLA charge pump */
	/* No need: we keep what is set in ROM code */
	//pmc_init_pll(0x3);

	/* Switch MCK on PLLA output */
	pmc_cfg_mck(BOARD_PRESCALER_PLLA);

	/* Enable External Reset */
	writel(AT91C_RSTC_KEY_UNLOCK | AT91C_RSTC_URSTEN,
					AT91C_BASE_RSTC + RSTC_RMR);

	/* initialize the dbgu */
	initialize_dbgu();
  usart_puts("... BOOTING ...\n");
  
#if defined(CONFIG_MATRIX)
  /* Initialize the matrix */
  matrix_init();
#endif

	/* Init timer */
	timer_init();
#ifdef CONFIG_DDRC
  /* Initialize MPDDR Controller */
	ddramc_init();
#endif
  
	/* Prepare L2 cache setup */
	l2cache_prepare();
  
  //LED init
  lighting_led_init();
  debug_leds_init();

#if defined(CONFIG_WITH_MMU)
#warning MMU activated
  mmu_tb_initialize(&memDescriptor, MEMORY_TRANSLATION_TABLE_BASE);
  mmu_set(MEMORY_TRANSLATION_TABLE_BASE);
  cp15_enable_mmu();
#if defined(CONFIG_WITH_CACHE)
#warning CACHE activated
  cp15_enable_dcache();
  cp15_enable_icache();
#endif /*CONFIG_CACHE_ENABLED*/
#endif

#if defined(CONFIG_WITH_MMU)
//Check MMU & Cache status
  if (cp15_is_mmu_enabled())
    usart_puts("MMU ENabled\r");
  else
    usart_puts("MMU DISabled\r");

  if (cp15_is_dcache_enabled())
    usart_puts("D Cache ENabled\r");
  else
    usart_puts("D Cache DISabled\r");

  if (cp15_is_icache_enabled())
    usart_puts("I Cache ENabled\r");
  else
    usart_puts("I Cache DISabled\r");
#endif

  
}
#endif /* #ifdef CONFIG_HW_INIT */

#ifdef CONFIG_DATAFLASH
#if defined(CONFIG_SPI)
void at91_spi0_hw_init(void)
{

#if defined(CONFIG_SPI_BUS1) && defined(CONFIG_SPI1_IOSET_3)
	const struct pio_desc spi_pins[] = {
		{"SPI1_SPCK",	AT91C_PIN_PD(25), 0, PIO_DEFAULT, PIO_PERIPH_A},
		{"SPI1_MOSI",	AT91C_PIN_PD(26), 0, PIO_DEFAULT, PIO_PERIPH_A},
		{"SPI1_MISO",	AT91C_PIN_PD(27), 0, PIO_DEFAULT, PIO_PERIPH_A},
		{"SPI1_NPCS",	AT91C_PIN_PD(28), 1, PIO_DEFAULT, PIO_OUTPUT},
		PIO_DESCRIPTION_END,
	};
#else
#error "Only SPI1 on IOSet 3 used !!!"
#endif

	pio_configure(spi_pins);
  
	pmc_sam9x5_enable_periph_clk(AT91C_ID_SPI1);
}
#endif

#if defined (CONFIG_QSPI)
void at91_qspi_hw_init(void)
{

#if defined(CONFIG_QSPI_BUS1) && defined(CONFIG_QSPI1_IOSET_2)
	const struct pio_desc qspi_pins[] = {
		{"QSPI1_SCK",	AT91C_PIN_PB(5),  0, PIO_DEFAULT, PIO_PERIPH_D},
		{"QSPI1_CS",	AT91C_PIN_PB(6),  0, PIO_DEFAULT, PIO_PERIPH_D},
		{"QSPI1_IO0",	AT91C_PIN_PB(7),  0, PIO_DEFAULT, PIO_PERIPH_D},
		{"QSPI1_IO1",	AT91C_PIN_PB(8),  0, PIO_DEFAULT, PIO_PERIPH_D},
		{"QSPI1_IO2",	AT91C_PIN_PB(9),  0, PIO_DEFAULT, PIO_PERIPH_D},
		{"QSPI1_IO3",	AT91C_PIN_PB(10), 0, PIO_DEFAULT, PIO_PERIPH_D},
		PIO_DESCRIPTION_END,
	};
#else
#error "Only QSPI1 on IOSet 2 used !!"
#endif

	pio_configure(qspi_pins);
	pmc_sam9x5_enable_periph_clk(AT91C_ID_QSPI1);
}
#endif
#endif