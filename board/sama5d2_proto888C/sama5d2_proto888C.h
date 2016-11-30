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
#ifndef __SAMA5D2_PROTO888C_H__
#define __SAMA5D2_PROTO888C_H__

//Check build target
#ifndef CONFIG_SAMA5D2_PROTO888C
#error Use only for Proto888C board !!
#endif

/*
 * PMC Settings
 */
#define BOARD_MAINOSC		12000000

/* PCK: 498MHz, MCK: 166MHz */
#define BOARD_PLLA_MULA		82

#define BOARD_PCK		((unsigned long)((BOARD_MAINOSC * (BOARD_PLLA_MULA + 1)) / 2))
#define BOARD_MCK		((unsigned long)((BOARD_MAINOSC * (BOARD_PLLA_MULA + 1)) / 2 / 3))

#define BOARD_CKGR_PLLA		(AT91C_CKGR_SRCA | AT91C_CKGR_OUTA_0)
#define BOARD_PLLACOUNT		(0x3F << 8)
#define BOARD_MULA		((AT91C_CKGR_MULA << 2) & (BOARD_PLLA_MULA << 18))
#define BOARD_DIVA		(AT91C_CKGR_DIVA & 1)

#define BOARD_PRESCALER_MAIN_CLOCK	(AT91C_PMC_PLLADIV2_2 \
					| AT91C_PMC_MDIV_3 \
					| AT91C_PMC_CSS_MAIN_CLK)

#define BOARD_PRESCALER_PLLA		(AT91C_PMC_PLLADIV2_2 \
					| AT91C_PMC_MDIV_3 \
					| AT91C_PMC_CSS_PLLA_CLK)

#define MASTER_CLOCK		BOARD_MCK

#define PLLA_SETTINGS		(BOARD_CKGR_PLLA | \
				BOARD_PLLACOUNT | \
				BOARD_MULA | \
				BOARD_DIVA)

/*
 * DBGU Settings
 */
#define	USART_BASE	AT91C_BASE_UART1
#define DBGU_ID_UART AT91C_ID_UART1

/*
 * DataFlash Settings
 */
#define CONFIG_SYS_SPI_CLOCK	AT91C_SPI_CLK
#define CONFIG_SYS_SPI_MODE	SPI_MODE3


#if defined(CONFIG_SPI_BUS0)
#error SPI0 not used in Proto 888B Bootstrap !
#elif defined(CONFIG_SPI_BUS1)

#define CONFIG_SYS_BASE_SPI	AT91C_BASE_SPI1
#define CONFIG_SYS_ID_SPI	AT91C_ID_SPI1

#if defined(CONFIG_SPI1_IOSET_3)
#define CONFIG_SYS_SPI_PCS  AT91C_PIN_PD(28)
#else
#error SPI1 used with IOSet 3 only !
#endif
#endif /* #if defined(CONFIG_SPI_BUS1) */

#if defined(CONFIG_QSPI)

#ifndef CONFIG_QSPI1_IOSET_2
#error QSPI1 on IOSET 2 only.
#endif

#if defined(CONFIG_QSPI_BUS1)
#define	CONFIG_SYS_BASE_QSPI		AT91C_BASE_QSPI1
#define	CONFIG_SYS_BASE_QSPI_MEM	AT91C_BASE_QSPI1_MEM
#define	CONFIG_SYS_ID_QSPI		AT91C_ID_QSPI1
#else
#error QSPI0 not used in Proto 888C Bootstrap !
#endif

#endif /*#if defined(CONFIG_QSPI)*/

/*
 * SDHC Settings
 */
#if defined(CONFIG_SDHC0) || defined(CONFIG_SDHC1)
#error NO SDHC used in proto 888C
#endif

/*
 * MMU Table storage : 16kb (4096 descriptors(32bits)) at the end of the SRAM1 area
 */
#define MEMORY_TRANSLATION_TABLE_BASE_ADDR (0x220000 - (16 * 1024))

//! The LED SCENE PIO
#define PROTO888C_PIO_LED_SCENE AT91C_PIN_PD(5)
#endif
