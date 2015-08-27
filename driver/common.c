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
#include "common.h"
#include "dataflash.h"
#include "nandflash.h"
#include "sdcard.h"
#include "flash.h"
#include "string.h"
#include "usart.h"
#include "secure.h"
#include "debug.h"

#if defined(CONFIG_ONLY_INTERNAL_RAM) || defined(CONFIG_UPLOAD_3RD_STAGE)
//! Nothing to load at all.
static const char* const BOOT_MSG_SUCCESS = "Init Done\n";
static const char* const BOOT_MSG_FAILED =  "Init Failure\n";
static const char* const BOOT_MSG_RECOVERY = "SHOULD NEVER HAPPENS\n";
static const char* const BOOT_MSG_INVALID = "SHOULD NEVER HAPPENS\n";

#else
static const char* const BOOT_MSG_SUCCESS = "Done to load image\n";
static const char* const BOOT_MSG_FAILED =  "Failed to load image\n";
static const char* const BOOT_MSG_RECOVERY = "Success to recovery\n";
static const char* const BOOT_MSG_INVALID = "Invalid image loaded\n";
#endif /* defined(CONFIG_ONLY_INTERNAL_RAM) || defined(CONFIG_UPLOAD_3RD_STAGE) */


//****************************************************************************
/**
 * This function will check the loaded application with the expected value at the known address : dest+4
 */
#ifdef CONFIG_CHECK_APPLICATION_LOAD
static int check_loaded_application( struct image_info const* const image, unsigned int addr_offset, unsigned int expected_value)
{
  const unsigned int value = *(unsigned int*)(image->dest+addr_offset);
  return value == expected_value ? 0 : -3;
}
//*** Signal the application invalid load. TO BE DEFINED per board.
extern void signal_invalid_application(void);
#else 
#define signal_invalid_application()
#endif /*CONFIG_CHECK_APPLICATION_LOAD*/
//****************************************************************************


int load_nothing (struct image_info* unused)
{
  //NOTHING TO DO
  usart_puts("NOTHING to LOAD\r\n");
  return 0;
}


#ifdef CONFIG_SDCARD
char filename[FILENAME_BUF_LEN];
#ifdef CONFIG_OF_LIBFDT
char of_filename[FILENAME_BUF_LEN];
#endif
#endif

void init_load_image(struct image_info *image)
{
	memset(image,		0, sizeof(*image));
#ifdef CONFIG_SDCARD
	memset(filename,	0, FILENAME_BUF_LEN);
#ifdef CONFIG_OF_LIBFDT
	memset(of_filename,	0, FILENAME_BUF_LEN);
#endif
#endif

	image->dest = (unsigned char *)JUMP_ADDR;
#ifdef CONFIG_OF_LIBFDT
	image->of_dest = (unsigned char *)OF_ADDRESS;
#endif

#ifdef CONFIG_FLASH
	image->offset = IMG_ADDRESS;
#if !defined(CONFIG_LOAD_LINUX) && !defined(CONFIG_LOAD_ANDROID)
	image->length = IMG_SIZE;
#endif
#ifdef CONFIG_OF_LIBFDT
	image->of_offset = OF_OFFSET;
#endif
#endif

#ifdef CONFIG_NANDFLASH
	image->offset = IMG_ADDRESS;
#if !defined(CONFIG_LOAD_LINUX) && !defined(CONFIG_LOAD_ANDROID)
	image->length = IMG_SIZE;
#endif
#ifdef CONFIG_OF_LIBFDT
	image->of_offset = OF_OFFSET;
#endif
#endif

#ifdef CONFIG_DATAFLASH
	image->offset = IMG_ADDRESS;
#if !defined(CONFIG_LOAD_LINUX) && !defined(CONFIG_LOAD_ANDROID)
	image->length = IMG_SIZE;
#endif
#ifdef CONFIG_OF_LIBFDT
	image->of_offset = OF_OFFSET;
#endif
#endif

#ifdef CONFIG_SDCARD
	image->filename = filename;
	strcpy(image->filename, IMAGE_NAME);
#ifdef CONFIG_OF_LIBFDT
	image->of_filename = of_filename;
#endif
#endif
  
#if defined(CONFIG_SECURE)
  image->dest -= sizeof(at91_secure_header_t);
#endif


}

void load_image_done(struct image_info *image, int retval)
{
	char *media;

#if defined(CONFIG_FLASH)
	media = "FLASH: ";
#elif defined(CONFIG_NANDFLASH)
	media = "NAND: ";
#elif defined(CONFIG_DATAFLASH)
	media = "SF: ";
#elif defined(CONFIG_SDCARD)
	media = "SD/MMC: ";
#else
	media = NULL;
#endif

	if (media)
		usart_puts(media);
  
  if (retval == 0){
    dbg_log(DEBUG_INFO,BOOT_MSG_SUCCESS);
  }
  if (retval == -1) {
    dbg_log(DEBUG_ERROR,BOOT_MSG_FAILED);
  while(1);
  }
  if (retval == -2) {
    dbg_log(DEBUG_INFO,BOOT_MSG_RECOVERY);
    while (1);
  }
  if (retval == -3) {
    dbg_log(DEBUG_INFO,BOOT_MSG_INVALID);
    while (1) signal_invalid_application();
  }

#if !defined(CONFIG_UPLOAD_3RD_STAGE)  
#if defined(CONFIG_SECURE)
  if (!retval)
    retval = secure_check(image->dest);
  image->dest += sizeof(at91_secure_header_t);
#elif defined (CONFIG_CHECK_APPLICATION_LOAD)
  retval = check_loaded_application(image, CONFIG_CHECK_APPLICATION_VAL_ADDR_OFFSET, CONFIG_CHECK_APPLICATION_VALUE);
#endif
#endif /*CONFIG_UPLOAD_3RD_STAGE*/
}
