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
 * Implementation of xDMA controller (XDMAC).
 *
 */

/*----------------------------------------------------------------------------
 *        Headers
 *----------------------------------------------------------------------------*/

#include "hardware.h"
#include "common.h"
#include "driver/at91_xdma.h"


/*----------------------------------------------------------------------------
 *        Exported functions
 *----------------------------------------------------------------------------*/

Xdmac *xdmac_get_instance(unsigned int index)
{
	if (index == 0)
		return (Xdmac *) AT91C_BASE_XDMAC0;
	else if (index == 1)
		return (Xdmac *) AT91C_BASE_XDMAC1;
	else
		return NULL;
}

unsigned int xdmac_get_periph_id(Xdmac *xdmac)
{
	if (xdmac == (Xdmac *)AT91C_BASE_XDMAC0)
		return AT91C_ID_XDMAC0;
	else if (xdmac == (Xdmac *)AT91C_BASE_XDMAC1)
		return AT91C_ID_XDMAC1;
	else
		return AT91C_ID_COUNTS;
}

unsigned int xdmac_get_type(Xdmac *xdmac)
{
	return xdmac->XDMAC_GTYPE;
}

unsigned int xdmac_get_config(Xdmac *xdmac)
{
	return xdmac->XDMAC_GCFG;
}

unsigned int xdmac_get_arbiter(Xdmac *xdmac)
{
	return xdmac->XDMAC_GWAC;
}

void xdmac_enable_global_it(Xdmac *xdmac, unsigned int int_mask)
{
	xdmac->XDMAC_GIE = int_mask;
}

void xdmac_disable_global_it(Xdmac *xdmac, unsigned int int_mask)
{
	xdmac->XDMAC_GID = int_mask;
}

unsigned int xdmac_get_global_it_mask(Xdmac *xdmac)
{
	return xdmac->XDMAC_GIM;
}

unsigned int xdmac_get_global_isr(Xdmac *xdmac)
{
	return xdmac->XDMAC_GIS;
}

unsigned int xdmac_get_masked_global_isr(Xdmac *xdmac)
{
	unsigned int mask = xdmac->XDMAC_GIM;

	return xdmac->XDMAC_GIS & mask;
}

void xdmac_enable_channel(Xdmac *xdmac, unsigned char channel)
{
	xdmac->XDMAC_GE |= XDMAC_GE_EN0 << channel;
}

void xdmac_enable_channels(Xdmac *xdmac, unsigned char channel_mask)
{
	xdmac->XDMAC_GE = channel_mask;
}

void xdmac_disable_channel(Xdmac *xdmac, unsigned char channel)
{
	xdmac->XDMAC_GD |= XDMAC_GD_DI0 << channel;
}

void xdmac_disable_channels(Xdmac *xdmac, unsigned char channel_mask)
{
	xdmac->XDMAC_GD = channel_mask;
}

unsigned int xdmac_get_global_channel_status(Xdmac *xdmac)
{
	return xdmac->XDMAC_GS;
}

void xdmac_suspend_read_channel(Xdmac *xdmac, unsigned char channel)
{
	xdmac->XDMAC_GRS |= XDMAC_GRS_RS0 << channel;
}

void xdmac_suspend_write_channel(Xdmac *xdmac, unsigned char channel)
{
	xdmac->XDMAC_GWS |= XDMAC_GWS_WS0 << channel;
}

void xdmac_suspend_read_write_channel(Xdmac *xdmac, unsigned char channel)
{
	xdmac->XDMAC_GRWS |= XDMAC_GRWS_RWS0 << channel;
}

void xdmac_resume_read_write_channel(Xdmac *xdmac, unsigned char channel)
{
	xdmac->XDMAC_GRWR |= XDMAC_GRWR_RWR0 << channel;
}

void xdmac_software_transfer_request(Xdmac *xdmac, unsigned char channel)
{
	xdmac->XDMAC_GSWR |= (XDMAC_GSWR_SWREQ0 << channel);
}

unsigned int xdmac_get_software_transfer_status(Xdmac *xdmac)
{
	return xdmac->XDMAC_GSWS;
}

void xdmac_software_flush_request(Xdmac *xdmac, unsigned char channel)
{
	xdmac->XDMAC_GSWF |= XDMAC_GSWF_SWF0 << channel;
}

void xdmac_enable_channel_it(Xdmac *xdmac, unsigned char channel, unsigned int int_mask)
{
	xdmac->XDMAC_CHID[channel].XDMAC_CIE = int_mask;
}

void xdmac_disable_channel_it(Xdmac *xdmac, unsigned char channel, unsigned int int_mask)
{
	xdmac->XDMAC_CHID[channel].XDMAC_CID = int_mask;
}

unsigned int xdmac_get_channel_it_mask(Xdmac *xdmac, unsigned char channel)
{
	return xdmac->XDMAC_CHID[channel].XDMAC_CIM;
}

unsigned int xdmac_get_channel_isr(Xdmac *xdmac, unsigned char channel)
{
	return xdmac->XDMAC_CHID[channel].XDMAC_CIS;
}

unsigned int xdmac_get_channel_isr_mask(Xdmac *xdmac, unsigned char channel)
{
	return xdmac->XDMAC_CHID[channel].XDMAC_CIM;
}

unsigned int xdmac_get_masked_channel_isr(Xdmac *xdmac, unsigned char channel)
{
	unsigned int mask = xdmac->XDMAC_CHID[channel].XDMAC_CIM;

	return xdmac->XDMAC_CHID[channel].XDMAC_CIS & mask;
}

void xdmac_set_src_addr(Xdmac *xdmac, unsigned char channel, void *addr)
{
	xdmac->XDMAC_CHID[channel].XDMAC_CSA = (unsigned int)addr;
}

void xdmac_set_dest_addr(Xdmac *xdmac, unsigned char channel, void *addr)
{
	xdmac->XDMAC_CHID[channel].XDMAC_CDA = (unsigned int)addr;
}

void xdmac_set_descriptor_addr(Xdmac *xdmac, unsigned char channel, void *addr,
		unsigned int ndaif)
{
	xdmac->XDMAC_CHID[channel].XDMAC_CNDA = (((unsigned int)addr) & 0xFFFFFFFC) | ndaif;
}

void xdmac_set_descriptor_control(Xdmac *xdmac, unsigned char channel,
		unsigned int config)
{
	xdmac->XDMAC_CHID[channel].XDMAC_CNDC = config;
}

void xdmac_set_microblock_control(Xdmac *xdmac, unsigned char channel,
		unsigned int ublen)
{
	xdmac->XDMAC_CHID[channel].XDMAC_CUBC = ublen;
}

void xdmac_set_block_control(Xdmac *xdmac, unsigned char channel, unsigned int blen)
{
	xdmac->XDMAC_CHID[channel].XDMAC_CBC = blen;
}

void xdmac_set_channel_config(Xdmac *xdmac, unsigned char channel, unsigned int config)
{
	xdmac->XDMAC_CHID[channel].XDMAC_CC = config;
}

unsigned int xdmac_get_channel_config(Xdmac *xdmac, unsigned char channel)
{
	return xdmac->XDMAC_CHID[channel].XDMAC_CC;
}

void xdmac_set_data_stride_mem_pattern(Xdmac *xdmac, unsigned char channel,
			       unsigned int dds_msp)
{
	xdmac->XDMAC_CHID[channel].XDMAC_CDS_MSP = dds_msp;
}

void xdmac_set_src_microblock_stride(Xdmac *xdmac, unsigned char channel,
		unsigned int subs)
{
	xdmac->XDMAC_CHID[channel].XDMAC_CSUS = subs;
}

void xdmac_set_dest_microblock_stride(Xdmac *xdmac, unsigned char channel,
		unsigned int dubs)
{
	xdmac->XDMAC_CHID[channel].XDMAC_CDUS = dubs;
}

unsigned int xdmac_get_channel_dest_addr(Xdmac *xdmac, unsigned char channel)
{
	return xdmac->XDMAC_CHID[channel].XDMAC_CDA;
}
