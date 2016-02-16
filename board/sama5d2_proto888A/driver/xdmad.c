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

/** \addtogroup xdmad_module
 *
 * \section Xdma xDma Configuration Usage
 *
 * To configure a XDMA channel, the user has to follow these few steps :
 * <ul>
 * <li> Initialize a XDMA driver instance by XDMAD_Initialize().</li>
 * <li> choose an available (disabled) channel using XDMAD_AllocateChannel().</li>
 * <li> After the XDMAC selected channel has been programmed, XDMAD_PrepareChannel() is to enable
 * clock and dma peripheral of the DMA, and set Configuration register to set up the transfer type
 * (memory or non-memory peripheral for source and destination) and flow control device.</li>
 * <li> Invoke XDMAD_StartTransfer() to start DMA transfer  or XDMAD_StopTransfer() to force stop DMA transfer.</li>
  * <li> Once the buffer of data is transferred, XDMAD_IsTransferDone() checks if DMA transfer is finished.</li>
 * <li> XDMAD_Handler() handles XDMA interrupt, and invoking XDMAD_SetCallback() if provided.</li>
 * </ul>
 *
 * Related files:\n
 * \ref xdmad.h\n
 * \ref xdmad.c\n
 */

/** \file */

/** \addtogroup dmad_functions
  @{*/

/*----------------------------------------------------------------------------
 *        Includes
 *----------------------------------------------------------------------------*/

#include "common.h"
#include "pmc.h"
#include "driver/xdmad.h"
#include "debug.h"


/*----------------------------------------------------------------------------
 *        Local definitions
 *----------------------------------------------------------------------------*/

#define XDMAD_CHANNELS (XDMAC_CONTROLLERS * XDMAC_CHANNELS)

/** DMA state for channel */
enum {
	XDMAD_STATE_FREE = 0,  /**< Free channel */
	XDMAD_STATE_ALLOCATED, /**< Allocated to some peripheral */
	XDMAD_STATE_STARTED,   /**< DMA started */
	XDMAD_STATE_DONE,      /**< DMA transfer done */
  XDMAD_STATE_READ_FAILED, /**< DMA Read failed */
  XDMAD_STATE_WRITE_FAILED, /**< DMA Write failed */
  XDMAD_STATE_OVERFLOW, /**< DMA Overflow condition */
  XDMAD_STATE_IN_PROGRESS, /**< DMA operation in progress (new block processed).*/
};

/** DMA driver channel */
struct _xdmad_channel
{
	Xdmac           *xdmac;     /**< XDMAC instance */
	unsigned int         id;        /**< Channel ID */
	xdmad_callback_t callback;  /**< Callback */
	void            *user_arg;  /**< Callback argument */
	unsigned char          src_txif;  /**< Source TX Interface ID */
	unsigned char          src_rxif;  /**< Source RX Interface ID */
	unsigned char          dest_txif; /**< Destination TX Interface ID */
	unsigned char          dest_rxif; /**< Destination RX Interface ID */
	volatile unsigned char state;     /**< Channel State */
};

/** DMA driver instance */
struct _xdmad {
	struct _xdmad_channel channels[XDMAD_CHANNELS];
	bool                  polling;
	unsigned char         polling_timeout;
};

static struct _xdmad _xdmad;

struct peripheral_xdma {
        unsigned int  id;   /**< Peripheral ID */
        unsigned char iftx; /**< DMA Interface for TX */
        unsigned char ifrx; /**< DMA Interface for RX */
};

static const struct peripheral_xdma _xdmac_peripherals[] = {
        { AT91C_ID_TWI0,        0,    1 },
        { AT91C_ID_TWI1,        2,    3 },
        { AT91C_ID_QSPI0,       4,    5 },
        { AT91C_ID_SPI0,        6,    7 },
        { AT91C_ID_SPI1,        8,    9 },
        { AT91C_ID_PWM,        10, 0xff },
        { AT91C_ID_FLEXCOM0,   11,   12 },
        { AT91C_ID_FLEXCOM1,   13,   14 },
        { AT91C_ID_FLEXCOM2,   15,   16 },
        { AT91C_ID_FLEXCOM3,   17,   18 },
        { AT91C_ID_FLEXCOM4,   19,   20 },
        { AT91C_ID_SSC0,       21,   22 },
        { AT91C_ID_SSC1,       23,   24 },
        { AT91C_ID_ADC,      0xff,   25 },
        { AT91C_ID_AES,        26,   27 },
        { AT91C_ID_TDES,       28,   29 },
        { AT91C_ID_SHA,        30, 0xff },
        { AT91C_ID_I2SC0,      31,   32 },
        { AT91C_ID_I2SC1,      33,   34 },
        { AT91C_ID_UART0,      35,   36 },
        { AT91C_ID_UART1,      37,   38 },
        { AT91C_ID_UART2,      39,   40 },
        { AT91C_ID_UART3,      41,   42 },
        { AT91C_ID_UART4,      43,   44 },
        { AT91C_ID_TC0,      0xff,   45 },
        { AT91C_ID_TC1,      0xff,   46 },
        { AT91C_ID_CLASSD,     47, 0xff },
        { AT91C_ID_QSPI0,      48,   49 },
        { AT91C_ID_PDMIC,    0xff,   50 },
};

/*----------------------------------------------------------------------------
 *        Local functions
 *----------------------------------------------------------------------------*/
static const struct peripheral_xdma *get_peripheral_xdma(unsigned int id, Xdmac *xdmac)
{
        int i;

        if ((unsigned int)xdmac != AT91C_BASE_XDMAC0 && (unsigned int)xdmac != AT91C_BASE_XDMAC1) {
                return NULL;
        }

        for (i = 0; i < ARRAY_SIZE(_xdmac_peripherals); i++) {
                if (_xdmac_peripherals[i].id == id) {
                        return &_xdmac_peripherals[i];
                }
        }

        return NULL;
}

static inline bool is_peripheral_on_xdma_controller(unsigned int id, Xdmac *xdmac)
{
  return get_peripheral_xdma(id, xdmac) != NULL;
}

static unsigned char get_peripheral_xdma_channel(unsigned int id, Xdmac *xdmac, bool transmit)
{
        const struct peripheral_xdma *periph_xdma = get_peripheral_xdma(id, xdmac);
        if (periph_xdma) {
                return transmit ? periph_xdma->iftx : periph_xdma->ifrx;
        } else {
                return 0xff;
        }
}

static inline struct _xdmad_channel *_xdmad_channel(unsigned int controller, unsigned int channel)
{
	return &_xdmad.channels[controller * XDMAC_CHANNELS + channel];
}

/**
 * \brief xDMA interrupt handler
 * \param pXdmad Pointer to DMA driver instance.
 */
static void xdmad_handler(void)
{
	unsigned int cont;

	for (cont= 0; cont< XDMAC_CONTROLLERS; cont++) 
  {
		unsigned int chan, gis, gcs;

		Xdmac *xdmac = xdmac_get_instance(cont);

		gis = xdmac_get_global_isr(xdmac);
		if ((gis & 0xFFFF) == 0)
			continue;

		gcs = xdmac_get_global_channel_status(xdmac);
    
    dbg_log(DEBUG_VERY_LOUD, "%s(), gis:%x, gcs:%x\n", __FUNCTION__, gis, gcs );

    for (chan = 0; chan < XDMAC_CHANNELS; chan++) 
    {
			struct _xdmad_channel *channel;
			bool execCallback = false;

			if (!(gis & (1 << chan)))
				continue;
           
			channel = _xdmad_channel(cont, chan);
			if (channel->state == XDMAD_STATE_FREE)
				continue;

			//if (!(gcs & (1 << chan)))  //With this condition, BIS is never seen in descriptor based transaction.
      {
				const unsigned int cis = xdmac_get_channel_isr(xdmac, chan);
        const unsigned int cim = xdmac_get_channel_isr_mask(xdmac, chan);
        
        dbg_log(DEBUG_LOUD, "%s(), Chan[%d]:cim:%x, cis:%x\n", __FUNCTION__, chan, cim, cis);

        if (cis & XDMAC_CIS_BIS)
        {
          //Single block or a list
          if ((xdmac_get_channel_it_mask(xdmac, chan) & XDMAC_CIM_LIM) && !(cis & XDMAC_CIS_LIS))
          {
            //one block in a list
            channel->state = XDMAD_STATE_IN_PROGRESS;
            execCallback = true;
          }
          else
          {
            //Single block
            channel->state = XDMAD_STATE_DONE;
            execCallback = true;
          }
        }
        else if (cis & XDMAC_CIS_LIS)
        {
          channel->state = XDMAD_STATE_DONE;
          execCallback = true;
        }
        else if (cis & XDMAC_CIS_DIS)
        {
          channel->state = XDMAD_STATE_DONE;
          execCallback = true;
        }
        else if (cis & XDMAC_CIS_RBEIS)
        {
          channel->state = XDMAD_STATE_READ_FAILED;
          execCallback = true;
        }
        else if (cis & XDMAC_CIS_WBEIS)
        {
          channel->state = XDMAD_STATE_WRITE_FAILED;
          execCallback = true;
        }
        else if (cis & XDMAC_CIS_ROIS)
        {
          channel->state = XDMAD_STATE_OVERFLOW;
          execCallback = true;
        }
        /* Execute callback if needed */
        if (execCallback && channel->callback)
        {
          channel->callback(channel, channel->user_arg);
        }
      }
		}
	}
}

/*----------------------------------------------------------------------------
 *        Exported functions
 *----------------------------------------------------------------------------*/

void xdmad_initialize(/*bool polling*/)
{
	unsigned int cont, chan;

	_xdmad.polling = 1;//FORCED VALUE

	for (cont = 0; cont < XDMAC_CONTROLLERS; cont++) {
		Xdmac* xdmac = xdmac_get_instance(cont);
		for (chan = 0; chan < XDMAC_CHANNELS; chan++) {
			xdmac_get_channel_isr(xdmac, chan);
			struct _xdmad_channel *channel = _xdmad_channel(cont, chan);
			channel->xdmac = xdmac;
			channel->id = chan;
			channel->callback = 0;
			channel->user_arg = 0;
			channel->src_txif = 0;
			channel->src_rxif = 0;
			channel->dest_txif = 0;
			channel->dest_rxif = 0;
			channel->state = XDMAD_STATE_FREE;
		}
#if 0
#error NOT YET supported, mut activate the interrupt and set the IRQ stack pointer !!
		if (!polling) {
			unsigned int pid = xdmac_get_periph_id(xdmac);
			/* enable interrupts */
      IRQ_ConfigureIT(pid, 0, xdmad_handler);
      IRQ_EnableIT(pid);
		}
#endif
    
  }
}

void xdmad_poll(void)
{
	if (_xdmad.polling)
		xdmad_handler();
}

struct _xdmad_channel *xdmad_allocate_channel(unsigned char src, unsigned char dest)
{
	unsigned int i;

	/* Reject peripheral to peripheral transfers */
	if (src != XDMAD_PERIPH_MEMORY && dest != XDMAD_PERIPH_MEMORY) {
		return NULL;
	}

	for (i = 0; i < XDMAD_CHANNELS; i++) {
		struct _xdmad_channel *channel = &_xdmad.channels[i];
		Xdmac *xdmac = channel->xdmac;

		if (channel->state == XDMAD_STATE_FREE) {
			/* Check if source peripheral matches this channel controller */
			if (src != XDMAD_PERIPH_MEMORY)
				if (!is_peripheral_on_xdma_controller(src, xdmac))
					continue;

			/* Check if destination peripheral matches this channel controller */
			if (dest != XDMAD_PERIPH_MEMORY)
				if (!is_peripheral_on_xdma_controller(dest, xdmac))
					continue;

			/* Allocate the channel */
			channel->state = XDMAD_STATE_ALLOCATED;
			channel->src_txif = get_peripheral_xdma_channel(src, xdmac, true);
			channel->src_rxif = get_peripheral_xdma_channel(src, xdmac, false);
			channel->dest_txif = get_peripheral_xdma_channel(dest, xdmac, true);
			channel->dest_rxif = get_peripheral_xdma_channel(dest, xdmac, false);

			return channel;
		}
	}
	return NULL;
}

unsigned int xdmad_free_channel(struct _xdmad_channel *channel)
{
	switch (channel->state) {
	case XDMAD_STATE_STARTED:
		return XDMAD_BUSY;
	case XDMAD_STATE_ALLOCATED:
	case XDMAD_STATE_DONE:
		channel->state = XDMAD_STATE_FREE;
		break;
	}
	return XDMAD_OK;
}

unsigned int xdmad_set_callback(struct _xdmad_channel *channel,
		xdmad_callback_t callback, void *user_arg)
{
	if (channel->state == XDMAD_STATE_FREE)
		return XDMAD_ERROR;
	else if (channel->state == XDMAD_STATE_STARTED)
		return XDMAD_BUSY;

	channel->callback = callback;
	channel->user_arg = user_arg;

	return XDMAD_OK;
}

unsigned int xdmad_prepare_channel(struct _xdmad_channel *channel)
{
	Xdmac *xdmac = channel->xdmac;

	if (channel->state == XDMAD_STATE_FREE)
		return XDMAD_ERROR;
	else if (channel->state == XDMAD_STATE_STARTED)
		return XDMAD_BUSY;

	/* Clear status */
	xdmac_get_global_channel_status(xdmac);
	xdmac_get_global_isr(xdmac);

	/* Enable clock of the DMA peripheral */
	pmc_enable_periph_clock(xdmac_get_periph_id(xdmac));

	/* Clear status */
	xdmac_get_channel_isr(xdmac, channel->id);

	/* Disables XDMAC interrupt for the given channel */
	xdmac_disable_global_it(xdmac, -1);
	xdmac_disable_channel_it(xdmac, channel->id, -1);

	/* Disable the given dma channel */
	xdmac_disable_channel(xdmac, channel->id);
	xdmac_set_src_addr(xdmac, channel->id, 0);
	xdmac_set_dest_addr(xdmac, channel->id, 0);
	xdmac_set_block_control(xdmac, channel->id, 0);
	xdmac_set_channel_config(xdmac, channel->id, XDMAC_CC_PROT_UNSEC);
	xdmac_set_descriptor_addr(xdmac, channel->id, 0, 0);
	xdmac_set_descriptor_control(xdmac, channel->id, 0);

	return XDMAD_OK;
}

bool xdmad_is_transfer_done(struct _xdmad_channel *channel)
{
	return (channel->state == XDMAD_STATE_DONE ) || (channel->state != XDMAD_STATE_STARTED && channel->state != XDMAD_STATE_IN_PROGRESS);
}

unsigned int xdmad_configure_transfer(struct _xdmad_channel *channel,
				  struct _xdmad_cfg *cfg,
				  unsigned int desc_cntrl,
				  void *desc_addr)
{
	if (channel->state == XDMAD_STATE_FREE)
		return XDMAD_ERROR;
	else if (channel->state == XDMAD_STATE_STARTED)
		return XDMAD_BUSY;

	Xdmac *xdmac = channel->xdmac;
  
	/* Clear status */
	xdmac_get_global_isr(xdmac);
	xdmac_get_channel_isr(xdmac, channel->id);
  
  //Set PERID in the configuration, for NVD2 & NVD3 must be done in the client code.
  cfg->cfg.bitfield.perid = xdmad_get_sync_perid(channel, cfg->cfg.bitfield.dsync);
  
  //Set the base configuration will be overridden by the descriptors if any according their respective types.
  xdmac_set_src_addr(xdmac, channel->id, cfg->src_addr);
  xdmac_set_dest_addr(xdmac, channel->id, cfg->dest_addr);
  xdmac_set_microblock_control(xdmac, channel->id, cfg->ublock_size);
  //xdmac_set_block_control(xdmac, channel->id, cfg->block_size > 1 ? cfg->block_size : 0);//Not needed as block_size == 0 : one block
  xdmac_set_block_control(xdmac, channel->id, cfg->block_size);
  xdmac_set_data_stride_mem_pattern(xdmac, channel->id, cfg->data_stride);
  xdmac_set_src_microblock_stride(xdmac, channel->id, cfg->src_ublock_stride);
  xdmac_set_dest_microblock_stride(xdmac, channel->id, cfg->dest_ublock_stride);
  xdmac_set_channel_config(xdmac, channel->id, cfg->cfg.uint32_value);

  //Check whether it's a descriptor or straight configuration based transaction
  if ((desc_cntrl & XDMAC_CNDC_NDE) == XDMAC_CNDC_NDE_DSCR_FETCH_EN)
  {
    /* Descriptor based transaction enabled*/
    xdmac_set_descriptor_addr(xdmac, channel->id, desc_addr, 0);
    xdmac_set_descriptor_control(xdmac, channel->id, desc_cntrl);
    xdmac_disable_channel_it(xdmac, channel->id, -1);

    //Needed even no interrupt used as it will drive the channel IS in the GIS register
    xdmac_enable_channel_it(xdmac, channel->id, 
                            XDMAC_CIE_LIE | XDMAC_CIE_BIE | XDMAC_CIE_RBIE |
                            XDMAC_CIE_WBIE | XDMAC_CIE_ROIE); 
  }
  else
  {
    //No descriptor
    xdmac_set_descriptor_addr(xdmac, channel->id, 0, 0);
    xdmac_set_descriptor_control(xdmac, channel->id, 0);
    //Note : interrupt must be activated even if not used otherwise, it won't toggle the GIS bit and then xdmad_handler() won't trig the modification.
    xdmac_enable_channel_it(xdmac, channel->id,
    XDMAC_CIE_BIE | XDMAC_CIE_DIE |
    XDMAC_CIE_FIE | XDMAC_CIE_RBIE |
    XDMAC_CIE_WBIE | XDMAC_CIE_ROIE);
  }
	return XDMAD_OK;
}

unsigned int xdmad_start_transfer(struct _xdmad_channel *channel)
{
	if (channel->state == XDMAD_STATE_FREE)
		return XDMAD_ERROR;
	else if (channel->state == XDMAD_STATE_STARTED)
		return XDMAD_BUSY;

	/* Change state to 'started' */
	channel->state = XDMAD_STATE_STARTED;

	/* Start DMA transfer */
	xdmac_enable_channel(channel->xdmac, channel->id);
	if (!_xdmad.polling) {
		xdmac_enable_global_it(channel->xdmac, 1 << channel->id);
	}

	return XDMAD_OK;
}

unsigned int xdmad_stop_transfer(struct _xdmad_channel *channel)
{
	Xdmac *xdmac = channel->xdmac;

	/* Disable channel */
	xdmac_disable_channel(xdmac, channel->id);

	/* Disable interrupts */
	xdmac_disable_channel_it(xdmac, channel->id, -1);

	/* Clear pending status */
	xdmac_get_channel_isr(xdmac, channel->id);
	xdmac_get_global_channel_status(xdmac);

	/* Change state to 'allocated' */
	channel->state = XDMAD_STATE_ALLOCATED;

	return XDMAD_OK;
}
//****************************************************
extern unsigned int xdmad_get_channel_id (struct _xdmad_channel const* const channel)
{
  return channel->id;
}
//****************************************************
unsigned int xdmad_get_sync_perid(struct _xdmad_channel *channel, unsigned int dsync)
{
  //The rule is : if sync_src == MEM2PER, sync itf is channel's dest_txif, if sync_src == PER2MEM, perid is in src_rxif
  return (dsync == XDMAC_CC_DSYNC_MEM2PER) ? channel->dest_txif : channel->src_rxif;
}
//****************************************************

/**@}*/
