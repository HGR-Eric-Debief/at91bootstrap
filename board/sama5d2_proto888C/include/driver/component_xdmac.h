/* ---------------------------------------------------------------------------- */
/*                  Atmel Microcontroller Software Support                      */
/*                       SAM Software Package License                           */
/* ---------------------------------------------------------------------------- */
/* Copyright (c) 2015, Atmel Corporation                                        */
/*                                                                              */
/* All rights reserved.                                                         */
/*                                                                              */
/* Redistribution and use in source and binary forms, with or without           */
/* modification, are permitted provided that the following condition is met:    */
/*                                                                              */
/* - Redistributions of source code must retain the above copyright notice,     */
/* this list of conditions and the disclaimer below.                            */
/*                                                                              */
/* Atmel's name may not be used to endorse or promote products derived from     */
/* this software without specific prior written permission.                     */
/*                                                                              */
/* DISCLAIMER:  THIS SOFTWARE IS PROVIDED BY ATMEL "AS IS" AND ANY EXPRESS OR   */
/* IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF */
/* MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT ARE   */
/* DISCLAIMED. IN NO EVENT SHALL ATMEL BE LIABLE FOR ANY DIRECT, INDIRECT,      */
/* INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT */
/* LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,  */
/* OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF    */
/* LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING         */
/* NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, */
/* EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.                           */
/* ---------------------------------------------------------------------------- */

#ifndef _SAMA5D2_XDMAC_COMPONENT_
#define _SAMA5D2_XDMAC_COMPONENT_

#include "hw_types.h"
/* ============================================================================= */
/**  SOFTWARE API DEFINITION FOR Extensible DMA Controller */
/* ============================================================================= */
/** \addtogroup SAMA5D2_XDMAC Extensible DMA Controller */
/*@{*/

#if !(defined(__ASSEMBLY__) || defined(RoRegAR_SYSTEMS_ASM__))
/** \brief XdmacChid hardware registers */
typedef struct {
  WoReg  XDMAC_CIE;     /**< \brief (XdmacChid Offset: 0x0) Channel Interrupt Enable Register */
  WoReg  XDMAC_CID;     /**< \brief (XdmacChid Offset: 0x4) Channel Interrupt Disable Register */
  RoReg  XDMAC_CIM;     /**< \brief (XdmacChid Offset: 0x8) Channel Interrupt Mask Register */
  RoReg  XDMAC_CIS;     /**< \brief (XdmacChid Offset: 0xC) Channel Interrupt Status Register */
  RwReg  XDMAC_CSA;     /**< \brief (XdmacChid Offset: 0x10) Channel Source Address Register */
  RwReg  XDMAC_CDA;     /**< \brief (XdmacChid Offset: 0x14) Channel Destination Address Register */
  RwReg  XDMAC_CNDA;    /**< \brief (XdmacChid Offset: 0x18) Channel Next Descriptor Address Register */
  RwReg  XDMAC_CNDC;    /**< \brief (XdmacChid Offset: 0x1C) Channel Next Descriptor Control Register */
  RwReg  XDMAC_CUBC;    /**< \brief (XdmacChid Offset: 0x20) Channel Microblock Control Register */
  RwReg  XDMAC_CBC;     /**< \brief (XdmacChid Offset: 0x24) Channel Block Control Register */
  RwReg  XDMAC_CC;      /**< \brief (XdmacChid Offset: 0x28) Channel Configuration Register */
  RwReg  XDMAC_CDS_MSP; /**< \brief (XdmacChid Offset: 0x2C) Channel Data Stride Memory Set Pattern */
  RwReg  XDMAC_CSUS;    /**< \brief (XdmacChid Offset: 0x30) Channel Source Microblock Stride */
  RwReg  XDMAC_CDUS;    /**< \brief (XdmacChid Offset: 0x34) Channel Destination Microblock Stride */
  RoReg  Reserved1[2];
} XdmacChid;
/** \brief Xdmac hardware registers */
#define XDMACCHID_NUMBER 16
typedef struct {
  RwReg  XDMAC_GTYPE;                  /**< \brief (Xdmac Offset: 0x00) Global Type Register */
  RoReg  XDMAC_GCFG;                   /**< \brief (Xdmac Offset: 0x04) Global Configuration Register */
  RwReg  XDMAC_GWAC;                   /**< \brief (Xdmac Offset: 0x08) Global Weighted Arbiter Configuration Register */
  WoReg  XDMAC_GIE;                    /**< \brief (Xdmac Offset: 0x0C) Global Interrupt Enable Register */
  WoReg  XDMAC_GID;                    /**< \brief (Xdmac Offset: 0x10) Global Interrupt Disable Register */
  RoReg  XDMAC_GIM;                    /**< \brief (Xdmac Offset: 0x14) Global Interrupt Mask Register */
  RoReg  XDMAC_GIS;                    /**< \brief (Xdmac Offset: 0x18) Global Interrupt Status Register */
  WoReg  XDMAC_GE;                     /**< \brief (Xdmac Offset: 0x1C) Global Channel Enable Register */
  WoReg  XDMAC_GD;                     /**< \brief (Xdmac Offset: 0x20) Global Channel Disable Register */
  RoReg  XDMAC_GS;                     /**< \brief (Xdmac Offset: 0x24) Global Channel Status Register */
  RwReg  XDMAC_GRS;                    /**< \brief (Xdmac Offset: 0x28) Global Channel Read Suspend Register */
  RwReg  XDMAC_GWS;                    /**< \brief (Xdmac Offset: 0x2C) Global Channel Write Suspend Register */
  WoReg  XDMAC_GRWS;                   /**< \brief (Xdmac Offset: 0x30) Global Channel Read Write Suspend Register */
  WoReg  XDMAC_GRWR;                   /**< \brief (Xdmac Offset: 0x34) Global Channel Read Write Resume Register */
  WoReg  XDMAC_GSWR;                   /**< \brief (Xdmac Offset: 0x38) Global Channel Software Request Register */
  RoReg  XDMAC_GSWS;                   /**< \brief (Xdmac Offset: 0x3C) Global Channel Software Request Status Register */
  WoReg  XDMAC_GSWF;                   /**< \brief (Xdmac Offset: 0x40) Global Channel Software Flush Request Register */
  RoReg  Reserved1[3];
       XdmacChid XDMAC_CHID[XDMACCHID_NUMBER]; /**< \brief (Xdmac Offset: 0x50) chid = 0 .. 15 */
  RoReg    Reserved2[747];
  RwReg   XDMAC_VERSION;                /**< \brief (Xdmac Offset: 0xFFC) XDMAC Version Register */
} Xdmac;
#endif /* !(defined(__ASSEMBLY__) || defined(RoRegAR_SYSTEMS_ASM__)) */
/* -------- XDMAC_GTYPE : (XDMAC Offset: 0x00) Global Type Register -------- */
#define XDMAC_GTYPE_NB_CH_Pos 0
#define XDMAC_GTYPE_NB_CH_Msk (0x1fu << XDMAC_GTYPE_NB_CH_Pos) /**< \brief (XDMAC_GTYPE) Number of Channels Minus One */
#define XDMAC_GTYPE_NB_CH(value) ((XDMAC_GTYPE_NB_CH_Msk & ((value) << XDMAC_GTYPE_NB_CH_Pos)))
#define XDMAC_GTYPE_FIFO_SZ_Pos 5
#define XDMAC_GTYPE_FIFO_SZ_Msk (0x7ffu << XDMAC_GTYPE_FIFO_SZ_Pos) /**< \brief (XDMAC_GTYPE) Number of Bytes */
#define XDMAC_GTYPE_FIFO_SZ(value) ((XDMAC_GTYPE_FIFO_SZ_Msk & ((value) << XDMAC_GTYPE_FIFO_SZ_Pos)))
#define XDMAC_GTYPE_NB_REQ_Pos 16
#define XDMAC_GTYPE_NB_REQ_Msk (0x7fu << XDMAC_GTYPE_NB_REQ_Pos) /**< \brief (XDMAC_GTYPE) Number of Peripheral Requests Minus One */
#define XDMAC_GTYPE_NB_REQ(value) ((XDMAC_GTYPE_NB_REQ_Msk & ((value) << XDMAC_GTYPE_NB_REQ_Pos)))
/* -------- XDMAC_GCFG : (XDMAC Offset: 0x04) Global Configuration Register -------- */
#define XDMAC_GCFG_CGDISREG (0x1u << 0) /**< \brief (XDMAC_GCFG) Configuration Registers Clock Gating Disable */
#define XDMAC_GCFG_CGDISPIPE (0x1u << 1) /**< \brief (XDMAC_GCFG) Pipeline Clock Gating Disable */
#define XDMAC_GCFG_CGDISFIFO (0x1u << 2) /**< \brief (XDMAC_GCFG) FIFO Clock Gating Disable */
#define XDMAC_GCFG_CGDISIF (0x1u << 3) /**< \brief (XDMAC_GCFG) Bus Interface Clock Gating Disable */
#define XDMAC_GCFG_BXKBEN (0x1u << 8) /**< \brief (XDMAC_GCFG) Boundary X Kilobyte Enable */
/* -------- XDMAC_GWAC : (XDMAC Offset: 0x08) Global Weighted Arbiter Configuration Register -------- */
#define XDMAC_GWAC_PW0_Pos 0
#define XDMAC_GWAC_PW0_Msk (0xfu << XDMAC_GWAC_PW0_Pos) /**< \brief (XDMAC_GWAC) Pool Weight 0 */
#define XDMAC_GWAC_PW0(value) ((XDMAC_GWAC_PW0_Msk & ((value) << XDMAC_GWAC_PW0_Pos)))
#define XDMAC_GWAC_PW1_Pos 4
#define XDMAC_GWAC_PW1_Msk (0xfu << XDMAC_GWAC_PW1_Pos) /**< \brief (XDMAC_GWAC) Pool Weight 1 */
#define XDMAC_GWAC_PW1(value) ((XDMAC_GWAC_PW1_Msk & ((value) << XDMAC_GWAC_PW1_Pos)))
#define XDMAC_GWAC_PW2_Pos 8
#define XDMAC_GWAC_PW2_Msk (0xfu << XDMAC_GWAC_PW2_Pos) /**< \brief (XDMAC_GWAC) Pool Weight 2 */
#define XDMAC_GWAC_PW2(value) ((XDMAC_GWAC_PW2_Msk & ((value) << XDMAC_GWAC_PW2_Pos)))
#define XDMAC_GWAC_PW3_Pos 12
#define XDMAC_GWAC_PW3_Msk (0xfu << XDMAC_GWAC_PW3_Pos) /**< \brief (XDMAC_GWAC) Pool Weight 3 */
#define XDMAC_GWAC_PW3(value) ((XDMAC_GWAC_PW3_Msk & ((value) << XDMAC_GWAC_PW3_Pos)))
/* -------- XDMAC_GIE : (XDMAC Offset: 0x0C) Global Interrupt Enable Register -------- */
#define XDMAC_GIE_IE0 (0x1u << 0) /**< \brief (XDMAC_GIE) XDMAC Channel 0 Interrupt Enable Bit */
#define XDMAC_GIE_IE1 (0x1u << 1) /**< \brief (XDMAC_GIE) XDMAC Channel 1 Interrupt Enable Bit */
#define XDMAC_GIE_IE4 (0x1u << 4) /**< \brief (XDMAC_GIE) XDMAC Channel 4 Interrupt Enable Bit */
#define XDMAC_GIE_IE5 (0x1u << 5) /**< \brief (XDMAC_GIE) XDMAC Channel 5 Interrupt Enable Bit */
#define XDMAC_GIE_IE6 (0x1u << 6) /**< \brief (XDMAC_GIE) XDMAC Channel 6 Interrupt Enable Bit */
#define XDMAC_GIE_IE7 (0x1u << 7) /**< \brief (XDMAC_GIE) XDMAC Channel 7 Interrupt Enable Bit */
#define XDMAC_GIE_IE8 (0x1u << 8) /**< \brief (XDMAC_GIE) XDMAC Channel 8 Interrupt Enable Bit */
#define XDMAC_GIE_IE9 (0x1u << 9) /**< \brief (XDMAC_GIE) XDMAC Channel 9 Interrupt Enable Bit */
#define XDMAC_GIE_IE10 (0x1u << 10) /**< \brief (XDMAC_GIE) XDMAC Channel 10 Interrupt Enable Bit */
#define XDMAC_GIE_IE11 (0x1u << 11) /**< \brief (XDMAC_GIE) XDMAC Channel 11 Interrupt Enable Bit */
#define XDMAC_GIE_IE12 (0x1u << 12) /**< \brief (XDMAC_GIE) XDMAC Channel 12 Interrupt Enable Bit */
#define XDMAC_GIE_IE13 (0x1u << 13) /**< \brief (XDMAC_GIE) XDMAC Channel 13 Interrupt Enable Bit */
#define XDMAC_GIE_IE14 (0x1u << 14) /**< \brief (XDMAC_GIE) XDMAC Channel 14 Interrupt Enable Bit */
#define XDMAC_GIE_IE15 (0x1u << 15) /**< \brief (XDMAC_GIE) XDMAC Channel 15 Interrupt Enable Bit */
/* -------- XDMAC_GID : (XDMAC Offset: 0x10) Global Interrupt Disable Register -------- */
#define XDMAC_GID_ID0 (0x1u << 0) /**< \brief (XDMAC_GID) XDMAC Channel 0 Interrupt Disable Bit */
#define XDMAC_GID_ID1 (0x1u << 1) /**< \brief (XDMAC_GID) XDMAC Channel 1 Interrupt Disable Bit */
#define XDMAC_GID_ID4 (0x1u << 4) /**< \brief (XDMAC_GID) XDMAC Channel 4 Interrupt Disable Bit */
#define XDMAC_GID_ID5 (0x1u << 5) /**< \brief (XDMAC_GID) XDMAC Channel 5 Interrupt Disable Bit */
#define XDMAC_GID_ID6 (0x1u << 6) /**< \brief (XDMAC_GID) XDMAC Channel 6 Interrupt Disable Bit */
#define XDMAC_GID_ID7 (0x1u << 7) /**< \brief (XDMAC_GID) XDMAC Channel 7 Interrupt Disable Bit */
#define XDMAC_GID_ID8 (0x1u << 8) /**< \brief (XDMAC_GID) XDMAC Channel 8 Interrupt Disable Bit */
#define XDMAC_GID_ID9 (0x1u << 9) /**< \brief (XDMAC_GID) XDMAC Channel 9 Interrupt Disable Bit */
#define XDMAC_GID_ID10 (0x1u << 10) /**< \brief (XDMAC_GID) XDMAC Channel 10 Interrupt Disable Bit */
#define XDMAC_GID_ID11 (0x1u << 11) /**< \brief (XDMAC_GID) XDMAC Channel 11 Interrupt Disable Bit */
#define XDMAC_GID_ID12 (0x1u << 12) /**< \brief (XDMAC_GID) XDMAC Channel 12 Interrupt Disable Bit */
#define XDMAC_GID_ID13 (0x1u << 13) /**< \brief (XDMAC_GID) XDMAC Channel 13 Interrupt Disable Bit */
#define XDMAC_GID_ID14 (0x1u << 14) /**< \brief (XDMAC_GID) XDMAC Channel 14 Interrupt Disable Bit */
#define XDMAC_GID_ID15 (0x1u << 15) /**< \brief (XDMAC_GID) XDMAC Channel 15 Interrupt Disable Bit */
/* -------- XDMAC_GIM : (XDMAC Offset: 0x14) Global Interrupt Mask Register -------- */
#define XDMAC_GIM_IM0 (0x1u << 0) /**< \brief (XDMAC_GIM) XDMAC Channel 0 Interrupt Mask Bit */
#define XDMAC_GIM_IM1 (0x1u << 1) /**< \brief (XDMAC_GIM) XDMAC Channel 1 Interrupt Mask Bit */
#define XDMAC_GIM_IM4 (0x1u << 4) /**< \brief (XDMAC_GIM) XDMAC Channel 4 Interrupt Mask Bit */
#define XDMAC_GIM_IM5 (0x1u << 5) /**< \brief (XDMAC_GIM) XDMAC Channel 5 Interrupt Mask Bit */
#define XDMAC_GIM_IM6 (0x1u << 6) /**< \brief (XDMAC_GIM) XDMAC Channel 6 Interrupt Mask Bit */
#define XDMAC_GIM_IM7 (0x1u << 7) /**< \brief (XDMAC_GIM) XDMAC Channel 7 Interrupt Mask Bit */
#define XDMAC_GIM_IM8 (0x1u << 8) /**< \brief (XDMAC_GIM) XDMAC Channel 8 Interrupt Mask Bit */
#define XDMAC_GIM_IM9 (0x1u << 9) /**< \brief (XDMAC_GIM) XDMAC Channel 9 Interrupt Mask Bit */
#define XDMAC_GIM_IM10 (0x1u << 10) /**< \brief (XDMAC_GIM) XDMAC Channel 10 Interrupt Mask Bit */
#define XDMAC_GIM_IM11 (0x1u << 11) /**< \brief (XDMAC_GIM) XDMAC Channel 11 Interrupt Mask Bit */
#define XDMAC_GIM_IM12 (0x1u << 12) /**< \brief (XDMAC_GIM) XDMAC Channel 12 Interrupt Mask Bit */
#define XDMAC_GIM_IM13 (0x1u << 13) /**< \brief (XDMAC_GIM) XDMAC Channel 13 Interrupt Mask Bit */
#define XDMAC_GIM_IM14 (0x1u << 14) /**< \brief (XDMAC_GIM) XDMAC Channel 14 Interrupt Mask Bit */
#define XDMAC_GIM_IM15 (0x1u << 15) /**< \brief (XDMAC_GIM) XDMAC Channel 15 Interrupt Mask Bit */
/* -------- XDMAC_GIS : (XDMAC Offset: 0x18) Global Interrupt Status Register -------- */
#define XDMAC_GIS_IS0 (0x1u << 0) /**< \brief (XDMAC_GIS) XDMAC Channel 0 Interrupt Status Bit */
#define XDMAC_GIS_IS1 (0x1u << 1) /**< \brief (XDMAC_GIS) XDMAC Channel 1 Interrupt Status Bit */
#define XDMAC_GIS_IS4 (0x1u << 4) /**< \brief (XDMAC_GIS) XDMAC Channel 4 Interrupt Status Bit */
#define XDMAC_GIS_IS5 (0x1u << 5) /**< \brief (XDMAC_GIS) XDMAC Channel 5 Interrupt Status Bit */
#define XDMAC_GIS_IS6 (0x1u << 6) /**< \brief (XDMAC_GIS) XDMAC Channel 6 Interrupt Status Bit */
#define XDMAC_GIS_IS7 (0x1u << 7) /**< \brief (XDMAC_GIS) XDMAC Channel 7 Interrupt Status Bit */
#define XDMAC_GIS_IS8 (0x1u << 8) /**< \brief (XDMAC_GIS) XDMAC Channel 8 Interrupt Status Bit */
#define XDMAC_GIS_IS9 (0x1u << 9) /**< \brief (XDMAC_GIS) XDMAC Channel 9 Interrupt Status Bit */
#define XDMAC_GIS_IS10 (0x1u << 10) /**< \brief (XDMAC_GIS) XDMAC Channel 10 Interrupt Status Bit */
#define XDMAC_GIS_IS11 (0x1u << 11) /**< \brief (XDMAC_GIS) XDMAC Channel 11 Interrupt Status Bit */
#define XDMAC_GIS_IS12 (0x1u << 12) /**< \brief (XDMAC_GIS) XDMAC Channel 12 Interrupt Status Bit */
#define XDMAC_GIS_IS13 (0x1u << 13) /**< \brief (XDMAC_GIS) XDMAC Channel 13 Interrupt Status Bit */
#define XDMAC_GIS_IS14 (0x1u << 14) /**< \brief (XDMAC_GIS) XDMAC Channel 14 Interrupt Status Bit */
#define XDMAC_GIS_IS15 (0x1u << 15) /**< \brief (XDMAC_GIS) XDMAC Channel 15 Interrupt Status Bit */
/* -------- XDMAC_GE : (XDMAC Offset: 0x1C) Global Channel Enable Register -------- */
#define XDMAC_GE_EN0 (0x1u << 0) /**< \brief (XDMAC_GE) XDMAC Channel 0 Enable Bit */
#define XDMAC_GE_EN1 (0x1u << 1) /**< \brief (XDMAC_GE) XDMAC Channel 1 Enable Bit */
#define XDMAC_GE_EN4 (0x1u << 4) /**< \brief (XDMAC_GE) XDMAC Channel 4 Enable Bit */
#define XDMAC_GE_EN5 (0x1u << 5) /**< \brief (XDMAC_GE) XDMAC Channel 5 Enable Bit */
#define XDMAC_GE_EN6 (0x1u << 6) /**< \brief (XDMAC_GE) XDMAC Channel 6 Enable Bit */
#define XDMAC_GE_EN7 (0x1u << 7) /**< \brief (XDMAC_GE) XDMAC Channel 7 Enable Bit */
#define XDMAC_GE_EN8 (0x1u << 8) /**< \brief (XDMAC_GE) XDMAC Channel 8 Enable Bit */
#define XDMAC_GE_EN9 (0x1u << 9) /**< \brief (XDMAC_GE) XDMAC Channel 9 Enable Bit */
#define XDMAC_GE_EN10 (0x1u << 10) /**< \brief (XDMAC_GE) XDMAC Channel 10 Enable Bit */
#define XDMAC_GE_EN11 (0x1u << 11) /**< \brief (XDMAC_GE) XDMAC Channel 11 Enable Bit */
#define XDMAC_GE_EN12 (0x1u << 12) /**< \brief (XDMAC_GE) XDMAC Channel 12 Enable Bit */
#define XDMAC_GE_EN13 (0x1u << 13) /**< \brief (XDMAC_GE) XDMAC Channel 13 Enable Bit */
#define XDMAC_GE_EN14 (0x1u << 14) /**< \brief (XDMAC_GE) XDMAC Channel 14 Enable Bit */
#define XDMAC_GE_EN15 (0x1u << 15) /**< \brief (XDMAC_GE) XDMAC Channel 15 Enable Bit */
/* -------- XDMAC_GD : (XDMAC Offset: 0x20) Global Channel Disable Register -------- */
#define XDMAC_GD_DI0 (0x1u << 0) /**< \brief (XDMAC_GD) XDMAC Channel 0 Disable Bit */
#define XDMAC_GD_DI1 (0x1u << 1) /**< \brief (XDMAC_GD) XDMAC Channel 1 Disable Bit */
#define XDMAC_GD_DI4 (0x1u << 4) /**< \brief (XDMAC_GD) XDMAC Channel 4 Disable Bit */
#define XDMAC_GD_DI5 (0x1u << 5) /**< \brief (XDMAC_GD) XDMAC Channel 5 Disable Bit */
#define XDMAC_GD_DI6 (0x1u << 6) /**< \brief (XDMAC_GD) XDMAC Channel 6 Disable Bit */
#define XDMAC_GD_DI7 (0x1u << 7) /**< \brief (XDMAC_GD) XDMAC Channel 7 Disable Bit */
#define XDMAC_GD_DI8 (0x1u << 8) /**< \brief (XDMAC_GD) XDMAC Channel 8 Disable Bit */
#define XDMAC_GD_DI9 (0x1u << 9) /**< \brief (XDMAC_GD) XDMAC Channel 9 Disable Bit */
#define XDMAC_GD_DI10 (0x1u << 10) /**< \brief (XDMAC_GD) XDMAC Channel 10 Disable Bit */
#define XDMAC_GD_DI11 (0x1u << 11) /**< \brief (XDMAC_GD) XDMAC Channel 11 Disable Bit */
#define XDMAC_GD_DI12 (0x1u << 12) /**< \brief (XDMAC_GD) XDMAC Channel 12 Disable Bit */
#define XDMAC_GD_DI13 (0x1u << 13) /**< \brief (XDMAC_GD) XDMAC Channel 13 Disable Bit */
#define XDMAC_GD_DI14 (0x1u << 14) /**< \brief (XDMAC_GD) XDMAC Channel 14 Disable Bit */
#define XDMAC_GD_DI15 (0x1u << 15) /**< \brief (XDMAC_GD) XDMAC Channel 15 Disable Bit */
/* -------- XDMAC_GS : (XDMAC Offset: 0x24) Global Channel Status Register -------- */
#define XDMAC_GS_ST0 (0x1u << 0) /**< \brief (XDMAC_GS) XDMAC Channel 0 Status Bit */
#define XDMAC_GS_ST1 (0x1u << 1) /**< \brief (XDMAC_GS) XDMAC Channel 1 Status Bit */
#define XDMAC_GS_ST4 (0x1u << 4) /**< \brief (XDMAC_GS) XDMAC Channel 4 Status Bit */
#define XDMAC_GS_ST5 (0x1u << 5) /**< \brief (XDMAC_GS) XDMAC Channel 5 Status Bit */
#define XDMAC_GS_ST6 (0x1u << 6) /**< \brief (XDMAC_GS) XDMAC Channel 6 Status Bit */
#define XDMAC_GS_ST7 (0x1u << 7) /**< \brief (XDMAC_GS) XDMAC Channel 7 Status Bit */
#define XDMAC_GS_ST8 (0x1u << 8) /**< \brief (XDMAC_GS) XDMAC Channel 8 Status Bit */
#define XDMAC_GS_ST9 (0x1u << 9) /**< \brief (XDMAC_GS) XDMAC Channel 9 Status Bit */
#define XDMAC_GS_ST10 (0x1u << 10) /**< \brief (XDMAC_GS) XDMAC Channel 10 Status Bit */
#define XDMAC_GS_ST11 (0x1u << 11) /**< \brief (XDMAC_GS) XDMAC Channel 11 Status Bit */
#define XDMAC_GS_ST12 (0x1u << 12) /**< \brief (XDMAC_GS) XDMAC Channel 12 Status Bit */
#define XDMAC_GS_ST13 (0x1u << 13) /**< \brief (XDMAC_GS) XDMAC Channel 13 Status Bit */
#define XDMAC_GS_ST14 (0x1u << 14) /**< \brief (XDMAC_GS) XDMAC Channel 14 Status Bit */
#define XDMAC_GS_ST15 (0x1u << 15) /**< \brief (XDMAC_GS) XDMAC Channel 15 Status Bit */
/* -------- XDMAC_GRS : (XDMAC Offset: 0x28) Global Channel Read Suspend Register -------- */
#define XDMAC_GRS_RS0 (0x1u << 0) /**< \brief (XDMAC_GRS) XDMAC Channel 0 Read Suspend Bit */
#define XDMAC_GRS_RS1 (0x1u << 1) /**< \brief (XDMAC_GRS) XDMAC Channel 1 Read Suspend Bit */
#define XDMAC_GRS_RS4 (0x1u << 4) /**< \brief (XDMAC_GRS) XDMAC Channel 4 Read Suspend Bit */
#define XDMAC_GRS_RS5 (0x1u << 5) /**< \brief (XDMAC_GRS) XDMAC Channel 5 Read Suspend Bit */
#define XDMAC_GRS_RS6 (0x1u << 6) /**< \brief (XDMAC_GRS) XDMAC Channel 6 Read Suspend Bit */
#define XDMAC_GRS_RS7 (0x1u << 7) /**< \brief (XDMAC_GRS) XDMAC Channel 7 Read Suspend Bit */
#define XDMAC_GRS_RS8 (0x1u << 8) /**< \brief (XDMAC_GRS) XDMAC Channel 8 Read Suspend Bit */
#define XDMAC_GRS_RS9 (0x1u << 9) /**< \brief (XDMAC_GRS) XDMAC Channel 9 Read Suspend Bit */
#define XDMAC_GRS_RS10 (0x1u << 10) /**< \brief (XDMAC_GRS) XDMAC Channel 10 Read Suspend Bit */
#define XDMAC_GRS_RS11 (0x1u << 11) /**< \brief (XDMAC_GRS) XDMAC Channel 11 Read Suspend Bit */
#define XDMAC_GRS_RS12 (0x1u << 12) /**< \brief (XDMAC_GRS) XDMAC Channel 12 Read Suspend Bit */
#define XDMAC_GRS_RS13 (0x1u << 13) /**< \brief (XDMAC_GRS) XDMAC Channel 13 Read Suspend Bit */
#define XDMAC_GRS_RS14 (0x1u << 14) /**< \brief (XDMAC_GRS) XDMAC Channel 14 Read Suspend Bit */
#define XDMAC_GRS_RS15 (0x1u << 15) /**< \brief (XDMAC_GRS) XDMAC Channel 15 Read Suspend Bit */
/* -------- XDMAC_GWS : (XDMAC Offset: 0x2C) Global Channel Write Suspend Register -------- */
#define XDMAC_GWS_WS0 (0x1u << 0) /**< \brief (XDMAC_GWS) XDMAC Channel 0 Write Suspend Bit */
#define XDMAC_GWS_WS1 (0x1u << 1) /**< \brief (XDMAC_GWS) XDMAC Channel 1 Write Suspend Bit */
#define XDMAC_GWS_WS4 (0x1u << 4) /**< \brief (XDMAC_GWS) XDMAC Channel 4 Write Suspend Bit */
#define XDMAC_GWS_WS5 (0x1u << 5) /**< \brief (XDMAC_GWS) XDMAC Channel 5 Write Suspend Bit */
#define XDMAC_GWS_WS6 (0x1u << 6) /**< \brief (XDMAC_GWS) XDMAC Channel 6 Write Suspend Bit */
#define XDMAC_GWS_WS7 (0x1u << 7) /**< \brief (XDMAC_GWS) XDMAC Channel 7 Write Suspend Bit */
#define XDMAC_GWS_WS8 (0x1u << 8) /**< \brief (XDMAC_GWS) XDMAC Channel 8 Write Suspend Bit */
#define XDMAC_GWS_WS9 (0x1u << 9) /**< \brief (XDMAC_GWS) XDMAC Channel 9 Write Suspend Bit */
#define XDMAC_GWS_WS10 (0x1u << 10) /**< \brief (XDMAC_GWS) XDMAC Channel 10 Write Suspend Bit */
#define XDMAC_GWS_WS11 (0x1u << 11) /**< \brief (XDMAC_GWS) XDMAC Channel 11 Write Suspend Bit */
#define XDMAC_GWS_WS12 (0x1u << 12) /**< \brief (XDMAC_GWS) XDMAC Channel 12 Write Suspend Bit */
#define XDMAC_GWS_WS13 (0x1u << 13) /**< \brief (XDMAC_GWS) XDMAC Channel 13 Write Suspend Bit */
#define XDMAC_GWS_WS14 (0x1u << 14) /**< \brief (XDMAC_GWS) XDMAC Channel 14 Write Suspend Bit */
#define XDMAC_GWS_WS15 (0x1u << 15) /**< \brief (XDMAC_GWS) XDMAC Channel 15 Write Suspend Bit */
/* -------- XDMAC_GRWS : (XDMAC Offset: 0x30) Global Channel Read Write Suspend Register -------- */
#define XDMAC_GRWS_RWS0 (0x1u << 0) /**< \brief (XDMAC_GRWS) XDMAC Channel 0 Read Write Suspend Bit */
#define XDMAC_GRWS_RWS1 (0x1u << 1) /**< \brief (XDMAC_GRWS) XDMAC Channel 1 Read Write Suspend Bit */
#define XDMAC_GRWS_RWS4 (0x1u << 4) /**< \brief (XDMAC_GRWS) XDMAC Channel 4 Read Write Suspend Bit */
#define XDMAC_GRWS_RWS5 (0x1u << 5) /**< \brief (XDMAC_GRWS) XDMAC Channel 5 Read Write Suspend Bit */
#define XDMAC_GRWS_RWS6 (0x1u << 6) /**< \brief (XDMAC_GRWS) XDMAC Channel 6 Read Write Suspend Bit */
#define XDMAC_GRWS_RWS7 (0x1u << 7) /**< \brief (XDMAC_GRWS) XDMAC Channel 7 Read Write Suspend Bit */
#define XDMAC_GRWS_RWS8 (0x1u << 8) /**< \brief (XDMAC_GRWS) XDMAC Channel 8 Read Write Suspend Bit */
#define XDMAC_GRWS_RWS9 (0x1u << 9) /**< \brief (XDMAC_GRWS) XDMAC Channel 9 Read Write Suspend Bit */
#define XDMAC_GRWS_RWS10 (0x1u << 10) /**< \brief (XDMAC_GRWS) XDMAC Channel 10 Read Write Suspend Bit */
#define XDMAC_GRWS_RWS11 (0x1u << 11) /**< \brief (XDMAC_GRWS) XDMAC Channel 11 Read Write Suspend Bit */
#define XDMAC_GRWS_RWS12 (0x1u << 12) /**< \brief (XDMAC_GRWS) XDMAC Channel 12 Read Write Suspend Bit */
#define XDMAC_GRWS_RWS13 (0x1u << 13) /**< \brief (XDMAC_GRWS) XDMAC Channel 13 Read Write Suspend Bit */
#define XDMAC_GRWS_RWS14 (0x1u << 14) /**< \brief (XDMAC_GRWS) XDMAC Channel 14 Read Write Suspend Bit */
#define XDMAC_GRWS_RWS15 (0x1u << 15) /**< \brief (XDMAC_GRWS) XDMAC Channel 15 Read Write Suspend Bit */
/* -------- XDMAC_GRWR : (XDMAC Offset: 0x34) Global Channel Read Write Resume Register -------- */
#define XDMAC_GRWR_RWR0 (0x1u << 0) /**< \brief (XDMAC_GRWR) XDMAC Channel 0 Read Write Resume Bit */
#define XDMAC_GRWR_RWR1 (0x1u << 1) /**< \brief (XDMAC_GRWR) XDMAC Channel 1 Read Write Resume Bit */
#define XDMAC_GRWR_RWR4 (0x1u << 4) /**< \brief (XDMAC_GRWR) XDMAC Channel 4 Read Write Resume Bit */
#define XDMAC_GRWR_RWR5 (0x1u << 5) /**< \brief (XDMAC_GRWR) XDMAC Channel 5 Read Write Resume Bit */
#define XDMAC_GRWR_RWR6 (0x1u << 6) /**< \brief (XDMAC_GRWR) XDMAC Channel 6 Read Write Resume Bit */
#define XDMAC_GRWR_RWR7 (0x1u << 7) /**< \brief (XDMAC_GRWR) XDMAC Channel 7 Read Write Resume Bit */
#define XDMAC_GRWR_RWR8 (0x1u << 8) /**< \brief (XDMAC_GRWR) XDMAC Channel 8 Read Write Resume Bit */
#define XDMAC_GRWR_RWR9 (0x1u << 9) /**< \brief (XDMAC_GRWR) XDMAC Channel 9 Read Write Resume Bit */
#define XDMAC_GRWR_RWR10 (0x1u << 10) /**< \brief (XDMAC_GRWR) XDMAC Channel 10 Read Write Resume Bit */
#define XDMAC_GRWR_RWR11 (0x1u << 11) /**< \brief (XDMAC_GRWR) XDMAC Channel 11 Read Write Resume Bit */
#define XDMAC_GRWR_RWR12 (0x1u << 12) /**< \brief (XDMAC_GRWR) XDMAC Channel 12 Read Write Resume Bit */
#define XDMAC_GRWR_RWR13 (0x1u << 13) /**< \brief (XDMAC_GRWR) XDMAC Channel 13 Read Write Resume Bit */
#define XDMAC_GRWR_RWR14 (0x1u << 14) /**< \brief (XDMAC_GRWR) XDMAC Channel 14 Read Write Resume Bit */
#define XDMAC_GRWR_RWR15 (0x1u << 15) /**< \brief (XDMAC_GRWR) XDMAC Channel 15 Read Write Resume Bit */
/* -------- XDMAC_GSWR : (XDMAC Offset: 0x38) Global Channel Software Request Register -------- */
#define XDMAC_GSWR_SWREQ0 (0x1u << 0) /**< \brief (XDMAC_GSWR) XDMAC Channel 0 Software Request Bit */
#define XDMAC_GSWR_SWREQ1 (0x1u << 1) /**< \brief (XDMAC_GSWR) XDMAC Channel 1 Software Request Bit */
#define XDMAC_GSWR_SWREQ4 (0x1u << 4) /**< \brief (XDMAC_GSWR) XDMAC Channel 4 Software Request Bit */
#define XDMAC_GSWR_SWREQ5 (0x1u << 5) /**< \brief (XDMAC_GSWR) XDMAC Channel 5 Software Request Bit */
#define XDMAC_GSWR_SWREQ6 (0x1u << 6) /**< \brief (XDMAC_GSWR) XDMAC Channel 6 Software Request Bit */
#define XDMAC_GSWR_SWREQ7 (0x1u << 7) /**< \brief (XDMAC_GSWR) XDMAC Channel 7 Software Request Bit */
#define XDMAC_GSWR_SWREQ8 (0x1u << 8) /**< \brief (XDMAC_GSWR) XDMAC Channel 8 Software Request Bit */
#define XDMAC_GSWR_SWREQ9 (0x1u << 9) /**< \brief (XDMAC_GSWR) XDMAC Channel 9 Software Request Bit */
#define XDMAC_GSWR_SWREQ10 (0x1u << 10) /**< \brief (XDMAC_GSWR) XDMAC Channel 10 Software Request Bit */
#define XDMAC_GSWR_SWREQ11 (0x1u << 11) /**< \brief (XDMAC_GSWR) XDMAC Channel 11 Software Request Bit */
#define XDMAC_GSWR_SWREQ12 (0x1u << 12) /**< \brief (XDMAC_GSWR) XDMAC Channel 12 Software Request Bit */
#define XDMAC_GSWR_SWREQ13 (0x1u << 13) /**< \brief (XDMAC_GSWR) XDMAC Channel 13 Software Request Bit */
#define XDMAC_GSWR_SWREQ14 (0x1u << 14) /**< \brief (XDMAC_GSWR) XDMAC Channel 14 Software Request Bit */
#define XDMAC_GSWR_SWREQ15 (0x1u << 15) /**< \brief (XDMAC_GSWR) XDMAC Channel 15 Software Request Bit */
/* -------- XDMAC_GSWS : (XDMAC Offset: 0x3C) Global Channel Software Request Status Register -------- */
#define XDMAC_GSWS_SWRS0 (0x1u << 0) /**< \brief (XDMAC_GSWS) XDMAC Channel 0 Software Request Status Bit */
#define XDMAC_GSWS_SWRS1 (0x1u << 1) /**< \brief (XDMAC_GSWS) XDMAC Channel 1 Software Request Status Bit */
#define XDMAC_GSWS_SWRS4 (0x1u << 4) /**< \brief (XDMAC_GSWS) XDMAC Channel 4 Software Request Status Bit */
#define XDMAC_GSWS_SWRS5 (0x1u << 5) /**< \brief (XDMAC_GSWS) XDMAC Channel 5 Software Request Status Bit */
#define XDMAC_GSWS_SWRS6 (0x1u << 6) /**< \brief (XDMAC_GSWS) XDMAC Channel 6 Software Request Status Bit */
#define XDMAC_GSWS_SWRS7 (0x1u << 7) /**< \brief (XDMAC_GSWS) XDMAC Channel 7 Software Request Status Bit */
#define XDMAC_GSWS_SWRS8 (0x1u << 8) /**< \brief (XDMAC_GSWS) XDMAC Channel 8 Software Request Status Bit */
#define XDMAC_GSWS_SWRS9 (0x1u << 9) /**< \brief (XDMAC_GSWS) XDMAC Channel 9 Software Request Status Bit */
#define XDMAC_GSWS_SWRS10 (0x1u << 10) /**< \brief (XDMAC_GSWS) XDMAC Channel 10 Software Request Status Bit */
#define XDMAC_GSWS_SWRS11 (0x1u << 11) /**< \brief (XDMAC_GSWS) XDMAC Channel 11 Software Request Status Bit */
#define XDMAC_GSWS_SWRS12 (0x1u << 12) /**< \brief (XDMAC_GSWS) XDMAC Channel 12 Software Request Status Bit */
#define XDMAC_GSWS_SWRS13 (0x1u << 13) /**< \brief (XDMAC_GSWS) XDMAC Channel 13 Software Request Status Bit */
#define XDMAC_GSWS_SWRS14 (0x1u << 14) /**< \brief (XDMAC_GSWS) XDMAC Channel 14 Software Request Status Bit */
#define XDMAC_GSWS_SWRS15 (0x1u << 15) /**< \brief (XDMAC_GSWS) XDMAC Channel 15 Software Request Status Bit */
/* -------- XDMAC_GSWF : (XDMAC Offset: 0x40) Global Channel Software Flush Request Register -------- */
#define XDMAC_GSWF_SWF0 (0x1u << 0) /**< \brief (XDMAC_GSWF) XDMAC Channel 0 Software Flush Request Bit */
#define XDMAC_GSWF_SWF1 (0x1u << 1) /**< \brief (XDMAC_GSWF) XDMAC Channel 1 Software Flush Request Bit */
#define XDMAC_GSWF_SWF4 (0x1u << 4) /**< \brief (XDMAC_GSWF) XDMAC Channel 4 Software Flush Request Bit */
#define XDMAC_GSWF_SWF5 (0x1u << 5) /**< \brief (XDMAC_GSWF) XDMAC Channel 5 Software Flush Request Bit */
#define XDMAC_GSWF_SWF6 (0x1u << 6) /**< \brief (XDMAC_GSWF) XDMAC Channel 6 Software Flush Request Bit */
#define XDMAC_GSWF_SWF7 (0x1u << 7) /**< \brief (XDMAC_GSWF) XDMAC Channel 7 Software Flush Request Bit */
#define XDMAC_GSWF_SWF8 (0x1u << 8) /**< \brief (XDMAC_GSWF) XDMAC Channel 8 Software Flush Request Bit */
#define XDMAC_GSWF_SWF9 (0x1u << 9) /**< \brief (XDMAC_GSWF) XDMAC Channel 9 Software Flush Request Bit */
#define XDMAC_GSWF_SWF10 (0x1u << 10) /**< \brief (XDMAC_GSWF) XDMAC Channel 10 Software Flush Request Bit */
#define XDMAC_GSWF_SWF11 (0x1u << 11) /**< \brief (XDMAC_GSWF) XDMAC Channel 11 Software Flush Request Bit */
#define XDMAC_GSWF_SWF12 (0x1u << 12) /**< \brief (XDMAC_GSWF) XDMAC Channel 12 Software Flush Request Bit */
#define XDMAC_GSWF_SWF13 (0x1u << 13) /**< \brief (XDMAC_GSWF) XDMAC Channel 13 Software Flush Request Bit */
#define XDMAC_GSWF_SWF14 (0x1u << 14) /**< \brief (XDMAC_GSWF) XDMAC Channel 14 Software Flush Request Bit */
#define XDMAC_GSWF_SWF15 (0x1u << 15) /**< \brief (XDMAC_GSWF) XDMAC Channel 15 Software Flush Request Bit */
/* -------- XDMAC_CIE : (XDMAC Offset: N/A) Channel Interrupt Enable Register -------- */
#define XDMAC_CIE_BIE (0x1u << 0) /**< \brief (XDMAC_CIE) End of Block Interrupt Enable Bit */
#define XDMAC_CIE_LIE (0x1u << 1) /**< \brief (XDMAC_CIE) End of Linked List Interrupt Enable Bit */
#define XDMAC_CIE_DIE (0x1u << 2) /**< \brief (XDMAC_CIE) End of Disable Interrupt Enable Bit */
#define XDMAC_CIE_FIE (0x1u << 3) /**< \brief (XDMAC_CIE) End of Flush Interrupt Enable Bit */
#define XDMAC_CIE_RBIE (0x1u << 4) /**< \brief (XDMAC_CIE) Read Bus Error Interrupt Enable Bit */
#define XDMAC_CIE_WBIE (0x1u << 5) /**< \brief (XDMAC_CIE) Write Bus Error Interrupt Enable Bit */
#define XDMAC_CIE_ROIE (0x1u << 6) /**< \brief (XDMAC_CIE) Request Overflow Error Interrupt Enable Bit */
/* -------- XDMAC_CID : (XDMAC Offset: N/A) Channel Interrupt Disable Register -------- */
#define XDMAC_CID_BID (0x1u << 0) /**< \brief (XDMAC_CID) End of Block Interrupt Disable Bit */
#define XDMAC_CID_LID (0x1u << 1) /**< \brief (XDMAC_CID) End of Linked List Interrupt Disable Bit */
#define XDMAC_CID_DID (0x1u << 2) /**< \brief (XDMAC_CID) End of Disable Interrupt Disable Bit */
#define XDMAC_CID_FID (0x1u << 3) /**< \brief (XDMAC_CID) End of Flush Interrupt Disable Bit */
#define XDMAC_CID_RBEID (0x1u << 4) /**< \brief (XDMAC_CID) Read Bus Error Interrupt Disable Bit */
#define XDMAC_CID_WBEID (0x1u << 5) /**< \brief (XDMAC_CID) Write Bus Error Interrupt Disable Bit */
#define XDMAC_CID_ROID (0x1u << 6) /**< \brief (XDMAC_CID) Request Overflow Error Interrupt Disable Bit */
/* -------- XDMAC_CIM : (XDMAC Offset: N/A) Channel Interrupt Mask Register -------- */
#define XDMAC_CIM_BIM (0x1u << 0) /**< \brief (XDMAC_CIM) End of Block Interrupt Mask Bit */
#define XDMAC_CIM_LIM (0x1u << 1) /**< \brief (XDMAC_CIM) End of Linked List Interrupt Mask Bit */
#define XDMAC_CIM_DIM (0x1u << 2) /**< \brief (XDMAC_CIM) End of Disable Interrupt Mask Bit */
#define XDMAC_CIM_FIM (0x1u << 3) /**< \brief (XDMAC_CIM) End of Flush Interrupt Mask Bit */
#define XDMAC_CIM_RBEIM (0x1u << 4) /**< \brief (XDMAC_CIM) Read Bus Error Interrupt Mask Bit */
#define XDMAC_CIM_WBEIM (0x1u << 5) /**< \brief (XDMAC_CIM) Write Bus Error Interrupt Mask Bit */
#define XDMAC_CIM_ROIM (0x1u << 6) /**< \brief (XDMAC_CIM) Request Overflow Error Interrupt Mask Bit */
/* -------- XDMAC_CIS : (XDMAC Offset: N/A) Channel Interrupt Status Register -------- */
#define XDMAC_CIS_BIS (0x1u << 0) /**< \brief (XDMAC_CIS) End of Block Interrupt Status Bit */
#define XDMAC_CIS_LIS (0x1u << 1) /**< \brief (XDMAC_CIS) End of Linked List Interrupt Status Bit */
#define XDMAC_CIS_DIS (0x1u << 2) /**< \brief (XDMAC_CIS) End of Disable Interrupt Status Bit */
#define XDMAC_CIS_FIS (0x1u << 3) /**< \brief (XDMAC_CIS) End of Flush Interrupt Status Bit */
#define XDMAC_CIS_RBEIS (0x1u << 4) /**< \brief (XDMAC_CIS) Read Bus Error Interrupt Status Bit */
#define XDMAC_CIS_WBEIS (0x1u << 5) /**< \brief (XDMAC_CIS) Write Bus Error Interrupt Status Bit */
#define XDMAC_CIS_ROIS (0x1u << 6) /**< \brief (XDMAC_CIS) Request Overflow Error Interrupt Status Bit */
/* -------- XDMAC_CSA : (XDMAC Offset: N/A) Channel Source Address Register -------- */
#define XDMAC_CSA_SA_Pos 0
#define XDMAC_CSA_SA_Msk (0xffffffffu << XDMAC_CSA_SA_Pos) /**< \brief (XDMAC_CSA) Channel x Source Address */
#define XDMAC_CSA_SA(value) ((XDMAC_CSA_SA_Msk & ((value) << XDMAC_CSA_SA_Pos)))
/* -------- XDMAC_CDA : (XDMAC Offset: N/A) Channel Destination Address Register -------- */
#define XDMAC_CDA_DA_Pos 0
#define XDMAC_CDA_DA_Msk (0xffffffffu << XDMAC_CDA_DA_Pos) /**< \brief (XDMAC_CDA) Channel x Destination Address */
#define XDMAC_CDA_DA(value) ((XDMAC_CDA_DA_Msk & ((value) << XDMAC_CDA_DA_Pos)))
/* -------- XDMAC_CNDA : (XDMAC Offset: N/A) Channel Next Descriptor Address Register -------- */
#define XDMAC_CNDA_NDAIF (0x1u << 0) /**< \brief (XDMAC_CNDA) Channel x Next Descriptor Interface */
#define XDMAC_CNDA_NDA_Pos 2
#define XDMAC_CNDA_NDA_Msk (0x3fffffffu << XDMAC_CNDA_NDA_Pos) /**< \brief (XDMAC_CNDA) Channel x Next Descriptor Address */
#define XDMAC_CNDA_NDA(value) ((XDMAC_CNDA_NDA_Msk & ((value) << XDMAC_CNDA_NDA_Pos)))
/* -------- XDMAC_CNDC : (XDMAC Offset: N/A) Channel Next Descriptor Control Register -------- */
#define XDMAC_CNDC_NDE (0x1u << 0) /**< \brief (XDMAC_CNDC) Channel x Next Descriptor Enable */
#define   XDMAC_CNDC_NDE_DSCR_FETCH_DIS (0x0u << 0) /**< \brief (XDMAC_CNDC) Descriptor fetch is disabled. */
#define   XDMAC_CNDC_NDE_DSCR_FETCH_EN (0x1u << 0) /**< \brief (XDMAC_CNDC) Descriptor fetch is enabled. */
#define XDMAC_CNDC_NDSUP (0x1u << 1) /**< \brief (XDMAC_CNDC) Channel x Next Descriptor Source Update */
#define   XDMAC_CNDC_NDSUP_SRC_PARAMS_UNCHANGED (0x0u << 1) /**< \brief (XDMAC_CNDC) Source parameters remain unchanged. */
#define   XDMAC_CNDC_NDSUP_SRC_PARAMS_UPDATED (0x1u << 1) /**< \brief (XDMAC_CNDC) Source parameters are updated when the descriptor is retrieved. */
#define XDMAC_CNDC_NDDUP (0x1u << 2) /**< \brief (XDMAC_CNDC) Channel x Next Descriptor Destination Update */
#define   XDMAC_CNDC_NDDUP_DST_PARAMS_UNCHANGED (0x0u << 2) /**< \brief (XDMAC_CNDC) Destination parameters remain unchanged. */
#define   XDMAC_CNDC_NDDUP_DST_PARAMS_UPDATED (0x1u << 2) /**< \brief (XDMAC_CNDC) Destination parameters are updated when the descriptor is retrieved. */
#define XDMAC_CNDC_NDVIEW_Pos 3
#define XDMAC_CNDC_NDVIEW_Msk (0x3u << XDMAC_CNDC_NDVIEW_Pos) /**< \brief (XDMAC_CNDC) Channel x Next Descriptor View */
#define XDMAC_CNDC_NDVIEW(value) ((XDMAC_CNDC_NDVIEW_Msk & ((value) << XDMAC_CNDC_NDVIEW_Pos)))
#define   XDMAC_CNDC_NDVIEW_NDV0 (0x0u << 3) /**< \brief (XDMAC_CNDC) Next Descriptor View 0 */
#define   XDMAC_CNDC_NDVIEW_NDV1 (0x1u << 3) /**< \brief (XDMAC_CNDC) Next Descriptor View 1 */
#define   XDMAC_CNDC_NDVIEW_NDV2 (0x2u << 3) /**< \brief (XDMAC_CNDC) Next Descriptor View 2 */
#define   XDMAC_CNDC_NDVIEW_NDV3 (0x3u << 3) /**< \brief (XDMAC_CNDC) Next Descriptor View 3 */
/* -------- XDMAC_CUBC : (XDMAC Offset: N/A) Channel Microblock Control Register -------- */
#define XDMAC_CUBC_UBLEN_Pos 0
#define XDMAC_CUBC_UBLEN_Msk (0xffffffu << XDMAC_CUBC_UBLEN_Pos) /**< \brief (XDMAC_CUBC) Channel x Microblock Length */
#define XDMAC_CUBC_UBLEN(value) ((XDMAC_CUBC_UBLEN_Msk & ((value) << XDMAC_CUBC_UBLEN_Pos)))
/* -------- XDMAC_CBC : (XDMAC Offset: N/A) Channel Block Control Register -------- */
#define XDMAC_CBC_BLEN_Pos 0
#define XDMAC_CBC_BLEN_Msk (0xfffu << XDMAC_CBC_BLEN_Pos) /**< \brief (XDMAC_CBC) Channel x Block Length */
#define XDMAC_CBC_BLEN(value) ((XDMAC_CBC_BLEN_Msk & ((value) << XDMAC_CBC_BLEN_Pos)))
/* -------- XDMAC_CC : (XDMAC Offset: N/A) Channel Configuration Register -------- */
#define XDMAC_CC_TYPE (0x1u << 0) /**< \brief (XDMAC_CC) Channel x Transfer Type */
#define   XDMAC_CC_TYPE_MEM_TRAN (0x0u << 0) /**< \brief (XDMAC_CC) Self triggered mode (Memory to Memory Transfer). */
#define   XDMAC_CC_TYPE_PER_TRAN (0x1u << 0) /**< \brief (XDMAC_CC) Synchronized mode (Peripheral to Memory or Memory to Peripheral Transfer). */
#define XDMAC_CC_MBSIZE_Pos 1
#define XDMAC_CC_MBSIZE_Msk (0x3u << XDMAC_CC_MBSIZE_Pos) /**< \brief (XDMAC_CC) Channel x Memory Burst Size */
#define XDMAC_CC_MBSIZE(value) ((XDMAC_CC_MBSIZE_Msk & ((value) << XDMAC_CC_MBSIZE_Pos)))
#define   XDMAC_CC_MBSIZE_SINGLE (0x0u << 1) /**< \brief (XDMAC_CC) The memory burst size is set to one. */
#define   XDMAC_CC_MBSIZE_FOUR (0x1u << 1) /**< \brief (XDMAC_CC) The memory burst size is set to four. */
#define   XDMAC_CC_MBSIZE_EIGHT (0x2u << 1) /**< \brief (XDMAC_CC) The memory burst size is set to eight. */
#define   XDMAC_CC_MBSIZE_SIXTEEN (0x3u << 1) /**< \brief (XDMAC_CC) The memory burst size is set to sixteen. */
#define XDMAC_CC_DSYNC (0x1u << 4) /**< \brief (XDMAC_CC) Channel x Synchronization */
#define   XDMAC_CC_DSYNC_PER2MEM (0x0u << 4) /**< \brief (XDMAC_CC) Peripheral to Memory transfer. */
#define   XDMAC_CC_DSYNC_MEM2PER (0x1u << 4) /**< \brief (XDMAC_CC) Memory to Peripheral transfer. */
#define XDMAC_CC_PROT (0x1u << 5) /**< \brief (XDMAC_CC) Channel x Protection */
#define   XDMAC_CC_PROT_SEC (0x0u << 5) /**< \brief (XDMAC_CC) Channel is secured. */
#define   XDMAC_CC_PROT_UNSEC (0x1u << 5) /**< \brief (XDMAC_CC) Channel is unsecured. */
#define XDMAC_CC_SWREQ (0x1u << 6) /**< \brief (XDMAC_CC) Channel x Software Request Trigger */
#define   XDMAC_CC_SWREQ_HWR_CONNECTED (0x0u << 6) /**< \brief (XDMAC_CC) Hardware request line is connected to the peripheral request line. */
#define   XDMAC_CC_SWREQ_SWR_CONNECTED (0x1u << 6) /**< \brief (XDMAC_CC) Software request is connected to the peripheral request line. */
#define XDMAC_CC_MEMSET (0x1u << 7) /**< \brief (XDMAC_CC) Channel x Fill Block of memory */
#define   XDMAC_CC_MEMSET_NORMAL_MODE (0x0u << 7) /**< \brief (XDMAC_CC) Memset is not activated. */
#define   XDMAC_CC_MEMSET_HW_MODE (0x1u << 7) /**< \brief (XDMAC_CC) Sets the block of memory pointed by DA field to the specified value. This operation is performed on 8, 16 or 32 bits basis. */
#define XDMAC_CC_CSIZE_Pos 8
#define XDMAC_CC_CSIZE_Msk (0x7u << XDMAC_CC_CSIZE_Pos) /**< \brief (XDMAC_CC) Channel x Chunk Size */
#define XDMAC_CC_CSIZE(value) ((XDMAC_CC_CSIZE_Msk & ((value) << XDMAC_CC_CSIZE_Pos)))
#define   XDMAC_CC_CSIZE_CHK_1 (0x0u << 8) /**< \brief (XDMAC_CC) 1 data transferred */
#define   XDMAC_CC_CSIZE_CHK_2 (0x1u << 8) /**< \brief (XDMAC_CC) 2 data transferred */
#define   XDMAC_CC_CSIZE_CHK_4 (0x2u << 8) /**< \brief (XDMAC_CC) 4 data transferred */
#define   XDMAC_CC_CSIZE_CHK_8 (0x3u << 8) /**< \brief (XDMAC_CC) 8 data transferred */
#define   XDMAC_CC_CSIZE_CHK_16 (0x4u << 8) /**< \brief (XDMAC_CC) 16 data transferred */
#define XDMAC_CC_DWIDTH_Pos 11
#define XDMAC_CC_DWIDTH_Msk (0x3u << XDMAC_CC_DWIDTH_Pos) /**< \brief (XDMAC_CC) Channel x Data Width */
#define XDMAC_CC_DWIDTH(value) ((XDMAC_CC_DWIDTH_Msk & ((value) << XDMAC_CC_DWIDTH_Pos)))
#define   XDMAC_CC_DWIDTH_BYTE (0x0u << 11) /**< \brief (XDMAC_CC) The data size is set to 8 bits */
#define   XDMAC_CC_DWIDTH_HALFWORD (0x1u << 11) /**< \brief (XDMAC_CC) The data size is set to 16 bits */
#define   XDMAC_CC_DWIDTH_WORD (0x2u << 11) /**< \brief (XDMAC_CC) The data size is set to 32 bits */
#define   XDMAC_CC_DWIDTH_DWORD (0x3u << 11) /**< \brief (XDMAC_CC) The data size is set to 64 bits */
#define XDMAC_CC_SIF (0x1u << 13) /**< \brief (XDMAC_CC) Channel x Source Interface Identifier */
#define   XDMAC_CC_SIF_AHB_IF0 (0x0u << 13) /**< \brief (XDMAC_CC) The data is read through the system bus interface 0. */
#define   XDMAC_CC_SIF_AHB_IF1 (0x1u << 13) /**< \brief (XDMAC_CC) The data is read through the system bus interface 1. */
#define XDMAC_CC_DIF (0x1u << 14) /**< \brief (XDMAC_CC) Channel x Destination Interface Identifier */
#define   XDMAC_CC_DIF_AHB_IF0 (0x0u << 14) /**< \brief (XDMAC_CC) The data is written through the system bus interface 0. */
#define   XDMAC_CC_DIF_AHB_IF1 (0x1u << 14) /**< \brief (XDMAC_CC) The data is written though the system bus interface 1. */
#define XDMAC_CC_SAM_Pos 16
#define XDMAC_CC_SAM_Msk (0x3u << XDMAC_CC_SAM_Pos) /**< \brief (XDMAC_CC) Channel x Source Addressing Mode */
#define XDMAC_CC_SAM(value) ((XDMAC_CC_SAM_Msk & ((value) << XDMAC_CC_SAM_Pos)))
#define   XDMAC_CC_SAM_FIXED_AM (0x0u << 16) /**< \brief (XDMAC_CC) The address remains unchanged. */
#define   XDMAC_CC_SAM_INCREMENTED_AM (0x1u << 16) /**< \brief (XDMAC_CC) The addressing mode is incremented (the increment size is set to the data size). */
#define   XDMAC_CC_SAM_UBS_AM (0x2u << 16) /**< \brief (XDMAC_CC) The microblock stride is added at the microblock boundary. */
#define   XDMAC_CC_SAM_UBS_DS_AM (0x3u << 16) /**< \brief (XDMAC_CC) The microblock stride is added at the microblock boundary, the data stride is added at the data boundary. */
#define XDMAC_CC_DAM_Pos 18
#define XDMAC_CC_DAM_Msk (0x3u << XDMAC_CC_DAM_Pos) /**< \brief (XDMAC_CC) Channel x Destination Addressing Mode */
#define XDMAC_CC_DAM(value) ((XDMAC_CC_DAM_Msk & ((value) << XDMAC_CC_DAM_Pos)))
#define   XDMAC_CC_DAM_FIXED_AM (0x0u << 18) /**< \brief (XDMAC_CC) The address remains unchanged. */
#define   XDMAC_CC_DAM_INCREMENTED_AM (0x1u << 18) /**< \brief (XDMAC_CC) The addressing mode is incremented (the increment size is set to the data size). */
#define   XDMAC_CC_DAM_UBS_AM (0x2u << 18) /**< \brief (XDMAC_CC) The microblock stride is added at the microblock boundary. */
#define   XDMAC_CC_DAM_UBS_DS_AM (0x3u << 18) /**< \brief (XDMAC_CC) The microblock stride is added at the microblock boundary, the data stride is added at the data boundary. */
#define XDMAC_CC_INITD (0x1u << 21) /**< \brief (XDMAC_CC) Channel Initialization Terminated (this bit is read-only) */
#define   XDMAC_CC_INITD_TERMINATED (0x0u << 21) /**< \brief (XDMAC_CC) Channel initialization is in progress. */
#define   XDMAC_CC_INITD_IN_PROGRESS (0x1u << 21) /**< \brief (XDMAC_CC) Channel initialization is completed. */
#define XDMAC_CC_RDIP (0x1u << 22) /**< \brief (XDMAC_CC) Read in Progress (this bit is read-only) */
#define   XDMAC_CC_RDIP_DONE (0x0u << 22) /**< \brief (XDMAC_CC) No Active read transaction on the bus. */
#define   XDMAC_CC_RDIP_IN_PROGRESS (0x1u << 22) /**< \brief (XDMAC_CC) A read transaction is in progress. */
#define XDMAC_CC_WRIP (0x1u << 23) /**< \brief (XDMAC_CC) Write in Progress (this bit is read-only) */
#define   XDMAC_CC_WRIP_DONE (0x0u << 23) /**< \brief (XDMAC_CC) No Active write transaction on the bus. */
#define   XDMAC_CC_WRIP_IN_PROGRESS (0x1u << 23) /**< \brief (XDMAC_CC) A Write transaction is in progress. */
#define XDMAC_CC_PERID_Pos 24
#define XDMAC_CC_PERID_Msk (0x7fu << XDMAC_CC_PERID_Pos) /**< \brief (XDMAC_CC) Channel x Peripheral Identifier */
#define XDMAC_CC_PERID(value) ((XDMAC_CC_PERID_Msk & ((value) << XDMAC_CC_PERID_Pos)))
/* -------- XDMAC_CDS_MSP : (XDMAC Offset: N/A) Channel Data Stride Memory Set Pattern -------- */
#define XDMAC_CDS_MSP_SDS_MSP_Pos 0
#define XDMAC_CDS_MSP_SDS_MSP_Msk (0xffffu << XDMAC_CDS_MSP_SDS_MSP_Pos) /**< \brief (XDMAC_CDS_MSP) Channel x Source Data stride or Memory Set Pattern */
#define XDMAC_CDS_MSP_SDS_MSP(value) ((XDMAC_CDS_MSP_SDS_MSP_Msk & ((value) << XDMAC_CDS_MSP_SDS_MSP_Pos)))
#define XDMAC_CDS_MSP_DDS_MSP_Pos 16
#define XDMAC_CDS_MSP_DDS_MSP_Msk (0xffffu << XDMAC_CDS_MSP_DDS_MSP_Pos) /**< \brief (XDMAC_CDS_MSP) Channel x Destination Data Stride or Memory Set Pattern */
#define XDMAC_CDS_MSP_DDS_MSP(value) ((XDMAC_CDS_MSP_DDS_MSP_Msk & ((value) << XDMAC_CDS_MSP_DDS_MSP_Pos)))
/* -------- XDMAC_CSUS : (XDMAC Offset: N/A) Channel Source Microblock Stride -------- */
#define XDMAC_CSUS_SUBS_Pos 0
#define XDMAC_CSUS_SUBS_Msk (0xffffffu << XDMAC_CSUS_SUBS_Pos) /**< \brief (XDMAC_CSUS) Channel x Source Microblock Stride */
#define XDMAC_CSUS_SUBS(value) ((XDMAC_CSUS_SUBS_Msk & ((value) << XDMAC_CSUS_SUBS_Pos)))
/* -------- XDMAC_CDUS : (XDMAC Offset: N/A) Channel Destination Microblock Stride -------- */
#define XDMAC_CDUS_DUBS_Pos 0
#define XDMAC_CDUS_DUBS_Msk (0xffffffu << XDMAC_CDUS_DUBS_Pos) /**< \brief (XDMAC_CDUS) Channel x Destination Microblock Stride */
#define XDMAC_CDUS_DUBS(value) ((XDMAC_CDUS_DUBS_Msk & ((value) << XDMAC_CDUS_DUBS_Pos)))
/* -------- XDMAC_VERSION : (XDMAC Offset: 0xFFC) XDMAC Version Register -------- */
#define XDMAC_VERSION_VERSION_Pos 0
#define XDMAC_VERSION_VERSION_Msk (0xfffu << XDMAC_VERSION_VERSION_Pos) /**< \brief (XDMAC_VERSION) Version of the Hardware Module */
#define XDMAC_VERSION_VERSION(value) ((XDMAC_VERSION_VERSION_Msk & ((value) << XDMAC_VERSION_VERSION_Pos)))
#define XDMAC_VERSION_MFN_Pos 16
#define XDMAC_VERSION_MFN_Msk (0x7u << XDMAC_VERSION_MFN_Pos) /**< \brief (XDMAC_VERSION) Metal Fix Number */
#define XDMAC_VERSION_MFN(value) ((XDMAC_VERSION_MFN_Msk & ((value) << XDMAC_VERSION_MFN_Pos)))

/*@}*/


#endif /* _SAMA5D2_XDMAC_COMPONENT_ */
