/*
 * ksz8041.h
 *
 * KSZ8041NL ethernet phy driver. Inspired by lan8742.c and .h from STM.
 *
 *  Created on: May 14, 2021
 *      Author: Karel Hevessy
 */

#ifndef INC_KSZ8041_H_
#define INC_KSZ8041_H_

/**
  ******************************************************************************
  * @file    lan8742.h
  * @author  MCD Application Team
  * @brief   This file contains all the functions prototypes for the
  *          lan8742.c PHY driver.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2017 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */


/* Includes ------------------------------------------------------------------*/
#include <stdint.h>

/** @addtogroup BSP
  * @{
  */

/** @addtogroup Component
  * @{
  */

/** @defgroup LAN8742
  * @{
  */
/* Exported constants --------------------------------------------------------*/
/** @defgroup LAN8742_Exported_Constants LAN8742 Exported Constants
  * @{
  */

/* Uncomment if you want to use hardware reset signal for the PHY */
/* Default: perform only software reset */
//#define USE_HARDWARE_RESET


/** @defgroup LAN8742_Registers_Mapping LAN8742 Registers Mapping
  * @{
//  */
#define KSZ8041_BCR      ((uint16_t)0x0000U)
#define KSZ8041_BSR      ((uint16_t)0x0001U)
#define KSZ8041_PHYI1R   ((uint16_t)0x0002U)
#define KSZ8041_PHYI2R   ((uint16_t)0x0003U)
#define KSZ8041_ANAR     ((uint16_t)0x0004U)
#define KSZ8041_ANLPAR   ((uint16_t)0x0005U)
#define KSZ8041_ANER     ((uint16_t)0x0006U)
#define KSZ8041_ANNPTR   ((uint16_t)0x0007U)
#define KSZ8041_ANNPRR   ((uint16_t)0x0008U)
//#define LAN8742_MMDACR   ((uint16_t)0x000DU)
//#define LAN8742_MMDAADR  ((uint16_t)0x000EU)
//#define LAN8742_ENCTR    ((uint16_t)0x0010U)
//#define LAN8742_MCSR     ((uint16_t)0x0011U)
//#define LAN8742_SMR      ((uint16_t)0x0012U)
//#define LAN8742_TPDCR    ((uint16_t)0x0018U)
//#define LAN8742_TCSR     ((uint16_t)0x0019U)
//#define LAN8742_SECR     ((uint16_t)0x001AU)
//#define LAN8742_SCSIR    ((uint16_t)0x001BU)
//#define LAN8742_CLR      ((uint16_t)0x001CU)
//#define LAN8742_ISFR     ((uint16_t)0x001DU)
//#define LAN8742_IMR      ((uint16_t)0x001EU)
#define KSZ8041_ICSR     ((uint16_t)0x001BU)
#define KSZ8041_CTRL2    ((uint16_t)0x001FU)
/**
  * @}
  */

/** @defgroup LAN8742_BCR_Bit_Definition LAN8742 BCR Bit Definition
  * @{
  */
#define KSZ8041_BCR_SOFT_RESET         ((uint16_t)0x8000U)
#define KSZ8041_BCR_LOOPBACK           ((uint16_t)0x4000U)
#define KSZ8041_BCR_SPEED_SELECT       ((uint16_t)0x2000U)
#define KSZ8041_BCR_AUTONEGO_EN        ((uint16_t)0x1000U)
#define KSZ8041_BCR_POWER_DOWN         ((uint16_t)0x0800U)
#define KSZ8041_BCR_ISOLATE            ((uint16_t)0x0400U)
#define KSZ8041_BCR_RESTART_AUTONEGO   ((uint16_t)0x0200U)
#define KSZ8041_BCR_DUPLEX_MODE        ((uint16_t)0x0100U)
#define KSZ8041_BCR_COLLISION_TEST     ((uint16_t)0x0080U)
#define KSZ8041_BCR_DISABLE_TRANSMIT   ((uint16_t)0x0001U)
/**
  * @}
  */

/** @defgroup LAN8742_BSR_Bit_Definition LAN8742 BSR Bit Definition
  * @{
  */
#define KSZ8041_BSR_100BASE_T4       ((uint16_t)0x8000U)
#define KSZ8041_BSR_100BASE_TX_FD    ((uint16_t)0x4000U)
#define KSZ8041_BSR_100BASE_TX_HD    ((uint16_t)0x2000U)
#define KSZ8041_BSR_10BASE_T_FD      ((uint16_t)0x1000U)
#define KSZ8041_BSR_10BASE_T_HD      ((uint16_t)0x0800U)
//#define LAN8742_BSR_100BASE_T2_FD    ((uint16_t)0x0400U)
//#define LAN8742_BSR_100BASE_T2_HD    ((uint16_t)0x0200U)
//#define LAN8742_BSR_EXTENDED_STATUS  ((uint16_t)0x0100U)
#define KSZ8041_BSR_AUTONEGO_CPLT    ((uint16_t)0x0020U)
#define KSZ8041_BSR_REMOTE_FAULT     ((uint16_t)0x0010U)
#define KSZ8041_BSR_AUTONEGO_ABILITY ((uint16_t)0x0008U)
#define KSZ8041_BSR_LINK_STATUS      ((uint16_t)0x0004U)
#define KSZ8041_BSR_JABBER_DETECT    ((uint16_t)0x0002U)
#define KSZ8041_BSR_EXTENDED_CAP     ((uint16_t)0x0001U)
/**
  * @}
  */
//
///** @defgroup LAN8742_PHYI1R_Bit_Definition LAN8742 PHYI1R Bit Definition
//  * @{
//  */
//#define LAN8742_PHYI1R_OUI_3_18           ((uint16_t)0xFFFFU)
///**
//  * @}
//  */
//
///** @defgroup LAN8742_PHYI2R_Bit_Definition LAN8742 PHYI2R Bit Definition
//  * @{
//  */
//#define LAN8742_PHYI2R_OUI_19_24          ((uint16_t)0xFC00U)
//#define LAN8742_PHYI2R_MODEL_NBR          ((uint16_t)0x03F0U)
//#define LAN8742_PHYI2R_REVISION_NBR       ((uint16_t)0x000FU)
///**
//  * @}
//  */
//
///** @defgroup LAN8742_ANAR_Bit_Definition LAN8742 ANAR Bit Definition
//  * @{
//  */
//#define LAN8742_ANAR_NEXT_PAGE               ((uint16_t)0x8000U)
//#define LAN8742_ANAR_REMOTE_FAULT            ((uint16_t)0x2000U)
//#define LAN8742_ANAR_PAUSE_OPERATION         ((uint16_t)0x0C00U)
//#define LAN8742_ANAR_PO_NOPAUSE              ((uint16_t)0x0000U)
//#define LAN8742_ANAR_PO_SYMMETRIC_PAUSE      ((uint16_t)0x0400U)
//#define LAN8742_ANAR_PO_ASYMMETRIC_PAUSE     ((uint16_t)0x0800U)
//#define LAN8742_ANAR_PO_ADVERTISE_SUPPORT    ((uint16_t)0x0C00U)
//#define LAN8742_ANAR_100BASE_TX_FD           ((uint16_t)0x0100U)
//#define LAN8742_ANAR_100BASE_TX              ((uint16_t)0x0080U)
//#define LAN8742_ANAR_10BASE_T_FD             ((uint16_t)0x0040U)
//#define LAN8742_ANAR_10BASE_T                ((uint16_t)0x0020U)
//#define LAN8742_ANAR_SELECTOR_FIELD          ((uint16_t)0x000FU)
///**
//  * @}
//  */
//
///** @defgroup LAN8742_ANLPAR_Bit_Definition LAN8742 ANLPAR Bit Definition
//  * @{
//  */
//#define LAN8742_ANLPAR_NEXT_PAGE            ((uint16_t)0x8000U)
//#define LAN8742_ANLPAR_REMOTE_FAULT         ((uint16_t)0x2000U)
//#define LAN8742_ANLPAR_PAUSE_OPERATION      ((uint16_t)0x0C00U)
//#define LAN8742_ANLPAR_PO_NOPAUSE           ((uint16_t)0x0000U)
//#define LAN8742_ANLPAR_PO_SYMMETRIC_PAUSE   ((uint16_t)0x0400U)
//#define LAN8742_ANLPAR_PO_ASYMMETRIC_PAUSE  ((uint16_t)0x0800U)
//#define LAN8742_ANLPAR_PO_ADVERTISE_SUPPORT ((uint16_t)0x0C00U)
//#define LAN8742_ANLPAR_100BASE_TX_FD        ((uint16_t)0x0100U)
//#define LAN8742_ANLPAR_100BASE_TX           ((uint16_t)0x0080U)
//#define LAN8742_ANLPAR_10BASE_T_FD          ((uint16_t)0x0040U)
//#define LAN8742_ANLPAR_10BASE_T             ((uint16_t)0x0020U)
//#define LAN8742_ANLPAR_SELECTOR_FIELD       ((uint16_t)0x000FU)
///**
//  * @}
//  */
//
///** @defgroup LAN8742_ANER_Bit_Definition LAN8742 ANER Bit Definition
//  * @{
//  */
//#define LAN8742_ANER_RX_NP_LOCATION_ABLE    ((uint16_t)0x0040U)
//#define LAN8742_ANER_RX_NP_STORAGE_LOCATION ((uint16_t)0x0020U)
//#define LAN8742_ANER_PARALLEL_DETECT_FAULT  ((uint16_t)0x0010U)
//#define LAN8742_ANER_LP_NP_ABLE             ((uint16_t)0x0008U)
//#define LAN8742_ANER_NP_ABLE                ((uint16_t)0x0004U)
//#define LAN8742_ANER_PAGE_RECEIVED          ((uint16_t)0x0002U)
//#define LAN8742_ANER_LP_AUTONEG_ABLE        ((uint16_t)0x0001U)
///**
//  * @}
//  */
//
///** @defgroup LAN8742_ANNPTR_Bit_Definition LAN8742 ANNPTR Bit Definition
//  * @{
//  */
//#define LAN8742_ANNPTR_NEXT_PAGE         ((uint16_t)0x8000U)
//#define LAN8742_ANNPTR_MESSAGE_PAGE      ((uint16_t)0x2000U)
//#define LAN8742_ANNPTR_ACK2              ((uint16_t)0x1000U)
//#define LAN8742_ANNPTR_TOGGLE            ((uint16_t)0x0800U)
//#define LAN8742_ANNPTR_MESSAGGE_CODE     ((uint16_t)0x07FFU)
///**
//  * @}
//  */
//
///** @defgroup LAN8742_ANNPRR_Bit_Definition LAN8742 ANNPRR Bit Definition
//  * @{
//  */
//#define LAN8742_ANNPTR_NEXT_PAGE         ((uint16_t)0x8000U)
//#define LAN8742_ANNPRR_ACK               ((uint16_t)0x4000U)
//#define LAN8742_ANNPRR_MESSAGE_PAGE      ((uint16_t)0x2000U)
//#define LAN8742_ANNPRR_ACK2              ((uint16_t)0x1000U)
//#define LAN8742_ANNPRR_TOGGLE            ((uint16_t)0x0800U)
//#define LAN8742_ANNPRR_MESSAGGE_CODE     ((uint16_t)0x07FFU)
///**
//  * @}
//  */
//
///** @defgroup LAN8742_MMDACR_Bit_Definition LAN8742 MMDACR Bit Definition
//  * @{
//  */
//#define LAN8742_MMDACR_MMD_FUNCTION       ((uint16_t)0xC000U)
//#define LAN8742_MMDACR_MMD_FUNCTION_ADDR  ((uint16_t)0x0000U)
//#define LAN8742_MMDACR_MMD_FUNCTION_DATA  ((uint16_t)0x4000U)
//#define LAN8742_MMDACR_MMD_DEV_ADDR       ((uint16_t)0x001FU)
///**
//  * @}
//  */
//
///** @defgroup LAN8742_ENCTR_Bit_Definition LAN8742 ENCTR Bit Definition
//  * @{
//  */
//#define LAN8742_ENCTR_TX_ENABLE             ((uint16_t)0x8000U)
//#define LAN8742_ENCTR_TX_TIMER              ((uint16_t)0x6000U)
//#define LAN8742_ENCTR_TX_TIMER_1S           ((uint16_t)0x0000U)
//#define LAN8742_ENCTR_TX_TIMER_768MS        ((uint16_t)0x2000U)
//#define LAN8742_ENCTR_TX_TIMER_512MS        ((uint16_t)0x4000U)
//#define LAN8742_ENCTR_TX_TIMER_265MS        ((uint16_t)0x6000U)
//#define LAN8742_ENCTR_RX_ENABLE             ((uint16_t)0x1000U)
//#define LAN8742_ENCTR_RX_MAX_INTERVAL       ((uint16_t)0x0C00U)
//#define LAN8742_ENCTR_RX_MAX_INTERVAL_64MS  ((uint16_t)0x0000U)
//#define LAN8742_ENCTR_RX_MAX_INTERVAL_256MS ((uint16_t)0x0400U)
//#define LAN8742_ENCTR_RX_MAX_INTERVAL_512MS ((uint16_t)0x0800U)
//#define LAN8742_ENCTR_RX_MAX_INTERVAL_1S    ((uint16_t)0x0C00U)
//#define LAN8742_ENCTR_EX_CROSS_OVER         ((uint16_t)0x0002U)
//#define LAN8742_ENCTR_EX_MANUAL_CROSS_OVER  ((uint16_t)0x0001U)
///**
//  * @}
//  */
//
///** @defgroup LAN8742_MCSR_Bit_Definition LAN8742 MCSR Bit Definition
//  * @{
//  */
//#define LAN8742_MCSR_EDPWRDOWN        ((uint16_t)0x2000U)
//#define LAN8742_MCSR_FARLOOPBACK      ((uint16_t)0x0200U)
//#define LAN8742_MCSR_ALTINT           ((uint16_t)0x0040U)
//#define LAN8742_MCSR_ENERGYON         ((uint16_t)0x0002U)
///**
//  * @}
//  */
//
///** @defgroup LAN8742_SMR_Bit_Definition LAN8742 SMR Bit Definition
//  * @{
//  */
//#define LAN8742_SMR_MODE       ((uint16_t)0x00E0U)
//#define LAN8742_SMR_PHY_ADDR   ((uint16_t)0x001FU)
///**
//  * @}
//  */
//
///** @defgroup LAN8742_TPDCR_Bit_Definition LAN8742 TPDCR Bit Definition
//  * @{
//  */
//#define LAN8742_TPDCR_DELAY_IN                 ((uint16_t)0x8000U)
//#define LAN8742_TPDCR_LINE_BREAK_COUNTER       ((uint16_t)0x7000U)
//#define LAN8742_TPDCR_PATTERN_HIGH             ((uint16_t)0x0FC0U)
//#define LAN8742_TPDCR_PATTERN_LOW              ((uint16_t)0x003FU)
///**
//  * @}
//  */
//
///** @defgroup LAN8742_TCSR_Bit_Definition LAN8742 TCSR Bit Definition
//  * @{
//  */
//#define LAN8742_TCSR_TDR_ENABLE           ((uint16_t)0x8000U)
//#define LAN8742_TCSR_TDR_AD_FILTER_ENABLE ((uint16_t)0x4000U)
//#define LAN8742_TCSR_TDR_CH_CABLE_TYPE    ((uint16_t)0x0600U)
//#define LAN8742_TCSR_TDR_CH_CABLE_DEFAULT ((uint16_t)0x0000U)
//#define LAN8742_TCSR_TDR_CH_CABLE_SHORTED ((uint16_t)0x0200U)
//#define LAN8742_TCSR_TDR_CH_CABLE_OPEN    ((uint16_t)0x0400U)
//#define LAN8742_TCSR_TDR_CH_CABLE_MATCH   ((uint16_t)0x0600U)
//#define LAN8742_TCSR_TDR_CH_STATUS        ((uint16_t)0x0100U)
//#define LAN8742_TCSR_TDR_CH_LENGTH        ((uint16_t)0x00FFU)
///**
//  * @}
//  */
//
///** @defgroup LAN8742_SCSIR_Bit_Definition LAN8742 SCSIR Bit Definition
//  * @{
//  */
//#define LAN8742_SCSIR_AUTO_MDIX_ENABLE    ((uint16_t)0x8000U)
//#define LAN8742_SCSIR_CHANNEL_SELECT      ((uint16_t)0x2000U)
//#define LAN8742_SCSIR_SQE_DISABLE         ((uint16_t)0x0800U)
//#define LAN8742_SCSIR_XPOLALITY           ((uint16_t)0x0010U)
///**
//  * @}
//  */
//
///** @defgroup LAN8742_CLR_Bit_Definition LAN8742 CLR Bit Definition
//  * @{
//  */
//#define LAN8742_CLR_CABLE_LENGTH       ((uint16_t)0xF000U)
///**
//  * @}
//  */
//
/** @defgroup KSZ8041_IMR_ISFR_Bit_Definition KSZ8041 IMR ISFR Bit Definition
  * @{
  */
#define KSZ8041_INT_8       ((uint16_t)0x8000U)
#define KSZ8041_INT_7       ((uint16_t)0x4000U)
#define KSZ8041_INT_6       ((uint16_t)0x2000U)
#define KSZ8041_INT_5       ((uint16_t)0x1000U)
#define KSZ8041_INT_4       ((uint16_t)0x0800U)
#define KSZ8041_INT_3       ((uint16_t)0x0400U)
#define KSZ8041_INT_2       ((uint16_t)0x0200U)
#define KSZ8041_INT_1       ((uint16_t)0x0100U)
/**
  * @}
  */
//
/** @defgroup LAN8742_PHYSCSR_Bit_Definition LAN8742 PHYSCSR Bit Definition
  * @{
  */
#define KSZ8041_CTRL2_AUTONEGO_DONE   ((uint16_t)0x0080U)
#define KSZ8041_CTRL2_HCDSPEEDMASK    ((uint16_t)0x001CU)
#define KSZ8041_CTRL2_10BT_HD         ((uint16_t)0x0004U)
#define KSZ8041_CTRL2_10BT_FD         ((uint16_t)0x0014U)
#define KSZ8041_CTRL2_100BTX_HD       ((uint16_t)0x0008U)
#define KSZ8041_CTRL2_100BTX_FD       ((uint16_t)0x0018U)
/**
  * @}
  */

/** @defgroup KSZ8041_Status KSZ8041_ Status
  * @{
  */

#define  KSZ8041_STATUS_READ_ERROR            ((int32_t)-5)
#define  KSZ8041_STATUS_WRITE_ERROR           ((int32_t)-4)
#define  KSZ8041_STATUS_ADDRESS_ERROR         ((int32_t)-3)
#define  KSZ8041_STATUS_RESET_TIMEOUT         ((int32_t)-2)
#define  KSZ8041_STATUS_ERROR                 ((int32_t)-1)
#define  KSZ8041_STATUS_OK                    ((int32_t) 0)
#define  KSZ8041_STATUS_LINK_DOWN             ((int32_t) 1)
#define  KSZ8041_STATUS_100MBITS_FULLDUPLEX   ((int32_t) 2)
#define  KSZ8041_STATUS_100MBITS_HALFDUPLEX   ((int32_t) 3)
#define  KSZ8041_STATUS_10MBITS_FULLDUPLEX    ((int32_t) 4)
#define  KSZ8041_STATUS_10MBITS_HALFDUPLEX    ((int32_t) 5)
#define  KSZ8041_STATUS_AUTONEGO_NOTDONE      ((int32_t) 6)
/**
  * @}
  */

/** @defgroup LAN8742_IT_Flags LAN8742 IT Flags
  * @{
  */
#define  KSZ8041_JABBER_IF                     KSZ8041_INT_8
#define  KSZ8041_RECEIVE_ERROR_IF              KSZ8041_INT_7
#define  KSZ8041_PAGE_RECEIVED_IF              KSZ8041_INT_6
#define  KSZ8041_PARALLEL_DETECTION_FAULT_IF   KSZ8041_INT_5
#define  KSZ8041_LINK_PARTNER_ACK_IF           KSZ8041_INT_4
#define  KSZ8041_LINK_DOWN_IF                  KSZ8041_INT_3
#define  KSZ8041_REMOTE_FAULT_IF               KSZ8041_INT_2
#define  KSZ8041_LINK_UP_IF                    KSZ8041_INT_1

#define  KSZ8041_JABBER_IE                     KSZ8041_INT_8 << 8
#define  KSZ8041_RECEIVE_ERROR_IE              KSZ8041_INT_7 << 8
#define  KSZ8041_PAGE_RECEIVED_IE              KSZ8041_INT_6 << 8
#define  KSZ8041_PARALLEL_DETECTION_FAULT_IE   KSZ8041_INT_5 << 8
#define  KSZ8041_LINK_PARTNER_ACK_IE           KSZ8041_INT_4 << 8
#define  KSZ8041_LINK_DOWN_IE                  KSZ8041_INT_3 << 8
#define  KSZ8041_REMOTE_FAULT_IE               KSZ8041_INT_2 << 8
#define  KSZ8041_LINK_UP_IE                    KSZ8041_INT_1 << 8
/**
  * @}
  */

/**
  * @}
  */

/* Exported types ------------------------------------------------------------*/
/** @defgroup KSZ8041_Exported_Types KSZ8041 Exported Types
  * @{
  */
typedef int32_t  (*ksz8041_Init_Func) (void);
typedef int32_t  (*ksz8041_DeInit_Func) (void);
typedef int32_t  (*ksz8041_ReadReg_Func)   (uint32_t, uint32_t, uint32_t *);
typedef int32_t  (*ksz8041_WriteReg_Func)  (uint32_t, uint32_t, uint32_t);
typedef int32_t  (*ksz8041_GetTick_Func)  (void);

typedef struct
{
  ksz8041_Init_Func      Init;
  ksz8041_DeInit_Func    DeInit;
  ksz8041_WriteReg_Func  WriteReg;
  ksz8041_ReadReg_Func   ReadReg;
  ksz8041_GetTick_Func   GetTick;
} ksz8041_IOCtx_t;


typedef struct
{
  uint32_t            DevAddr;
  uint32_t            Is_Initialized;
  ksz8041_IOCtx_t     IO;
  void               *pData;
} ksz8041_Object_t;
/**
  * @}
  */

/* Exported macro ------------------------------------------------------------*/
/* Exported functions --------------------------------------------------------*/
/** @defgroup LAN8742_Exported_Functions LAN8742 Exported Functions
  * @{
  */
int32_t KSZ8041_RegisterBusIO(ksz8041_Object_t *pObj, ksz8041_IOCtx_t *ioctx);
int32_t KSZ8041_Init(ksz8041_Object_t *pObj);
int32_t KSZ8041_DeInit(ksz8041_Object_t *pObj);
int32_t KSZ8041_DisablePowerDownMode(ksz8041_Object_t *pObj);
int32_t KSZ8041_EnablePowerDownMode(ksz8041_Object_t *pObj);
int32_t KSZ8041_StartAutoNego(ksz8041_Object_t *pObj);
int32_t KSZ8041_GetLinkState(ksz8041_Object_t *pObj);
int32_t KSZ8041_SetLinkState(ksz8041_Object_t *pObj, uint32_t LinkState);
//int32_t LAN8742_EnableLoopbackMode(lan8742_Object_t *pObj);
//int32_t LAN8742_DisableLoopbackMode(lan8742_Object_t *pObj);
int32_t KSZ8041_EnableIT(ksz8041_Object_t *pObj, uint32_t Interrupt);
int32_t KSZ8041_DisableIT(ksz8041_Object_t *pObj, uint32_t Interrupt);
//int32_t LAN8742_ClearIT(lan8742_Object_t *pObj, uint32_t Interrupt);
int32_t KSZ8041_GetITStatusClear(ksz8041_Object_t *pObj);
/**
  * @}
  */


/**
  * @}
  */

/**
  * @}
  */

/**
  * @}
  */

/**
  * @}
  */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/




#endif /* INC_KSZ8041_H_ */
