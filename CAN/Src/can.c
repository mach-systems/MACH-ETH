/*
 * can.c
 *
 *  Created on: 9. 8. 2021
 *      Author: petr_kolar
 */

#include "string.h"
#include "can.h"
#include "config.h"
#include "packetControl.h"
#include "commControl.h"
#include "stdbool.h"
#include <usbd_cdc_if.h>    /* CDC_Transmit_HS() */
#include <cmsis_os2.h>
#include "config.h"         /* HW_VERSION */
#include "tcpServer.h"
#include <stdio.h>

extern CONFIGURATION_CAN configCan1;
extern CONFIGURATION_CAN configCan2;

extern FDCAN_HandleTypeDef hfdcan1;
extern FDCAN_HandleTypeDef hfdcan3;

static const uint8_t DLCtoBytes[] =
{ 0, 1, 2, 3, 4, 5, 6, 7, 8, 12, 16, 20, 24, 32, 48, 64 };

bool canChannel1Run;
bool canChannel2Run;

uint8_t canChannel1ChannelToTX = TCP_CHANNEL;
uint8_t canChannel2ChannelToTX = TCP_CHANNEL;

bool led1CanGreenBlink;
bool led2CanGreenBlink;
bool led1CanRedBlink;
bool led2CanRedBlink;

uint16_t counter1CanBlinkRed;
uint16_t counter1CanBlinkGreen;
uint16_t counter2CanBlinkRed;
uint16_t counter2CanBlinkGreen;

uint8_t ConfigureCAN(uint8_t channel)
{

    FDCAN_HandleTypeDef *pfdcan;
    CONFIGURATION_CAN *canConf;
    if (channel == CAN1_NUM)
    {
        pfdcan = &CAN1;
        canConf = &configCan1;
    }
    else if (channel == CAN2_NUM)
    {
        pfdcan = &CAN2;
        canConf = &configCan2;
    }
    else
    {
        //Channel not implemented error
        return CAN_CHANNEL_NOT_IMPLEMENTED;
    }
    HAL_FDCAN_DeInit(pfdcan);

    //Set parameters
    if (canConf->CanConfiguration.Mode)
    {
        pfdcan->Init.Mode = FDCAN_MODE_BUS_MONITORING;
    }
    else
    {
        pfdcan->Init.Mode = FDCAN_MODE_NORMAL;
    }

    if (canConf->CanConfiguration.Protocol == CAN)
    {
        pfdcan->Init.FrameFormat = FDCAN_FRAME_CLASSIC;
    }
    else if (canConf->CanConfiguration.Protocol == ISO_CAN_FD)
        pfdcan->Init.FrameFormat = FDCAN_FRAME_FD_BRS;

    pfdcan->Init.NominalSyncJumpWidth =
            canConf->CanConfiguration.ArbitrationSJW;

    pfdcan->Init.DataSyncJumpWidth = canConf->CanConfiguration.DataSJW;

    pfdcan->Init.TxElmtSize = FDCAN_DATA_BYTES_64;
    pfdcan->Init.TxBuffersNbr = 8;
    pfdcan->Init.RxFifo0ElmtSize = FDCAN_DATA_BYTES_64;
    pfdcan->Init.RxBuffersNbr = 8;
    pfdcan->Init.AutoRetransmission = DISABLE;

    pfdcan->Init.NominalPrescaler =
            canConf->CanConfiguration.ArbitrationPrescaler;
    pfdcan->Init.NominalTimeSeg1 =
            canConf->CanConfiguration.ArbitrationTimeSegment1;
    pfdcan->Init.NominalTimeSeg2 =
            canConf->CanConfiguration.ArbitrationTimeSegment2;
    pfdcan->Init.DataPrescaler = canConf->CanConfiguration.DataPrescaler;
    pfdcan->Init.DataTimeSeg1 = canConf->CanConfiguration.DataTimeSegment1;
    pfdcan->Init.DataTimeSeg2 = canConf->CanConfiguration.DataTimeSegment2;

    if (HAL_FDCAN_Init(pfdcan) != HAL_OK)
        return CAN_ERROR;
    return 0;
}

uint8_t CanStartChannel(uint8_t channel)
{

    FDCAN_HandleTypeDef *phfdcan;
    FDCAN_FilterTypeDef filter;

    if (channel == CAN1_NUM)
    {
        phfdcan = &CAN1;
        if (canChannel1Run == true)
            return CAN_IS_ALREADY_RUNNING;
    }
    else if (channel == CAN2_NUM)
    {
        phfdcan = &CAN2;
        if (canChannel2Run == true)
            return CAN_IS_ALREADY_RUNNING;
    }
    else
    {
        //Channel not implemented error
        return CAN_CHANNEL_NOT_IMPLEMENTED;
    }

    ConfigureCAN(channel);

    HAL_StatusTypeDef status;
    status = HAL_FDCAN_ConfigInterruptLines(phfdcan,
            FDCAN_IT_RX_FIFO0_NEW_MESSAGE | FDCAN_TT_FLAG_ERROR_LEVEL_CHANGE
                    | FDCAN_IT_ARB_PROTOCOL_ERROR, FDCAN_INTERRUPT_LINE0);
    if (status != HAL_OK)
        return CAN_ERROR;

    status = HAL_FDCAN_ActivateNotification(phfdcan,
            FDCAN_IT_RX_FIFO0_NEW_MESSAGE | FDCAN_TT_FLAG_ERROR_LEVEL_CHANGE
                    | FDCAN_IT_ARB_PROTOCOL_ERROR, 0);
    if (status != HAL_OK)
        return CAN_ERROR;

    filter.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
    filter.IdType = FDCAN_STANDARD_ID;
    filter.FilterType = FDCAN_FILTER_MASK;
    filter.FilterID1 = 0x0000;
    filter.FilterID2 = 0x0000;
    filter.FilterIndex = 0;

    if (HAL_FDCAN_ConfigFilter(phfdcan, &filter) != HAL_OK)
        return CAN_ERROR;

    status = HAL_FDCAN_Start(phfdcan);
    if (status != HAL_OK)
        return CAN_ERROR;
    else if (channel == CAN1_NUM)
    {
        canChannel1Run = true;
        LED_CAN1_GREEN_ON();
        LED_CAN1_RED_OFF();
    }
    else if (channel == CAN2_NUM)
    {
        canChannel2Run = true;
        LED_CAN2_GREEN_ON();
        LED_CAN2_RED_OFF();
    }
    else
    {
        //Channel not implemented error
        return CAN_CHANNEL_NOT_IMPLEMENTED;
    }

    return 0;
}

uint8_t CanStopChannel(uint8_t channel)
{
    if (channel == CAN1_NUM)
    {
        HAL_FDCAN_Stop(&CAN1);
        canChannel1Run = false;
        LED_CAN1_GREEN_OFF();
        LED_CAN1_RED_OFF();
    }
    else if (channel == CAN2_NUM)
    {
        HAL_FDCAN_Stop(&CAN2);
        canChannel2Run = false;
        LED_CAN2_GREEN_OFF();
        LED_CAN2_RED_OFF();
    }
    else
    {
        return CAN_CHANNEL_NOT_IMPLEMENTED;
    }
    return CAN_OK;
}

uint8_t CanSendMessage(CanMessageStruct *message)
{

    bool messageProtocol = false; // Check if message must be send by FDCAN
    FDCAN_TxHeaderTypeDef TxHeader;
    HAL_StatusTypeDef ret;

    TxHeader.Identifier = message->Id;
    if (!message->EXTId)
    {
        TxHeader.IdType = FDCAN_STANDARD_ID;
    }
    else
    {
        TxHeader.IdType = FDCAN_EXTENDED_ID;
    }

    if (!message->RTR)
    {
        TxHeader.TxFrameType = FDCAN_DATA_FRAME;
    }
    else
    {
        TxHeader.TxFrameType = FDCAN_REMOTE_FRAME;
    }

    if (!message->BRS)
    {
        TxHeader.BitRateSwitch = FDCAN_BRS_OFF;
    }
    else
    {
        TxHeader.BitRateSwitch = FDCAN_BRS_ON;
        messageProtocol = true;
    }

    if (!message->ESI)
    {
        TxHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    }
    else
    {
        TxHeader.ErrorStateIndicator = FDCAN_ESI_PASSIVE;
        messageProtocol = true;

    }

    if (!message->FDF)
    {
        TxHeader.FDFormat = FDCAN_CLASSIC_CAN;
    }
    else
    {
        TxHeader.FDFormat = FDCAN_FD_CAN;
        messageProtocol = true;
    }

    if (IntToDLC(message->DLC, &TxHeader.DataLength))
    {
        return CAN_WRONG_DLC;
    }

    TxHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    TxHeader.MessageMarker = 0;

    if (message->CANChannel == CAN1_NUM)
    {
        if (configCan1.CanConfiguration.Protocol == CAN && messageProtocol)
            return CAN_WRONG_PROTOCOL;
        if (!canChannel1Run)
            return CAN_IS_NOT_RUNNING;
        ret = HAL_FDCAN_AddMessageToTxFifoQ(&CAN1,
                (FDCAN_TxHeaderTypeDef*) &TxHeader, message->Data);
        if (ret != HAL_OK)
        {
            blinkLedCan(RED_CAN_LED1);
            if ( CAN1.ErrorCode == HAL_FDCAN_ERROR_FIFO_FULL)
                return CAN_FIFO_FULL;
            else
                return CAN_ERROR;
        }
        else
        {
            blinkLedCan(GREEN_CAN_LED1);
        }
    }
    else if (message->CANChannel == CAN2_NUM)
    {
        if (configCan2.CanConfiguration.Protocol == CAN && messageProtocol)
            return CAN_WRONG_PROTOCOL;
        if (!canChannel2Run)
            return CAN_IS_NOT_RUNNING;
        ret = HAL_FDCAN_AddMessageToTxFifoQ(&CAN2,
                (FDCAN_TxHeaderTypeDef*) &TxHeader, message->Data);
        if (ret != HAL_OK)
        {
            blinkLedCan(RED_CAN_LED2);
            if ( CAN1.ErrorCode == HAL_FDCAN_ERROR_FIFO_FULL)
                return CAN_FIFO_FULL;
            else
                return CAN_ERROR;
        }
        else
        {
            blinkLedCan(GREEN_CAN_LED2);
        }
    }
    else
    {
        //Channel not implemented error
        return CAN_CHANNEL_NOT_IMPLEMENTED;
    }

    return 0;

}

void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
    FDCAN_RxHeaderTypeDef RxHeader;
    uint8_t RxData[CAN_MAX_DATALEN];
    if ((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) != RESET)
    {
        /* Retrieve Rx messages from RX FIFO0 */
        if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &RxHeader, RxData)
                != HAL_OK)
        {
            return;
        }

        /* Respond back with the same data and header */
    }

    if (hfdcan == &CAN1)
    {
        blinkLedCan(GREEN_CAN_LED1);

    }
    else if (hfdcan == &CAN2)
    {
        blinkLedCan(GREEN_CAN_LED2);
    }

    uint8_t datalen = DLCtoBytes[RxHeader.DataLength >> 16];

    /* It is strongly suggested to keep the possibility to jump to System Booloader from application */
    if (RxHeader.Identifier == 0x1fffffff && datalen == 4 && RxData[0] == 0
            && RxData[1] == 1 && RxData[2] == 2 && RxData[3] == 3)
    {
#ifdef Bootloader   /* HTTP Bootloader */
        /* Cannot go to bootloader directly from ISR */
        BootloaderRequest = 2;
#else
#ifdef NoBootloader /* STM System Bootloader */
        BootloaderRequest = 1;
#endif
#endif

        return;
    }

    /* Send the information about received CAN frame to the virtual COM port */
    char usbSendBuffer[USB_BUFFER_SIZE];
    uint8_t dataWritten = 0;
    dataWritten += snprintf(usbSendBuffer + dataWritten, USB_BUFFER_SIZE,
            "ID: %lx, ", RxHeader.Identifier);
    char *chan = (hfdcan == &CAN1) ? "CAN1" : "CAN2";
    dataWritten += snprintf(usbSendBuffer + dataWritten, USB_BUFFER_SIZE,
            "channel: %s, ", chan);
    dataWritten += snprintf(usbSendBuffer + dataWritten, USB_BUFFER_SIZE,
            "datalen: %d, ", datalen);
    dataWritten += snprintf(usbSendBuffer + dataWritten, USB_BUFFER_SIZE,
            "data: ");
    for (uint8_t i = 0; i < datalen; i++)
    {
        dataWritten += snprintf(usbSendBuffer + dataWritten, USB_BUFFER_SIZE,
                "%x", RxData[i]);
        if (i != datalen - 1)
            dataWritten += snprintf(usbSendBuffer + dataWritten,
                    USB_BUFFER_SIZE, ", ");
    }
    dataWritten += snprintf(usbSendBuffer + dataWritten, USB_BUFFER_SIZE,
            "\r\n");

    TcpEnqueueResponse((uint8_t*) usbSendBuffer, dataWritten);
    CDC_Transmit_HS((uint8_t*) usbSendBuffer, dataWritten);
}

void HAL_FDCAN_ErrorCallback(FDCAN_HandleTypeDef *hfdcan)
{
    FDCAN_ProtocolStatusTypeDef ProtocolStatus;
    HAL_FDCAN_GetProtocolStatus(hfdcan, &ProtocolStatus);
    if (hfdcan == &CAN1)
    {

        if (ProtocolStatus.RxBRSflag
                && ProtocolStatus.DataLastErrorCode != FDCAN_PROTOCOL_ERROR_NONE
                && ProtocolStatus.DataLastErrorCode
                        != FDCAN_PROTOCOL_ERROR_NO_CHANGE)
        {
            blinkLedCan(RED_CAN_LED1);
        }
        if (ProtocolStatus.LastErrorCode != FDCAN_PROTOCOL_ERROR_NONE
                && ProtocolStatus.DataLastErrorCode
                        != FDCAN_PROTOCOL_ERROR_NO_CHANGE)
            blinkLedCan(RED_CAN_LED1);
        if (ProtocolStatus.ErrorPassive == 1)
            LED_CAN1_RED_ON();

    }
    else if (hfdcan == &CAN2)
    {
        if (ProtocolStatus.RxBRSflag
                && ProtocolStatus.DataLastErrorCode != FDCAN_PROTOCOL_ERROR_NONE
                && ProtocolStatus.DataLastErrorCode
                        != FDCAN_PROTOCOL_ERROR_NO_CHANGE)
        {
            blinkLedCan(RED_CAN_LED2);
        }

        if (ProtocolStatus.LastErrorCode != FDCAN_PROTOCOL_ERROR_NONE
                && ProtocolStatus.DataLastErrorCode
                        != FDCAN_PROTOCOL_ERROR_NO_CHANGE)
            blinkLedCan(RED_CAN_LED2);
        if (ProtocolStatus.ErrorPassive == 1)
            LED_CAN2_RED_ON();
    }
    else
    {
        return;
    }

}

uint8_t IntToDLC(uint8_t dataLength, uint32_t *pDLC)
{

    switch (dataLength)
    {
        case 0:
            *pDLC = FDCAN_DLC_BYTES_0;
            break;
        case 1:
            *pDLC = FDCAN_DLC_BYTES_1;
            break;
        case 2:
            *pDLC = FDCAN_DLC_BYTES_2;
            break;
        case 3:
            *pDLC = FDCAN_DLC_BYTES_3;
            break;
        case 4:
            *pDLC = FDCAN_DLC_BYTES_4;
            break;
        case 5:
            *pDLC = FDCAN_DLC_BYTES_5;
            break;
        case 6:
            *pDLC = FDCAN_DLC_BYTES_6;
            break;
        case 7:
            *pDLC = FDCAN_DLC_BYTES_7;
            break;
        case 8:
            *pDLC = FDCAN_DLC_BYTES_8;
            break;
        case 12:
            *pDLC = FDCAN_DLC_BYTES_12;
            break;
        case 16:
            *pDLC = FDCAN_DLC_BYTES_16;
            break;
        case 20:
            *pDLC = FDCAN_DLC_BYTES_20;
            break;
        case 24:
            *pDLC = FDCAN_DLC_BYTES_24;
            break;
        case 32:
            *pDLC = FDCAN_DLC_BYTES_32;
            break;
        case 48:
            *pDLC = FDCAN_DLC_BYTES_48;
            break;
        case 64:
            *pDLC = FDCAN_DLC_BYTES_64;
            break;
        default:
            return CAN_WRONG_DLC;
            break;
    }
    return CAN_OK;
}

uint8_t BaudToParametrs(uint8_t baudRate, uint8_t sPoint, uint8_t *prescaler,
        uint8_t *pTseg1, uint8_t *pTseg2)
{
    // This parameters were computed for 40 MHz clock

    switch (baudRate)
    {

        case BAUDRATE_125k:
            if (sPoint <= SP_75)
            {
                *prescaler = 16;
                *pTseg1 = 14;
                *pTseg2 = 5;
            }
            else if (sPoint <= SP_82_5)
            {
                *prescaler = 16;
                *pTseg1 = 15;
                *pTseg2 = 4;
            }
            else
            {
                *prescaler = 20;
                *pTseg1 = 13;
                *pTseg2 = 2;
                break;
            }
            break;
        case BAUDRATE_250k:
            if (sPoint <= SP_75)
            {
                *prescaler = 8;
                *pTseg1 = 14;
                *pTseg2 = 5;
            }
            else if (sPoint <= SP_82_5)
            {
                *prescaler = 8;
                *pTseg1 = 15;
                *pTseg2 = 4;
            }
            else
            {
                *prescaler = 10;
                *pTseg1 = 13;
                *pTseg2 = 2;
            }
            break;
        case BAUDRATE_500k:
            if (sPoint <= SP_75)
            {
                *prescaler = 4;
                *pTseg1 = 14;
                *pTseg2 = 5;
            }
            else if (sPoint <= SP_82_5)
            {
                *prescaler = 4;
                *pTseg1 = 15;
                *pTseg2 = 4;
            }
            else
            {
                *prescaler = 5;
                *pTseg1 = 13;
                *pTseg2 = 2;
            }
            break;

        case BAUDRATE_1M:
            if (sPoint <= SP_75)
            {
                *prescaler = 2;
                *pTseg1 = 14;
                *pTseg2 = 5;
            }
            else if (sPoint <= SP_82_5)
            {
                *prescaler = 2;
                *pTseg1 = 15;
                *pTseg2 = 4;
            }
            else
            {
                *prescaler = 5;
                *pTseg1 = 6;
                *pTseg2 = 1;
            }
            break;
        case BAUDRATE_2M:
            if (sPoint <= SP_75)
            {
                *prescaler = 1;
                *pTseg1 = 14;
                *pTseg2 = 5;
            }
            else if (sPoint <= SP_82_5)
            {
                *prescaler = 1;
                *pTseg1 = 15;
                *pTseg2 = 4;
            }
            else
            {
                *prescaler = 2;
                *pTseg1 = 8;
                *pTseg2 = 1;
            }
            break;
        case BAUDRATE_4M:
            if (sPoint <= SP_75)
            {
                *prescaler = 1;
                *pTseg1 = 7;
                *pTseg2 = 2;
            }
            else if (sPoint <= SP_82_5)
            {
                *prescaler = 1;
                *pTseg1 = 7;
                *pTseg2 = 2;
            }
            else
            {
                *prescaler = 1;
                *pTseg1 = 8;
                *pTseg2 = 1;
            }
            break;
        case BAUDRATE_8M:
            if (sPoint <= SP_75)
            {
                *prescaler = 1;
                *pTseg1 = 3;
                *pTseg2 = 1;
            }
            else if (sPoint <= SP_82_5)
            {
                *prescaler = 1;
                *pTseg1 = 3;
                *pTseg2 = 1;
            }
            else
            {
                *prescaler = 1;
                *pTseg1 = 3;
                *pTseg2 = 1;
            }
            break;
        default:
            break;
    }
    return 0;
}

uint8_t blinkLedCan(uint8_t Led)
{
    switch (Led)
    {

        case GREEN_CAN_LED1:
            if (led1CanGreenBlink)
            {
                return 1;
            }
            else
            {
                led1CanGreenBlink = 1;
                return 0;
            }
            break;

        case GREEN_CAN_LED2:
            if (led2CanGreenBlink)
            {
                return 1;
            }
            else
            {
                led2CanGreenBlink = 1;
                return 0;
            }
        case RED_CAN_LED1:
            if (led1CanRedBlink)
            {
                return 1;
            }
            else
            {
                led1CanRedBlink = 1;
                return 0;
            }
        case RED_CAN_LED2:
            if (led2CanRedBlink)
            {
                return 1;
            }
            else
            {
                led2CanRedBlink = 1;
                return 0;
            }
        default:
            break;
    }
    return 1;
}

void CanLedTimerCallback(void)
{
    if (led1CanGreenBlink)
    {
        counter1CanBlinkGreen++;
        if (counter1CanBlinkGreen < GREEN_BLINK_DURATION)
            LED_CAN1_GREEN_OFF();
        else
            LED_CAN1_GREEN_ON();
        if (counter1CanBlinkGreen >= GREEN_BLINK_DURATION * 2)
            counter1CanBlinkGreen = led1CanGreenBlink = 0;
    }
    else if (led2CanGreenBlink)
    {
        counter2CanBlinkGreen++;
        if (counter2CanBlinkGreen < GREEN_BLINK_DURATION)
            LED_CAN2_GREEN_OFF();
        else
            LED_CAN2_GREEN_ON();
        if (counter2CanBlinkGreen >= GREEN_BLINK_DURATION * 2)
            counter2CanBlinkGreen = led2CanGreenBlink = 0;
    }
    else if (led1CanRedBlink)
    {
        counter1CanBlinkRed++;
        if (counter1CanBlinkRed < RED_BLINK_DURATION)
            LED_CAN1_RED_ON();
        else
            LED_CAN1_RED_OFF();
        if (counter1CanBlinkRed >= RED_BLINK_DURATION * 2)
            counter1CanBlinkRed = led1CanRedBlink = 0;
    }
    else if (led2CanRedBlink)
    {
        counter2CanBlinkRed++;
        if (counter2CanBlinkRed < RED_BLINK_DURATION)
            LED_CAN2_RED_ON();
        else
            LED_CAN2_RED_OFF();
        if (counter2CanBlinkRed >= RED_BLINK_DURATION * 2)
            counter2CanBlinkRed = led2CanRedBlink = 0;
    }

}

