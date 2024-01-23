/*
 * commControl.c
 *
 *  Created on: May 14, 2021
 *      Author: Karel Hevessy
 */

#include <commControl.h>
#include <usbd_cdc_if.h>    /* CDC_Transmit_HS() */
#include <cmsis_os2.h>
#include "config.h"         /* HW_VERSION */
#include "tcpServer.h"
#include "packetControl.h"

extern osMessageQueueId_t UsbRxQueueHandle;

static void sendToCan(uint8_t* pData, uint16_t length, uint8_t channel);


void TcpDataReceived(uint8_t* pData, uint16_t length)
{
  length = length > MAX_CMD_LEN ? MAX_CMD_LEN : length;
  sendToCan(pData, length, TCP_CHANNEL);
}

void UsbProtocolTask(void* arg)
{
  while (((USBD_CDC_HandleTypeDef*)hUsbDeviceHS.pClassData)->TxState != 0)
    osDelay(1000);

  while (1)
  {
    GenericMessageType usbRxBuffer;     /* Reception buffer */
    osStatus_t queueState;

    /* Get received data from queue */
    while ((queueState = osMessageQueueGet(UsbRxQueueHandle, &usbRxBuffer, NULL, 0xffff)) != osOK);
    uint16_t length = usbRxBuffer.Datalen > MAX_CMD_LEN ? MAX_CMD_LEN : usbRxBuffer.Datalen;
    sendToCan(usbRxBuffer.Data, length, USB_CHANNEL);
  }
}

void UsbDataReceived(uint8_t* pData, uint16_t length)
{
  GenericMessageType usbRxBuffer;
  length = length > MAX_CMD_LEN ? MAX_CMD_LEN : length;
  memcpy(usbRxBuffer.Data, pData, length);
  usbRxBuffer.Datalen = length;
  uint32_t timeout = (__get_IPSR() != 0U) ? 0 : QUEUE_PUT_TIMEOUT;
  /* Just put the data to queue */
  osMessageQueuePut(UsbRxQueueHandle, &usbRxBuffer, 0, timeout);
}

void sendToCan(uint8_t* pData, uint16_t length, uint8_t channel)
{
  uint8_t retCode=0;
  for (int i = 0; i < length; i+=8)
  {
    CanMessageStruct message;
    message.BRS = false;
    message.CANChannel  = CAN1_NUM;
    message.DLC = (length-i) > 8 ? 8 : (length-i);
    message.Data  = &pData[i];
    message.ESI = false;
    message.EXTId = false;
    message.FDF = false;
    message.Id  = 0x11;
    message.RTR = false;
    retCode = CanSendMessage(&message);
    while (retCode == CAN_FIFO_FULL)
    {
        osDelay(10);
        retCode = CanSendMessage(&message);
    }
  }
  return;
}

void WaitAndDrainQueue(uint8_t* pDest, uint16_t size, uint16_t* pResSize,
                       osMessageQueueId_t queueHandle)
{
    if ((NULL != pDest) && (NULL != pResSize))
    {
        GenericMessageType elem;
        osStatus_t getState;

        while ((getState = osMessageQueueGet(queueHandle, &elem, NULL, 0xffff)) != osOK);
        elem.Datalen = (elem.Datalen > MAX_CMD_LEN) ? MAX_CMD_LEN : elem.Datalen;
        *pResSize = elem.Datalen;
        memcpy(pDest, elem.Data, elem.Datalen);

        while (*pResSize < (size - MAX_CMD_LEN) &&
               (osOK == osMessageQueueGet(queueHandle, &elem, NULL, 0)))
        {
            elem.Datalen = (elem.Datalen > MAX_CMD_LEN) ? MAX_CMD_LEN : elem.Datalen;
            memcpy(pDest + *pResSize, elem.Data, elem.Datalen);
            *pResSize += elem.Datalen;
        }
    }
}
