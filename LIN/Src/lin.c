/*
 * lin.c
 *
 *  Created on: Sep 18, 2020
 *      Author: Karel Hevessy
 */

#include <string.h>   /* memset(), memcpy() */
#include "lin.h"
#include "lin_timer.h"
#include "lin_uart.h"
#include "packetControl.h"
#include "commControl.h"
#include "main.h"     /* Pin name definitions */
#include "config.h"

/* Current scheduler delay */
uint16_t LinDelay;

uint8_t linIsTransmitting;  /* 1 when the transmission is in progress */

uint8_t schedulerRunning;
uint8_t schedulerIndex;

LinScheduler scheduler;

/* Frame to send */
LIN_Frame txFrame;
uint8_t txFrameDataIndex;
/* Data to receive (raw data) */
uint8_t rxData[RX_BUFFER_LENGTH];
uint8_t rxDataIndex;

/* What part of the LIN message we will be transmitting */
LinState linTransmitState;

uint8_t linStopChannelRequestFlag;

/* Determines if we will transmit anything or will be only receiving */
uint8_t linStopTx;

/* One if we are receiving the slave response */
uint8_t linReceiving;

/* Response timeout counter variables */
uint16_t responseTimeoutValue;
uint16_t responseTimeoutMax;

/* Timeout of receiving break (in case of short of LIN to Vbat) */
uint8_t breakTmrRunning;
uint16_t breakTmrVal;

/* LIN channel settings - mode and so on */
LinInitStruct LinSettings;

/* TxFrameBuffer write index*/
uint8_t LinWriteIndex;

/* TxFrameBuffer read index*/
uint8_t LinReadIndex;

uint8_t linTransmittionPending;

uint8_t channelToResponse;

uint16_t slaveResponseMaxIndex; /* Number of used slave responses */
uint16_t slaveResponseIndex;    /* Index to the buffer we are currently working with */
uint8_t slaveResponseDataIndex; /* Index to currently received slave data (index to the data in the buffer) */

/* Buffers for slave responses */
SlaveResponseDataStruct SlaveResponse[LIN_ID_COUNT + 1];  /* One free space for other id's data */

/* Index to loopback control data */
uint8_t linControlDataIndex;

/* Frame for loopback checking of sent data and loopback control */
LinControlStruct linLoopbackControl;
LinState LinControlState;

/*  */
uint8_t restartLinUart;

/* Time counters for Led blink*/
uint16_t counterLinBlinkRed;
uint16_t counterLinBlinkGreen;

/* Led Flags for blinking */
bool ledLinRedBlink;
bool ledLinGreenBlink;

/* If true send frame asynchronous else send frame from TX buffer */
/* Note: in this implementation, all frames should be set synchronously from TxFrameBuffer */
uint8_t transmitAsyncSet;

/* Timeout of receiving brake (in case of short of LIN to Vbat) */
uint8_t breakTmrRunning;
uint16_t breakTmrVal;

uint8_t loopbackTmrRunning;
uint16_t loopbackTmrVal;

LIN_Frame TxFrameBuffer[LIN_FRAME_TX_BUFFER_LENGTH];    // Buffer of LIN Tx frames

bool linEchoTx = false;
bool linEchoRx = true;

uint8_t linChannelToTX = TCP_CHANNEL;

bool linRun;

extern CONFIGURATION_LIN configLin;

void LinErrorCallback(uint8_t linId, uint8_t error)
{
  blinkLedLin(RED_LIN_LED);
}

void SchedulerTimerCallback(void)
{
  /* LinMaintenance */
  if (linStopChannelRequestFlag && !linIsTransmitting  && !linTransmittionPending)
  {
      LinStop();
      LinChannelStoppedCallback();
  }
  if (restartLinUart)
  {
      restartLinUart = 0;
      LinUartReset();
  }

  if (schedulerRunning)
  {
    LinDelay--;
    if (LinDelay == 0)
    {
      // Note: asynchronous transmission cannot be used in this implementation
      //LinTransmitFrameAsync(&scheduler.Row[schedulerIndex].Frame);
      LIN_Frame frame = scheduler.Row[schedulerIndex].Frame;

      if (LinWriteToTxBuffer(frame.ID, frame.Type, frame.ChecksumType, frame.Length, frame.Data))
      {
        schedulerIndex = (schedulerIndex + 1) % scheduler.NumberOfFrames;
      }
      LinDelay = scheduler.Row[schedulerIndex].Delay;
    }
  }
}

void LinSchedulerStart(void)
{
  if (scheduler.NumberOfFrames)
  {
    linIsTransmitting = 0;
    schedulerIndex = 0;
    schedulerRunning = 1;
    LinDelay = scheduler.Row[schedulerIndex].Delay;
  }
}

void LinSchedulerStop(void)
{
  schedulerRunning = 0;
}

uint8_t InitLin(LinInitStruct ConfLinSettings)
{
#ifdef LIN_SLAVE_MODE
  #error Lin slave not implemented.
#endif

  if (linRun)
  {
	return 1;
  }

  // Set led
  LED_LIN_RED_OFF();
  LED_LIN_GREEN_ON();

  LinSettings.AMLR = ConfLinSettings.AMLR;
  LinSettings.Baudrate = ConfLinSettings.Baudrate;
  LinSettings.ChecksumType = ConfLinSettings.ChecksumType;
  LinSettings.DeviceMode = ConfLinSettings.DeviceMode;

  HAL_GPIO_WritePin(LIN_MASTER_GPIO_Port, LIN_MASTER_Pin, GPIO_PIN_SET);
  /* Initialize frames in the scheduler */
  initLinScheduler();

  /* Configure the uart and timer peripherals */
  InitLinUartBaudrate(LinSettings.Baudrate);
  LinTimeoutTimerInit(LinSettings.Baudrate);

  /* Configure the UART interrupts */
  LinUartEnableBreakInterrupt();
  LinUartEnableRxInterrupt();
  LinUartDisableTxInterrupt();
  linStopChannelRequestFlag = 0;
  resetStateMachine();
  linRun = true;
  LinWriteIndex = 0;
  LinReadIndex = 0;
  //LinSchedulerStart();
  return 0;
}

void initLinScheduler(void)
{
  scheduler.Row[0].Frame.ID = 0x1;
  scheduler.Row[0].Frame.Length = 4;
  memset((void*) scheduler.Row[0].Frame.Data, 0x0, scheduler.Row[0].Frame.Length);
  scheduler.Row[0].Frame.ChecksumType = ENHANCED_CHECKSUM;
  scheduler.Row[0].Frame.Type = MASTER_RESPONSE;
  scheduler.Row[0].Delay = 25;

  // Master request would look like this
//  scheduler.Row[1].Frame.ID = 0x2;
//  scheduler.Row[1].Frame.ResponseLength = 1;
//  scheduler.Row[1].Frame.ChecksumType = ENHANCED_CHECKSUM;
//  scheduler.Row[1].Frame.Type = MASTER_REQUEST;
//  scheduler.Row[1].Delay = 50;

  scheduler.NumberOfFrames = 1;
}

uint8_t LinTransmitFrameAsync(LIN_Frame* pFrame)
{
  uint8_t ret = 0;

  if (!linIsTransmitting && !linReceiving)
  {
    transmitAsyncSet=1;
    /* We can begin sending the data */
    txFrame.ID = getParityIdentifier(pFrame->ID);
    if (pFrame->Type == MASTER_RESPONSE)
      memcpy((void*) txFrame.Data, pFrame->Data, pFrame->Length);

    if (pFrame->Type == MASTER_REQUEST)
      txFrame.ResponseLength = 0; /* No response yet */

    txFrame.Length = pFrame->Length;
    txFrame.Type = pFrame->Type;
    txFrame.ChecksumType = pFrame->ChecksumType;
    if (pFrame->ChecksumType == CHECKSUM_CLASSICAL || pFrame->ID == 0x3c || pFrame->ID == 0x3d || pFrame->ID == 0x3e || pFrame->ID == 0x3f)
    {
      txFrame.Checksum = calcChecksum(pFrame->Data, pFrame->Length,
                                        CHECKSUM_CLASSICAL, pFrame->ID);
    }
    else
    {
      txFrame.Checksum = calcChecksum(pFrame->Data, pFrame->Length,
                                        pFrame->ChecksumType, pFrame->ID);
    }

    /* Reset the index for receiving the data */
    rxDataIndex = 0;
    linTransmitState = STATE_SYNC_BREAK;
    linIsTransmitting = 1;  /* Transmission in progress */
    linStopTx = 0;
    LinUartEnableTxInterrupt();
    ret = 1;
  }
  return ret;
}

void LinTimeoutTimerCallback(void)
{
  if (breakTmrRunning)
  {
      /* Waiting for break - in case when LIN is shorted to Vbat */
      breakTmrVal++;
      if (breakTmrVal >= BREAK_TIMEOUT_MAX)
      {
          LinTimeoutTimerStop();
          breakTmrVal = breakTmrRunning = 0;
          LinReadIndex = LinWriteIndex = 0;
          linResetAll();
      }
  }
  else if (loopbackTmrRunning)
  {
    /* Waiting for loopback */
    loopbackTmrVal++;
    if (loopbackTmrVal >= BREAK_TIMEOUT_MAX)
    {
      LinTimeoutTimerStop();
      loopbackTmrVal = loopbackTmrRunning = 0;
      LinReadIndex = LinWriteIndex = 0;
      linResetAll();
    }
  }
  else if (linReceiving) /* Timeout for response running */
  {
    if (responseTimeoutValue < responseTimeoutMax)
      responseTimeoutValue++;
    else
    {
      LinTimeoutTimerStop();
      linReceiving = 0;

      if (LinSettings.DeviceMode == LIN_MODE_MASTER)
      {
        linIsTransmitting = 0;
        linTransmittionPending = 0;

        /* Hard-coded length of messages - timeout occurred */
        if (LinSettings.AMLR == HARDCODED)
        {
          LinErrorCallback(TxFrameBuffer[LinReadIndex].ID, PROTOCOL_ERR_LIN_TIMEOUT_OVERRUN);
        }
        else  /* Variable message length - timeout did not have to occur */
        {
          /* Response length is with the sync + id bytes and checksum ... some data received*/
          if (TxFrameBuffer[LinReadIndex].ResponseLength > 3)
          {
            if (processMasterReceivedFrame(&TxFrameBuffer[LinReadIndex], rxData))
              MasterRequestTxRxCallback(&TxFrameBuffer[LinReadIndex]);
            else
              LinErrorCallback(TxFrameBuffer[LinReadIndex].ID & 0x3f, PROTOCOL_ERR_LIN_CHECKSUM);
          }
          /* No data received */
          else
          {
            LinErrorCallback(TxFrameBuffer[LinReadIndex].ID & 0x3f, PROTOCOL_ERR_LIN_TIMEOUT_OVERRUN);
          }
        }

        transmitAsyncSet=0;
        if (LinReadIndex != LinWriteIndex)
          LinReadIndex++;

        if(LinReadIndex>=LIN_FRAME_TX_BUFFER_LENGTH)
        {
          LinReadIndex = 0;
        }

        if((LinReadIndex!=LinWriteIndex) && (linStopChannelRequestFlag == 0) )
        {
          linIsTransmitting = 1;
        }
        else
        {
           if(linStopChannelRequestFlag == 0)
           {
               linIsTransmitting = 0;
           }
           else
           {
               LinStop();
               LinChannelStoppedCallback();
           }
        }
      }
    } /* Timeout has elapsed */
  } /* linReceiving */
}

void LinRxCallback(uint8_t byteReceived)
{
  rxData[rxDataIndex] = byteReceived;

  if (linLoopbackControl.Enabled)
  {
    loopbackStateMachine(byteReceived);
  }
  else /* loopback disabled */
  {
    if (TxFrameBuffer[LinReadIndex].Type == MASTER_REQUEST && linReceiving)
    {
      /* Note: ResponseLength is without the sync break */
      TxFrameBuffer[LinReadIndex].ResponseLength++;

      if (LinSettings.AMLR == HARDCODED)
      {
        /* Last byte received */
        if (rxDataIndex >= TxFrameBuffer[LinReadIndex].Length + 4 - 1)
        {
          /* length = data bytes + break 1 B + sync 1 B + id 1 B + checksum 1 B - 1 (it is an index) */
          linTransmittionPending = 0;
          linReceiving = 0;
          if (processMasterReceivedFrame(&TxFrameBuffer[LinReadIndex], rxData))
            MasterRequestTxRxCallback(&TxFrameBuffer[LinReadIndex]);
          else
            LinErrorCallback(TxFrameBuffer[LinReadIndex].ID, PROTOCOL_ERR_LIN_CHECKSUM);

          if (LinReadIndex != LinWriteIndex)
            LinReadIndex = (LinReadIndex + 1) % LIN_FRAME_TX_BUFFER_LENGTH;

          if ((LinReadIndex != LinWriteIndex) && (linStopChannelRequestFlag == 0))
          {
            linIsTransmitting = 1;
          }
          else
          {
            if (linStopChannelRequestFlag == 0)
            {
                linIsTransmitting = 0;
            }
            else
            {
              LinStop();
              LinChannelStoppedCallback();
            }
          }
        }
      }
      else  /* AMLR == AUTOLEN */
      {
        responseTimeoutValue = 0; /* Auto message length detection - just collect data */
      }
    }
  }

  rxDataIndex++;
  rxDataIndex %= RX_BUFFER_LENGTH;
}

uint8_t loopbackStateMachine(uint8_t byteReceived)
{
  uint8_t LinControlError = 0;

  switch (LinControlState)
  {
    case STATE_SYNC_BREAK:
      LinControlState = STATE_SYNC_FIELD;
      if (byteReceived != 0x00)
      {
        LinControlError = 1;
      }
      break;

    case STATE_SYNC_FIELD:
      LinControlState = STATE_ID;
      if (byteReceived != 0x55)
      {
        LinControlError = 1;
      }
      break;

    case STATE_ID:
      LinControlState = STATE_DATA;
      linControlDataIndex = 0;
      if (byteReceived != getParityIdentifier(linLoopbackControl.LinControlFrame.ID))
      //if (byteReceived != linLoopbackControl.LinControlFrame.ID)
      {
        LinControlError = 1;
      }
      break;

    case STATE_DATA:
      if (byteReceived != linLoopbackControl.LinControlFrame.Data[linControlDataIndex])
      {
        LinControlError = 1;
      }
      linControlDataIndex++;
      if (linControlDataIndex >= linLoopbackControl.LinControlFrame.Length)
      {
        LinControlState = STATE_CHECKSUM;
      }
      break;

    case STATE_CHECKSUM:
      LinControlState = STATE_SYNC_BREAK;
      linLoopbackControl.Enabled = 0; /* After checksum, disable loopback control */
      if (loopbackTmrRunning)
      {
        loopbackTmrRunning = 0;
        LinTimeoutTimerStop();
      }

      if (byteReceived != linLoopbackControl.LinControlFrame.Checksum)
      {
        LinControlError = 1;
      }

      if (!LinControlError)
      {
        linTransmittionPending = 0;     // Transmission ended, silent on bus
        MasterResponseTxCallback(&linLoopbackControl.LinControlFrame);
        if (LinReadIndex != LinWriteIndex)
            LinReadIndex++;

        if(LinReadIndex>=LIN_FRAME_TX_BUFFER_LENGTH)
        {
          LinReadIndex = 0;
        }
        if ((LinReadIndex != LinWriteIndex) && (linStopChannelRequestFlag == 0))
        {
          linIsTransmitting = 1;
        }
        else
        {
          if (linStopChannelRequestFlag == 0)
          {
              linIsTransmitting = 0;
          }
          else
          {
              LinStop();
              LinChannelStoppedCallback();
          }
        }
      }
      break;

    /* Invalid state*/
    default:
      LinControlError = 1;
      break;
  }

  if (LinControlError) /* Some error happened */
  {
    LinControlError = 0;
    linTransmittionPending = 0;
    /* We haven't received what we sent, skip the packet */
    resetStateMachine();
    LinErrorCallback(SlaveResponse[slaveResponseIndex].LinID, PROTOCOL_ERR_LIN_BUS_COLLISION); /* Report bus error */

    if (LinReadIndex != LinWriteIndex)
    {
        LinReadIndex++;                         // skip the packet
        if (LinReadIndex >= LIN_FRAME_TX_BUFFER_LENGTH)
            LinReadIndex = 0;
    }
    if ((LinReadIndex!=LinWriteIndex) && (linStopChannelRequestFlag == 0))
    {
        linIsTransmitting = 1;
    }
    else
    {
      if (linStopChannelRequestFlag == 0)
      {
        linIsTransmitting = 0;
      }
      else
      {
        LinStop();
        LinChannelStoppedCallback();
      }
    }
  }
  return LinControlError;
}

uint8_t GetConfigurationLin(void)
{
    uint8_t linConfigurationByte = 0;
    linConfigurationByte |= (configLin.LinConfiguration.Baudrate + 1);
    if (configLin.LinConfiguration.DeviceMode == LIN_MODE_MASTER)
    {
    	linConfigurationByte |= 4;
    }
    linConfigurationByte |= (configLin.LinConfiguration.autoStart<<4);
    linConfigurationByte |= (configLin.LinConfiguration.AMLR << 5);
    linConfigurationByte |= (configLin.LinConfiguration.ChecksumType << 6);

    return linConfigurationByte;
}

void LinTxCallback(void)
{
  LIN_Frame *txFrameToSend;
  if (transmitAsyncSet)
  {
    txFrameToSend = &txFrame;
  }
  else
  {
    txFrameToSend = &TxFrameBuffer[LinReadIndex];
  }

  if (LinSettings.DeviceMode == LIN_MODE_MASTER)
  {
    if (linStopTx) /* Stop transmitting (done or waiting for the response) */
    {
      if (txFrameToSend->Type == MASTER_REQUEST)
      {
        linReceiving = 1;
        linIsTransmitting = 0;
        if (LinSettings.AMLR == HARDCODED){
          txFrameToSend->Length = txFrameToSend->ID & 0x20 ? (txFrameToSend->ID & 0x10 ? 8:4) : 2;
        }
        calcAndUpdateTimeoutValue(txFrameToSend->Length);
        timeoutTimerStart();
        /* Disable the tx interrupt */
        LinUartDisableTxInterrupt();
      }
      else
      {
        linIsTransmitting = linReceiving = 0;
        /* Disable the tx interrupt */
        LinUartDisableTxInterrupt();
      }
    }
    else /* Continue with the transmission */
    {
      linTransmittionPending = 1;
      switch (linTransmitState)
      {
        case STATE_SYNC_BREAK:
          LinUartSendBreak();
          linTransmitState = STATE_SYNC_FIELD;
          /* Trigger timer waiting for break (for the situation when LIN is shorted to Vcc) */
          breakTmrRunning = 1;
          breakTmrVal = 0;
          timeoutTimerStart();
          LinControlState = STATE_SYNC_BREAK;
          linLoopbackControl.Enabled = 1;
          /* Disable the tx interrupt until the break appears */
          LinUartDisableTxInterrupt();
          break;

        case STATE_ID:
          if (txFrameToSend->Type == MASTER_RESPONSE)
          {
            if (txFrameToSend->Length != 0)
            {
              linTransmitState = STATE_DATA;
              txFrameDataIndex = 0;
            }
            else
              linTransmitState = STATE_CHECKSUM;
          }
          else if (txFrameToSend->Type == MASTER_REQUEST)
          {
            linTransmitState = STATE_SYNC_BREAK;
            /* Do not transmit anything else, wait for response */
            linStopTx = 1;
          }
          else
            linResetAll();

          /* Continue only if reset did not happen */
          if (txFrameToSend->Type == MASTER_REQUEST || txFrameToSend->Type == MASTER_RESPONSE)
          {
            LinUartPutChar(txFrameToSend->ID);
            /* For master request, we do not need loopback */
            linLoopbackControl.Enabled = !linStopTx;
            if (!linStopTx)
            {
              /* Set up the parameters needed by the loopback */
              linLoopbackControl.LinControlFrame.ID = txFrameToSend->ID;
              linLoopbackControl.LinControlFrame.Length = txFrameToSend->Length;
            }
          }
          break;

        case STATE_DATA:
          linLoopbackControl.LinControlFrame.Data[txFrameDataIndex] = txFrameToSend->Data[txFrameDataIndex];
          if (txFrameDataIndex + 1 >= txFrameToSend->Length)
          {
            /* Last data byte will be transmitted, then the checksum */
            linTransmitState = STATE_CHECKSUM;
          }
          LinUartPutChar(txFrameToSend->Data[txFrameDataIndex]);
          txFrameDataIndex++;
          break;

        case STATE_CHECKSUM:
          linLoopbackControl.LinControlFrame.Checksum = txFrameToSend->Checksum;
          linTransmitState = STATE_SYNC_BREAK;
          linStopTx = 1;  /* Last byte will be sent */
          timeoutTimerStart();
          loopbackTmrRunning = 1;
          LinUartPutChar(txFrameToSend->Checksum);
          break;

        default: /* Bad state */
          linResetAll();
          break;
      }
    }
  }
}

void LinLineBreakCallback(void)
{
  if (linTransmitState == STATE_SYNC_FIELD)
  {
    /* Stop the timer waiting for line break */
    LinTimeoutTimerStop();
    breakTmrRunning = 0;
    /* Enable the tx interrupt again */
    LinUartEnableTxInterrupt();
    linTransmitState = STATE_ID;
    LinUartPutChar(0x55);
  }
  else
  {
    linResetAll();
  }
}

void linResetAll(void)
{
  LinTimeoutTimerStop();
  LinUartReset();
  resetStateMachine();
  linTransmittionPending = 0;
}

void resetStateMachine(void)
{
  linStopTx = linIsTransmitting = linReceiving = linTransmittionPending = 0;
  rxDataIndex = 0;
  linTransmitState = STATE_SYNC_BREAK;
}

void LinFramingErrorCallback(void)
{
  if (LinSettings.DeviceMode == LIN_MODE_MASTER)
  {
    if (linTransmitState == STATE_SYNC_FIELD)
    {
      /* Enable the tx interrupt again */
      LinUartEnableTxInterrupt();
      linTransmitState = STATE_ID;
      LinUartPutChar(0x55);
    }
    else
    {
      /* Line break detected when not wanted - error */
      linResetAll();
    }
  }
}

uint8_t getParityIdentifier(uint8_t id)
{
   uint8_t newId = id & 0x3Fu;
   newId |= (  (id >> 0u & 0x1u) ^ (id >> 1u & 0x1u)
             ^ (id >> 2u & 0x1u) ^ (id >> 4u & 0x1u)) << 6u;
   newId |= (  (id >> 1u & 0x1u) ^ (id >> 3u & 0x1u)
             ^ (id >> 4u & 0x1u) ^ (id >> 5u & 0x1u) ^ 0x1u) << 7u;
   return newId;
}

uint8_t calcChecksum(uint8_t* pAddress, uint8_t size, uint8_t checksumType,
                     uint8_t linId)
{
  uint16_t sum = 0x0000;
  uint8_t i = 0;
  if (ENHANCED_CHECKSUM == checksumType)
  {
    sum += getParityIdentifier(linId);
  }

  for (i = 0; i < size; i++)
    sum += pAddress[i];

  sum = (sum & 0x00FFu) + ((sum & 0xFF00u) >> 8u);
  if (sum & 0xFF00u) /* Did adding the carry bits result in a carry? */
  {
    sum += 1;  /* Add the last carry */
  }

   sum &= 0x00FFu;
   return (uint8_t)(~sum);
}

uint8_t UpdateTxFrameData(uint8_t ID, uint8_t* pData, uint8_t size)
{
  uint8_t ret = 0;
  /* Prevent buffer overflow */
  if (size > LIN_DATA_LENGTH)
    size = LIN_DATA_LENGTH;

  /* Try to find the frame and update it */
  for (uint8_t i = 0; i < scheduler.NumberOfFrames && i < MAX_SCHEDULER_LENGTH; i++)
  {
    if (scheduler.Row[i].Frame.ID == ID && scheduler.Row[i].Frame.Type == MASTER_RESPONSE)
    {
      memcpy(scheduler.Row[i].Frame.Data, pData, size);
      scheduler.Row[i].Frame.Length = size;
      ret = 1;
      break;
    }
  }
  return ret;
}

LIN_Frame* FindRxFrame(uint8_t ID)
{
  LIN_Frame* ret = NULL;
  /* Try to find the frame */
  for (uint8_t i = 0; i < scheduler.NumberOfFrames && i < MAX_SCHEDULER_LENGTH; i++)
  {
    if (scheduler.Row[i].Frame.ID == ID && scheduler.Row[i].Frame.Type == MASTER_REQUEST)
    {
      ret = &scheduler.Row[i].Frame;
      break;
    }
  }
  return ret;
}

void timeoutTimerStart(void)
{
  responseTimeoutValue = 0;

  /* Reset the counter value and enable the timer and its interrupt */
  LinTimeoutTimerStart();
}

void timeoutTimerStop(void)
{
  /* Stop the timer and its interrupt */
  LinTimeoutTimerStop();
}

int16_t LinGetFreeTx(void)
{
    int16_t r;
    r = (int16_t) LinReadIndex - (int16_t) LinWriteIndex;
    if (r <= 0)
        r += LIN_FRAME_TX_BUFFER_LENGTH;
    return r - 1;
}

uint8_t LinWriteToTxBuffer(uint8_t linId,uint8_t msgType,uint8_t chksumType, uint8_t length, uint8_t data[])
{
    uint8_t i;
    linLoopbackControl.ExpectedDataLength = 0;
    linLoopbackControl.ReceivedDataLength = 0;

    if (LinGetFreeTx() >= 1)
    {
      TxFrameBuffer[LinWriteIndex].Type = msgType;
      TxFrameBuffer[LinWriteIndex].ID = getParityIdentifier(linId);
      TxFrameBuffer[LinWriteIndex].Length = length;
      TxFrameBuffer[LinWriteIndex].ChecksumType = chksumType;

      if (msgType == MASTER_RESPONSE)
      {
        for (i = 0; i < length; i++)
        {
          TxFrameBuffer[LinWriteIndex].Data[i] = data[i];
        }
        if ((linId == 0x3c) || (linId == 0x3d) || linId == 0x3e || linId == 0x3f)
        {
          TxFrameBuffer[LinWriteIndex].Checksum = calcChecksum(&data[0], length, CLASSICAL_CHECKSUM, linId);
        }
        else
        {
          TxFrameBuffer[LinWriteIndex].Checksum = calcChecksum(&data[0], length, chksumType, linId);
        }

      }

      if (msgType == MASTER_REQUEST)
        TxFrameBuffer[LinWriteIndex].ResponseLength = 0; /* No response yet */

      LinWriteIndex++;

      if (linTransmittionPending == 0)
      {
        rxDataIndex = 0;
        linTransmitState = STATE_SYNC_BREAK;
        linIsTransmitting = 1;
        linStopTx = 0;
        LinUartEnableTxInterrupt();
      }

      if (LinWriteIndex >= LIN_FRAME_TX_BUFFER_LENGTH)
      {
        LinWriteIndex = 0;
      }
      return 1;
    }
    else
    {
      return 0;
    }
}

void calcAndUpdateTimeoutValue(uint8_t numberOfBytes)
{
  responseTimeoutMax = (numberOfBytes + 1) * 14 + 1;
}

void MasterRequestTxRxCallback(LIN_Frame* pFrame)
{
  blinkLedLin(GREEN_LIN_LED);
  /* Try to find correct buffer in the scheduler for the response */
  LIN_Frame* pFr = FindRxFrame(pFrame->ID & 0x3f);
  /* Copy the data to the scheduler frame buffer */
  if (pFr != NULL)
    memcpy(pFr->Data, pFrame->Data, pFrame->Length);

  if (pFrame->ResponseLength > 0)
  {
  }
}

void MasterResponseTxCallback(LIN_Frame* pFrame)
{
  blinkLedLin(GREEN_LIN_LED);
}

uint8_t processMasterReceivedFrame(LIN_Frame* pTxFrame, uint8_t* pRxData)
{
  blinkLedLin(GREEN_LIN_LED);
  uint8_t ret = 0;
  /* Correct checksum - data successfully received */
  /* Prevent buffer overflow */
  pTxFrame->ResponseLength %= RX_BUFFER_LENGTH;
  uint8_t index = 3;
  index %= RX_BUFFER_LENGTH;
  LinChecksum checksumType=pTxFrame->ChecksumType;
  uint8_t unprotectID=pTxFrame->ID & 0x3f;
  if (unprotectID == 0x3c ||unprotectID == 0x3d || unprotectID == 0x3e ||unprotectID == 0x3f ){
    checksumType = CHECKSUM_CLASSICAL;
  }
  if (pRxData[pTxFrame->ResponseLength] == calcChecksum(pRxData + index, pTxFrame->ResponseLength - 3,
      checksumType, pTxFrame->ID))
  {
    pTxFrame->ResponseLength -= 3;
    /* We know that LIN_DATA_LENGTH < RX_BUFFER_LENGTH - 3*/
    for (uint8_t i = 0; i < pTxFrame->ResponseLength && i < LIN_DATA_LENGTH; i++)
      pTxFrame->Data[i] = rxData[i + 3];
    ret = 1;
  }
  return ret;
}

uint8_t LinStop(void)
{
  linRun = false;
  LinUartDisableRxInterrupt();
  LinUartDisableTxInterrupt();
  LinUartDisableBreakInterrupt();
  LinTimeoutTimerStop();
  linReceiving = 0;
  linTransmittionPending = 0;
  linStopChannelRequestFlag = 0;
  ledLinRedBlink = 0;
  ledLinGreenBlink = 0;
  return 0;
}

void LinAutoStart(void)
{
  if (GetConfigurationLIN().LinConfiguration.autoStart)
  {
    InitLin(GetConfigurationLIN().LinConfiguration);
  }
}

uint8_t LinStopChannelRequest(uint8_t linChannel, uint8_t channelToTXResponse)
{
  channelToResponse=channelToTXResponse;
  switch (linChannel)
  {
    case 0:
      linStopChannelRequestFlag = 1;
      return 0;
      break;

    default:
      return 1;
      break;
  }
}

void LinChannelStoppedCallback(void)
{
  // Set led
  LED_LIN_RED_OFF();
  LED_LIN_GREEN_OFF();
}

void LinLedTimerCallback(void)
{
  if (ledLinGreenBlink)
  {
    counterLinBlinkGreen++;
    if (counterLinBlinkGreen < GREEN_BLINK_DURATION)
      LED_LIN_GREEN_OFF();
    else
      LED_LIN_GREEN_ON();
    if (counterLinBlinkGreen >= GREEN_BLINK_DURATION*2)
      counterLinBlinkGreen = ledLinGreenBlink=0;
  }
  else if (ledLinRedBlink)
  {
    counterLinBlinkRed++;
    if (counterLinBlinkRed < RED_BLINK_DURATION)
      LED_LIN_RED_ON();
    else
      LED_LIN_RED_OFF();
    if (counterLinBlinkRed >= RED_BLINK_DURATION*2)
      counterLinBlinkRed = ledLinRedBlink=0;
  }
}
uint8_t blinkLedLin(uint8_t Led)
{
  switch(Led)
  {
    case RED_LIN_LED:
      if (ledLinRedBlink)
      {
        return 1;
      }
      else
      {
        ledLinRedBlink=1;
        return 0;
      }
      break;

    case GREEN_LIN_LED:
      if (ledLinGreenBlink)
      {
        return 1;
      }
      else
      {
        ledLinGreenBlink = 1;
        return 0;
      }
      break;

    default:
      break;
  }
  return 1;
}
