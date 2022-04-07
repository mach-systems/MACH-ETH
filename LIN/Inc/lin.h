/*
 * lin.h
 *
 *  Created on: Sep 18, 2020
 *      Author: Karel Hevessy
 */

#ifndef INC_LIN_H_
#define INC_LIN_H_

#include <stdint.h>
#include "stdbool.h"

/* Slave mode not implemented */
#define LIN_MASTER_MODE

/*
 * Hardware defines
 */
#define LIN_UART                USART3


/*
 * LIN frame defines
 */
#define LIN_DATA_LENGTH       8

#define CLASSICAL_CHECKSUM    0
#define ENHANCED_CHECKSUM     1

#define MASTER_REQUEST        1
#define MASTER_RESPONSE       2

#define MAX_SCHEDULER_LENGTH  10
#define RX_BUFFER_LENGTH      32

/* Timeout of receiving the break character */
#define BREAK_TIMEOUT_MAX     1000

#define LIN_ID_COUNT          64

#define BUFFER_DIR_RX         0
#define BUFFER_DIR_TX         1

#define GREEN_BLINK_DURATION  100
#define RED_BLINK_DURATION    300

#define GREEN_LIN_LED         0
#define RED_LIN_LED           1

#define LIN_FRAME_TX_BUFFER_LENGTH 10

/*
 * Slave frame data struct
 */
typedef struct
{
    uint8_t RxIndex;
    uint8_t LinID;
    uint8_t BufferDir : 1;    /* Rx or Tx */
    uint8_t RxDataLength;
    uint8_t TxDataLength;
    uint8_t Data[LIN_DATA_LENGTH + 1];
    uint8_t Checksum;
    uint8_t ChecksumType;
} SlaveResponseDataStruct;

/*
 * LIN frame structure
 */
typedef struct
{
    uint8_t ID;
    uint8_t Data[LIN_DATA_LENGTH];
    uint8_t Length;
    uint8_t Checksum;
    uint8_t ChecksumType;
    uint8_t Type;
    uint8_t ResponseLength;
} LIN_Frame;

/*
 * LIN scheduler row
 */
typedef struct
{
  LIN_Frame Frame;
  uint16_t Delay;
} SchedulingTableRow;

/*
 * LIN scheduler structure
 */
typedef struct
{
  SchedulingTableRow Row[MAX_SCHEDULER_LENGTH];
  uint8_t NumberOfFrames;
} LinScheduler;

/*
 * LIN transmission states
 */
typedef enum
{
  STATE_SYNC_BREAK,
  STATE_SYNC_FIELD,
  STATE_ID,
  STATE_DATA,
  STATE_CHECKSUM,
  STATE_END
} LinState;

/*
 * LIN mode - Master or Slave
 */
typedef enum
{
  LIN_MODE_MASTER,
  LIN_MODE_SLAVE,
  LIN_SNIFFER_MODE
} LinMode;

/*
 * LIN AMLR - hardcoded or not
 */
typedef enum
{
  HARDCODED,
  AUTOLEN
} LinAmlr;

/*
 * Baud rate - 9600 or 19200
 */
typedef enum
{
  BAUDRATE_9600,
  BAUDRATE_19200
} LinBaud;

/*
 * Checksum type - classical or enhanced
 */
typedef enum
{
  CHECKSUM_CLASSICAL,
  CHECKSUM_ENHANCED
} LinChecksum;

/*
 * Settings of the LIN channel - mode, checksum type, baud rate and amlr
 */
typedef struct
{
  LinMode DeviceMode;
  LinChecksum ChecksumType;
  LinBaud Baudrate;
  LinAmlr AMLR;
  bool autoStart;
  bool payload;
} LinInitStruct;

typedef struct
{
  LIN_Frame LinControlFrame;
  uint8_t Enabled;                        /* Enable or disable the loopback check */
  uint8_t ExpectedDataLength;
  uint8_t ReceivedDataLength;
} LinControlStruct;


extern uint8_t linChannelToTX;

extern bool linEchoTx;
extern bool linEchoRx;

extern bool linRun;

/*
 * Should be called with period of 1 ms.
 */
void SchedulerTimerCallback(void);

/*
 * Start the LIN scheduler.
 */
void LinSchedulerStart(void);

/*
 * Stop the LIN scheduler.
 */
void LinSchedulerStop(void);

/*
 * Initialize the LIN hardware and internal variables.
 */
uint8_t InitLin(LinInitStruct ConfLinSettings);

/*
 * Initialize the (hard coded) scheduler.
 */
void initLinScheduler(void);

/*
 * Begin transmission of the frame (for the master only).
 * NOTE: this function cannot be used together with LinWriteToTxBuffer()
 */
uint8_t LinTransmitFrameAsync(LIN_Frame* pFrame);

/*
 * Timeout timer period elapsed.
 */
void LinTimeoutTimerCallback(void);

/*
 * Callback for UART rx interrupt.
 */
void LinRxCallback(uint8_t byteReceived);

/*
 * Callback for UART tx interrupt.
 */
void LinTxCallback(void);

/*
 * Callback for UART line break detection.
 */
void LinLineBreakCallback(void);

/*
 * Reset the LIN uart peripheral and all the state variables.
 */
void linResetAll(void);

/*
 * Reset all the state variables
 */
void resetStateMachine(void);

/*
 * Callback for UART framing error.
 */
void LinFramingErrorCallback(void);

/*
 * Calculate the LIN PID.
 */
uint8_t getParityIdentifier(uint8_t id);

/*
 * Calculate the checksum
 */
uint8_t calcChecksum(uint8_t* pAddress, uint8_t size, uint8_t checksumType,
                     uint8_t linId);

/*
 * Update data of Master Response frame with ID. Return 1 if success, 0 if frame
 * could not be found.
 */
uint8_t UpdateTxFrameData(uint8_t ID, uint8_t* pData, uint8_t size);

/*
 * Try to find the Master Request frame with ID. Return pointer to the frame when
 * found, NULL when not found.
 */
LIN_Frame* FindRxFrame(uint8_t ID);

/*
 * Initialize the timer value variable and start the timer.
 */
void timeoutTimerStart(void);

/*
 * Stop the timer (just calls LinTimeoutTimerStop(); is here just to be
 * consistent with timeoutTimerStart()).
 */
void timeoutTimerStop(void);

/*
 * Calculate the timeout value for some number of bytes and update it.
 */
void calcAndUpdateTimeoutValue(uint8_t numberOfBytes);

/*
 * Some error happened callback
 */
void LinErrorCallback(uint8_t linId, uint8_t error);

/*
 * Common FSM for master and slave loop back (control that the sent data were
 * really seen on the bus).
 */
uint8_t loopbackStateMachine(uint8_t byteReceived);

/*
 * Master received a response.
 */
void MasterRequestTxRxCallback(LIN_Frame* pFrame);

/*
 * Master successfully sent a frame on the bus.
 */
void MasterResponseTxCallback(LIN_Frame* pFrame);

/*
 * Callback when something bad happens on LIN bus - parameter error is one of
 * the error defines (see above).
 */
void LinErrorCallback(uint8_t linId, uint8_t error);

/*
 * Process the master received data. Common for both automatic length recognition
 * and hard-coded length parts.
 * We must do this shifting because the UART module will receive everything
 * including the sync break and sync field.
 */
uint8_t processMasterReceivedFrame(LIN_Frame* pTxFrame, uint8_t* pRxData);

/*
 *  This function is call on start of program. It checks the LIN autostart bit and start LIN bus if is set.
 */
void LinAutoStart(void);

/*
 *  Stop the LIN bus.
 */
uint8_t LinStop();

/*
 * Process the configuration to configuration LIN register
 */
uint8_t GetConfigurationLin(void);

/*
 * Write Lin Frame to TX Buffer
 */
uint8_t LinWriteToTxBuffer(uint8_t linId,uint8_t msgType,uint8_t chksumType, uint8_t length, uint8_t data[]);

/*
 * Set linStopChannelRequestFlag to true -> request to stop LIN is set
 */
uint8_t LinStopChannelRequest(uint8_t linChannel,uint8_t channelToTXResponse);

/*
 * Call when LIN is stopped
 */
void LinChannelStoppedCallback();

void LinLedTimerCallback();

uint8_t blinkLedLin(uint8_t Led);



#endif /* INC_LIN_H_ */
