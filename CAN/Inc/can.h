/*
 * can.h
 *
 *  Created on: 9. 8. 2021
 *      Author: petr_kolar
 */

#ifndef INC_CAN_H_
#define INC_CAN_H_

#include "stdbool.h"
#include "main.h"

#define CAN_MAX_DATALEN                 64

#define CAN1                            hfdcan3
#define CAN2                            hfdcan1

#define CAN1_NUM                        0
#define CAN2_NUM                        1

#define RX_FIFO_SIZE                    32
#define TX_FIFO_SIZE                    32

#define	MESSAGE_HEADER_SIZE             5
#define MESSAGE_HEADER_SIZE_EXT_ID      7
#define CONF_MESSAGE_SIZE_CAN           11

#define CAN_OK                          0
#define CAN_ERROR                       1
#define CAN_CHANNEL_NOT_IMPLEMENTED     2
#define CAN_WRONG_DLC                   3
#define CAN_WRONG_PROTOCOL              4
#define CAN_IS_NOT_RUNNING              5
#define CAN_INCORRECT_PARAMETER         6
#define CAN_FIFO_FULL                   7
#define CAN_IS_ALREADY_RUNNING          8

#define CORRECTION_CONST                80 //This is valid only for 40MHz CLK

#define FDCAN_CLOCK                     80000000    // Peripheral has 80 MHz clock

#define GREEN_BLINK_DURATION            100
#define RED_BLINK_DURATION              300

#define FDCAN_COMPENSATION_THRESHOLD    2000000 /* Delay compensation shall be used for Data baud rate above 2 Mbit */

#define GREEN_CAN_LED1                0
#define RED_CAN_LED1                  1
#define GREEN_CAN_LED2                2
#define RED_CAN_LED2                  3

#define TDC_FILTER  0x0

#include "stdbool.h"
#include "main.h"

/*
 * Baud rate - 125k to 8M
 */

typedef enum
{
    BAUDRATE_125k,
    BAUDRATE_250k,
    BAUDRATE_500k,
    BAUDRATE_1M,
    BAUDRATE_2M,
    BAUDRATE_4M,
    BAUDRATE_8M
} CanBaud;

typedef enum
{
    SP_60,
    SP_62_5,
    SP_65,
    SP_67_5,
    SP_70,
    SP_72_5,
    SP_75,
    SP_77_5,
    SP_80,
    SP_82_5,
    SP_85,
    SP_87_5,
    SP_90
} SamplePoint;

typedef enum
{
    NormalMode, SilentMode
} AcknowledgeMode;

typedef enum
{
    CAN, ISO_CAN_FD
} CanProtocol;

/*
 * Settings of the LIN channel - mode, checksum type, baud rate and amlr
 */
typedef struct
{
    CanProtocol Protocol;
    CanBaud DataBaud;
    CanBaud ArbitrationBaud;
    SamplePoint DataSPoint;
    SamplePoint ArbitrationSPoint;
    AcknowledgeMode Mode;
    uint8_t DataSJW;
    uint8_t ArbitrationSJW;
    bool AutoStart;
    bool PreciseTimingSet;
    uint16_t ArbitrationTimeSegment1;
    uint8_t ArbitrationTimeSegment2;
    uint8_t ArbitrationPrescaler;
    uint8_t DataTimeSegment1;
    uint8_t DataTimeSegment2;
    uint8_t DataPrescaler;
} CanInitStruct;


typedef struct
{
    bool FDF;
    bool ESI;
    bool BRS;
    bool RTR;
    bool EXTId;
    uint8_t CANChannel;
    uint8_t DLC;
    uint32_t Id;
    uint8_t *Data;
} CanMessageStruct;

extern bool canChannel1Run;
extern bool canChannel2Run;

extern uint8_t canChannel1ChannelToTX;
extern uint8_t canChannel2ChannelToTX;

extern bool canEchoTxChannel1;
extern bool canEchoTxChannel2;

extern bool canEchoRxChannel1;
extern bool canEchoRxChannel2;

/*
 * Start CAN channel
 */
uint8_t CanStartChannel(uint8_t channel);

/*
 * Stop CAN channel
 */
uint8_t CanStopChannel(uint8_t channel);

/*
 * Start CAN channel if is setup its autoStart bit.
 */
void CanAutoStart();

/*
 *  Configure CAN. For configuration is use CONFIGURATION_CAN structure.
 */
uint8_t ConfigureCAN(uint8_t channel);

/*
 *  Send CAN message.
 */
uint8_t CanSendMessage(CanMessageStruct *message);

//Auxiliary functions

/*
 * Convert integer value to HAL DLC CANFD value.
 */
uint8_t IntToDLC(uint8_t dataLength, uint32_t *DLC);

/*
 * 	Convert message data to output data.
 */
uint8_t ToOutputMessageFormat(uint8_t *data, CanMessageStruct *message);

/*
 *	Get can configuration values for specified baudrate.
 */
uint8_t BaudToParametrs(uint8_t baudRate, uint8_t *pSPoint, uint8_t *prescaler,
        uint16_t  *pTseg1, uint8_t *pTseg2, uint8_t dataBr);

/*
 * Convert HAL message to standard message.
 */
void RxMessageToFormat(FDCAN_RxHeaderTypeDef *RxHeader, uint8_t RxData[],
        CanMessageStruct *message);

/*
 *
 */
uint8_t blinkLedCan(uint8_t Led);



void CanLedTimerCallback(void);

#endif /* INC_CAN_H_ */
