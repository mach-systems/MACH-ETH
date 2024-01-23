/*
 * spiControl.h
 *
 * This module manages the TCP received messages.
 *
 *  Created on: May 14, 2021
 *      Author: Karel Hevessy
 */

#ifndef INC_COMMCONTROL_H_
#define INC_COMMCONTROL_H_


#include <stdint.h>
#include <cmsis_os2.h>


/* LIN errors */
#define PROTOCOL_ERR_LIN_CHECKSUM           0xB0    /* Bad checksum received */
#define PROTOCOL_ERR_LIN_BUS_COLLISION      0xB1    /* Loopback control failed - probably bus collision */
#define PROTOCOL_ERR_LIN_TIMEOUT_OVERRUN    0xB2    /* Some timer overran */
#define PROTOCOL_ERR_LIN_BUFFER_FULL        0xB3    /* Buffer for slave messages is full */

/* CAN errors */
#define PROTOCOL_ERR_CAN_CRC                0xC0    /* CAN CRC error */
#define PROTOCOL_ERR_CAN_BIT                0xC1    /* CAN Bit error */
#define PROTOCOL_ERR_CAN_FORM               0xC2    /* CAN Form error */
#define PROTOCOL_ERR_CAN_AC                 0xC3    /* CAN Acknowledge error */
#define PROTOCOL_ERR_CAN_STUFF              0xC4    /* CAN Bit Stuff error */

#define COMM_CHANNEL_COUNT              2   /* We have two communication channels: USB and TCP */
#define TCP_CHANNEL                     0   /* Communication channel number and index */
#define USB_CHANNEL                     1   /*  to the global variables array         */

#define RESPONSE_BUFFER_SIZE            3200

/*
 * Callback for TCP layer for when packet is received.
 */
void TcpDataReceived(uint8_t* pData, uint16_t length);

/*
 * Called from interrupt when some data from USB is received. Puts received data
 * to queue which is read from normal task (so that we are not parsing protocol
 * from interrupt, it causes problems when trying to communicate via SPI as the
 * scheduler does not switch tasks during interrupts).
 */
void UsbDataReceived(uint8_t* pData, uint16_t length);

/*
 * Task for reading queue that is written by UsbDataReceived().
 * parseDataprotocol() called from here.
 */
void UsbProtocolTask(void* arg);

/*
 * Blocking wait until queue has any data, then take all the available elements
 * from it. Suppose that element is of the type GenericMessageType!
 */
void WaitAndDrainQueue(uint8_t* pDest, uint16_t size, uint16_t* pResSize,
                       osMessageQueueId_t queueHandle);



#endif /* INC_COMMCONTROL_H_ */
