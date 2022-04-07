/*
 * packetControl.h
 *
 *  Created on: May 21, 2021
 *      Author: Karel Hevessy
 */

#ifndef INC_PACKETCONTROL_H_
#define INC_PACKETCONTROL_H_

#include <stdint.h>
#include "config.h"

#define QUEUE_PUT_TIMEOUT               80
#define MAX_CMD_LEN                     400U

typedef struct T_MESSAGE
{
    uint8_t Data[MAX_CMD_LEN];
    uint16_t Datalen;
} GenericMessageType;



#endif /* INC_PACKETCONTROL_H_ */
