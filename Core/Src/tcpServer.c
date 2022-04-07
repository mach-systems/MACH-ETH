/*
 * Copyright (c) 2001-2003 Swedish Institute of Computer Science.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 * 3. The name of the author may not be used to endorse or promote products
 *    derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT
 * SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT
 * OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING
 * IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY
 * OF SUCH DAMAGE.
 *
 * This file is part of the lwIP TCP/IP stack.
 *
 * Author: Adam Dunkels <adam@sics.se>
 *
 */
/* Includes ------------------------------------------------------------------*/
#include <commControl.h>
#include "tcpServer.h"
#include "packetControl.h"
#include "lwip/opt.h"
#include "config.h"

#if LWIP_NETCONN

#include "lwip/sys.h"
#include "lwip/api.h"
#include "lwip.h"
#include <netif.h>  /* netif_is_link_up() */
#include <tcp.h>    /* tcp_abort() */
#include "main.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define TCPECHO_THREAD_PRIO  (osPriorityNormal)
#define TCPSEND_THREAD_PRIO     (osPriorityNormal)

/* Private macro -------------------------------------------------------------*/
#define RESPONSE_BUFFER_SIZE    384
#define RECV_TIMEOUT            1000    /* 1 second timeout */
/* Private variables ---------------------------------------------------------*/

/* Private function prototypes -----------------------------------------------*/
osStatus_t tcpSendResponse(uint8_t* pData, uint16_t length);
/* Private functions ---------------------------------------------------------*/
struct netconn *newconn;

void TcpServerThread(void *arg)
{
  struct netconn *conn;
  err_t err;
  LWIP_UNUSED_ARG(arg);

  /* Create a new connection identifier. */
  /* Bind connection to port determined by configuration */
  conn = netconn_new(NETCONN_TCP);
  /* Apply our configuration settings */
  /* !!! Configuration must already be loaded !!! */
  CONFIGURATION config = GetConfiguration();
  netconn_bind(conn, IP_ADDR_ANY, config.Port);

  LWIP_ERROR("tcpecho: invalid conn", (conn != NULL), return;);

  /* Tell connection to go into listening mode. */
  netconn_listen(conn);

  osMessageQueueId_t queueId = *(osMessageQueueId_t*) arg;
  UNUSED(queueId);

  while (1)
  {
    netconn_set_recvtimeout(conn, 0);
    while (!netif_is_link_up(&gnetif))
      osDelay(100);
    /* Grab new connection. */
    err = netconn_accept(conn, &newconn);
    /* Process the new connection. */
    if (err == ERR_OK)
    {
      struct netbuf *buf = NULL;
      void *data;
      u16_t len;
rcv:
      netconn_set_recvtimeout(newconn, 1000);
      err_t error = netconn_recv(newconn, &buf);
      if (!netif_is_link_up(&gnetif))
      {
        /* The cable was disconnected, so we must manually inform stack about the abort */
        /* If this function is not called, device runs out of memory!!! */
        tcp_abort(newconn->pcb.tcp);
      }

      if (error == ERR_OK || error == ERR_TIMEOUT) /* Data received or nothing happened */
      {
        /* If timeout, buffer will be empty */
        if (buf != NULL)
        {
          do {
            netbuf_data(buf, &data, &len);
            TcpDataReceived(data, len);
          } while (netbuf_next(buf) >= 0);
          netbuf_delete(buf);
        }
        goto rcv;
      }
      else
      {
        /* Some other error - e.g. ERR_ABRT (-13) */
        netbuf_delete(buf);
        /* Close connection and discard connection identifier. */
        netconn_close(newconn);
        netconn_delete(newconn);
      }
    }
  }
}
/*-----------------------------------------------------------------------------------*/
osStatus_t TcpEnqueueResponse(uint8_t* pData, uint16_t length)
{
  GenericMessageType msg;
  msg.Datalen = length > MAX_CMD_LEN ? MAX_CMD_LEN : length;
  for (uint16_t i = 0; i < msg.Datalen; i++)
    msg.Data[i] = pData[i];
  /* Determine if called from interrupt */
  uint32_t timeout = (__get_IPSR() != 0U) ? 0 : QUEUE_PUT_TIMEOUT;
  return osMessageQueuePut(TcpTxQueueHandle, &msg, 0, timeout);
}
/*-----------------------------------------------------------------------------------*/
void TcpSendThread(void *arg)
{
  osStatus_t queueState;
  GenericMessageType tcpTransmitBuffer;
  while (1)
  {
    while ((queueState = osMessageQueueGet(TcpTxQueueHandle, &tcpTransmitBuffer, NULL, 0xffff)) != osOK);
    if (newconn->type != NETCONN_INVALID)
      netconn_write(newconn, tcpTransmitBuffer.Data, tcpTransmitBuffer.Datalen, NETCONN_COPY);
  }
}
/*-----------------------------------------------------------------------------------*/
void TcpServerInit(void* argument)
{
  sys_thread_new("tcpserver_thread", TcpServerThread, argument, (configMINIMAL_STACK_SIZE * 32 + 1024), TCPECHO_THREAD_PRIO);
  sys_thread_new("TCP send thread", TcpSendThread, NULL, configMINIMAL_STACK_SIZE * 32, TCPSEND_THREAD_PRIO);
}
/*-----------------------------------------------------------------------------------*/

#endif /* LWIP_NETCONN */
