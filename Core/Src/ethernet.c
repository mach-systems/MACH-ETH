/*
 * ethernet.c
 *
 *  Created on: Jan 14, 2021
 *      Author: Karel Hevessy
 *
 */

#include "ethernet.h"
#include <string.h>


/* Global variables from ethernetif.c */
extern ETH_HandleTypeDef heth;
extern ETH_TxPacketConfig TxConfig;

/* ETH Setting from ethernetif.c */
#define ETH_DMA_TRANSMIT_TIMEOUT               ( 20U )

err_t low_level_output_user(struct netif *netif, struct pbuf *p)
{
  uint32_t i=0;
  struct pbuf *q;
  err_t errval = ERR_OK;
  ETH_BufferTypeDef Txbuffer[ETH_TX_DESC_CNT];

  memset(Txbuffer, 0 , ETH_TX_DESC_CNT*sizeof(ETH_BufferTypeDef));

  for(q = p; q != NULL; q = q->next)
  {
    if(i >= ETH_TX_DESC_CNT)
      return ERR_IF;

    Txbuffer[i].buffer = q->payload;
    Txbuffer[i].len = q->len;

    if(i>0)
    {
      Txbuffer[i-1].next = &Txbuffer[i];
    }

    if(q->next == NULL)
    {
      Txbuffer[i].next = NULL;
    }

    i++;
  }

  TxConfig.Length =  p->tot_len;
  TxConfig.TxBuffer = Txbuffer;

  /* This is the important line because of which this file is needed */
  /* Without it, the stack is very slow and looses some packets */
  SCB_CleanInvalidateDCache();
  HAL_ETH_Transmit(&heth, &TxConfig, ETH_DMA_TRANSMIT_TIMEOUT);

  return errval;
}
