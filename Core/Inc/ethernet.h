/*
 * ethernet.h
 *
 * Function low_level_output_user() overrides the low_level_output() function
 * from the file ethernetif.c
 *
 *  Created on: Jan 14, 2021
 *      Author: Karel Hevessy
 */

#ifndef INC_ETHERNET_H_
#define INC_ETHERNET_H_

#include "netif/ethernet.h"

/**
 * This function should do the actual transmission of the packet. The packet is
 * contained in the pbuf that is passed to the function. This pbuf
 * might be chained.
 *
 * @param netif the lwip network interface structure for this ethernetif
 * @param p the MAC packet to send (e.g. IP packet including MAC addresses and type)
 * @return ERR_OK if the packet could be sent
 *         an err_t value if the packet couldn't be sent
 *
 * @note Returning ERR_MEM here if a DMA queue of your MAC is full can lead to
 *       strange results. You might consider waiting for space in the DMA queue
 *       to become availale since the stack doesn't retry to send a packet
 *       dropped because of memory failure (except for the TCP timers).
 *
 * !!! This function is instead of low_level_output() in ethernetif.c, as we
 *     must do SCB_CleanInvalidateDCache() (on line 73) and we the file
 *     ethernetif.c !!!
 *
 */
err_t low_level_output_user(struct netif *netif, struct pbuf *p);



#endif /* INC_ETHERNET_H_ */
