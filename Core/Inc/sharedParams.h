/*
 * sharedParams.h
 *
 * Share parameters in RAM with bootloader. Inspired by open-source code from
 * OpenBLT source:
 * www.feaser.com/en/blog/2018/07/data-exchange-between-openblt-and-your-firmware
 *
 *  Created on: 7. 9. 2021
 *      Author: Karel Hevessy
 */

#ifndef INC_SHAREDPARAMS_H_
#define INC_SHAREDPARAMS_H_

#include <stdint.h>

/*
 * Check that there is some valid data stored.
 */
uint8_t CheckSharedParams(void);

/*
 * Initialize shared parameter section.
 */
void InitSharedParams(void);

/*
 * Read shared data from supplied index (if there are any valid data).
 */
uint8_t GetSharedData(uint8_t index);

/*
 * Store data to supplied index. Initialize if not valid.
 */
uint8_t SetSharedData(uint8_t index, uint8_t data);



#endif /* INC_SHAREDPARAMS_H_ */
