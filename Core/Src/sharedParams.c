/*
 * sharedParams.c
 *
 *  Created on: 7. 9. 2021
 *      Author: Karel Hevessy
 */

#include "sharedParams.h"
#include "config.h"

#define SHARED_PARAMS_ID            0x01234567U
#define SHARED_DATA_LENGTH          56

#define DTCM_START                  0x20000000
#define DTCM_END                    0x20020000

typedef struct S_PARAMS
{
    uint32_t Constant;
    uint8_t Data[SHARED_DATA_LENGTH];
    uint8_t Checksum;
} SharedParams;

static SharedParams SharedParameters __attribute__((section(".shared")));

uint8_t paramsValidFlag;

/*
 * Needed when device is reset RIGHT AFTER the write to DTCM SRAM.
 * See https://community.st.com/s/article/FAQ-STM32H7-SRAM-Backup-SRAM-content-is-not-preserved-after-reset
 * and page 140 in RM0468 (ref. manual for STM32H735 family)
 */
void flushEcc(void* ptr, uint32_t bytes);



uint8_t CheckSharedParams(void)
{
  uint8_t ret = 0;
  if (SharedParameters.Constant == SHARED_PARAMS_ID)
  {
    uint8_t crc = 0;
    AddToCrc(&crc, SharedParameters.Data, SHARED_DATA_LENGTH);
    ret = (crc == SharedParameters.Checksum);
  }
  return ret;
}

void InitSharedParams(void)
{
  if (!CheckSharedParams())
  {
    for (uint8_t i = 0; i < SHARED_DATA_LENGTH; i++)
      SharedParameters.Data[i] = 0;
    SharedParameters.Constant = SHARED_PARAMS_ID;
    SharedParameters.Checksum = 0;
    AddToCrc(&SharedParameters.Checksum, SharedParameters.Data, SHARED_DATA_LENGTH);
  }
  paramsValidFlag = 1;  /* Shared data are surely valid */
}

uint8_t GetSharedData(uint8_t index)
{
  uint8_t ret = 0;
  if (CheckSharedParams() && index < SHARED_DATA_LENGTH)
    ret = SharedParameters.Data[index];
  return ret;
}

uint8_t SetSharedData(uint8_t index, uint8_t data)
{
  if (!paramsValidFlag)
    InitSharedParams();
  uint8_t ret = 0;
  if (index < SHARED_DATA_LENGTH)
  {
    SharedParameters.Data[index] = data;
    SharedParameters.Checksum = 0;
    AddToCrc(&SharedParameters.Checksum, SharedParameters.Data, SHARED_DATA_LENGTH);
    // Needed when unaligned access was done and reset will happen right after this call
    flushEcc(&SharedParameters, sizeof(SharedParameters));
  }
  return ret;
}

void flushEcc(void* ptr, uint32_t bytes)
{
    if ((uint32_t) ptr >= DTCM_START && (uint32_t) ptr < DTCM_END)
    {
        uint32_t* address = (uint32_t*) ((uint32_t) ptr & 0xfffffffc);
        uint32_t* end = (uint32_t*) (((uint32_t) ptr + bytes) & 0xfffffffc);
        volatile uint32_t value;
        do
        {
            value = *address;
            *address = value;
            address++;
        } while (address < end && address < (uint32_t*) DTCM_END);
    }
}
