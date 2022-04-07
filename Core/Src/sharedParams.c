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

typedef struct S_PARAMS
{
    uint32_t Constant;
    uint8_t Data[SHARED_DATA_LENGTH];
    uint8_t Checksum;
} SharedParams;

static SharedParams SharedParameters __attribute__((section(".shared")));

uint8_t paramsValidFlag;

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
  }
  return ret;
}
