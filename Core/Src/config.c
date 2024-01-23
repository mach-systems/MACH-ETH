/*
 * config.c
 *
 *  Created on: Jun 27, 2021
 *      Author: Karosa
 */

#include <stdio.h>      /* snprintf() */
#include <string.h>     /* memcpy() */
#include <lwip.h>       /* heth */
#include "config.h"
#include "eeprom_emul.h"

CONFIGURATION Config;
EXTRA_CONFIGURATION ExtraConfig;
PRODUCT_INFORMATION ProductInfo;
CONFIGURATION_CAN configCan1;
CONFIGURATION_CAN configCan2;
CONFIGURATION_LIN configLin;

/* Needed for the EEPROM emulation */
/* Virtual address defined by the user: 0xFFFF value is prohibited */
uint16_t VirtAddVarTab[NB_OF_VAR];

// Load address is 0x80bffe0
const volatile uint16_t __attribute__((section(".version"))) FirmwareVersion  = VERSION_MINOR + (256 * VERSION_MAJOR);
#ifdef Bootloader
    const volatile uint64_t __attribute__((section(".signature"))) Signature = 0x55aa55aa55aa55aa;
#endif


void InitNonVolatileData(void)
{
  // Prevent the hard-coded version to be optimized out (if the section is set
  // as KEEP() in the linker, it will be preserved either way)
  (void) FirmwareVersion;

  // Initialize array of virtual addresses
  for (uint16_t i = 0; i < NB_OF_VAR; i++)
    VirtAddVarTab[i] = i + 1;

  HAL_FLASH_Unlock();
  EE_Init();

  if (!ReadConfiguration())
    DefaultConfiguration();
  if (!ReadExtraConfiguration())
    DefaultExtraConfiguration();
  if (!ReadConfigurationLIN())
    DefaultConfigurationLIN();
  if (!ReadConfigurationCAN1())
    DefaultConfigurationCanCh1();
  if (!ReadConfigurationCAN2())
    DefaultConfigurationCanCh2();

  uint8_t i;
  for (i = 0; i < sizeof(PRODUCT_INFORMATION) / 2; i++)
    EE_ReadVariable(INFO_BASE_ADDR + i, ((uint16_t*) &ProductInfo + i));
}

uint8_t CreateConfiguration(uint8_t IpAddress[IP_ADDRESS_LENGTH], uint16_t Port, uint8_t MacAddress[MAC_ADDRESS_LENGTH])
{
  Config.IpAddress[0] = IpAddress[0];
  Config.IpAddress[1] = IpAddress[1];
  Config.IpAddress[2] = IpAddress[2];
  Config.IpAddress[3] = IpAddress[3];

  Config.Port = Port;

  Config.MacAddress[0] = MacAddress[0];
  Config.MacAddress[1] = MacAddress[1];
  Config.MacAddress[2] = MacAddress[2];
  Config.MacAddress[3] = MacAddress[3];
  Config.MacAddress[4] = MacAddress[4];
  Config.MacAddress[5] = MacAddress[5];
  return 1;
}

uint8_t ReadConfiguration(void)
{
  uint8_t ret = 0;
  uint16_t i, readData, status = 0;
  CONFIGURATION tmp;
  for (i = 0; i < sizeof(CONFIGURATION) / 2; i++)
    status |= EE_ReadVariable(CONFIG_BASE_ADDR + i, ((uint16_t*) &tmp + i));

  if (status == 0)
    status |= EE_ReadVariable(CONFIG_BASE_ADDR + i, &readData);
  Config = tmp;

  if (status == 0 && readData == ArrayChecksum((uint8_t*) &tmp, sizeof(CONFIGURATION)))
    ret = 1;

  return ret;
}

uint8_t ReadExtraConfiguration(void)
{
  uint8_t ret = 0;
  uint16_t i, readData, status = 0;
  EXTRA_CONFIGURATION tmp;
  for (i = 0; i < sizeof(EXTRA_CONFIGURATION) / 2; i++)
    status |= EE_ReadVariable(EXTRA_CONFIG_BASE_ADDR + i, ((uint16_t*) &tmp + i));

  if (status == 0)
    status |= EE_ReadVariable(EXTRA_CONFIG_BASE_ADDR + i, &readData);
  ExtraConfig = tmp;

  if (status == 0 && readData == ArrayChecksum((uint8_t*) &tmp, sizeof(EXTRA_CONFIGURATION)))
    ret = 1;

  return ret;
}

CONFIGURATION GetConfiguration(void)
{
  return Config;
}

EXTRA_CONFIGURATION GetExtraConfiguration(void)
{
  return ExtraConfig;
}

const EXTRA_CONFIGURATION* GetExtraConfigurationAddr(void)
{
  return &ExtraConfig;
}

CONFIGURATION_LIN GetConfigurationLIN(void)
{
  return configLin;
}

CONFIGURATION_CAN GetConfigurationCAN1(void)
{
  return configCan1;
}

CONFIGURATION_CAN GetConfigurationCAN2(void)
{
  return configCan2;
}


uint8_t WriteConfiguration(void)
{
  uint16_t i, status = 0;
  for (i = 0; i < sizeof(CONFIGURATION) / 2; i++)
    status |= EE_WriteVariable(CONFIG_BASE_ADDR + i, *((uint16_t*) &Config + i));

  if (status == 0)
    status |= EE_WriteVariable(CONFIG_BASE_ADDR + i, ArrayChecksum((uint8_t*) &Config, sizeof(CONFIGURATION)));

  return (status == 0);
}

uint8_t WriteExtraConfiguration(void)
{
  uint16_t i, status = 0;
  for (i = 0; i < sizeof(EXTRA_CONFIGURATION) / 2; i++)
    status |= EE_WriteVariable(EXTRA_CONFIG_BASE_ADDR + i, *((uint16_t*) &ExtraConfig + i));

  if (status == 0)
    status |= EE_WriteVariable(EXTRA_CONFIG_BASE_ADDR + i, ArrayChecksum((uint8_t*) &ExtraConfig, sizeof(EXTRA_CONFIGURATION)));

  return (status == 0);
}

uint8_t WriteConfigurationLIN(void)
{
  uint16_t i, status = 0;
  for (i = 0; i < sizeof(CONFIGURATION_LIN) / 2; i++)
    status |= EE_WriteVariable(LIN_CONFIG_BASE_ADDR + i, *((uint16_t*) &configLin + i));

  if (status == 0)
      status |= EE_WriteVariable(LIN_CONFIG_BASE_ADDR + i, ArrayChecksum((uint8_t*) &configLin, sizeof(CONFIGURATION_LIN)));

  return (status == 0);
}

uint8_t WriteConfigurationCAN1(void)
{
  uint16_t i, status = 0;
  for (i = 0; i < sizeof(CONFIGURATION_CAN) / 2; i++)
    status |= EE_WriteVariable(CAN1_CONFIG_BASE_ADDR + i, *((uint16_t*) &configCan1 + i));

  if (status == 0)
      status |= EE_WriteVariable(CAN1_CONFIG_BASE_ADDR + i, ArrayChecksum((uint8_t*) &configCan1, sizeof(CONFIGURATION_CAN)));

  return (status == 0);
}
uint8_t WriteConfigurationCAN2(void)
{
  uint16_t i, status = 0;
  for (i = 0; i < sizeof(CONFIGURATION_CAN) / 2; i++)
    status |= EE_WriteVariable(CAN2_CONFIG_BASE_ADDR + i, *((uint16_t*) &configCan2 + i));

  if (status == 0)
      status |= EE_WriteVariable(CAN2_CONFIG_BASE_ADDR + i, ArrayChecksum((uint8_t*) &configCan2, sizeof(CONFIGURATION_CAN)));

  return (status == 0);
}

uint8_t ReadConfigurationLIN(void)
{
  uint8_t ret = 0;
  uint16_t i, readData, status = 0;
  CONFIGURATION_LIN tmp;
  for (i = 0; i < sizeof(CONFIGURATION_LIN) / 2; i++)
      status |= EE_ReadVariable(LIN_CONFIG_BASE_ADDR + i, ((uint16_t*) &tmp + i));

  status |= EE_ReadVariable(LIN_CONFIG_BASE_ADDR + i, &readData);
  configLin = tmp;

  if (status == 0 && readData == ArrayChecksum((uint8_t*) &tmp, sizeof(CONFIGURATION_LIN)))
  {
    ret = 1;
  }
  return ret;
}

uint8_t ReadConfigurationCAN1(void)
{
  uint8_t ret = 0;
  uint16_t i, readData, status = 0;
  CONFIGURATION_CAN tmp;
  for (i = 0; i < sizeof(CONFIGURATION_CAN) / 2; i++)
      status |= EE_ReadVariable(CAN1_CONFIG_BASE_ADDR + i, ((uint16_t*) &tmp + i));

  status |= EE_ReadVariable(CAN1_CONFIG_BASE_ADDR + i, &readData);
  configCan1 = tmp;

  if (status == 0 && readData == ArrayChecksum((uint8_t*) &tmp, sizeof(CONFIGURATION_CAN)))
  {
    ret = 1;
  }
  return ret;
}

uint8_t ReadConfigurationCAN2(void)
{
  uint8_t ret = 0;
  uint16_t i, readData, status = 0;
  CONFIGURATION_CAN tmp;
  for (i = 0; i < sizeof(CONFIGURATION_CAN) / 2; i++)
      status |= EE_ReadVariable(CAN2_CONFIG_BASE_ADDR + i, ((uint16_t*) &tmp + i));

  status |= EE_ReadVariable(CAN2_CONFIG_BASE_ADDR + i, &readData);
  configCan2 = tmp;

  if (status == 0 && readData == ArrayChecksum((uint8_t*) &tmp, sizeof(CONFIGURATION_CAN)))
  {
    ret = 1;
  }
  return ret;
}

void DefaultConfigurationLIN(void)
{
  configLin.LinConfiguration.DeviceMode = DEFAULT_MODE_LIN;
  configLin.LinConfiguration.ChecksumType = DEFAULT_CHECKSUM_LIN;
  configLin.LinConfiguration.Baudrate = DEFAULT_BAUDRATE_LIN;
  configLin.LinConfiguration.AMLR = DEFAULT_AMLR_LIN;
  configLin.LinConfiguration.autoStart = DEFAULT_AUTOSTART_LIN;
}

void DefaultConfigurationCanCh1(void)
{
  configCan1.CanConfiguration.DataBaud = DEFAULT_DBAUD_CAN_CH1;
  configCan1.CanConfiguration.ArbitrationBaud = DEFAULT_NBAUD_CAN_CH1;
  configCan1.CanConfiguration.Mode = DEFAULT_MODE_CAN_CH1;
  configCan1.CanConfiguration.DataSJW = DEFAULT_DSJW_CAN_CH1;
  configCan1.CanConfiguration.ArbitrationSJW = DEFAULT_NSJW_CAN_CH1;
  configCan1.CanConfiguration.DataSPoint = DEFAULT_DSP_CAN_CH1;
  configCan1.CanConfiguration.ArbitrationSPoint = DEFAULT_NSP_CAN_CH1;
  configCan1.CanConfiguration.AutoStart = DEFAULT_AUTOSTART_CAN_CH1;
  configCan1.CanConfiguration.Protocol = DEFAULT_PROTOCOL_CAN_CH1;
  configCan1.CanConfiguration.PreciseTimingSet = DEFAULT_PRECISE_TIM_CAN_CH1;
  configCan1.CanConfiguration.DataPrescaler = DEFAULT_DATA_PRESCALER_CAN_CH1;
  configCan1.CanConfiguration.DataTimeSegment1 = DEFAULT_DATA_TIME_SEG1_CAN_CH1;
  configCan1.CanConfiguration.DataTimeSegment2 = DEFAULT_DATA_TIME_SEG2_CAN_CH1;
  configCan1.CanConfiguration.ArbitrationPrescaler = DEFAULT_ARBITRATION_PRESCALER_CAN_CH1;
  configCan1.CanConfiguration.ArbitrationTimeSegment1 = DEFAULT_ARBITRATION_TIME_SEG1_CAN_CH1;
  configCan1.CanConfiguration.ArbitrationTimeSegment2 = DEFAULT_ARBITRATION_TIME_SEG2_CAN_CH1;
}

void DefaultConfigurationCanCh2(void)
{
  configCan2.CanConfiguration.DataBaud = DEFAULT_DBAUD_CAN_CH2;
  configCan2.CanConfiguration.ArbitrationBaud = DEFAULT_NBAUD_CAN_CH2;
  configCan2.CanConfiguration.Mode = DEFAULT_MODE_CAN_CH2;
  configCan2.CanConfiguration.DataSJW = DEFAULT_DSJW_CAN_CH2;
  configCan2.CanConfiguration.ArbitrationSJW = DEFAULT_NSJW_CAN_CH2;
  configCan2.CanConfiguration.DataSPoint = DEFAULT_DSP_CAN_CH2;
  configCan2.CanConfiguration.ArbitrationSPoint = DEFAULT_NSP_CAN_CH2;
  configCan2.CanConfiguration.AutoStart = DEFAULT_AUTOSTART_CAN_CH2;
  configCan2.CanConfiguration.Protocol = DEFAULT_PROTOCOL_CAN_CH2;
  configCan2.CanConfiguration.PreciseTimingSet = DEFAULT_PRECISE_TIM_CAN_CH2;
  configCan2.CanConfiguration.DataPrescaler = DEFAULT_DATA_PRESCALER_CAN_CH2;
  configCan2.CanConfiguration.DataTimeSegment1 = DEFAULT_DATA_TIME_SEG1_CAN_CH2;
  configCan2.CanConfiguration.DataTimeSegment2 = DEFAULT_DATA_TIME_SEG2_CAN_CH2;
  configCan2.CanConfiguration.ArbitrationPrescaler = DEFAULT_ARBITRATION_PRESCALER_CAN_CH2;
  configCan2.CanConfiguration.ArbitrationTimeSegment1 = DEFAULT_ARBITRATION_TIME_SEG1_CAN_CH2;
  configCan2.CanConfiguration.ArbitrationTimeSegment2 = DEFAULT_ARBITRATION_TIME_SEG2_CAN_CH2;
}

uint8_t ArraySum(const uint8_t* pArr, uint8_t size)
{
  uint8_t sum = 0;
  uint16_t i;
  for (i = 0; i < size; i++)
    sum += pArr[i];
  return sum;
}

CRC_DATATYPE ArrayChecksum(const uint8_t* pArr, uint16_t size)
{
  CRC_DATATYPE crc = 0;
  AddToCrc(&crc, pArr, size);
  return crc;
}

void DefaultConfiguration(void)
{
  Config.IpAddress[0] = DEFAULT_IP0;
  Config.IpAddress[1] = DEFAULT_IP1;
  Config.IpAddress[2] = DEFAULT_IP2;
  Config.IpAddress[3] = DEFAULT_IP3;
  Config.IpMask[0] = DEFAULT_MASK0;
  Config.IpMask[1] = DEFAULT_MASK1;
  Config.IpMask[2] = DEFAULT_MASK2;
  Config.IpMask[3] = DEFAULT_MASK3;

  Config.Port = DEFAULT_PORT;

  // MAC address does not have any default value
}

void DefaultExtraConfiguration(void)
{
  ExtraConfig.DefaultGateway[0] = DEFAULT_GW0;
  ExtraConfig.DefaultGateway[1] = DEFAULT_GW1;
  ExtraConfig.DefaultGateway[2] = DEFAULT_GW2;
  ExtraConfig.DefaultGateway[3] = DEFAULT_GW3;

  memset((void*) ExtraConfig.Reserved, 0xff, EXTRA_CONFIG_RESERVED_SIZE);
}

uint8_t ConfigChangeMac(uint8_t* pData)
{
  Config.MacAddress[0] = pData[0];
  Config.MacAddress[1] = pData[1];
  Config.MacAddress[2] = pData[2];
  Config.MacAddress[3] = pData[3];
  Config.MacAddress[4] = pData[4];
  Config.MacAddress[5] = pData[5];
  return 1;
}

uint8_t ConfigChangeIp(const uint8_t* pData)
{
  Config.IpAddress[0] = pData[0];
  Config.IpAddress[1] = pData[1];
  Config.IpAddress[2] = pData[2];
  Config.IpAddress[3] = pData[3];
  return 1;
}

uint8_t ConfigChangeGw(const uint8_t* pData)
{
    ExtraConfig.DefaultGateway[0] = pData[0];
    ExtraConfig.DefaultGateway[1] = pData[1];
    ExtraConfig.DefaultGateway[2] = pData[2];
    ExtraConfig.DefaultGateway[3] = pData[3];
    return 1;
}

void MaskToArray(uint8_t encoded, uint8_t* pArr)
{
  encoded = encoded > 32 ? 32 : encoded;

  uint32_t ones = 1 << (32 - encoded);
  ones--;
  ones = ~ones;
  for (uint8_t i = 0; i < 4; i++)
    pArr[i] = (ones >> (24 - 8 * i) & 0xff);
}

void MaskFromArray(uint8_t* pEncoded, uint8_t* pArr)
{
  uint32_t raw = 0;
  uint8_t i;
  for (i = 0; i < 4; i++)
    raw += ((uint32_t) pArr[i] << (24 - 8 * i));
  for (i = 0; i < 32; i++)
    if (((raw >> (31 - i)) & 0x1) == 0)
      break;
  *pEncoded = i;
}

uint8_t ConfigChangeMask(uint8_t encoded)
{
  uint8_t ipMask[IP_MASK_LENGTH];
  MaskToArray(encoded, ipMask);

  Config.IpMask[0] = ipMask[0];
  Config.IpMask[1] = ipMask[1];
  Config.IpMask[2] = ipMask[2];
  Config.IpMask[3] = ipMask[3];
  return 1;
}

uint8_t ConfigChangePort(const uint16_t num)
{
  Config.Port = num;
  return 1;
}

void SetHwInfo(uint8_t HwInfo[HW_INFO_LENGTH])
{
  ProductInfo.HardwareInfo[0] = HwInfo[0];
  ProductInfo.HardwareInfo[1] = HwInfo[1];
  ProductInfo.HardwareInfo[2] = HwInfo[2];
  ProductInfo.HardwareInfo[3] = HwInfo[3];
  ProductInfo.HardwareInfo[4] = HwInfo[4];
  ProductInfo.HardwareInfo[5] = HwInfo[5];
}

void SetSerialNumber(uint8_t SN[SN_LENGTH])
{
  ProductInfo.SerialNumber[0] = SN[0];
  ProductInfo.SerialNumber[1] = SN[1];
  ProductInfo.SerialNumber[2] = SN[2];
  ProductInfo.SerialNumber[3] = SN[3];
}

PRODUCT_INFORMATION GetInformation(void)
{
  return ProductInfo;
}

uint8_t WriteInformation(void)
{
  uint16_t i;
  for (i = 0; i < sizeof(PRODUCT_INFORMATION) / 2; i++)
    EE_WriteVariable(INFO_BASE_ADDR + i, *((uint16_t*) &ProductInfo + i));
  return 1;
}

void AddToCrc(CRC_DATATYPE* crc, const uint8_t* pData, size_t length)
{
    size_t i, j;
    for (i = 0; i < length; i++)
    {
        *crc ^= pData[i];
        for (j = 0; j < 8; j++)
        {
            if ((*crc & 0x80) != 0)
                *crc = (uint8_t)((*crc << 1) ^ 0x07);
            else
                *crc <<= 1;
        }
    }
}
