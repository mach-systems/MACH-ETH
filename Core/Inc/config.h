/*
 * config.h
 *
 *  Created on: Jun 27, 2021
 *      Author: Karosa
 */

#ifndef INC_CONFIG_H_
#define INC_CONFIG_H_

#include <stdint.h>
#include <can.h>
#include <lin.h>
#include <stddef.h> /* size_t */



/* Default IP address and IPv4 length */
#define IP_ADDRESS_LENGTH           4
#define DEFAULT_IP0                 192
#define DEFAULT_IP1                 168
#define DEFAULT_IP2                 1
#define DEFAULT_IP3                 100
#define IP_MASK_LENGTH              4
#define DEFAULT_MASK0               255
#define DEFAULT_MASK1               255
#define DEFAULT_MASK2               255
#define DEFAULT_MASK3               0
#define DEFAULT_GW0                 0
#define DEFAULT_GW1                 0
#define DEFAULT_GW2                 0
#define DEFAULT_GW3                 0

/* Default LIN interface setting */
#define DEFAULT_MODE_LIN                        LIN_MODE_MASTER
#define DEFAULT_CHECKSUM_LIN                    CHECKSUM_ENHANCED
#define DEFAULT_BAUDRATE_LIN                    BAUDRATE_19200
#define DEFAULT_AMLR_LIN                        AUTOLEN
#define DEFAULT_AUTOSTART_LIN                   false

/* Default CAN1 interface setting */
#define DEFAULT_DBAUD_CAN_CH1                   BAUDRATE_2M
#define DEFAULT_NBAUD_CAN_CH1                   BAUDRATE_500k
#define DEFAULT_MODE_CAN_CH1                    NormalMode
#define DEFAULT_DSJW_CAN_CH1                    4
#define DEFAULT_NSJW_CAN_CH1                    8
#define DEFAULT_DSP_CAN_CH1                     SP_80
#define DEFAULT_NSP_CAN_CH1                     SP_80
#define DEFAULT_PROTOCOL_CAN_CH1                ISO_CAN_FD
#define DEFAULT_AUTOSTART_CAN_CH1               false
#define DEFAULT_PRECISE_TIM_CAN_CH1             false
#define DEFAULT_DATA_PRESCALER_CAN_CH1          2
#define DEFAULT_DATA_TIME_SEG1_CAN_CH1          15
#define DEFAULT_DATA_TIME_SEG2_CAN_CH1          4
#define DEFAULT_ARBITRATION_PRESCALER_CAN_CH1   8
#define DEFAULT_ARBITRATION_TIME_SEG1_CAN_CH1   15
#define DEFAULT_ARBITRATION_TIME_SEG2_CAN_CH1   4

/* Default CAN2 interface setting */
#define DEFAULT_DBAUD_CAN_CH2                   BAUDRATE_2M
#define DEFAULT_NBAUD_CAN_CH2                   BAUDRATE_500k
#define DEFAULT_MODE_CAN_CH2                    NormalMode
#define DEFAULT_DSJW_CAN_CH2                    4
#define DEFAULT_NSJW_CAN_CH2                    8
#define DEFAULT_DSP_CAN_CH2                     SP_80
#define DEFAULT_NSP_CAN_CH2                     SP_80
#define DEFAULT_PROTOCOL_CAN_CH2                ISO_CAN_FD
#define DEFAULT_AUTOSTART_CAN_CH2               false
#define DEFAULT_PRECISE_TIM_CAN_CH2             false
#define DEFAULT_DATA_PRESCALER_CAN_CH2          2
#define DEFAULT_DATA_TIME_SEG1_CAN_CH2          15
#define DEFAULT_DATA_TIME_SEG2_CAN_CH2          4
#define DEFAULT_ARBITRATION_PRESCALER_CAN_CH2   8
#define DEFAULT_ARBITRATION_TIME_SEG1_CAN_CH2   15
#define DEFAULT_ARBITRATION_TIME_SEG2_CAN_CH2   4

/* Default TCP port */
#define DEFAULT_PORT                8000

/* MAC address length */
#define MAC_ADDRESS_LENGTH        	6

#define CONFIG_BASE_ADDR            1                                       /* Address for saving configuration - first address is 1 */
#define INFO_BASE_ADDR              CONFIG_BASE_ADDR \
                                    + sizeof(CONFIGURATION) / 2 + 1         /* Save the information right after the configuration (and checksums) */
#define EXTRA_CONFIG_BASE_ADDR      INFO_BASE_ADDR \
                                    + sizeof(PRODUCT_INFORMATION)/ 2 + 1
#define LIN_CONFIG_BASE_ADDR        EXTRA_CONFIG_BASE_ADDR \
                                    + sizeof(EXTRA_CONFIGURATION)/ 2 + 1    /* LIN configuration */
#define CAN1_CONFIG_BASE_ADDR       LIN_CONFIG_BASE_ADDR \
                                    + sizeof(CONFIGURATION_LIN)/ 2 + 1      /* CAN1 configuration */
#define CAN2_CONFIG_BASE_ADDR       CAN1_CONFIG_BASE_ADDR \
                                    + sizeof(CONFIGURATION_CAN)/ 2 + 1      /* CAN2 configuration */

/* Lengths of the PRODUCT_INFORMATION arrays */
#define SN_LENGTH           4
#define HW_INFO_LENGTH      6

#define EXTRA_CONFIG_RESERVED_SIZE  508     /* Total size of extra configuration */
#define LIN_CONFIG_RESERVED_SIZE    122     /* 128 - 6 = 122 */
#define CAN_CONFIG_RESERVED_SIZE    110     /* 128 - 18 = 110 */

#define VERSION_MINOR       0x01
#define VERSION_MAJOR       0x01

#define RESPONSE_SUCCESS    0
#define RESPONSE_CHKSM_ERR  1
#define RESPONSE_ID_ERR     2

#define CRC_DATATYPE        uint8_t



typedef uint8_t uint8;
typedef uint16_t uint16;
typedef uint32_t uint32;
typedef uint8_t boolean;


/*
 * Structure for configuration of the device.
 * Size of this struct must be always even! Or the writing to the EEPROM won't work
 */
typedef struct SConfig
{
    uint8_t IpAddress[IP_ADDRESS_LENGTH];
    uint8_t IpMask[IP_MASK_LENGTH];
    uint16_t Port;
    uint8_t MacAddress[MAC_ADDRESS_LENGTH];
} CONFIGURATION;

/*
 * Additional configuration. Total size is 1024 bytes.
 */
typedef struct SMoreConfig
{
    uint8_t DefaultGateway[IP_ADDRESS_LENGTH];
    uint8_t Reserved[EXTRA_CONFIG_RESERVED_SIZE];
} EXTRA_CONFIGURATION;

typedef struct LinConfig
{
    LinInitStruct LinConfiguration;
    uint8_t Reserved[LIN_CONFIG_RESERVED_SIZE];
} CONFIGURATION_LIN;

typedef struct CANConfig
{
    CanInitStruct CanConfiguration;
    uint8_t Reserved[CAN_CONFIG_RESERVED_SIZE];
} CONFIGURATION_CAN;

/*
 * Structure for product information (serial number and HW info number).
 */
typedef struct SProductInfo
{
    uint8_t SerialNumber[SN_LENGTH];
    uint8_t HardwareInfo[HW_INFO_LENGTH];
} PRODUCT_INFORMATION;


/*
 * Should be called at the beginning of the program.
 */
void InitNonVolatileData(void);

/*
 * Create the configuration array from supplied data.
 */
uint8_t CreateConfiguration(uint8_t IpAddress[IP_ADDRESS_LENGTH], uint16_t Port, uint8_t MacAddress[MAC_ADDRESS_LENGTH]);

/*
 * Try to load the configuration from flash. !!! Changes the global configuration
 * value even if the checksum is incorrect !!!
 */
uint8_t ReadConfiguration(void);

uint8_t ReadExtraConfiguration(void);

uint8_t ReadConfigurationLIN(void);

uint8_t ReadConfigurationCAN1(void);

uint8_t ReadConfigurationCAN2(void);


/*
 * Getter for the configuration.
 */
CONFIGURATION GetConfiguration(void);

EXTRA_CONFIGURATION GetExtraConfiguration(void);

const EXTRA_CONFIGURATION* GetExtraConfigurationAddr(void);

CONFIGURATION_LIN GetConfigurationLIN(void);

CONFIGURATION_CAN GetConfigurationCAN1(void);

CONFIGURATION_CAN GetConfigurationCAN2(void);


/*
 * Save configuration to the FLASH.
 */
uint8_t WriteConfiguration(void);

/*
 * Save extra configuration to the FLASH.
 */
uint8_t WriteExtraConfiguration(void);

/*
 * Save configuration LIN to the FLASH.
 */
uint8_t WriteConfigurationLIN(void);

/*
 * Save configuration CAN1 to the FLASH.
 */
uint8_t WriteConfigurationCAN1(void);

/*
 * Save configuration CAN2 to the FLASH.
 */
uint8_t WriteConfigurationCAN2(void);

/*
 * Sum bytes in the supplied array.
 */
uint8_t ArraySum(const uint8_t* pArr, uint8_t size);

/*
 * Count one-byte sum of the supplied array.
 */
CRC_DATATYPE ArrayChecksum(const uint8_t* pArr, uint16_t size);

/*
 * Apply default configuration by the defines in this file (config.h).
 */
void DefaultConfiguration(void);

/*
 * Apply default extra configuration by the defines in this file (config.h).
 */
void DefaultExtraConfiguration(void);

/*
 * Change device MAC address (only in configuration array).
 */
uint8_t ConfigChangeMac(uint8_t* pData);

/*
 * Change IPv4 address (only in configuration array).
 */
uint8_t ConfigChangeIp(const uint8_t* pData);

/*
 * Change default gateway (only in the configuration array).
 */
uint8_t ConfigChangeGw(const uint8_t* pData);

/*
 * Convert mask from one-byte encoded value (e.g. 24) to the array value
 * (e.g. 255.255.255.0).
 */
void MaskToArray(uint8_t encoded, uint8_t* pArr);

/*
 * Convert one-byte encoded value to mask (see previous function).
 */
void MaskFromArray(uint8_t* pEncoded, uint8_t* pArr);

/*
 * Change subnet mask (only in configuration array). Supplied mask is in one-byte
 * encoded format.
 */
uint8_t ConfigChangeMask(uint8_t encoded);

/*
 * Change used TCP port (only in configuration array.
 */
uint8_t ConfigChangePort(const uint16_t num);

/*
 * Set hardware info.
 */
void SetHwInfo(uint8_t HwInfo[HW_INFO_LENGTH]);

/*
 * Set the device serial number.
 */
void SetSerialNumber(uint8_t SN[SN_LENGTH]);

/*
 * Getter for product information.
 */
PRODUCT_INFORMATION GetInformation(void);

/*
 * Save product information to FLASH.
 */
uint8_t WriteInformation(void);


/*
 * Add bytes to checksum crc8. Function taken from project CAN-LIN-ECU.
 * Credit for algorithm: rcgldr from https://stackoverflow.com/
 * Polynomial: 0x07
 */
void AddToCrc(CRC_DATATYPE* crc, const uint8_t* pData, size_t length);


/*
 * Apply default configuration by the defines in this file (config.h) Lin only.
 */
void DefaultConfigurationLIN(void);

/*
 * Apply default configuration by the defines in this file (config.h) CAN1 only.
 */
void DefaultConfigurationCanCh1(void);

/*
 * Apply default configuration by the defines in this file (config.h) CAN2 only.
 */
void DefaultConfigurationCanCh2(void);

/*
 * Other modules may want to read the firmware version constant variable.
 */
extern const volatile uint16_t FirmwareVersion;


#endif /* INC_CONFIG_H_ */
