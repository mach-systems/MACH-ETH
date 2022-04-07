/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32h7xx_hal.h"

#include "stm32h7xx_ll_tim.h"
#include "stm32h7xx_ll_usart.h"
#include "stm32h7xx_ll_rcc.h"
#include "stm32h7xx_ll_bus.h"
#include "stm32h7xx_ll_cortex.h"
#include "stm32h7xx_ll_system.h"
#include "stm32h7xx_ll_utils.h"
#include "stm32h7xx_ll_pwr.h"
#include "stm32h7xx_ll_gpio.h"
#include "stm32h7xx_ll_dma.h"

#include "stm32h7xx_ll_exti.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <cmsis_os2.h>
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */
extern TIM_HandleTypeDef htim14;
extern TIM_HandleTypeDef htim16;
extern TIM_HandleTypeDef htim17;

extern osMessageQueueId_t SpiTxQueueHandle;
extern osMessageQueueId_t SpiRxQueueHandle;
extern osMessageQueueId_t Spi2TxQueueHandle;
extern osMessageQueueId_t Spi2RxQueueHandle;
extern osMessageQueueId_t TcpTxQueueHandle;
/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */
extern SPI_HandleTypeDef hspi1;

extern SPI_HandleTypeDef hspi2;
extern SPI_HandleTypeDef hspi3;
/* Anyone can request transition to the bootloader */
extern uint8_t BootloaderRequest;
/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */
/*
 * Get value on analog input.
 */
uint16_t GetAdcMeasurement(void);
/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define DIPSW_1_Pin GPIO_PIN_2
#define DIPSW_1_GPIO_Port GPIOE
#define DIPSW_2_Pin GPIO_PIN_4
#define DIPSW_2_GPIO_Port GPIOE
#define DIPSW_3_Pin GPIO_PIN_5
#define DIPSW_3_GPIO_Port GPIOE
#define DIPSW_4_Pin GPIO_PIN_13
#define DIPSW_4_GPIO_Port GPIOC
#define LED_3_GREEN_Pin GPIO_PIN_14
#define LED_3_GREEN_GPIO_Port GPIOC
#define LED_2_GREEN_Pin GPIO_PIN_15
#define LED_2_GREEN_GPIO_Port GPIOC
#define SD_DETECT_Pin GPIO_PIN_0
#define SD_DETECT_GPIO_Port GPIOC
#define SPI_SD_MISO_Pin GPIO_PIN_2
#define SPI_SD_MISO_GPIO_Port GPIOC
#define SPI_SD_MOSI_Pin GPIO_PIN_3
#define SPI_SD_MOSI_GPIO_Port GPIOC
#define SPI_SD_CS_Pin GPIO_PIN_0
#define SPI_SD_CS_GPIO_Port GPIOA
#define ETH_RST_Pin GPIO_PIN_3
#define ETH_RST_GPIO_Port GPIOA
#define LED_CAN1_RED_Pin GPIO_PIN_4
#define LED_CAN1_RED_GPIO_Port GPIOA
#define MCU_AIN_Pin GPIO_PIN_0
#define MCU_AIN_GPIO_Port GPIOB
#define LED_CAN1_GREEN_Pin GPIO_PIN_1
#define LED_CAN1_GREEN_GPIO_Port GPIOB
#define LED_CAN2_RED_Pin GPIO_PIN_7
#define LED_CAN2_RED_GPIO_Port GPIOE
#define LED_CAN2_GREEN_Pin GPIO_PIN_8
#define LED_CAN2_GREEN_GPIO_Port GPIOE
#define ETH_INTRP_Pin GPIO_PIN_10
#define ETH_INTRP_GPIO_Port GPIOB
#define LIN_MASTER_Pin GPIO_PIN_14
#define LIN_MASTER_GPIO_Port GPIOB
#define ETH_RXER_Pin GPIO_PIN_15
#define ETH_RXER_GPIO_Port GPIOB
#define LIN_CS_Pin GPIO_PIN_10
#define LIN_CS_GPIO_Port GPIOD
#define ADC_5V_Pin GPIO_PIN_11
#define ADC_5V_GPIO_Port GPIOD
#define CAN_STB_Pin GPIO_PIN_14
#define CAN_STB_GPIO_Port GPIOD
#define PUSH_PULL_DRIVE_Pin GPIO_PIN_6
#define PUSH_PULL_DRIVE_GPIO_Port GPIOC
#define SPI_SD_SCK_Pin GPIO_PIN_9
#define SPI_SD_SCK_GPIO_Port GPIOA
#define LED_LIN_RED_Pin GPIO_PIN_10
#define LED_LIN_RED_GPIO_Port GPIOA
#define LED_LIN_GREEN_Pin GPIO_PIN_15
#define LED_LIN_GREEN_GPIO_Port GPIOA
#define ETH_BOOT_Pin GPIO_PIN_5
#define ETH_BOOT_GPIO_Port GPIOD
#define LED_RS232_RED_Pin GPIO_PIN_6
#define LED_RS232_RED_GPIO_Port GPIOB
#define LED_RS232_GREEN_Pin GPIO_PIN_7
#define LED_RS232_GREEN_GPIO_Port GPIOB
#define LED_1_GREEN_Pin GPIO_PIN_8
#define LED_1_GREEN_GPIO_Port GPIOB
#define LED_1_RED_Pin GPIO_PIN_9
#define LED_1_RED_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */
#define USB_BUFFER_SIZE     512
/* Dip-switch reading macros */
/* Returns 0 for off, 1 for on */
#define READ_DIP1()             !HAL_GPIO_ReadPin(DIPSW_1_GPIO_Port, DIPSW_1_Pin)
#define READ_DIP2()             !HAL_GPIO_ReadPin(DIPSW_2_GPIO_Port, DIPSW_2_Pin)
#define READ_DIP3()             !HAL_GPIO_ReadPin(DIPSW_3_GPIO_Port, DIPSW_3_Pin)
#define READ_DIP4()             !HAL_GPIO_ReadPin(DIPSW_4_GPIO_Port, DIPSW_4_Pin)

#define READ_ETH_BOOT()         !HAL_GPIO_ReadPin(ETH_BOOT_GPIO_Port, ETH_BOOT_Pin)

/* Push-pull control */
#define PUSH_PULL_DRIVE_ON()    HAL_GPIO_WritePin(PUSH_PULL_DRIVE_GPIO_Port, PUSH_PULL_DRIVE_Pin, GPIO_PIN_SET)
#define PUSH_PULL_DRIVE_OFF()   HAL_GPIO_WritePin(PUSH_PULL_DRIVE_GPIO_Port, PUSH_PULL_DRIVE_Pin, GPIO_PIN_RESET)
#define PUSH_PULL_DRIVE(x)      (x) == 1 ? PUSH_PULL_DRIVE_ON() : PUSH_PULL_DRIVE_OFF();

/* LED control */
#define LED_CAN1_RED_ON()       HAL_GPIO_WritePin(LED_CAN1_RED_GPIO_Port, LED_CAN1_RED_Pin, GPIO_PIN_RESET)
#define LED_CAN1_RED_OFF()      HAL_GPIO_WritePin(LED_CAN1_RED_GPIO_Port, LED_CAN1_RED_Pin, GPIO_PIN_SET)
#define LED_CAN1_GREEN_ON()     HAL_GPIO_WritePin(LED_CAN1_GREEN_GPIO_Port, LED_CAN1_GREEN_Pin, GPIO_PIN_RESET)
#define LED_CAN1_GREEN_OFF()    HAL_GPIO_WritePin(LED_CAN1_GREEN_GPIO_Port, LED_CAN1_GREEN_Pin, GPIO_PIN_SET)
#define LED_CAN2_RED_ON()       HAL_GPIO_WritePin(LED_CAN2_RED_GPIO_Port, LED_CAN2_RED_Pin, GPIO_PIN_RESET)
#define LED_CAN2_RED_OFF()      HAL_GPIO_WritePin(LED_CAN2_RED_GPIO_Port, LED_CAN2_RED_Pin, GPIO_PIN_SET)
#define LED_CAN2_GREEN_ON()     HAL_GPIO_WritePin(LED_CAN2_GREEN_GPIO_Port, LED_CAN2_GREEN_Pin, GPIO_PIN_RESET)
#define LED_CAN2_GREEN_OFF()    HAL_GPIO_WritePin(LED_CAN2_GREEN_GPIO_Port, LED_CAN2_GREEN_Pin, GPIO_PIN_SET)

#define LED_LIN_RED_ON()        HAL_GPIO_WritePin(LED_LIN_RED_GPIO_Port, LED_LIN_RED_Pin, GPIO_PIN_RESET)
#define LED_LIN_RED_OFF()       HAL_GPIO_WritePin(LED_LIN_RED_GPIO_Port, LED_LIN_RED_Pin, GPIO_PIN_SET)
#define LED_LIN_GREEN_ON()      HAL_GPIO_WritePin(LED_LIN_GREEN_GPIO_Port, LED_LIN_GREEN_Pin, GPIO_PIN_RESET)
#define LED_LIN_GREEN_OFF()     HAL_GPIO_WritePin(LED_LIN_GREEN_GPIO_Port, LED_LIN_GREEN_Pin, GPIO_PIN_SET)

#define LED_RS232_RED_ON()      HAL_GPIO_WritePin(LED_RS232_RED_GPIO_Port, LED_RS232_RED_Pin, GPIO_PIN_RESET)
#define LED_RS232_RED_OFF()     HAL_GPIO_WritePin(LED_RS232_RED_GPIO_Port, LED_RS232_RED_Pin, GPIO_PIN_SET)
#define LED_RS232_GREEN_ON()    HAL_GPIO_WritePin(LED_RS232_GREEN_GPIO_Port, LED_RS232_GREEN_Pin, GPIO_PIN_RESET)
#define LED_RS232_GREEN_OFF()   HAL_GPIO_WritePin(LED_RS232_GREEN_GPIO_Port, LED_RS232_GREEN_Pin, GPIO_PIN_SET)

#define LED1_RED_ON()           HAL_GPIO_WritePin(LED_1_RED_GPIO_Port, LED_1_RED_Pin, GPIO_PIN_RESET)
#define LED1_RED_OFF()          HAL_GPIO_WritePin(LED_1_RED_GPIO_Port, LED_1_RED_Pin, GPIO_PIN_SET)
#define LED1_GREEN_ON()         HAL_GPIO_WritePin(LED_1_GREEN_GPIO_Port, LED_1_GREEN_Pin, GPIO_PIN_RESET)
#define LED1_GREEN_OFF()        HAL_GPIO_WritePin(LED_1_GREEN_GPIO_Port, LED_1_GREEN_Pin, GPIO_PIN_SET)
#define LED2_GREEN_ON()         HAL_GPIO_WritePin(LED_2_GREEN_GPIO_Port, LED_2_GREEN_Pin, GPIO_PIN_RESET)
#define LED2_GREEN_OFF()        HAL_GPIO_WritePin(LED_2_GREEN_GPIO_Port, LED_2_GREEN_Pin, GPIO_PIN_SET)
#define LED3_GREEN_ON()         HAL_GPIO_WritePin(LED_3_GREEN_GPIO_Port, LED_3_GREEN_Pin, GPIO_PIN_RESET)
#define LED3_GREEN_OFF()        HAL_GPIO_WritePin(LED_3_GREEN_GPIO_Port, LED_3_GREEN_Pin, GPIO_PIN_SET)

#define GET_5V_BRANCH_VOLTAGE() HAL_GPIO_ReadPin(ADC_5V_GPIO_Port, ADC_5V_Pin)

#define ADC_CONVERSION          (2.5 / 65536)
#define IO_VALUE(x)             ((double) x * ADC_CONVERSION * 2 * 1000)


#define FLASH_BASE_ADDR             (uint32_t)(FLASH_BASE)
#define FLASH_END_ADDR              (uint32_t)(0x080FFFFF)

/* Base address of the Flash sectors Bank 1 */
#define ADDR_FLASH_SECTOR_0_BANK1   ((uint32_t)0x08000000) /* Base @ of Sector 0, 128 Kbytes */
#define ADDR_FLASH_SECTOR_1_BANK1   ((uint32_t)0x08020000) /* Base @ of Sector 1, 128 Kbytes */
#define ADDR_FLASH_SECTOR_2_BANK1   ((uint32_t)0x08040000) /* Base @ of Sector 2, 128 Kbytes */
#define ADDR_FLASH_SECTOR_3_BANK1   ((uint32_t)0x08060000) /* Base @ of Sector 3, 128 Kbytes */
#define ADDR_FLASH_SECTOR_4_BANK1   ((uint32_t)0x08080000) /* Base @ of Sector 4, 128 Kbytes */
#define ADDR_FLASH_SECTOR_5_BANK1   ((uint32_t)0x080A0000) /* Base @ of Sector 5, 128 Kbytes */
#define ADDR_FLASH_SECTOR_6_BANK1   ((uint32_t)0x080C0000) /* Base @ of Sector 6, 128 Kbytes */
#define ADDR_FLASH_SECTOR_7_BANK1   ((uint32_t)0x080E0000) /* Base @ of Sector 7, 128 Kbytes */
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
