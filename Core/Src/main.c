/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : MACH ETH Example
  ******************************************************************************
  * @attention
  *
  * Product: MACH ETH
  *
  * Company: MACH SYSTEMS s.r.o.
  * Company web: www.machsystems.cz
  *
  * Product description:
  * - 10/100 Ethernet port
  * - 2 CAN-HS channels with CAN FD support
  * - LIN channel
  * - USB 2.0
  * - RS-232
  * - Analogue input and digital output
  * - 4 DIP switches
  * - 10 status LEDs
  * - Open communication protocol for integration over Ethernet and USB
  * - User-programmable
  * - Externally or USB-powered
  * - Table-top use or DIN-rail mount
  *
  * MCU: STM32H735VG
  * IDE: STM32CubeIDE v1.9.0
  * HAL: STM32Cube FW_H7 V1.10.0
  *
  *
  * Connector pinout of the device:
  * ===============================
  * Connector X5 (D-SUB Male):
  *   - 1 AI1 (0-5V)
  *   - 2 CAN1 L
  *   - 3 GND
  *   - 4 CAN2 L
  *   - 5 GND
  *   - 6 LIN
  *   - 7 CAN1 H
  *   - 8 CAN2 H
  *   - 9 Vin/Vbat
  *
  * Connector X6 (D-SUB Male):
  *   - 1 DO1  (5 V push-pull)
  *   - 2 NC
  *   - 3 GND
  *   - 4 NC
  *   - 5 GND
  *   - 6 RS-232 TxD (out)
  *   - 7 NC
  *   - 8 NC
  *   - 9 RS-232 RxD (in)
  *
  * Description of the example:
  * ===========================
  * CAN-FD:
  *  Default setting for CAN is 500k, 2M, SP = 80 %.
  *  Everything received on CAN is sent to USB and to TCP connection (if there
  *  is).
  *
  * LIN:
  *   In LIN scheduler, there is one frame:
  * ID 0x01 - Master Response
  *         - 4 data bytes
  *         - LSB of bytes 0, 1, 2, 3 contain respectively state of the
  *           dip switches
  *
  * Bootloader:
  *  CAN frame with extended frame ID 0x1fffffff and first four data bytes 0x00
  * 0x01 0x02 0x03 (on CAN1 or CAN2) will reset the device to HTTP bootloader.
  * If you then connect to the device via ethernet and connect from web browser
  * (Google Chrome recommended), you can upload new firmware to the device. If
  * the binary is built as without bootloader, device will reset to System
  * Bootloader. If you then connect USB cable, you can upload new firmware. See
  * below for build possibilities. Furthermore, see manual for more info.
  *
  * Virtual COM Port:
  *   If there is anything received, there is a frame transmitted on CAN1.
  *
  * EEPROM:
  *  There is running an EEPROM emulation in flash. Program tries to load conf-
  * iguration from there.
  *
  * RS-232:
  *  Received data are sent back. Uncomment TEST_UART for this example.
  * Settings are: 115200, 8 B, 1 stop bit, no parity.
  *
  * Analog Input:
  *  There is one analog input that can measure 0 to 5 V. You can get its value
  * using GetAdcMeasurement().
  *
  * Output:
  *  There is one 5 V push-pull. There are macros defined in main.h (in section
  * Push-pull control) for controlling it, but they are not used in this example.
  *
  *  Also, there are three LEDs, macros for using them are in main.h in section
  * User LED controls.
  *
  *  Note that currently there are two possible build options, Bootloader and
  * NoBootloader. Each has its own linker file. First is to be used with our
  * TCP bootloader, you can upload this app via a web browser (we recommend
  * Google Chrome). See manual for details. Second one should be used without
  * bootloader.
  *
  *
  *
  * Mapping of the peripherals to their names:
  * ==========================================
  * CAN1: FDCAN3 peripheral
  * CAN2: FDCAN1 peripheral
  *
  * LIN:   USART3
  * RS232: UART5
  *
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  *
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"
#include "lwip.h"
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "tcpServer.h"
#include <string.h>         /* memcpy() */
#include "lin.h"
#include <usbd_cdc_if.h>
#include "eeprom_emul.h"
#include "config.h"
#include "commControl.h"    /*  ProtocolTimeoutCallback() */
#include "sharedParams.h"
#include "can.h"
#include "packetControl.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define CAN_MAX_DATALEN     64
#define LIN_TX_DATA_SIZE    4

#define TEST_UART

//#define TEST_CAN

#define SYSTEM_BOOT_ADDR    0x1FF09800
#define HTTP_BOOT_ADDR      0x08000000
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

FDCAN_HandleTypeDef hfdcan1;
FDCAN_HandleTypeDef hfdcan3;

SPI_HandleTypeDef hspi2;

TIM_HandleTypeDef htim13;
TIM_HandleTypeDef htim14;
TIM_HandleTypeDef htim16;

UART_HandleTypeDef huart5;

/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for usbProtocolTask */
osThreadId_t usbProtocolTaskHandle;
const osThreadAttr_t usbProtocolTask_attributes = {
  .name = "usbProtocolTask",
  .stack_size = 1024 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for UsbRxQueue */
osMessageQueueId_t UsbRxQueueHandle;
const osMessageQueueAttr_t UsbRxQueue_attributes = {
  .name = "UsbRxQueue"
};
/* Definitions for TcpTxQueue */
osMessageQueueId_t TcpTxQueueHandle;
const osMessageQueueAttr_t TcpTxQueue_attributes = {
  .name = "TcpTxQueue"
};
/* USER CODE BEGIN PV */
FDCAN_RxHeaderTypeDef RxHeader;
uint8_t RxData[CAN_MAX_DATALEN];
FDCAN_TxHeaderTypeDef TxHeader;
uint8_t TxData[CAN_MAX_DATALEN];

uint8_t LinGlobalData[LIN_DATA_LENGTH];

/* String to send via uart */
uint8_t aTextInfoStart[] = "\r\nUART Debug Example: Received data will be sent back:\r\n";
uint8_t DbgReceive = 'a';

uint16_t gTimerCnt = 0;

/* FATFs process status */
int32_t ProcessStatus = 0;

/* Represents state of the dipswitches *at the start* */
uint8_t GlobalBtnState;

/* Request to go to bootloader */
uint8_t BootloaderRequest = 0;

uint8_t linTxData[LIN_TX_DATA_SIZE] = {0, 0, 0};
LIN_Frame* pLinRxFrame;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void PeriphCommonClock_Config(void);
static void MPU_Config(void);
static void MX_GPIO_Init(void);
static void MX_FDCAN1_Init(void);
static void MX_FDCAN3_Init(void);
static void MX_UART5_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_TIM7_Init(void);
static void MX_TIM13_Init(void);
static void MX_TIM14_Init(void);
static void MX_TIM16_Init(void);
static void MX_ADC1_Init(void);
static void MX_SPI2_Init(void);
void StartDefaultTask(void *argument);
extern void UsbProtocolTask(void *argument);

/* USER CODE BEGIN PFP */
/*
 * Read the DIP switches.
 */
void testDipsw(void);

/*
 * Test the outputs.
 */
void testIo(void);

/*
 * Jump to the system bootloader.
 * Almost entirely from:
 * https://community.st.com/s/article/STM32H7-bootloader-jump-from-application
 * Other source:
 * https://www.programmersought.com/article/56553023460/
 * https://stm32f4-discovery.net/2017/04/tutorial-jump-system-memory-software-stm32/
 */
void jumpToBootloader(uint32_t address);

/*
 * Blink with all LEDs
 */
void ledTest(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
  /* USER CODE END 1 */

  /* MPU Configuration--------------------------------------------------------*/
  MPU_Config();

  /* Enable D-Cache---------------------------------------------------------*/
  SCB_EnableDCache();

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

/* Configure the peripherals common clocks */
  PeriphCommonClock_Config();

  /* USER CODE BEGIN SysInit */
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_FDCAN1_Init();
  MX_FDCAN3_Init();
  MX_UART5_Init();
  MX_USART3_UART_Init();
  MX_TIM7_Init();
  MX_TIM13_Init();
  MX_TIM14_Init();
  MX_TIM16_Init();
  MX_ADC1_Init();
  MX_SPI2_Init();
  /* USER CODE BEGIN 2 */
  HAL_ADCEx_Calibration_Start(&hadc1, ADC_CALIB_OFFSET, ADC_SINGLE_ENDED);
  HAL_ADCEx_Calibration_Start(&hadc1, ADC_CALIB_OFFSET_LINEARITY, ADC_SINGLE_ENDED);

  InitNonVolatileData();
  // Disable interrupt pin for now
  HAL_NVIC_DisableIRQ(EXTI9_5_IRQn);

  LED_RS232_GREEN_OFF();
  LED_RS232_RED_OFF();
  LED_CAN1_GREEN_OFF();
  LED_CAN1_RED_OFF();
  LED_CAN2_GREEN_OFF();
  LED_CAN2_RED_OFF();
  LED_LIN_GREEN_OFF();
  LED_LIN_RED_OFF();
  LED1_GREEN_OFF();
  LED1_RED_OFF();
  LED2_GREEN_OFF();
  LED3_GREEN_OFF();

  testDipsw();
  testIo();

  /* Enable the CAN transceiver */
  HAL_GPIO_WritePin(CAN_STB_GPIO_Port, CAN_STB_Pin, GPIO_PIN_RESET);

  /* Start CAN FD peripherals with default configuration*/
  DefaultConfigurationCanCh1();
  CanStartChannel(CAN1_NUM);
  DefaultConfigurationCanCh2();
  CanStartChannel(CAN2_NUM);

  /* Wake up the LIN transceiver */
  HAL_GPIO_WritePin(LIN_CS_GPIO_Port, LIN_CS_Pin, GPIO_PIN_SET);

  /* Start LIN peripheral with default configuration*/
  DefaultConfigurationLIN();
  InitLin(GetConfigurationLIN().LinConfiguration);

  HAL_TIM_Base_Start_IT(&htim13);

#ifdef TEST_UART
  /* Do not send ending '\0' */
  HAL_UART_Transmit(&huart5, aTextInfoStart, sizeof(aTextInfoStart) - 1, HAL_MAX_DELAY);
#endif /* TEST_UART */
  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the queue(s) */
  /* creation of UsbRxQueue */
  UsbRxQueueHandle = osMessageQueueNew (4, sizeof(GenericMessageType), &UsbRxQueue_attributes);

  /* creation of TcpTxQueue */
  TcpTxQueueHandle = osMessageQueueNew (4, sizeof(GenericMessageType), &TcpTxQueue_attributes);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* creation of usbProtocolTask */
  usbProtocolTaskHandle = osThreadNew(UsbProtocolTask, NULL, &usbProtocolTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

  /* Start scheduler */
  osKernelStart();
  /* We should never get here as control is now taken by the scheduler */
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Supply configuration update enable
  */
  HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 96;
  RCC_OscInitStruct.PLL.PLLP = 2;
  RCC_OscInitStruct.PLL.PLLQ = 12;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_2;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV2;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief Peripherals Common Clock Configuration
  * @retval None
  */
void PeriphCommonClock_Config(void)
{
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  /** Initializes the peripherals clock
  */
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_ADC|RCC_PERIPHCLK_UART5
                              |RCC_PERIPHCLK_FDCAN|RCC_PERIPHCLK_USART3;
  PeriphClkInitStruct.PLL2.PLL2M = 4;
  PeriphClkInitStruct.PLL2.PLL2N = 40;
  PeriphClkInitStruct.PLL2.PLL2P = 8;
  PeriphClkInitStruct.PLL2.PLL2Q = 3;
  PeriphClkInitStruct.PLL2.PLL2R = 2;
  PeriphClkInitStruct.PLL2.PLL2RGE = RCC_PLL2VCIRANGE_2;
  PeriphClkInitStruct.PLL2.PLL2VCOSEL = RCC_PLL2VCOWIDE;
  PeriphClkInitStruct.PLL2.PLL2FRACN = 0;
  PeriphClkInitStruct.FdcanClockSelection = RCC_FDCANCLKSOURCE_PLL2;
  PeriphClkInitStruct.Usart234578ClockSelection = RCC_USART234578CLKSOURCE_PLL2;
  PeriphClkInitStruct.AdcClockSelection = RCC_ADCCLKSOURCE_PLL2;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_MultiModeTypeDef multimode = {0};
  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc1.Init.Resolution = ADC_RESOLUTION_16B;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ConversionDataManagement = ADC_CONVERSIONDATA_DR;
  hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc1.Init.LeftBitShift = ADC_LEFTBITSHIFT_NONE;
  hadc1.Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure the ADC multi-mode
  */
  multimode.Mode = ADC_MODE_INDEPENDENT;
  if (HAL_ADCEx_MultiModeConfigChannel(&hadc1, &multimode) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_9;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_16CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  sConfig.OffsetSignedSaturation = DISABLE;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief FDCAN1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_FDCAN1_Init(void)
{

  /* USER CODE BEGIN FDCAN1_Init 0 */

  /* USER CODE END FDCAN1_Init 0 */

  /* USER CODE BEGIN FDCAN1_Init 1 */

  /* USER CODE END FDCAN1_Init 1 */
  hfdcan1.Instance = FDCAN1;
  hfdcan1.Init.FrameFormat = FDCAN_FRAME_FD_BRS;
  hfdcan1.Init.Mode = FDCAN_MODE_NORMAL;
  hfdcan1.Init.AutoRetransmission = DISABLE;
  hfdcan1.Init.TransmitPause = DISABLE;
  hfdcan1.Init.ProtocolException = DISABLE;
  hfdcan1.Init.NominalPrescaler = 4;
  hfdcan1.Init.NominalSyncJumpWidth = 3;
  hfdcan1.Init.NominalTimeSeg1 = 14;
  hfdcan1.Init.NominalTimeSeg2 = 3;
  hfdcan1.Init.DataPrescaler = 2;
  hfdcan1.Init.DataSyncJumpWidth = 3;
  hfdcan1.Init.DataTimeSeg1 = 14;
  hfdcan1.Init.DataTimeSeg2 = 3;
  hfdcan1.Init.MessageRAMOffset = 0;
  hfdcan1.Init.StdFiltersNbr = 1;
  hfdcan1.Init.ExtFiltersNbr = 1;
  hfdcan1.Init.RxFifo0ElmtsNbr = 64;
  hfdcan1.Init.RxFifo0ElmtSize = FDCAN_DATA_BYTES_64;
  hfdcan1.Init.RxFifo1ElmtsNbr = 0;
  hfdcan1.Init.RxFifo1ElmtSize = FDCAN_DATA_BYTES_8;
  hfdcan1.Init.RxBuffersNbr = 8;
  hfdcan1.Init.RxBufferSize = FDCAN_DATA_BYTES_8;
  hfdcan1.Init.TxEventsNbr = 8;
  hfdcan1.Init.TxBuffersNbr = 8;
  hfdcan1.Init.TxFifoQueueElmtsNbr = 32;
  hfdcan1.Init.TxFifoQueueMode = FDCAN_TX_FIFO_OPERATION;
  hfdcan1.Init.TxElmtSize = FDCAN_DATA_BYTES_64;
  if (HAL_FDCAN_Init(&hfdcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN FDCAN1_Init 2 */

  /* USER CODE END FDCAN1_Init 2 */

}

/**
  * @brief FDCAN3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_FDCAN3_Init(void)
{

  /* USER CODE BEGIN FDCAN3_Init 0 */

  /* USER CODE END FDCAN3_Init 0 */

  /* USER CODE BEGIN FDCAN3_Init 1 */

  /* USER CODE END FDCAN3_Init 1 */
  hfdcan3.Instance = FDCAN3;
  hfdcan3.Init.FrameFormat = FDCAN_FRAME_FD_BRS;
  hfdcan3.Init.Mode = FDCAN_MODE_NORMAL;
  hfdcan3.Init.AutoRetransmission = DISABLE;
  hfdcan3.Init.TransmitPause = DISABLE;
  hfdcan3.Init.ProtocolException = DISABLE;
  hfdcan3.Init.NominalPrescaler = 4;
  hfdcan3.Init.NominalSyncJumpWidth = 3;
  hfdcan3.Init.NominalTimeSeg1 = 14;
  hfdcan3.Init.NominalTimeSeg2 = 3;
  hfdcan3.Init.DataPrescaler = 2;
  hfdcan3.Init.DataSyncJumpWidth = 3;
  hfdcan3.Init.DataTimeSeg1 = 14;
  hfdcan3.Init.DataTimeSeg2 = 3;
  hfdcan3.Init.MessageRAMOffset = 0;
  hfdcan3.Init.StdFiltersNbr = 1;
  hfdcan3.Init.ExtFiltersNbr = 1;
  hfdcan3.Init.RxFifo0ElmtsNbr = 64;
  hfdcan3.Init.RxFifo0ElmtSize = FDCAN_DATA_BYTES_64;
  hfdcan3.Init.RxFifo1ElmtsNbr = 0;
  hfdcan3.Init.RxFifo1ElmtSize = FDCAN_DATA_BYTES_8;
  hfdcan3.Init.RxBuffersNbr = 8;
  hfdcan3.Init.RxBufferSize = FDCAN_DATA_BYTES_8;
  hfdcan3.Init.TxEventsNbr = 8;
  hfdcan3.Init.TxBuffersNbr = 8;
  hfdcan3.Init.TxFifoQueueElmtsNbr = 32;
  hfdcan3.Init.TxFifoQueueMode = FDCAN_TX_FIFO_OPERATION;
  hfdcan3.Init.TxElmtSize = FDCAN_DATA_BYTES_64;
  if (HAL_FDCAN_Init(&hfdcan3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN FDCAN3_Init 2 */

  /* USER CODE END FDCAN3_Init 2 */

}

/**
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_4BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 0x0;
  hspi2.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  hspi2.Init.NSSPolarity = SPI_NSS_POLARITY_LOW;
  hspi2.Init.FifoThreshold = SPI_FIFO_THRESHOLD_01DATA;
  hspi2.Init.TxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
  hspi2.Init.RxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
  hspi2.Init.MasterSSIdleness = SPI_MASTER_SS_IDLENESS_00CYCLE;
  hspi2.Init.MasterInterDataIdleness = SPI_MASTER_INTERDATA_IDLENESS_00CYCLE;
  hspi2.Init.MasterReceiverAutoSusp = SPI_MASTER_RX_AUTOSUSP_DISABLE;
  hspi2.Init.MasterKeepIOState = SPI_MASTER_KEEP_IO_STATE_DISABLE;
  hspi2.Init.IOSwap = SPI_IO_SWAP_DISABLE;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

}

/**
  * @brief TIM7 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM7_Init(void)
{

  /* USER CODE BEGIN TIM7_Init 0 */

  /* USER CODE END TIM7_Init 0 */

  LL_TIM_InitTypeDef TIM_InitStruct = {0};

  /* Peripheral clock enable */
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_TIM7);

  /* TIM7 interrupt Init */
  NVIC_SetPriority(TIM7_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),5, 0));
  NVIC_EnableIRQ(TIM7_IRQn);

  /* USER CODE BEGIN TIM7_Init 1 */

  /* USER CODE END TIM7_Init 1 */
  TIM_InitStruct.Prescaler = 0;
  TIM_InitStruct.CounterMode = LL_TIM_COUNTERMODE_UP;
  TIM_InitStruct.Autoreload = 65535;
  LL_TIM_Init(TIM7, &TIM_InitStruct);
  LL_TIM_DisableARRPreload(TIM7);
  LL_TIM_SetTriggerOutput(TIM7, LL_TIM_TRGO_RESET);
  LL_TIM_DisableMasterSlaveMode(TIM7);
  /* USER CODE BEGIN TIM7_Init 2 */

  /* USER CODE END TIM7_Init 2 */

}

/**
  * @brief TIM13 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM13_Init(void)
{

  /* USER CODE BEGIN TIM13_Init 0 */

  /* USER CODE END TIM13_Init 0 */

  /* USER CODE BEGIN TIM13_Init 1 */

  /* USER CODE END TIM13_Init 1 */
  htim13.Instance = TIM13;
  htim13.Init.Prescaler = 71;
  htim13.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim13.Init.Period = 999;
  htim13.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim13.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim13) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM13_Init 2 */

  /* USER CODE END TIM13_Init 2 */

}

/**
  * @brief TIM14 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM14_Init(void)
{

  /* USER CODE BEGIN TIM14_Init 0 */

  /* USER CODE END TIM14_Init 0 */

  /* USER CODE BEGIN TIM14_Init 1 */

  /* USER CODE END TIM14_Init 1 */
  htim14.Instance = TIM14;
  htim14.Init.Prescaler = 9999;
  htim14.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim14.Init.Period = 71;
  htim14.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim14.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim14) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM14_Init 2 */

  /* USER CODE END TIM14_Init 2 */

}

/**
  * @brief TIM16 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM16_Init(void)
{

  /* USER CODE BEGIN TIM16_Init 0 */

  /* USER CODE END TIM16_Init 0 */

  /* USER CODE BEGIN TIM16_Init 1 */

  /* USER CODE END TIM16_Init 1 */
  htim16.Instance = TIM16;
  htim16.Init.Prescaler = 1;
  htim16.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim16.Init.Period = 35999;
  htim16.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim16.Init.RepetitionCounter = 0;
  htim16.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim16) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM16_Init 2 */

  /* USER CODE END TIM16_Init 2 */

}

/**
  * @brief UART5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART5_Init(void)
{

  /* USER CODE BEGIN UART5_Init 0 */

  /* USER CODE END UART5_Init 0 */

  /* USER CODE BEGIN UART5_Init 1 */

  /* USER CODE END UART5_Init 1 */
  huart5.Instance = UART5;
  huart5.Init.BaudRate = 115200;
  huart5.Init.WordLength = UART_WORDLENGTH_8B;
  huart5.Init.StopBits = UART_STOPBITS_1;
  huart5.Init.Parity = UART_PARITY_NONE;
  huart5.Init.Mode = UART_MODE_TX_RX;
  huart5.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart5.Init.OverSampling = UART_OVERSAMPLING_16;
  huart5.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart5.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart5.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart5) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart5, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart5, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart5) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART5_Init 2 */

  /* USER CODE END UART5_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  LL_USART_InitTypeDef USART_InitStruct = {0};

  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* Peripheral clock enable */
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_USART3);

  LL_AHB4_GRP1_EnableClock(LL_AHB4_GRP1_PERIPH_GPIOD);
  /**USART3 GPIO Configuration
  PD8   ------> USART3_TX
  PD9   ------> USART3_RX
  */
  GPIO_InitStruct.Pin = LL_GPIO_PIN_8|LL_GPIO_PIN_9;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_MEDIUM;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_7;
  LL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /* USART3 interrupt Init */
  NVIC_SetPriority(USART3_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),5, 0));
  NVIC_EnableIRQ(USART3_IRQn);

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  USART_InitStruct.PrescalerValue = LL_USART_PRESCALER_DIV1;
  USART_InitStruct.BaudRate = 19200;
  USART_InitStruct.DataWidth = LL_USART_DATAWIDTH_8B;
  USART_InitStruct.StopBits = LL_USART_STOPBITS_1;
  USART_InitStruct.Parity = LL_USART_PARITY_NONE;
  USART_InitStruct.TransferDirection = LL_USART_DIRECTION_TX_RX;
  USART_InitStruct.OverSampling = LL_USART_OVERSAMPLING_16;
  LL_USART_Init(USART3, &USART_InitStruct);
  LL_USART_SetTXFIFOThreshold(USART3, LL_USART_FIFOTHRESHOLD_1_8);
  LL_USART_SetRXFIFOThreshold(USART3, LL_USART_FIFOTHRESHOLD_1_8);
  LL_USART_DisableFIFO(USART3);
  LL_USART_SetLINBrkDetectionLen(USART3, LL_USART_LINBREAK_DETECT_10B);
  LL_USART_DisableOverrunDetect(USART3);
  LL_USART_DisableDMADeactOnRxErr(USART3);
  LL_USART_ConfigLINMode(USART3);

  /* USER CODE BEGIN WKUPType USART3 */

  /* USER CODE END WKUPType USART3 */

  LL_USART_Enable(USART3);

  /* Polling USART3 initialisation */
  while((!(LL_USART_IsActiveFlag_TEACK(USART3))) || (!(LL_USART_IsActiveFlag_REACK(USART3))))
  {
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, LED_3_GREEN_Pin|LED_2_GREEN_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, SPI_SD_CS_Pin|ETH_RST_Pin|LED_CAN1_RED_Pin|LED_LIN_RED_Pin
                          |LED_LIN_GREEN_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LED_CAN1_GREEN_Pin|LED_RS232_RED_Pin|LED_RS232_GREEN_Pin|LED_1_GREEN_Pin
                          |LED_1_RED_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, LED_CAN2_RED_Pin|LED_CAN2_GREEN_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LIN_MASTER_GPIO_Port, LIN_MASTER_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LIN_CS_GPIO_Port, LIN_CS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(CAN_STB_GPIO_Port, CAN_STB_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(PUSH_PULL_DRIVE_GPIO_Port, PUSH_PULL_DRIVE_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : DIPSW_1_Pin DIPSW_2_Pin DIPSW_3_Pin */
  GPIO_InitStruct.Pin = DIPSW_1_Pin|DIPSW_2_Pin|DIPSW_3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : DIPSW_4_Pin SD_DETECT_Pin */
  GPIO_InitStruct.Pin = DIPSW_4_Pin|SD_DETECT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : LED_3_GREEN_Pin LED_2_GREEN_Pin PUSH_PULL_DRIVE_Pin */
  GPIO_InitStruct.Pin = LED_3_GREEN_Pin|LED_2_GREEN_Pin|PUSH_PULL_DRIVE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : SPI_SD_CS_Pin */
  GPIO_InitStruct.Pin = SPI_SD_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(SPI_SD_CS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : ETH_RST_Pin LED_CAN1_RED_Pin LED_LIN_RED_Pin LED_LIN_GREEN_Pin */
  GPIO_InitStruct.Pin = ETH_RST_Pin|LED_CAN1_RED_Pin|LED_LIN_RED_Pin|LED_LIN_GREEN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : LED_CAN1_GREEN_Pin LIN_MASTER_Pin LED_RS232_RED_Pin LED_RS232_GREEN_Pin
                           LED_1_GREEN_Pin LED_1_RED_Pin */
  GPIO_InitStruct.Pin = LED_CAN1_GREEN_Pin|LIN_MASTER_Pin|LED_RS232_RED_Pin|LED_RS232_GREEN_Pin
                          |LED_1_GREEN_Pin|LED_1_RED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : LED_CAN2_RED_Pin LED_CAN2_GREEN_Pin */
  GPIO_InitStruct.Pin = LED_CAN2_RED_Pin|LED_CAN2_GREEN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : ETH_INTRP_Pin */
  GPIO_InitStruct.Pin = ETH_INTRP_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(ETH_INTRP_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : ETH_RXER_Pin */
  GPIO_InitStruct.Pin = ETH_RXER_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(ETH_RXER_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LIN_CS_Pin CAN_STB_Pin */
  GPIO_InitStruct.Pin = LIN_CS_Pin|CAN_STB_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : ADC_5V_Pin */
  GPIO_InitStruct.Pin = ADC_5V_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(ADC_5V_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : ETH_BOOT_Pin */
  GPIO_InitStruct.Pin = ETH_BOOT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(ETH_BOOT_GPIO_Port, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart == &huart5)
  {
    HAL_UART_Transmit_IT(&huart5, &DbgReceive, 1);
    HAL_UART_Receive_IT(&huart5, &DbgReceive, 1);
  }
}

void testDipsw(void)
{
  GlobalBtnState = READ_DIP1();
  GlobalBtnState |= READ_DIP2() << 1;
  GlobalBtnState |= READ_DIP3() << 2;
  GlobalBtnState |= READ_DIP4() << 3;
}

void testIo(void)
{
  HAL_GPIO_WritePin(PUSH_PULL_DRIVE_GPIO_Port, PUSH_PULL_DRIVE_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(PUSH_PULL_DRIVE_GPIO_Port, PUSH_PULL_DRIVE_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(PUSH_PULL_DRIVE_GPIO_Port, PUSH_PULL_DRIVE_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(PUSH_PULL_DRIVE_GPIO_Port, PUSH_PULL_DRIVE_Pin, GPIO_PIN_RESET);
}

void jumpToBootloader(uint32_t address)
{
  uint32_t i=0;
  void (*SysMemBootJump)(void);

  /* Set the address of the entry point to bootloader */
  volatile uint32_t BootAddr = address;

  /* Disable all interrupts */
  __disable_irq();

  /* Disable Systick timer */
  SysTick->CTRL = 0;

  /* Set the clock to the default state */
  HAL_RCC_DeInit();

  /* Deinitialize all the peripherals */
  HAL_DeInit();

  /* Must disable the caches! */
  SCB_DisableDCache();
  SCB_DisableICache();

  /* Clear Interrupt Enable Register & Interrupt Pending Register */
  for (i = 0; i < 5; i++)
  {
    NVIC->ICER[i] = 0xFFFFFFFF;
    NVIC->ICPR[i] = 0xFFFFFFFF;
  }

  /* Set vector table offset to 0 */
  SCB->VTOR = 0;

  /* Re-enable all interrupts */
  __enable_irq();

  /* Set up the jump to booloader address + 4 */
  SysMemBootJump = (void (*)(void)) (*((uint32_t *) ((BootAddr + 4))));

  /* Set the main stack pointer to the bootloader stack */
  __set_MSP(*(uint32_t *)BootAddr);

  /* RTOS in engineering, this statement is very important to privileged mode, use the MSP pointer */
  __set_CONTROL(0);

  /* Call the function to jump to bootloader location */
  SysMemBootJump();

  /* Jump is done successfully */
  while (1)
  {
    /* Code should never reach this loop */
  }
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
}

void ledTest(void)
{
  LED_RS232_GREEN_ON();
  LED_RS232_RED_ON();
  LED_CAN1_GREEN_ON();
  LED_CAN1_RED_ON();
  LED_CAN2_GREEN_ON();
  LED_CAN2_RED_ON();
  LED_LIN_GREEN_ON();
  LED_LIN_RED_ON();
  LED1_GREEN_ON();
  LED1_RED_ON();
  LED2_GREEN_ON();
  LED3_GREEN_ON();

  LED_RS232_GREEN_OFF();
  LED_RS232_RED_OFF();
  LED_CAN1_GREEN_OFF();
  LED_CAN1_RED_OFF();
  LED_CAN2_GREEN_OFF();
  LED_CAN2_RED_OFF();
  LED_LIN_GREEN_OFF();
  LED_LIN_RED_OFF();
  LED1_GREEN_OFF();
  LED1_RED_OFF();
  LED2_GREEN_OFF();
  LED3_GREEN_OFF();
}

uint16_t GetAdcMeasurement(void)
{
  HAL_ADC_Start(&hadc1);
  HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY);
  uint32_t value = HAL_ADC_GetValue(&hadc1);

  return IO_VALUE(value);
}
/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* init code for LWIP */
  MX_LWIP_Init();

  /* init code for USB_DEVICE */
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN 5 */
  /* Initialize the TCP server thread */
  TcpServerInit(argument);
  /* Set up LIN to send status of dipswitches */
  initLinScheduler();
  LinSchedulerStart();
#ifdef TEST_CAN
  HAL_StatusTypeDef ret;
  if ((ret = HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, (FDCAN_TxHeaderTypeDef*) &TxHeader, TxData)) != HAL_OK)
    Error_Handler();
  if ((ret = HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan3, (FDCAN_TxHeaderTypeDef*) &TxHeader, TxData)) != HAL_OK)
    Error_Handler();
#endif /* TEST_CAN */

  HAL_UART_Receive_IT(&huart5, &DbgReceive, 1);

  /* Infinite loop */
   for(;;)
   {
   /* Dip switches active in zero */
     if (!READ_DIP1())
       linTxData[0] |= 1;
     else
       linTxData[0] &= 0;

     if (!READ_DIP2())
       linTxData[1] |= 1;
     else
       linTxData[1] &= 0;

     if (!READ_DIP3())
       linTxData[2] |= 1;
     else
       linTxData[2] &= 0;

     if (!READ_DIP4())
       linTxData[3] |= 1;
     else
       linTxData[3] &= 0;

     if (READ_ETH_BOOT())
       BootloaderRequest = 2;

     UpdateTxFrameData(1, linTxData, LIN_TX_DATA_SIZE);
     if (BootloaderRequest == 1)
       jumpToBootloader(SYSTEM_BOOT_ADDR);
     else if (BootloaderRequest == 2)
     {
       InitSharedParams();
       SetSharedData(0, 1);
       jumpToBootloader(HTTP_BOOT_ADDR);
     }

     osDelay(100);
   }
  /* USER CODE END 5 */
}

/* MPU Configuration */

void MPU_Config(void)
{
  MPU_Region_InitTypeDef MPU_InitStruct = {0};

  /* Disables the MPU */
  HAL_MPU_Disable();

  /** Initializes and configures the Region and the memory to be protected
  */
  MPU_InitStruct.Enable = MPU_REGION_ENABLE;
  MPU_InitStruct.Number = MPU_REGION_NUMBER0;
  MPU_InitStruct.BaseAddress = 0x0;
  MPU_InitStruct.Size = MPU_REGION_SIZE_4GB;
  MPU_InitStruct.SubRegionDisable = 0x87;
  MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL0;
  MPU_InitStruct.AccessPermission = MPU_REGION_NO_ACCESS;
  MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_DISABLE;
  MPU_InitStruct.IsShareable = MPU_ACCESS_SHAREABLE;
  MPU_InitStruct.IsCacheable = MPU_ACCESS_NOT_CACHEABLE;
  MPU_InitStruct.IsBufferable = MPU_ACCESS_NOT_BUFFERABLE;

  HAL_MPU_ConfigRegion(&MPU_InitStruct);

  /** Initializes and configures the Region and the memory to be protected
  */
  MPU_InitStruct.Number = MPU_REGION_NUMBER1;
  MPU_InitStruct.BaseAddress = 0x30000000;
  MPU_InitStruct.Size = MPU_REGION_SIZE_1KB;
  MPU_InitStruct.SubRegionDisable = 0x0;
  MPU_InitStruct.AccessPermission = MPU_REGION_FULL_ACCESS;
  MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_ENABLE;
  MPU_InitStruct.IsShareable = MPU_ACCESS_NOT_SHAREABLE;
  MPU_InitStruct.IsBufferable = MPU_ACCESS_BUFFERABLE;

  HAL_MPU_ConfigRegion(&MPU_InitStruct);

  /** Initializes and configures the Region and the memory to be protected
  */
  MPU_InitStruct.Number = MPU_REGION_NUMBER2;
  MPU_InitStruct.BaseAddress = 0x30004000;
  MPU_InitStruct.Size = MPU_REGION_SIZE_16KB;
  MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL1;
  MPU_InitStruct.IsShareable = MPU_ACCESS_SHAREABLE;
  MPU_InitStruct.IsBufferable = MPU_ACCESS_NOT_BUFFERABLE;

  HAL_MPU_ConfigRegion(&MPU_InitStruct);
  /* Enables the MPU */
  HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);

}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM6 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */
  if (htim == &htim13)
  {
    /* LIN scheduler timer tick */
    SchedulerTimerCallback();

    /* LED LIN blink control timer tick */
    LinLedTimerCallback();

    /* LED CAN blink control timer tick */
    CanLedTimerCallback();
  }

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM6) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
