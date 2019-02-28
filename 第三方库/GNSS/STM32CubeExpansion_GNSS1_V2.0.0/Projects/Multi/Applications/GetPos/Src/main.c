/**
  *******************************************************************************
  * @file    Applications/GetPos/Src/main.c
  * @author  AST/CL
  * @version V2.0.0
  * @date    Apr-2018
  * @brief   This application shows how real time GNSS data received by the GNSS
  *          Teseo-LIV3F device can be displayed through a serial connection and 
  *          a serial terminal on a PC.
  *

  * \section GetPos_Example_Description Example Description
   Main function to show how real time GNSS data received by the Teseo-LIV3F device can
   be displayed, through a serial connection and a serial terminal, on a PC.
   The GetPos application is built on top of the FreeRTOS support introducing
   a task (consumer) to parse the messages (enqueued in a shared queue)
   coming from the Teseo-LIV3F device; and a task (listener) to parse commands coming from the serial terminal.
   Furthermore this sample application shows three advanced features supported by the Teseo-LIV3F device:
   - \c Geofencing
   - \c Odometer
   - \c DataLogging
   
   The development of the application includes contribution by APG Division.
   The Teseo-LIV3F device sends via a UART (or I2C) interface the received GNSS data to the STM32 
   microcontroller, hosted on the Nucleo board, according to the NMEA 0183 Version 4.0 protocol.
   The user can decide at compile time the interface to receive data (setting the macro configUSE_I2C in file
   gnss_app_cfg.h).
   The user can enable a specific feature at compile time (setting the corresponding macro in file gnss_app_cfg.h).
   This GetPos sample application is able to:
   - establish a serial connection between the STM32 Nucleo and X-NUCLEO-GNSS1 boards and 
        the PC
   - allow the user to select among different options for getting in a human readable format 
        information related to the acquired GNSS position, the satellites in view, the active 
        satellites, and more.
   - allow the user to upgrade the firmware of the Teseo-LIV3F device (option 0 of the main menu).

   This GetPos sample application allows also the use to run commands enabling three advanced features:
   - \c Geofencing - allows the Teseo-LIV3F receiver to raise a NMEA message when the resolved GNSS position is close to or entering or exiting from a specific circle
   - \c Odometer - provides information on the traveled distance using only the resolved GNSS position
   - \c DataLogging - allows the Teseo-LIV3F receiver to save locally on the flash the resolved GNSS position to be retrieved on demand from the Host

   After connecting the STM32 Nucleo board and the X-NUCLEO-GNSS1A1 expansion board and the 
   GPS/GLONASS antenna to the connector on the X-NUCLEO-GNSS1A1 expansion board,
   connect the STM32 Nucleo board to your PC.
   Drag and drop GetPos-*.bin (in Binary folder) on Nucleo drive.

   Run a Serial Terminal (e.g. TeraTerm) on your PC and open a serial connection using the 
   following parameters:
   - \c baud_rate: 115200
   - \c data: 8 bit
   - \c parity: none
   - \c stop: 1bit
   - \c flow_control: none
   - \c transmit_delay: 5msec/char, 5msec/line

   Reset the STM32 Nucleo board and select an option from the main menu appearing on Serial Terminal.

  * \section GetPos_HW_SW_Env Hardware and Software environment

  - This example runs on STM32 Nucleo devices with GNSS STM32 expansion board
    (X-NUCLEO-GNSS1A1)
  - This example has been tested with STMicroelectronics:
    - NUCLEO-F401RE RevC board
    - NUCLEO-L476RG RevC board
    and can be easily tailored to any other supported device and development board.
    This example runs also on the NUCLEO-F411RE RevC board, even if the chip could
    be not exploited at its best since the projects are configured for the
    NUCLEO-F401RE target board.

  * \section GetPos_Usage Usage 

 In order to make the program work, you must do the following:
 - WARNING: before opening the project with any toolchain be sure your folder
   installation path is not too in-depth since the toolchain may report errors
   after building.
 - Open IAR toolchain (this firmware has been successfully tested with
   Embedded Workbench V7.80.4).
   Alternatively you can use the Keil uVision toolchain (this firmware
   has been successfully tested with V5.24) or the System Workbench for
   STM32 (this firmware has been successfully tested with Version 2.3.1).
 - Rebuild all files and load your image into target memory.
 - Run the example.
 - Alternatively, you can download the pre-built binaries in "Binary" 
   folder included in the distributed package.
  * 
  *******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT(c) 2018 STMicroelectronics</center></h2>
  *
  * Licensed under MCD-ST Liberty SW License Agreement V2, (the "License");
  * You may not use this file except in compliance with the License.
  * You may obtain a copy of the License at:
  *
  *        www.st.com/software_license_agreement_liberty_v2
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ********************************************************************************
  */

/* Includes ------------------------------------------------------------------*/

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "cmsis_os.h"
#include "gnss_utils.h"
#include "gnss_app_cfg.h"

#if (configUSE_FEATURE == 1)
#include "gnss_feature_cfg_data.h"
#endif /* configUSE_FEATURE */

#include "gnss_fw_upgrade.h"

#include "config_bus.h"
#include "x_nucleo_gnss1a1.h"
#include "gnss_data_if.h"

/* Private defines -----------------------------------------------------------*/

#ifdef USE_STM32L0XX_NUCLEO
#define CONSUMER_STACK_SIZE 512
#else
#define CONSUMER_STACK_SIZE 1024
#endif /* USE_STM32L0XX_NUCLEO */

/* Global variables ----------------------------------------------------------*/

/* Instance of GNSS Handler */
GNSS_HandleTypeDef pGNSS;

/* Mutex for console UART access */
osMutexId consoleMutexHandle;
/* Mutex for GNSS data access */
osMutexId gnssDataMutexHandle;

/* Tasks handle */
osThreadId teseoConsumerTaskHandle;
osThreadId consoleParseTaskHandle;

/* Private variables ---------------------------------------------------------*/

static GNSSParser_Data_t GNSSParser_Data;
#ifndef USE_STM32L0XX_NUCLEO
static int gnss_feature = 0x0;
#endif /* USE_STM32L0XX_NUCLEO */

/* Global function prototypes ------------------------------------------------*/
void Error_Handler(void);

/* Private function prototypes -----------------------------------------------*/
static void GPIO_Config(void);
static void SystemClock_Config(void);

static void Console_Mutex_Init(void);

static void Teseo_Consumer_Task_Init(void);
static void TeseoConsumerTask(void const * argument);

#ifndef USE_STM32L0XX_NUCLEO
static void GNSSData_Mutex_Init(void);
static void Console_Parse_Task_Init(void);
static void ConsoleParseTask(void const * argument);

static void ConsoleRead(uint8_t *string);

static void AppCmdProcess(char *com);
static void AppCfgMsgList(int level);
#endif /* USE_STM32L0XX_NUCLEO */

#if (configUSE_FEATURE == 1)
static void AppEnFeature(char *command);
#endif /* configUSE_FEATURE */

#if (configUSE_GEOFENCE == 1)
static void AppGeofenceCfg(char *command);
#endif /* configUSE_GEOFENCE */

#if (configUSE_ODOMETER == 1)
static void AppOdometerOp(char *command);
#endif /* configUSE_ODOMETER */

#if (configUSE_DATALOG == 1)
static void AppDatalogOp(char *command);
#endif /* configUSE_DATALOG */

int main(void)
{  
  /* MCU Configuration----------------------------------------------------------*/

  /* Reset all peripherals and initialize the Flash interface and the Systick. */
  HAL_Init();
  
  /* Configure the system clock */
  SystemClock_Config();

  GPIO_Config();

  /* Single instance for the GPS driver */
#if (configUSE_I2C == 1)
  GNSS_I2C_Init();
  GNSS_Init(&pGNSS, GNSS_BUS_I2C);
#else
  GNSS_UART_Init(GNSS_UART_BAUD_RATE);
  GNSS_Init(&pGNSS, GNSS_BUS_UART);
#endif /* configUSE_I2C */

  IO_UART_Init();

  /* Create the mutex for accessing the Console UART */
  Console_Mutex_Init();
  
  /* Create the thread(s) */
  Teseo_Consumer_Task_Init();

#ifndef USE_STM32L0XX_NUCLEO
  Console_Parse_Task_Init();
#endif /* USE_STM32L0XX_NUCLEO */

  GNSS_DATA_IF_ConsoleWrite((uint8_t *)"Booting...\r\n");

  /* Start scheduler */
  osKernelStart();
  
  /* We should never get here as control is now taken by the scheduler */
  
  /* Infinite loop */
  while (1) {}
  
}

/*	
 * This function creates the Mutex for Console UART access
 */
static void Console_Mutex_Init(void)
{
  osMutexDef(mutex);
  consoleMutexHandle = osMutexCreate(osMutex(mutex));
}

/*	
 * This function creates the Mutex for GNSS Data access
 */
#ifndef USE_STM32L0XX_NUCLEO
static void GNSSData_Mutex_Init(void)
{
  osMutexDef(mutex1);
  gnssDataMutexHandle = osMutexCreate(osMutex(mutex1));
}
#endif /* USE_STM32L0XX_NUCLEO */

/*	
 * This function creates the task reading the messages coming from Teseo
 */
static void Teseo_Consumer_Task_Init(void)
{
  osThreadDef(teseoConsumerTask, TeseoConsumerTask, osPriorityNormal, 0, CONSUMER_STACK_SIZE);

  teseoConsumerTaskHandle = osThreadCreate(osThread(teseoConsumerTask), NULL);
}

/*	
 * This function creates the task reading input from the console
 */
#ifndef USE_STM32L0XX_NUCLEO
static void Console_Parse_Task_Init(void)
{
  osThreadDef(consoleParseTask, ConsoleParseTask, osPriorityNormal, 0, 1024);
  consoleParseTaskHandle = osThreadCreate(osThread(consoleParseTask), NULL);
}
#endif /* USE_STM32L0XX_NUCLEO */

/*
 * Configures GPIO
 */
static void GPIO_Config(void)
{
   GPIO_InitTypeDef GPIO_InitStruct;
  
  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  
  /* Configure GPIO pin Output Level */
  GNSS_RST_PIN_SET();
  
  /* Configure Reset */
  GPIO_InitStruct.Pin = GNSS_RST_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GNSS_RST_PORT, &GPIO_InitStruct);
#if 0
  /*Configure GPIO pin : PA5 */
  GPIO_InitStruct.Pin = GNSS_WAKEUP_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GNSS_WAKEUP_PORT, &GPIO_InitStruct);
#endif
}

/*
 * System Clock Configuration 
 */
#ifdef USE_STM32F4XX_NUCLEO
static void SystemClock_Config(void)
{
  
  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  
  __HAL_RCC_PWR_CLK_ENABLE();
  
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);
  
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  
  /**Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
    |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  
  /**Configure the Systick interrupt time 
  */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);
  
  /**Configure the Systick 
  */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);
  
  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 15, 0);
}
#endif /* ifdef USE_STM32F4XX_NUCLEO */

#ifdef USE_STM32L4XX_NUCLEO
static void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

  /* MSI is enabled after System reset, activate PLL with MSI as source */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.MSICalibrationValue = RCC_MSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 40;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLP = 7;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 15, 0);
}
#endif /* USE_STM32L4XX_NUCLEO */

#ifdef USE_STM32L0XX_NUCLEO
static void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

    /**Configure the main internal regulator output voltage 
    */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_5;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_MSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 3, 0);
}
#endif /* USE_STM32L0XX_NUCLEO */

/* TeseoConsumerTask function */
void TeseoConsumerTask(void const * argument)
{
  GNSSParser_Status_t status, check;
  const GNSS_MsgTypeDef *gnssMsg;
  
#if (configUSE_ODOMETER == 1)
  gnss_feature |= ODOMETER;
#endif /* configUSE_ODOMETER */

#if (configUSE_GEOFENCE == 1)
  gnss_feature |= GEOFENCE;
#endif /* configUSE_GEOFENCE */

#if (configUSE_DATALOG == 1)
  gnss_feature |= DATALOG;
#endif /* configUSE_DATALOG */

  GNSS_Bus_Reset(&pGNSS);
  
  /* Create the mutex for accessing the GNSS Data */
#ifndef USE_STM32L0XX_NUCLEO
  GNSSData_Mutex_Init();
#endif /* USE_STM32L0XX_NUCLEO */

  //GNSS_DATA_IF_ConsoleWrite("\n\rTeseo Consumer Task running\n\r");
  GNSS_PARSER_Init(&GNSSParser_Data);

  for(;;)
  {    
    gnssMsg = GNSS_Get_Buffer(&pGNSS);
    if(gnssMsg == NULL)
    {
      continue;
    }
    
    check = GNSS_PARSER_CheckSanity((uint8_t *)gnssMsg->buf, gnssMsg->len);

    //GNSS_DATA_IF_ConsoleWrite("got ");
    //(check == GNSS_PARSER_OK) ? GNSS_DATA_IF_ConsoleWrite("Good sentence: ") : GNSS_DATA_IF_ConsoleWrite("!!!Bad sentence: ");
    //GNSS_DATA_IF_ConsoleWrite((uint8_t *)gnssMsg->buf);
    //GNSS_DATA_IF_ConsoleWrite("\n\r");


    if(check != GNSS_PARSER_ERROR){

      for(int m = 0; m < NMEA_MSGS_NUM; m++) {

#ifndef USE_STM32L0XX_NUCLEO
        osMutexWait(gnssDataMutexHandle, osWaitForever);
        status = GNSS_PARSER_ParseMsg(&GNSSParser_Data, (eNMEAMsg)m, (uint8_t *)gnssMsg->buf);
        osMutexRelease(gnssDataMutexHandle);

        if((status != GNSS_PARSER_ERROR) && ((eNMEAMsg)m == PSTMVER)) {
          GNSS_DATA_IF_GetPSTMVerInfo(&GNSSParser_Data);
        }
#else
        status = GNSS_PARSER_ParseMsg(&GNSSParser_Data, (eNMEAMsg)m, (uint8_t *)gnssMsg->buf);
        if((status != GNSS_PARSER_ERROR) && ((eNMEAMsg)m == GPGGA)) {
            GNSS_DATA_IF_GetValidInfo(&GNSSParser_Data);
        }
#endif /* USE_STM32L0XX_NUCLEO */


#if (configUSE_GEOFENCE == 1)
        if((status != GNSS_PARSER_ERROR) && ((eNMEAMsg)m == PSTMGEOFENCE)) {
          GNSS_DATA_IF_GetGeofenceInfo(&pGNSS, &GNSSParser_Data);
        }
#endif /* configUSE_GEOFENCE */

#if (configUSE_ODOMETER == 1)
        if((status != GNSS_PARSER_ERROR) && ((eNMEAMsg)m == PSTMODO)) {
          GNSS_DATA_IF_GetOdometerInfo(&pGNSS, &GNSSParser_Data);
        }
#endif /* configUSE_ODOMETER */

#if (configUSE_DATALOG == 1)
        if((status != GNSS_PARSER_ERROR) && ((eNMEAMsg)m == PSTMDATALOG)) {
          GNSS_DATA_IF_GetDatalogInfo(&pGNSS, &GNSSParser_Data);
        }
#endif /* configUSE_DATALOG */
 
#ifndef USE_STM32L0XX_NUCLEO
        if((status != GNSS_PARSER_ERROR) && ((eNMEAMsg)m == PSTMSGL)) {
          GNSS_DATA_IF_GetMsglistAck(&pGNSS, &GNSSParser_Data);
        }

        if((status != GNSS_PARSER_ERROR) && ((eNMEAMsg)m == PSTMSAVEPAR)) {
          GNSS_DATA_IF_GetGNSSAck(&pGNSS, &GNSSParser_Data);
        }
#endif /* USE_STM32L0XX_NUCLEO */

      }
    }

    GNSS_Release_Buffer(&pGNSS, gnssMsg);

  }
}

/* ConsoleParseTask function */
#ifndef USE_STM32L0XX_NUCLEO
void ConsoleParseTask(void const * argument)
{ 
  char cmd[32] = {0};
  uint8_t ch;
  
  showCmds();
  for(;;)
  {
    ch = '\0';

    while(!GNSS_DATA_IF_ConsoleReadable())
    {
      osThreadYield();
    }
    GNSS_DATA_IF_ConsoleRead(&ch, 1, 1000);

    if((ch > 31 && ch < 126))
    {
      GNSS_DATA_IF_ConsoleWriteChar(&ch);
    }
    if(ch == '\r')
    {
      GNSS_DATA_IF_ConsoleWrite((uint8_t *)"\n\r");

      if (strlen(cmd) > 0)
      {
        cmd[strlen(cmd)] = '\0';
        AppCmdProcess(cmd);
        memset(cmd, 0, sizeof(cmd));
      }
      else
      {
        showPrompt();
      }
    }
    else if((ch > 31 && ch < 126))
    {
      cmd[strlen(cmd)] = ch;
    }
  }
}
#endif /* USE_STM32L0XX_NUCLEO */

/*	
 * This function reads a string from the console and stores it in the 'string' param
 */
#ifndef USE_STM32L0XX_NUCLEO
static void ConsoleRead(uint8_t *string)
{
  uint8_t ch;

  while(1)
  {
    ch = '\0';

    GNSS_DATA_IF_ConsoleRead(&ch, 1, 1000);

    if((ch > 31 && ch < 126))
    {
      GNSS_DATA_IF_ConsoleWriteChar(&ch);
    }
    if(ch == '\r')
    {
      GNSS_DATA_IF_ConsoleWrite((uint8_t *)"\n\r");
      string[strlen((char *)string)] = '\0';
      break;
    }
    else
    {
      if((ch > 31 && ch < 126)) {
        string[strlen((char *)string)] = ch;
      }
    }
  }
}

/* Parse the input command from console */
static void AppCmdProcess(char *com)
{    
  uint8_t tracks[MAX_STR_LEN];
  uint8_t secs[MAX_STR_LEN];
  char msg[128];
  uint8_t status;
  static int32_t tracked = 0;
  static int features_configured = 0;

  // 0 - FWUPG
  if((strcmp((char *)com, "0") == 0 || strcmp((char *)com, "fwupg") == 0)) {
    GNSS_Bus_Disable(&pGNSS);
    vTaskSuspend(teseoConsumerTaskHandle);

    GNSS_DATA_IF_ConsoleWrite((uint8_t *)"To start the FW Upgrade process disconnect and launch the FWUPG tool.\n\rType '0' to return to this prompt again.\n\r");
    FWUpgrade();

    GNSS_Bus_Enable(&pGNSS);
    vTaskResume(teseoConsumerTaskHandle);
    showPrompt();
  }

  else if(strcmp((char *)com, "y") == 0) {
    //GNSS_DATA_IF_ConsoleWrite((uint8_t *)"Saving Configuration...");
    if(!features_configured)
    {
      AppCfgMsgList(gnss_feature);
      features_configured = 1;
    }
    else
    {
      GNSS_DATA_IF_ConsoleWrite((uint8_t *)"Configuration already saved");
    }
    GNSS_DATA_IF_ConsoleWrite((uint8_t *)"\r\n>"); 
  }

	else if(strcmp((char *)com, "n") == 0) {
    GNSS_DATA_IF_ConsoleWrite((uint8_t *)"\r\n>"); 
  }
	
  // 1 - GETPOS / 2 - LASTPOS
  else if((strcmp((char *)com, "1") == 0 || strcmp((char *)com, "getpos") == 0) ||
     (strcmp((char *)com, "2") == 0 || strcmp((char *)com, "lastpos") == 0)) {
    osMutexWait(gnssDataMutexHandle, osWaitForever);
    GNSS_DATA_IF_GetValidInfo(&GNSSParser_Data);
    osMutexRelease(gnssDataMutexHandle);
  }

  // 3 - WAKEUPSTATUS
  else if(strcmp((char *)com, "3") == 0 || strcmp((char *)com, "wakestatus") == 0) {
    GNSS_Wakeup_Status(&pGNSS, &status);
    
    GNSS_DATA_IF_ConsoleWrite((uint8_t *)"WakeUp Status: "); 
    status == 0 ? GNSS_DATA_IF_ConsoleWrite((uint8_t *)"0") : GNSS_DATA_IF_ConsoleWrite((uint8_t *)"1");
    GNSS_DATA_IF_ConsoleWrite((uint8_t *)"\r\n>"); 
  }   
  
  // 4 - HELP
  else if(strcmp((char *)com, "4") == 0 || strcmp((char *)com, "help") == 0) {
    showCmds();
  }

  // 5 - DEBUG
  else if(strcmp((char *)com, "5") == 0 || strcmp((char *)com, "debug") == 0) {
    GNSSParser_Data.debug = (GNSSParser_Data.debug == DEBUG_ON ? DEBUG_OFF : DEBUG_ON);
    if(GNSSParser_Data.debug == DEBUG_OFF)
      GNSS_DATA_IF_ConsoleWrite((uint8_t *)"Debug: OFF\r\n>");
    else
      GNSS_DATA_IF_ConsoleWrite((uint8_t *)"Debug: ON\r\n>");
  }

  // 6 - TRACKPOS
  else if(strcmp((char *)com, "6") == 0 || strcmp((char *)com, "track") == 0) {
    uint32_t t, s;
    do {
      memset(tracks, 0, 16);
      sprintf(msg, "How many positions do you want to track? (max allowed %d)\r\n>", (int)MAX_STOR_POS);
      GNSS_DATA_IF_ConsoleWrite((uint8_t *)msg);                
      ConsoleRead((uint8_t *)tracks);
    } while(atoi((char *)tracks) < 0 || atoi((char *)tracks) > MAX_STOR_POS);
    do {
      memset(secs, 0, 16);
      GNSS_DATA_IF_ConsoleWrite((uint8_t *)"How many seconds do you want to delay while tracking? (>= 0)\r\n> ");
      ConsoleRead((uint8_t *)secs);
    } while(atoi((char *)secs) < 0);
    t = strtoul((char *)tracks, NULL, 10);
    s = strtoul((char *)secs, NULL, 10);

    tracked = GNSS_DATA_IF_TrackGotPos(&GNSSParser_Data, t, s);
    if(tracked > 0){
      GNSS_DATA_IF_ConsoleWrite((uint8_t *)"Last tracking process went good.\r\n\n>");  
    }
    else
      GNSS_DATA_IF_ConsoleWrite((uint8_t *)"Last tracking process went bad.\r\n\n>");        
  }

  // 7 - LASTTRACK
  else if(strcmp((char *)com, "7") == 0 || strcmp((char *)com, "lasttrack") == 0) {
    if(tracked > 0){
      GNSS_DATA_IF_ConsoleWrite((uint8_t *)"Acquired positions:\r\n");
      GNSS_DATA_IF_PrintTrackedPositions(tracked);
    }
    else
      GNSS_DATA_IF_ConsoleWrite((uint8_t *)"Last tracking process went bad.\r\n\n>");
  }
  
  // 8 - GETFWVER
  else if(strcmp((char *)com, "8") == 0 || strcmp((char *)com, "getfwver") == 0) {
    memset(com, 0, MAX_STR_LEN);      
    GNSS_DATA_IF_ConsoleWrite((uint8_t *)"Type \"$PSTMGETSWVER\"   to get the GNSSLIB version \r\n");
    GNSS_DATA_IF_ConsoleWrite((uint8_t *)"Type \"$PSTMGETSWVER,1\" to get the OS20LIB version \r\n");
    GNSS_DATA_IF_ConsoleWrite((uint8_t *)"Type \"$PSTMGETSWVER,2\" to get the GPSAPP version \r\n");
    GNSS_DATA_IF_ConsoleWrite((uint8_t *)"Type \"$PSTMGETSWVER,4\" to get the WAASLIB version \r\n");
    GNSS_DATA_IF_ConsoleWrite((uint8_t *)"Type \"$PSTMGETSWVER,6\" to get the BINIMG version \r\n");
    GNSS_DATA_IF_ConsoleWrite((uint8_t *)"Type \"$PSTMGETSWVER,7\" to get the board version \r\n");
    GNSS_DATA_IF_ConsoleWrite((uint8_t *)"Type \"$PSTMGETSWVER,8\" to get the STAGPSLIB version \r\n");
    GNSS_DATA_IF_ConsoleWrite((uint8_t *)"\nType the command now:\r\n> ");
  }
  
  // GETFWVER,x
  else if(strncmp((char *)com, "$PSTMGETSWVER", strlen("$PSTMGETSWVER")) == 0) {
    GNSS_DATA_IF_SendCommand(&pGNSS, com);
  }

  // 9 - GET Fix data for single or combined Satellite navigation system
  else if(strcmp((char *)com, "9") == 0 || strcmp((char *)com, "getgnsmsg") == 0) {
    osMutexWait(gnssDataMutexHandle, osWaitForever);
    GNSS_DATA_IF_GetGNSInfo(&GNSSParser_Data);
    osMutexRelease(gnssDataMutexHandle);
  }

  // 10 - GET GPS Pseudorange Noise Statistics
  else if(strcmp((char *)com, "10") == 0 || strcmp((char *)com, "getgpgst") == 0) {
    osMutexWait(gnssDataMutexHandle, osWaitForever);
    GNSS_DATA_IF_GetGPGSTInfo(&GNSSParser_Data);
    osMutexRelease(gnssDataMutexHandle);
  }

  // 11 - GET Recommended Minimum Specific GPS/Transit data
  else if(strcmp((char *)com, "11") == 0 || strcmp((char *)com, "getgprmc") == 0) {
    osMutexWait(gnssDataMutexHandle, osWaitForever);
    GNSS_DATA_IF_GetGPRMCInfo(&GNSSParser_Data);
    osMutexRelease(gnssDataMutexHandle);
  }

  // 12 - GET GPS DOP and Active Satellites
  else if(strcmp((char *)com, "12") == 0 || strcmp((char *)com, "getgsamsg") == 0) {
    osMutexWait(gnssDataMutexHandle, osWaitForever);
    GNSS_DATA_IF_GetGSAInfo(&GNSSParser_Data);
    osMutexRelease(gnssDataMutexHandle);
  }
  
  // 13 - GET GPS Satellites in View
  else if(strcmp((char *)com, "13") == 0 || strcmp((char *)com, "getgsvmsg") == 0) {
    osMutexWait(gnssDataMutexHandle, osWaitForever);
    GNSS_DATA_IF_GetGSVInfo(&GNSSParser_Data);
    osMutexRelease(gnssDataMutexHandle);
  }

#if (configUSE_FEATURE == 1)
  // 14 - EN-FEATURE
  else if(strcmp((char *)com, "14") == 0 || strcmp((char *)com, "en-feature") == 0) {
#if (configUSE_GEOFENCE == 1)
    GNSS_DATA_IF_ConsoleWrite((uint8_t *)"Type \"GEOFENCE,1\" to enable geofence\r\n");
    GNSS_DATA_IF_ConsoleWrite((uint8_t *)"Type \"GEOFENCE,0\" to disable geofence\r\n");
#endif /* configUSE_GEOFENCE */
 
#if (configUSE_ODOMETER == 1)
    GNSS_DATA_IF_ConsoleWrite((uint8_t *)"Type \"ODO,1\" to enable odometer\r\n");
    GNSS_DATA_IF_ConsoleWrite((uint8_t *)"Type \"ODO,0\" to disable odometer\r\n");
#endif /* configUSE_ODOMETER */

#if (configUSE_DATALOG == 1)
    GNSS_DATA_IF_ConsoleWrite((uint8_t *)"Type \"DATALOG,1\" to enable datalog\r\n");
    GNSS_DATA_IF_ConsoleWrite((uint8_t *)"Type \"DATALOG,0\" to disable datalog\r\n");
#endif /* configUSE_DATALOG */

    GNSS_DATA_IF_ConsoleWrite((uint8_t *)"\nType the command now:\r\n> ");
  }
#endif /* configUSE_FEATURE */
  
#if (configUSE_FEATURE == 1)
  else if(strncmp((char *)com, "GEOFENCE,1", strlen("GEOFENCE,1")) == 0 ||
          strncmp((char *)com, "GEOFENCE,0", strlen("GEOFENCE,0")) == 0 ||
          strncmp((char *)com, "ODO,1", strlen("ODO,1")) == 0 ||
          strncmp((char *)com, "ODO,0", strlen("ODO,0")) == 0 ||
          strncmp((char *)com, "DATALOG,1", strlen("DATALOG,1")) == 0 ||
          strncmp((char *)com, "DATALOG,0", strlen("DATALOG,0")) == 0) {
    AppEnFeature(com);
  }
#endif /* configUSE_FEATURE */

#if (configUSE_GEOFENCE == 1)
  // 15 - CONF-GEOFENCE
  else if(strcmp((char *)com, "15") == 0 || strcmp((char *)com, "conf-geofence") == 0) {
    memset(com, 0, MAX_STR_LEN);
    GNSS_DATA_IF_ConsoleWrite((uint8_t *)"Type \"Geofence-Lecce\" to config circle in Lecce \r\n");
    GNSS_DATA_IF_ConsoleWrite((uint8_t *)"Type \"Geofence-Catania\" to config circle in Catania \r\n");
    GNSS_DATA_IF_ConsoleWrite((uint8_t *)"Type the command:\r\n> ");
  }
  // GEOFENCE-CIRCLE
  else if(strncmp((char *)com, "Geofence-Lecce", strlen("Geofence-Lecce")) == 0 ||
          strncmp((char *)com, "Geofence-Catania", strlen("Geofence-Catania")) == 0) {
    AppGeofenceCfg(com);
  }
  
  // 16 - REQ-GEOFENCE
  else if(strcmp((char *)com, "16") == 0 || strcmp((char *)com, "req-geofence") == 0) {
    GNSS_DATA_IF_SendCommand(&pGNSS, "$PSTMGEOFENCEREQ");
  }
#endif /* configUSE_GEOFENCE */

#if (configUSE_ODOMETER == 1)
  // 17 - ODOMETER (START/STOP)
  else if(strcmp((char *)com, "17") == 0 || strcmp((char *)com, "odometer-op") == 0) {
    memset(com, 0, MAX_STR_LEN);
    GNSS_DATA_IF_ConsoleWrite((uint8_t *)"Type \"START-ODO\" to start odometer\r\n");
    GNSS_DATA_IF_ConsoleWrite((uint8_t *)"Type \"STOP-ODO\"  to stop odometer\r\n");
    GNSS_DATA_IF_ConsoleWrite((uint8_t *)"Type the command:\r\n> ");
  }

  // Odometer op
  else if(strncmp((char *)com, "START-ODO", strlen("START-ODO")) == 0 ||
          strncmp((char *)com, "STOP-ODO", strlen("STOP-ODO")) == 0) {
    AppOdometerOp(com);
  }
#endif /* configUSE_ODOMETER */

#if (configUSE_DATALOG == 1)  
  // 18 - DATALOG (START/STOP/ERASE)
  else if(strcmp((char *)com, "18") == 0 || strcmp((char *)com, "datalog-op") == 0) {
    memset(com, 0, MAX_STR_LEN);
    GNSS_DATA_IF_ConsoleWrite((uint8_t *)"Type \"CONFIG-DATALOG\" to config datalog\r\n");
    GNSS_DATA_IF_ConsoleWrite((uint8_t *)"Type \"START-DATALOG\" to start datalog\r\n");
    GNSS_DATA_IF_ConsoleWrite((uint8_t *)"Type \"STOP-DATALOG\"  to stop datalog\r\n");
    GNSS_DATA_IF_ConsoleWrite((uint8_t *)"Type \"ERASE-DATALOG\"  to erase datalog\r\n");
    GNSS_DATA_IF_ConsoleWrite((uint8_t *)"Type the command:\r\n> ");
  }
  // Datalog op
  else if(strncmp((char *)com, "CONFIG-DATALOG", strlen("CONFIG-DATALOG")) == 0 ||
          strncmp((char *)com, "START-DATALOG", strlen("START-DATALOG")) == 0 ||
          strncmp((char *)com, "STOP-DATALOG", strlen("STOP-DATALOG")) == 0 ||
          strncmp((char *)com, "ERASE-DATALOG", strlen("ERASE-DATALOG")) == 0) {
    AppDatalogOp(com);
  }
#endif /* configUSE_DATALOG */

  // 19 - EXT-HELP
  else if(strcmp((char *)com, "19") == 0 || strcmp((char *)com, "ext-help") == 0) {
    printHelp();
  }
  
  else {
    GNSS_DATA_IF_ConsoleWrite((uint8_t *)"Command not valid.\r\n\n>");
  }
}

/* CfgMessageList */
static void AppCfgMsgList(int level)
{
  GNSS_DATA_IF_CfgMessageList(&pGNSS, level);
}
#endif /* USE_STM32L0XX_NUCLEO */

/* Enable feature */
#if (configUSE_FEATURE == 1)
static void AppEnFeature(char *command)
{
  if(strcmp(command, "GEOFENCE,1") == 0) {
    GNSS_DATA_IF_EnableGeofence(&pGNSS, 1);
  }
  if(strcmp(command, "GEOFENCE,0") == 0) {
    GNSS_DATA_IF_EnableGeofence(&pGNSS, 0);
  }
  if(strcmp(command, "ODO,1") == 0) {
    GNSS_DATA_IF_EnableOdo(&pGNSS, 1);
  }
  if(strcmp(command, "ODO,0") == 0) {
    GNSS_DATA_IF_EnableOdo(&pGNSS, 0);
  }
  if(strcmp(command, "DATALOG,1") == 0) {
    GNSS_DATA_IF_EnableDatalog(&pGNSS, 1);
  }
  if(strcmp(command, "DATALOG,0") == 0) {
    GNSS_DATA_IF_EnableDatalog(&pGNSS, 0);
  }
}
#endif /* configUSE_FEATURE */

/* Geofence configure */
#if (configUSE_GEOFENCE == 1)
static void AppGeofenceCfg(char *command)
{ 
  if(strcmp(command, "Geofence-Lecce") == 0) {
    GNSS_DATA_IF_ConfigGeofence(&pGNSS, &Geofence_STLecce);
  }
  if(strcmp(command, "Geofence-Catania") == 0) {
    GNSS_DATA_IF_ConfigGeofence(&pGNSS, &Geofence_Catania);
  }
}
#endif /* configUSE_GEOFENCE */

/* Odometer configure */
#if (configUSE_ODOMETER == 1)
static void AppOdometerOp(char *command)
{ 
  if(strcmp(command, "START-ODO") == 0) {
    GNSS_DATA_IF_StartOdo(&pGNSS, 1);
  }
  if(strcmp((char *)command, "STOP-ODO") == 0) {
    GNSS_DATA_IF_StopOdo(&pGNSS);
  }
}
#endif /* configUSE_ODOMETER */

/* Datalog configure */
#if (configUSE_DATALOG == 1)
static void AppDatalogOp(char *command)
{
  if(strcmp(command, "CONFIG-DATALOG") == 0) {
    GNSS_DATA_IF_ConfigDatalog(&pGNSS, &SampleDatalog);
  }
  if(strcmp(command, "START-DATALOG") == 0) {
    GNSS_DATA_IF_StartDatalog(&pGNSS);
  }
  if(strcmp(command, "STOP-DATALOG") == 0) {
    GNSS_DATA_IF_StopDatalog(&pGNSS);
  }
  if(strcmp(command, "ERASE-DATALOG") == 0) {
    GNSS_DATA_IF_EraseDatalog(&pGNSS);
  }
}
#endif /* configUSE_DATALOG */

/*
 * This function is executed in case of error occurrence.
 */
void Error_Handler(void)
{
  /* User can add his own implementation to report the HAL error return state */
  while(1) 
  {
  }
}

#ifdef USE_FULL_ASSERT

/*
 * Reports the name of the source file and the source line number
 * where the assert_param error has occurred.
 */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* User can add his own implementation to report the file name and line number,
  ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
}

#endif 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
