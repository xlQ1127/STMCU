/**
  *******************************************************************************
  * @file    Applications/Virtual_COM_Port/Src/main.c
  * @author  AST/CL
  * @version V2.0.0
  * @date    Apr-2018
  * @brief   Application to be loaded on an STM32 Nucleo board in order to 
  *          use the Teseo-Suite tool available on st.com
  *

  * \section Virtual_COM_Port_Example_Description Example Description
   Virtual_COM_Port is the application to be loaded in order to use the Teseo-Suite
   tool also for the GNSS Teseo-LIV3 mounted on the X-NUCLEO-GNSS1A1 expansion boards.
   The Virtual_COM_Port application is built on top of the FreeRTOS support introducing
   a task (consumer) to parse the messages (enqueued in a shared queue)
   coming from the Teseo-LIV3F device; and a task (listener) to parse commands coming from the Teseo-Suite.

   Plug the X-NUCLEO-GNSS1A1 expansion board on top of a STM32 Nucleo-F401RE.
   Connect the STM32 Nucleo board to your PC.
   Drag and drop Virtual_COM_Port-*.bin (in Binary folder) on Nucleo drive.
   Run the Teseo-Suite tool (available for the download on st.com) on your Win PC, select a Serial Port 
   and open a serial connection through the Teseo-Suite GUI using the following parameters:
   - \c baud_rate: 115200
   - \c data: 8 bit
   - \c parity: none
   - \c stop: 1bit
   - \c flow_control: none

   and enjoy.    
   For all info on how to use the Teseo-Suite, please, refer to the related documentation
   available on st.com

  * \section Virtual_COM_Port_HW_SW_Env Hardware and Software environment

  - This example runs on STM32 Nucleo devices with GNSS STM32 expansion board
    (X-NUCLEO-GNSS1A1)
  - This example has been tested with STMicroelectronics:
    - NUCLEO-F401RE RevC board
    - NUCLEO-L476RG RevC board
    - NUCLEO-L073RZ RevC board
    and can be easily tailored to any other supported device and development board.
    This example runs also on the NUCLEO-F411RE RevC board, even if the chip could
    be not exploited at its best since the projects are configured for the
    NUCLEO-F401RE target board.

  * \section Virtual_COM_Port_Usage Usage 

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

#ifdef USE_STM32F4XX_NUCLEO
#include "stm32f4xx_nucleo.h"
#endif /* USE_STM32F4XX_NUCLEO */

#ifdef USE_STM32L4XX_NUCLEO
#include "stm32l4xx_nucleo.h"
#endif /* USE_STM32L4XX_NUCLEO */

#ifdef USE_STM32L0XX_NUCLEO
#include "stm32l0xx_nucleo.h"
#endif /* USE_STM32L0XX_NUCLEO */

#include "cmsis_os.h"
#include "gnss_app_cfg.h"

#include "config_bus.h"
#include "x_nucleo_gnss1a1.h"
#include "gnss_data_if.h"

/* Private defines -----------------------------------------------------------*/
#ifdef USE_STM32L0XX_NUCLEO
#define CONSOLE_STACK_SIZE 128
#else
#define CONSOLE_STACK_SIZE 512
#endif /* USE_STM32L0XX_NUCLEO */

/* Global variables ----------------------------------------------------------*/

/* Instance of GNSS Handler */
GNSS_HandleTypeDef pGNSS;

/* Mutex for console UART access */
osMutexId consoleMutexHandle;

/* Tasks handle */
osThreadId teseoConsumerTaskHandle;
osThreadId consoleParseTaskHandle;

/* Private variables ---------------------------------------------------------*/

/* User Button flag */
static int btnRst = 0;

/* Global function prototypes ------------------------------------------------*/

void Error_Handler(void);

/* Private function prototypes -----------------------------------------------*/

static void GPIO_Config(void);
static void SystemClock_Config(void);

static void Console_Mutex_Init(void);
static void Teseo_Consumer_Task_Init(void);
static void Console_Parse_Task_Init(void);

static void TeseoConsumerTask(void const * argument);
static void ConsoleParseTask(void const * argument);

int main(void)
{  
  /* MCU Configuration----------------------------------------------------------*/
  
  /* Reset all peripherals and initialize the Flash interface and the Systick. */
  HAL_Init();
  
  BSP_PB_Init(BUTTON_KEY, BUTTON_MODE_EXTI);  
  
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
  Console_Parse_Task_Init();

  GNSS_DATA_IF_ConsoleWrite((uint8_t *)"Booting...\r\n");

  /* Start scheduler */
  osKernelStart();
  
  /* We should never get here as control is now taken by the scheduler */
  
  /* Infinite loop */
  while (1) {}
  
}

/* This function creates the Mutex for Console UART access */
static void Console_Mutex_Init(void)
{
  osMutexDef(mutex);
  consoleMutexHandle = osMutexCreate(osMutex(mutex));
}

/* This function creates the task reading the messages coming from Teseo */
static void Teseo_Consumer_Task_Init(void)
{
  osThreadDef(teseoConsumerTask, TeseoConsumerTask, osPriorityNormal, 0, 128);
  teseoConsumerTaskHandle = osThreadCreate(osThread(teseoConsumerTask), NULL);
}

/* This function creates the task reading input from the cocsole */
static void Console_Parse_Task_Init(void)
{
  osThreadDef(consoleParseTask, ConsoleParseTask, osPriorityNormal, 0, CONSOLE_STACK_SIZE);
  consoleParseTaskHandle = osThreadCreate(osThread(consoleParseTask), NULL);
}

/* Configures GPIO */
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

/* System Clock Configuration */
#ifdef USE_STM32F4XX_NUCLEO
void SystemClock_Config(void)
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
void SystemClock_Config(void)
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
void SystemClock_Config(void)
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
  //GNSSParser_Status_t check;
  const GNSS_MsgTypeDef *gnssMsg;

  GNSS_Bus_Reset(&pGNSS);

  //GNSS_DATA_IF_ConsoleWrite("\n\rTeseo Consumer Task running\n\r");

  for(;;)
  {
    if(btnRst == 1)
    {
      GNSS_Bus_Reset(&pGNSS);
      btnRst = 0;
    }

    gnssMsg = GNSS_Get_Buffer(&pGNSS);
    if(gnssMsg == NULL)
    {
      continue;
    }

    //check = GNSS_PARSER_CheckSanity(gnssMsg->buf, gnssMsg->len);

    //GNSS_DATA_IF_ConsoleWrite("got ");
    //(check == GNSS_PARSER_OK) ? GNSS_DATA_IF_ConsoleWrite("Good sentence: ") : GNSS_DATA_IF_ConsoleWrite("!!!Bad sentence: ");
    GNSS_DATA_IF_ConsoleWrite((uint8_t *)gnssMsg->buf);
    //GNSS_DATA_IF_ConsoleWrite("\n\r");

    GNSS_Release_Buffer(&pGNSS, gnssMsg);

  }
}

/* Serial Port Parse Task function */
void ConsoleParseTask(void const * argument)
{
  GNSS_StatusTypeDef status;
  uint8_t buffer[128];
  uint16_t rxLen;

  for(;;)
  {
    while(!GNSS_DATA_IF_ConsoleReadable()) {
      osThreadYield();
    }
    memset(buffer, 0, sizeof(buffer));
    GNSS_DATA_IF_ConsoleRead(buffer, sizeof(buffer), 100);
    rxLen = strlen((char *)buffer);

    status = GNSS_Bus_Write(&pGNSS, buffer, rxLen, 5000);

    if (status != GNSS_OK) {
      GNSS_DATA_IF_ConsoleWrite((uint8_t *)"Error sending command\n\n");
    }
  }  
}

/* User Button detection callback. */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  btnRst = 1;
}

/* This function is executed in case of error occurrence. */
void Error_Handler(void)
{
  /* User can add his own implementation to report the HAL error return state */
  while(1) 
  {
  }
}

#ifdef USE_FULL_ASSERT

/* Reports the name of the source file and the source line number
* where the assert_param error has occurred.
*/
void assert_failed(uint8_t* file, uint32_t line)
{
  /* User can add his own implementation to report the file name and line number,
  ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
}

#endif

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

