/**
  *******************************************************************************
  * @file    Applications/SimOSGetPos/Src/main.c
  * @author  AST/CL
  * @version V2.0.0
  * @date    Apr-2018
  * @brief   This application shows how real time GNSS data received by the GNSS
  *          Teseo-LIV3F device can be displayed through a serial connection and 
  *          a serial terminal on a PC.
  *

  * \section SimOSGetPos_Example_Description Example Description
   Main function to show how real time GNSS data ($GPGGA) received by the Teseo-LIV3F device can
   be displayed, through a serial connection and a serial terminal, on a PC.
   This application is tailored for STM32 Nucleo L0 family.

   The Teseo-LIV3F device sends via a UART interface the received GNSS data to the STM32 
   microcontroller, hosted on the Nucleo board, according to the NMEA 0183 Version 4.0 protocol.

   This SimOSGetPos sample application is able to:
   - establish a serial connection between the STM32 Nucleo and X-NUCLEO-GNSS1 boards and 
        the PC
   - parse periodic $GPGGA sentences.

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

   Reset the STM32 Nucleo board and select an option from the main menu appearing on Serial Terminal.

  * \section SimOSGetPos_HW_SW_Env Hardware and Software environment

  - This example runs on STM32 Nucleo devices with GNSS STM32 expansion board
    (X-NUCLEO-GNSS1A1)
  - This example has been tested with STMicroelectronics:
    - NUCLEO-L073RZ RevC board
    and can be easily tailored to any other supported device and development board.
    This example runs also on the NUCLEO-F411RE RevC board, even if the chip could
    be not exploited at its best since the projects are configured for the
    NUCLEO-F401RE target board.

  * \section SimOSGetPos_Usage Usage 

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

#include "gnss_app_cfg.h"

#if (configUSE_FEATURE == 1)
#include "gnss_feature_cfg_data.h"
#endif /* configUSE_FEATURE */

#include "gnss_fw_upgrade.h"

#include "config_bus.h"
#include "x_nucleo_gnss1a1.h"
#include "gnss_data_if.h"


/* Private defines -----------------------------------------------------------*/

/* Global variables ----------------------------------------------------------*/

/* Instance of GNSS Handler */
GNSS_HandleTypeDef pGNSS;

/* Private variables ---------------------------------------------------------*/

static GNSSParser_Data_t GNSSParser_Data;

/* Private function prototypes -----------------------------------------------*/
static void GPIO_Config(void);
static void SystemClock_Config(void);
static void TeseoConsumerTask(void const * argument);

/* Global function prototypes  -----------------------------------------------*/
void Error_Handler(void);

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

  GNSS_DATA_IF_ConsoleWrite((uint8_t *)"Booting...\r\n");
 
  /* Infinite loop */
  while (1)
  {
    TeseoConsumerTask(NULL);
  }
  
}

/* Config GPIO */
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

/* Config System Clock */
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

        status = GNSS_PARSER_ParseMsg(&GNSSParser_Data, (eNMEAMsg)m, (uint8_t *)gnssMsg->buf);

        if((status != GNSS_PARSER_ERROR) && ((eNMEAMsg)m == GPGGA)) {
            GNSS_DATA_IF_GetValidInfo(&GNSSParser_Data);
        }
        if((status != GNSS_PARSER_ERROR) && ((eNMEAMsg)m == PSTMGEOFENCE)) {
          GNSS_DATA_IF_GetGeofenceInfo(&pGNSS, &GNSSParser_Data);
        }
      }
    }

    GNSS_Release_Buffer(&pGNSS, gnssMsg);

  }
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
