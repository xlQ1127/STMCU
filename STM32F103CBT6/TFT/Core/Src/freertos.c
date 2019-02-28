/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * Copyright (c) 2018 STMicroelectronics International N.V. 
  * All rights reserved.
  *
  * Redistribution and use in source and binary forms, with or without 
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice, 
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other 
  *    contributors to this software may be used to endorse or promote products 
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this 
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under 
  *    this license is void and will automatically terminate your rights under 
  *    this license. 
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS" 
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT 
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT 
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, 
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "gpio.h"
#include "rtc.h"
#include "tim.h"
#include "usart.h"
#include "crc.h"

#include "gui.h"
#include "DIALOG.h"

#include "BSP_Key.h"
#include "BSP_LED.h"

#include "BSP_SHT20.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
uint8_t RTCAlarm_Flag = 0;
uint16_t LED_Speed = 100;
uint8_t WKUP_Flag = 0;

RTC_TimeTypeDef SetTime;
RTC_AlarmTypeDef SetAlarm;
/* USER CODE END Variables */
osThreadId defaultTaskHandle;
osThreadId GUITaskHandle;
osThreadId KeyTaskHandle;
osThreadId AlarmTaskHandle;
osThreadId StopTaskHandle;
osMessageQId KeyQueueHandle;
osMessageQId StopQueueHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
void MX_NVIC_Init(void);
extern void SystemClock_Config(void);
extern WM_HWIN CreateDesktop(void);
void HSE_Init(void);
void PreStopMode(void);
void AftStopMode(void);
/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void const *argument);
void StartGUITask(void const *argument);
void StartKeyTask(void const *argument);
void StartAlarmTask(void const *argument);
void StartStopTask(void const *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* Pre/Post sleep processing prototypes */
void PreSleepProcessing(uint32_t *ulExpectedIdleTime);
void PostSleepProcessing(uint32_t *ulExpectedIdleTime);

/* USER CODE BEGIN PREPOSTSLEEP */
__weak void PreSleepProcessing(uint32_t *ulExpectedIdleTime)
{

  HAL_PWR_EnterSLEEPMode(PWR_LOWPOWERREGULATOR_ON, PWR_SLEEPENTRY_WFI); //PWR_LOWPOWERREGULATOR_ON  PWR_MAINREGULATOR_ON
  /* place for user code */
}

__weak void PostSleepProcessing(uint32_t *ulExpectedIdleTime)
{

  /* place for user code */
}
/* USER CODE END PREPOSTSLEEP */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void)
{
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  //	 GUI_X_InitOS();
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityLow, 0, 64);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* definition and creation of GUITask */
  osThreadDef(GUITask, StartGUITask, osPriorityBelowNormal, 0, 512);
  GUITaskHandle = osThreadCreate(osThread(GUITask), NULL);

  /* definition and creation of KeyTask */
  osThreadDef(KeyTask, StartKeyTask, osPriorityRealtime, 0, 128);
  KeyTaskHandle = osThreadCreate(osThread(KeyTask), NULL);

  /* definition and creation of AlarmTask */
  osThreadDef(AlarmTask, StartAlarmTask, osPriorityLow, 0, 64);
  AlarmTaskHandle = osThreadCreate(osThread(AlarmTask), NULL);

  /* definition and creation of StopTask */
  osThreadDef(StopTask, StartStopTask, osPriorityLow, 0, 128);
  StopTaskHandle = osThreadCreate(osThread(StopTask), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* Create the queue(s) */
  /* definition and creation of KeyQueue */
  osMessageQDef(KeyQueue, 10, Key_Statu_Typedef);
  KeyQueueHandle = osMessageCreate(osMessageQ(KeyQueue), NULL);

  /* definition and creation of StopQueue */
  osMessageQDef(StopQueue, 1, uint8_t);
  StopQueueHandle = osMessageCreate(osMessageQ(StopQueue), NULL);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */
}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used 
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void const *argument)
{

  /* USER CODE BEGIN StartDefaultTask */

  /* Infinite loop */
  for (;;)
  {

    LED2_ON;
    osDelay(LED_Speed);
    LED2_OFF;

    LED3_ON;
    osDelay(LED_Speed);
    LED3_OFF;

    LED4_ON;
    osDelay(LED_Speed);
    LED4_OFF;

    LED5_ON;
    osDelay(LED_Speed);
    LED5_OFF;

    LED4_ON;
    osDelay(LED_Speed);
    LED4_OFF;

    LED3_ON;
    osDelay(LED_Speed);
    LED3_OFF;
  }
  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_StartGUITask */
/**
* @brief Function implementing the GUITask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartGUITask */
void StartGUITask(void const *argument)
{
  /* USER CODE BEGIN StartGUITask */
  RTC_TimeTypeDef Time;
  char strTime[8];

  osEvent KeyEvent;

  GUI_Init();
  GUI_SetBkColor(GUI_BLACK);
  GUI_SetColor(GUI_GREEN);
  GUI_Clear();
  GUI_SetFont(GUI_FONT_13_ASCII);
  //	 GUI_SetFont(GUI_FONT_32_ASCII);
  //  GUI_DispStringHCenterAt( GUI_GetVersionString(),80,64 );
  //	 GUI_SetFont(GUI_FONT_13_ASCII);
  //	 GUI_DispDecAt( 0,0,0,1 );
  //	 osThreadResume(&KeyTaskHandle);
  //	 CreateMenu();

  CreateDesktop();

  //	 osThreadResume(KeyTaskHandle);
  /* Infinite loop */
  for (;;)
  {

    HAL_RTC_GetTime(&hrtc, &Time, RTC_FORMAT_BIN);

    sprintf(strTime, "%02d:%02d:%02d", Time.Hours, Time.Minutes, Time.Seconds);
    GUI_DispStringAt(strTime, 0, 0);
    sprintf(strTime, "%02.1fC %02.1f%%", Get_SHT20_TEMP(), Get_SHT20_HUM());
    GUI_DispStringAt(strTime, 85, 0);

    KeyEvent = osMessageGet(KeyQueueHandle, osWaitForever);

    if (((Key_Statu_Typedef *)KeyEvent.value.p)->Key_Special_Down_Flag == 1)
    {
      GUI_SendKeyMsg(GUI_KEY_DOWN, 1);
    }
    else
    {
      if (((Key_Statu_Typedef *)KeyEvent.value.p)->Key_Down_Flag == 1)
      {
        GUI_SendKeyMsg(GUI_KEY_TAB, 1);
      }
    }

    if (((Key_Statu_Typedef *)KeyEvent.value.p)->Key_Special_Up_Flag == 1)
    {
      GUI_SendKeyMsg(GUI_KEY_UP, 1);
    }
    else
    {
      if (((Key_Statu_Typedef *)KeyEvent.value.p)->Key_Up_Flag == 1)
      {
        GUI_SendKeyMsg(GUI_KEY_BACKTAB, 1);
      }
    }

    if (((Key_Statu_Typedef *)KeyEvent.value.p)->Key_Right_Flag == 1)
    {
      GUI_SendKeyMsg(GUI_KEY_ENTER, 1);
    }

    if (((Key_Statu_Typedef *)KeyEvent.value.p)->Key_Left_Flag == 1)
    {
      GUI_SendKeyMsg(GUI_KEY_BACKSPACE, 1);
    }

    GUI_Exec();
    osDelay(20);
  }
  /* USER CODE END StartGUITask */
}

/* USER CODE BEGIN Header_StartKeyTask */
/**
* @brief Function implementing the KeyTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartKeyTask */
void StartKeyTask(void const *argument)
{
  /* USER CODE BEGIN StartKeyTask */
  //	 RTC_TimeTypeDef Time;
  //	 RTC_AlarmTypeDef ATime;

  Key_Statu_Typedef Key;

  Key_init(&Key);
  //	 osThreadSuspend(NULL);
  /* Infinite loop */
  for (;;)
  {
    Scan_Key(&Key);
    if (Key.Count > 80)
    {
      if (Key.Key_Special_Flag)
      {
        Key.Key_Special_Flag = 2;
      }
      else if (Key.Key_Left_Flag)
      {
        Key.Key_Left_Flag = 2;
      }

      else if (Key.Key_Right_Flag)
      {
        Key.Key_Right_Flag = 2;
      }

      else if (Key.Key_Down_Flag)
      {
        Key.Key_Down_Flag = 2;
      }

      else if (Key.Key_Up_Flag)
      {
        Key.Key_Up_Flag = 2;
      }

      else if (Key.Key_WKUP_Flag)
      {
        Key.Key_WKUP_Flag = 2;
        osMessagePut(StopQueueHandle, (uint32_t)Key.Key_WKUP_Flag, 10);
      }
    }

    osMessagePut(KeyQueueHandle, (uint32_t)&Key, 0);
    //   GUI_DispDecAt(  Key.Key_Down_Flag  ,0,0,1);
    //			GUI_DispDecAt(  Key.Key_Left_Flag  ,10,0,1);
    osDelay(10);
  }
  /* USER CODE END StartKeyTask */
}

/* USER CODE BEGIN Header_StartAlarmTask */
/**
* @brief Function implementing the AlarmTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartAlarmTask */
void StartAlarmTask(void const *argument)
{
  /* USER CODE BEGIN StartAlarmTask */

  /* Infinite loop */
  for (;;)
  {
    if (RTCAlarm_Flag == 1)
    {
      LL_GPIO_ResetOutputPin(Vib_Moto_GPIO_Port, Vib_Moto_Pin);
      osDelay(500);
      LL_GPIO_SetOutputPin(Vib_Moto_GPIO_Port, Vib_Moto_Pin);
      osDelay(500);
      LL_GPIO_ResetOutputPin(Vib_Moto_GPIO_Port, Vib_Moto_Pin);
    }
    else
    {
      osDelay(1000);
    }
  }
  /* USER CODE END StartAlarmTask */
}

/* USER CODE BEGIN Header_StartStopTask */
/**
* @brief Function implementing the StopTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartStopTask */
void StartStopTask(void const *argument)
{
  /* USER CODE BEGIN StartStopTask */

  //	 osEvent StopEvent;
  /* Infinite loop */
  for (;;)
  {

    //			 StopEvent=osMessageGet(StopQueueHandle,1);

    if (WKUP_Flag == 1)
    {

      //					 HAL_PWR_EnableWakeUpPin(PWR_WAKEUP_PIN1);
      //					 HAL_PWR_EnterSTANDBYMode();
      //				  PreStopMode();
      HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_2);
      HAL_PWR_EnterSTOPMode(PWR_MAINREGULATOR_ON, PWR_SLEEPENTRY_WFI);
      //					 HSE_Init();
      //					 HAL_Init();
      //					 SystemClock_Config();
      HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
      //					 AftStopMode();
    }

    osDelay(500);
  }
  /* USER CODE END StartStopTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
void HAL_GPIO_EXTI0Callback(RTC_HandleTypeDef *hrtc)
{
}

void HAL_RTC_AlarmAEventCallback(RTC_HandleTypeDef *hrtc)
{
  RTCAlarm_Flag = 1;
}
void HSE_Init(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /**Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI | RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL6;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}
void PreStopMode(void)
{
  HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_2);
  HAL_TIM_Base_Stop(&htim3);
  HAL_UART_DeInit(&huart1);
  HAL_UART_DeInit(&huart2);
  HAL_CRC_DeInit(&hcrc);
  HAL_TIM_Base_DeInit(&htim3);
  HAL_I2C_DeInit(&hi2c2);

  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_GPIOC);
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_GPIOA);
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_GPIOB);

  /**/
  GPIO_InitStruct.Pin = LL_GPIO_PIN_14 | LL_GPIO_PIN_15;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ANALOG;
  LL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = LL_GPIO_PIN_1 | LL_GPIO_PIN_2 | LL_GPIO_PIN_3 | LL_GPIO_PIN_4 | LL_GPIO_PIN_5 | LL_GPIO_PIN_6 | LL_GPIO_PIN_7 | LL_GPIO_PIN_8 | LL_GPIO_PIN_9 | LL_GPIO_PIN_10 | LL_GPIO_PIN_11 | LL_GPIO_PIN_12 | LL_GPIO_PIN_13 | LL_GPIO_PIN_14 | LL_GPIO_PIN_15;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ANALOG;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = LL_GPIO_PIN_0 | LL_GPIO_PIN_1 | LL_GPIO_PIN_2 | LL_GPIO_PIN_10 | LL_GPIO_PIN_11 | LL_GPIO_PIN_12 | LL_GPIO_PIN_13 | LL_GPIO_PIN_14 | LL_GPIO_PIN_15 | LL_GPIO_PIN_3 | LL_GPIO_PIN_4 | LL_GPIO_PIN_5 | LL_GPIO_PIN_6 | LL_GPIO_PIN_7 | LL_GPIO_PIN_8 | LL_GPIO_PIN_9;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ANALOG;
  LL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  LL_APB2_GRP1_DisableClock(LL_APB2_GRP1_PERIPH_GPIOA);
  LL_APB2_GRP1_DisableClock(LL_APB2_GRP1_PERIPH_GPIOB);
  LL_APB2_GRP1_DisableClock(LL_APB2_GRP1_PERIPH_GPIOC);

  osThreadTerminate(defaultTaskHandle);
  osThreadTerminate(GUITaskHandle);
  osThreadTerminate(KeyTaskHandle);
  osThreadTerminate(AlarmTaskHandle);

  osMessageDelete(KeyQueueHandle);
  osMessageDelete(StopQueueHandle);

  HAL_DeInit();
}

void AftStopMode(void)
{
  HAL_Init();
  SystemClock_Config();
  MX_GPIO_Init();
  MX_USART2_UART_Init();
  MX_I2C2_Init();
  MX_TIM3_Init();
  MX_RTC_Init();
  MX_CRC_Init();
  MX_USART1_UART_Init();
  //	 MX_NVIC_Init();
  HAL_TIM_Base_Start(&htim3);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);

  //  MX_FREERTOS_Init();

  /* Start scheduler */
  //  osKernelStart();
  //		GUI_Init();
  //		GUI_SetBkColor(GUI_BLACK);
  //		GUI_SetColor(GUI_GREEN);
  //		GUI_Clear();
  //	 GUI_SetFont(GUI_FONT_13_ASCII);
  //	 CreateDesktop();
}

/* USER CODE END Application */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
