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
  * Copyright (c) 2019 STMicroelectronics International N.V. 
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
#include "bsp_key.h"
#include "bsp_led.h"
#include "bsp_ws2812.h"
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

/* USER CODE END Variables */
osThreadId defaultTaskHandle;
osThreadId KeyTaskHandle;
osThreadId WS2812TaskHandle;
osThreadId DesklampTskHandle;
osThreadId rainbowTaskHandle;
osThreadId TheaterTaskHandle;
osMessageQId KeyQueueHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
   
/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void const * argument);
void StartKeyTask(void const * argument);
void StartWS2812Task(void const * argument);
void StartDesklampTask(void const * argument);
void StartrainbowTask(void const * argument);
void StartTheaterTask(void const * argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */
       
  /* USER CODE END Init */

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
  /* definition and creation of KeyQueue */
  osMessageQDef(KeyQueue, 8, Key_Statu_Typedef);
  KeyQueueHandle = osMessageCreate(osMessageQ(KeyQueue), NULL);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityLow, 0, 64);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* definition and creation of KeyTask */
  osThreadDef(KeyTask, StartKeyTask, osPriorityRealtime, 0, 128);
  KeyTaskHandle = osThreadCreate(osThread(KeyTask), NULL);

  /* definition and creation of WS2812Task */
  osThreadDef(WS2812Task, StartWS2812Task, osPriorityAboveNormal, 0, 256);
  WS2812TaskHandle = osThreadCreate(osThread(WS2812Task), NULL);

  /* definition and creation of DesklampTsk */
  osThreadDef(DesklampTsk, StartDesklampTask, osPriorityNormal, 0, 128);
  DesklampTskHandle = osThreadCreate(osThread(DesklampTsk), NULL);

  /* definition and creation of rainbowTask */
  osThreadDef(rainbowTask, StartrainbowTask, osPriorityNormal, 0, 128);
  rainbowTaskHandle = osThreadCreate(osThread(rainbowTask), NULL);

  /* definition and creation of TheaterTask */
  osThreadDef(TheaterTask, StartTheaterTask, osPriorityNormal, 0, 128);
  TheaterTaskHandle = osThreadCreate(osThread(TheaterTask), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used 
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void const * argument)
{

  /* USER CODE BEGIN StartDefaultTask */
  /* Infinite loop */
  for(;;)
  {
			 LED1_ON;
			 osDelay(50);
			 LED1_OFF;
    osDelay(950);
  }
  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_StartKeyTask */
/**
* @brief Function implementing the KeyTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartKeyTask */
void StartKeyTask(void const * argument)
{
  /* USER CODE BEGIN StartKeyTask */
	 Key_Statu_Typedef Key;
	 
	 Key_init(&Key);
	 Scan_Key(&Key);
  /* Infinite loop */
  for(;;)
  {
			 Scan_Key(&Key);
			 osMessagePut(KeyQueueHandle,(uint32_t)(&Key),0);
    osDelay(50);
  }
  /* USER CODE END StartKeyTask */
}

/* USER CODE BEGIN Header_StartWS2812Task */
/**
* @brief Function implementing the WS2812Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartWS2812Task */
void StartWS2812Task(void const * argument)
{
  /* USER CODE BEGIN StartWS2812Task */
	 osEvent Key_Event;

  /* Infinite loop */
  for(;;)
  {
			Key_Event=osMessageGet(KeyQueueHandle,10);
			 
			if(  ( (Key_Statu_Typedef *)(Key_Event.value.p) )->Key1_Flag == Key_Pressed )
			{
				osThreadResume(DesklampTskHandle);
				osThreadSuspend(rainbowTaskHandle);
				osThreadSuspend(TheaterTaskHandle);
			}

	  if  ( ( (Key_Statu_Typedef *)(Key_Event.value.p) )->Key2_Flag == Key_Pressed )
   {
    	osThreadResume(rainbowTaskHandle);
				 osThreadSuspend(DesklampTskHandle);
			 	osThreadSuspend(TheaterTaskHandle);
			}				

			if ( ( (Key_Statu_Typedef *)(Key_Event.value.p) )->Key3_Flag == Key_Pressed )
   {
     	osThreadResume(TheaterTaskHandle);
				  osThreadSuspend(DesklampTskHandle);
				  osThreadSuspend(rainbowTaskHandle);
			}			
			
			
			
    osDelay(100);
  }
  /* USER CODE END StartWS2812Task */
}

/* USER CODE BEGIN Header_StartDesklampTask */
/**
* @brief Function implementing the DesklampTas thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartDesklampTask */
void StartDesklampTask(void const * argument)
{
  /* USER CODE BEGIN StartDesklampTask */
	 WS2812_HandleTypeDef hws2812;
	
	 WS2812_Init(&hws2812,16);
	
	 osThreadSuspend(NULL);
  /* Infinite loop */
  for(;;)
  {


			 ws2812_colorWipe(&hws2812,WS_WHITE,100);  //只需操作WS2812 一次

    osDelay(100);
  }
  /* USER CODE END StartDesklampTask */
}

/* USER CODE BEGIN Header_StartrainbowTask */
/**
* @brief Function implementing the rainbowTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartrainbowTask */
void StartrainbowTask(void const * argument)
{
  /* USER CODE BEGIN StartrainbowTask */
		WS2812_HandleTypeDef hws2812;
	
	 WS2812_Init(&hws2812,16);
	
	 osThreadSuspend(NULL);
  /* Infinite loop */
  for(;;)
  {
			
			 ws2812_rainbow(&hws2812,100);     //需循环控制WS2812
			 osDelay(10);

  }
  /* USER CODE END StartrainbowTask */
}

/* USER CODE BEGIN Header_StartTheaterTask */
/**
* @brief Function implementing the TheaterTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTheaterTask */
void StartTheaterTask(void const * argument)
{
  /* USER CODE BEGIN StartTheaterTask */
	 uint32_t color=0;
	 uint8_t i=0;
	 WS2812_HandleTypeDef hws2812;
	
	 WS2812_Init(&hws2812,16);
	
	 osThreadSuspend(NULL);
  /* Infinite loop */
  for(;;)
  {
			 color=Wheel(i);
			 ws2812_blink_all(&hws2812,color ,500);           //需循环控制WS2812
			 i++;
    osDelay(10);
  }
  /* USER CODE END StartTheaterTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
     
/* USER CODE END Application */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
