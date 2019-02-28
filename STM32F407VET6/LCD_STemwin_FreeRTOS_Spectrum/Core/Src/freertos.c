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
//外设
#include "adc.h"

//第三方库
#include "gui.h"
#include "DIALOG.h"
#include "arm_math.h"
//BSP库
#include "bsp_led.h"

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
osThreadId GUITaskHandle;
osThreadId ADCTaskHandle;
osMessageQId ADCVHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
   
WM_HWIN CreateSpectrum(void);			
			
/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void const * argument);
void StartGUITask(void const * argument);
void StartADCTask(void const * argument);

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

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityLow, 0, 64);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* definition and creation of GUITask */
  osThreadDef(GUITask, StartGUITask, osPriorityLow, 0, 4096);
  GUITaskHandle = osThreadCreate(osThread(GUITask), NULL);

  /* definition and creation of ADCTask */
  osThreadDef(ADCTask, StartADCTask, osPriorityRealtime, 0, 1024);
  ADCTaskHandle = osThreadCreate(osThread(ADCTask), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* Create the queue(s) */
  /* definition and creation of ADCV */
  osMessageQDef(ADCV, 8, float);
  ADCVHandle = osMessageCreate(osMessageQ(ADCV), NULL);

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
void StartDefaultTask(void const * argument)
{

  /* USER CODE BEGIN StartDefaultTask */
  /* Infinite loop */
  for(;;)
  {
			
			 LED1_ON;
		  LED2_ON;
			 osDelay(50);
			 LED1_OFF;
			 LED2_OFF;
    osDelay(950);
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
void StartGUITask(void const * argument)
{
  /* USER CODE BEGIN StartGUITask */
	 osEvent ADCEVENT;
//	
	 WM_HWIN hItem;
		GRAPH_DATA_Handle hGRAPH_Data;
		GRAPH_SCALE_Handle hGRAPH_Scale; //刻度的句柄
	
	 short Cap;

	 GUI_Init();
	 GUI_SetBkColor(GUI_BLUE);
	 GUI_Clear();
	 GUI_DispStringAtCEOL("ADC",160,120);
//	
	 hItem =CreateSpectrum();
  hItem = WM_GetDialogItem(hItem, 0x800 + 0x01);
	
	 hGRAPH_Data = GRAPH_DATA_YT_Create(GUI_GREEN, 240, 0, 0);
  GRAPH_AttachData(hItem, hGRAPH_Data);
		GRAPH_DATA_YT_SetAlign(hGRAPH_Data, GRAPH_ALIGN_LEFT); //数据左对齐
	
  GRAPH_SetBorder(hItem, 30, 0, 0, 0);
	 GRAPH_SetGridDistX(hItem, 10); //设置栅格距离
  GRAPH_SetGridDistY(hItem, 10);
  GRAPH_SetGridVis(hItem, 1);
	 
		hGRAPH_Scale = GRAPH_SCALE_Create(30, GUI_TA_RIGHT, GRAPH_SCALE_CF_VERTICAL | GRAPH_SCALE_CF_HORIZONTAL, 10); //刻度创建
  GRAPH_AttachScale(hItem, hGRAPH_Scale);
		GRAPH_SCALE_SetFactor(hGRAPH_Scale,1); //设置用于计算要绘制的编号的因子，也就是刻度的比例
  /* Infinite loop */
  for(;;)
  {
			
				 ADCEVENT=osMessageGet(ADCVHandle,10);
		  	Cap=(short) (ADCEVENT.value.v);
				 GRAPH_DATA_YT_AddValue(hGRAPH_Data,Cap);
			
				 GUI_Exec(); 
     osDelay(10);
  }
  /* USER CODE END StartGUITask */
}

/* USER CODE BEGIN Header_StartADCTask */
/**
* @brief Function implementing the ADCTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartADCTask */
void StartADCTask(void const * argument)
{
  /* USER CODE BEGIN StartADCTask */
	 float ADC_V[1024];
	 float ADC_Data;
	 float ADC_O[512];

  arm_cfft_instance_f32 arm_cfft_sR_f32_len1024;
  /* Infinite loop */
  for(;;)
  {
			
//    HAL_ADC_Start_DMA(&hadc1,&ADC_Data,1);
//			 ADC_V = (float) (ADC_Data*(3.3/4095));
//			 ADC_V*=70;
//    HAL_ADC_Stop_DMA(&hadc1);
//			for(uint16_t i=0;i<1024;i++)
//			{
//			 	ADC_V[i] = arm_sin_f32(ADC_Data);
//				 ADC_Data+=PI/512;
//			}

			
//			if(ADC_Data>2*PI) ADC_Data=0;
//			arm_abs_f32(&ADC_V,&ADC_V,1);
//			ADC_V*=220;
			
			
// /* Process the data through the CFFT/CIFFT module */
//  arm_cfft_f32(&arm_cfft_sR_f32_len1024, ADC_V, 0, 1);

//  /* Process the data through the Complex Magnitude Module for
//  calculating the magnitude at each bin */
//  arm_cmplx_mag_f32(ADC_V, ADC_O, 1024);

//		osMessagePut(ADCVHandle,(uint32_t)&ADC_O,0);
  osDelay(5);
  }
  /* USER CODE END StartADCTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
     
/* USER CODE END Application */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
