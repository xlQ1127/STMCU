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
#include "gui.h"
#include "BSP_Key.h"
#include "BSP_LED.h"
#include "BSP_FDC2214.h"

#include "stdlib.h"
#include "string.h"

#include "fatfs.h"

#include "DIALOG.h"
#include "arm_math.h"
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
uint8_t Gest=0;

float32_t Cap_Mean=0;

/* USER CODE END Variables */
osThreadId defaultTaskHandle;
osThreadId GUITaskHandle;
osThreadId KeyTaskHandle;
osThreadId FDCTaskHandle;
osMessageQId KeyQueueHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
WM_HWIN CreateMenu(void);
/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void const *argument);
void StartGUITask(void const *argument);
void StartKeyTask(void const *argument);
void StartFDCTask(void const *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

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
  osThreadDef(KeyTask, StartKeyTask, osPriorityHigh, 0, 128);
  KeyTaskHandle = osThreadCreate(osThread(KeyTask), NULL);

  /* definition and creation of FDCTask */
  osThreadDef(FDCTask, StartFDCTask, osPriorityRealtime, 0, 256);
  FDCTaskHandle = osThreadCreate(osThread(FDCTask), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* Create the queue(s) */
  /* definition and creation of KeyQueue */
  osMessageQDef(KeyQueue, 10, Key_Statu_Typedef);
  KeyQueueHandle = osMessageCreate(osMessageQ(KeyQueue), NULL);

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
    osDelay(100);
    LED2_OFF;

    LED3_ON;
    osDelay(100);
    LED3_OFF;

    LED4_ON;
    osDelay(100);
    LED4_OFF;

    LED5_ON;
    osDelay(100);
    LED5_OFF;

    LED4_ON;
    osDelay(100);
    LED4_OFF;

    LED3_ON;
    osDelay(100);
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
//  WM_HWIN hItem;
  osEvent KeyEvent;
//  uint8_t i = 0;
//  char s[16];

	
  GUI_Init();
  GUI_SetBkColor(GUI_BLUE);
  GUI_SetColor(GUI_GREEN);
  GUI_Clear();

  CreateMenu();

  //  short Cap[128];

  //  GRAPH_DATA_Handle hFDC_GRAPH_Data;
  //  GRAPH_SCALE_Handle hFDC_GRAPH_Scale; //刻度的句柄
  //  hItem = FDC_Graph();
  //  hItem = WM_GetDialogItem(hItem, 0x800 + 0x01);
  //  GRAPH_SetBorder(hItem, 0, 0, 0, 0);
  // USER START (Optionally insert additional code for further widget initialization) hData = GRAPH_DATA_YT_Create(GUI_GREEN,240, &m, 1);
  //  hFDC_GRAPH_Data = GRAPH_DATA_YT_Create(GUI_GREEN, 128, Cap, 128);
  //  GRAPH_AttachData(hItem, hFDC_GRAPH_Data);

  //  GRAPH_SetGridDistX(hItem, 10); //设置栅格距离
  //  GRAPH_SetGridDistY(hItem, 10);
  //  GRAPH_SetGridVis(hItem, 1);

  //  GRAPH_DATA_YT_SetAlign(hFDC_GRAPH_Data, GRAPH_ALIGN_LEFT); //数据左对齐
  //                                                         //  GRAPH_DATA_YT_Clear(hFDC_GRAPH_Data);                  //清除数据对象的所有数据项

  //  hFDC_GRAPH_Scale = GRAPH_SCALE_Create(30, GUI_TA_RIGHT, GRAPH_SCALE_CF_VERTICAL | GRAPH_SCALE_CF_HORIZONTAL, 10); //刻度创建
  //  GRAPH_AttachScale(hItem, hFDC_GRAPH_Scale);
  //  GRAPH_SCALE_SetFactor(hFDC_GRAPH_Scale, 10); //设置用于计算要绘制的编号的因子，也就是刻度的比例

  //  FDC2214_Init();
  //  hItem =

  /* Infinite loop */
  for (;;)
  {
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
      GUI_SendKeyMsg(GUI_KEY_SPACE, 1);
      GUI_SendKeyMsg(GUI_KEY_BACKSPACE, 1);
    }
    //    for (i = 0; i < 128; i++)
    //    {
    //      Cap[i] = (short)(FDC2214_Calculate_Cap(FCD2214_GetCap_Data(0)) / 10);
    //  //				Cap[i]=i*i;
    //      GRAPH_DATA_YT_AddValue(hFDC_GRAPH_Data, Cap[i]);
    //      osDelay(10);
    //    }

    GUI_Exec();
    osDelay(100);
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
  Key_Statu_Typedef Key;
  Key_init(&Key);
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

        //        osMessagePut(StopQueueHandle, (uint32_t)Key.Key_WKUP_Flag, 10);
      }
    }
    osMessagePut(KeyQueueHandle, (uint32_t)&Key, 0);

    osDelay(100);
  }
  /* USER CODE END StartKeyTask */
}

/* USER CODE BEGIN Header_StartFDCTask */
/**
* @brief Function implementing the FDCTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartFDCTask */
void StartFDCTask(void const *argument)
{
  /* USER CODE BEGIN StartFDCTask */
  float32_t FDC_Cap = 0;
	
	 float32_t Cap_Buf[2]={0}; 

  uint8_t FDC_Buf_index = 0;
	 uint8_t Cap_Mean_index = 0;
	 float32_t Cap_rate = 0;
	 
  //  char Data_buf[22] = {0};

  FDC2214_Init();
  //  FRESULT Res;
  //  char Fat_Wdata[1024] = {0};
  //  uint32_t byteswritten, bytesread; /* File write/read counts */

  //  MX_FATFS_Init();
  //  Res = f_mount(&USERFatFS, (const TCHAR *)USERPath, 1);
  //  if (Res != FR_OK)
  //  {
  //    while (1)
  //      ;
  //  }

  /* Infinite loop */
  for (;;)
  {
    /************数据变化率判断 *************/
    FDC_Cap = FDC2214_Calculate_Cap(FCD2214_GetCap_Data(0));
//			 printf("Cap:%10.10f\r\n", FDC_Cap);

						Cap_Buf[FDC_Buf_index]=FDC_Cap;
						FDC_Buf_index++;
						if (FDC_Buf_index >= 2)
						{
								FDC_Buf_index = 0;
							
					/* 变化率太高则不稳定 具体数值可以设置为 cap_buf(0)的2倍  每200ms 算一次变化率*/
							 Cap_rate=(Cap_Buf[1]-Cap_Buf[0])*5;
//						 	printf("Rate:%10.10f\r\n", Cap_rate);
							 if( Cap_rate >=50.0f  && Cap_rate <=-50.0f)
								{
									Cap_Mean=0;
								 Gest=0;
								}
								/************稳定数值（变化率约为0.2左右）基础上 判断手势 *************/
							 else if( Cap_rate <= 15.0f && Cap_rate >= -15.0f )
								{
									 Cap_Mean_index++;
									 Cap_Mean+=(Cap_Buf[1]+Cap_Buf[0])/2;		
									 if(Cap_Mean_index>=2)
										{
										 Cap_Mean/=2;
											printf("Mean:%10.10f\r\n", Cap_Mean);		
										}
          							
								
								}								
				  }
				
    //    Cap_Buf[FDC_index] = FDC_Cap;
    //    sprintf(Data_buf, "%10.10f\n", FDC_Cap);
    //    FDC_Data = FCD2214_GetCap_Data(0);
    //     strcat((char *)Fat_Wdata, (char *)Data_buf);

    //					Res = f_open(&USERFile, "Record.txt", FA_CREATE_ALWAYS | FA_WRITE);
    //					if (Res != FR_OK)
    //					{
    //							while (1)
    //									;
    //					}
    //    Res = f_write(&USERFile, Fat_Wdata, sizeof(Fat_Wdata), (void *)&byteswritten);
    //    if (Res != FR_OK)
    //    {
    //      while (1)
    //        ;
    //    }
    //    f_close(&USERFile);
    osDelay(10);
  }
  /* USER CODE END StartFDCTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
