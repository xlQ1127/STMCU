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
#include "bsp_driver_sd.h"
#include "gui.h"
#include "fatfs.h"
#include "sd_diskio.h"

#include "DIALOG.h"

#include "string.h"

#include "printf_scanf.h"
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
unsigned long RunTimeCounter;
  FRESULT res;  
		UINT bw;
extern TIM_HandleTypeDef htim7;
/* USER CODE END Variables */
osThreadId defaultTaskHandle;
osThreadId myTask02Handle;
osThreadId myTask03Handle;
osThreadId myTask04Handle;
osMutexId myMutex01Handle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
   WM_HWIN CalWindow(void);
			void _WriteByte2File(U8 Data, void * p) ;
/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void const * argument);
void StartTask02(void const * argument);
void StartTask03(void const * argument);
void StartTask04(void const * argument);

extern void MX_FATFS_Init(void);
void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* Hook prototypes */
void configureTimerForRunTimeStats(void);
unsigned long getRunTimeCounterValue(void);

static int _GetData(void * p, const U8 ** ppData, unsigned NumBytesReq, U32 Off)
{
  unsigned int NumBytesRead;
  
  /* Set file pointer to the required position */
  f_lseek((FIL *)p, Off);
  
  /* Read data into buffer */
  f_read((FIL *)p, (U8 *)*ppData, NumBytesReq, &NumBytesRead);
   
  /* Return number of available bytes */
  return NumBytesRead;  
}

void _cbNotify(GUI_HMEM hMem, int Notification, U32 CurrentFrame) 
{ 
  WM_HWIN  hItem;
  switch (Notification) 
  {   
  case GUI_MOVIE_NOTIFICATION_PREDRAW:
    break;
    
  case GUI_MOVIE_NOTIFICATION_POSTDRAW:
    break;
    
  case GUI_MOVIE_NOTIFICATION_STOP:
    break;
  }
}

/* USER CODE BEGIN 1 */
/* Functions needed when configGENERATE_RUN_TIME_STATS is on */
__weak void configureTimerForRunTimeStats(void)
{
  RunTimeCounter=0UL;
}

__weak unsigned long getRunTimeCounterValue(void)
{
return RunTimeCounter;
}
/* USER CODE END 1 */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */
  GUI_Init();
//	 GUI_SetBkColor(GUI_RED);
//	 GUI_Clear();  
	
	 HAL_TIM_Base_Start_IT(&htim7);
	 printf("FreeRTOS\r\n");
  /* USER CODE END Init */

  /* Create the mutex(es) */
  /* definition and creation of myMutex01 */
  osMutexDef(myMutex01);
  myMutex01Handle = osMutexCreate(osMutex(myMutex01));

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
  osThreadDef(defaultTask, StartDefaultTask, osPriorityRealtime, 0, 4096);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* definition and creation of myTask02 */
  osThreadDef(myTask02, StartTask02, osPriorityNormal, 0, 4096);
  myTask02Handle = osThreadCreate(osThread(myTask02), NULL);

  /* definition and creation of myTask03 */
  osThreadDef(myTask03, StartTask03, osPriorityNormal, 0, 4096);
  myTask03Handle = osThreadCreate(osThread(myTask03), NULL);

  /* definition and creation of myTask04 */
  osThreadDef(myTask04, StartTask04, osPriorityLow, 0, 4096);
  myTask04Handle = osThreadCreate(osThread(myTask04), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

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
  /* init code for FATFS */
  MX_FATFS_Init();

  /* USER CODE BEGIN StartDefaultTask */
                                        /* FatFs function common retSDult code */
//		uint32_t byteswritten, bytesread;                     /* File write/read counts */
//		uint8_t wtext[] = "FreeRTOS STemwin FatFs"; /* File write buffer */
//		uint8_t rtext[100]; 	
//	
//	 MX_FATFS_Init();

//  if(f_mount(&SDFatFS, (TCHAR const*)SDPath, 0) != FR_OK)
//  {
//    /* FatFs Initialization Error */
//    Error_Handler();
//  }
//  else
//  {
//      /* Create and Open a new text file object with write access */
//      if(f_open(&SDFile, "STM32.TXT", FA_CREATE_ALWAYS | FA_WRITE) != FR_OK)
//      {
//        /* 'STM32.TXT' file Open for write Error */
//        Error_Handler();
//      }
//      else
//      {
//        /* Write data to the text file */
//        res = f_write(&SDFile, wtext, sizeof(wtext), (void *)&byteswritten);
//        
//        if((byteswritten == 0) || (res != FR_OK))
//        {
//          /* 'STM32.TXT' file Write or EOF Error */
//          Error_Handler();
//        }
//        else
//        {
//          /* Close the open text file */
//          f_close(&SDFile);
//         
//        /* Open the text file object with read access */
//        if(f_open(&SDFile, "STM32.TXT", FA_READ) != FR_OK)
//        {
//          /* 'STM32.TXT' file Open for read Error */
//          Error_Handler();
//        }
//        else
//        {
//          /* Read data from the text file */
//          res = f_read(&SDFile, rtext, sizeof(rtext), (void *)&bytesread);
//         
//          if((bytesread == 0) || (res != FR_OK))
//          {
//            /* 'STM32.TXT' file Read or EOF Error */
//            Error_Handler();
//          }
//          else
//          {
//            /* Close the open text file */
//            f_close(&SDFile);
//            
//            /* Compare read data with the expected data */
//            if((bytesread != byteswritten))
//            {               
//              /* Read data is different from the expected data */
//              Error_Handler();
//            }
//            else
//            {
//          /* Success of the demo: no error occurrence */
//																printf("Success of the demo\r\n");
//            }
//          }
//        }
//      }
//    }
//  }

  /* Infinite loop */
  for(;;)
  {
			 HAL_GPIO_TogglePin(GPIOA,D2_Pin|D3_Pin);
    osDelay(500);
  }
  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_StartTask02 */
/**
* @brief Function implementing the myTask02 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTask02 */
void StartTask02(void const * argument)
{
  /* USER CODE BEGIN StartTask02 */
//	 uint8_t i=0;
//  GUI_SetBkColor(GUI_RED);
//	 GUI_SetColor(GUI_BLUE);
  /* Infinite loop */
  for(;;)
  {
//   osSemaphoreWait(myMutex01Handle,osWaitForever); 
//			GUI_DispDecAt(i,0,0,8);
//   osSemaphoreRelease(myMutex01Handle);
//			i++;
   osDelay(1000);
  }
  /* USER CODE END StartTask02 */
}

/* USER CODE BEGIN Header_StartTask03 */
/**
* @brief Function implementing the myTask03 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTask03 */
void StartTask03(void const * argument)
{
  /* USER CODE BEGIN StartTask03 */
	 GUI_MOVIE_HANDLE hMovie;
	
  if(f_mount(&SDFatFS, (TCHAR const*)SDPath, 0) != FR_OK)
  {
    /* FatFs Initialization Error */
    Error_Handler();
  }
		
	 if(f_open(&SDFile, "TWD.emf", FA_OPEN_EXISTING | FA_READ) == FR_OK)
  {
	 hMovie=GUI_MOVIE_CreateEx(_GetData,&SDFile,_cbNotify);
		GUI_MOVIE_SetPeriod (hMovie, 40);
		}
	 GUI_MOVIE_DrawFrame(hMovie,1,0,0);
//  CalWindow();
  /* Infinite loop */
  for(;;)
  {
			GUI_MOVIE_Show(hMovie,80,60,0);
//			GUI_TOUCH_Exec();
//   GUI_Delay(60);
  }
  /* USER CODE END StartTask03 */
}

/* USER CODE BEGIN Header_StartTask04 */
/**
* @brief Function implementing the myTask04 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTask04 */
void StartTask04(void const * argument)
{
  /* USER CODE BEGIN StartTask04 */
	 uint8_t buf1[128];
	 char    buf2[128];
	
	 char buf[20];
		uint8_t	 Pic_Name = 0;
//	 memset(buf,0,128);
  /* Infinite loop */
  for(;;)
  {		
			
			if(HAL_GPIO_ReadPin(KEY1_GPIO_Port,KEY1_Pin)!=GPIO_PIN_SET)
			{			
					HAL_Delay(10);
					if(HAL_GPIO_ReadPin(KEY1_GPIO_Port,KEY1_Pin)!=GPIO_PIN_SET)
					{				
						osThreadList(buf1);			
						osSemaphoreWait(myMutex01Handle,osWaitForever); 
						printf("%s\r\n",buf1);
						osSemaphoreRelease(myMutex01Handle);
						vTaskGetRunTimeStats(buf2);
						osSemaphoreWait(myMutex01Handle,osWaitForever); 
						printf("%s\r\n",buf2);
						osSemaphoreRelease(myMutex01Handle);
					}
			}
		
			if(HAL_GPIO_ReadPin(KEY0_GPIO_Port,KEY0_Pin)!=GPIO_PIN_SET)
			{			
					HAL_Delay(10);
					if(HAL_GPIO_ReadPin(KEY0_GPIO_Port,KEY0_Pin)!=GPIO_PIN_SET)
					{															
					}
			}
			osDelay(1000);
  }
  /* USER CODE END StartTask04 */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
