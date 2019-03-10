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
#include "string.h"

#include "adc.h"
#include "crc.h"
#include "usart.h"

#include "BSP_LED.h"
#include "BSP_NRF24L01.h"
#include "BSP_OLED.h"
#include "BSP_Key.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define NRF_TX_MAXRETRY 5
#define NRF_RX_MAXRETRY 5


/* Roll  innner INDEX_PID 1 
         outer  INDEX_PID 4
			pitch inner  INDEX_PID 7 
			      outer  INDEX_PID 10
*/

#define INDEX_PID  1
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
typedef __packed struct{
	uint8_t Function_Code; //1B
	float P; //4B
	float I; //4B
	float D; //4B
	uint8_t Select_Index;//1B
}PID_Typedef;    //26B


typedef __packed struct{
	uint8_t Function_Code;
	uint8_t Key_Left_Flag; 
	uint8_t Key_Right_Flag; 
	uint8_t key_Count;  //4B
	int16_t LX;  //2B YAW       -180~+180
	int16_t LY;  //2B Throttle  -2000~+2000
	int8_t RX;  //1B Roll      -30~+30
	int8_t RY;  //1B Pitch     -30~+30
	uint8_t Heart_Beat; //1B
}JoyKey_Typedef;   //10B

typedef __packed struct{
	uint8_t Function_Code;
	float Yaw;  //4B
	float Pitch;  //4B
	float Roll;  //4B
}Gesture_Typedef;

/* USER CODE END Variables */
osThreadId DefaultTaskHandle;
osThreadId NRFTaskHandle;
osThreadId OLED_TaskHandle;
osThreadId JoyKeyTaskHandle;
osThreadId PIDTaskHandle;
osMessageQId JoyKeyQueueHandle;
osMessageQId GestureHandle;
osMessageQId PIDQueueHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void const * argument);
void StartNRFTask(void const * argument);
void StartOLED_Task(void const * argument);
void StartJoyKeyTask(void const * argument);
void StartPIDTask(void const * argument);

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
  /* definition and creation of DefaultTask */
  osThreadDef(DefaultTask, StartDefaultTask, osPriorityLow, 0, 64);
  DefaultTaskHandle = osThreadCreate(osThread(DefaultTask), NULL);

  /* definition and creation of NRFTask */
  osThreadDef(NRFTask, StartNRFTask, osPriorityRealtime, 0, 512);
  NRFTaskHandle = osThreadCreate(osThread(NRFTask), NULL);

  /* definition and creation of OLED_Task */
  osThreadDef(OLED_Task, StartOLED_Task, osPriorityNormal, 0, 512);
  OLED_TaskHandle = osThreadCreate(osThread(OLED_Task), NULL);

  /* definition and creation of JoyKeyTask */
  osThreadDef(JoyKeyTask, StartJoyKeyTask, osPriorityNormal, 0, 512);
  JoyKeyTaskHandle = osThreadCreate(osThread(JoyKeyTask), NULL);

  /* definition and creation of PIDTask */
  osThreadDef(PIDTask, StartPIDTask, osPriorityBelowNormal, 0, 128);
  PIDTaskHandle = osThreadCreate(osThread(PIDTask), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
		
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* Create the queue(s) */
  /* definition and creation of JoyKeyQueue */
  osMessageQDef(JoyKeyQueue, 8, JoyKey_Typedef);
  JoyKeyQueueHandle = osMessageCreate(osMessageQ(JoyKeyQueue), NULL);

  /* definition and creation of Gesture */
  osMessageQDef(Gesture, 8, Gesture_Typedef);
  GestureHandle = osMessageCreate(osMessageQ(Gesture), NULL);

  /* definition and creation of PIDQueue */
  osMessageQDef(PIDQueue, 8, PID_Typedef);
  PIDQueueHandle = osMessageCreate(osMessageQ(PIDQueue), NULL);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* USER CODE END RTOS_QUEUES */
}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the DefaultTask thread.
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
		  LED_ON;
				osDelay(50);
				LED_OFF;
			
    osDelay(950);
  }
  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_StartNRFTask */
/**
* @brief Function implementing the NRFTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartNRFTask */
void StartNRFTask(void const * argument)
{
  /* USER CODE BEGIN StartNRFTask */
	 Gesture_Typedef Gesture;

	 osEvent JoyKey_Event;
	 osEvent PID_Event;
	

	 uint8_t NRF_Tx_buf[32];  //1字节功能码+30字节数据-------------------
  uint8_t NRF_Rx_buf[32];  
  uint32_t CRC_Buf[31];

 	uint8_t NRF_Mode=1;//0 无  1 发送 2 接收
	 __IO uint8_t Function_Code='C';
	 uint8_t NRF_Rx_index=0;
  uint8_t Time_Index=0;

 	NRF24L01_Init(40,NRF_Mode);//发送
	 
  /* Infinite loop */
  for(;;)
  { 
//			 if( Time_Index >50 )
//				{
//			  Function_code= 'R';
//					Gesture.Function_Code=Function_code;
//					Time_Index=0;
//			 }
//				else
//				{
				JoyKey_Event=osMessagePeek(JoyKeyQueueHandle,0);
			
		 	PID_Event=osMessagePeek(PIDQueueHandle,0);

    switch( NRF_Mode )
				{					
					case 1:
							NRF24L01_TX_Mode();
							NRF_Mode=0;
						break;
					
					case 2:
							NRF24L01_RX_Mode();
							NRF_Mode=0;
						break;				
					default:
						break;						
				}
				
				if( ((PID_Typedef *)(PID_Event.value.p))->Function_Code == 'S')
				{
					Function_Code='S';
				}
				else
				{
					Function_Code=((JoyKey_Typedef *)(JoyKey_Event.value.p))->Function_Code;
				}	
				
    switch( Function_Code )
				{
					case 'D':		
						NRF_Tx_buf[0]='D';
					 memcpy(CRC_Buf,NRF_Tx_buf,31 );
					 NRF_Tx_buf[31] =(uint8_t) HAL_CRC_Calculate(&hcrc, CRC_Buf, 31);
						NRF24L01_TxPacket(NRF_Tx_buf);
						break;	
					
					case 'C':
						memcpy( NRF_Tx_buf,((JoyKey_Typedef *)(JoyKey_Event.value.p)),sizeof(JoyKey_Typedef) );
					 ((JoyKey_Typedef *)(JoyKey_Event.value.p))->Heart_Beat++;
					 memcpy(CRC_Buf,NRF_Tx_buf,31 );
					 NRF_Tx_buf[31] =(uint8_t)HAL_CRC_Calculate(&hcrc, CRC_Buf, 31);
						NRF24L01_TxPacket(NRF_Tx_buf);
						break;
					
					case 'R':  
							NRF24L01_TxPacket(NRF_Tx_buf);		
							if( NRF_Mode!=2 )
							{
										NRF24L01_RX_Mode();
							}				
							while(++NRF_Rx_index)
								{ 
											HAL_Delay(1);//加入该延时 即可收   原因未知？？？
											NRF24L01_RxPacket(NRF_Rx_buf);
											memcpy( &Gesture,NRF_Rx_buf,sizeof(Gesture_Typedef) );
											osMessagePut(GestureHandle,(uint32_t)&Gesture,0);
											if( NRF_Rx_index>NRF_RX_MAXRETRY ) 
											{
												NRF_Rx_index=0;
												Function_Code='C';
												break;
											}																					
								}		
						NRF_Mode=1;																	
						break;		
					
					case 'S':
						memcpy( NRF_Tx_buf,((PID_Typedef *)(PID_Event.value.p)),sizeof(PID_Typedef));
					 NRF24L01_TxPacket(NRF_Tx_buf);
						break;		
					
					default:
						break;						
				}
    Time_Index++;
    osDelay(5);
  }
  /* USER CODE END StartNRFTask */
}

/* USER CODE BEGIN Header_StartOLED_Task */
/**
* @brief Function implementing the OLED_Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartOLED_Task */
void StartOLED_Task(void const * argument)
{
  /* USER CODE BEGIN StartOLED_Task */
  char str_buf1[16],str_buf2[10],str_buf3[19];
	 osEvent JoyKey_Event;
//  osEvent Gesture_Event;
	 osEvent PID_Event;

	 uint8_t Function_Code;
  /* Infinite loop */
  for(;;)
  {
  PID_Event=osMessagePeek(PIDQueueHandle,5);
			
		JoyKey_Event=osMessagePeek(JoyKeyQueueHandle,5);	
			
  if( ((PID_Typedef *)(PID_Event.value.p))->Function_Code == 'S')
		{
			Function_Code='S';
		}
		else
		{
   Function_Code=((JoyKey_Typedef *)(JoyKey_Event.value.p))->Function_Code;
		}	
			sprintf( str_buf1,"Function_Code: %c",  Function_Code);			   
			OLED_ShowString(0,0,(uint8_t *)str_buf1,12);		
			
			switch( Function_Code )
			{
				case 'S':
					sprintf( str_buf3,"P:%02.2f", ((PID_Typedef *)(PID_Event.value.p))->P  );			   
					OLED_ShowString(0,1,(uint8_t *)str_buf3,12);		

					sprintf( str_buf3,"I:%02.2f",((PID_Typedef *)(PID_Event.value.p))->I 		);	   
					OLED_ShowString(0,2,(uint8_t *)str_buf3,12);					

					sprintf( str_buf3,"D:%02.2f",((PID_Typedef *)(PID_Event.value.p))->D  );			   
					OLED_ShowString(0,3,(uint8_t *)str_buf3,12);
					sprintf( str_buf3,"index:%1d",
																																									((PID_Typedef *)(PID_Event.value.p))->Select_Index );			   
					OLED_ShowString(0,4,(uint8_t *)str_buf3,12);					
			break;
				
			case 'C':
						sprintf( str_buf1,"LX:%4d LY:%5d",
																																									((JoyKey_Typedef *)(JoyKey_Event.value.p))->LX,
																																									((JoyKey_Typedef *)(JoyKey_Event.value.p))->LY );			   
					OLED_ShowString(0,1,(uint8_t *)str_buf1,12);	

					sprintf( str_buf2,"RX:%3d RY:%3d",
																																									((JoyKey_Typedef *)(JoyKey_Event.value.p))->RX,
																																									((JoyKey_Typedef *)(JoyKey_Event.value.p))->RY);			   
					OLED_ShowString(0,2,(uint8_t *)str_buf2,12);					
			break;				
			
				default :
//					OLED_Clear();
					break;
			}
/*
	  Gesture_Event=osMessageGet(GesturePIDHandle,10);
			sprintf( str_buf,"%3.1f %3.1f %3.1f",
			                                    ((Gesture_Typedef *)(Gesture_Event.value.p))->Pitch,
																																							((Gesture_Typedef *)(Gesture_Event.value.p))->Roll,
																																							((Gesture_Typedef *)(Gesture_Event.value.p))->Yaw );			   
   OLED_ShowString(0,6,(uint8_t *)str_buf,12);					
*/			

   
			osDelay(20);
  }
  /* USER CODE END StartOLED_Task */
}

/* USER CODE BEGIN Header_StartJoyKeyTask */
/**
* @brief Function implementing the JoyKeyTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartJoyKeyTask */
void StartJoyKeyTask(void const * argument)
{
  /* USER CODE BEGIN StartJoyKeyTask */
	  JoyKey_Typedef JoyKey;
	
	  Key_Statu_Typedef Key;
	
	 __IO uint8_t Function_code='C'; 	 //RC 功能码(注意大写)   C:控制  R:请求数据  S:设置PID H:hover mode  D:dead mode

	 uint32_t ADC[4];
	 const int8_t ADC_adj[4]={0,0,0,0};
		
	 Key_init(&Key);
  /* Infinite loop */
  for(;;)
  { 
			 HAL_ADC_Start_DMA(&hadc1,ADC,4); 
    JoyKey.LX=(int16_t)(ADC[0]-4095/2)/11.375f+ADC_adj[0];  //YAW       179~-180
			 JoyKey.LY=(int16_t)((4095-ADC[1])/2-4095/4)+ADC_adj[1];   //Throttle -1000 ~ +1000
			 JoyKey.RX=(int8_t) -(ADC[2]/68-30+ADC_adj[2]);       //Roll  
			 JoyKey.RY=(int8_t) (ADC[3]/68-30+ADC_adj[3]);       //Pitch 	       -30~30
			 HAL_ADC_Stop_DMA(&hadc1);
	  
			 if(JoyKey.LX <20 && JoyKey.LX > -20 )
				{
				 JoyKey.LX=0;
				}		
			 if(JoyKey.RX <3 && JoyKey.RX > -3 )
				{
				 JoyKey.RX=0;
				}				
			 if(JoyKey.RY <3 && JoyKey.RY > -3 )
				{
				 JoyKey.RY=0;
				}						
				
			 Scan_Key(&Key);
			
			 JoyKey.key_Count=Key.Count;
			 JoyKey.Key_Left_Flag=Key.Key_Left_Flag;
			 JoyKey.Key_Right_Flag=Key.Key_Right_Flag;
			 
				if(JoyKey.Key_Left_Flag == 1 && JoyKey.Key_Right_Flag != 1 && JoyKey.key_Count >5 && JoyKey.key_Count <10)//防止同时按下左右两键
					{
						 switch( Function_code )
		    	{	
								case 'D':
								 Function_code='C';
						   break;
								case 'C':
         Function_code='D';
						   break;
							default:
         Function_code='D';
						   break;
							}
					}
				JoyKey.Function_Code=Function_code;
				osMessagePut(JoyKeyQueueHandle,(uint32_t)&JoyKey,0);//传送队列的地址

    osDelay(20);
  }
  /* USER CODE END StartJoyKeyTask */
}

/* USER CODE BEGIN Header_StartPIDTask */
/**
* @brief Function implementing the PIDTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartPIDTask */
void StartPIDTask(void const * argument)
{
  /* USER CODE BEGIN StartPIDTask */
  osEvent JoyKey_Event;
	 PID_Typedef PID_Set;
  /* Infinite loop */
  for(;;)
  {
			JoyKey_Event=osMessagePeek(JoyKeyQueueHandle,0);	
			
			if( ((JoyKey_Typedef *)(JoyKey_Event.value.p))->Key_Right_Flag == 1 && 
				   ((JoyKey_Typedef *)(JoyKey_Event.value.p))->Key_Left_Flag != 1 && 
			    ((JoyKey_Typedef *)(JoyKey_Event.value.p))->key_Count > 10 &&
       ((JoyKey_Typedef *)(JoyKey_Event.value.p))->key_Count < 15			)//防止同时按下左右两键
					{
						 switch( PID_Set.Function_Code )
		    	{	
								
								case 'C':
//								 Function_code='S';
//								 osThreadResume(PIDTaskHandle);
								 PID_Set.Function_Code='S';
						   break;
								case 'S':
//         Function_code='C';
//								 osThreadSuspend(PIDTaskHandle);
								 PID_Set.Function_Code='C';
						   break;
								case 'D':
									PID_Set.Function_Code='S';
//									Function_code='S';
						   break;
							default:
								PID_Set.Function_Code='S';
//         Function_code='S';
						   break;
							
							}
					}	
			if( PID_Set.Function_Code == 'S' )
			{
					switch(PID_Set.Select_Index)
						{
							case INDEX_PID:
						  if(((JoyKey_Typedef *)(JoyKey_Event.value.p))->RY<-29)
								{
								 PID_Set.P+=1.0f;
								}
						  if(((JoyKey_Typedef *)(JoyKey_Event.value.p))->RY>29)
								{
								 PID_Set.P-=1.0f;
								}										
						  if(((JoyKey_Typedef *)(JoyKey_Event.value.p))->LY<-990)
								{
								 PID_Set.P+=0.01f;
								}
						  if(((JoyKey_Typedef *)(JoyKey_Event.value.p))->LY>990)
								{
								 PID_Set.P-=0.01f;
								}
								if(PID_Set.P<=0)
								{
								 PID_Set.P=0.0f;
								}					
								break;
								
							case INDEX_PID+1:
						  if(((JoyKey_Typedef *)(JoyKey_Event.value.p))->RY<-29)
								{
								 PID_Set.I+=1.0f;
								}
						  if(((JoyKey_Typedef *)(JoyKey_Event.value.p))->RY>29)
								{
								 PID_Set.I-=1.0f;
								}										
						  if(((JoyKey_Typedef *)(JoyKey_Event.value.p))->LY<-990)
								{
								 PID_Set.I+=0.01f;
								}
						  if(((JoyKey_Typedef *)(JoyKey_Event.value.p))->LY>990)
								{
								 PID_Set.I-=0.01f;
								}
								if(PID_Set.I<=0)
								{
								 PID_Set.I=0.0f;
								}	
								break;		
								
							case INDEX_PID+2:
						  if(((JoyKey_Typedef *)(JoyKey_Event.value.p))->RY<-29)
								{
								 PID_Set.D+=1.0f;
								}
						  if(((JoyKey_Typedef *)(JoyKey_Event.value.p))->RY>29)
								{
								 PID_Set.D-=1.0f;
								}											
						  if(((JoyKey_Typedef *)(JoyKey_Event.value.p))->LY<-990)
								{
								 PID_Set.D+=0.01f;
								}
						  if(((JoyKey_Typedef *)(JoyKey_Event.value.p))->LY>990)
								{
								 PID_Set.D-=0.01f;
								}
								if(PID_Set.D<=0)
								{
								 PID_Set.D=0.0f;
								}	
								break;
							
							default:
       break;								
						}
				 	if( ((JoyKey_Typedef *)(JoyKey_Event.value.p))->LX < -170 )
      {
							PID_Set.Select_Index++;
						}			
					 if( ((JoyKey_Typedef *)(JoyKey_Event.value.p))->LX > 170 )
						{
						 PID_Set.Select_Index--;
						}
						if(PID_Set.Select_Index>=INDEX_PID+2)
						{
						PID_Set.Select_Index=INDEX_PID+2;
						}	
						if(PID_Set.Select_Index<=INDEX_PID)
						{
						PID_Set.Select_Index=INDEX_PID;
						}		
						osMessagePut(PIDQueueHandle,(uint32_t)&PID_Set, 0);
					}
    osDelay(20);
  }
  /* USER CODE END StartPIDTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
