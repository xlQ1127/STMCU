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
#include "crc.h"

#include "string.h"
#include "arm_math.h"

#include "inv_mpu.h"
#include "BSP_MPU6050.h"
#include "BSP_Motor.h"
#include "BSP_LED.h"
#include "BSP_NRF24L01.h"
#include "BSP_Motor.h"

#include "printf_scanf.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define NRF_TX_MAXRETRY 5
#define NRF_RX_MAXRETRY 1
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
typedef struct{
	float Pitch;
	float Roll;
	float Yaw;
	float Gyro_x;
	float Gyro_y;
	float Gyro_z;
}Gesture_Typedef;   //3*4B   36B

typedef __packed struct{
	uint8_t Function_Code;
	float P	 ;       //4B
	float I  ;
	float D  ;
 uint8_t index;	
}PID_Typedef;  

typedef __packed struct{
uint8_t  Function_Code;
uint8_t	 Key_Left_Flag;
uint8_t	 Key_Right_Flag;
uint8_t  Key_Count;	    //4B
int16_t  LX;
int16_t  LY;            //4B
int8_t   RX;
int8_t   RY;             //2B
uint8_t  Heart_Beat;	
}Joykey_Typedef;  // 10B

/* USER CODE END Variables */
osThreadId defaultTaskHandle;
osThreadId NRFTaskHandle;
osThreadId FCTaskHandle;
osMessageQId GestureQueueHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
   
/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void const * argument);
void StartNRFTask(void const * argument);
void StartFCTask(void const * argument);

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

  /* definition and creation of NRFTask */
  osThreadDef(NRFTask, StartNRFTask, osPriorityRealtime, 0, 512);
  NRFTaskHandle = osThreadCreate(osThread(NRFTask), NULL);

  /* definition and creation of FCTask */
  osThreadDef(FCTask, StartFCTask, osPriorityHigh, 0, 256);
  FCTaskHandle = osThreadCreate(osThread(FCTask), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* Create the queue(s) */
  /* definition and creation of GestureQueue */
  osMessageQDef(GestureQueue, 8, Gesture_Typedef);
  GestureQueueHandle = osMessageCreate(osMessageQ(GestureQueue), NULL);

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
				LED1_OFF;
		  LED2_OFF;		
		  HAL_Delay(50);
				LED1_ON;
			 LED2_ON;
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
	 osEvent Geture_Event;
	 Joykey_Typedef JoyKey;
	 PID_Typedef PID_Set;
	 uint8_t PID_Set_Index=0;
	
	 uint8_t Heart_Beat_Last=0;
	
	 uint8_t NRF_Mode=2; //1:TX 2:RX
	
		uint8_t NRF_Tx_buf[32];
  uint8_t NRF_Rx_buf[32];
		uint8_t NRF_state=0;
  
	 uint32_t CRC_Buf[32];
	 uint8_t CRC_Value=0;
	__IO uint8_t Function_Code='C';
	
	 uint8_t NRF_Tx_index=0;

		PID_Parameter_Typedef Outer_Pitch_PID_Par;
		PID_Parameter_Typedef Inner_Pitch_PID_Par;
		
		PID_Parameter_Typedef Outer_Roll_PID_Par;
		PID_Parameter_Typedef Inner_Roll_PID_Par;
		
		PID_Parameter_Typedef Yaw_PID_Par;	
	 
		
		Control_PID_Init(&Outer_Roll_PID_Par);
		Control_PID_Init(&Inner_Roll_PID_Par);
		Control_PID_Init(&Outer_Pitch_PID_Par);
		Control_PID_Init(&Inner_Pitch_PID_Par);	
		Control_PID_Init(&Yaw_PID_Par);
		
	 NRF24L01_Init(40,NRF_Mode);

  /* Infinite loop */
  for(;;)
  { 
			 switch(NRF_Mode)
				{
					case 1:
								NRF24L01_TX_Mode();
							 NRF_Mode=0;
		   			NRF_state=1;
					break;
					case 2:
								NRF24L01_RX_Mode();
							 NRF_Mode=0;
					   NRF_state=0;
					break;
					default:
						break;		
				}
				
				if(NRF_state)
				{
					NRF24L01_TxPacket(NRF_Tx_buf);
				}
				else
				{
					NRF24L01_RxPacket(NRF_Rx_buf);	
				}
				
				Function_Code=NRF_Rx_buf[0];
				memcpy(CRC_Buf,NRF_Rx_buf,31);
				CRC_Value=	(uint8_t) HAL_CRC_Calculate(&hcrc,CRC_Buf,31);	
    switch(Function_Code)
				{			
					case 'D':
							PWM_Set(&htim2,0,0,0,0);
						break;		
					
					case 'C':
						memcpy(&JoyKey,NRF_Rx_buf,sizeof(Joykey_Typedef));
					 Geture_Event=osMessageGet(GestureQueueHandle,5);
						if( (JoyKey.Heart_Beat != Heart_Beat_Last) && (CRC_Value == NRF_Rx_buf[31]) )         
						{ 	 
//							 PWM_Set(&htim2,JoyKey.LY,JoyKey.LY,JoyKey.LY,JoyKey.LY  
//												taskENTER_CRITICAL();				
//							
																Control_Function( JoyKey.LY,JoyKey.RY,JoyKey.RX,
																																		((Gesture_Typedef *)(Geture_Event.value.p))->Pitch,
																																		((Gesture_Typedef *)(Geture_Event.value.p))->Roll,
																																		((Gesture_Typedef *)(Geture_Event.value.p))->Yaw,
																																		((Gesture_Typedef *)(Geture_Event.value.p))->Gyro_x,
																																		((Gesture_Typedef *)(Geture_Event.value.p))->Gyro_y,
																																		((Gesture_Typedef *)(Geture_Event.value.p))->Gyro_z,
																																			&Outer_Pitch_PID_Par,&Inner_Pitch_PID_Par,
																																			&Outer_Roll_PID_Par,&Inner_Roll_PID_Par,
																																			&Yaw_PID_Par );
																ANO_DT_Send_Status( ((Gesture_Typedef *)(Geture_Event.value.p))->Roll,((Gesture_Typedef *)(Geture_Event.value.p))->Pitch, 
				                    ((Gesture_Typedef *)(Geture_Event.value.p))->Yaw , 0 ,0 ,0 );
            				ANO_DT_Send_Senser(0,0,0,((Gesture_Typedef *)(Geture_Event.value.p))->Gyro_x, 
				                    ((Gesture_Typedef *)(Geture_Event.value.p))->Gyro_y, ((Gesture_Typedef *)(Geture_Event.value.p))->Gyro_z,
				                   0,0,0,0 );																				
//													taskEXIT_CRITICAL();	
							     	Heart_Beat_Last=JoyKey.Heart_Beat; //更新通信心跳 
//												}
						
						}	
						break;
						
						case 'S':
						memcpy(&PID_Set,NRF_Rx_buf,sizeof(PID_Typedef));
						if( PID_Set.index >=1 && PID_Set.index<= 3 )
						{
						 PID_Set_Index=1;//选择Roll内环PID
						}
						else if ( PID_Set.index >=4 && PID_Set.index<= 6 )
						{
							PID_Set_Index=2;//选择Roll外环PID					
						}				
						else if ( PID_Set.index >=7 && PID_Set.index<= 9 )
						{
							PID_Set_Index=3;//选择Pitch内环PID					
						}			
						else if ( PID_Set.index >=10 && PID_Set.index<= 12 )
						{
							PID_Set_Index=4;//选择Pitch外环PID					
						}												
						else
						{
							PID_Set_Index=0;
						}	
						
						switch(PID_Set_Index)
						{
								case 1: 
													Control_PID_Set( &Inner_Roll_PID_Par,PID_Set.P,PID_Set.I,PID_Set.D );
								break;
								case 2: 
													Control_PID_Set( &Outer_Roll_PID_Par,PID_Set.P,PID_Set.I,PID_Set.D );
								break;
								case 3: 
													Control_PID_Set( &Inner_Pitch_PID_Par,PID_Set.P,PID_Set.I,PID_Set.D );
								break;	
								case 4: 
													Control_PID_Set( &Outer_Pitch_PID_Par,PID_Set.P,PID_Set.I,PID_Set.D );
								break;									
        default:
        break;									
						}

						break;		
						
					case 'R':
						memcpy( NRF_Tx_buf , Geture_Event.value.p,sizeof(Gesture_Typedef) );
						for(NRF_Tx_index=0;NRF_Tx_index<5;NRF_Tx_index++)
						{
						 NRF24L01_TxPacket(NRF_Tx_buf);
						}
						break;	
									
					default:
      break;						
				}


//				
//				ANO_DT_Send_PID(1,Inner_Roll_PID_Par.Kp,Inner_Roll_PID_Par.Ki,Inner_Roll_PID_Par.Kd,0,0,0,0,0,0);													                  
													                  
    osDelay(10);
		}
  /* USER CODE END StartNRFTask */
}

/* USER CODE BEGIN Header_StartFCTask */
/**
* @brief Function implementing the FCTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartFCTask */
void StartFCTask(void const * argument)
{
  /* USER CODE BEGIN StartFCTask */
  float Pitch=0.0f,Roll=0.0f,Yaw=0.0f;
//	 float GX,GY,GZ;
//	 float GX_Pre,GY_Pre,GZ_Pre;
	
	 short Gyro[3];
	 Gesture_Typedef Gesture;
	  
	 while( Pitch<=0.1f && Pitch>=-0.1 &&
         Roll<=0.1f && Roll>=-0.1f &&
         Yaw<=0.1f &&  Yaw>=-0.1f		) 
	 {
		  mpu_dmp_get_data(&Pitch,&Roll,&Yaw);
		}
		MPU6050ReadGyro(Gyro);
		
  /* Infinite loop */
  for(;;)
  {

			mpu_dmp_get_data(&Pitch,&Roll,&Yaw);
			MPU6050ReadGyro(Gyro);
			
			Gesture.Pitch=Pitch;
			Gesture.Roll=Roll;
			Gesture.Yaw=Yaw;
			
//			GX=Gyro[0]/16.04f;
//			GY=Gyro[1]/16.04f;
//			GZ=Gyro[2]/16.04f;
			
//			Gesture.Gyro_x = LPF_1st(GX_Pre,GX,0.386f);
//			Gesture.Gyro_y =	LPF_1st(GY_Pre,GY,0.386f);
//			Gesture.Gyro_z =	LPF_1st(GZ_Pre,GZ,0.386f);
			
//			GX_Pre=GX;
//			GY_Pre=GY;
//			GZ_Pre=GZ;
			
			
			
			
//无低通滤波 一阶
			Gesture.Gyro_x=(float)(Gyro[0]/16.04f);
			Gesture.Gyro_y=(float)(Gyro[1]/16.04f);
			Gesture.Gyro_z=(float)(Gyro[2]/16.04f);
			
			osMessagePut(GestureQueueHandle,(uint32_t)&Gesture,5);
   osDelay(5);
  }
  /* USER CODE END StartFCTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
     
/* USER CODE END Application */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
