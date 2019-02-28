
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2018 STMicroelectronics
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
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f1xx_hal.h"
#include "crc.h"
#include "i2c.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* USER CODE BEGIN Includes */
#include "BSP_NRF24L01.h"
#include "BSP_LED.h"
#include "inv_mpu.h"
#include "BSP_MPU6050.h"
#include "PID_Control.h"
#include "stdlib.h"
#include "string.h"
#include "arm_math.h"
#include "eeprom_emulation.h"
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
typedef __packed struct{
uint8_t  FlyMode;
float  LX;
float  LY;
float  RX;
float  RY;
}sJoykey;

typedef  struct{
float Pitch;
float Roll;
float Yaw;
}sPosture;

typedef __packed struct {	
	uint8_t PID_Code;
	float Roll_P	;
	float Roll_I  ;
	float Roll_D  ;
	float Pitch_P	;
	float Pitch_I  ;
	float Pitch_D  ;	
}sPID;

sJoykey * Joykey_Buf;
sPosture Curent_Posture;
sPID * PID_Set;

int16_t VirtAddVarTab[NB_OF_VAR]={0x1234};

uint8_t  NRF_RX_Buff[45];
uint8_t  NRF_TX_Buff[45];

uint8_t cmp_p[9]={0};
int16_t Gyro[3]; 


uint32_t CRC_Buf[10];
uint32_t CRC_Value;


uint16_t EE_Data[24];


uint8_t Timer4_IT_PID_Flag=0;
uint16_t Request_Data_Flag=0;//0:Do not request 1:do request


arm_pid_instance_f32 arm_Pitch;
arm_pid_instance_f32 arm_Roll;
float arm_Pitch_out,arm_Roll_out;
int16_t M1,M2,M3,M4;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_NVIC_Init(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  *
  * @retval None
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
 
  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_I2C2_Init();
  MX_SPI1_Init();
  MX_TIM1_Init();
  MX_USART1_UART_Init();
  MX_CRC_Init();
  MX_TIM4_Init();

  /* Initialize interrupts */
  MX_NVIC_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_Base_Start(&htim1);
  HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_4);

	 Joykey_Buf=(sJoykey*)malloc(sizeof(sJoykey));
		PID_Set=(sPID*)malloc(sizeof(sPID));
		memset(NRF_RX_Buff,0,30);
		memset(NRF_TX_Buff,0,30);
		memset(CRC_Buf,0,40);
		
//		HAL_FLASH_Unlock();
//		EE_Init();
//		HAL_FLASH_Lock();
		
		while(NRF24L01_Check())
		{
		;
		}
		Blink_LED(25,10);	
		NRF24L01_RX_Mode();
		mpu_dmp_init();
//		PID_init(0,0,0,0,0,0);
		
		arm_Roll.Kp=0;
		arm_Roll.Ki=0;
		arm_Roll.Kd=0;
		arm_pid_init_f32(&arm_Roll,0);
		arm_Pitch.Kp=1.80f;
		arm_Pitch.Ki=0.02f;
		arm_Pitch.Kd=3.10f;
		arm_pid_init_f32(&arm_Pitch,0);		
		
  NRF24L01_RX_Mode();	  
		HAL_TIM_Base_Start_IT(&htim4);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */
		 
					NRF24L01_RxPacket(NRF_RX_Buff);
			
					if(NRF_RX_Buff[0]==0x05)//PID Set Mode
						{ 
									memcpy(CRC_Buf,NRF_RX_Buff,25);				
									CRC_Value = HAL_CRC_Calculate(&hcrc,CRC_Buf,10);			
									if( (uint8_t)CRC_Value==NRF_RX_Buff[25])
											{
												
												memcpy(PID_Set,NRF_RX_Buff,25);				
//											PID_init(PID_Set->Roll_P,PID_Set->Roll_I,PID_Set->Roll_D,
//																				PID_Set->Pitch_P,PID_Set->Pitch_I,PID_Set->Pitch_D);
												arm_Roll.Kp=PID_Set->Roll_P;
												arm_Roll.Ki=PID_Set->Roll_I;
												arm_Roll.Kd=PID_Set->Roll_D;
												arm_pid_init_f32(&arm_Roll,0);
												arm_Pitch.Kp=PID_Set->Pitch_P;
												arm_Pitch.Ki=PID_Set->Pitch_I;
												arm_Pitch.Kd=PID_Set->Pitch_P;
												arm_pid_init_f32(&arm_Pitch,0);	
												
												Blink_LED(5,1000);
											}
											
										else
											{
//													PID_init(0,0,0,0,0,0);
															arm_Roll.Kp=0;
															arm_Roll.Ki=0;
															arm_Roll.Kd=0;
															arm_pid_init_f32(&arm_Roll,0);
															arm_Pitch.Kp=0;
															arm_Pitch.Ki=0;
															arm_Pitch.Kd=0;
															arm_pid_init_f32(&arm_Pitch,0);		
											}
						}
				
						else 
						{		
								if(Timer4_IT_PID_Flag>=10)//10ms
									{
													Timer4_IT_PID_Flag=0;		
													if(NRF_RX_Buff[0]==0x01)
													{
																memcpy(CRC_Buf,NRF_RX_Buff,17);
																CRC_Value = HAL_CRC_Calculate(&hcrc,CRC_Buf,5);
																if((uint8_t)CRC_Value==NRF_RX_Buff[17])
																{
																	memcpy(Joykey_Buf,NRF_RX_Buff,sizeof(sJoykey));
																}
																
																else
																{
																	Joykey_Buf->LX=0;       //YAW    
																	Joykey_Buf->LY=0;       //Throttle 
																	Joykey_Buf->RX=0;       //Roll  
																	Joykey_Buf->RY=0;       //Pitch 
																}         										
													}
													
													else
													{
														memset(NRF_RX_Buff,0,30);
													}	
												
               mpu_dmp_get_data(&Curent_Posture.Pitch,&Curent_Posture.Roll,&Curent_Posture.Yaw);
													  if( Curent_Posture.Pitch == 0 || Curent_Posture.Roll==0 || Curent_Posture.Yaw ==0 )
															{
																Blink_LED(20,3);
																HAL_NVIC_SystemReset();
															}
															
													if(Joykey_Buf->LY>200)
													{
														
														
													arm_Pitch_out = arm_pid_f32(&arm_Pitch,Curent_Posture.Pitch);
													arm_Roll_out  = arm_pid_f32(&arm_Roll ,Curent_Posture.Roll);
														
													M1=(int16_t) ( Joykey_Buf->LY - arm_Roll_out - arm_Pitch_out );
													M2=(int16_t) ( Joykey_Buf->LY + arm_Roll_out - arm_Pitch_out );
													M3=(int16_t) ( Joykey_Buf->LY + arm_Roll_out + arm_Pitch_out );
													M4=(int16_t) ( Joykey_Buf->LY - arm_Roll_out + arm_Pitch_out );
													if(M1>=2000)
														{
															M1=2000;
														}
													if(M2>=2000)
														{
															M2=2000;
														}
													if(M3>=2000)
														{
															M3=2000;
														}
													if(M4>=2000)
														{
															M4=2000;
														}
													if(M1<=0)
														{
															M1=0;
														}
													if(M2<=0)
														{
															M2=0;
														}
													if(M3<=0)
														{
															M3=0;
														}
													if(M4<=0)
														{
															M4=0;
														}												
													PWM_Set(  &htim1,M1,M2,M3,M4 );

													}
													
													else
													{
													PWM_Set(  &htim1,0,0,0,0 );
													}
//													MPU6050ReadGyro(Gyro);
//													PID_CONTROL( Curent_Posture.Pitch, Curent_Posture.Yaw, Curent_Posture.Roll,
//																			-0,0,0,
//																		(float)(Gyro[0])*Gyro_Gr,(float)Gyro_Gr*(Gyro[1]),(float)( Gyro[2]),
//																			Joykey_Buf->LY ,Joykey_Buf->FlyMode);															
														
														/*		PID_CONTROL(  Curent_Posture.Pitch, Curent_Posture.Yaw, Curent_Posture.Roll,
																												-J_Buff.RX,J_Buff.RY,J_Buff.LX,
																													(float) Gyro[0]*Gyro_Gr,(float)Gyro_Gr*Gyro[1],(float)Gyro[2],
																													J_Buff.LY,J_Buff.FlyMode);	
															ANO_DT_Send_Status(Curent_Posture.Roll,Curent_Posture.Pitch,Curent_Posture.Yaw,
																																		0,0,0);								
														*/				
									}		

						}
	
		

  }
  /* USER CODE END 3 */

}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
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
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Enables the Clock Security System 
    */
  HAL_RCC_EnableCSS();

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/**
  * @brief NVIC Configuration.
  * @retval None
  */
static void MX_NVIC_Init(void)
{
  /* TIM4_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(TIM4_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(TIM4_IRQn);
}

/* USER CODE BEGIN 4 */ 
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
 
	Timer4_IT_PID_Flag++;

}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  file: The file name as string.
  * @param  line: The line in file as a number.
  * @retval None
  */
void _Error_Handler(char *file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
