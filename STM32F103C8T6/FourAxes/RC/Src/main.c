/* USER CODE BEGIN Header */
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
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "crc.h"
#include "dma.h"
#include "i2c.h"
#include "spi.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdlib.h"
#include "string.h"
#include "BSP_LED.h"
#include "BSP_NRF24L01.h"
#include "BSP_OLED.h"
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

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
typedef __packed struct{	
	uint8_t PID_Code;
	float Roll_P	;
	float Roll_I  ;
	float Roll_D  ;
	float Pitch_P	;
	float Pitch_I  ;
	float Pitch_D  ;	
}sPID;

typedef struct{
float Pitch;
float Roll;
float Yaw;
}sPosture;

typedef __packed struct{
uint8_t  FlyMode;
float  LX;
float  LY;
float  RX;
float  RY;
}sJoykey;

sPID* PID_Set;
sPID     PID_Setted;
sPosture* Posture;

sJoykey  * JoyKey_TX;

uint8_t  Control=0;      //0: NO Control and Do Nothing; 1:Start Fly and RemoteControl; 
	                        //2:Stop Fly and Stay where it were ; 3:Stop ALL Motor;

uint8_t SET_PID_Flag=0;  //0:no need set pid 1:need set pid
uint8_t Shutdown_Flag=0,Flying_Flag=0,SET_Flag_4OLED=0;//For OLED 
uint8_t UART_IT_Flag=0;

uint32_t   ADC_Data[4]={0,0,0,0};
const float ADC_adj[4]={2.5f,-201.0f,0,0};

uint8_t UART_RX_Buf[86];
uint8_t UART_TX_Buf[64];

uint8_t NRF_TX_Buf[30];
uint8_t NRF_RX_Buf[13];
uint32_t CRCBuf[10];
uint32_t uwCRCValue;

char strtmp[10];


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_NVIC_Init(void);
/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

sPID Str2PID(char * Str_buf);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

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
  MX_DMA_Init();
  MX_SPI2_Init();
  MX_ADC1_Init();
  MX_I2C1_Init();
  MX_CRC_Init();
  MX_USART1_UART_Init();

  /* Initialize interrupts */
  MX_NVIC_Init();
  /* USER CODE BEGIN 2 */
  while( NRF24L01_Check() ) ;
		NRF24L01_TX_Mode();
		OLED_Init();
		OLED_Clear();
		HAL_ADCEx_Calibration_Start(&hadc1);
		HAL_ADC_Start_DMA(&hadc1,ADC_Data,4);	
		Blink_LED(50,10);
		HAL_UART_Receive_IT(&huart1,UART_RX_Buf,86);
		
	 JoyKey_TX = (sJoykey *)malloc(sizeof(sJoykey));
		PID_Set = (sPID *)malloc(sizeof(sPID));
		Posture = (sPosture *)malloc(sizeof(sPosture));
		OLED_ShowString(15,4,(uint8_t *)"Initiate OK",12);
		
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

					if(SET_PID_Flag==1 && UART_IT_Flag==1)
					{
									UART_IT_Flag=0;									
									if(SET_Flag_4OLED==0)
									{
										OLED_Clear();
										OLED_ShowString(0,0,(uint8_t *)"Setting PID.....",12);
										PID_Set->PID_Code =  0x95; //Setting PID mode code
									}
							  *PID_Set=Str2PID( (char *)UART_RX_Buf);
									
									memcpy(NRF_TX_Buf,PID_Set,25);
									
         memcpy(CRCBuf,NRF_TX_Buf,25);		
									
									uwCRCValue = HAL_CRC_Calculate(&hcrc, CRCBuf, 10);
									NRF_TX_Buf[25] =		(uint8_t)uwCRCValue;			
									
									if( NRF24L01_TxPacket( NRF_TX_Buf )!=TX_OK)  Blink_LED(20,3);
									
									sprintf( strtmp,"%3.2f %3.2f",PID_Set->Roll_P,PID_Set->Pitch_P);
									OLED_ShowString(0,1,(uint8_t *)strtmp,12);
									sprintf( strtmp,"%3.2f %3.2f",PID_Set->Roll_I,PID_Set->Pitch_I);		
									OLED_ShowString(0,2,(uint8_t *)strtmp,12);			
									sprintf( strtmp,"%3.2f %3.2f",PID_Set->Roll_D,PID_Set->Pitch_D);						 
									OLED_ShowString(0,3,(uint8_t *)strtmp,12);
					}
					else
					{
								switch(Control)
										{
											case 0: 
													if(Shutdown_Flag==0)
													{
													OLED_Clear();
													OLED_ShowString(0,0,(uint8_t *)"Shutdown Mode",12);
													NRF_TX_Buf[0]=Control;
													Shutdown_Flag=1;	
													}
													if( NRF24L01_TxPacket( (uint8_t *)NRF_TX_Buf )!=TX_OK)  Blink_LED(20,3); 
													break;
										
											case 1: 	
													if(Flying_Flag==0)
													{
													OLED_Clear();
													OLED_ShowString(0,0,(uint8_t *)"Flying   Mode",12);
													JoyKey_TX->FlyMode = Control;
													Flying_Flag=1;
													}				
													JoyKey_TX->LX = ( (int32_t)ADC_Data[0]-4095/2)/11.375f+ADC_adj[0];        //YAW   
													JoyKey_TX->LY = ( (4095-(int32_t)ADC_Data[1])-4095/2)+ADC_adj[1];        //Throttle 
													JoyKey_TX->RX = ( (int32_t)ADC_Data[2]-4095/2)/68+ADC_adj[2];            //Roll  
													JoyKey_TX->RY = ( (int32_t)ADC_Data[3]-4095/2)/68+ADC_adj[3];            //Pitch 			
												
													memcpy(NRF_TX_Buf,JoyKey_TX,17);
												
													memcpy(CRCBuf,NRF_TX_Buf,17);		
													
													uwCRCValue = HAL_CRC_Calculate(&hcrc, CRCBuf, 5);
												 NRF_TX_Buf[17] = uwCRCValue;	
													if( NRF24L01_TxPacket( (uint8_t *)NRF_TX_Buf )!=TX_OK)  Blink_LED(20,3); 										
													break;     
											
											case 2:   //Hover mode
													NRF_TX_Buf[0]=Control;										 
													if( NRF24L01_TxPacket( (uint8_t *)NRF_TX_Buf )!=TX_OK)  Blink_LED(20,3);
													break;
											
											default:
													break;							
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
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
  /**Enables the Clock Security System 
  */
  HAL_RCC_EnableCSS();
}

/**
  * @brief NVIC Configuration.
  * @retval None
  */
static void MX_NVIC_Init(void)
{
  /* EXTI0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(EXTI0_IRQn, 2, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);
  /* EXTI1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(EXTI1_IRQn, 2, 0);
  HAL_NVIC_EnableIRQ(EXTI1_IRQn);
  /* USART1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(USART1_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(USART1_IRQn);
}

/* USER CODE BEGIN 4 */
uint32_t StrToIntFix(char *_pStr, uint8_t _ucLen)
{
	char *p;
	uint32_t ulInt;
	uint8_t i;
	uint8_t ucTemp;

	p = _pStr;
	
	ulInt = 0;
	for (i = 0; i < _ucLen; i++)
	{
		ucTemp = *p;

		if ((ucTemp >= '0') && (ucTemp <= '9'))
		{
			ulInt = ulInt * 10 + (ucTemp - '0');
			p++;
		}
	}

	return ulInt;
}

sPID Str2PID(char * Str_buf)//SET,ROLL_P:1000,ROLL_I:1000,ROLL_D:1000,PITCH_P:1000,PITCH_I:1000,PITCH_D:1000
{
  char *p;
	 sPID P2F;
	 uint16_t uiRoll_P,uiRoll_I,uiRoll_D,
	          uiPitch_P,uiPitch_I,uiPitch_D;
	
	 p=(char*)Str_buf;
	
  p=strstr( p,"SET,"); 
  if(p)
	 { 	 
		 p=strstr(p,"ROLL_P:"); 
		 p+=7;
		 uiRoll_P=StrToIntFix(p,5);
		 p++;
						 
		 p=strstr(p,"ROLL_I:");
   p+=7;
		 uiRoll_I=StrToIntFix(p,5);
		 p++;
									 
		 p=strstr(p,"ROLL_D:");
		 p+=7;
		 uiRoll_D=StrToIntFix(p,5);
		 p++;			 
									 
		 p=strstr(p,"PITCH_P:");
		 p+=8;	 
		 uiPitch_P=StrToIntFix(p,5);
		 p++;
										 
		 p=strstr(p,"PITCH_I:");
		 p+=8;
		 uiPitch_I=StrToIntFix(p,5);
		 p++;
										 
		 p=strstr(p,"PITCH_D:");
		 p+=8;
		 uiPitch_D=StrToIntFix(p,5);
								  
			P2F.Roll_P=(float)uiRoll_P;
			P2F.Roll_I=(float)uiRoll_I;
			P2F.Roll_D=(float)uiRoll_D;
			P2F.Pitch_P=(float)uiPitch_P;
			P2F.Pitch_I=(float)uiPitch_I;
			P2F.Pitch_D=(float)uiPitch_D;
			
			P2F.Roll_P*=0.01f;
   P2F.Roll_I*=0.01f;
	  P2F.Roll_D*=0.01f;
	  P2F.Pitch_P*=0.01f;
	  P2F.Pitch_I*=0.01f;
	  P2F.Pitch_D*=0.01f;
	 }
    
	 else
	 {
	  uiRoll_P=0;
		 uiRoll_I=0;
		 uiRoll_D=0;
   uiPitch_P=0;
		 uiPitch_I=0;
		 uiPitch_D=0; 
	  P2F.Roll_P=0;
   P2F.Roll_I=0;
	  P2F.Roll_D=0;
	  P2F.Pitch_P=0;
	  P2F.Pitch_I=0;
	  P2F.Pitch_D=0;
	 }
	 
return P2F;
}


void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if(GPIO_Pin==GPIO_PIN_0)
	{
		   if( HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_0)==GPIO_PIN_RESET )
		   { 
//			  HAL_Delay(1);
//			  if( HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_1)==GPIO_PIN_SET )
//			    {
//		       OLED_ShowString(0,0,(uint8_t *)"Both",16); 
//			    }
//						else
//						   {					
//						   }
	      SET_PID_Flag=1;//start PID Set		
      }
	}
  if(GPIO_Pin==GPIO_PIN_1)
	   {
					if( HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_1)==GPIO_PIN_SET )
						{
	//					HAL_Delay(1);
	//			    if( HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_0)==GPIO_PIN_RESET )
	//			      {
	//			        OLED_Clear();
	//		         OLED_ShowString(0,0,(uint8_t *)"Both",16); 
	//			      }
	//						else
	//						   {					
	//						   }
									SET_PID_Flag=0;//Stop PID Set
									Control=1;			
							}
				}
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if(huart->Instance==USART1)
	{
  UART_IT_Flag=1;
		HAL_UART_Receive_IT(&huart1,UART_RX_Buf,86);
	}
}



/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
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
void assert_failed(uint8_t *file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
