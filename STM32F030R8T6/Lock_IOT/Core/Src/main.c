/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "dma.h"
#include "rtc.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
//软件
#include "systemstatus.h"
#include "FaceReco.h"

//外设
#include "wtn6040.h"
#include "wifi.h"
#include "motor.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct
{
  uint8_t IR1_State;
  uint8_t IR2_State;
} IR_StateTypedef;
IR_StateTypedef IR_State;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
uint8_t WIFI_Data=0; 
uint8_t UC_Data=0;

uint8_t DoorCMD = 0;  //全局变量 
uint8_t DoorStatus = DOOR_OPEN;
int     VisitorNum;
	
uint32_t ADC_Data[2];
uint32_t Tim_IT_Count=0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_NVIC_Init(void);
/* USER CODE BEGIN PFP */
void StandbyMode(void);
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
  MX_USART2_UART_Init();
  MX_USART1_UART_Init();
  MX_RTC_Init();
  MX_ADC_Init();
  MX_TIM17_Init();

  /* Initialize interrupts */
  MX_NVIC_Init();
  /* USER CODE BEGIN 2 */
  EX_VDD_ON;
		
  IR1_ON;
  IR2_ON;
		
  SendMessage(0);
  HAL_Delay(100);
  
  LED0_ON;
  LED1_ON;

//		SendMessage(4);		
		
		HAL_ADCEx_Calibration_Start(&hadc);
		
		
		
		HAL_UART_Receive_IT(&hWIFI, &WIFI_Data, 1); 
		HAL_UART_Receive_IT(&hUCUART, &UC_Data, 1);
		
		wifi_protocol_init();
		mcu_reset_wifi();
  mcu_get_reset_wifi_flag();
  mcu_start_wifitest();
//		product_info_update();

  HAL_TIM_Base_Start_IT(&htim17);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    IR_State.IR1_State = IR1_IN;
    IR_State.IR2_State = IR2_IN;

    if (DoorCMD == FACE_CHECKED && FaceCheck.Open == 0)  //人脸检测
    {
      FaceCMDSemd();
      SendMessage(15);
      DoorCMD = DOOR_WAIT;
					
				 	FaceCheck.Open = 1;
					 FaceReco_Init();
    }
    else if (DoorCMD == DOOR_OPEN) //开门
    {
      SendMessage(16);
					 LED0_ON;
      DoorOpen();
					
      DoorCMD = DOOR_WAIT;
      LED0_OFF;
      SendMessage(7);
					
      DoorStatus = DOOR_OPEN;
    }
    else if (DoorCMD == DOOR_CLOSE) //关门
    {
      LED0_ON;
      DoorClose();
      DoorCMD = DOOR_WAIT;
      LED0_OFF;
      SendMessage(8);
      DoorStatus = DOOR_CLOSE;
    }
    else if (DoorCMD == BAT_WARN) //发出低电警告  约一分钟一次警告
    {
      LED0_ON;
      LED1_OFF;
      SendMessage(1);
      DoorCMD = DOOR_WAIT;
    }
    else if (DoorCMD == DOOR_SLEEP) //睡眠模式
    {
      EX_VDD_OFF;
      LED1_OFF;
      LED0_OFF;
      DoorCMD = DOOR_WAIT;
      StandbyMode();
    }
    else if (DoorCMD == NET_DISCONC) //无网络
    {
      SendMessage(13);
      DoorCMD = DOOR_WAIT;
    }
    else if (DoorCMD == NET_CONC) //有网络
    {
      SendMessage(12);
      DoorCMD = DOOR_WAIT;
    }
    else
    {
      DoorCMD = DOOR_WAIT;
    }
				
						
				  if(FaceCheck.Open == 1)
						{
								FaceCheck.VisitorNum = FaceDataCheck();
								if (FaceCheck.VisitorNum >= -1)
								{
										FaceCheck.Open = 0;
										if (FaceCheck.VisitorNum >= 0)
										{
												DoorCMD = DOOR_OPEN;
										}
								}
						}	 
				
						
						if(Tim_IT_Count>10)
						{
							
								Tim_IT_Count=0;
							 B_DET_ON;
        HAL_ADC_Start_DMA(&hadc, ADC_Data, 2);
								Battery.AD = ADC_Data[1] * 2475 / 80;
	       HAL_ADC_Stop_DMA(&hadc);
							 B_DET_OFF;
								if( Battery.AD < 6000 )
								{
										Battery.WarnCount++;
										if(Battery.WarnCount > 6)//100 * 10s
										{
												DoorCMD = BAT_WARN;
												if( Battery.AD > 5500 )
												{
														Battery.Status = medium;
												}
												else if( Battery.AD > 5000 )
												{
														Battery.Status = low;
												}
												else
												{
														Battery.Status = poweroff;
												}
												Battery.WarnCount = 0;
										}
											
								}
								else
								{
										Battery.WarnCount = 0;
										Battery.Status = high;
								}					
						}
				 all_data_update();  //涂鸦云数据点上报
     wifi_uart_service();
		
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

  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_LSI
                              |RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL6;
  RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV1;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1|RCC_PERIPHCLK_RTC;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_HSI;
  PeriphClkInit.RTCClockSelection = RCC_RTCCLKSOURCE_LSI;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
  /** Enables the Clock Security System 
  */
  HAL_RCC_EnableCSS();
}

/**
  * @brief NVIC Configuration.
  * @retval None
  */
static void MX_NVIC_Init(void)
{
  /* EXTI0_1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(EXTI0_1_IRQn, 2, 0);
  HAL_NVIC_EnableIRQ(EXTI0_1_IRQn);
  /* EXTI4_15_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(EXTI4_15_IRQn, 2, 0);
  HAL_NVIC_EnableIRQ(EXTI4_15_IRQn);
  /* TIM17_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(TIM17_IRQn, 3, 0);
  HAL_NVIC_EnableIRQ(TIM17_IRQn);
  /* DMA1_Channel2_3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel2_3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel2_3_IRQn);
  /* USART1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(USART1_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(USART1_IRQn);
  /* USART2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(USART2_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(USART2_IRQn);
}

/* USER CODE BEGIN 4 */
void StandbyMode(void)
{
  __HAL_RCC_PWR_CLK_ENABLE();
  HAL_PWR_EnableBkUpAccess();
  __HAL_RCC_BACKUPRESET_FORCE();
  __HAL_RCC_BACKUPRESET_RELEASE();
  HAL_PWR_DisableWakeUpPin(PWR_WAKEUP_PIN1);
  __HAL_PWR_CLEAR_FLAG(PWR_FLAG_WU);
  HAL_PWR_EnableWakeUpPin(PWR_WAKEUP_PIN1);
  HAL_PWR_EnterSTANDBYMode();
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
void assert_failed(char *file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
