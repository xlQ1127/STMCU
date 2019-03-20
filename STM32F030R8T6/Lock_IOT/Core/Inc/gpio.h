/**
  ******************************************************************************
  * File Name          : gpio.h
  * Description        : This file contains all the functions prototypes for 
  *                      the gpio  
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __gpio_H
#define __gpio_H
#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* USER CODE BEGIN Private defines */
#define EX_VDD_ON   HAL_GPIO_WritePin(EX_VDD_CON_GPIO_Port, EX_VDD_CON_Pin, GPIO_PIN_RESET)
#define EX_VDD_OFF  HAL_GPIO_WritePin(EX_VDD_CON_GPIO_Port, EX_VDD_CON_Pin, GPIO_PIN_SET)

#define B_DET_ON   HAL_GPIO_WritePin(B_DET_CON_GPIO_Port, B_DET_CON_Pin, GPIO_PIN_SET)
#define B_DET_OFF  HAL_GPIO_WritePin(B_DET_CON_GPIO_Port, B_DET_CON_Pin, GPIO_PIN_RESET)

#define IR1_ON       HAL_GPIO_WritePin(IR1_TX_GPIO_Port, IR1_TX_Pin, GPIO_PIN_RESET);
#define IR1_OFF      HAL_GPIO_WritePin(IR1_TX_GPIO_Port, IR1_TX_Pin, GPIO_PIN_SET);

#define IR2_ON       HAL_GPIO_WritePin(IR2_TX_GPIO_Port, IR2_TX_Pin, GPIO_PIN_RESET);
#define IR2_OFF      HAL_GPIO_WritePin(IR2_TX_GPIO_Port, IR2_TX_Pin, GPIO_PIN_SET);

#define LED0_ON       HAL_GPIO_WritePin(LED0_GPIO_Port, LED0_Pin, GPIO_PIN_RESET);
#define LED0_OFF      HAL_GPIO_WritePin(LED0_GPIO_Port, LED0_Pin, GPIO_PIN_SET);

#define LED1_ON       HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_RESET);
#define LED1_OFF      HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_SET);

#define IR1_IN       HAL_GPIO_ReadPin(IR1_RX_GPIO_Port,IR1_RX_Pin);
#define IR2_IN       HAL_GPIO_ReadPin(IR2_RX_GPIO_Port,IR2_RX_Pin);

#define KEY_RST_IN      HAL_GPIO_ReadPin(KEY_RST_GPIO_Port,KEY_RST_Pin);
#define KEY_OPEN_IN     HAL_GPIO_ReadPin(KEY_OPEN_GPIO_Port,KEY_OPEN_Pin);
#define KEY_CLOSE_IN    HAL_GPIO_ReadPin(KEY_CLOSE_GPIO_Port,KEY_CLOSE_Pin);

/* USER CODE END Private defines */

void MX_GPIO_Init(void);

/* USER CODE BEGIN Prototypes */

/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif
#endif /*__ pinoutConfig_H */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
