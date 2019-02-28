#ifndef __BSP_LED_H
#define __BSP_LED_H		

#include "main.h"
#include "stm32f1xx_hal.h"


#define LD2_ON    HAL_GPIO_WritePin(GPIOB, LED2_Pin, GPIO_PIN_RESET);
#define LD2_OFF   HAL_GPIO_WritePin(GPIOB, LED2_Pin, GPIO_PIN_SET);

#define LD3_ON    HAL_GPIO_WritePin(GPIOB, LED3_Pin, GPIO_PIN_RESET);
#define LD3_OFF   HAL_GPIO_WritePin(GPIOB, LED3_Pin, GPIO_PIN_SET);

#define LD4_ON    HAL_GPIO_WritePin(GPIOB, LED4_Pin, GPIO_PIN_RESET);
#define LD4_OFF   HAL_GPIO_WritePin(GPIOB, LED4_Pin, GPIO_PIN_SET);
 
#define LD5_ON    HAL_GPIO_WritePin(GPIOB, LED5_Pin, GPIO_PIN_RESET);
#define LD5_OFF   HAL_GPIO_WritePin(GPIOB, LED5_Pin, GPIO_PIN_SET);


void Forward_Blink(uint16_t Speed);

void Backward_Blink(uint16_t Speed);

#endif 
