#ifndef __BSP_LED_H__
#define __BSP_LED_H__

#include "gpio.h"
#include "stm32f4xx_hal.h"

#define LED1_ON  HAL_GPIO_WritePin(GPIOA, D2_Pin, GPIO_PIN_RESET); 
#define LED1_OFF HAL_GPIO_WritePin(GPIOA, D2_Pin, GPIO_PIN_SET); 
#define LED2_ON  HAL_GPIO_WritePin(GPIOA, D3_Pin, GPIO_PIN_RESET); 
#define LED2_OFF HAL_GPIO_WritePin(GPIOA, D3_Pin, GPIO_PIN_SET); 


void Blink_LED(uint32_t Time_ms, uint32_t Times); //µ•∏ˆLED…¡À∏

void LED_Erro_Alarm(void);

#endif
