#ifndef __BSP_LED_H__
#define __BSP_LED_H__

#include "stm32f1xx_hal.h"
#include "gpio.h"

#define LED_ON LL_GPIO_ResetOutputPin(LED1_GPIO_Port, LED1_Pin)
#define LED_OFF LL_GPIO_SetOutputPin(LED1_GPIO_Port, LED1_Pin)

#define LED2_ON LL_GPIO_ResetOutputPin(LED2_GPIO_Port, LED2_Pin)
#define LED2_OFF LL_GPIO_SetOutputPin(LED2_GPIO_Port, LED2_Pin)

#define LED3_ON LL_GPIO_ResetOutputPin(LED3_GPIO_Port, LED3_Pin)
#define LED3_OFF LL_GPIO_SetOutputPin(LED3_GPIO_Port, LED3_Pin)

#define LED4_ON LL_GPIO_ResetOutputPin(LED4_GPIO_Port, LED4_Pin)
#define LED4_OFF LL_GPIO_SetOutputPin(LED4_GPIO_Port, LED4_Pin)

#define LED5_ON LL_GPIO_ResetOutputPin(LED5_GPIO_Port, LED5_Pin)
#define LED5_OFF LL_GPIO_SetOutputPin(LED5_GPIO_Port, LED5_Pin)

void Blink_LED(uint32_t Time_ms, uint32_t Times); //µ•∏ˆLED…¡À∏

#endif
