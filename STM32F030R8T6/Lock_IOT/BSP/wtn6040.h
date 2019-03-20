#ifndef WTN6040_H
#define WTN6040_H

#include "stm32f0xx_hal.h"
#include "gpio.h"

#define WTN_CLKLOW     HAL_GPIO_WritePin(WTN_CLK_GPIO_Port, WTN_CLK_Pin, GPIO_PIN_RESET)
#define WTN_CLKHIGH    HAL_GPIO_WritePin(WTN_CLK_GPIO_Port, WTN_CLK_Pin, GPIO_PIN_SET)
  
#define WTN_DATLOW     HAL_GPIO_WritePin(WTN_DAT_GPIO_Port, WTN_DAT_Pin, GPIO_PIN_RESET)
#define WTN_DATHIGH    HAL_GPIO_WritePin(WTN_DAT_GPIO_Port, WTN_DAT_Pin, GPIO_PIN_SET)



void SendMessage(uint8_t num);

#endif
