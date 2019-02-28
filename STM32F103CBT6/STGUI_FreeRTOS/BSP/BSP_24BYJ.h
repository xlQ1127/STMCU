#ifndef __BSP_24BYJ_H
#define __BSP_24BYJ_H		

#include "main.h"
#include "stm32f1xx_hal.h"
	
typedef struct
{
uint8_t Direction ;
uint8_t Speed     ;
}BYJ_CMD_TypeDef ;




#define A_In_Set     HAL_GPIO_WritePin(GPIOA,BYJ_A_Pin,GPIO_PIN_SET)
#define A_In_Reset   HAL_GPIO_WritePin(GPIOA,BYJ_A_Pin,GPIO_PIN_RESET)

#define B_In_Set     HAL_GPIO_WritePin(GPIOA,BYJ_B_Pin,GPIO_PIN_SET)
#define B_In_Reset   HAL_GPIO_WritePin(GPIOA,BYJ_B_Pin,GPIO_PIN_RESET)
          
#define C_In_Set     HAL_GPIO_WritePin(GPIOA,BYJ_C_Pin,GPIO_PIN_SET)
#define C_In_Reset   HAL_GPIO_WritePin(GPIOA,BYJ_C_Pin,GPIO_PIN_RESET)

#define D_In_Set     HAL_GPIO_WritePin(GPIOA,BYJ_D_Pin,GPIO_PIN_SET)
#define D_In_Reset   HAL_GPIO_WritePin(GPIOA,BYJ_D_Pin,GPIO_PIN_RESET)

#define Defualt_Rotate_Speed 1

void BYJ_Rotate_Clockwise(uint8_t Speed);
void BYJ_Rotate_Anticlockwise(uint8_t Speed);
void BYJ_Stop(void);

#endif 
