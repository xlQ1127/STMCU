#include "led.h"

void LED_GPIO_Config(void)   
{          
      GPIO_InitTypeDef GPIO_InitStructure;
	
      RCC_APB2PeriphClockCmd( RCC_APB2Periph_GPIOA, ENABLE);
	
      GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_15  ;          
      GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_Out_PP;             
      GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;  
	
      GPIO_Init(GPIOA, &GPIO_InitStructure);              	
}   
