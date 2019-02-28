#ifndef _LED_H_  
#define _LED_H_   
  	
#include "stm32f10x.h"   
 
#define ON  1   
#define OFF 0   
 
#define LED1(a) if (a) GPIO_SetBits(GPIOB,GPIO_Pin_1);  else  GPIO_ResetBits(GPIOB,GPIO_Pin_1);
								
void LED_GPIO_Config(void);

#endif 
