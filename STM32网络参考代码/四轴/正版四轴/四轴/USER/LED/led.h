#ifndef _LED_H_  
#define _LED_H_   
  	
#include "stm32f10x.h"   
 
#define ON  1   
#define OFF 0   
 
#define LED1(a) if (a) GPIO_SetBits(GPIOA,GPIO_Pin_15);  else  GPIO_ResetBits(GPIOA,GPIO_Pin_15);
								
void LED_GPIO_Config(void);

#endif 
