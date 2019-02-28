#ifndef __LED_H
#define __LED_H

#include "stm32f4xx.h"

#define LED1_OFF GPIO_SetBits(GPIOC,GPIO_Pin_0)
#define LED1_ON GPIO_ResetBits(GPIOC,GPIO_Pin_0)
#define LED2_OFF GPIO_SetBits(GPIOC,GPIO_Pin_1)
#define LED2_ON GPIO_ResetBits(GPIOC,GPIO_Pin_1)
#define LED3_OFF GPIO_SetBits(GPIOC,GPIO_Pin_2)
#define LED3_ON GPIO_ResetBits(GPIOC,GPIO_Pin_2)
#define LED4_OFF GPIO_SetBits(GPIOC,GPIO_Pin_3)
#define LED4_ON GPIO_ResetBits(GPIOC,GPIO_Pin_3)

#define	LED1_Tog GPIOC->ODR^=GPIO_Pin_0; 
#define	LED2_Tog GPIOC->ODR^=GPIO_Pin_1;
#define	LED3_Tog GPIOC->ODR^=GPIO_Pin_2;
#define	LED4_Tog GPIOC->ODR^=GPIO_Pin_3;


void LED_Init(void);


#endif 

