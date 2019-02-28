#ifndef _PWM_OUTPUT_H_   
#define _PWM_OUTPUT_H_  

#include "stm32f10x.h" 


void TIM4_PWM_Init(void);
extern void TIM4_Mode_Config(u16 CCR1_Val,u16 CCR2_Val,u16 CCR3_Val,u16 CCR4_Val);


#endif 				
