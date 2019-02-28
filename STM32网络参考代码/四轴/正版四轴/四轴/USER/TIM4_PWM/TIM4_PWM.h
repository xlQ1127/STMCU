#ifndef _PWM_OUTPUT_H_   
#define _PWM_OUTPUT_H_  

#include "stm32f10x.h" 


#define Moto_PwmMax 999

void TIM4_PWM_Init(void);
extern void TIM4_Mode_Config(u16 CCR1_Val,u16 CCR2_Val,u16 CCR3_Val,u16 CCR4_Val);
void Moto_PwmRflash(int16_t MOTO1_PWM,int16_t MOTO2_PWM,int16_t MOTO3_PWM,int16_t MOTO4_PWM);

#endif 				
