#ifndef _SYSTICK_H_   
#define _SYSTICK_H_  

#include "stm32f10x.h"  

void SysTick_Init(void);
void TimingDelay_Decrement(void);
void Delay_us(uint32_t nTime);

#endif 
