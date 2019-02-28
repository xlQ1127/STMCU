#ifndef _HWTIMER_H_
#define _HWTIMER_H_


#include "stm32f10x.h"


void Timer3_4_Init(TIM_TypeDef * TIMx, unsigned short arr, unsigned short psc);


#endif
