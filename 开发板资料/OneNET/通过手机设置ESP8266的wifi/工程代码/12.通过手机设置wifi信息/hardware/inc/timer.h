#ifndef _TIMER_H_
#define _TIMER_H_

#include "stm32f10x.h"





typedef struct
{

	unsigned char timer6Out : 2;
	unsigned char reverse : 6;

} TIM_INFO;

extern TIM_INFO timInfo;






void Timer1_8_Init(TIM_TypeDef * TIMx, unsigned short arr, unsigned short psc);

void Timer6_7_Init(TIM_TypeDef * TIMx, unsigned short arr, unsigned short psc);

void UCOS_TimerInit(void);


#endif
