#ifndef __PWNIN_H
#define	__PWNIN_H

#include "stm32f4xx.h"

extern int pwmout1,pwmout2,pwmout3,pwmout4;

void TIM3_IRQHandler(void);
void pwm_in_init(void); 

#endif/* __PWNIN */
