#ifndef __ADC_H
#define __ADC_H

#include "stm32f10x.h"
void Adc_Init(void);
void Get_Adc_Average(u16 adc[],u8 times);
void Delay(unsigned int i);
#endif
