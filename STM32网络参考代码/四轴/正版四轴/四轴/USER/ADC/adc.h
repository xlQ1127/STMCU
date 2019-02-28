#ifndef _ADC_H_  
#define _ADC_H_   

#include "stm32f10x.h"

extern float  VCC;
extern u16 ADC_ConvertedValue;

void ADC1_Init(void);

#endif
