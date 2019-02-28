#ifndef _ADC_H_  
#define _ADC_H_   

#include "stm32f10x.h"

extern float  VCC_yao;
extern int16_t  accelerator;
extern int16_t  Pitch_ta;
extern int16_t  Roll_ta;
extern int16_t  Yaw_ta;

extern u16 ADC_ConvertedValue[];

void ADC1_Init(void);
void ADC1_Value(void);

#endif
