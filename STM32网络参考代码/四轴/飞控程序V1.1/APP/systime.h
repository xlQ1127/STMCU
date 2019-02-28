#ifndef __SYSTIME_H__
#define __SYSTIME_H__

#include "stm32f10x.h"

extern volatile u32 time5_tick;//Time5计数器

extern s16 acc[3],gyro[3],mag[3];

void System_Time_Init(void);
void TIM5_Config(void);//TIM5初始化

void TIM5_Stop(void);//关闭TIM5
void TIM5_Start(void);//开启TIM5

#endif


