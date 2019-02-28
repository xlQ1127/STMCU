#ifndef __RC_H__
#define __RC_H__

#include "stm32f10x.h"

#define MAX_ANGLE 30
#define MAX_GYRO 280 //最快转动角速度280度每秒

extern u16 RC_CH[8];

void RC_Init(void);
void TIM4_Init(void);


u16 Value_2_Thr(void);
float Value_2_Roll(void);
float Value_2_Pitch(void);
s16 Vaule_2_Gyro(void);


void Channel_Adjust(void);//遥控器RC各通道校准(进入遥控器校准程序，校准完毕后必须复位才能退出！)
u8 Channel_Config(void);//读取Flash里的的RC通道校准值



#endif

