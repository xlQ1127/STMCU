#ifndef __SYSTICK_H__
#define __SYSTICK_H__

#include "stm32f10x.h"


void Delay_Init(u8 SYSCLK);//延时函数初始化
void delay_ms(u16 nms);//ms延时程序
void delay_us(u32 nus);//us延时程序



#endif


