#ifndef __LED_H
#define __LED_H	 
#include "sys.h"
//Mini STM32开发板
//LED驱动代码			 
//正点原子@ALIENTEK
//2010/5/27

//LED端口定义
#define LED1 PFout(6)// PF6
#define LED2 PFout(7)// PF7	
#define LED3 PFout(8)// PF8
#define LED4 PFout(9)// PF9
#define LED5 PFout(10)//PF10

void LED_Init(void);//初始化		 				    
#endif

















