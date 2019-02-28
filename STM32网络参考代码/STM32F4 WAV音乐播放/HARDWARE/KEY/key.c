#include <stm32f10x_lib.h>
#include "key.h"
#include "delay.h"
//Mini STM32开发板
//按键输入 驱动代码			 
//正点原子@ALIENTEK
//2010/5/27   

//按键初始化函数
//特别注意：在该函数之后，JTAG将无法使用（SWD也无法使用）
//如果想JTAG仿真，可以屏蔽该函数。
//PA0.13.15 设置成输入
void KEY_Init(void)
{
	RCC->APB2ENR|=1<<2;     //使能PORTA时钟
	RCC->APB2ENR|=1<<4;     //使能PORTC时钟
	RCC->APB2ENR|=1<<5;     //使能PORTD时钟

	GPIOA->CRL&=0XFFFFFFF0;//PA0设置成输入	  
	GPIOA->CRL|=0X00000008;   
	GPIOA->CRH&=0XFFFFFFF0;//PA8设置成输入	  
	GPIOA->CRH|=0X00000008; 				   
	GPIOA->ODR|=0x101;	   //PA0,A8上拉

	GPIOC->CRH&=0XFF0FFFFF;//PC13设置成输入	  
	GPIOC->CRH|=0X00800000; 				   
	GPIOC->ODR|=1<<13;	   //PC13上拉

	GPIOD->CRL&=0XFFFF0FFF;//PD3设置成输入	  
	GPIOD->CRL|=0X00008000;    				   
	GPIOD->ODR|=1<<3;	   //PD3上拉
} 
//按键处理函数
//返回按键值
//0，没有任何按键按下
//1，KEY0按下
//2，KEY1按下
//3，KEY2按下 WK_UP
//注意此函数有响应优先级,KEY0>KEY1>KEY2!!
u8 KEY_Scan(void)
{	 
	static u8 key_up=1;//按键按松开标志	

	if(key_up&&(KEY1==0||KEY2==0||KEY3==0||KEY4==0))
	{
		delay_ms(10);//去抖动 
		key_up=0;
		if(KEY1==0)
		{
			return 1;
		}
		else if(KEY2==0)
		{
			return 2;
		}
		else if(KEY3==0)
		{
			return 3;
		}
		else if(KEY4==0)
		{
			return 4;
		}
	}else if(KEY1==1&&KEY2==1&&KEY3==1&&KEY4==1)key_up=1; 	    
	return 0;// 无按键按下
}




















