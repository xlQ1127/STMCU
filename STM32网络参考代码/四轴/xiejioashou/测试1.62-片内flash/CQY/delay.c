/*************************************************
STM32 系统systick精确定时支持文件

MCU：STM32F103CBT6，库函数版本：3.5.0 KEIL

陈秋阳2013-01-16

*************************************************/


#include "stm32f10x.h"
#include "delay.h"
static u8   fac_us=0;//us延时倍乘数 
static u16 fac_ms=0;//ms延时倍乘数 

void Delay_s(u16 i)
{ u16 t;
  
	while(i)
	{
	for (t=65535;t>0;t--);
  i--;
  }


}





//----------------------毫秒延时----------------------------------------//
void delay_ms(u16 nms) 
{        
u32 temp; 
SysTick->CTRL&=0xfffffffb; //选择内部时钟 HCLK/8 
fac_us=8;       
fac_ms=(u16)fac_us*1000;
     
SysTick->LOAD=(u32)nms*fac_ms;//时间加载 
//SysTick_SetReload((u32)nms*fac_ms); 
SysTick->VAL =0x00;            //清空计数器 
//SysTick_CounterCmd(SysTick_Counter_Clear); 
SysTick->CTRL=0x01 ;           //开始倒数 
//SysTick_CounterCmd(SysTick_Counter_Enable); 
do 
{ 
   temp=SysTick->CTRL; 
} 
while(temp&0x01&&!(temp&(1<<16)));//等待时间到达    
SysTick->CTRL=0x00;        //关闭计数器 
SysTick->VAL =0x00;        //清空计数器   
      
} 
//----------------------微秒延时----------------------------------------//
void delay_us(u32 us) 
{         
u32 temp; 

SysTick->CTRL&=0xfffffffb;	//选择内部时钟 HCLK/8 
fac_us=8;       
fac_ms=(u16)fac_us*1000;

     
SysTick->LOAD=(u32)us*fac_us;//时间加载 
//SysTick_SetReload((u32)nms*fac_ms); 
SysTick->VAL =0x00;            //清空计数器 
//SysTick_CounterCmd(SysTick_Counter_Clear); 
SysTick->CTRL=0x01 ;           //开始倒数 
//SysTick_CounterCmd(SysTick_Counter_Enable); 
do 
{ 
   temp=SysTick->CTRL; 
} 
while(temp&0x01&&!(temp&(1<<16)));//等待时间到达    
SysTick->CTRL=0x00;        //关闭计数器 
SysTick->VAL =0x00;        //清空计数器   
      
} 
