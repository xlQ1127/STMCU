#include "SysTick.h"

static u8  fac_us=0;//us延时倍乘数
static u16 fac_ms=0;//ms延时倍乘数


/*
 * 函数名：Delay_Init
 * 描述  ：SYSTICK的时钟固定为HCLK时钟的1/8
 * 输入  ：SYSCLK(系统时钟/1MHz)
 * 输出  ：无
 * 调用  ：外部调用 
 */
void Delay_Init(u8 SYSCLK)
{
	//bit2清空,选择外部时钟  HCLK/8
	SysTick_CLKSourceConfig(SysTick_CLKSource_HCLK_Div8);
	fac_us=SYSCLK/8;		    
	fac_ms=(u16)fac_us*1000;
}

/*
 * 函数名：delay_ms
 * 描述  ：ms延时程序
 * 输入  ：nms 最大值1864!!!
 * 输出  ：无
 * 调用  ：外部调用 
 */
void delay_ms(u16 nms)
{	 		  	  
	u32 temp;		   
	SysTick->LOAD=(u32)nms*fac_ms;//时间加载(SysTick->LOAD为24bit)
	SysTick->VAL =0x00;           //清空计数器
	SysTick->CTRL=0x01 ;          //开始倒数  
	do
	{
		temp=SysTick->CTRL;
	}
	while(temp&0x01&&!(temp&(1<<16)));//等待时间到达   
	SysTick->CTRL=0x00;       //关闭计数器
	SysTick->VAL =0X00;       //清空计数器	  	    
}

/*
 * 函数名：delay_us
 * 描述  ：us延时程序
 * 输入  ：nus 最大值1864135!!! 
 * 输出  ：无
 * 调用  ：外部调用 
 */
void delay_us(u32 nus)
{		
	u32 temp;	    	 
	SysTick->LOAD=nus*fac_us; //时间加载	  		 
	SysTick->VAL=0x00;        //清空计数器
	SysTick->CTRL=0x01 ;      //开始倒数 	 
	do
	{
		temp=SysTick->CTRL;
	}
	while(temp&0x01&&!(temp&(1<<16)));//等待时间到达   
	SysTick->CTRL=0x00;       //关闭计数器
	SysTick->VAL =0X00;       //清空计数器	 
}





