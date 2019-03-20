#include "common.h"

//////所有的外设硬件操作

#define  TIM_PSC     479   ////100KHZ 

static uint32 fac_us, fac_ms;

void Delay_Init(void)

{
	SysTick_CLKSourceConfig(SysTick_CLKSource_HCLK_Div8);//systick时钟= HCLK/8 
	fac_us = SystemCoreClock / 8000000;
	fac_ms = fac_us * 1000;
}

void delay_us(uint16 us)
{
	uint32_t temp;

	SysTick->LOAD = us * fac_us;                  //时间加载
	SysTick->VAL = 0x00;                        //清除计数器

	SysTick->CTRL |= SysTick_CTRL_ENABLE_Msk;//打开systick定时器，开始倒计时

	do
	{
		temp = SysTick->CTRL;
	} while ((temp & 0x01) && !(temp&(1 << 16)));

	SysTick->CTRL &= ~SysTick_CTRL_ENABLE_Msk;//关闭systick定时器
	SysTick->VAL = 0x00;//清除计数器
}

void delay_ms(uint16 ms)
{
	uint32_t temp;

	SysTick->LOAD = ms * fac_ms;//时间加载
	SysTick->VAL = 0x00;        //清除计数器
	SysTick->CTRL |= SysTick_CTRL_ENABLE_Msk;///打开systick定时器，开始倒计时

	do
	{
		temp = SysTick->CTRL;
	} while ((temp & 0x01) && !(temp&(1 << 16)));

	SysTick->CTRL &= ~SysTick_CTRL_ENABLE_Msk;//关闭systick定时器
	SysTick->VAL = 0x00;       //清除计数器  
}

void uart1_sendchar(uint8 ch)
{
	USART1->TDR = ch;
	while ((USART1->ISR & 0x40) == 0);
}

void uart1_sendbuff(uint8 * buff, uint16 size)
{
	while (size--)
		uart1_sendchar(*buff++);
}

void uart2_sendchar(uint8 ch)
{
	USART2->TDR = ch;
	while ((USART2->ISR & 0x40) == 0);
}

void uart2_sendbuff(uint8 * buff, uint16 size)
{
	while (size--)
		uart2_sendchar(*buff++);
}