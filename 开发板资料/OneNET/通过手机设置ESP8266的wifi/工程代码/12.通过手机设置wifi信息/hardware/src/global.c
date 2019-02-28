#include "stm32f10x.h"








void Enable_Interrupt(void) //开启全局中断
{

	

}

void Disable_Interrupt(void) //关闭全局中断
{

	

}

void Event_Cmd_On(void) //开启中断事务
{

	TIM_Cmd(TIM6, ENABLE);
	//USART_ITConfig(USART2, USART_IT_RXNE, ENABLE); //使能接收中断

}

void Event_Cmd_Off(void) //关闭中断事务
{

	TIM_Cmd(TIM6, DISABLE);
	//USART_ITConfig(USART2, USART_IT_RXNE, DISABLE); //使能接收中断

}
