/**
	************************************************************
	************************************************************
	************************************************************
	*	文件名： 	hwtimer.c
	*
	*	作者： 		张继瑞
	*
	*	日期： 		2016-11-23
	*
	*	版本： 		V1.0
	*
	*	说明： 		单片机定时器初始化
	*
	*	修改记录：	
	************************************************************
	************************************************************
	************************************************************
**/

//协议层
#include "onenet.h"

//硬件驱动
#include "hwtimer.h"


/*
************************************************************
*	函数名称：	Timer3_4_Init
*
*	函数功能：	Timer3或4的定时配置
*
*	入口参数：	TIMx：TIM3 或者 TIM4
*				arr：重载值
*				psc分频值
*
*	返回参数：	无
*
*	说明：		timer3和timer4只具有更新中断功能
************************************************************
*/
void Timer3_4_Init(TIM_TypeDef * TIMx, unsigned short arr, unsigned short psc)
{

	TIM_TimeBaseInitTypeDef timer_initstruct;
	NVIC_InitTypeDef nvic_initstruct;
	
	if(TIMx == TIM3)
	{
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
		
		nvic_initstruct.NVIC_IRQChannel = TIM3_IRQn;
	}
	else
	{
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);
		
		nvic_initstruct.NVIC_IRQChannel = TIM4_IRQn;
	}
	
	timer_initstruct.TIM_CounterMode = TIM_CounterMode_Up;
	timer_initstruct.TIM_Period = arr;
	timer_initstruct.TIM_Prescaler = psc;
	
	TIM_TimeBaseInit(TIMx, &timer_initstruct);
	
	TIM_ITConfig(TIMx, TIM_IT_Update, ENABLE); //使能更新中断
	
	nvic_initstruct.NVIC_IRQChannelCmd = ENABLE;
	nvic_initstruct.NVIC_IRQChannelPreemptionPriority = 1;
	nvic_initstruct.NVIC_IRQChannelSubPriority = 1;
	
	NVIC_Init(&nvic_initstruct);
	
	TIM_Cmd(TIMx, ENABLE); //使能定时器

}

/*
************************************************************
*	函数名称：	TIM3_IRQHandler
*
*	函数功能：	Timer3更新中断服务函数
*
*	入口参数：	无
*
*	返回参数：	无
*
*	说明：		
************************************************************
*/
void TIM3_IRQHandler(void)
{

	if(TIM_GetITStatus(TIM3, TIM_IT_Update) == SET)
	{
		
		OneNET_CmdHandle();
		
		TIM_ClearITPendingBit(TIM3, TIM_IT_Update);
	}

}

/*
************************************************************
*	函数名称：	TIM4_IRQHandler
*
*	函数功能：	Timer4更新中断服务函数
*
*	入口参数：	无
*
*	返回参数：	无
*
*	说明：		
************************************************************
*/
//void TIM4_IRQHandler(void)
//{

//	if(TIM_GetITStatus(TIM4, TIM_IT_Update) == SET)
//	{
//		TIM_ClearITPendingBit(TIM4, TIM_IT_Update);
//	}

//}
