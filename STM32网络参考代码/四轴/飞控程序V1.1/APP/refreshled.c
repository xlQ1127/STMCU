#include "refreshled.h"
#include "led.h"




volatile u8 led_state;//定义led状态
u32 time6_tick;//Time6计数器

void Led_Refresh_Init(void )//闪烁初始化
{
	TIM6_Config();
	time6_tick = 0;
	
	led_state = 0xFF;
}
/*
 * 函数名：TIM6_Config
 * 描述  ：TIM6配置 NVIC中断配置 10ms中断一次
 * 输入  ：无
 * 输出  ：无
 * 调用  ：外部调用	
 */
void TIM6_Config(void)
{
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

	//设置NVIC中TIM6中断
	NVIC_InitStructure.NVIC_IRQChannel = TIM6_IRQn;//通道设置为TIM6
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority =3;//抢占3
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;//响应3
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;//打开TIM6中断通道
	NVIC_Init(&NVIC_InitStructure);//写入配置
	
	//设置TIM6
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM6,ENABLE);//开启TIM6时钟
  //TIM_DeInit(TIM6);//TIM6初始化为缺省值

	TIM_TimeBaseStructure.TIM_Period=10000;//设定计数器自动重装值
	TIM_TimeBaseStructure.TIM_Prescaler= (72 - 1);//时钟预分频数 72M/72
  TIM_TimeBaseStructure.TIM_ClockDivision=TIM_CKD_DIV1;//采样分频 TDTS = Tck_tim
  TIM_TimeBaseStructure.TIM_CounterMode=TIM_CounterMode_Up;//向上计数模式
	
  TIM_TimeBaseInit(TIM6,&TIM_TimeBaseStructure);//配置TIM6
    
	TIM_ClearFlag(TIM6,TIM_FLAG_Update);//清除溢出中断标志
  TIM_ITConfig(TIM6,TIM_IT_Update,ENABLE);//开启溢出中断
	TIM_Cmd(TIM6,ENABLE);//开启TIM6外设
		
}


/*
 * 函数名：TIM6_Start
 * 描述  ：开启TIM6
 * 输入  ：无
 * 输出  ：无
 * 调用  ：外部调用	
 */
void TIM6_Start(void)
{
	time6_tick = 0;
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM6,ENABLE);
	TIM_Cmd(TIM6,ENABLE);
}

/*
 * 函数名：TIM6_Stop
 * 描述  ：关闭TIM2
 * 输入  ：无
 * 输出  ：无
 * 调用  ：外部调用	
 */
void TIM6_Stop(void)
{
	TIM_Cmd(TIM6,DISABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM6,DISABLE);
}


//*****TIM6中断函数---10ms---*****//
void TIM6_IRQHandler(void)
{
	if(TIM_GetITStatus(TIM6,TIM_IT_Update) == SET)//检测TIM6溢出中断是否发生
	{
		TIM_ClearITPendingBit(TIM6,TIM_IT_Update);
		
		switch(led_state)
		{
				case 0:Led_Flash0();break;
				case 1:Led_Flash1();break;
				case 2:Led_Flash2();break;
				case 3:Led_Flash3();break;
		}
		time6_tick++;
		time6_tick = time6_tick%200;//2s一个周期
	}		
}



void Led_Flash0(void) //LED 1次慢闪+2次快闪
{
	if(time6_tick == 0)
	{
		LED0(ON);
	}
	else if(time6_tick == 15)
	{
		LED0(OFF);
	}
		
	if(time6_tick == 100)
	{
		LED0(ON);
	}
	else if(time6_tick == 105)
	{
		LED0(OFF);
	}
	else if(time6_tick == 110)
	{
		LED0(ON);
	}
	else if(time6_tick == 115)
	{
		LED0(OFF);
	}
}

void Led_Flash1(void) //LED 2次快闪
{
	if(time6_tick == 100)
	{
		LED0(ON);
	}
	else if(time6_tick == 105)
	{
		LED0(OFF);
	}
	else if(time6_tick == 110)
	{
		LED0(ON);
	}
	else if(time6_tick == 115)
	{
		LED0(OFF);
	}
}

void Led_Flash2(void) //LED 1次慢闪
{
	if(time6_tick == 0)
	{
		LED0(ON);
	}
	else if(time6_tick == 15)
	{
		LED0(OFF);
	}
}


void Led_Flash3(void) //LED 4次快闪
{
	if(time6_tick == 100)
	{
		LED0(ON);
	}
	else if(time6_tick == 105)
	{
		LED0(OFF);
	}
	else if(time6_tick == 110)
	{
		LED0(ON);
	}
	else if(time6_tick == 115)
	{
		LED0(OFF);
	}
	else if(time6_tick == 120)
	{
		LED0(ON);
	}
	else if(time6_tick == 125)
	{
		LED0(OFF);
	}
	else if(time6_tick == 130)
	{
		LED0(ON);
	}
	else if(time6_tick == 135)
	{
		LED0(OFF);
	}
}











