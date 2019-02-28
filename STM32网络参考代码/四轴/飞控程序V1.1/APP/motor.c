#include "motor.h"

//TIM3  PWM  输出通道：
//channel1  channel2  channel3  channel4
//PA6       PA7       PB0       PB1

/*
 * 函数名：TIM3_PWM_Init
 * 描述  ：TIM3_PWM输出配置
 * 输入  ：无
 * 输出  ：无
 * 调用  ：外部调用
 */
void TIM3_PWM_Init(u16 times)
{
	
	GPIO_InitTypeDef GPIO_InitStructure;

	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_OCInitTypeDef  TIM_OCInitStructure;

	
	//TIM3_PWM IO口配置
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA|RCC_APB2Periph_GPIOB,ENABLE);//GPIOA GPIOB 时钟开启

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6|GPIO_Pin_7;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; 
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;//复用推挽输出 
	GPIO_Init(GPIOA, &GPIO_InitStructure);//GPIOA 6 7口配置

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0|GPIO_Pin_1;
	GPIO_Init(GPIOB, &GPIO_InitStructure);//GPIOB 0 1 口配置 


	//TIM3_PWM配置
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3,ENABLE);//TIM3 时钟开启

	TIM_TimeBaseStructure.TIM_Period = times-1;//20000次为一个定时周期
  TIM_TimeBaseStructure.TIM_Prescaler = (72 - 1);//时钟预分频数 72M/72(1M/20000) = 50HZ
 	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;//设置时钟分频系数：不分频
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;//向上计数模式(0->?)
	TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);//配置TIM3

	/* PWM1 Mode configuration: Channel1 */
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;//配置为PWM模式1
 	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;	
 	TIM_OCInitStructure.TIM_Pulse = 0;//设置跳变值
 	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;//当定时器计数值小于CCR1_Val时为高电平

 	TIM_OC1Init(TIM3, &TIM_OCInitStructure);
 	TIM_OC1PreloadConfig(TIM3, TIM_OCPreload_Enable);//使能通道1

	/* PWM1 Mode configuration: Channel2 */
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
  TIM_OCInitStructure.TIM_Pulse = 0;//设置通道2的电平跳变值，输出另外一个占空比的PWM

  TIM_OC2Init(TIM3, &TIM_OCInitStructure);
	TIM_OC2PreloadConfig(TIM3, TIM_OCPreload_Enable);//使能通道2

	/* PWM1 Mode configuration: Channel3 */
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
  TIM_OCInitStructure.TIM_Pulse = 0;//设置通道3的电平跳变值，输出另外一个占空比的PWM
	
	TIM_OC3Init(TIM3, &TIM_OCInitStructure);
  TIM_OC3PreloadConfig(TIM3, TIM_OCPreload_Enable);//使能通道3

	/* PWM1 Mode configuration: Channel4 */
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
  TIM_OCInitStructure.TIM_Pulse = 0;//设置通道4的电平跳变值，输出另外一个占空比的PWM

  TIM_OC4Init(TIM3, &TIM_OCInitStructure);	
	TIM_OC4PreloadConfig(TIM3, TIM_OCPreload_Enable);//使能通道4
	
	TIM_ARRPreloadConfig(TIM3, ENABLE);//使能TIM3重载寄存器ARR
  /* TIM3 enable counter */
  TIM_Cmd(TIM3, ENABLE);//使能定时器3
	
	Set_Motor(1000,1000,1000,1000);
}

//电机的刷新速率50-499
void Motor_Init(u16 rate)
{
	u16 temp = rate;
	
	if(temp>499)//对频率进行限制
	{
		temp = 499;
	}
	if(temp<50)
	{
		temp =50;
	}
	
	TIM3_PWM_Init(1000000/temp);
}


//控制电机转速
//1000-2000
void Set_Motor(u16 M1,u16 M2,u16 M3,u16 M4)
{
		TIM_SetCompare1(TIM3,M1);//1000-2000,1ms-2ms
		TIM_SetCompare2(TIM3,M2);
		TIM_SetCompare3(TIM3,M3);
		TIM_SetCompare4(TIM3,M4);
}

//控制舵机角度
//0-180
void Set_Servo(u16 S1,u16 S2,u16 S3,u16 S4)
{
	if(S1>180 || S2>180 || S3>180 || S4>180) return;
	else
	{
		TIM_SetCompare1(TIM3,(float)S1*11.111+500);//500-2500,0.5ms-2.5ms
		TIM_SetCompare2(TIM3,(float)S2*11.111+500);
		TIM_SetCompare3(TIM3,(float)S3*11.111+500);
		TIM_SetCompare4(TIM3,(float)S4*11.111+500);
	}
}


