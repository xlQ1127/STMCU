/******************** (C) COPYRIGHT 2014 Air Nano Team ***************************
 * 文件名  ：moto.c
 * 描述    ：电机驱动配置         
 * 实验平台：Air Nano四轴飞行器
 * 库版本  ：ST3.5.0
 * 作者    ：Air Nano Team 
 * 淘宝    ：http://byd2.taobao.com
**********************************************************************************/
#include "moto.h"
#include "board_config.h"

void Tim1_init(void)        //TIM1  为高级定时器  【预留通道】 未用
{
	TIM_TimeBaseInitTypeDef		TIM_TimeBaseStructure;
	TIM_OCInitTypeDef  				TIM_OCInitStructure;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);
	GPIO_PinRemapConfig(GPIO_FullRemap_TIM1 ,ENABLE);  
	
	/**********************************************************
	72 000 000/72=1M
	1000 000/2500=400Hz
	所以产生的PWM为400Hz
	周期为2.5ms，对应2500的计算值，1ms~2ms对应的计算值为1000~2000；
	**********************************************************/
	TIM_TimeBaseStructure.TIM_Period = 2499;		  //计数上线	2500
	TIM_TimeBaseStructure.TIM_Prescaler = 71; 	  //pwm时钟分频
	TIM_TimeBaseStructure.TIM_ClockDivision = 0;	
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;		//向上计数
	TIM_TimeBaseStructure.TIM_RepetitionCounter=0;//重复寄存器，用于自动更新pwm占空比
	TIM_TimeBaseInit(TIM1, &TIM_TimeBaseStructure);//===================================================【时基初始化】
	
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_Pulse = 1000;         //电平跳变计数值
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
	
	//下面几个参数是高级定时器才会用到，通用定时器不用配置
    TIM_OCInitStructure.TIM_OCNPolarity=TIM_OCNPolarity_Low;     //设置互补端输出极性
	TIM_OCInitStructure.TIM_OutputNState=TIM_OutputNState_Enable;//使能互补端输出
	TIM_OCInitStructure.TIM_OCIdleState=TIM_OCIdleState_Reset;   //死区后输出状态
	TIM_OCInitStructure.TIM_OCNIdleState=TIM_OCNIdleState_Reset; //死区后互补端输出状态
	//==================================================================================================【各通道输出模式配置】
	TIM_OC1Init(TIM1, &TIM_OCInitStructure);          //初始化通道一
	TIM_OC1PreloadConfig(TIM1, TIM_OCPreload_Enable);                   //使能通道一比较寄存器 【CCR】
	TIM_OC2Init(TIM1, &TIM_OCInitStructure);          //初始化通道二
	TIM_OC2PreloadConfig(TIM1, TIM_OCPreload_Enable);                   //使能通道二比较寄存器 【CCR】
	TIM_OC3Init(TIM1, &TIM_OCInitStructure);          //初始化通道三
	TIM_OC3PreloadConfig(TIM1, TIM_OCPreload_Enable);                   //使能通道三比较寄存器 【CCR】
	TIM_OC4Init(TIM1, &TIM_OCInitStructure);          //初始化通道四
	TIM_OC4PreloadConfig(TIM1, TIM_OCPreload_Enable);                   //使能通道四比较寄存器 【CCR】
	
	TIM_ARRPreloadConfig(TIM1, ENABLE);//使能重装载寄存器 【ARR】
	TIM_Cmd(TIM1, ENABLE);             //使能 【TIM1】
  TIM_CtrlPWMOutputs(TIM1, ENABLE);
}



void Tim4_init(void)//************************************************//【TIM4 PWM输出配置】
{                                                                     //【电机输出四通道】
	TIM_TimeBaseInitTypeDef		TIM_TimeBaseStructure;
	TIM_OCInitTypeDef  				TIM_OCInitStructure;
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);
	/******************************************************************
	72 000 000/72=1M
	1000 000/2500=400Hz
//===================================================================》》》》》】
 【所以产生的PWM为400Hz】                                            》》》》》】
 【周期为2.5ms，对应2500的计算值】，【1ms~2ms对应的计算值为1000~2000】 》》》》】
//===================================================================》》》》》】
	******************************************************************/
	TIM_TimeBaseStructure.TIM_Period = 2499;		//计数上线	
	TIM_TimeBaseStructure.TIM_Prescaler = 71;  	//pwm时钟分频
	TIM_TimeBaseStructure.TIM_ClockDivision = 0;	
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;		//向上计数
	TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure);
	
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_Pulse = 1000;
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
	
	TIM_OC1Init(TIM4, &TIM_OCInitStructure);
	TIM_OC1PreloadConfig(TIM4, TIM_OCPreload_Enable);
	TIM_OC2Init(TIM4, &TIM_OCInitStructure);
	TIM_OC2PreloadConfig(TIM4, TIM_OCPreload_Enable);
	TIM_OC3Init(TIM4, &TIM_OCInitStructure);
	TIM_OC3PreloadConfig(TIM4, TIM_OCPreload_Enable);
	TIM_OC4Init(TIM4, &TIM_OCInitStructure);
	TIM_OC4PreloadConfig(TIM4, TIM_OCPreload_Enable);
	
	TIM_ARRPreloadConfig(TIM4, ENABLE);
	TIM_Cmd(TIM4, ENABLE);
}

void PWM_OUT_Config(void) //========================================================【PWM输出GPIO配置】
{
	GPIO_InitTypeDef GPIO_InitStructure; //=========================【GPIOB-6-7-8-9】 【TIM4 四通道PWM输出】

  /* GPIOA and GPIOC clock enable */
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE); 

	GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_6 | GPIO_Pin_7 | GPIO_Pin_8 | GPIO_Pin_9;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;		    // 复用推挽输出
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOE, ENABLE); 
                                     //===========================【GPIOE-9-11-13-14】 【TIM1 四通道PWM输出】【未用】
	GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_9 | GPIO_Pin_11 | GPIO_Pin_13 | GPIO_Pin_14;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;		    // 复用推挽输出
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOE, &GPIO_InitStructure);
	
	Tim4_init();	
	Tim1_init();
}


/*====================================================================================================*/
/*====================================================================================================*
**函数 : pwmWriteMotor
**功能 : PWM写入电机
**输入 : 
**輸出 : None
**备注 : None
**====================================================================================================*/
/*====================================================================================================*/
void pwmWriteMotor(uint8_t index, uint16_t value)    //【此函数没有用到】
{    
	if(value > Moto_PwmMax)  value = Moto_PwmMax;      //输出正反向限幅
	if(value <= 0)           value = 0;                //应联系PWM输出周期参数
	// pwmWritePtr(index, value);
}

void writeMotors(int16_t *Moter)                     //循环检查四个电机PWM数值
{
    uint8_t i;

    for (i = 0; i < 4; i++)
        pwmWriteMotor(i, Moter[i]);                  //【i】电机号； 【Moter[i]】PWM写入值
}




//============================================================================================================//
void moto_PwmRflash(s16 *Moter)                      //循环更新四个电机PWM输出值
{		
	for(u8 i=0;i<MOTOR_NUM;i++)                        //Moter[i]里存放相应电机PWM输出值
	{
     if(*(Moter+i) > Moto_PwmMax)  *(Moter+i) = Moto_PwmMax; //【 Moto_PwmMax=1000 】
  }
	for(u8 i=0;i<MOTOR_NUM;i++)
	{
     if(*(Moter+i) <= 0 )  *(Moter+i) = 0;                   //输出正反向限幅【0——1000】范围
  }                                                          //电机PWM输出被限定在【1ms——2ms】
	
	if(MOTOR_NUM ==4 ){               //【循环更新各路PWM输出CCR】
		TIM4->CCR1 = 1000 + *(Moter++);
		TIM4->CCR2 = 1000 + *(Moter++);
		TIM4->CCR3 = 1000 + *(Moter++);
		TIM4->CCR4 = 1000 + *Moter;
	}
	else if(MOTOR_NUM == 6){          //六轴
		TIM4->CCR1 = 1000 + *(Moter++);
		TIM4->CCR2 = 1000 + *(Moter++);
		TIM4->CCR3 = 1000 + *(Moter++);
		TIM4->CCR4 = 1000 + *(Moter++);
		TIM1->CCR1 = 1000 + *(Moter++);
		TIM1->CCR2 = 1000 + *Moter;
	}
}

void moto_STOP(void)        //电机怠速  
{
	if(MOTOR_NUM ==4 ){	      //四轴        四轴使用 TIM4四路PWM输出控制
		TIM4->CCR1 = 1000;
		TIM4->CCR2 = 1000;
		TIM4->CCR3 = 1000;
		TIM4->CCR4 = 1000;
	}
	else if(MOTOR_NUM == 6){  //六轴        六轴使用 TIM4四路 TIM1两路PWM输出控制
		TIM4->CCR1 = 1000;
		TIM4->CCR2 = 1000;
		TIM4->CCR3 = 1000;
		TIM4->CCR4 = 1000;
		TIM1->CCR1 = 1000;
		TIM1->CCR2 = 1000;
	}
}

