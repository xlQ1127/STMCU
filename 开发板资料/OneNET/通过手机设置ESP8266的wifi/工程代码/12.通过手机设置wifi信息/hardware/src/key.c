#include "stm32f10x.h"

#include "key.h"
#include "delay.h"
#include "led.h"







//PC13--SW5
//PC12--SW4
//PC11--SW3
//PD2--SW2

void Key_Init(void)
{

	GPIO_InitTypeDef gpioInitStructure;
	EXTI_InitTypeDef extiInitStructure;
	NVIC_InitTypeDef nvicInitStructure;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC | RCC_APB2Periph_GPIOD, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
	
	//IO配置
	gpioInitStructure.GPIO_Mode = GPIO_Mode_IPU;
	gpioInitStructure.GPIO_Pin = GPIO_Pin_11 | GPIO_Pin_12 | GPIO_Pin_13;
	gpioInitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	//IO初始化
	GPIO_Init(GPIOC, &gpioInitStructure);
	
	//IO配置
	gpioInitStructure.GPIO_Pin = GPIO_Pin_2;
	GPIO_Init(GPIOD, &gpioInitStructure);

#if(KEY_INT == 1)
	
	//中断配置		PC11
	GPIO_EXTILineConfig(GPIO_PortSourceGPIOC, GPIO_PinSource11);
	extiInitStructure.EXTI_Line = EXTI_Line11;
	extiInitStructure.EXTI_LineCmd = ENABLE;
	extiInitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
	extiInitStructure.EXTI_Trigger = EXTI_Trigger_Falling;
	//中断初始化
	EXTI_Init(&extiInitStructure);
	
	//中断配置		PC12
	GPIO_EXTILineConfig(GPIO_PortSourceGPIOC, GPIO_PinSource12);
	extiInitStructure.EXTI_Line = EXTI_Line12;
	//中断初始化
	EXTI_Init(&extiInitStructure);
	
	//中断配置		PC13
	GPIO_EXTILineConfig(GPIO_PortSourceGPIOC, GPIO_PinSource13);
	extiInitStructure.EXTI_Line = EXTI_Line13;
	//中断初始化
	EXTI_Init(&extiInitStructure);
	
	//中断配置		PD2
	GPIO_EXTILineConfig(GPIO_PortSourceGPIOD, GPIO_PinSource2);
	extiInitStructure.EXTI_Line = EXTI_Line2;
	//中断初始化
	EXTI_Init(&extiInitStructure);
	
	nvicInitStructure.NVIC_IRQChannel = EXTI15_10_IRQn;
	nvicInitStructure.NVIC_IRQChannelCmd = ENABLE;
	nvicInitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	nvicInitStructure.NVIC_IRQChannelSubPriority = 1;
	NVIC_Init(&nvicInitStructure);
	
	nvicInitStructure.NVIC_IRQChannel = EXTI2_IRQn;
	nvicInitStructure.NVIC_IRQChannelCmd = ENABLE;
	nvicInitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	nvicInitStructure.NVIC_IRQChannelSubPriority = 2;
	NVIC_Init(&nvicInitStructure);
	
#endif
	

}

unsigned char KeyScan(GPIO_TypeDef* GPIOX, unsigned int NUM)
{
	
	if(GPIOX == GPIOC)
	{
		if(!GPIO_ReadInputDataBit(GPIOC, NUM)) //按下  为低
		{
			return KEYDOWN;
		}
		else					   //弹起  为高
		{
			return KEYUP;
		}
	}
	else if(GPIOX == GPIOD)
	{
		if(!GPIO_ReadInputDataBit(GPIOD, NUM)) //按下  为低
		{
			return KEYDOWN;
		}
		else					 //弹起  为高
		{
			return KEYUP;
		}
	}
	
	return KEYUP;
	
}

unsigned char Keyboard(void)
{
	
	static unsigned char keyStatus = KEYNONE;
	static unsigned int keyBusyFlag = 0;
	
	if(KeyScan(GPIOC, KEY0) == KEYDOWN && !(keyBusyFlag & (~(1 << 0)))) //如果按下 且其他按键未按下
	{
		if(keyStatus == KEY0DOWN) //如果一直按下
		{
			return KEY0DOWNREPEAT; //返回一直按下
		}
		
		keyBusyFlag |= 1 << 0;//此按键处于忙状态
		keyStatus = KEY0DOWN;
		return keyStatus;
	}
	else if(KeyScan(GPIOC, KEY0) == KEYUP && keyStatus == KEY0DOWN) //如果释放
	{
		keyBusyFlag &= ~(1 << 0);//此按键处于空闲状态
		keyStatus = KEY0UP;
		return keyStatus;
	}
	
	if(KeyScan(GPIOC, KEY1) == KEYDOWN && !(keyBusyFlag & (~(1 << 1)))) //如果按下 且其他按键未按下
	{
		if(keyStatus == KEY1DOWN) //如果一直按下
		{
			return KEY1DOWNREPEAT; //返回一直按下
		}
		
		keyBusyFlag |= 1 << 1;
		keyStatus = KEY1DOWN;
		return keyStatus;
	}
	else if(KeyScan(GPIOC, KEY1) == KEYUP && keyStatus == KEY1DOWN) //如果释放
	{
		keyBusyFlag &= ~(1 << 1);
		keyStatus = KEY1UP;
		return keyStatus;
	}
	
	if(KeyScan(GPIOC, KEY2) == KEYDOWN && !(keyBusyFlag & (~(1 << 2)))) //如果按下 且其他按键未按下
	{
		if(keyStatus == KEY2DOWN) //如果一直按下
		{
			return KEY2DOWNREPEAT; //返回一直按下
		}
		
		keyBusyFlag |= 1 << 2;
		keyStatus = KEY2DOWN;
		return keyStatus;
	}
	else if(KeyScan(GPIOC, KEY2) == KEYUP && keyStatus == KEY2DOWN) //如果释放
	{
		keyBusyFlag &= ~(1 << 2);
		keyStatus = KEY2UP;
		return keyStatus;
	}
	
	if(KeyScan(GPIOD, KEY3) == KEYDOWN && !(keyBusyFlag & (~(1 << 3)))) //如果按下 且其他按键未按下
	{
		if(keyStatus == KEY3DOWN) //如果一直按下
		{
			return KEY3DOWNREPEAT; //返回一直按下
		}
		
		keyBusyFlag |= 1 << 3;
		keyStatus = KEY3DOWN;
		return keyStatus;
	}
	else if(KeyScan(GPIOD, KEY3) == KEYUP && keyStatus == KEY3DOWN) //如果释放
	{
		keyBusyFlag &= ~(1 << 3);
		keyStatus = KEY3UP;
		return keyStatus;
	}
	
	keyBusyFlag = 0;
	keyStatus = KEYNONE;
	return keyStatus;
	
}

void EXTI15_10_IRQHandler(void)
{

	if(EXTI_GetITStatus(EXTI_Line11) == SET)
	{
		DelayXms(15);
		
		if(GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_11) == 0)
		{
			//do something ...
			if(Led_Red_Get() == 1)
				Led_Red_Set(LED_OFF);
			else
				Led_Red_Set(LED_ON);
		}
	}
	
	if(EXTI_GetITStatus(EXTI_Line12) == SET)
	{
		DelayXms(15);
		
		if(GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_12) == 0)
		{
			//do something ...
			if(Led_Green_Get() == 1)
				Led_Green_Set(LED_OFF);
			else
				Led_Green_Set(LED_ON);
		}
	}
	
	if(EXTI_GetITStatus(EXTI_Line13) == SET)
	{
		DelayXms(15);
		
		if(GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_13) == 0)
		{
			//do something ...
//			if(Led_Blue_Get() == 1)
//				Led_Blue_Set(LED_OFF);
//			else
//				Led_Blue_Set(LED_ON);
		}
	}
	
	EXTI_ClearITPendingBit(EXTI_Line11);
	EXTI_ClearITPendingBit(EXTI_Line12);
	EXTI_ClearITPendingBit(EXTI_Line13);

}

void EXTI2_IRQHandler(void)
{

	if(EXTI_GetITStatus(EXTI_Line2) == SET)
	{
		DelayXms(15);
		
		if(GPIO_ReadInputDataBit(GPIOD, GPIO_Pin_2) == 0)
		{
			//do something ...
			if(Led_Yellow_Get() == 1)
				Led_Yellow_Set(LED_OFF);
			else
				Led_Yellow_Set(LED_ON);
		}
	}
	
	EXTI_ClearITPendingBit(EXTI_Line2);

}
