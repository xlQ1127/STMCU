#include "stm32f10x_conf.h"

#include "led.h"






//RED_LED		PC7
//GREEN_LED		PC8
//YELLOW_LED	PC10
//BLUE_LED		PA12

LED_STATUS ledStatus;

void Led_Init(void)
{
	
	GPIO_InitTypeDef gpioInitStrcut;

	//使能时钟
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOC, ENABLE);
	
	//IO配置
	gpioInitStrcut.GPIO_Mode = GPIO_Mode_Out_PP;
	gpioInitStrcut.GPIO_Pin = GPIO_Pin_10;
	gpioInitStrcut.GPIO_Speed = GPIO_Speed_50MHz;
	//IO初始化
	GPIO_Init(GPIOC, &gpioInitStrcut);
	
	//IO配置
	gpioInitStrcut.GPIO_Pin = GPIO_Pin_12;
	//IO初始化
	GPIO_Init(GPIOA, &gpioInitStrcut);

}

//高电平开灯		低电平关灯
void Led_Red_Set(_Bool status)
{

	GPIO_WriteBit(GPIOC, GPIO_Pin_7, status == LED_ON ? Bit_SET : Bit_RESET);

}

_Bool Led_Red_Get(void)
{

	return GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_7);

}

void Led_Green_Set(_Bool status)
{

	GPIO_WriteBit(GPIOC, GPIO_Pin_8, status == LED_ON ? Bit_SET : Bit_RESET);

}

_Bool Led_Green_Get(void)
{

	return GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_8);

}

void Led_Yellow_Set(_Bool status)
{

	GPIO_WriteBit(GPIOC, GPIO_Pin_10, status == LED_ON ? Bit_SET : Bit_RESET);

}

_Bool Led_Yellow_Get(void)
{

	return GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_10);

}

void Led_Blue_Set(_Bool status)
{

	GPIO_WriteBit(GPIOA, GPIO_Pin_12, status == LED_ON ? Bit_SET : Bit_RESET);

}

_Bool Led_Blue_Get(void)
{

	return GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_12);

}
