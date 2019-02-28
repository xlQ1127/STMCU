#include "led.h"


/*
 * 函数名：LED_GPIO_Config
 * 描述  ：配置LED用到的I/O口
 * 输入  ：无
 * 输出  ：无
 * 调用  ：外部调用
 */
void LED_GPIO_Config(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;//定义IO口配置 结构体

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC,ENABLE);//使能GPIOC时钟


	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13;//选中引脚13
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;//IO输出速度50Hz
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;//推挽输出
	GPIO_Init(GPIOC, &GPIO_InitStructure);//配置GPIOC_13引脚
	

	GPIO_SetBits(GPIOC,GPIO_Pin_13);//PC13置位

}






