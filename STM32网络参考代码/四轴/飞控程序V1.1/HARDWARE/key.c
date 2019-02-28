#include "key.h"
#include "SysTick.h"

/*
 * 函数名：Key_Init
 * 描述  ：按键初始化
 * 输入  ：无
 * 输出  ：无
 * 调用  ：外部调用 
 */
void Key_Config(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;//定义IO口配置 结构体
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA,ENABLE);//使能GPIOA时钟
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;//选中引脚0
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD;//下拉输入
	GPIO_Init(GPIOA, &GPIO_InitStructure);//配置GPIOA_0引脚

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13 | GPIO_Pin_15;//选中引脚13 15
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;//上拉输入
	GPIO_Init(GPIOA, &GPIO_InitStructure);//配置GPIOA_0引脚			
}

/*
 * 函数名：Key_Scan
 * 描述  ：按键扫描
 * 输入  ：无
 * 输出  ：
 *		  0:没有按键按下
 *		  1:KEY0按下
 *		  2:KEY1按下
 *		  3:KEY2按下
 * 调用  ：外部调用 
 */
u8 Key_Scan(void)
{
	
	if(GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_0) == 1)
	{
		delay_ms(5);
		if(GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_0) == 1)
		{
			while(GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_0) == 1);
			return 3;
		}
		else return 0;
	}
	else if(GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_15) == 0)
	{
		delay_ms(5);
		if(GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_15) == 0) 
		{
			while(GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_15) == 0);	
			return 2;
		}
		else return 0;	
	}
	else if(GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_13) == 0)
	{
		delay_ms(5);
		if(GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_13) == 0)
		{
			while(GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_13) == 0);
			return 1;
		}
		else return 0;	
	}
	else return 0;			
}













