#include "main.h"

HOUSE_INFO houseInfo={0,0,0,0,0,0,0};

/**
  * @brief  各元器件初始化
**/
void House_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    //PA8-->Bee;PA11-->Fire;PA12-->Hum;
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
	BEE_OFF;
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
}

void Status_Scan(void)
{
	if(GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_11)==0)
	{
		houseInfo.Fire_Alarm=1;
		AlarmToOneNet();
		while(GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_11)==0)
		{
			BEE_ON;
			DelayMs(100);
			BEE_OFF;
			DelayMs(50);
		}
		OneNet_DevLink(DEVICEID,APIKEY);
		DelayMs(3000);
	} 
	else
	{
		houseInfo.Fire_Alarm=0;
	}
	
	if(GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_12)==1)
	{
		if(houseInfo.Safe_Mode)
		{
			houseInfo.Active_Alarm=2;
			AlarmToOneNet();
			while(GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_12)==1)
			{
				BEE_ON;
				DelayMs(50);
				BEE_OFF;
				DelayMs(10);
			}
			OneNet_DevLink(DEVICEID,APIKEY);
			DelayMs(3000);
		} else
		{
			houseInfo.Active_Alarm=1;
		}
	} 
	else
	{
		houseInfo.Active_Alarm=0;
	}
}


