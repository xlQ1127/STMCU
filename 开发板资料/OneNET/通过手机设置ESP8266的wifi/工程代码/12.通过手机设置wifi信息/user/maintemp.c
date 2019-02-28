#include "stm32f10x.h"

#include "led.h"
#include "delay.h"
#include "key.h"
#include "lcd1602.h"
#include "usart.h"
#include "onenet.h"
#include "timer.h"
#include "i2c.h"
#include "gy30.h"
#include "adxl345.h"
#include "sht20.h"
#include "iwdg.h"

#include "esp8266.h"
#include "m6311.h"

#include <string.h>








void Hardware_Init(void)
{
	
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);

	Delay_Init();
	
	Led_Init();
	
	Key_Init();
	
	Lcd1602_Init();
	
	Usart2_Init(115200); //初始化串口   115200bps
	
	IIC_Init();
	
	ADXL345_Init();
	
	GY30_Init();
	
	if(RCC_GetFlagStatus(RCC_FLAG_IWDGRST) != SET) //如果不是看门狗复位则初始化8266
	{
		ESP8266_Init();
		//M6311_Init();
	}
	else
	{
		RCC_ClearFlag();
		
		Lcd1602DisString(0x8F, "1");
	}
	
	OneNet_DevLink(DEVICEID, APIKEY);
	
	Timer1_8_Init(TIM8, 300, 3599);
	
	Timer6_7_Init(TIM6, 2000, 35999); //1s中断一次
	
	Iwdg_Init(4, 1250); //64分频，每秒625次，重载1250次，2s

}

int main()
{
	
	Hardware_Init();
	
	while(1)
	{
		
		Iwdg_Feed();
		
		switch(Keyboard())
		{
			case KEY0DOWN:
			case KEY0DOWNREPEAT:
				
				//ESP8266_SendData("STM32F103 ESP8266 Test\r\n");
			
				++ledStatus.LedRedSta;
				ledStatus.LedRedSta %= 300;
				TIM_SetCompare2(TIM8, ledStatus.LedRedSta);
			
			break;
			
			case KEY1DOWN:
			case KEY1DOWNREPEAT:
				
				++ledStatus.LedGreenSta;
				ledStatus.LedGreenSta %= 300;
				TIM_SetCompare3(TIM8, ledStatus.LedGreenSta);
			
			break;
				
			case KEY2DOWN:
				
				if(ledStatus.LedBlueSta == LED_ON)
				{
					ledStatus.LedBlueSta = LED_OFF;
					Led_Blue_Set(LED_OFF);
				}
				else
				{
					ledStatus.LedBlueSta = LED_ON;
					Led_Blue_Set(LED_ON);
				}
				
				OneNet_SendData();
			
			break;
			
			case KEY3DOWN:
				
				if(ledStatus.LedYellowSta == LED_ON)
				{
					ledStatus.LedYellowSta = LED_OFF;
					Led_Yellow_Set(LED_OFF);
				}
				else
				{
					ledStatus.LedYellowSta = LED_ON;
					Led_Yellow_Set(LED_ON);
				}
				
				OneNet_SendData();
			
			break;
		}
		
		UsartReciveFlag(&usart2Info);
		if(usart2Info.usartReceiveFlag == REV_OK)
		{
			usart2Info.usartReceiveFlag = REV_WAIT;
			
			EDPKitCmd(&usart2Info);
			
			OneNetApp(&usart2Info);
			
			Usart2_RcvClr();
			
			Iwdg_Feed();
		}
		
		if(timInfo.timer6Out == 1)
		{
			timInfo.timer6Out = 0;
			
			OneNet_SendData();
			
			Iwdg_Feed();
		}
		else if(timInfo.timer6Out == 2)
		{
			timInfo.timer6Out = 0;
			
			HeartBeat(&usart2Info);
			
			Iwdg_Feed();
		}
		
		DelayXms(100);
		
		ADXL345_GetValue();
		GY30_GetValue();
		
		Lcd1602DisString(0x80, "X%0.1f,Y%0.1f,Z%0.1f", adxlInfo.incidence_Xf, adxlInfo.incidence_Yf, adxlInfo.incidence_Zf);
		
		SHT20_GetValue();
		Lcd1602DisString(0xC0, "%0.1fC,%0.1f%%,%dLX", sht20Info.tempreture, sht20Info.humidity, gy30Info.lightVal);
		
		Iwdg_Feed();
	
	}

}
