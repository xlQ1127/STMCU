#include "stm32f10x.h"

#include "esp8266.h"
#include "delay.h"
#include "led.h"
#include "usart.h"
#include "lcd1602.h"
#include "iwdg.h"

#include <string.h>






ESP8266_INFO esp8266Info = {"", "", "NULL", "NULL", "876", "192.168.4.2", "", "OneNET", "12345678", "8086", 0, 0, 0, 0, 0}; //netWork ssid pswd 保留


//reset		PC4
//USART2

void ESP8266_QuitTrans(void) //退出透传模式
{

	while((USART2->SR & 0X40) == 0);	//等待发送空
	USART2->DR = '+';      
	DelayXms(15);					//大于串口组帧时间(10ms)
	
	while((USART2->SR & 0X40) == 0);	//等待发送空
	USART2->DR = '+';        
	DelayXms(15);					//大于串口组帧时间(10ms)
	
	while((USART2->SR & 0X40) == 0);	//等待发送空
	USART2->DR = '+';        
	DelayXms(500);					//等待500ms
	Iwdg_Feed();
	
	ESP8266_SendCmd("AT+CIPMODE=0\r\n", "OK"); //关闭透传模式

}

void ESP8266_ApInit(void)
{
	
	char cfgBuffer[70];
	
	ESP8266_QuitTrans();
	DelayMs(10);
	
	ESP8266_SendCmd("AT\r\n", "OK");
	
	ESP8266_SendCmd("AT+CWMODE=2\r\n", "OK");
	
	ESP8266_SendCmd("AT+RST\r\n", "OK");
	
	DelayMs(500);Iwdg_Feed();
	
	memset(cfgBuffer, 0, 70);
	
	strcpy(cfgBuffer, "AT+CWSAP=\"");
	strcat(cfgBuffer, esp8266Info.apName);
	strcat(cfgBuffer, "\",\"");
	strcat(cfgBuffer, esp8266Info.apPass);
	strcat(cfgBuffer, "\",1,4\r\n\"");
	ESP8266_SendCmd(cfgBuffer, "OK");
	
	memset(cfgBuffer, 0, 70);
	
	strcpy(cfgBuffer, "AT+CIPSTART=\"TCP\",\"");
	strcat(cfgBuffer, esp8266Info.apip);
	strcat(cfgBuffer, "\",");
	strcat(cfgBuffer, esp8266Info.apPort);
	strcat(cfgBuffer, "\r\n");
	while(ESP8266_SendCmd(cfgBuffer, "OK"))
	{
		Led_Blue_Set(LED_ON);
		DelayMs(500);Iwdg_Feed();
		Led_Blue_Set(LED_OFF);
		DelayMs(500);Iwdg_Feed();
	}
	
	ESP8266_SendCmd("AT+CIPMODE=1\r\n", "OK");
	
	ESP8266_SendCmd("AT+CIPSEND\r\n", ">");
	
	esp8266Info.netWork = 0; //局网模式

}

unsigned char ESP8266_StaInit(void)
{
	
	unsigned char errCount = 0;
	char cfgBuffer[70];
	
	ESP8266_QuitTrans();
	DelayMs(10);
	
	ESP8266_SendCmd("AT\r\n", "OK");
	
	ESP8266_SendCmd("AT+CWMODE=1\r\n", "OK");
	
	ESP8266_SendCmd("AT+RST\r\n", "OK");
	
	DelayMs(500);Iwdg_Feed();
	
	ESP8266_SendCmd("AT+CIFSR\r\n", "OK");
	
	memset(cfgBuffer, 0, 70);
	
	strcpy(cfgBuffer, "AT+CWJAP=\"");
	strcat(cfgBuffer, esp8266Info.staName);
	strcat(cfgBuffer, "\",\"");
	strcat(cfgBuffer, esp8266Info.staPass);
	strcat(cfgBuffer, "\"\r\n\"");
	while(ESP8266_SendCmd(cfgBuffer, "OK"))
	{
		Led_Blue_Set(LED_ON);
		DelayMs(500);Iwdg_Feed();
		Led_Blue_Set(LED_OFF);
		DelayMs(500);Iwdg_Feed();
		
		if(++errCount >= 5)
			return 1; //代表wifi信息错误
	}
	
	while(ESP8266_SendCmd("AT+CIPSTART=\"TCP\",\"183.230.40.39\",876\r\n", "OK"))
	{
		Led_Yellow_Set(LED_ON);
		DelayMs(500);Iwdg_Feed();
		Led_Yellow_Set(LED_OFF);
		DelayMs(500);Iwdg_Feed();
		
		if(++errCount >= 5)
			return 2; //代表平台信息错误
	}
	
	ESP8266_SendCmd("AT+CIPMODE=1\r\n", "OK");
	
	ESP8266_SendCmd("AT+CIPSEND\r\n", ">");
	
	esp8266Info.netWork = 1; //互联网模式
	
	return ESP_OK;

}

_Bool ESP8266_SendCmd(char *cmd, char *res)
{
	
	unsigned int timeOut = 200;
	
	Usart_SendString(USART2, (unsigned char *)cmd, strlen((const char *)cmd));
	
	while(timeOut--)
	{
		UsartReciveFlag(&usart2Info);
		if(usart2Info.usartReceiveFlag == REV_OK)
		{
			usart2Info.usartReceiveFlag = REV_WAIT;
			
			if(strstr((const char *)usart2Info.usartBuf, res) != NULL)
			{
				Usart2_RcvClr();
				
				return 0;
			}
		}
		
		DelayXms(10);
		Iwdg_Feed();
	}
	
	return 1;

}

void ESP8266_Mode(void)
{

	Lcd1602_Clear(0xFF); //清除两行显示内容
	Lcd1602_DisString(0x80, "Wait ESP8266 STA"); //提示，先进行sta模式连接到路由
	
	esp8266Info.err = ESP8266_StaInit(); //获取连接结果	0-成功
	
	if(esp8266Info.err == 1) //如果路由信息错误，则启动ap模式 通过手机获取ssid和password
	{
		Lcd1602_Clear(0xFF); //清除两行显示内容
		Lcd1602_DisString(0x80, "Wifi info Error"); //这个是说wifi的ssid和pswd设置错了
		Lcd1602_DisString(0xC0, "Use APP -> 8266"); //先用手机设置里边的wifi连接ap，然后用网络通讯调试器监测端口，等待连上
		
		ESP8266_ApInit(); //使用ap模式
		
		Lcd1602_Clear(0xFF); //清除两行显示内容
		Lcd1602_DisString(0x80, "ESP8266 AP OK   "); //提示ap ok
	}
	else if(esp8266Info.err == 2) //如果是平台信息错误，没有测试会不会错，也没有做重新设置平台ip和port的功能，后续会测试并加上
	{
		Lcd1602_Clear(0xFF);
		Lcd1602_DisString(0x80, "PT info Error");
		Lcd1602_DisString(0xC0, "Use APP -> 8266");
		
		ESP8266_ApInit();
		
		Lcd1602_Clear(0xFF);
		Lcd1602_DisString(0x80, "ESP8266 AP OK   ");
	}
	else
		Lcd1602_DisString(0x80, "ESP8266 STA OK  "); //这个代表使用sta模式ok了
	
	DelayMs(1500); //延时2s
	Iwdg_Feed();

}

_Bool ESP8266GetStatus(void)
{
	
	_Bool status = 1;

	Usart_SendString(USART2, "AT+CIPSTATUS\r\n",  14);
	
	while(1)
	{
		UsartReciveFlag(&usart2Info);
		if(usart2Info.usartReceiveFlag == REV_OK)
		{
			usart2Info.usartReceiveFlag = REV_WAIT;
			
			if(strstr((const char *)usart2Info.usartBuf, "STATUS:2")) //获得IP
			{
				status = 1;
			}
			else if(strstr((const char *)usart2Info.usartBuf, "STATUS:3")) //建立连接
			{
				status = 0;
			}
			else if(strstr((const char *)usart2Info.usartBuf, "STATUS:4")) //失去连接
			{
				status = 1;
			}
			else if(strstr((const char *)usart2Info.usartBuf, "STATUS:5")) //物理掉线
			{
				status = 1;
			}
			
			memset(usart2Info.usartBuf, 0, sizeof(usart2Info.usartBuf));
			
			break;
		}
	}
	
	return status;

}
