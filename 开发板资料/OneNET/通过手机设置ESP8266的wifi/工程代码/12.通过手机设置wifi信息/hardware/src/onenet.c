#include "stm32f10x.h"

#include "onenet.h"
#include "edpkit.h"
#include "delay.h"
#include "led.h"
#include "sht20.h"
#include "gy30.h"
#include "adxl345.h"
#include "lcd1602.h"

#include <string.h>
#include <stdio.h>






EdpPacket *send_pkg;




unsigned int Litter2Num(unsigned char *litter)
{

	unsigned int num = 0;
	int numTemp[3];
	
	numTemp[0] = litter[0] - 48;
	numTemp[1] = litter[1] - 48;
	numTemp[2] = litter[2] - 48;
	
	if(numTemp[2] >= 0 && numTemp[2] <=9)
		num = numTemp[0] * 100 + numTemp[1] * 10 + numTemp[2];
	else if(numTemp[1] >= 0 && numTemp[1] <=9)
		num = numTemp[0] * 10 + numTemp[1];
	else
		num = numTemp[0];
	
	return (unsigned int)num;

}

void OneNet_DevLink(const char* devid, const char* auth_key)
{

	Usart2_RcvClr();

	send_pkg = PacketConnect1(devid, auth_key);
	DelayXms(1);
	
	Usart_SendString(USART2, send_pkg->_data, send_pkg->_write_pos);  //发送设备连接请求数据
	DelayXms(10);
	
	DeleteBuffer(&send_pkg);
	DelayXms(1);
	
}

void FillSendBuf(char *buf)
{
	
	char text[10];
	
	memset(text, 0, sizeof(text));
	
	strcat(buf, ",;");

	//红
	sprintf(text, "%d;", ledStatus.LedRedSta);
	strcat(buf, "Red_Led,");
	strcat(buf, text);
	
	memset(text, 0, sizeof(text));
	
	//绿
	sprintf(text, "%d;", ledStatus.LedGreenSta);
	strcat(buf, "Green_Led,");
	strcat(buf, text);
	
	memset(text, 0, sizeof(text));
	
	//蓝
	if(ledStatus.LedBlueSta == LED_ON)
	{
		strcat(buf, "Blue_Led,1;");
	}
	else
	{
		strcat(buf, "Blue_Led,0;");
	}
	
	//黄
	if(ledStatus.LedYellowSta == LED_ON)
	{
		strcat(buf, "Yellow_Led,1;");
	}
	else
	{
		strcat(buf, "Yellow_Led,0;");
	}
	
	strcat(buf, "temperature,");
	sprintf(text, "%0.1f;", sht20Info.tempreture);
	strcat(buf, text);
	
	memset(text, 0, sizeof(text));
	
	strcat(buf, "humidity,");
	sprintf(text, "%0.1f;", sht20Info.humidity);
	strcat(buf, text);
	
	memset(text, 0, sizeof(text));
	
	strcat(buf, "light,");
	sprintf(text, "%d;", gy30Info.lightVal);
	strcat(buf, text);
	
	memset(text, 0, sizeof(text));
	
	strcat(buf, "Xg,");
	sprintf(text, "%0.2f;", adxlInfo.incidence_Xf);
	strcat(buf, text);
	
	memset(text, 0, sizeof(text));
	
	strcat(buf, "Yg,");
	sprintf(text, "%0.2f;", adxlInfo.incidence_Yf);
	strcat(buf, text);
	
	memset(text, 0, sizeof(text));
	
	strcat(buf, "Zg,");
	sprintf(text, "%0.2f;", adxlInfo.incidence_Zf);
	strcat(buf, text);

}

void OneNet_SendData(void)
{
	
	char send_buf[256];
	
	memset(send_buf, 0, 256);

	Usart2_RcvClr();
	
	FillSendBuf(send_buf);
	
	send_pkg = PacketSavedataSimpleString(DEVICEID, send_buf);
	
	Usart_SendString(USART2, send_pkg->_data, send_pkg->_write_pos);	//向平台上传数据点
	DelayXms(1);
	
	DeleteBuffer(&send_pkg);
	DelayXms(10);
	
}

void HeartBeat(USART_INFO *usartInfo) //心跳请求
{

	unsigned char heartBeat[2] = {PINGREQ, 0}; //心跳请求
	unsigned short timeOut = 300;
	
	DelayUs(10);
			
	Usart_SendString(USART2, heartBeat, sizeof(heartBeat));	//向平台上传心跳请求
	while(--timeOut)
	{
		UsartReciveFlag(&usart2Info);
		if(usart2Info.usartReceiveFlag == REV_OK)
		{
			usart2Info.usartReceiveFlag = REV_WAIT;
					
			if(usart2Info.usartBuf[0] == PINGRESP) //心跳响应
			{
				Lcd1602_DisString(0xCF, "0");
			}
			else
			{
				Lcd1602_DisString(0xCF, "1");
				OneNet_DevLink(DEVICEID, APIKEY); //和平台建立设备连接
			}
	
			break;
		}
				
		DelayXms(10);
	}
	
	if(timeOut == 0)
	{
		Lcd1602_DisString(0xCF, "1");
		OneNet_DevLink(DEVICEID, APIKEY); //和平台建立设备连接
	}
	
	Usart2_RcvClr();

}

void EDPKitCmd(USART_INFO *usartInfo)
{

	if(usartInfo->usartBuf[0] == CONNRESP) //连接响应
	{
		Lcd1602_DisString(0xCE, "%d", usartInfo->usartBuf[3]); //
		
		//0		连接成功
		//1		验证失败：协议错误
		//2		验证失败：设备ID鉴权失败
		//3		验证失败：服务器失败
		//4		验证失败：用户ID鉴权失败
		//5		验证失败：未授权
		//6		验证失败：激活码未分配
		//7		验证失败：该设备已被激活
		//8		验证失败：重复发送连接请求包
		//9		验证失败：重复发送连接请求包
	}

}

void OneNetApp(USART_INFO *usartInfo)
{

	if(strstr((char *)usart2Info.usartCmdBuf, "redled"))
	{
		ledStatus.LedRedSta = Litter2Num(usart2Info.usartExtBuf);
		TIM_SetCompare2(TIM8, ledStatus.LedRedSta);
				
		OneNet_SendData();
	}
	else if(strstr((char *)usart2Info.usartCmdBuf, "greenled"))
	{
		ledStatus.LedGreenSta = Litter2Num(usart2Info.usartExtBuf);
		TIM_SetCompare3(TIM8, ledStatus.LedGreenSta);
				
		OneNet_SendData();
	}
	else if(strstr((char *)usart2Info.usartCmdBuf, "blueled"))
	{
		if(usart2Info.usartExtBuf[0] == '1')
		{
			ledStatus.LedBlueSta = LED_ON;
			Led_Blue_Set(LED_ON);
		}
		else if(usart2Info.usartExtBuf[0] == '0')
		{
			ledStatus.LedBlueSta = LED_OFF;
			Led_Blue_Set(LED_OFF);
		}
				
		OneNet_SendData();
	}
	else if(strstr((char *)usart2Info.usartCmdBuf, "yellowled"))
	{
		if(usart2Info.usartExtBuf[0] == '1')
		{
			ledStatus.LedYellowSta = LED_ON;
			Led_Yellow_Set(LED_ON);
		}
		else if(usart2Info.usartExtBuf[0] == '0')
		{
			ledStatus.LedYellowSta = LED_OFF;
			Led_Yellow_Set(LED_OFF);
		}
			
		OneNet_SendData();
	}

}
