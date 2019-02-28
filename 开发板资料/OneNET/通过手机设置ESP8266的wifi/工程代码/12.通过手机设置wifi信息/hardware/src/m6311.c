#include "stm32f10x.h"

#include "m6311.h"
#include "usart.h"
#include "delay.h"
#include "led.h"
#include "iwdg.h"

#include <string.h>







void M6311_Init(void)
{

	GPIO_InitTypeDef gpioInitStruct;
	
	gpioInitStruct.GPIO_Mode = GPIO_Mode_Out_PP;
	gpioInitStruct.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1;
	gpioInitStruct.GPIO_Speed = GPIO_Speed_50MHz;
	
	GPIO_Init(GPIOA, &gpioInitStruct);
	
	M6311_PWR_ON; //上电
	DelayXms(50);
	
	//复位模块	如果不复位的话，在死机重启时，6311是无法初始化成功的
	M6311_RST_ON;
	DelayXms(100);
	M6311_RST_OFF;
	
	M6311_SendCmd("AT+SSYS?\r\n","OK"); //切换sim卡   0-内置卡		1-外置卡	这里使用外置卡
	M6311_SendCmd("AT+SIM1\r\n","OK"); //检测外置卡是否存在		返回+SIM1: EXSIT
	M6311_SendCmd("AT+CPIN?\r\n", "+CPIN: READY"); //确保SIM卡PIN码解锁，返回READY，表示解锁成功
	M6311_SendCmd("AT+CREG?\r\n","0,1"); //确认网络搜索成功,OK		//与例程0,5不同  我的卡显示0,0  然后是0,1，就不变了。 
	M6311_SendCmd("AT+CSQ\r\n","OK"); //查询信号强度,OK
	M6311_SendCmd("AT+CGACT=1,1\r\n","OK"); //激活
	M6311_SendCmd("AT+CGATT=1\r\n","OK");
	M6311_SendCmd("AT+CMMUX=0\r\n","OK"); //single way
	M6311_SendCmd("AT+CMMODE=1\r\n","OK"); //配置透传
	M6311_SendCmd("AT+CMTCFG=1,1024,1\r\n","OK"); //配置透传，最大长度2000字节，间隔是100ms。配置成hex模式
	M6311_SendCmd("AT+IPSTART=\"TCP\",\"183.230.40.39\",876\r\n","CONNECT"); //连接平台
	
	

}

_Bool M6311_SendCmd(char *cmd, char *res) //这个和8266不同，因为要等待解锁sim卡 获取网络等，而且我没调试到底哪些指令需要重复发，所以干脆全部都这样
{
	
	Usart_SendString(USART2, (unsigned char *)cmd, strlen((const char *)cmd));
	
	while(1)
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
			else if(strstr((const char *)usart2Info.usartBuf, "ERROR") != NULL)
			{
				Usart_SendString(USART2, (unsigned char *)cmd, strlen((const char *)cmd));
				
				continue;
			}
			else
			{
				DelayXms(500);Iwdg_Feed();
				
				Usart_SendString(USART2, (unsigned char *)cmd, strlen((const char *)cmd));
			}
		}
		
		DelayXms(10);
		Iwdg_Feed();
	}

}
