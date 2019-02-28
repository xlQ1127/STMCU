#include "includes.h" //ucos

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
#include "stmflash.h"

#include "esp8266.h"
#include "m6311.h"

#include <string.h>



//看门狗任务
#define IWDG_TASK_PRIO		7
#define IWDG_STK_SIZE		64
OS_STK IWDG_TASK_STK[IWDG_STK_SIZE];
void IWDG_Task(void *pdata);

//按键任务
#define KEY_TASK_PRIO		8
#define KEY_STK_SIZE		512
__align(8) OS_STK KEY_TASK_STK[KEY_STK_SIZE]; //UCOS使用浮点数进行printf、sprintf时一定要8字节对齐
void KEY_Task(void *pdata);

//心跳任务
#define HEART_TASK_PRIO		9
#define HEART_STK_SIZE		256
__align(8) OS_STK HEART_TASK_STK[HEART_STK_SIZE]; //UCOS使用浮点数进行printf、sprintf时一定要8字节对齐
void HEART_Task(void *pdata);

//串口任务
#define USART_TASK_PRIO		10
#define USART_STK_SIZE		1024
__align(8) OS_STK USART_TASK_STK[USART_STK_SIZE]; //UCOS使用浮点数进行printf、sprintf时一定要8字节对齐
void USART_Task(void *pdata);

//传感器任务
#define SENSOR_TASK_PRIO	11
#define SENSOR_STK_SIZE		512
__align(8) OS_STK SENSOR_TASK_STK[SENSOR_STK_SIZE]; //UCOS使用浮点数进行printf、sprintf时一定要8字节对齐
void SENSOR_Task(void *pdata);

//AP设置密码任务
#define APMODE_TASK_PRIO	6 //优先级要比较高才行，我设置为12的时候，会有一定几率出现连接不到手机server上
#define APMODE_STK_SIZE		512
OS_STK APMODE_TASK_STK[APMODE_STK_SIZE]; //
void APMODE_Task(void *pdata);






#define WIFI_GPRS	1 //1-wifi		0-gprs




void Hardware_Init(void)
{
	
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);

	Delay_Init();
	
	Led_Init();
	
	Key_Init();
	
	Lcd1602_Init();
	
	//Usart1_Init(115200); //初始化串口   115200bps
	Usart2_Init(115200); //初始化串口   115200bps
	
	IIC_Init();
	
	ADXL345_Init();
	
	GY30_Init();
	
	Iwdg_Init(4, 1250); //64分频，每秒625次，重载1250次，2s
	
#if(WIFI_GPRS == 1)

	if(Flash_NeedErase()) //如果有数据
	{
		memset(esp8266Info.staName, 0, sizeof(esp8266Info.staName)); //清除之前的内容
		Flash_Read(SSID_ADDRESS, esp8266Info.staName, sizeof(esp8266Info.staName)); //读出ssid
		Lcd1602_DisString(0x80, "%s", esp8266Info.staName); //显示
		DelayXms(10); //延时，也许不是必须的
		
		memset(esp8266Info.staPass, 0, sizeof(esp8266Info.staPass)); //清除之前的内容
		Flash_Read(PSWD_ADDRESS, esp8266Info.staPass, sizeof(esp8266Info.staPass)); //读出password
		Lcd1602_DisString(0xC0, "%s", esp8266Info.staPass); //显示

		DelayMs(1500); //延时提示
		Iwdg_Feed();
	}
	else //没有数据
	{
		Lcd1602_DisString(0x80, "No Wifi Info in Flash");
		DelayMs(1500); //延时提示
		Iwdg_Feed();
	}
	
	ESP8266_Mode(); //这个函数会先使用sta client连接路由和平台，如果其中任何一个出错就会使用ap client(先用手机设置里边的wifi连接ap，
					//然后用网络通讯调试器监测端口，等待连上)，连接到手机APP上，根据提示输入对应的信息。
	
#elif(WIFI_GPRS == 0)
	
	Lcd1602_DisString(0x80, "Wait M6311...   ");
	
	M6311_Init();
	esp8266Info.netWork = 1; //互联网模式
	
	Lcd1602_DisString(0x80, "M6311 OK        ");
	
	DelayMs(1500);
	
#endif
	
	if(esp8266Info.netWork) //如果是ap模式，就不发送连接平台的数据
		OneNet_DevLink(DEVICEID, APIKEY);
	
	if(RCC_GetFlagStatus(RCC_FLAG_IWDGRST) == SET) //如果是看门狗复位则提示
	{
		RCC_ClearFlag();
		
		Lcd1602_DisString(0x8F, "1");
	}
	
	Timer1_8_Init(TIM8, 300, 3599);
	
	UCOS_TimerInit();

}

int main(void)
{
	
	Hardware_Init();

	OSInit();
	
	OSTaskCreate(IWDG_Task, (void *)0, (OS_STK*)&IWDG_TASK_STK[IWDG_STK_SIZE - 1], IWDG_TASK_PRIO);
	
	OSTaskCreate(KEY_Task, (void *)0, (OS_STK*)&KEY_TASK_STK[KEY_STK_SIZE - 1], KEY_TASK_PRIO);
	
	OSTaskCreate(HEART_Task, (void *)0, (OS_STK*)&HEART_TASK_STK[HEART_STK_SIZE - 1], HEART_TASK_PRIO);
	
	OSTaskCreate(USART_Task, (void *)0, (OS_STK*)&USART_TASK_STK[USART_STK_SIZE - 1], USART_TASK_PRIO);
	
	OSTaskCreate(SENSOR_Task, (void *)0, (OS_STK*)&SENSOR_TASK_STK[SENSOR_STK_SIZE - 1], SENSOR_TASK_PRIO);
	
	OSTaskCreate(APMODE_Task, (void *)0, (OS_STK*)&APMODE_TASK_STK[APMODE_STK_SIZE - 1], APMODE_TASK_PRIO);
	
	OSStart();

}

void IWDG_Task(void *pdata)
{

	while(1)
	{
	
		Iwdg_Feed();
		
		OSTimeDly(50);
	
	}

}

void KEY_Task(void *pdata)
{

	while(1)
	{
	
		switch(Keyboard())
		{
			case KEY0DOWN:
			
				++ledStatus.LedRedSta;
				ledStatus.LedRedSta %= 300;
				TIM_SetCompare2(TIM8, ledStatus.LedRedSta);
			
			break;
			
			case KEY1DOWN:
				
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
				
				if(esp8266Info.netWork) //只有连接到互联网才启动数据发送
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
				
				if(esp8266Info.netWork) //只有连接到互联网才启动数据发送
					OneNet_SendData();
			
			break;
		}
		
		OSTimeDly(10);
	
	}

}

void HEART_Task(void *pdata)
{

	while(1)
	{
	
		if(esp8266Info.netWork) //只有连接到互联网才启动心跳和数据间隔发送
		{
			OSTimeDlyHMSM(0, 0, 30, 0);
			OneNet_SendData();
			
			OSTimeDlyHMSM(0, 0, 30, 0);
			HeartBeat(&usart2Info);
		}
		else //当使用AP模式时，这里引导设置密码
		{
			OSTimeDly(20);
		}
	
	}

}

void USART_Task(void *pdata)
{

	while(1)
	{
	
		UsartReciveFlag(&usart2Info);
		if(usart2Info.usartReceiveFlag == REV_OK)
		{
			usart2Info.usartReceiveFlag = REV_WAIT;
			
			if(esp8266Info.netWork) //如果是sta模式，则收到的数据用于开灯之类的
			{
				EDPKitCmd(&usart2Info);
				
				OneNetApp(&usart2Info);
			}
			else //如果是ap模式
			{
				if(esp8266Info.ssidOK == 1) //接收到ssid
				{
					memset(esp8266Info.staName, 0, sizeof(esp8266Info.staName)); //清除之前的内容
					memcpy(esp8266Info.staName, usart2Info.usartBuf, sizeof(esp8266Info.staName));
					
					//写入前线擦除片区
					FLASH_Unlock();	//解锁	必须解锁才能擦除
					FLASH_ErasePage(SSID_ADDRESS); //擦除整个页
					FLASH_Lock(); //上锁
					
					//保存参数
					Flash_Write(SSID_ADDRESS, esp8266Info.staName, strlen(esp8266Info.staName)); //写入ssid
					
					esp8266Info.ssidOK = 2; //完成ssid设置第二步
				}
				else if(esp8266Info.pswdOK == 1) //接收到pswd
				{
					memset(esp8266Info.staPass, 0, sizeof(esp8266Info.staPass)); //清除之前的内容
					memcpy(esp8266Info.staPass, usart2Info.usartBuf, sizeof(esp8266Info.staPass));
					//保存参数
					Flash_Write(PSWD_ADDRESS, esp8266Info.staPass, strlen(esp8266Info.staPass)); //写入password
					
					esp8266Info.pswdOK = 2; //完成pswd设置第二步
				}
			}
			
			Usart2_RcvClr();
		}
		
		OSTimeDly(2);
	
	}

}

void SENSOR_Task(void *pdata) //
{

	while(1)
	{
		
		if(esp8266Info.netWork)
		{
			TIM_Cmd(TIM6, DISABLE);
		
			ADXL345_GetValue();Iwdg_Feed();
			GY30_GetValue();Iwdg_Feed();
			SHT20_GetValue();Iwdg_Feed();
			
			Lcd1602_DisString(0x80, "X%0.1f,Y%0.1f,Z%0.1f", adxlInfo.incidence_Xf, adxlInfo.incidence_Yf, adxlInfo.incidence_Zf);
			Lcd1602_DisString(0xC0, "%0.1fC,%0.1f%%,%dLX", sht20Info.tempreture, sht20Info.humidity, gy30Info.lightVal);
			
			TIM_Cmd(TIM6, ENABLE);
		}
		
		OSTimeDly(100);
	
	}

}

void APMODE_Task(void *pdata)
{

	while(1)
	{
	
		if(esp8266Info.netWork == 0) //当使用AP模式时，这里引导设置密码
		{
			if(esp8266Info.ssidOK == 0)
			{
				UsartPrintf(USART2, "请输入Wifi名字\r\n"); //可选的提示
				
				Lcd1602_Clear(0x80); //清除第一行显示内容
				Lcd1602_DisString(0x80, "Enter SSID"); //tip
				
				esp8266Info.ssidOK = 1; //完成ssid设置第一步
			}
			else if(esp8266Info.ssidOK == 2)
			{
				UsartPrintf(USART2, "Wifi名字: %s\r\n", esp8266Info.staName); //可选的提示
				
				Lcd1602_Clear(0x80); //清除第一行显示内容
				Lcd1602_DisString(0x80, "%s", esp8266Info.staName); //显示输入的ssid
				
				esp8266Info.ssidOK = 3; //完成ssid设置最后一步
			}
			
			else if(esp8266Info.pswdOK == 0 && esp8266Info.ssidOK == 3) //如果ssid设置完毕，则开始pswd的设置
			{
				UsartPrintf(USART2, "请输入Wifi密码\r\n"); //可选的提示
				
				Lcd1602_Clear(0xC0); //清除第二行显示内容
				Lcd1602_DisString(0xC0, "Enter PassWord"); //tip
				
				esp8266Info.pswdOK = 1; //完成pswd设置第一步
			}
			else if(esp8266Info.pswdOK == 2 && esp8266Info.ssidOK == 3)
			{
				UsartPrintf(USART2, "Wifi密码: %s\r\n", esp8266Info.staPass); //可选的提示
				
				Lcd1602_Clear(0xC0); //清除第二行显示内容
				Lcd1602_DisString(0xC0, "%s", esp8266Info.staPass); //tip
				
				esp8266Info.pswdOK = 3; //完成pswd设置最后一步
				
				OSTimeDlyHMSM(0, 0, 5, 0); //切换任务5s左右，这里当提示使用
				Lcd1602_Clear(0xFF); //清除两行显示内容
			}
			
			if(esp8266Info.pswdOK == 3 && esp8266Info.ssidOK == 3) //当ssid和pswd都设置完成
			{
				TIM_Cmd(TIM2, DISABLE); //关闭定时器
				
				ESP8266_Mode(); //自动选择模式，119行有描述
				
				if(esp8266Info.netWork) //是sta模式了，代表接入了互联网，则连接平台
					OneNet_DevLink(DEVICEID, APIKEY);
				else //如果还是AP模式，代表重新传入的ssid和password还是错的，则要对引导变量清零初始化
				{
					esp8266Info.ssidOK = 0; //清理重新引导
					esp8266Info.pswdOK = 0; //清理重新引导
				}
				
				TIM_Cmd(TIM2, ENABLE);
			}
		}
		
		OSTimeDly(20);
	
	}

}
