/**
	************************************************************
	************************************************************
	************************************************************
	*	文件名： 	main.c
	*
	*	作者： 		张继瑞
	*
	*	日期： 		2017-01-011
	*
	*	版本： 		V1.0
	*
	*	说明： 		接入onenet，上传数据和命令控制
	*
	*	修改记录：	
	************************************************************
	************************************************************
	************************************************************
**/

//单片机头文件
#include "stm32f10x.h"

//框架
#include "framework.h"

//网络协议层
#include "onenet.h"
#include "fault.h"

//网络设备
#include "net_device.h"

//硬件驱动
#include "led.h"
#include "delay.h"
#include "usart.h"
#include "hwtimer.h"
#include "i2c.h"
#include "sht20.h"
#include "at24c02.h"
#include "selfcheck.h"
#include "rtc.h"

//中文数据流
#include "dataStreamName.h"

//C库
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <time.h>


#define NET_COUNT	7			//错误计数

#define NET_TIME	60			//设定时间--单位秒

unsigned short timerCount = 0;	//时间计数--单位秒


char myTime[24];


//数据流
DATA_STREAM dataStream[] = {
								{ZW_REDLED, &led_status.Led5Sta, TYPE_BOOL, 1},
								{ZW_GREENLED, &led_status.Led4Sta, TYPE_BOOL, 1},
								{ZW_YELLOWLED, &led_status.Led3Sta, TYPE_BOOL, 1},
								{ZW_BLUELED, &led_status.Led2Sta, TYPE_BOOL, 1},
								{ZW_TEMPERATURE, &sht20_info.tempreture, TYPE_FLOAT, 1},
								{ZW_HUMIDITY, &sht20_info.humidity, TYPE_FLOAT, 1},
								{ZW_TIME, myTime, TYPE_STRING, 1},
								{"GPS", &gps, TYPE_GPS, 0},
								{ZW_ERRTYPE, &net_fault_info.net_fault_level_r, TYPE_UCHAR, 1},
							};
unsigned char dataStreamCnt = sizeof(dataStream) / sizeof(dataStream[0]);


/*
************************************************************
*	函数名称：	Hardware_Init
*
*	函数功能：	硬件初始化
*
*	入口参数：	无
*
*	返回参数：	无
*
*	说明：		初始化单片机功能以及外接设备
************************************************************
*/
void Hardware_Init(void)
{
	
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);								//中断控制器分组设置

	Delay_Init();																//systick初始化
	
	Usart1_Init(115200); 														//初始化串口   115200bps
#if(USART_DMA_RX_EN)
	USARTx_ResetMemoryBaseAddr(USART_DEBUG, (unsigned int)alterInfo.alterBuf, sizeof(alterInfo.alterBuf), USART_RX_TYPE);
#endif
	
	Led_Init();																	//LED初始化
	
	IIC_Init();																	//软件IIC总线初始化
	
	RTC_Init();																	//初始化RTC
	
	Check_PowerOn(); 															//上电自检

	if(RCC_GetFlagStatus(RCC_FLAG_IWDGRST) == SET) 								//如果是看门狗复位则提示
	{
		UsartPrintf(USART_DEBUG, "WARN:	IWDG Reboot\r\n");
		
		RCC_ClearFlag();														//清除看门狗复位标志位
		
		net_fault_info.net_fault_level = net_fault_info.net_fault_level_r
														= NET_FAULT_LEVEL_5;	//错误等级5
		
		net_device_info.reboot = 1;
	}
	else
	{
		UsartPrintf(USART_DEBUG, "2.DEVID: %s,     APIKEY: %s\r\n"
								, onenet_info.devID, onenet_info.apiKey);
		
		net_device_info.reboot = 0;
	}
	
	//Iwdg_Init(4, 1250); 														//64分频，每秒625次，重载1250次，2s
	
	Timer3_4_Init(TIM3, 49, 35999);												//72MHz，36000分频-500us，50重载值。则中断周期为500us * 50 = 25ms
	
	UsartPrintf(USART_DEBUG, "3.Hardware init OK\r\n");							//提示初始化完成

}

/*
************************************************************
*	函数名称：	USART_Task
*
*	函数功能：	处理平台下发的命令
*
*	入口参数：	void类型的参数指针
*
*	返回参数：	无
*
*	说明：		串口接收任务。在数据模式下时，等待平台下发的命令并解析、处理
************************************************************
*/
void USART_Task(void)
{
	
	if(onenet_info.cmd_ptr)
	{
		OneNet_RevPro(onenet_info.cmd_ptr);
		
		onenet_info.cmd_ptr = NULL;
	}

}

/*
************************************************************
*	函数名称：	STATUS_Task
*
*	函数功能：	状态检测
*
*	入口参数：	无
*
*	返回参数：	无
*
*	说明：		
************************************************************
*/
void STATUS_Task(void)
{

	OneNet_Status();			//状态检查

}

/*
************************************************************
*	函数名称：	SEND_Task
*
*	函数功能：	上传传感器数据
*
*	入口参数：	void类型的参数指针
*
*	返回参数：	无
*
*	说明：		数据发送任务
************************************************************
*/
void SEND_Task(void)
{

	onenet_info.sendData = SEND_TYPE_DATA;		//上传数据到平台

}

/*
************************************************************
*	函数名称：	SENSOR_Task
*
*	函数功能：	传感器数据采集、显示
*
*	入口参数：	无
*
*	返回参数：	无
*
*	说明：		传感器数据采集任务。进行外接传感器的数据采集、读取、显示
************************************************************
*/
void SENSOR_Task(void)
{
	
	if(check_info.SHT20_OK == DEV_OK) 									//只有设备存在时，才会读取值和显示
	{
		SHT20_GetValue();												//采集传感器数据
	}

}

/*
************************************************************
*	函数名称：	DATA_Task
*
*	函数功能：	数据发送主任务
*
*	入口参数：	无
*
*	返回参数：	无
*
*	说明：		
************************************************************
*/
void DATA_Task(void)
{
	
	switch(onenet_info.sendData)
	{
		case SEND_TYPE_DATA:
			
			onenet_info.sendData = OneNet_SendData(FORMAT_TYPE3, onenet_info.devID, onenet_info.apiKey, dataStream, dataStreamCnt);//上传数据到平台

		break;
	}

}

/*
************************************************************
*	函数名称：	DATALIST_Task
*
*	函数功能：	循环发送链表里边待发送的数据块
*
*	入口参数：	无
*
*	返回参数：	无
*
*	说明：		
************************************************************
*/
void DATALIST_Task(void)
{

	if(NET_DEVICE_CheckListHead())
	{
		NET_DEVICE_SendData(NET_DEVICE_GetListHeadBuf(), NET_DEVICE_GetListHeadLen());
		NET_DEVICE_DeleteDataSendList();
	}

}

/*
************************************************************
*	函数名称：	FAULT_Task
*
*	函数功能：	网络状态错误处理
*
*	入口参数：	无
*
*	返回参数：	无
*
*	说明：		故障处理任务。当发生网络错误、设备错误时，会标记对应标志位，然后集中进行处理
************************************************************
*/
void FAULT_Task(void)
{

	if(net_fault_info.net_fault_level != NET_FAULT_LEVEL_0)					//如果错误标志被设置
	{
		UsartPrintf(USART_DEBUG, "WARN:	NET Fault Process\r\n");
		
		NET_Fault_Process();												//进入错误处理函数
	}

}

/*
************************************************************
*	函数名称：	NET_Task
*
*	函数功能：	网络连接、平台接入
*
*	入口参数：	无
*
*	返回参数：	无
*
*	说明：		网络连接任务任务。会在心跳检测里边检测网络连接状态，如果有错，会标记状态，然后在这里进行重连
************************************************************
*/
void NET_Task(void)
{

	if(onenet_info.netWork == 0)
	{
		if(!onenet_info.netWork && (check_info.NET_DEVICE_OK == DEV_OK))		//当没有网络 且 网络模块检测到时
		{
			if(!NET_DEVICE_Init(onenet_info.protocol, onenet_info.ip, onenet_info.port))//初始化网络设备，能连入网络
			{
				onenet_info.netWork = 1;

				if(gps.flag)
					dataStream[7].flag = 1;									//GPS就绪，准备上传
			}
		}
		if(check_info.NET_DEVICE_OK == DEV_ERR) 								//当网络设备未做检测
		{
			if(!NET_DEVICE_Exist())											//网络设备检测
			{
				UsartPrintf(USART_DEBUG, "NET Device :Ok\r\n");
				check_info.NET_DEVICE_OK = DEV_OK;							//检测到网络设备，标记
			}
			else
				UsartPrintf(USART_DEBUG, "NET Device :Error\r\n");
		}
	}

}

/*
************************************************************
*	函数名称：	CLOCK_Task
*
*	函数功能：	网络校时、时间显示
*
*	入口参数：	无
*
*	返回参数：	无
*
*	说明：		
************************************************************
*/
void CLOCK_Task(void)
{
	
#if(NET_TIME_EN == 1)
	static unsigned int second = 0, second_pre = 0, err_count = 0;	//second是实时时间，second_pre差值比较。err_count获取计时
	static struct tm *time;
	static _Bool get_net_time = 1;
#endif

#if(NET_TIME_EN == 1)
	if(get_net_time)												//需要获取时间
	{
		dataStream[6].flag = 0;										//不上传时间
		
		if(FW_GetTicks() - err_count >= 24000)						//十分钟还获取不到则重新获取(25ms一次)
		{
			err_count = 0;
			net_device_info.net_time = 0;
			onenet_info.netWork = 0;
			NET_DEVICE_ReConfig(0);
		}
		
		if(net_device_info.net_time)
		{
			second = RTC_GetCounter();
			
			if(((net_device_info.net_time <= second + 300) && (net_device_info.net_time >= second - 300)) || (second <= 100))
			{														//如果在±5分钟内，则认为时间正确
				RTC_SetTime(net_device_info.net_time + 4);			//设置RTC时间，加4是补上大概的时间差
				
				get_net_time = 0;
				err_count = 0;
				
				dataStream[6].flag = 1;								//上传时间
			}
		}
	}
	
	second = RTC_GetCounter();										//获取秒值
	
	if(second > second_pre)
	{
		second_pre = second;
		time = localtime((const time_t *)&second);					//将秒值转为tm结构所表示的时间
		
		memset(myTime, 0, sizeof(myTime));
		snprintf(myTime, sizeof(myTime), "%d-%d-%d %d:%d:%d",
						time->tm_year + 1900, time->tm_mon + 1, time->tm_mday,
						time->tm_hour, time->tm_min, time->tm_sec);
	
		if(time->tm_hour == 0 && time->tm_min == 0 && time->tm_sec == 0)//每天0点时，更新一次时间
		{
			get_net_time = 1;
			err_count = FW_GetTicks();
			net_device_info.net_time = 0;
			onenet_info.netWork = 0;
			NET_DEVICE_ReConfig(0);
		}
	}
#endif

}

/*
************************************************************
*	函数名称：	NET_Timer
*
*	函数功能：	定时检查网络状态标志位
*
*	入口参数：	无
*
*	返回参数：	无
*
*	说明：		定时器任务。定时检查网络状态，若持续超过设定时间无网络连接，则进行平台重连
************************************************************
*/
void NET_Timer(void)
{
	
	if(onenet_info.errCount >= NET_COUNT)								//如果发送错误计数达到NET_COUNT次
	{
		UsartPrintf(USART_DEBUG, "Tips:	Timer Check Err-Send\r\n");
		
		onenet_info.errCount = 0;
		
		net_fault_info.net_fault_level = NET_FAULT_LEVEL_1;				//错误等级1
	}
	
	if(onenet_info.netWork == 0)											//如果在规定时间内网络还未接入成功
	{
		if(++timerCount >= NET_TIME) 									//如果网络断开超时
		{
			UsartPrintf(USART_DEBUG, "Tips:	Timer Check Err-Init\r\n");
			
			timerCount = 0;
			
			net_fault_info.net_fault_level = NET_FAULT_LEVEL_3;			//错误等级3
		}
	}
	else
	{
		timerCount = 0;													//清除计数
	}

}

/*
************************************************************
*	函数名称：	main
*
*	函数功能：	
*
*	入口参数：	无
*
*	返回参数：	0
*
*	说明：		
************************************************************
*/
int main(void)
{

	Hardware_Init();									//硬件初始化
	
	NET_DEVICE_IO_Init();								//网络设备IO初始化
	NET_DEVICE_Reset();									//网络设备复位
	
	FW_Init();											//框架层初始化
														//创建任务
	FW_CreateTask(USART_Task, 4);
	
	FW_CreateTask(STATUS_Task, 4000);
	
	FW_CreateTask(SEND_Task, 3000);
	
	FW_CreateTask(SENSOR_Task, 100);
	
	FW_CreateTask(DATA_Task, 10);
	
	FW_CreateTask(DATALIST_Task, 100);
	
	FW_CreateTask(FAULT_Task, 10);
	
	FW_CreateTask(NET_Task, 10);
	
	FW_CreateTask(CLOCK_Task, 20);
	
	FW_CreateTask(NET_Timer, 20);
	
	UsartPrintf(USART_DEBUG, "Running...\r\n");
	
	FW_StartSchedule();									//开始任务调度

}
