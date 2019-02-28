/**
	************************************************************
	************************************************************
	************************************************************
	*	文件名： 	main.c
	*
	*	作者： 		张继瑞
	*
	*	日期： 		2017-05-08
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

//网络协议层
#include "onenet.h"

//网络设备
#include "esp8266.h"

//硬件驱动
#include "delay.h"
#include "led.h"
#include "usart.h"

//C库
#include <string.h>


extern unsigned char data_bytes;

extern unsigned char m_cmd, rev_cmd_cnt;

extern unsigned char r_addr, r_len;


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
	
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);	//中断控制器分组设置

	Delay_Init();									//systick初始化
	
	Usart1_Init(115200);							//串口1，打印信息用
	
	Usart2_Init(115200);							//串口2，驱动ESP8266用
	
	Led_Init();										//LED初始化
	
	UsartPrintf(USART_DEBUG, " Hardware init OK\r\n");
	
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
	
	unsigned char *dataPtr = NULL;
	
	Hardware_Init();				//初始化外围硬件
	
	ESP8266_Init();					//初始化ESP8266
	
	while(OneNet_DevLink())			//接入OneNET
		DelayXms(500);
	
	while(1)
	{
		
		dataPtr = ESP8266_GetIPD(0);
		if(dataPtr != NULL)
			OneNet_RevPro(dataPtr, data_bytes);
		
		if(rev_cmd_cnt > 0)											//如果收到平台的查询命令
		{
			--rev_cmd_cnt;
			
			//根据命令码、寄存器地址、寄存器地址长度吧要上传的值放入链表里
			//本例程命令码为3代表查询传感器数据，为4代表控制相关外设
			switch(m_cmd)
			{
				case 4:												//控制外设
				{
					switch(r_addr)
					{
						//打开红色LED：{"cmd":"0104000100010A60"}
						case 1: Led5_Set(LED_ON); break;
						//关闭红色LED：{"cmd":"0104000200010A90"}
						case 2: Led5_Set(LED_OFF); break;
						
						//打开绿色LED：{"cmd":"010400030001CAC1"}
						case 3: Led4_Set(LED_ON); break;
						//关闭绿色LED：{"cmd":"0104000400010B70"}
						case 4: Led4_Set(LED_OFF); break;
						
						//打开黄色LED：{"cmd":"010400050001CB21"}
						case 5: Led3_Set(LED_ON); break;
						//关闭黄色LED：{"cmd":"010400060001CBD1"}
						case 6: Led3_Set(LED_OFF); break;
						
						//打开蓝色LED：{"cmd":"0104000700010B80"}
						case 7: Led2_Set(LED_ON); break;
						//关闭蓝色LED：{"cmd":"01040008000108B0"}
						case 8: Led2_Set(LED_OFF); break;
						
						default: break;
					}
				}
				break;
				
				default:
				break;
			}
		}
		
		DelayXms(10);
	
	}

}
