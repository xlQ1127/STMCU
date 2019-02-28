/**
	************************************************************
	************************************************************
	************************************************************
	*	文件名： 	fault.c
	*
	*	作者： 		张继瑞
	*
	*	日期： 		2016-11-23
	*
	*	版本： 		V1.0
	*
	*	说明： 		网络错误处理
	*
	*	修改记录：	
	************************************************************
	************************************************************
	************************************************************
**/

//协议
#include "onenet.h"
#include "fault.h"

//网络设备
#include "net_device.h"

//硬件驱动
#include "usart.h"
#include "selfcheck.h"


NET_FAULT_INFO net_fault_info;


//==========================================================
//	函数名称：	Fault_Process
//
//	函数功能：	错误处理
//
//	入口参数：	无
//
//	返回参数：	无
//
//	说明：		
//==========================================================
void NET_Fault_Process(void)
{

	switch(net_fault_info.net_fault_level)
	{
		case NET_FAULT_LEVEL_0:
		
			UsartPrintf(USART_DEBUG, "WARN:	NET_FAULT_LEVEL_0\r\n");
		
		break;
		
		case NET_FAULT_LEVEL_1:											//错误等级1-负责重连IP
		
			UsartPrintf(USART_DEBUG, "WARN:	NET_FAULT_LEVEL_1\r\n");
		
			if(NET_DEVICE_ReLink(onenet_info.ip, onenet_info.port) == 0)
			{
				if(onenet_info.protocol == 10)							//如果是HTTP协议，IP连接成功即可
					onenet_info.netWork = 1;
				else													//其他协议需要登录
					onenet_info.netWork = 0;
				
				net_fault_info.net_fault_level = NET_FAULT_LEVEL_0;		//将错误等级降至最低
			}
			else if(++net_fault_info.net_fault_count >= 3)				//如果3次未连上
			{
				net_fault_info.net_fault_count = 0;
				
				net_fault_info.net_fault_level = NET_FAULT_LEVEL_2;		//将错误等级升高
			}
		
		break;
		
		case NET_FAULT_LEVEL_2:											//错误等级2-负责重新注册网络
		
			UsartPrintf(USART_DEBUG, "WARN:	NET_FAULT_LEVEL_2\r\n");
			
			check_info.NET_DEVICE_OK = DEV_ERR;
			NET_DEVICE_ReConfig(0);
			
			net_device_info.netWork = 0;
			onenet_info.netWork = 0;
			
			net_fault_info.net_fault_level = NET_FAULT_LEVEL_0;			//将错误等级降至最低
		
		break;
		
		case NET_FAULT_LEVEL_3:											//错误等级3-负责复位网络设备
		
			UsartPrintf(USART_DEBUG, "WARN:	NET_FAULT_LEVEL_3\r\n");
			
			NET_DEVICE_Reset();
			
			net_fault_info.net_fault_level = NET_FAULT_LEVEL_2;			//将错误等级降低
		
		break;
		
		case NET_FAULT_LEVEL_4:											//错误等级4-负责给网络设备重新上电
		
			UsartPrintf(USART_DEBUG, "WARN:	NET_FAULT_LEVEL_4\r\n");
		
			//power off
			
			//power on
		
			net_fault_info.net_fault_level = NET_FAULT_LEVEL_3;			//将错误等级降低
		
		break;
		
		case NET_FAULT_LEVEL_5:
		
			UsartPrintf(USART_DEBUG, "WARN:	NET_FAULT_LEVEL_5\r\n");
		
			net_fault_info.net_fault_level = NET_FAULT_LEVEL_0;
		
		break;
		
		default:
		break;
	}

}
