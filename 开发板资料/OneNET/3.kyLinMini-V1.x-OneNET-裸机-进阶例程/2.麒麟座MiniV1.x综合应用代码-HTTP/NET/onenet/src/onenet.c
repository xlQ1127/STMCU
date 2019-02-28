/**
	************************************************************
	************************************************************
	************************************************************
	*	文件名： 	onenet.c
	*
	*	作者： 		张继瑞
	*
	*	日期： 		2017-05-27
	*
	*	版本： 		V1.0
	*
	*	说明： 		OneNET平台应用示例
	*
	*	修改记录：	
	************************************************************
	************************************************************
	************************************************************
**/

//单片机头文件
#include "stm32f10x.h"

//网络设备
#include "net_device.h"

//协议文件
#include "onenet.h"
#include "fault.h"
#include "httpkit.h"

//硬件驱动
#include "usart.h"

//图片数据文件
#include "image_2k.h"

//C库
#include <string.h>
#include <stdio.h>


ONETNET_INFO onenet_info = {"5616708", "kPl=kf99QxL2acVvVCHssUPWZKs=",
							"183.230.40.33", "80",
							10,
							NULL, 0, 0, 0, 0};



//==========================================================
//	函数名称：	OneNet_SendData
//
//	函数功能：	上传数据到平台
//
//	入口参数：	type：发送数据的格式
//				devid：设备ID
//				apikey：设备apikey
//				streamArray：数据流
//				streamArrayNum：数据流个数
//
//	返回参数：	0-成功		1-失败
//
//	说明：		
//==========================================================
unsigned char OneNet_SendData(FORMAT_TYPE type, char *devid, char *apikey, DATA_STREAM *streamArray, unsigned short streamArrayCnt)
{
	
	HTTP_PACKET_STRUCTURE httpPacket = {NULL, 0, 0, 0};										//协议包
	
	_Bool status = SEND_TYPE_OK;
	short body_len = 0;
	
	if(!onenet_info.netWork)
		return SEND_TYPE_DATA;
	
	onenet_info.errCount++;
	
	UsartPrintf(USART_DEBUG, "Tips:	OneNet_SendData-HTTP_TYPE%d\r\n", type);
	
	body_len = DSTREAM_GetDataStream_Body_Measure(type, streamArray, streamArrayCnt, 0);	//获取当前需要发送的数据流的总长度
	if(body_len > 0)
	{
		if(HTTP_Post_PacketSaveData(devid, apikey, body_len, NULL, (SaveDataType)type, &httpPacket) == 0)
		{
			body_len = DSTREAM_GetDataStream_Body(type, streamArray, streamArrayCnt, httpPacket._data, httpPacket._size, httpPacket._len);
			
			if(body_len > 0)
			{
				httpPacket._len += body_len;
				UsartPrintf(USART_DEBUG, "Send %d Bytes\r\n", httpPacket._len);
				//NET_DEVICE_SendData(httpPacket._data, httpPacket._len);					//上传数据到平台
				NET_DEVICE_AddDataSendList(httpPacket._data, httpPacket._len);				//加入链表
			}
			else
				UsartPrintf(USART_DEBUG, "WARN:	DSTREAM_GetDataStream_Body Failed\r\n");
				
			HTTP_DeleteBuffer(&httpPacket);													//删包
		}
		else
			UsartPrintf(USART_DEBUG, "WARN:	HTTP_NewBuffer Failed\r\n");
	}
	else if(body_len < 0)
		return SEND_TYPE_OK;
	else
		status = SEND_TYPE_DATA;
	
	net_fault_info.net_fault_level_r = NET_FAULT_LEVEL_0;										//发送之后清除标记
	
	return status;
	
}

//==========================================================
//	函数名称：	OneNet_RegisterDevice
//
//	函数功能：	注册设备
//
//	入口参数：	apikey：master-key(产品APIKEY)
//				title：设备名
//				auth_info:设备的鉴权信息
//				desc：描述信息
//				Private：是否公开
//
//	返回参数：	0-成功		1-失败
//
//	说明：		非HTTP协议的设备，调用此函数前需要断开当前连接，再连接HTTP协议的ip
//==========================================================
unsigned char OneNet_RegisterDevice(const char *apikey, const char *title, const char *auth_info, const char *desc, const char *Private)
{
	
	HTTP_PACKET_STRUCTURE httpPacket = {NULL, 0, 0, 0};							//协议包

	if(!onenet_info.netWork)														//如果网络为连接 或 不为数据收发模式
		return SEND_TYPE_REGISTER;
	
	if(HTTP_Post_PacketDeviceRegister(apikey, title, auth_info, desc, Private, &httpPacket) == 0)
	{
		//NET_DEVICE_SendData(httpPacket._data, httpPacket._len);				//上传数据到平台
		NET_DEVICE_AddDataSendList(httpPacket._data, httpPacket._len);				//加入链表
		HTTP_DeleteBuffer(&httpPacket);											//删包
	}
	
	return SEND_TYPE_OK;

}

//==========================================================
//	函数名称：	OneNet_Status
//
//	函数功能：	连接状态检查
//
//	入口参数：	无
//
//	返回参数：	无
//
//	说明：		
//==========================================================
void OneNet_Status(void)
{
	
	unsigned char errType = 0;
	
	if(!onenet_info.netWork)														//如果网络为连接
		return;
	
	errType = NET_DEVICE_Check();												//网络设备状态检查
	if(errType == CHECK_CLOSED || errType == CHECK_GOT_IP)
		net_fault_info.net_fault_level = net_fault_info.net_fault_level_r = NET_FAULT_LEVEL_1;
	else if(errType == CHECK_NO_DEVICE || errType == CHECK_NO_CARD)
		net_fault_info.net_fault_level = net_fault_info.net_fault_level_r = NET_FAULT_LEVEL_3;
	else
		net_fault_info.net_fault_level = net_fault_info.net_fault_level_r = NET_FAULT_LEVEL_0;
	
}

//==========================================================
//	函数名称：	OneNET_CmdHandle
//
//	函数功能：	读取平台rb中的数据
//
//	入口参数：	无
//
//	返回参数：	无
//
//	说明：		
//==========================================================
void OneNET_CmdHandle(void)
{
	
	unsigned char *dataPtr = NULL, *ipdPtr = NULL;		//数据指针

	dataPtr = NET_DEVICE_Read();						//等待数据

	if(dataPtr != NULL)									//数据有效
	{
		ipdPtr = NET_DEVICE_GetIPD(dataPtr);			//检查是否是平台数据
		if(ipdPtr != NULL)
		{
			net_device_info.send_ok = 1;
			
			if(net_device_info.netWork)
				onenet_info.cmd_ptr = ipdPtr;			//标记平台下发命令
			else
				net_device_info.cmd_ipd = (char *)ipdPtr;
		}
		else
		{
			if(strstr((char *)dataPtr, "SEND OK") != NULL)
			{
				net_device_info.send_ok = 1;
			}
			else if(strstr((char *)dataPtr, "CLOSE") != NULL && net_device_info.netWork)
			{
				UsartPrintf(USART_DEBUG, "WARN:	连接断开，准备重连\r\n");
				
				net_fault_info.net_fault_level = net_fault_info.net_fault_level_r = NET_FAULT_LEVEL_1;
			}
			else
				NET_DEVICE_CmdHandle((char *)dataPtr);
		} 
	}

}

//==========================================================
//	函数名称：	OneNet_RevPro
//
//	函数功能：	平台返回数据检测
//
//	入口参数：	dataPtr：平台返回的数据
//
//	返回参数：	无
//
//	说明：		
//==========================================================
void OneNet_RevPro(unsigned char *dataPtr)
{
	
	char *devid = NULL;

	if(strstr((char *)dataPtr, "CLOSED"))
	{
		UsartPrintf(USART_DEBUG, "TCP CLOSED\r\n");
		
		net_fault_info.net_fault_level = net_fault_info.net_fault_level_r = NET_FAULT_LEVEL_1;
		
		onenet_info.errCount++;
	}
	else
	{
		//这里用来检测是否发送成功
		if(strstr((char *)dataPtr, "succ"))
		{
			UsartPrintf(USART_DEBUG, "Tips:	Send OK\r\n");
			
			if(strstr((char *)dataPtr, "device_id"))
			{
				if(HTTP_UnPacketDeviceRegister(dataPtr, &devid) == 0)
				{
					UsartPrintf(USART_DEBUG, "devid: %s\r\n", devid);
					HTTP_FreeBuffer(devid);
				}
			}
			
			onenet_info.errCount = 0;
		}
		else
		{
			UsartPrintf(USART_DEBUG, "Tips:	Send Err\r\n");
			onenet_info.errCount++;
		}
	}

}
