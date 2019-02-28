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
#include "edpkit.h"

//硬件驱动
#include "usart.h"
#include "delay.h"
#include "led.h"

//图片数据文件
#include "image_2k.h"

//C库
#include <string.h>
#include <stdio.h>
#include <stdlib.h>


ONETNET_INFO onenet_info = {"6580246", "iUZqKW6xIjpGngggBH=66VCGzqg=",
							"183.230.40.39", "876",
							1,
							NULL, 0, 0, 0, 1, 0};



//==========================================================
//	函数名称：	OneNet_DevLink
//
//	函数功能：	与onenet创建连接
//
//	入口参数：	devid：创建设备的devid或产品ID
//				auth_key：创建设备的masterKey或apiKey或设备鉴权信息
//
//	返回参数：	无
//
//	说明：		与onenet平台建立连接，成功或会标记oneNetInfo.netWork网络状态标志
//==========================================================
void OneNet_DevLink(const char* devid, const char* auth_key)
{
	
	EDP_PACKET_STRUCTURE edpPacket = {NULL, 0, 0, 0};				//协议包
	
	unsigned char timeOut = 200;
	
	UsartPrintf(USART_DEBUG, "OneNet_DevLink\r\n"
                        "DEVID: %s,     APIKEY: %s\r\n"
                        , devid, auth_key);

#if 1
	if(EDP_PacketConnect1(devid, auth_key, 256, &edpPacket) == 0)	//根据devid 和 apikey封装协议包
#else
	if(EDP_PacketConnect2(proid, auth_key, 256, &edpPacket) == 0)	//根据产品id 和 鉴权信息封装协议包
#endif
	
	{
		NET_DEVICE_SendData(edpPacket._data, edpPacket._len);		//上传平台
		//NET_DEVICE_AddDataSendList(edpPacket._data, edpPacket._len);//加入链表
		
		EDP_DeleteBuffer(&edpPacket);								//删包
		
		while(!onenet_info.netWork && --timeOut)
		{
			OneNet_RevPro(onenet_info.cmd_ptr);
			DelayXms(10);
		}
	}
	else
		UsartPrintf(USART_DEBUG, "WARN:	EDP_PacketConnect Failed\r\n");
	
	if(onenet_info.netWork)											//如果接入成功
	{
		onenet_info.errCount = 0;
	}
	else
	{
		if(++onenet_info.errCount >= 5)								//如果超过设定次数后，还未接入平台
		{
			onenet_info.netWork = 0;
			onenet_info.errCount = 0;
			
			net_fault_info.net_fault_level =
			net_fault_info.net_fault_level_r =
			NET_FAULT_LEVEL_3;										//错误等级3
		}
	}
	
}

//==========================================================
//	函数名称：	OneNet_PushData
//
//	函数功能：	PUSHDATA
//
//	入口参数：	dst_devid：接收设备的devid
//				data：数据内容
//				data_len：数据长度
//
//	返回参数：	0-发送成功	1-失败
//
//	说明：		设备与设备之间的通信
//==========================================================
_Bool OneNet_PushData(const char* dst_devid, const char* data, unsigned int data_len)
{
	
	EDP_PACKET_STRUCTURE edpPacket = {NULL, 0, 0, 0};							//协议包
	
	if(!onenet_info.netWork)														//如果网络未连接 或 不为数据收发模式
		return 1;
	
	if(EDP_PacketPushData(dst_devid, data, data_len, &edpPacket) == 0)
	{
		//NET_DEVICE_SendData(edpPacket._data, edpPacket._len);					//上传平台
		NET_DEVICE_AddDataSendList(edpPacket._data, edpPacket._len);			//加入链表
		
		EDP_DeleteBuffer(&edpPacket);											//删包
	}
	else
		UsartPrintf(USART_DEBUG, "WARN:	OneNet_PushData Failed\r\n");
	
	return 0;

}

//==========================================================
//	函数名称：	OneNet_SendData
//
//	函数功能：	上传数据到平台
//
//	入口参数：	type：发送数据的格式
//
//	返回参数：	SEND_TYPE_OK-发送成功	SEND_TYPE_DATA-需要重送
//
//	说明：		
//==========================================================
unsigned char OneNet_SendData(FORMAT_TYPE type, char *devid, char *apikey, DATA_STREAM *streamArray, unsigned short streamArrayCnt)
{
	
	EDP_PACKET_STRUCTURE edpPacket = {NULL, 0, 0, 0};											//协议包
	
	_Bool status = SEND_TYPE_OK;
	short body_len = 0;
	
	if(!onenet_info.netWork)
		return SEND_TYPE_DATA;
	
	if(type < 1 && type > 5)
		return SEND_TYPE_DATA;
	
	UsartPrintf(USART_DEBUG, "Tips:	OneNet_SendData-EDP_TYPE%d\r\n", type);
	
	if(type != kTypeBin)																		//二进制文件吧全部工作做好，不需要执行这些
	{
		body_len = DSTREAM_GetDataStream_Body_Measure(type, streamArray, streamArrayCnt, 0);	//获取当前需要发送的数据流的总长度
		
		if(body_len > 0)
		{
			if(EDP_PacketSaveData(devid, body_len, NULL, (SaveDataType)type, &edpPacket) == 0)	//封包
			{
				body_len = DSTREAM_GetDataStream_Body(type, streamArray, streamArrayCnt, edpPacket._data, edpPacket._size, edpPacket._len);
				
				if(body_len > 0)
				{
					edpPacket._len += body_len;
					//NET_DEVICE_SendData(edpPacket._data, edpPacket._len);						//上传数据到平台
					NET_DEVICE_AddDataSendList(edpPacket._data, edpPacket._len);				//加入链表
					UsartPrintf(USART_DEBUG, "Send %d Bytes\r\n", edpPacket._len);
				}
				else
					UsartPrintf(USART_DEBUG, "WARN:	DSTREAM_GetDataStream_Body Failed\r\n");
				
				EDP_DeleteBuffer(&edpPacket);													//删包
			}
			else
				UsartPrintf(USART_DEBUG, "WARN:	EDP_NewBuffer Failed\r\n");
		}
		else if(body_len < 0)
			return SEND_TYPE_OK;
		else
			status = SEND_TYPE_DATA;
	}
	else
	{
		OneNet_SendData_Picture(devid, Array, sizeof(Array));
	}
	
	net_fault_info.net_fault_level_r = NET_FAULT_LEVEL_0;										//发送之后清除标记
	
	return status;
	
}

//==========================================================
//	函数名称：	OneNet_SendData_EDPType2
//
//	函数功能：	上传二进制数据到平台
//
//	入口参数：	devid：设备ID(推荐为NULL)
//				picture：图片数据
//				pic_len：图片数据长度
//
//	返回参数：	无
//
//	说明：		若是低速设备，数据量大时，建议使用网络设备的透传模式
//				上传图片是，强烈建议devid字段为空，否则平台会将图片数据下发到设备
//==========================================================
#define PKT_SIZE 1024
void OneNet_SendData_Picture(char *devid, const char* picture, unsigned int pic_len)
{
	
	EDP_PACKET_STRUCTURE edpPacket = {NULL, 0, 0, 0};					//协议包

	char type_bin_head[] = "{\"ds_id\":\"pic\"}";						//图片数据头
	unsigned char *pImage = (unsigned char *)picture;
	
	if(EDP_PacketSaveData(devid, pic_len, type_bin_head, kTypeBin, &edpPacket) == 0)
	{	
		UsartPrintf(USART_DEBUG, "Send %d Bytes\r\n", edpPacket._len);
		NET_DEVICE_SendData(edpPacket._data, edpPacket._len);			//上传数据到平台
		
		EDP_DeleteBuffer(&edpPacket);									//删包
		
		UsartPrintf(USART_DEBUG, "image len = %d\r\n", pic_len);
		
		while(pic_len > 0)
		{
			DelayXms(300);												//传图时，时间间隔会大一点，这里额外增加一个延时
			
			if(pic_len >= PKT_SIZE)
			{
				NET_DEVICE_SendData(pImage, PKT_SIZE);					//串口发送分片
				
				pImage += PKT_SIZE;
				pic_len -= PKT_SIZE;
			}
			else
			{
				NET_DEVICE_SendData(pImage, (unsigned short)pic_len);	//串口发送最后一个分片
				pic_len = 0;
			}
		}
		
		UsartPrintf(USART_DEBUG, "image send ok\r\n");
	}
	else
		UsartPrintf(USART_DEBUG, "EDP_PacketSaveData Failed\r\n");

}

//==========================================================
//	函数名称：	OneNet_HeartBeat
//
//	函数功能：	发送心跳请求
//
//	入口参数：	无
//
//	返回参数：	SEND_TYPE_OK-发送成功	SEND_TYPE_HEART-需要重送
//
//	说明：		
//==========================================================
unsigned char OneNet_SendData_Heart(void)
{
	
	EDP_PACKET_STRUCTURE edpPacket = {NULL, 0, 0, 0};		//协议包
	
	if(!onenet_info.netWork)									//如果网络为连接 或 不为数据收发模式
		return SEND_TYPE_HEART;
	
	if(EDP_PacketPing(&edpPacket))
		return SEND_TYPE_HEART;
	
	onenet_info.heartBeat = 0;
	
	//NET_DEVICE_SendData(edpPacket._data, edpPacket._len);	//向平台上传心跳请求
	NET_DEVICE_AddDataSendList(edpPacket._data, edpPacket._len);//加入链表
	
	EDP_DeleteBuffer(&edpPacket);							//删包
	
	return SEND_TYPE_OK;
	
}

//==========================================================
//	函数名称：	OneNet_HeartBeat_Check
//
//	函数功能：	发送心跳后的心跳检测
//
//	入口参数：	无
//
//	返回参数：	0-成功	1-等待
//
//	说明：		基于调用时基，runCount每隔此函数调用一次的时间自增
//				达到设定上限检测心跳标志位是否就绪
//				上限时间可以不用太精确
//==========================================================
_Bool OneNet_Check_Heart(void)
{
	
	static unsigned char runCount = 0;
	
	if(!onenet_info.netWork)
		return 1;

	if(onenet_info.heartBeat == 1)
	{
		runCount = 0;
		onenet_info.errCount = 0;
		
		return 0;
	}
	
	if(++runCount >= 40)
	{
		runCount = 0;
		
		UsartPrintf(USART_DEBUG, "HeartBeat TimeOut: %d\r\n", onenet_info.errCount);
		onenet_info.sendData = SEND_TYPE_HEART;		//发送心跳请求
		
		if(++onenet_info.errCount >= 3)
		{
			unsigned char errType = 0;
			
			onenet_info.errCount = 0;
			
			errType = NET_DEVICE_Check();												//网络设备状态检查
			if(errType == CHECK_CONNECTED || errType == CHECK_CLOSED || errType == CHECK_GOT_IP)
				net_fault_info.net_fault_level = net_fault_info.net_fault_level_r = NET_FAULT_LEVEL_1;
			else if(errType == CHECK_NO_DEVICE || errType == CHECK_NO_CARD)
				net_fault_info.net_fault_level = net_fault_info.net_fault_level_r = NET_FAULT_LEVEL_3;
			else
				net_fault_info.net_fault_level = net_fault_info.net_fault_level_r = NET_FAULT_LEVEL_0;
		}
	}
	
	return 1;

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
void OneNet_RevPro(unsigned char *cmd)
{
	
	EDP_PACKET_STRUCTURE edpPacket = {NULL, 0, 0, 0};	//协议包
	
	char *cmdid_devid = NULL;
	unsigned short cmdid_len = 0;
	char *req = NULL;
	unsigned int req_len = 0;
	unsigned char type = 0;
	
	short result = 0;

	char *dataPtr = NULL;
	char numBuf[10];
	int num = 0;
	
	type = EDP_UnPacketRecv(cmd);
	switch(type)										//判断是pushdata还是命令下发
	{
		case CONNRESP:
		
			switch(EDP_UnPacketConnectRsp(cmd))
			{
				case 0:
					UsartPrintf(USART_DEBUG, "Tips:	连接成功\r\n");
					onenet_info.netWork = 1;
				break;
				
				case 1:UsartPrintf(USART_DEBUG, "WARN:	连接失败：协议错误\r\n");break;
				case 2:UsartPrintf(USART_DEBUG, "WARN:	连接失败：设备ID鉴权失败\r\n");break;
				case 3:UsartPrintf(USART_DEBUG, "WARN:	连接失败：服务器失败\r\n");break;
				case 4:UsartPrintf(USART_DEBUG, "WARN:	连接失败：用户ID鉴权失败\r\n");break;
				case 5:UsartPrintf(USART_DEBUG, "WARN:	连接失败：未授权\r\n");break;
				case 6:UsartPrintf(USART_DEBUG, "WARN:	连接失败：授权码无效\r\n");break;
				case 7:UsartPrintf(USART_DEBUG, "WARN:	连接失败：激活码未分配\r\n");break;
				case 8:UsartPrintf(USART_DEBUG, "WARN:	连接失败：该设备已被激活\r\n");break;
				case 9:UsartPrintf(USART_DEBUG, "WARN:	连接失败：重复发送连接请求包\r\n");break;
				
				default:UsartPrintf(USART_DEBUG, "ERR:	连接失败：未知错误\r\n");break;
			}
		
		break;
			
		case DISCONNECT:
		
			UsartPrintf(USART_DEBUG, "WARN:	连接断开，准备重连\r\n");
				
			net_fault_info.net_fault_level = net_fault_info.net_fault_level_r = NET_FAULT_LEVEL_1;
		
		break;
		
		case PINGRESP:
		
			UsartPrintf(USART_DEBUG, "Tips:	HeartBeat OK\r\n");
			onenet_info.heartBeat = 1;
		
		break;
		
		case PUSHDATA:									//解pushdata包
			
			result = EDP_UnPacketPushData(cmd, &cmdid_devid, &req, &req_len);
		
			if(result == 0)
				UsartPrintf(USART_DEBUG, "src_devid: %s, req: %s, req_len: %d\r\n", cmdid_devid, req, req_len);
			
		break;
		
		case CMDREQ:									//解命令包
			
			result = EDP_UnPacketCmd(cmd, &cmdid_devid, &cmdid_len, &req, &req_len);
			
			if(result == 0)								//解包成功，则进行命令回复的组包
			{
				EDP_PacketCmdResp(cmdid_devid, cmdid_len, req, req_len, &edpPacket);
				UsartPrintf(USART_DEBUG, "cmdid: %s, req: %s, req_len: %d\r\n", cmdid_devid, req, req_len);
			}
			
		break;
			
		case SAVEACK:
			
			if(cmd[3] == MSG_ID_HIGH && cmd[4] == MSG_ID_LOW)
			{
				UsartPrintf(USART_DEBUG, "Tips:	Send %s\r\n", cmd[5] ? "Err" : "Ok");
			}
			else
				UsartPrintf(USART_DEBUG, "Tips:	Message ID Err\r\n");
			
		break;
			
		default:
			result = -1;
		break;
	}
	
	if(result == -1)
		return;
	
	dataPtr = strchr(req, '}');							//搜索'}'

	if(dataPtr != NULL)									//如果找到了
	{
		dataPtr++;
		
		while(*dataPtr >= '0' && *dataPtr <= '9')		//判断是否是下发的命令控制数据
		{
			numBuf[num++] = *dataPtr++;
		}
		
		num = atoi((const char *)numBuf);				//转为数值形式
		
		if(strstr((char *)req, "redled"))				//搜索"redled"
		{
			if(num == 1)								//控制数据如果为1，代表开
			{
				Led5_Set(LED_ON);
			}
			else if(num == 0)							//控制数据如果为0，代表关
			{
				Led5_Set(LED_OFF);
			}
		}
														//下同
		else if(strstr((char *)req, "greenled"))
		{
			if(num == 1)
			{
				Led4_Set(LED_ON);
			}
			else if(num == 0)
			{
				Led4_Set(LED_OFF);
			}
		}
		else if(strstr((char *)req, "yellowled"))
		{
			if(num == 1)
			{
				Led3_Set(LED_ON);
			}
			else if(num == 0)
			{
				Led3_Set(LED_OFF);
			}
		}
		else if(strstr((char *)req, "blueled"))
		{
			if(num == 1)
			{
				Led2_Set(LED_ON);
			}
			else if(num == 0)
			{
				Led2_Set(LED_OFF);
			}
		}
	}
	
	if(type == PUSHDATA && result == 0)					//如果是pushdata 且 解包成功
	{
		EDP_FreeBuffer(cmdid_devid);					//释放内存
		EDP_FreeBuffer(req);
	}
	else if(type == CMDREQ && result == 0)				//如果是命令包 且 解包成功
	{
		EDP_FreeBuffer(cmdid_devid);					//释放内存
		EDP_FreeBuffer(req);
														//回复命令
		//NET_DEVICE_SendData(edpPacket._data, edpPacket._len);//上传平台
		NET_DEVICE_AddDataSendList(edpPacket._data, edpPacket._len);//加入链表
		EDP_DeleteBuffer(&edpPacket);					//删包
		onenet_info.sendData = SEND_TYPE_DATA;			//标记数据反馈
	}

}
