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
#include "modbuskit.h"

//硬件驱动
#include "usart.h"
#include "delay.h"

//C库
#include <string.h>
#include <stdio.h>
#include <stdlib.h>


#define MODBUS_SLAVE_ADDR		1


ONETNET_INFO onenet_info = {"1234567", "1234567", "113237",
							"183.230.40.42", "2987",
							6,
							NULL,
							0, 0, 0, 0, 0, NULL, NULL,
							0, 0, 0, 1, 0};



//==========================================================
//	函数名称：	OneNet_DevLink
//
//	函数功能：	与onenet创建连接
//
//	入口参数：	serial：序列号
//				pswd：密码
//				devid：创建设备的devid或产品ID
//
//	返回参数：	无
//
//	说明：		与onenet平台建立连接，成功或会标记oneNetInfo.netWork网络状态标志
//==========================================================
void OneNet_DevLink(const char *serial, const char *pswd, const char *proid)
{
	
	MODBUS_PACKET_STRUCTURE modbusPacket = {NULL, 0, 0, 0};				//协议包
	
	UsartPrintf(USART_DEBUG, "OneNet_DevLink\r\nPROID: %s,	SERIAL: %s,	PSWD: %s\r\n"
								, proid, serial, pswd);
	
	if(MODBUS_Connect(serial, pswd, proid, &modbusPacket) == 0)
	{
		NET_DEVICE_SendData(modbusPacket._data, modbusPacket._len);		//上传平台
		//NET_DEVICE_AddDataSendList(edpPacket._data, edpPacket._len);	//加入链表
		
		MODBUS_DeleteBuffer(&modbusPacket);								//删包
		
		onenet_info.s_addr = MODBUS_SLAVE_ADDR;
	}
	
}

//==========================================================
//	函数名称：	OneNet_SendData
//
//	函数功能：	上传数据到平台
//
//	入口参数：	无
//
//	返回参数：	SEND_TYPE_OK-发送成功	SEND_TYPE_DATA-需要重送
//
//	说明：		
//==========================================================
unsigned char OneNet_SendData(unsigned short *value_table, unsigned short value_table_cnt)
{
	
	MODBUS_PACKET_STRUCTURE modbusPacket = {NULL, 0, 0, 0};					//协议包
	
	if(MODBUS_PacketCmd(onenet_info.s_addr, onenet_info.m_cmd, value_table, value_table_cnt, &modbusPacket) == 0)
	{
		UsartPrintf(USART_DEBUG, "Tips:	ModBus Send %d Bytes\r\n", modbusPacket._len);
		
		NET_DEVICE_AddDataSendList(modbusPacket._data, modbusPacket._len);	//加入链表
		
		MODBUS_DeleteBuffer(&modbusPacket);									//删包
		
		return SEND_TYPE_OK;
	}
	else
		return SEND_TYPE_DATA;
	
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
	
	MODBUS_PACKET_STRUCTURE modbusPacket = {NULL, 0, 0, 0};				//协议包
	
	if(!onenet_info.netWork)												//如果网络为连接 或 不为数据收发模式
		return SEND_TYPE_HEART;
	
	if(MODBUS_PacketPing(&modbusPacket))
		return SEND_TYPE_HEART;
	
	onenet_info.heartBeat = 1;
	
	//NET_DEVICE_SendData(edpPacket._data, edpPacket._len);				//向平台上传心跳请求
	NET_DEVICE_AddDataSendList(modbusPacket._data, modbusPacket._len);	//加入链表
	
	MODBUS_DeleteBuffer(&modbusPacket);									//删包
	
	UsartPrintf(USART_DEBUG, "Tips:	HeartBeat Ok\r\n");
	
	return SEND_TYPE_OK;
	
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
	
	unsigned char *dataPtr = NULL, *ipdPtr = NULL;					//数据指针

	dataPtr = NET_DEVICE_Read();									//等待数据

	if(dataPtr != NULL)												//数据有效
	{
		ipdPtr = NET_DEVICE_GetIPD(dataPtr);						//检查是否是平台数据
		if(ipdPtr != NULL)
		{
			net_device_info.send_ok = 1;
			
			if(net_device_info.netWork)
				onenet_info.cmd_ptr = ipdPtr;						//标记平台下发命令
			else
				net_device_info.cmd_ipd = (char *)ipdPtr;
		}
		else
		{
			if(strstr((char *)dataPtr, "SEND OK") != NULL)
			{
				net_device_info.send_ok = 1;
				
				UsartPrintf(USART_DEBUG, "Tips:	Send Ok\r\n");
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
//	说明：		平台主动下发查询命令
//==========================================================
void OneNet_RevPro(unsigned char *cmd, unsigned short len)
{
	
	unsigned char s_addr = 0;
	
	//平台手动命令下发格式：{"cmd":"123456"}
	//注：命令体只能为数字字符，且个数必须是偶数，否则最后一个数据将被忽略而不被下发
	//命令无需专门回复，就像正常上传数据那样即可
	
	if(MODBUS_UnPacketCmd(&s_addr, &onenet_info.m_cmd, &onenet_info.r_addr, &onenet_info.r_len, cmd, len) == 0)
	{
		UsartPrintf(USART_DEBUG, "从机地址: %X, 命令: %X, 寄存器地址: %X, 长度: %x\r\n",
									s_addr, onenet_info.m_cmd, onenet_info.r_addr, onenet_info.r_len);
		
		if(s_addr == onenet_info.s_addr)
		{
			if(++onenet_info.rev_cmd_cnt >= 250)
				onenet_info.rev_cmd_cnt = 0;
		}
	}
	else
		UsartPrintf(USART_DEBUG, "WARN:	MODBUS_UnPacketCmd Err\r\n");

}


/******************************************************************************************
										消息队列
******************************************************************************************/

//==========================================================
//	函数名称：	OneNet_CheckListHead
//
//	函数功能：	检查消息链表头是否为空
//
//	入口参数：	无
//
//	返回参数：	0-空	1-不为空
//
//	说明：		
//==========================================================
_Bool OneNet_CheckListHead(void)
{

	if(onenet_info.head == NULL)
		return 0;
	else
		return 1;

}

//==========================================================
//	函数名称：	OneNet_GetListHeadBuf
//
//	函数功能：	获取链表里需要发送的数据指针
//
//	入口参数：	无
//
//	返回参数：	获取链表里需要发送的数据指针
//
//	说明：		
//==========================================================
unsigned short *OneNet_GetListHeadBuf(void)
{

	return onenet_info.head->buf;

}

//==========================================================
//	函数名称：	OneNet_GetListHeadLen
//
//	函数功能：	获取链表里需要发送的数据长度
//
//	入口参数：	无
//
//	返回参数：	获取链表里需要发送的数据长度
//
//	说明：		
//==========================================================
unsigned short OneNet_GetListHeadLen(void)
{

	return onenet_info.head->dataLen;

}

//==========================================================
//	函数名称：	OneNet_AddDataSendList
//
//	函数功能：	在消息链表尾新增一个消息链表
//
//	入口参数：	buf：需要发送的数据
//				dataLen：数据长度(半字个数)
//
//	返回参数：	0-成功	其他-失败
//
//	说明：		异步发送方式
//==========================================================
unsigned char OneNet_AddDataSendList(unsigned short *buf ,unsigned short dataLen)
{
	
	struct MODBUS_LIST *current = (struct MODBUS_LIST *)NET_MallocBuffer(sizeof(struct MODBUS_LIST));
																	//分配内存
	
	if(current == NULL)
		return 1;
	
	current->buf = (unsigned short *)NET_MallocBuffer(dataLen << 1);//分配内存
	if(current->buf == NULL)
	{
		NET_FreeBuffer(current);									//失败则释放
		return 2;
	}
	
	if(onenet_info.head == NULL)										//如果head为NULL
		onenet_info.head = current;									//head指向当前分配的内存区
	else															//如果head不为NULL
		onenet_info.end->next = current;								//则end指向当前分配的内存区
	
	memcpy(current->buf, buf, dataLen << 1);						//复制数据
	current->dataLen = dataLen;
	current->next = NULL;											//下一段为NULL
	
	onenet_info.end = current;										//end指向当前分配的内存区
	
	return 0;

}

//==========================================================
//	函数名称：	OneNet_DeleteDataSendList
//
//	函数功能：	从链表头删除一个链表
//
//	入口参数：	无
//
//	返回参数：	无
//
//	说明：		
//==========================================================
_Bool OneNet_DeleteDataSendList(void)
{
	
	struct MODBUS_LIST *next = onenet_info.head->next;		//保存链表头的下一段数据地址
	
	onenet_info.head->dataLen = 0;
	onenet_info.head->next = NULL;
	NET_FreeBuffer(onenet_info.head->buf);					//释放内存
	NET_FreeBuffer(onenet_info.head);						//释放内存
	
	onenet_info.head = next;									//链表头指向下一段数据
	
	return 0;

}
