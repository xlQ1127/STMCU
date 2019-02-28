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
#include "mqttkit.h"

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


ONETNET_INFO onenet_info = {"5616839", "6Uvvwf=ab7O3ErvEKxyFILZxZ0s=",
							"77247", "test",
							"183.230.40.39", "6002",
							7,
							NULL, 0, 0, 0, 1};



//==========================================================
//	函数名称：	OneNet_DevLink
//
//	函数功能：	与onenet创建连接
//
//	入口参数：	devid：创建设备的devid
//				proid：产品ID
//				auth_key：创建设备的masterKey或apiKey或设备鉴权信息
//
//	返回参数：	无
//
//	说明：		与onenet平台建立连接，成功或会标记oneNetInfo.netWork网络状态标志
//==========================================================
void OneNet_DevLink(const char* devid, const char *proid, const char* auth_info)
{
	
	MQTT_PACKET_STRUCTURE mqttPacket = {NULL, 0, 0, 0};					//协议包
	
	unsigned char timeOut = 200;
	
	UsartPrintf(USART_DEBUG, "OneNet_DevLink\r\n"
							"PROID: %s,	AUIF: %s,	DEVID:%s\r\n"
                        , proid, auth_info, devid);
	
	if(MQTT_PacketConnect(proid, auth_info, devid, 256, 0, MQTT_QOS_LEVEL0, NULL, NULL, 0, &mqttPacket) == 0)
	{
		NET_DEVICE_SendData(mqttPacket._data, mqttPacket._len);			//上传平台
		//NET_DEVICE_AddDataSendList(mqttPacket._data, mqttPacket._len);//加入链表
		
		MQTT_DeleteBuffer(&mqttPacket);									//删包
		
		while(!onenet_info.netWork && --timeOut)
			DelayXms(10);
	}
	else
		UsartPrintf(USART_DEBUG, "WARN:	MQTT_PacketConnect Failed\r\n");
	
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
//	函数名称：	OneNet_DisConnect
//
//	函数功能：	与平台断开连接
//
//	入口参数：	无
//
//	返回参数：	0-成功		1-失败
//
//	说明：		
//==========================================================
_Bool OneNet_DisConnect(void)
{

	MQTT_PACKET_STRUCTURE mqttPacket = {NULL, 0, 0, 0};							//协议包

	if(!onenet_info.netWork)
		return 1;
	
	if(MQTT_PacketDisConnect(&mqttPacket) == 0)
	{
		//NET_DEVICE_SendData(mqttPacket._data, mqttPacket._len);				//向平台发送订阅请求
		NET_DEVICE_AddDataSendList(mqttPacket._data, mqttPacket._len);			//加入链表
		
		MQTT_DeleteBuffer(&mqttPacket);											//删包
	}
	
	return 0;

}

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
//	返回参数：	SEND_TYPE_OK-发送成功	SEND_TYPE_DATA-需要重送
//
//	说明：		
//==========================================================
unsigned char OneNet_SendData(FORMAT_TYPE type, char *devid, char *apikey, DATA_STREAM *streamArray, unsigned short streamArrayCnt)
{
	
	MQTT_PACKET_STRUCTURE mqttPacket = {NULL, 0, 0, 0};											//协议包
	
	_Bool status = SEND_TYPE_OK;
	short body_len = 0;
	
	if(!onenet_info.netWork)
		return SEND_TYPE_DATA;
	
	UsartPrintf(USART_DEBUG, "Tips:	OneNet_SendData-MQTT_TYPE%d\r\n", type);
	
	body_len = DSTREAM_GetDataStream_Body_Measure(type, streamArray, streamArrayCnt, 0);		//获取当前需要发送的数据流的总长度
	if(body_len > 0)
	{
		if(MQTT_PacketSaveData(devid, body_len, NULL, (uint8)type, &mqttPacket) == 0)
		{
			body_len = DSTREAM_GetDataStream_Body(type, streamArray, streamArrayCnt, mqttPacket._data, mqttPacket._size, mqttPacket._len);
			
			if(body_len > 0)
			{
				mqttPacket._len += body_len;
				UsartPrintf(USART_DEBUG, "Send %d Bytes\r\n", mqttPacket._len);
				//NET_DEVICE_SendData(mqttPacket._data, mqttPacket._len);						//上传数据到平台
				NET_DEVICE_AddDataSendList(mqttPacket._data, mqttPacket._len);					//加入链表
			}
			else
				UsartPrintf(USART_DEBUG, "WARN:	DSTREAM_GetDataStream_Body Failed\r\n");
				
			MQTT_DeleteBuffer(&mqttPacket);														//删包
		}
		else
			UsartPrintf(USART_DEBUG, "WARN:	MQTT_NewBuffer Failed\r\n");
	}
	else if(body_len < 0)
		return SEND_TYPE_OK;
	else
		status = SEND_TYPE_DATA;
	
	net_fault_info.net_fault_level_r = NET_FAULT_LEVEL_0;										//发送之后清除标记
	
	return status;
	
}

//==========================================================
//	函数名称：	OneNet_HeartBeat
//
//	函数功能：	心跳检测
//
//	入口参数：	无
//
//	返回参数：	SEND_TYPE_OK-发送成功	SEND_TYPE_DATA-需要重送
//
//	说明：		
//==========================================================
unsigned char OneNet_SendData_Heart(void)
{
	
	MQTT_PACKET_STRUCTURE mqttPacket = {NULL, 0, 0, 0};			//协议包
	
	if(!onenet_info.netWork)										//如果网络为连接
		return SEND_TYPE_HEART;
	
	if(MQTT_PacketPing(&mqttPacket))
		return SEND_TYPE_HEART;
	
	onenet_info.heartBeat = 0;
	
	//NET_DEVICE_SendData(mqttPacket._data, mqttPacket._len);		//向平台上传心跳请求
	NET_DEVICE_AddDataSendList(mqttPacket._data, mqttPacket._len);//加入链表
	
	MQTT_DeleteBuffer(&mqttPacket);								//删包
	
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
//	函数名称：	OneNet_Publish
//
//	函数功能：	发布消息
//
//	入口参数：	topic：发布的主题
//				msg：消息内容
//
//	返回参数：	SEND_TYPE_OK-成功	SEND_TYPE_PUBLISH-需要重送
//
//	说明：		
//==========================================================
unsigned char OneNet_Publish(const char *topic, const char *msg)
{

	MQTT_PACKET_STRUCTURE mqttPacket = {NULL, 0, 0, 0};							//协议包

	if(!onenet_info.netWork)
		return SEND_TYPE_PUBLISH;
	
	UsartPrintf(USART_DEBUG, "Publish Topic: %s, Msg: %s\r\n", topic, msg);
	
	if(MQTT_PacketPublish(MQTT_PUBLISH_ID, topic, msg, strlen(msg), MQTT_QOS_LEVEL2, 0, 1, &mqttPacket) == 0)
	{
		//NET_DEVICE_SendData(mqttPacket._data, mqttPacket._len);				//向平台发送订阅请求
		NET_DEVICE_AddDataSendList(mqttPacket._data, mqttPacket._len);			//加入链表
		
		MQTT_DeleteBuffer(&mqttPacket);											//删包
	}
	
	return SEND_TYPE_OK;

}

//==========================================================
//	函数名称：	OneNet_Subscribe
//
//	函数功能：	订阅
//
//	入口参数：	topics：订阅的topic
//				topic_cnt：topic个数
//
//	返回参数：	SEND_TYPE_OK-成功	SEND_TYPE_SUBSCRIBE-需要重发
//
//	说明：		
//==========================================================
unsigned char OneNet_Subscribe(const char *topics[], unsigned char topic_cnt)
{
	
	unsigned char i = 0;
	
	MQTT_PACKET_STRUCTURE mqttPacket = {NULL, 0, 0, 0};							//协议包

	if(!onenet_info.netWork)
		return SEND_TYPE_SUBSCRIBE;
	
	for(; i < topic_cnt; i++)
		UsartPrintf(USART_DEBUG, "Subscribe Topic: %s\r\n", topics[i]);
	
	if(MQTT_PacketSubscribe(MQTT_SUBSCRIBE_ID, MQTT_QOS_LEVEL2, topics, topic_cnt, &mqttPacket) == 0)
	{
		//NET_DEVICE_SendData(mqttPacket._data, mqttPacket._len);				//向平台发送订阅请求
		NET_DEVICE_AddDataSendList(mqttPacket._data, mqttPacket._len);			//加入链表
		
		MQTT_DeleteBuffer(&mqttPacket);											//删包
	}
	
	return SEND_TYPE_OK;

}

//==========================================================
//	函数名称：	OneNet_UnSubscribe
//
//	函数功能：	取消订阅
//
//	入口参数：	topics：订阅的topic
//				topic_cnt：topic个数
//
//	返回参数：	SEND_TYPE_OK-发送成功	SEND_TYPE_UNSUBSCRIBE-需要重发
//
//	说明：		
//==========================================================
unsigned char OneNet_UnSubscribe(const char *topics[], unsigned char topic_cnt)
{
	
	unsigned char i = 0;
	
	MQTT_PACKET_STRUCTURE mqttPacket = {NULL, 0, 0, 0};							//协议包

	if(!onenet_info.netWork)
		return SEND_TYPE_UNSUBSCRIBE;
	
	for(; i < topic_cnt; i++)
		UsartPrintf(USART_DEBUG, "UnSubscribe Topic: %s\r\n", topics[i]);
	
	if(MQTT_PacketUnSubscribe(MQTT_UNSUBSCRIBE_ID, topics, topic_cnt, &mqttPacket) == 0)
	{
		//NET_DEVICE_SendData(mqttPacket._data, mqttPacket._len);				//向平台发送取消订阅请求
		NET_DEVICE_AddDataSendList(mqttPacket._data, mqttPacket._len);			//加入链表
		
		MQTT_DeleteBuffer(&mqttPacket);											//删包
	}
	
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
	
	unsigned char *dataPtr = NULL, *ipdPtr = NULL;		//数据指针

	dataPtr = NET_DEVICE_Read();						//等待数据

	if(dataPtr != NULL)									//数据有效
	{
		ipdPtr = NET_DEVICE_GetIPD(dataPtr);			//检查是否是平台数据
		if(ipdPtr != NULL)
		{
			net_device_info.send_ok = 1;
			
			if(net_device_info.netWork)
				OneNet_RevPro(ipdPtr);					//集中处理
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
	
	MQTT_PACKET_STRUCTURE mqttPacket = {NULL, 0, 0, 0};								//协议包
	
	char *req_payload = NULL;
	char *cmdid_topic = NULL;
	unsigned char type = 0;
	unsigned char qos = 0;
	static unsigned short pkt_id = 0;
	
	short result = 0;

	char *dataPtr = NULL;
	char numBuf[10];
	int num = 0;
	
	type = MQTT_UnPacketRecv(cmd);
	switch(type)
	{
		case MQTT_PKT_CONNACK:
		
			switch(MQTT_UnPacketConnectAck(cmd))
			{
				case 0:
					UsartPrintf(USART_DEBUG, "Tips:	连接成功\r\n");
					onenet_info.netWork = 1;
				break;
				
				case 1:UsartPrintf(USART_DEBUG, "WARN:	连接失败：协议错误\r\n");break;
				case 2:UsartPrintf(USART_DEBUG, "WARN:	连接失败：非法的clientid\r\n");break;
				case 3:UsartPrintf(USART_DEBUG, "WARN:	连接失败：服务器失败\r\n");break;
				case 4:UsartPrintf(USART_DEBUG, "WARN:	连接失败：用户名或密码错误\r\n");break;
				case 5:UsartPrintf(USART_DEBUG, "WARN:	连接失败：非法链接(比如token非法)\r\n");break;
				
				default:UsartPrintf(USART_DEBUG, "ERR:	连接失败：未知错误\r\n");break;
			}
		
		break;
		
		case MQTT_PKT_PINGRESP:
		
			UsartPrintf(USART_DEBUG, "Tips:	HeartBeat OK\r\n");
			onenet_info.heartBeat = 1;
		
		break;
		
		case MQTT_PKT_CMD:															//命令下发
			
			result = MQTT_UnPacketCmd(cmd, &cmdid_topic, &req_payload);				//解出topic和消息体
			if(result == 0)
			{
				UsartPrintf(USART_DEBUG, "cmdid: %s, req: %s\r\n", cmdid_topic, req_payload);
				
				if(MQTT_PacketCmdResp(cmdid_topic, req_payload, &mqttPacket) == 0)	//命令回复组包
				{
					UsartPrintf(USART_DEBUG, "Tips:	Send CmdResp\r\n");
					
					//NET_DEVICE_SendData(mqttPacket._data, mqttPacket._len);		//回复命令
					NET_DEVICE_AddDataSendList(mqttPacket._data, mqttPacket._len);	//加入链表
					MQTT_DeleteBuffer(&mqttPacket);									//删包
				}
			}
		
		break;
			
		case MQTT_PKT_PUBLISH:														//接收的Publish消息
		
			result = MQTT_UnPacketPublish(cmd, &cmdid_topic, &req_payload, &qos, &pkt_id);
			if(result == 0)
			{
				UsartPrintf(USART_DEBUG, "topic: %s\r\npayload: %s\r\n", cmdid_topic, req_payload);
				
				switch(qos)
				{
					case 1:															//收到publish的qos为1，设备需要回复Ack
					
						if(MQTT_PacketPublishAck(pkt_id, &mqttPacket) == 0)
						{
							UsartPrintf(USART_DEBUG, "Tips:	Send PublishAck\r\n");
							//NET_DEVICE_SendData(mqttPacket._data, mqttPacket._len);
							NET_DEVICE_AddDataSendList(mqttPacket._data, mqttPacket._len);//加入链表
							MQTT_DeleteBuffer(&mqttPacket);
						}
					
					break;
					
					case 2:															//收到publish的qos为2，设备先回复Rec
																					//平台回复Rel，设备再回复Comp
						if(MQTT_PacketPublishRec(pkt_id, &mqttPacket) == 0)
						{
							UsartPrintf(USART_DEBUG, "Tips:	Send PublishRec\r\n");
							//NET_DEVICE_SendData(mqttPacket._data, mqttPacket._len);
							NET_DEVICE_AddDataSendList(mqttPacket._data, mqttPacket._len);//加入链表
							MQTT_DeleteBuffer(&mqttPacket);
						}
					
					break;
					
					default:
						break;
				}
			}
		
		break;
			
		case MQTT_PKT_PUBACK:														//发送Publish消息，平台回复的Ack
		
			if(MQTT_UnPacketPublishAck(cmd) == 0)
				UsartPrintf(USART_DEBUG, "Tips:	MQTT Publish Send OK\r\n");
			
		break;
			
		case MQTT_PKT_PUBREC:														//发送Publish消息，平台回复的Rec，设备需回复Rel消息
		
			if(MQTT_UnPacketPublishRec(cmd) == 0)
			{
				UsartPrintf(USART_DEBUG, "Tips:	Rev PublishRec\r\n");
				if(MQTT_PacketPublishRel(MQTT_PUBLISH_ID, &mqttPacket) == 0)
				{
					UsartPrintf(USART_DEBUG, "Tips:	Send PublishRel\r\n");
					//NET_DEVICE_SendData(mqttPacket._data, mqttPacket._len);
					NET_DEVICE_AddDataSendList(mqttPacket._data, mqttPacket._len);	//加入链表
					MQTT_DeleteBuffer(&mqttPacket);
				}
			}
		
		break;
			
		case MQTT_PKT_PUBREL:														//收到Publish消息，设备回复Rec后，平台回复的Rel，设备需再回复Comp
			
			if(MQTT_UnPacketPublishRel(cmd, pkt_id) == 0)
			{
				UsartPrintf(USART_DEBUG, "Tips:	Rev PublishRel\r\n");
				if(MQTT_PacketPublishComp(MQTT_PUBLISH_ID, &mqttPacket) == 0)
				{
					UsartPrintf(USART_DEBUG, "Tips:	Send PublishComp\r\n");
					//NET_DEVICE_SendData(mqttPacket._data, mqttPacket._len);
					NET_DEVICE_AddDataSendList(mqttPacket._data, mqttPacket._len);	//加入链表
					MQTT_DeleteBuffer(&mqttPacket);
				}
			}
		
		break;
		
		case MQTT_PKT_PUBCOMP:														//发送Publish消息，平台返回Rec，设备回复Rel，平台再返回的Comp
		
			if(MQTT_UnPacketPublishComp(cmd) == 0)
			{
				UsartPrintf(USART_DEBUG, "Tips:	Rev PublishComp\r\n");
			}
		
		break;
			
		case MQTT_PKT_SUBACK:														//发送Subscribe消息的Ack
		
			if(MQTT_UnPacketSubscribe(cmd) == 0)
				UsartPrintf(USART_DEBUG, "Tips:	MQTT Subscribe OK\r\n");
			else
				UsartPrintf(USART_DEBUG, "Tips:	MQTT Subscribe Err\r\n");
		
		break;
			
		case MQTT_PKT_UNSUBACK:														//发送UnSubscribe消息的Ack
		
			if(MQTT_UnPacketUnSubscribe(cmd) == 0)
				UsartPrintf(USART_DEBUG, "Tips:	MQTT UnSubscribe OK\r\n");
			else
				UsartPrintf(USART_DEBUG, "Tips:	MQTT UnSubscribe Err\r\n");
		
		break;
		
		default:
			result = -1;
		break;
	}
	
	if(result == -1)
		return;
	
	dataPtr = strchr(req_payload, '}');					//搜索'}'

	if(dataPtr != NULL)									//如果找到了
	{
		dataPtr++;
		
		while(*dataPtr >= '0' && *dataPtr <= '9')		//判断是否是下发的命令控制数据
		{
			numBuf[num++] = *dataPtr++;
		}
		
		num = atoi((const char *)numBuf);				//转为数值形式
		
		if(strstr((char *)req_payload, "redled"))		//搜索"redled"
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
		else if(strstr((char *)req_payload, "greenled"))
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
		else if(strstr((char *)req_payload, "yellowled"))
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
		else if(strstr((char *)req_payload, "blueled"))
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

	if(type == MQTT_PKT_CMD || type == MQTT_PKT_PUBLISH)
	{
		MQTT_FreeBuffer(cmdid_topic);
		MQTT_FreeBuffer(req_payload);
		onenet_info.sendData = SEND_TYPE_DATA;
	}

}
