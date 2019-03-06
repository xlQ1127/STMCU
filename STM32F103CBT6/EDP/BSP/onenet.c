/**

	*	说明： 		与onenet平台的数据交互接口层
	*
	*	修改记录：	V1.0：协议封装、返回判断都在同一个文件，并且不同协议接口不同。
	*				V1.1：提供统一接口供应用层使用，根据不同协议文件来封装协议相关的内容。
	************************************************************
**/


#include "esp8266.h"

#include "gpio.h"

#include "onenet.h"
#include "edpkit.h"

//C库
#include <string.h>
#include <stdio.h>

#include "bsp_led.h"


#define DEVID	"518514833"
#define APIKEY	"865ZyHxWrl0YL2cxFQ6rI5=AV90="

extern WIFI_Frame_Typedef wifi_frame;

//==========================================================
//	函数名称：	OneNet_DevLink
//	函数功能：	与onenet创建连接
//	返回参数：	1-成功	0-失败
//	说明：		与onenet平台建立连接
//==========================================================
uint8_t OneNet_DevLink(void)
{
	
	EDP_PACKET_STRUCTURE edpPacket = {NULL, 0, 0, 0};				//协议包
	unsigned char *dataPtr;
	uint8_t OneNet_Flag=0;
	
		if( 	EDP_PacketConnect1(DEVID, APIKEY, 256, &edpPacket) ==0  )
		{
					while( WIFI_SendData(edpPacket._data, edpPacket._len) )
					{
						memset(wifi_frame.RX_Buffer,0,WIFI_BUF_LEN);;
					}	
					memset(wifi_frame.RX_Buffer,0,WIFI_BUF_LEN);
					dataPtr = WIFI_GetIPD(500);
					if(dataPtr != NULL)
					{
							if(EDP_UnPacketRecv(dataPtr) == CONNRESP)
							{
									EDP_DeleteBuffer(&edpPacket);								//删包
									OneNet_Flag = EDP_UnPacketConnectRsp(dataPtr) ;
							}
					}
		}
		
		return OneNet_Flag;
}

unsigned char OneNet_FillBuf(char *buf)
{
	char text[16];
	uint8_t LED2_State=0,LED3_State=0,LED4_State=0;
	
	LED2_State  = LL_GPIO_IsOutputPinSet(LED2_GPIO_Port,LED2_Pin);
	LED2_State  = !LED2_State;
	
	LED3_State  = LL_GPIO_IsOutputPinSet(LED3_GPIO_Port,LED3_Pin);
	LED3_State  = !LED3_State;
	
	LED4_State  = LL_GPIO_IsOutputPinSet(LED4_GPIO_Port,LED4_Pin);
	LED4_State  = !LED4_State;	
	
	memset(text, 0, sizeof(text));
	
	strcpy(buf, "{");
	
	memset(text, 0, sizeof(text));
	sprintf(text, "\"LED1\":%d,", LED2_State );
	strcat(buf, text);
	
	memset(text, 0, sizeof(text));
	sprintf(text, "\"LED2\":%d,", LED3_State );
	strcat(buf, text);
	
	memset(text, 0, sizeof(text));
	sprintf(text, "\"LED3\":%d",  LED4_State );
	strcat(buf, text);
	
	
	strcat(buf, "}");
	
	return strlen(buf);
}

//==========================================================
//	函数名称：	OneNet_SendData
//	函数功能：	上传数据到平台
//	入口参数：	type：发送数据的格式		
//==========================================================
void OneNet_SendData(void)
{
	
	EDP_PACKET_STRUCTURE edpPacket = {NULL, 0, 0, 0};												//协议包
	char buf[128];
	short body_len = 0, i = 0;
	
	memset(buf, 0, sizeof(buf));
	
	body_len = OneNet_FillBuf(buf);																	//获取当前需要发送的数据流的总长度
	
	if(body_len)
	{
			if(EDP_PacketSaveData(DEVID, body_len, NULL, kTypeSimpleJsonWithoutTime, &edpPacket) == 0)	//封包
			{
				for(; i < body_len; i++)
					{
						edpPacket._data[edpPacket._len++] = buf[i];
					}
					WIFI_SendData(edpPacket._data, edpPacket._len);										//上传数据到平台
					EDP_DeleteBuffer(&edpPacket);															//删包
			}
	}
}



//==========================================================
//	函数名称：	OneNet_RevPro
//	函数功能：	平台返回数据检测
//	入口参数：	dataPtr：平台返回的数据
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
		case CMDREQ:									//解命令包
			result = EDP_UnPacketCmd(cmd, &cmdid_devid, &cmdid_len, &req, &req_len);
			if(result == 0)								//解包成功，则进行命令回复的组包
			{
				EDP_PacketCmdResp(cmdid_devid, cmdid_len, req, req_len, &edpPacket);
			}
		break;
			
		case SAVEACK:
		break;
			
		default:
			result = -1;
		break;
	}
	
//	memset(wifi_frame.RX_Buffer,0,WIFI_BUF_LEN);
	
	if(result == -1)
		return;
	
	dataPtr = strchr(req, '{');							//搜索'{'
	if(dataPtr != NULL)									//如果找到了
	{
		dataPtr = strstr((char *)req, "LED1");	
		dataPtr+=4;
  while(*dataPtr>'0'&&*dataPtr<'9') 
		{
		 numBuf[num++] = *dataPtr++;
		}
		if( dataPtr )				//搜索"LED"
		{
			num = atoi((const char *)numBuf);				//转为数值形式
			if(num == 1)								//控制数据如果为1，代表开
			{
				LED2_ON;
			}
			else if(num == 0)							//控制数据如果为0，代表关
			{
				LED2_OFF;
			}
		}
		else if(strstr((char *)req, "LED2"))
		{
			if(num == 1)
			{
				LED3_ON;
			}
			else if(num == 0)
			{
				LED3_OFF;
			}
		}
		else if(strstr((char *)req, "LED3"))
		{
			if(num == 1)
			{
				LED4_ON;
			}
			else if(num == 0)
			{
				LED4_OFF;
			}
		}
	
	}
	
	if(type == CMDREQ && result == 0)						//如果是命令包 且 解包成功
	{
		EDP_FreeBuffer(cmdid_devid);						//释放内存
		EDP_FreeBuffer(req);
															//回复命令
		WIFI_SendData(edpPacket._data, edpPacket._len);	//上传平台
		EDP_DeleteBuffer(&edpPacket);						//删包
	}
	 memset(wifi_frame.RX_Buffer,0,WIFI_BUF_LEN);
	 wifi_frame.Pointer=0;
}

