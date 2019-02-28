#include "main.h"

/**
 * @brief  EDP数据包发送
 * @param  buffer: 要发送的数据缓冲区地址
* @param  len: 要发送的数据缓长度
* @param  sockfd：兼容linux socket api: STM32下无意义
 * @retval 发送的数据长度
 **/
int32_t DoSend(int32_t sockfd, const uint8_t *buffer, uint32_t len)
{
    memset(usart2_rcv_buf, 0, sizeof(usart2_rcv_buf));
    usart2_rcv_len = 0;
    usart2_write(USART2, (uint8_t*)buffer, len);
    /* wululu test print send bytes */
    hexdump((const uint8_t *)buffer, len);
    return len;
}

/**
  * @brief  和平台建立设备连接
**/
void OneNet_DevLink(const char* devid, const char* auth_key)
{
	memset(usart2_rcv_buf,0,strlen((const char *)usart2_rcv_buf));
	usart2_rcv_len=0;			
	
	printf("%s\r\n","[ESP8266_DevLink]ENTER device link...");

	send_pkg = PacketConnect1(devid,auth_key);
	mDelay(200);
	usart2_write(USART2,send_pkg->_data,send_pkg->_write_pos);  //发送设备连接请求数据
	mDelay(500);
	DeleteBuffer(&send_pkg);
	mDelay(200);

	printf("%s\r\n","[ESP8266_DevLink]EXIT device link...");
}


/*
*  @brief  EDP协议向Onenet上传温湿度
 */
void Save_DataToOneNet(void)
{
    EdpPacket* send_pkg;
   
    char send_buf[100];
		memset(send_buf,0,100);
 
    sprintf(send_buf, ",;temperature,%0.1f;hum,%0.1f;"
					"Safe,%d;Door,%d;Light,%d;Fan,%d;"
					"Active,%d;"
					   ,	sht20Info.tempture,sht20Info.hum
						,	houseInfo.Safe_Mode,houseInfo.Door_Status,houseInfo.Light_Status,houseInfo.Fan_Status
						,	houseInfo.Active_Alarm				
								);
    printf("%s %d   %s\r\n", __func__, __LINE__, send_buf);
    send_pkg = PacketSavedataSimpleString(NULL, send_buf);

    DoSend(0, (const uint8_t *)send_pkg->_data, send_pkg->_write_pos);
    DeleteBuffer(&send_pkg);
}

/*
*  @brief  EDP协议向Onenet触发警报
 */
void AlarmToOneNet(void)
{
	EdpPacket* send_pkg;
   
    char send_buf[100];
		memset(send_buf,0,100);
 
    sprintf(send_buf, ",;Active,%d;Fire,%d",houseInfo.Active_Alarm,houseInfo.Fire_Alarm);				
								
    printf("%s %d   %s\r\n", __func__, __LINE__, send_buf);
    send_pkg = PacketSavedataSimpleString(NULL, send_buf);

    DoSend(0, (const uint8_t *)send_pkg->_data, send_pkg->_write_pos);
    DeleteBuffer(&send_pkg);	
}

/*
*  @brief  EDP协议向Onenet上传温湿度、光强、目标温湿度、目标光强、目标浇水次数
 
void Analyze_CMD(void)
{
		if((NULL != strstr((const char *)usart2_cmd_buf, "AUT1")))  //约定平台控制命令"AUT1"为打开红色LED灯
		{
				LED_RED_ON;	
				boxinfo.Auto_NF=1;
		}
		if((NULL != strstr((const char *)usart2_cmd_buf, "FAN1")))  //约定平台控制命令"FAN1"为打开绿色LED灯
		{
				LED_GREEN_ON;	
				boxinfo.fan_NF=1;
		}		
		if((NULL != strstr((const char *)usart2_cmd_buf, "AUT0")))  //约定平台控制命令"AUT0"为关闭红色LED灯
		{
				LED_RED_OFF;	
				boxinfo.Auto_NF=0;
		}
		if((NULL != strstr((const char *)usart2_cmd_buf, "FAN0")))  //约定平台控制命令"FAN0"为关闭绿色LED灯
		{
				LED_GREEN_OFF;
				boxinfo.fan_NF=0;	
		}
		if((NULL != strstr((const char *)usart2_cmd_buf, "WAT")))		
		{
				boxinfo.water_NF=1;
		}
		
		if((NULL != strstr((const char *)usart2_cmd_buf, "ATH")))  //约定平台控制命令"ATHx"为调整目标温度值
		{
				sht20Info.Aim_tempture = HextoNum(usart2_cmd_buf,4);
		}
		if((NULL != strstr((const char *)usart2_cmd_buf, "ARH")))  //约定平台控制命令"ARHx"为调整目标湿度值
		{
				sht20Info.Aim_hum = HextoNum(usart2_cmd_buf,4);
		}
		if((NULL != strstr((const char *)usart2_cmd_buf, "LIG")))  //约定平台控制命令"LIGx"为调整目标光强值
		{
				gy30Info.Aim_liVal = HextoNum(usart2_cmd_buf,4);
		}
		
		
}*/

/*
 *  @brief  EDP协议向Onenet发送心跳包
 */
void Ping_Server(void)
{
	int8_t timeout = 5;
	unsigned char heartBeat[2] = {0xC0,0x00};
	memset(usart2_rcv_buf,0,sizeof(usart2_rcv_buf));
	usart2_rcv_len=0;	
	
	usart2_write(USART2, heartBeat, sizeof(heartBeat));
	mDelay(1000);
	
	while(timeout--)
	{
		hexdump(usart2_rcv_buf,3);
		if(usart2_rcv_buf[0] == 0xd0 && usart2_rcv_buf[1] == 0)
			break;
		
		memset(usart2_rcv_buf,0,sizeof(usart2_rcv_buf));
		usart2_rcv_len=0;
		usart2_write(USART2, heartBeat, sizeof(heartBeat));
		OneNet_DevLink(DEVICEID,APIKEY);    //和平台建立设备连接
		mDelay(2000);
	}
	if(timeout < 2)
	{
		ESP8266_Reset();
		ESP8266_Init();
	}
}
