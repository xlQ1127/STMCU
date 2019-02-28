#include "main.h"


EdpPacket* send_pkg;
char send_buf[MAX_SEND_BUF_LEN];

/**
* @brief  ESP8266 复位IO口初始化
**/
void ESP8266_IO_Init(void)
{

	GPIO_InitTypeDef gpioInitStruct;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);

	//复位
	gpioInitStruct.GPIO_Mode = GPIO_Mode_Out_PP;
	gpioInitStruct.GPIO_Pin = GPIO_Pin_5;
	gpioInitStruct.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &gpioInitStruct);

	ESP8266_RST_OFF;

}

/**
  * @brief  ESP8266 硬件重启
**/
void ESP8266_Reset(void)
{

	ESP8266_RST_ON;
	mDelay(100);
	ESP8266_RST_OFF;
	mDelay(5000);
}

/**
  * @brief  ESP8266 初始化
**/
void ESP8266_Init(void)
{
	int8_t status;		
	status = ESP8266_CheckStatus(20);
	if(-5 == status || -4 == status || -1 == status)
	{
			ESP8266_JoinAP();
			ESP8266_SendCmd(CIPSTART,"OK",2200);
	}
	if(-2 == status)
			ESP8266_SendCmd(CIPSTART,"OK",2200);
	
	ESP8266_Translink(20);
	OneNet_DevLink(DEVICEID,APIKEY);    //和平台建立设备连接	
}
/**
  * @brief  ESP8266 AT指令测试
**/
void ESP8266_ATtest(void)
{
		printf("%s\r\n","[ESP8266_Init]ENTER AT.");
		ESP8266_SendCmd(AT,"OK",1000); 			
		printf("%s\r\n","[ESP8266_Init]EXIT AT.");		
}

/**
  * @brief  ESP8266连接WiFi
**/
void ESP8266_JoinAP(void)
{		
		printf("%s\r\n","[ESP8266_Init]ENTER CWMODE.");
		ESP8266_SendCmd(CWMODE,"OK",1000);		
		printf("%s\r\n","[ESP8266_Init]EXIT CWMODE.");
	
		printf("%s\r\n","[ESP8266_Init]ENTER RST.");
		ESP8266_SendCmd(RST,"OK",2000);	
		printf("%s\r\n","[ESP8266_Init]EXIT RST.");
	
		printf("%s\r\n","[ESP8266_Init]ENTER AT.");
		ESP8266_SendCmd(AT,"OK",1000);	
		printf("%s\r\n","[ESP8266_Init]EXIT AT.");
	
#ifdef OPSMART
		ESP8266_SendCmd(STARTSMART,"OK",2000);	
		printf("%s\r\n","[ESP8266_Init]ENTER SmartConfig.");
		ESP8266_SendCmd(NULL,"connected",2000);
		ESP8266_SendCmd(STOPSMART,"OK",2000);
		printf("%s\r\n","[ESP8266_Init]EXIT SmartConfig.");
#else 
		printf("%s\r\n","[ESP8266_Init]ENTER CWJAP");
		ESP8266_SendCmd(CWJAP,"OK",2200);
		printf("%s\r\n","[ESP8266_Init]EXIT CWJAP");	
#endif
}

/**
  * @brief  进入透传模式
**/
void ESP8266_Translink(int timeOut)
{		
		printf("%s\r\n","[ESP8266_Init]ENTER CIPMODE.");
		ESP8266_SendCmd(CIPMODE,"OK",1000);
		printf("%s\r\n","[ESP8266_Init]EXIT CIPMODE.");
	
		printf("%s\r\n","[ESP8266_Init]ENTER CIPSEND.");
		ESP8266_SendCmd(CIPSEND,"OK",1000);
		printf("%s\r\n","[ESP8266_Init]EXIT CIPSEND.");		
}

/**
  * @brief  生成LED当前状态的上传数据，分割字符串格式
**/
void GetSendBuf(void)
{
		char text[25] = {0};
			
		LED_GetValue();

		memset(send_buf,0,MAX_SEND_BUF_LEN);
		
		strcat(send_buf, ",;");	
		strcat(send_buf, "red_statu,");
		sprintf(text,"%d",red_value);
		strcat(send_buf, text);
		strcat(send_buf, ";");
		
		strcat(send_buf, "green_statu,");
		sprintf(text,"%d",green_value);
		strcat(send_buf, text);
		strcat(send_buf, ";");

		strcat(send_buf, "yellow_statu,");
		sprintf(text,"%d",yellow_value);
		strcat(send_buf, text);
		strcat(send_buf, ";");

		strcat(send_buf, "blue_statu,");
		sprintf(text,"%d",blue_value);
		strcat(send_buf, text);
	//	strcat(send_buf, ";");	
}


/**
  * @brief  发送一条AT指令
**/
void ESP8266_SendCmd(char* cmd, char* result, int timeOut)
{
    while(1)
    {
        memset(usart2_rcv_buf,0,sizeof(usart2_rcv_buf));
				usart2_rcv_len=0;
        usart2_write(USART2, (unsigned char *)cmd, strlen((const char *)cmd));
        mDelay(timeOut);
       
        if((NULL != strstr((const char *)usart2_rcv_buf, result)))	//判断是否有预期的结果
        {
            break;
        }
    }
}


/**
  * @brief  检测ESP8266连接状态
**/
int ESP8266_CheckStatus(int timeOut)
{
	int32 res=0;
	int32 count=0;

	memset(usart2_rcv_buf,0,sizeof(usart2_rcv_buf));
	usart2_rcv_len=0;
	
	printf("%s\r\n","[ESP8266_CheckStatus]ENTER check status...");
	usart2_write(USART2,CIPSTATUS,strlen(CIPSTATUS));
	for(count=0;count<timeOut;count++)
	{
			mDelay(1000);
			if((NULL != strstr((const char *)usart2_rcv_buf,"STATUS:4")))  //失去连接
			{
					res=-4;
					break;
			}
			else if((NULL != strstr((const char *)usart2_rcv_buf,"STATUS:3")))  //建立连接
			{
					res=0;	
					break;
			}
			else if((NULL != strstr((const char *)usart2_rcv_buf,"STATUS:2")))  //获得IP
			{
					res=-2;
					break;				
			}
			else if((NULL != strstr((const char *)usart2_rcv_buf,"STATUS:5")))  //物理掉线
			{
					res=-5;
					break;
			}
			else if((NULL != strstr((const char *)usart2_rcv_buf,"ERROR")))   
			{
					res=-1;
					break;
			}
			else
			{
					usart2_write(USART2,"+++",3);
			}
	}	
	printf("[ESP8266_CheckStatus]Status is %d\r\n",res);
	return res;	
}

/**
  * @brief  向平台上传LED当前状态数据
**/
void ESP8266_SendDat(void)
{		
//		int32 count=0;

		memset(usart2_rcv_buf,0,sizeof(usart2_rcv_buf));
		usart2_rcv_len=0;			
		printf("%s\r\n","[ESP8266_SendDat]ENTER Senddata...");
/*		usart2_write(USART2,CIPSEND,strlen(CIPSEND));  //向ESP8266发送数据透传指令
		for(count=0;count<40;count++)
		{
				mDelay(100);
				if((NULL != strstr((const char *)usart2_rcv_buf,">")))
				{
						break;
				}
		}	*/
	
		GetSendBuf();	
		printf("%s\r\n",send_buf);
		send_pkg = PacketSavedataSimpleString(NULL,send_buf);   
		usart2_write(USART2,send_pkg->_data,send_pkg->_write_pos);	//向平台上传数据点
		DeleteBuffer(&send_pkg);
//		mDelay(500);

//		usart2_write(USART2,"+++",3);  //向ESP8266发送+++结束透传，使ESP8266返回指令模式
//		mDelay(200);
		printf("%s\r\n","[ESP8266_SendDat]EXIT Senddata...");
}









