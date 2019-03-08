/* Includes ------------------------------------------------------------------*/
#include "esp8266.h"

#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include "usart.h"

#define USE_LL_DELAY  1 
#define USE_HAL_DELAY 0
#define USE_OS_DELAY  0



WIFI_Frame_Typedef wifi_frame;

uint8_t UART_TX_Buf[512];

uint8_t Dat[35]={
                 0X80,
	                0X21,//消息长度
	                0X80,
	                0X00,0X08,
	                0X34,0X35,0X30,0X31,0X33,0X39,0X35,0X33,
	                0X05,
	                0X00,0X13,//下方数据长度
	                0X2C,0X3B,
                 0X44,0X30,0X31,0X2C,//D01,
	                0,0,0X2E,0,//00.0
	                0X3B,//;
	                0X44,0X30,0X32,0X2C,//D02,
                 0,0,0X2E,0,//00.0      
};

/*
0为成功发送AT命令
1为失败
*/
uint8_t WIFI_ATCMD(char* CMD,char * Token1,char * Token2,uint16_t timems)
{
	 static uint8_t Token1_Flag=0,Token2_Flag=0,Err_Flag=0;
	 static uint16_t Len=0;

	 wifi_frame.Pointer=0;
	 memset(wifi_frame.RX_Buffer,0,WIFI_BUF_LEN);
	
	 Len=strlen(CMD);
	 memcpy(UART_TX_Buf,(uint8_t *)CMD,Len);
	 
  if( HAL_UART_Transmit(&huart2,UART_TX_Buf, Len, 0xFF) !=HAL_OK )
		{
			return 1;
		}
		
		#if USE_LL_DELAY
		LL_mDelay(timems);
		#endif
		#if USE_HAL_DELAY
		HAL_Delay(timems);
		#endif  
		
		if( ( Token1 == 0 ) && ( Token2 == 0 ) )
		{
			return 0;
		}
		else
		{
							if (strstr((char *)wifi_frame.RX_Buffer, Token1) != NULL)
							{
								Token1_Flag=0;
							}
							else
							{
							Token1_Flag=1;
							}
							if(strstr((char *)wifi_frame.RX_Buffer, Token2) != NULL)
							{
							Token2_Flag=0;
							}
							else
							{
							Token2_Flag=1;
							}		
							
							if (strstr((char *)wifi_frame.RX_Buffer, "ERROR\r\n") != NULL)
							{
								Err_Flag=1;
							}
							else
							{
								Err_Flag=0;
							}
							
							if( (Token1_Flag==0 || Token2_Flag==0) && Err_Flag==0 )
							{
								return 0;
							}
							else
							{
								return 1;
							}
							
		}
			

}




/*
0为成功连接WIFI接入点
1为失败
*/
uint8_t WIFI_JAP_Type1(char* ssid, char* Password)//AT+CWJAP="ssid","Password"
{
	 char tmp[128];
	 sprintf( tmp, "AT+CWJAP=\"%s\",\"%s\"\r\n", ssid, Password);
  if( WIFI_ATCMD(tmp,"WIFI CONNECTED","OK",10000) !=0 )
		{
		return 1;
		}			
		else
		{
		return 0;
		}
}


/*
0为成功连接WIFI接入点
1为失败
*/
uint8_t WIFI_JAP_Type2(char* ssid, char* Password)//AT+CWJAP="\"ssid\"","Password"
{
	 char tmp[128];
	 sprintf( tmp, "AT+CWJAP=\"\\\"%s\\\"\",\"%s\"\r\n", ssid, Password);
  if( WIFI_ATCMD(tmp,"OK",NULL,15000) !=0 )
		{
		return 1;
		}			
		else
		{
		return 0;
		}
}




/*
0为成功设置ESP工作模式
1为失败
1:Station模式 
2:AP 模式 
3:AP兼Station模式
AT+CWMODE=Mode
*/
uint8_t WIFI_CWMODE(uint8_t Mode)
{
	 char tmp[128];
	 sprintf( tmp, "AT+CWMODE=%d\r\n",Mode);
  if( WIFI_ATCMD(tmp,"OK",NULL,2) !=0 )
		{
		return 1;
		}			
		else
		{
		return 0;
		}
}

/*
链接ONENET IP
0为成功连接ONENET IP
1为失败
*/
uint8_t WIFI_ConnectIP(void)
{
	 static char tmp[128];
	
	 sprintf( tmp, "AT+CIPSTART=\"TCP\",\"jjfaedp.hedevice.com\",876\r\n");
  if( WIFI_ATCMD(tmp,"CONNECT","ALREADY CONNECTED",1000) !=0 )
		{
	 	return 1;
		}			
		else
		{
			return 0;
		}
}





 




/*
链接ONENET IP
0为成功连接ONENET IP
1为失败
*/
uint8_t WIFI_ConnectOnenet(uint8_t* buf)
{
  static char cmd[32];
	 
	 sprintf(cmd,"AT+CIPSEND=%d\r\n",buf[1]+2);

	 if( WIFI_ATCMD(cmd,">",NULL,10) !=0 )
		{
		return 1;
		}			
		else
		{
	  HAL_UART_Transmit(&huart2,buf,buf[1]+2,0xff);
			if( WIFI_ATCMD(NULL,"SEND OK",NULL,5)!=0 )
			{
			return 1;
			}
			else
			{
			return 0;
			}
		}
}


/*
链接ONENET IP
0为成功连接ONENET IP
1为失败
*/
uint8_t WIFI_Onenet_Send(float D01,float D02)
{
	 static char tmp_Buf[4];
	
		sprintf( tmp_Buf,"%2.1f",D01);
	  Dat[22]=tmp_Buf[0];
	  Dat[23]=tmp_Buf[1];
	  Dat[25]=tmp_Buf[3];
	
		sprintf( tmp_Buf,"%2.1f",D02);
	  Dat[31]=tmp_Buf[0];
	  Dat[32]=tmp_Buf[1];
	  Dat[34]=tmp_Buf[3];
	
   
		if( WIFI_ATCMD("AT+CIPSEND=35\r\n",">",NULL,5) !=0 )
		{
		return 1;
		}			
		else
		{
	  HAL_UART_Transmit(&huart2,(uint8_t *)Dat,35,0xff);
			if( WIFI_ATCMD( NULL,"SEND OK",NULL,5)!=0 )
			{
			return 1;
			}
			else
			{
				return 0;	
			}
		}
}


uint8_t WIFI_Onenet_RecD03(void)
{
  char *p;
	 uint8_t D03;
	
	 p=(char *)&wifi_frame.RX_Buffer;
	 p = strstr(p,"D03");
	 if( p )
		{
		  p+=3;
			 D03=atoi(p);		
		}
		return D03;
}

uint8_t WIFI_Onenet_RecD04(void)
{
  char *p;
	 uint8_t D04;
	
		p=(char *)&wifi_frame.RX_Buffer;
		p = strstr(p,"D04");
	 if( p )
		{
		  p+=3;
			 D04=atoi(p);		
		}
		return D04;
}

uint8_t WIFI_Onenet_RecD05(void)
{
  char *p;
	 uint8_t D05;
	
		p=(char *)&wifi_frame.RX_Buffer;
		p = strstr(p,"D05");
		if( p )
		{
		  p+=3;
			 D05=atoi(p);		
		}
		
		return D05;
}


uint8_t WIFI_SendData(unsigned char *data, unsigned short len)
{

	static char cmd[32];
	static uint8_t WIFI_Res=0;
	
 sprintf(cmd,"AT+CIPSEND=%d\r\n",len);

	if( WIFI_ATCMD(cmd, ">",NULL,20) != 1)				//收到‘>’时可以发送数据
	{
		while ( HAL_UART_Transmit(&huart2,data,len,0xff) != HAL_OK )
  WIFI_Res = 0;
	}
	else
	{
		WIFI_Res = 1;
	}
	return WIFI_Res;
}

unsigned char * WIFI_GetIPD(unsigned short timeOut)
{

	char *ptrIPD = NULL;
	
	do
	{
			ptrIPD = strstr((char *)wifi_frame.RX_Buffer, "IPD,");				//搜索“IPD”头
			if(ptrIPD == NULL)											//如果没找到，可能是IPD头的延迟，还是需要等待一会，但不会超过设定的时间
			{
				//UsartPrintf(USART_DEBUG, "\"IPD\" not found\r\n");
			}
			else
			{
				ptrIPD = strchr(ptrIPD, ':');							//找到':'
				if(ptrIPD != NULL)
				{
					ptrIPD++;
					return (unsigned char *)(ptrIPD);
				}
				else
					return NULL;
		 }
	
		#if USE_LL_DELAY
		LL_mDelay(1);
		#endif
		#if USE_HAL_DELAY
		HAL_Delay(timems);
		#endif 													//延时等待
	} while(timeOut--);
	
	return NULL;														//超时还未找到，返回空指针

}
