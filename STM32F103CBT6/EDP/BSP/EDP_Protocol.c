#include "EDP_Protocol.h"                

#include "string.h"   
#include "stdlib.h"  



uint8_t* ONENET_EDPConnect_Msg( char* DevID,char* APIKey,uint16_t CN_Time,uint8_t Msg_Len)
{
	uint8_t* Msg; 
  uint16_t DevID_Len;
	uint16_t API_Key_Len;
	uint16_t i,j=0;
	
	DevID_Len=strlen(DevID);;
	API_Key_Len=strlen(APIKey);
	
	Msg=(uint8_t*)malloc(Msg_Len);
	
	Msg[0]=OneNET_CONNREQ;
	
	Msg[2]=0x00;
 Msg[3]=0x03;
	
	Msg[4]=0X45;//E
	Msg[5]=0X44;//D
	Msg[6]=0X50;//P
	
	Msg[7]=0X01;//协议版本
	Msg[8]=0X40;
	
	Msg[9]=CN_Time>>8;
	Msg[10]=CN_Time; //连接保持时间 300s
	
	Msg[11]=DevID_Len>>8;
	Msg[12]=DevID_Len;
	
	for(i=13;i<=DevID_Len+13;i++)
	{
	Msg[i]=DevID[j++];
	}
	
	Msg[13+DevID_Len]=API_Key_Len>>8;
	Msg[14+DevID_Len]=API_Key_Len;
  
	j=0;
	for(i=15+DevID_Len;i<=API_Key_Len+15+DevID_Len;i++)
	{
	Msg[i]=APIKey[j++];
	}
	
	Msg[1]=15+DevID_Len+API_Key_Len;
	
  return Msg;
}
