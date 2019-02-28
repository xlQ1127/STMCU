#ifndef __EDP_PROTOCOL_H__
#define __EDP_PROTOCOL_H__

#include "stm32f1xx_hal.h"

/*----------------------------消息类型---------------------------------------*/
enum {
	OneNET_CONNREQ=0x10,    //连接请求
	OneNET_CONNRESP=0x20,   //连接响应
	OneNET_PUSHDATA=0x30,   //转发(透传)数据
	OneNET_UPDATEREQ=0x50,   //升级请求
	
	OneNET_UPDATERESP=0x60,  //升级响应
  OneNET_SAVEDATA=0x80,    //存储(转发)数据
  OneNET_SAVEACK=0x90,     //存储确认
  OneNET_CMDREQ=0xA0,      //命令请求
	
  OneNET_CMDRESP=0xB0,    //命令响应
  OneNET_PINGREQ=0xC0,	   //心跳请求
	OneNET_PINGRESP=0xD0,   //心跳响应
	OneNET_ENCRYPTREQ=0xE0, //加密请求
	
  OneNET_ENCRYPTRESP=0xF0,	//加密响应
	OneNET_UPDATE=0x50       //
};




uint8_t* ONENET_EDPConnect_Msg( char* DevID,char* APIKey,uint16_t CN_Time,uint8_t Msg_Len);





#endif
