#ifndef _ONENET_H_
#define _ONENET_H_


#include "usart.h"






#define   DEVICEID   "改为自己的devid"
#define   APIKEY     "改为自己的apikey"




void OneNet_DevLink(const char* devid, const char* auth_key);

void OneNet_SendData(void);

void HeartBeat(USART_INFO *usartInfo);

void EDPKitCmd(USART_INFO *usartInfo);

void OneNetApp(USART_INFO *usartInfo);


#endif
