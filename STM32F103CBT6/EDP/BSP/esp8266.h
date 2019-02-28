#ifndef __ESP8266_H
#define __ESP8266_H

#include "stm32f1xx_hal.h"




extern uint8_t WIFI_RX_Buffer[128];

uint8_t WIFI_ATCMD(char* CMD,char * Token1,char * Token2,uint16_t timems);
uint8_t WIFI_CWMODE(uint8_t Mode);
uint8_t WIFI_JAP_Type1(char* ssid, char* Password);//AT+CWJAP="ssid","Password";
uint8_t WIFI_JAP_Type2(char* ssid, char* Password);//AT+CWJAP="\"ssid\"","Password"
uint8_t WIFI_ConnectIP(void);
uint8_t WIFI_ConnectOnenet(uint8_t* Buf);
uint8_t WIFI_Onenet_Send(float D01,float D02);
uint8_t WIFI_Onenet_RecD03(void);
uint8_t WIFI_Onenet_RecD04(void);
uint8_t WIFI_Onenet_RecD05(void);

#endif /* __ESP8266_H */

