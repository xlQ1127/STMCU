#ifndef _ESP8266_H_
#define _ESP8266_H_







typedef struct
{
	
	unsigned char staip[16];
	unsigned char stamac[18];
	char staName[20];
	char staPass[20];
	unsigned char staPort[10];
	
	char apip[16];
	unsigned char apmac[18];
	char apName[30];
	char apPass[20];
	char apPort[10];
	
	unsigned char netWork : 1; //0-局网模式(AP)		1-互联网模式(STA)
	unsigned char ssidOK : 2;
	unsigned char pswdOK : 2;
	unsigned char err : 2; //错误类型
	unsigned char reverse : 1; //保留

} ESP8266_INFO;

extern ESP8266_INFO esp8266Info;

#define ESP_OK		0
#define ESP_FAIL	1





unsigned char ESP8266_StaInit(void);

void ESP8266_ApInit(void);

void ESP8266_Mode(void);

_Bool ESP8266_SendCmd(char *cmd, char *res);


#endif
