#ifndef   ESP8266_H_H
#define   ESP8266_H_H

#define   OPSMART
#define   DEVICEID   "12345678"
#define   APIKEY     "yourkeynumber"

#define   AT          "AT\r\n"	
#define   CWMODE      "AT+CWMODE_DEF=1\r\n"
#define   RST         "AT+RST\r\n"
#define   CIFSR       "AT+CIFSR\r\n"
#define   STARTSMART  "AT+CWSTARTSMART\r\n"
#define   STOPSMART   "AT+CWSTOPSMART\r\n"

#define   CIPSTART    "AT+CIPSTART=\"TCP\",\"183.230.40.39\",876\r\n"
#define   CIPMODE     "AT+CIPMODE=1\r\n"
#define   CIPSEND     "AT+CIPSEND\r\n"
#define   CIPSTATUS   "AT+CIPSTATUS\r\n"

#define   MAX_SEND_BUF_LEN  1024

#define ESP8266_RST_ON		GPIO_ResetBits(GPIOB, GPIO_Pin_5)
#define ESP8266_RST_OFF		GPIO_SetBits(GPIOB, GPIO_Pin_5)

extern    EdpPacket* send_pkg;
extern    char send_buf[MAX_SEND_BUF_LEN];

void ESP8266_IO_Init(void);
void ESP8266_Reset(void);
void ESP8266_Init(void);
void ESP8266_ATtest(void);
void ESP8266_JoinAP(void);
void ESP8266_Init(void);
void ESP8266_Translink(int timeOut);
void GetSendBuf(void);
void ESP8266_SendCmd(char* cmd, char* result, int timeOut);

int ESP8266_CheckStatus(int timeOut);
void ESP8266_SendDat(void);

#endif


