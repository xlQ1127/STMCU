#ifndef _USART_H_
#define _USART_H_

#include "stm32f10x.h"





typedef struct
{

	unsigned char startRcv : 1;
	unsigned char stopRcv : 1;
	unsigned char reverse : 6;
	
	unsigned char usartReceiveFlag;
	
	unsigned short usartLen;
	unsigned short usartLenPre;
	unsigned short usartCmdLen;
	unsigned short usartExtLen;
	
	unsigned char usartBuf[300];
	unsigned char usartCmdBuf[100];
	unsigned char usartExtBuf[30];

} USART_INFO;

#define REV_OK		1
#define REV_WAIT	0

extern USART_INFO usart2Info;




void Usart1_Init(unsigned int baud);

void Usart2_Init(unsigned int baud);

void Usart_SendString(USART_TypeDef *USARTx, unsigned char *str, unsigned short len);

void UsartPrintf(USART_TypeDef *USARTx, char *fmt,...);

void UsartReciveFlag(USART_INFO *usartInfo);

void Usart2_RcvClr(void);


#endif
