#include "includes.h"

#include "usart.h"
#include "delay.h"
#include "iwdg.h"
#include "global.h"

#include <stdarg.h>
#include <string.h>






//Debug USART1		TX--PA9	RX--PA10
//WIFI	USART2		TX--PA2	RX--PA3

USART_INFO usart2Info;

void Usart1_Init(unsigned int baud)
{

	GPIO_InitTypeDef gpioInitStruct;
	USART_InitTypeDef usartInitStruct;
	NVIC_InitTypeDef nvicInitStruct;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);
	
	//PA9	TXD
	gpioInitStruct.GPIO_Mode = GPIO_Mode_AF_PP;
	gpioInitStruct.GPIO_Pin = GPIO_Pin_9;
	gpioInitStruct.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &gpioInitStruct);
	
	//PA10	RXD
	gpioInitStruct.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	gpioInitStruct.GPIO_Pin = GPIO_Pin_10;
	gpioInitStruct.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &gpioInitStruct);
	
	usartInitStruct.USART_BaudRate = baud;
	usartInitStruct.USART_HardwareFlowControl = USART_HardwareFlowControl_None; //无硬件流控
	usartInitStruct.USART_Mode = USART_Mode_Rx | USART_Mode_Tx; //接收和发送
	usartInitStruct.USART_Parity = USART_Parity_No; //无校验
	usartInitStruct.USART_StopBits = USART_StopBits_1; //1位停止位
	usartInitStruct.USART_WordLength = USART_WordLength_8b; //8位数据位
	USART_Init(USART1, &usartInitStruct);
	
	USART_Cmd(USART1, ENABLE); //使能串口
	
	USART_ITConfig(USART1, USART_IT_RXNE, ENABLE); //使能接收中断
	
	nvicInitStruct.NVIC_IRQChannel = USART1_IRQn;
	nvicInitStruct.NVIC_IRQChannelCmd = ENABLE;
	nvicInitStruct.NVIC_IRQChannelPreemptionPriority = 0;
	nvicInitStruct.NVIC_IRQChannelSubPriority = 2;
	NVIC_Init(&nvicInitStruct);

}

void Usart2_Init(unsigned int baud)
{

	GPIO_InitTypeDef gpioInitStruct;
	USART_InitTypeDef usartInitStruct;
	NVIC_InitTypeDef nvicInitStruct;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);
	
	//PA2	TXD
	gpioInitStruct.GPIO_Mode = GPIO_Mode_AF_PP;
	gpioInitStruct.GPIO_Pin = GPIO_Pin_2;
	gpioInitStruct.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &gpioInitStruct);
	
	//PA3	RXD
	gpioInitStruct.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	gpioInitStruct.GPIO_Pin = GPIO_Pin_3;
	gpioInitStruct.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &gpioInitStruct);
	
	usartInitStruct.USART_BaudRate = baud;
	usartInitStruct.USART_HardwareFlowControl = USART_HardwareFlowControl_None; //无硬件流控
	usartInitStruct.USART_Mode = USART_Mode_Rx | USART_Mode_Tx; //接收和发送
	usartInitStruct.USART_Parity = USART_Parity_No; //无校验
	usartInitStruct.USART_StopBits = USART_StopBits_1; //1位停止位
	usartInitStruct.USART_WordLength = USART_WordLength_8b; //8位数据位
	USART_Init(USART2, &usartInitStruct);
	
	USART_Cmd(USART2, ENABLE); //使能串口
	
	USART_ITConfig(USART2, USART_IT_RXNE, ENABLE); //使能接收中断
	
	nvicInitStruct.NVIC_IRQChannel = USART2_IRQn;
	nvicInitStruct.NVIC_IRQChannelCmd = ENABLE;
	nvicInitStruct.NVIC_IRQChannelPreemptionPriority = 0;
	nvicInitStruct.NVIC_IRQChannelSubPriority = 2;
	NVIC_Init(&nvicInitStruct);
	
	Usart2_RcvClr();

}

void Usart_SendString(USART_TypeDef *USARTx, unsigned char *str, unsigned short len)
{

	unsigned short count = 0;
	
	for(; count < len; count++)
	{
		USART_SendData(USARTx, *str++);
		while(USART_GetFlagStatus(USARTx, USART_FLAG_TC) == RESET);
	}

}

void UsartPrintf(USART_TypeDef *USARTx, char *fmt,...)
{

	unsigned char UsartPrintfBuf[500];
	va_list ap;
	unsigned char *pStr = UsartPrintfBuf;
	
	va_start(ap, fmt);
	vsprintf(UsartPrintfBuf, fmt, ap);
	va_end(ap);
	
	while(*pStr != 0)
	{
		USART_SendData(USARTx, *pStr++);
		while(USART_GetFlagStatus(USARTx, USART_FLAG_TC) == RESET);
	}

}

void UsartReciveFlag(USART_INFO *usartInfo)
{

	while(1)
	{
		if(usartInfo->usartLen == 0) //如果接收计数为0 则说明没有处于接收数据中，所以直接跳出，结束函数
			break;
		
		Event_Cmd_Off();
		if(usartInfo->usartLen == usartInfo->usartLenPre) //如果上一次的值和这次相同，则说明接收完毕
		{
			usartInfo->usartReceiveFlag = REV_OK; //
			
			usartInfo->usartLen = 0; //清0接收计数
			
			Event_Cmd_On();
			
			break; //跳出
		}
		
		usartInfo->usartLenPre = usartInfo->usartLen; //置为相同
		
		DelayXms(1); //这个延时是为了在上一步完成之后，如果还没有接收完成，usartLen和usartLenPre还是会不相等的
	}

}

void Usart2_RcvClr(void)
{

	usart2Info.startRcv = 0;
	usart2Info.stopRcv = 0;
	
	usart2Info.usartLen = 0;
	usart2Info.usartCmdLen = 0;
	usart2Info.usartExtLen = 0;
	
	memset(usart2Info.usartBuf, 0, sizeof(usart2Info.usartBuf));
	memset(usart2Info.usartCmdBuf, 0, sizeof(usart2Info.usartCmdBuf));
	memset(usart2Info.usartExtBuf, 0, sizeof(usart2Info.usartExtBuf));

}

void USART1_IRQHandler(void)
{
	
	OSIntEnter();

	if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET) //接收中断(接收到的数据必须是0x0d 0x0a结尾)
	{
		
		USART_ClearFlag(USART1, USART_FLAG_RXNE);
	}
	
	OSIntExit();

}

void USART2_IRQHandler(void)
{
	
	unsigned char data = 0;
	
	OSIntEnter();

	if(USART_GetITStatus(USART2, USART_IT_RXNE) != RESET) //接收中断(接收到的数据必须是0x0d 0x0a结尾)
	{
		data = USART2->DR;
		
		usart2Info.usartBuf[usart2Info.usartLen++] = data;
		
		if(usart2Info.stopRcv == 1)
		{
			usart2Info.usartExtBuf[usart2Info.usartExtLen++] = data;
		}
		
		if(data == '{')
		{
			usart2Info.startRcv = 1;
			usart2Info.stopRcv = 0;
		}
		if(usart2Info.startRcv == 1)
		{
			if(data == '}')
			{
				usart2Info.startRcv = 0;
				usart2Info.stopRcv = 1;
			}
			else
				usart2Info.usartCmdBuf[usart2Info.usartCmdLen++] = data;
		}
		
		USART_ClearFlag(USART2, USART_FLAG_RXNE);
	}
	
	OSIntExit();

}
