#include "stm32f10x.h"

#include "lcd1602.h"
#include "delay.h"

#include <stdarg.h>






//RW		PA11
//RS		PC6
//EN		PC3
//DATA0~4	PB5~9
//DATA5~7	PC0~2

#define RS_H	GPIO_SetBits(GPIOC, GPIO_Pin_6)
#define RS_L	GPIO_ResetBits(GPIOC, GPIO_Pin_6)

#define RW_H	GPIO_SetBits(GPIOA, GPIO_Pin_11)
#define RW_L	GPIO_ResetBits(GPIOA, GPIO_Pin_11)

#define EN_H	GPIO_SetBits(GPIOC, GPIO_Pin_3)
#define EN_L	GPIO_ResetBits(GPIOC, GPIO_Pin_3)

#define BUSY_IO_OUT		{GPIOC->CRL &= ~(3 << 8);GPIOC->CRL &= ~(3 << 10);GPIOC->CRL |= 2 << 10;}
#define BUSY_IO_IN		{GPIOC->CRL |= 3 << 8;GPIOC->CRL &= ~(3 << 10);}

void Lcd1602_CheckBusy(void)
{
	
	BUSY_IO_IN

	RS_L;
	RW_H;
	
	EN_H;
	while(GPIO_ReadOutputDataBit(GPIOC, GPIO_Pin_2) == 1);
	EN_L;
	
	BUSY_IO_OUT

}

void Lcd1602_SendByte(unsigned char byte)
{
	
	unsigned short value = 0;
	
	value = GPIO_ReadOutputData(GPIOB);
	value &= ~(0x001F << 5);
	value |= ((unsigned short)byte & 0x001F) << 5;
	GPIO_Write(GPIOB, value);
	
	value = GPIO_ReadOutputData(GPIOC);
	value &= ~(0x0007 << 0);
	value |= ((unsigned short)byte & 0x00E0) >> 5;
	GPIO_Write(GPIOC, value);
	
	DelayUs(10);

}

void Lcd1602_WriteCom(unsigned char byte)
{

	RS_L;
	RW_L;
	
	Lcd1602_SendByte(byte);
	
	EN_H;
	DelayUs(20);
	EN_L;
	DelayUs(5);

}

void Lcd1602_WriteCom_Busy(unsigned char byte)
{
	
	//Lcd1602_CheckBusy();
	DelayXms(10);

	RS_L;
	RW_L;
	
	Lcd1602_SendByte(byte);
	
	EN_H;
	DelayUs(20);
	EN_L;
	DelayUs(5);

}

void Lcd1602_WriteData(unsigned char byte)
{

	RS_H;
	RW_L;
	
	Lcd1602_SendByte(byte);

	EN_H;
	DelayUs(20);
	EN_L;
	DelayUs(5);

}

void Lcd1602_Init(void)
{

	GPIO_InitTypeDef gpioInitStrcut;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB | RCC_APB2Periph_GPIOC, ENABLE);
	
	gpioInitStrcut.GPIO_Mode = GPIO_Mode_Out_PP;
	gpioInitStrcut.GPIO_Pin = GPIO_Pin_5 | GPIO_Pin_6 | GPIO_Pin_7 | GPIO_Pin_8 | GPIO_Pin_9;
	gpioInitStrcut.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &gpioInitStrcut);
	
	gpioInitStrcut.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3 | GPIO_Pin_6;
	GPIO_Init(GPIOC, &gpioInitStrcut);
	
	gpioInitStrcut.GPIO_Pin = GPIO_Pin_11;
	GPIO_Init(GPIOA, &gpioInitStrcut);
	
	DelayXms(15);
    Lcd1602_WriteCom(0x38);
    DelayXms(5);
    Lcd1602_WriteCom(0x38);
    DelayXms(5);
    Lcd1602_WriteCom(0x38);
    Lcd1602_WriteCom_Busy(0x38);
    Lcd1602_WriteCom_Busy(0x08);
    Lcd1602_WriteCom_Busy(0x01);
    Lcd1602_WriteCom_Busy(0x06);
    Lcd1602_WriteCom_Busy(0x0c);
	
    EN_L;

}

void Lcd1602_Clear(unsigned char pos)
{

	switch(pos)
	{
		case 0x80:
			
			Lcd1602_DisString(0x80, "                ");
		
		break;
		
		case 0xC0:
			
			Lcd1602_DisString(0xC0, "                ");
		
		break;
		
		case 0xFF:
			
			Lcd1602_WriteCom_Busy(0x01);
		
		break;
	}

}

void Lcd1602_DisString(unsigned char pos, char *fmt,...)
{

	unsigned char LcdPrintfBuf[33];
	unsigned char count = 0;
	va_list ap;
	unsigned char *pStr = LcdPrintfBuf;
	
	va_start(ap,fmt);
	vsprintf(LcdPrintfBuf, fmt, ap);
	va_end(ap);
	
	Lcd1602_WriteCom_Busy(pos);
	
	while(*pStr != 0)
	{
		Lcd1602_WriteData(*pStr++);
		
		if(++count > 15 && pos == 0x80)
		{
			count = 0;
			Lcd1602_WriteCom_Busy(pos + 0x40);
			DelayXms(1);
		}
	}

}
