#ifndef __24C02_H__
#define __24C02_H__

#include "stm32f10x.h"
#include "myiic.h"


void AT24C02_Init(void);//初始化IIC接口
					  
void AT24C02_WriteOneByte(u8 WriteAddr,u8 DataToWrite);		//指定地址写入一个字节
u8 AT24C02_ReadOneByte(u8 ReadAddr);							//指定地址读取一个字节

void AT24C02_WriteLenByte(u8 WriteAddr,u32 DataToWrite,u8 Len);//指定地址开始写入指定长度的数据
u32 AT24C02_ReadLenByte(u8 ReadAddr,u8 Len);					//指定地址开始读取指定长度数据

void AT24C02_Write(u8 WriteAddr,u8 *pBuffer,u8 NumToWrite);	//从指定地址开始写入指定长度的数据
void AT24C02_Read(u8 ReadAddr,u8 *pBuffer,u8 NumToRead);   	//从指定地址开始读出指定长度的数据

u8 AT24C02_Check(void);  //检查器件
void AT24C02_Init(void); //初始化IIC








#endif


