#include "24c02.h"


/*
 * 函数名：AT24C02_Init
 * 描述  ：AT24C02初始化(IO口初始化)
 * 输入  ：无
 * 输出  ：无
 * 调用  ：外部调用
 */
void AT24C02_Init(void)
{
	I2C_Init_IO();
}

/*
 * 函数名：AT24C02_WriteOneByte
 * 描述  ：在AT24C02指定地址写入一个数据
 * 输入  ：WriteAddr  :写入数据的目的地址
 		   DataToWrite:要写入的数据
 * 输出  ：无
 * 调用  ：外部调用
 */
void AT24C02_WriteOneByte(u8 WriteAddr,u8 DataToWrite)
{
	Single_Write(WriteAddr,DataToWrite,0xA0);
}

/*
 * 函数名：AT24C02_ReadOneByte
 * 描述  ：在AT24C02指定地址读出一个数据
 * 输入  ：ReadAddr:开始读数的地址
 * 输出  ：读到的数据
 * 调用  ：外部调用
 */
u8 AT24C02_ReadOneByte(u8 ReadAddr)
{
	u8 databyte;
	databyte = Single_Read(ReadAddr,0xA0);
	return databyte;
		
}

//在AT24C02里面的指定地址开始写入长度为Len的数据
//该函数用于写入16bit或者32bit的数据！！！
//先低字节 再高字节！！！
//WriteAddr  :开始写入的地址  
//DataToWrite:数据数组首地址
//Len        :要写入数据的长度2,4
void AT24C02_WriteLenByte(u8 WriteAddr,u32 DataToWrite,u8 Len)
{
	u8 i;
	for(i=0;i<Len;i++)
	{
		AT24C02_WriteOneByte(WriteAddr+i,(DataToWrite>>(8*i))&0xFF);	
	} 	
}


/*
 * 函数名：AT24C02_ReadLenByte
 * 描述  ：在AT24C02里面的指定地址开始读出长度为Len的数据,该函数用于读出16bit或者32bit的数据.
 		   存放方式为：先低字节 再高字节！！！
 * 输入  ：ReadAddr   :开始读出的地址
 		   Len        :要读出数据的长度2,4
 * 输出  ：读到的数据
 * 调用  ：外部调用
 */
u32 AT24C02_ReadLenByte(u8 ReadAddr,u8 Len)
{
	u8 i;
	u32 databyte=0;
	for(i=0;i<Len;i++)
	{
		databyte <<= 8;
		databyte += AT24C02_ReadOneByte(ReadAddr+Len-i-1);	
	}
	return databyte;	
}


/*
 * 函数名：AT24C02_Write
 * 描述  ：在AT24C02里面写入数组
 * 输入  ：WriteAddr :开始写入的首地址
		   pBuffer   :数据数组首地址
		   NumToWrite:要写入数据的个数
 * 输出  ：无
 * 调用  ：外部调用
 */
void AT24C02_Write(u8 WriteAddr,u8 *pBuffer,u8 NumToWrite)
{
	u8 i;
	for(i=0;i<NumToWrite;i++)
	{
		AT24C02_WriteOneByte(WriteAddr+i,pBuffer[i]);	
	}	
}

//读出写入AT24C02里的数组


/*
 * 函数名：AT24C02_Read
 * 描述  ：读出写入AT24C02里的数组
 * 输入  ：ReadAddr :开始读出的首地址
		   pBuffer  :数据数组首地址
           NumToRead:要读出数据的个数
 * 输出  ：以pBuffer为指针的一位数组
 * 调用  ：外部调用
 */
void AT24C02_Read(u8 ReadAddr,u8 *pBuffer,u8 NumToRead)
{
	u8 i;
	for(i=0;i<NumToRead;i++)
	{
		pBuffer[i] = AT24C02_ReadOneByte(ReadAddr+i);	
	}
}












