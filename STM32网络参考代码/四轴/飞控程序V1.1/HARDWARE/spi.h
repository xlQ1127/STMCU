#ifndef __SPI_H__
#define __SPI_H__

#include "stm32f10x.h"


void SPIx_Init(void);			 //初始化SPI口
void SPIx_SetSpeed(u8 SpeedSet); //设置SPI速度   
u8 SPIx_ReadWriteByte(u8 TxData);//SPI总线读写一个字节

#endif


