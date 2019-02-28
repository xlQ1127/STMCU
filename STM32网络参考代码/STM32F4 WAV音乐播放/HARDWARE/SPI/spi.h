#ifndef __SPI_H
#define __SPI_H
#include "sys.h"
//Mini STM32开发板
//SPI 驱动 V1.1
//正点原子@ALIENTEK
//2010/5/13	

// SPI总线速度设置 
#define SPI_SPEED_2   0
#define SPI_SPEED_4   1
#define SPI_SPEED_8   2
#define SPI_SPEED_16  3
#define SPI_SPEED_256 4
						  	    													  
void SPIx_Init(SPI_TypeDef *SPIx);			 //初始化SPI口
void SPIx_SetSpeed(SPI_TypeDef *SPIx,u8 SpeedSet); //设置SPI速度   
u8 SPIx_ReadWriteByte(SPI_TypeDef *SPIx,u8 TxData);//SPI总线读写一个字节
		 
#endif

