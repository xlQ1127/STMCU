#ifndef __W25X16_H__
#define __W25X16_H__

#include "stm32f10x.h"
#include "spi.h"
#include "SysTick.h"

//CS选中FLASH
/*
#define	SPI_FLASH_CS(a) if(a)\
					    GPIO_SetBits(GPIOA,GPIO_Pin_2);\
				        else\
					    GPIO_ResetBits(GPIOA,GPIO_Pin_2);
*/
#define SPI_FLASH_CS_H 	GPIOA->BSRR = GPIO_Pin_2
#define SPI_FLASH_CS_L	GPIOA->BRR = GPIO_Pin_2


#define W25Q16 	0XEF14					 
#define SPI_FLASH_TYPE	W25Q16
//W25X16读写
#define FLASH_ID 0XEF14

//指令表
#define W25X_WriteEnable		0x06//写使能
#define W25X_WriteDisable		0x04//写保护 
#define W25X_ReadStatusReg		0x05//读状态寄存器
#define W25X_WriteStatusReg		0x01//写状态寄存器 
#define W25X_ReadData			0x03//读数据
#define W25X_FastReadData		0x0B//快读 
#define W25X_FastReadDual		0x3B//快读双输出 
#define W25X_PageProgram		0x02//页编程 
#define W25X_BlockErase			0xD8//块擦除(64K) 
#define W25X_SectorErase		0x20//扇区擦除(4K) 
#define W25X_ChipErase			0xC7//芯片擦除
#define W25X_PowerDown			0xB9//掉电
#define W25X_ReleasePowerDown	0xAB//释放掉电 
#define W25X_DeviceID			0xAB// 器件ID
#define W25X_ManufactDeviceID	0x90//制造/器件ID 
#define W25X_JedecDeviceID		0x9F// JEDEC ID

void SPI_Flash_Init(void);
u16  SPI_Flash_ReadID(void);  	    //读取FLASH ID
u8	 SPI_Flash_ReadSR(void);        //读取状态寄存器
void SPI_Flash_Wait_Busy(void);     //等待空闲 
void SPI_FLASH_Write_SR(u8 sr);  	//写状态寄存器
void SPI_FLASH_Write_Enable(void);  //写使能 
void SPI_FLASH_Write_Disable(void);	//写保护
void SPI_Flash_Erase_Chip(void);    	  //整片擦除
void SPI_Flash_Erase_Sector(u32 Dst_Addr);//扇区擦除
void SPI_Flash_PowerDown(void);           //进入掉电模式
void SPI_Flash_WAKEUP(void);			  //唤醒

void SPI_Flash_Init(void);//初始化Flash
//SPI在一页(0~65535)内写入少于256个字节的数据
void SPI_Flash_Write_Page(u8* pBuffer,u32 WriteAddr,u16 NumByteToWrite);
//无检验写SPI FLASH
void SPI_Flash_Write_NoCheck(u8* pBuffer,u32 WriteAddr,u16 NumByteToWrite);

void SPI_Flash_Read(u8* pBuffer,u32 ReadAddr,u16 NumByteToRead);   //读取flash
void SPI_Flash_Write(u8* pBuffer,u32 WriteAddr,u16 NumByteToWrite);//写入flash










#endif

