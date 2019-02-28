#ifndef __STMFLASH_H__
#define __STMFLASH_H__

#include "stm32f10x.h"

#define PAGE1 0x0807F800	//255页

#define ADJUST_FLAG 0xF00F 


void STMFLASH_Read(u32 ReadAddr,u16 *pBuffer,u16 NumToRead);//从指定地址开始读出指定长度的数据

u8 STMFLASH_Write(u32 PageAddr,u16 *pBuffer,u16 NumToWrite);//从指定地址开始写入指定长度的数据



#endif





