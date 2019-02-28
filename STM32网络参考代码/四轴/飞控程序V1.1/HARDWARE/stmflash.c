#include "stmflash.h"


//读一个半字数据
u16 STMFLASH_ReadHalfWord(u32 faddr)
{
	return *(vu16*)faddr; 
}

//读多个半字数据
void STMFLASH_Read(u32 ReadAddr,u16 *pBuffer,u16 NumToRead)   	
{
	u16 i;
	for(i=0;i<NumToRead;i++)
	{
		pBuffer[i]=STMFLASH_ReadHalfWord(ReadAddr);//读取2个字节.
		ReadAddr+=2;//偏移2个字节.	
	}
}

//写多个数据到一个页
u8 STMFLASH_Write(u32 PageAddr,u16 *pBuffer,u16 NumToWrite)	
{
	
	FLASH_Status temp_status;//状态
	u16 i;
	
	FLASH_Unlock();//解锁
	
	temp_status = FLASH_ErasePage(PageAddr);//擦除一个页
	if(temp_status != FLASH_COMPLETE)
	{
		FLASH_Lock();
		return 0;
	}
	
	for(i=0;i<NumToWrite;i++)//数据写入
	{
		temp_status = FLASH_ProgramHalfWord(PageAddr+2*i,pBuffer[i]);
		if(temp_status != FLASH_COMPLETE)
		{
			FLASH_Lock();
			return 0;
		}
	}
	
	FLASH_Lock();//上锁
	return 1;
}






