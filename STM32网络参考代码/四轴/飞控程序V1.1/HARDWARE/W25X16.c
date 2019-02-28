#include "W25X16.h"

//W25X16
//256个字节为一页
//16页为一个扇区Sectors(4K)
//16个扇区为1个块区Block(64K)
//容量为2M字节,共有32个块区,512个扇区,8192个页


/*
 * 函数名：SPI_Flash_Init
 * 描述  ：初始化SPI FLASH的IO口
 * 输入  ：无
 * 输出  ：无
 * 调用  ：外部调用
 */
void SPI_Flash_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA,ENABLE);//GPIOA 时钟使能 
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2|GPIO_Pin_3|GPIO_Pin_4;  //SPI CS
 	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;  //推挽输出
 	GPIO_Init(GPIOA, &GPIO_InitStructure);
 	GPIO_SetBits(GPIOA,GPIO_Pin_2|GPIO_Pin_3|GPIO_Pin_4);
	
	SPIx_SetSpeed(SPI_BaudRatePrescaler_4);//4分频
	SPIx_Init();	
}


/*
 * 函数名：SPI_Flash_ReadID
 * 描述  ：读取芯片ID W25X16的ID:0xEF14
 * 输入  ：无
 * 输出  ：芯片的ID号
 * 调用  ：外部调用
 */
u16  SPI_Flash_ReadID(void)
{
	u16 temp = 0;
	SPI_FLASH_CS_L;
	SPIx_ReadWriteByte(W25X_ManufactDeviceID);
	SPIx_ReadWriteByte(0x00);
	SPIx_ReadWriteByte(0x00);
	SPIx_ReadWriteByte(0x00);
	temp |= (SPIx_ReadWriteByte(0xFF)<<8);
	temp |= SPIx_ReadWriteByte(0xFF);	
	SPI_FLASH_CS_H;
	return temp;	
}




/*
 * 函数名：SPI_Flash_ReadSR
 * 描述  ：读取SPI_FLASH的状态寄存器
		   BIT7  6   5   4   3   2   1   0
		   SPR   RV  TB BP2 BP1 BP0 WEL BUSY
 * 输入  ：无
 * 输出  ：状态寄存器的值
 		   SPR:默认0,状态寄存器保护位,配合WP使用
		   TB,BP2,BP1,BP0:FLASH区域写保护设置
		   WEL:写使能锁定
		   BUSY:忙标记位(1,忙;0,空闲)
		   默认:0x00
 * 调用  ：外部调用
 */
u8 SPI_Flash_ReadSR(void)
{
	u8 byte;
	SPI_FLASH_CS_L;
	SPIx_ReadWriteByte(W25X_ReadStatusReg);
	byte = SPIx_ReadWriteByte(0xFF);
	SPI_FLASH_CS_H;
	return byte;		
}


/*
 * 函数名：SPI_Flash_Wait_Busy
 * 描述  ：等待空闲
 * 输入  ：无
 * 输出  ：无
 * 调用  ：外部调用
 */
void SPI_Flash_Wait_Busy(void)
{ 
	while(SPI_Flash_ReadSR()&0x01);
}


/*
 * 函数名：SPI_FLASH_Write_SR
 * 描述  ：SPI_FLASH状态寄存器(只有SPR,TB,BP2,BP1,BP0(bit 7,5,4,3,2)可以写!!!)
 * 输入  ：写入数值
 * 输出  ：无
 * 调用  ：外部调用
 */
void SPI_FLASH_Write_SR(u8 sr)
{
	SPI_FLASH_CS_L;
	SPIx_ReadWriteByte(W25X_WriteStatusReg);
	SPIx_ReadWriteByte(sr);	
	SPI_FLASH_CS_H;	
}

/*
 * 函数名：SPI_FLASH_Write_Enable
 * 描述  ：SPI_FLASH写使能(将WEL置位)
 * 输入  ：无
 * 输出  ：无
 * 调用  ：外部调用
 */
void SPI_FLASH_Write_Enable(void)
{
	SPI_FLASH_CS_L;
	SPIx_ReadWriteByte(W25X_WriteEnable);
	SPI_FLASH_CS_H;	
}


/*
 * 函数名：SPI_FLASH_Write_Disable
 * 描述  ：SPI_FLASH写禁止(将WEL清零)
 * 输入  ：无
 * 输出  ：无
 * 调用  ：外部调用
 */
void SPI_FLASH_Write_Disable(void)
{
	SPI_FLASH_CS_L;
	SPIx_ReadWriteByte(W25X_WriteDisable);
	SPI_FLASH_CS_H;		
}

/*
 * 函数名：SPI_Flash_Erase_Chip
 * 描述  ：擦除整个芯片 25s 
 * 输入  ：无
 * 输出  ：无
 * 调用  ：外部调用
 */
void SPI_Flash_Erase_Chip(void)
{
	SPI_FLASH_Write_Enable();
	SPI_Flash_Wait_Busy();
	SPI_FLASH_CS_L;
	SPIx_ReadWriteByte(W25X_ChipErase);
	SPI_FLASH_CS_H;
	SPI_Flash_Wait_Busy();
}


/*
 * 函数名：SPI_Flash_Erase_Sector
 * 描述  ：擦除一个扇区(擦除一个山区的最少时间:150ms)
 * 输入  ：Dst_Addr:扇区地址 0~511 for w25x16
 * 输出  ：无
 * 调用  ：外部调用
 */
void SPI_Flash_Erase_Sector(u32 Dst_Addr)
{
	Dst_Addr *= 4096;
	SPI_FLASH_Write_Enable();
	SPI_Flash_Wait_Busy();
	SPI_FLASH_CS_L;
	SPIx_ReadWriteByte(W25X_SectorErase);
	SPIx_ReadWriteByte((u8)((Dst_Addr)>>16));  //发送24bit地址    
    SPIx_ReadWriteByte((u8)((Dst_Addr)>>8));   
    SPIx_ReadWriteByte((u8)Dst_Addr);
	SPI_FLASH_CS_H;	
	SPI_Flash_Wait_Busy();
}


/*
 * 函数名：SPI_Flash_PowerDown
 * 描述  ：进入掉电模式
 * 输入  ：无
 * 输出  ：无
 * 调用  ：外部调用
 */
void SPI_Flash_PowerDown(void)
{
	SPI_FLASH_CS_L;
	SPIx_ReadWriteByte(W25X_PowerDown);
	SPI_FLASH_CS_H;
	delay_us(3);//等待TPD 		
}


/*
 * 函数名：SPI_Flash_WAKEUP
 * 描述  ：唤醒
 * 输入  ：无
 * 输出  ：无
 * 调用  ：外部调用
 */
void SPI_Flash_WAKEUP(void)
{
	SPI_FLASH_CS_L;
	SPIx_ReadWriteByte(W25X_ReleasePowerDown);
	SPI_FLASH_CS_H;
	delay_us(3);//等待TRES1	
}



/*
 * 函数名：SPI_Flash_Write_Page
 * 描述  ：SPI在一页(0~65535)内写入少于256个字节的数据
 * 输入  ：pBuffer:数据存储区
		   WriteAddr:开始写入的地址 0~1677216(24bit)
           NumByteToWrite:要写入的字节数(最大256),该数不应该超过该页的剩余字节数!!!
 * 输出  ：无
 * 调用  ：外部调用
 */
void SPI_Flash_Write_Page(u8* pBuffer,u32 WriteAddr,u16 NumByteToWrite)
{
	u16 i;
	SPI_FLASH_Write_Enable();
	SPI_Flash_Wait_Busy();
	SPI_FLASH_CS_L;
	SPIx_ReadWriteByte(W25X_PageProgram);
	SPIx_ReadWriteByte((u8)((WriteAddr)>>16));  //发送24bit地址    
    SPIx_ReadWriteByte((u8)((WriteAddr)>>8));   
    SPIx_ReadWriteByte((u8)WriteAddr);

	for(i=0;i<NumByteToWrite;i++)
	{
		SPIx_ReadWriteByte(pBuffer[i]);
	}
	SPI_FLASH_CS_H;
	SPI_Flash_Wait_Busy();	
}

/*
 * 函数名：SPI_Flash_Write_NoCheck
 * 描述  ：无检验写SPI FLASH 具有自动换页功能(必须确保所写的地址范围内的数据全部为0XFF,否则在非0XFF处写入的数据将失败!)
 * 输入  ：pBuffer:数据存储区
		   WriteAddr:开始写入的地址 0~1677216(24bit)
           NumByteToWrite:要写入的字节数(最大256),该数不应该超过该页的剩余字节数!!!
 * 输出  ：pBuffer:数据存储区
	       WriteAddr:开始写入的地址(24bit)
		   NumByteToWrite:要写入的字节数(最大65535)
 * 调用  ：外部调用
 */
void SPI_Flash_Write_NoCheck(u8* pBuffer,u32 WriteAddr,u16 NumByteToWrite)
{
	/*
	u16 pagermain;
	pagermain = 256 - (WriteAddr%256);//单页剩余的字节数
	if(NumByteToWrite <= pagermain) pagermain = NumByteToWrite;//不大于256个字节
	while(1)
	{
		SPI_Flash_Write_Page(pBuffer,WriteAddr,pagermain);
		if(pagermain == NumByteToWrite) break;//写入结束了
		else
		{
			pBuffer = pBuffer + pagermain;
			WriteAddr = WriteAddr + pagermain;
			
			NumByteToWrite = NumByteToWrite - pagermain;//减去已经写入了的字节数
			if(NumByteToWrite>256) pagermain = 256;//一次可以写入256个字节
			else pagermain = NumByteToWrite;//不够256个字节了 			
		}		
	}
	*/
	u16 x;
	x = 256 - (WriteAddr%256);//第一次写入的最大数据
	while(1)
	{
		if(NumByteToWrite > x)
		{
			
				SPI_Flash_Write_Page(pBuffer,WriteAddr,x);
				NumByteToWrite = NumByteToWrite - x;
				pBuffer = pBuffer + x;
				WriteAddr = WriteAddr + x;
				
				x = 256;//以后每次写入的最大数据位256					
				
		}
		else
		{
			SPI_Flash_Write_Page(pBuffer,WriteAddr,NumByteToWrite);
			break;	
		}
	}
				
}



/*
 * 函数名：SPI_Flash_Read
 * 描述  ：在指定地址开始读取指定长度的数据
 * 输入  ：pBuffer:数据存储区
		   ReadAddr:开始读取的地址(24bit)
 		   NumByteToRead:要读取的字节数(最大65535)
 * 输出  ：无
 * 调用  ：外部调用
 */
void SPI_Flash_Read(u8* pBuffer,u32 ReadAddr,u16 NumByteToRead)
{
	u16 i;
	SPI_FLASH_CS_L;                            //使能器件   
    SPIx_ReadWriteByte(W25X_ReadData);         //发送读取命令   
    SPIx_ReadWriteByte((u8)((ReadAddr)>>16));  //发送24bit地址    
    SPIx_ReadWriteByte((u8)((ReadAddr)>>8));   
    SPIx_ReadWriteByte((u8)ReadAddr);
	for(i=0;i<NumByteToRead;i++)
	{
		pBuffer[i] = SPIx_ReadWriteByte(0xFF);//循环读数		
	}
	SPI_FLASH_CS_H; 	
}



/*
 * 函数名：SPI_Flash_Write
 * 描述  ：写SPI FLASH 在指定地址开始写入指定长度的数据(写入之前，先擦除将要写入的扇区!!!!) 
 * 输入  ：pBuffer:数据存储区
		   WriteAddr:开始写入的地址(24bit)
           NumByteToWrite:要写入的字节数(最大65535)
 * 输出  ：无
 * 调用  ：外部调用
 */
void SPI_Flash_Write(u8* pBuffer,u32 WriteAddr,u16 NumByteToWrite)
{
	u16 startSector,endSector;
	u16 i;
	startSector = (WriteAddr/4096);  //开始的扇区数
	endSector = ((WriteAddr+NumByteToWrite)/4096)+1; //结束的扇区数
	for(i=startSector;i<=endSector;i++) //擦除要写入的扇区
	{
		SPI_Flash_Erase_Sector(i);	
	}
	
	SPI_Flash_Write_NoCheck(pBuffer,WriteAddr,NumByteToWrite);//写入数据			
}






