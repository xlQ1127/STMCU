#include "spi.h"

//Mini STM32开发板
//SPI 驱动 V1.2
//正点原子@ALIENTEK
//2010/5/23	

//以下是SPI模块的初始化代码，配置成主机模式，访问SD Card/W25X16/24L01/JF24C							  
//SPI口初始化
//这里针是对SPI1的初始化
//void SPIx_Init(void)
//{
//	RCC->APB2RSTR|=1<<12;      //SPI1复位 
//	RCC->APB2RSTR&=~(1<<12);   //SPI1结束复位
//		 
//	RCC->APB2ENR|=1<<2;       //PORTA时钟使能 	 
//	RCC->APB2ENR|=1<<12;      //SPI1时钟使能 
//		   
//	//这里只针对SPI口初始化
//	GPIOA->CRL&=0X000FFFFF; 
//	GPIOA->CRL|=0XBBB00000;//PA5.6.7复用 	    
//	GPIOA->ODR|=0X7<<5;    //PA5.6.7上拉
//		
//	SPI1->CR1|=0<<10;//全双工模式	
//	SPI1->CR1|=1<<9; //软件nss管理
//	SPI1->CR1|=1<<8;  
//
//	SPI1->CR1|=1<<2; //SPI主机
//	SPI1->CR1|=0<<11;//8bit数据格式	
//	//对24L01要设置 CPHA=0;CPOL=0;
//	SPI1->CR1|=1<<1; //CPOL=0时空闲模式下SCK为1 
//	SPI1->CR1|=1<<0; //第一个时钟的下降沿,CPHA=1 CPOL=1
//	  
//	SPI1->CR1|=7<<3; //Fsck=Fcpu/256
//	SPI1->CR1|=0<<7; //MSBfirst   
//	SPI1->CR1|=1<<6; //SPI设备使能
//	SPIx_ReadWriteByte(0xff);//启动传输		 
//}   

void SPIx_Init(SPI_TypeDef *SPIx)
{
	if(SPIx==SPI1)
	{
		RCC->APB2RSTR|=1<<12;      //SPI1复位 
		RCC->APB2RSTR&=~(1<<12);   //SPI1结束复位
		RCC->APB2ENR|=1<<2;       //PORTA时钟使能 	 
		RCC->APB2ENR|=1<<12;      //SPI1时钟使能		   
		//这里只针对SPI口初始化
		GPIOA->CRL&=0X000FFFFF; 
		GPIOA->CRL|=0XBBB00000;//PA5.6.7复用 	    
		GPIOA->ODR|=0X7<<5;    //PA5.6.7上拉
	}
	if(SPIx==SPI2)
	{
		RCC->APB1RSTR|=1<<14;      //SPI2复位 
		RCC->APB1RSTR&=~(1<<14);   //SPI2结束复位
		RCC->APB2ENR|=1<<3;       //PORTB时钟使能 	 
		RCC->APB1ENR|=1<<14;      //SPI2时钟使能		   
		//这里只针对SPI口初始化
		GPIOA->CRL&=0X000FFFFF; 
		GPIOA->CRL|=0XBBB00000;//PA5.6.7复用 	    
		GPIOA->ODR|=0X7<<5;    //PA5.6.7上拉
	}
	if(SPIx==SPI3)
	{
		RCC->APB1RSTR|=1<<15;      //SPI3复位 
		RCC->APB1RSTR&=~(1<<15);   //SPI3结束复位
		RCC->APB2ENR|=1<<3;       //PORTB时钟使能 	 
		RCC->APB1ENR|=1<<15;      //SPI1时钟使能		   
		//这里只针对SPI口初始化
		GPIOA->CRL&=0X000FFFFF; 
		GPIOA->CRL|=0XBBB00000;//PA5.6.7复用 	    
		GPIOA->ODR|=0X7<<5;    //PA5.6.7上拉
	}
	SPIx->CR1|=0<<10;//全双工模式
	SPIx->CR1|=1<<9;//软件nss管理
	SPIx->CR1|=1<<8;
	SPIx->CR1|=1<<2;//SPI主机
	SPIx->CR1|=0<<11;//8bit数据格式
	//对24L01要设置CPHA=0;CPOL=0;
	SPIx->CR1|=1<<1;//CPOL=0时空闲模式下SCK为1
	SPIx->CR1|=1<<0;//第一个时钟的下降沿,CPHA=1CPOL=1
	SPIx->CR1|=7<<3;//Fsck=Fcpu/256
	SPIx->CR1|=0<<7;//MSBfirst
	SPIx->CR1|=1<<6;//SPI设备使能
	SPIx_ReadWriteByte(SPIx,0xff);//启动传输				 
} 
//SPI 速度设置函数
//SpeedSet:
//SPI_SPEED_2   2分频   (SPI 36M@sys 72M)
//SPI_SPEED_4   4分频   (SPI 18M@sys 72M)
//SPI_SPEED_8   8分频   (SPI 9M@sys 72M)
//SPI_SPEED_16  16分频  (SPI 4.5M@sys 72M)
//SPI_SPEED_256 256分频 (SPI 281.25K@sys 72M)
void SPIx_SetSpeed(SPI_TypeDef *SPIx,u8 SpeedSet)
{
	SPIx->CR1&=0XFFC7;//Fsck=Fcpu/256
	switch(SpeedSet)
	{
		case SPI_SPEED_2://二分频
			SPIx->CR1|=0<<3;//Fsck=Fpclk/2=36Mhz
			break;
		case SPI_SPEED_4://四分频
			SPIx->CR1|=1<<3;//Fsck=Fpclk/4=18Mhz
			break;
		case SPI_SPEED_8://八分频
			SPIx->CR1|=2<<3;//Fsck=Fpclk/8=9Mhz
			break;
		case SPI_SPEED_16://十六分频
			SPIx->CR1|=3<<3;//Fsck=Fpclk/16=4.5Mhz
			break;
		case SPI_SPEED_256://256分频
			SPIx->CR1|=7<<3;//Fsck=Fpclk/16=281.25Khz
			break;
	}		 
	SPIx->CR1|=1<<6; //SPI设备使能	  
} 
//SPIx 读写一个字节
//TxData:要写入的字节
//返回值:读取到的字节
u8 SPIx_ReadWriteByte(SPI_TypeDef *SPIx,u8 TxData)
{		
	u8 retry=0;				 
	while((SPIx->SR&1<<1)==0)//等待发送区空	
	{
		retry++;
		if(retry>200)return 0;
	}			  
	SPIx->DR=TxData;	 	  //发送一个byte 
	retry=0;
	while((SPIx->SR&1<<0)==0) //等待接收完一个byte  
	{
		retry++;
		if(retry>200)return 0;
	}	  						    
	return SPIx->DR;          //返回收到的数据				    
}































