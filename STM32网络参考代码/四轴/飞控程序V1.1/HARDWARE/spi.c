#include "spi.h"



SPI_InitTypeDef SPI_InitStructure; //定义SPI配置 结构体

/*
 * 函数名：SPIx_Init
 * 描述  ：SPI1的初始化
 * 输入  ：无
 * 输出  ：无
 * 调用  ：外部调用
 */
void SPIx_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;//定义IO口配置 结构体

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA|RCC_APB2Periph_SPI1,ENABLE);//SPI GPIOA 时钟使能 
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5|GPIO_Pin_6|GPIO_Pin_7;//选中引脚5,6,7
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;//IO输出速度50Hz
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;//复用推挽输出
	GPIO_Init(GPIOA,&GPIO_InitStructure);//配置引脚
	GPIO_SetBits(GPIOA, GPIO_Pin_5 | GPIO_Pin_6| GPIO_Pin_7);//PA 5,6,7 输出高
	
	SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;//SPI设置为双线双向全双工
	SPI_InitStructure.SPI_Mode = SPI_Mode_Master;//设置为主 SPI  
	SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;//SPI发送接收 8 位帧结构  
	SPI_InitStructure.SPI_CPOL = SPI_CPOL_High;//选择了串行时钟的稳态:时钟悬空高
	SPI_InitStructure.SPI_CPHA = SPI_CPHA_2Edge;//数据捕获于第二个时钟沿 
	SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;//NSS信号由硬件（NSS管脚）还是软件（使用SSI位）管理:内部NSS信号有SSI位控制
	SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_256;//波特率预分频值为 256   
	SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;//数据传输从 MSB 位开始 
	SPI_InitStructure.SPI_CRCPolynomial = 7;//SPI_CRCPolynomial定义了用于 CRC值计算的多项式 
	SPI_Init(SPI1, &SPI_InitStructure); //配置SPI1
	
	SPI_Cmd(SPI1, ENABLE);//SPI1使能
	SPIx_ReadWriteByte(0xFF);//启动传输	
}

/*
 * 函数名：SPIx_SetSpeed
 * 描述  ：SPI1速度设置函数
 * 输入  ：SpeedSet
 		   SPI_BaudRatePrescaler_2   2分频   (SPI 36M@sys 72M)
		   SPI_BaudRatePrescaler_8   8分频   (SPI 9M@sys 72M)
		   SPI_BaudRatePrescaler_16  16分频  (SPI 4.5M@sys 72M)
		   SPI_BaudRatePrescaler_256 256分频 (SPI 281.25K@sys 72M)
 * 输出  ：无
 * 调用  ：外部调用
 */
void SPIx_SetSpeed(u8 SpeedSet)
{
	SPI_InitStructure.SPI_BaudRatePrescaler = SpeedSet ;
  	SPI_Init(SPI1,&SPI_InitStructure);
	SPI_Cmd(SPI1,ENABLE);	
}

/*
 * 函数名：SPIx_ReadWriteByte
 * 描述  ：SPI1 读写一个字节(全双工模式，写入数据的同时读取数据)
 * 输入  ：TxData:写入的字节
 * 输出  ：返回读出的字节
 * 调用  ：外部调用
 */
u8 SPIx_ReadWriteByte(u8 TxData)
{
	u8 retry = 0;
	while(SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) != SET)//检测发送缓冲区是否清空
	{
		retry++;
		if(retry>200) return 0;	
	}
	SPI_I2S_SendData(SPI1, TxData);////通过外设SPI1发送一个数据
	retry = 0;
	while(SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_RXNE) != SET)//检测接受缓存是否非空
	{
		retry++;
		if(retry>200) return 0;	
	}
	return SPI_I2S_ReceiveData(SPI1); //返回通过SPIx最近接收的数据
}











