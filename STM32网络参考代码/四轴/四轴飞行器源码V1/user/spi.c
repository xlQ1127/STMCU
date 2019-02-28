/***********************************************

标题: spi.c
作者: 秋阳电子
网址：http://qiuyangdz.taobao.com
日期: 2014/05/18
版本：v1.0
功能: spi初始化以spi及数据读写
说明：spi1外设的初始化
*************************************************/
#include "stm32f10x.h"
#include "stm32f10x_spi.h"
#include "spi.h"
/*************************************************

名称：spi_init(void)
功能：spi外设1初始化
输入参数：无
输出参数：无
返回值：  无
**************************************************/
void spi_init(void)
{
  SPI_InitTypeDef  SPI_InitStructure;
  GPIO_InitTypeDef GPIO_InitStructure;
 
  /* 配置SPI1管脚 */
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO|RCC_APB2Periph_GPIOA, ENABLE);

  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5 | GPIO_Pin_6 |GPIO_Pin_7;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP; 
  GPIO_Init(GPIOA, &GPIO_InitStructure);

  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_Init(GPIOA, &GPIO_InitStructure);
 
  /* SPI1配置选项 */
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1 ,ENABLE);
   
  SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex; 
  SPI_InitStructure.SPI_Mode = SPI_Mode_Master;
  SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;
  SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;
  SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge;
  SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;
  SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_16;
  SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
  SPI_InitStructure.SPI_CRCPolynomial = 7;

  SPI_Init(SPI1, &SPI_InitStructure);

  /* 使能SPI1 */
  SPI_Cmd(SPI1, ENABLE); 	          
}
/*************************************************

名称：spi_rw_byte(unsigned char dt)
功能：spi字节读写
输入参数：写入值
输出参数：无
返回值：  读出值
**************************************************/
u8 spi_rw_byte(unsigned char dt)
{
  while(SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET);
  SPI_I2S_SendData(SPI1, dt);

  while(SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_RXNE) == RESET);
  return SPI_I2S_ReceiveData(SPI1);
}
/*************************************************

名称：spi_nss_low(void)
功能：spi nss管脚置低
输入参数：无
输出参数：无
返回值：  无
**************************************************/
void spi_nss_low(void)
{
  GPIO_ResetBits(GPIOA, GPIO_Pin_4);
}
/*************************************************

名称：spi_nss_high(void)
功能：spi nss管脚置高
输入参数：无
输出参数：无
返回值：  无
**************************************************/
void spi_nss_high(void)
{
  GPIO_SetBits(GPIOA, GPIO_Pin_4);
}
/*************************************************

名称：spi_rw(u8 *data_buff, u8 byte_quantity, u8 reg_address, u8 control_byte)
功能：spi多字节读写
输入参数：
    u8 *data_buff     数据指针
	u8 byte_quantity  读写字节数量
	u8 reg_address    寄存器地址
	u8 control_byte   读写控制标识
输出参数：无
返回值：  无
**************************************************/
void spi_rw(u8 *data_buff, u8 byte_quantity, u8 reg_address, u8 control_byte)
{
  u8 i;
  if(control_byte == 0)  //write
  { 		 
    spi_nss_low();

	spi_rw_byte(reg_address);

	for(i = 0; i < byte_quantity; i++)
	{
	  spi_rw_byte(*data_buff);
	  data_buff++;
	}

	spi_nss_high();
  }
  else if(control_byte == 1)
  {
    spi_nss_low();

	spi_rw_byte(reg_address);

	for(i = 0; i < byte_quantity; i++)
	{
	  *data_buff = spi_rw_byte(0);
	  data_buff++;
	}

	spi_nss_high();
  }
}
/***************************END OF FILE**********************************************************************/
