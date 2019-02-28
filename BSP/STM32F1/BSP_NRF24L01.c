#include "BSP_NRF24L01.h"
/*
#define hNRF hspix  在对应的硬件定义（SPI.h文件）
*/
static uint8_t TX_ADDRESS[5] = {0x1A, 0x3B, 0x5C, 0x7D, 0x9E}; //本地地址
static uint8_t RX_ADDRESS[5] = {0x1A, 0x3B, 0x5C, 0x7D, 0x9E}; //接收地址

/**
  * 函数功能: 往串行Flash读取写入一个字节数据并接收一个字节数据
  * 输入参数: byte：待发送数据
  * 返 回 值: uint8_t：接收到的数据
  * 说    明：无
  */
uint8_t SPIx_ReadWriteByte(SPI_HandleTypeDef *hspi, uint8_t byte)
{
  uint8_t d_read, d_send = byte;
  if (HAL_SPI_TransmitReceive(hspi, &d_send, &d_read, 1, 0xFF) != HAL_OK)
  {
    d_read = 0xFF;
  }
  return d_read;
}

/**
  * 函数功能: SPI写寄存器
  * 输入参数: 无
  * 返 回 值: 无
  * 说    明：reg:指定寄存器地址
  *           
  */
uint8_t NRF24L01_Write_Reg(uint8_t reg, uint8_t value)
{
  uint8_t status;
  NRF24L01_SPI_CS_ENABLE();                //使能SPI传输
  status = SPIx_ReadWriteByte(&hNRF, reg); //发送寄存器号
  SPIx_ReadWriteByte(&hNRF, value);        //写入寄存器的值
  NRF24L01_SPI_CS_DISABLE();               //禁止SPI传输
  return (status);                         //返回状态值
}

/**
  * 函数功能: 读取SPI寄存器值
  * 输入参数: 无
  * 返 回 值: 无
  * 说    明：reg:要读的寄存器
  *           
  */
uint8_t NRF24L01_Read_Reg(uint8_t reg)
{
  uint8_t reg_val;
  NRF24L01_SPI_CS_ENABLE();                  //使能SPI传输
  SPIx_ReadWriteByte(&hNRF, reg);            //发送寄存器号
  reg_val = SPIx_ReadWriteByte(&hNRF, 0XFF); //读取寄存器内容
  NRF24L01_SPI_CS_DISABLE();                 //禁止SPI传输
  return (reg_val);                          //返回状态值
}

/**
  * 函数功能: 在指定位置读出指定长度的数据
  * 输入参数: 无
  * 返 回 值: 此次读到的状态寄存器值 
  * 说    明：无
  *           
  */
uint8_t NRF24L01_Read_Buf(uint8_t reg, uint8_t *pBuf, uint8_t len)
{
  uint8_t status, uint8_t_ctr;

  NRF24L01_SPI_CS_ENABLE();                //使能SPI传输
  status = SPIx_ReadWriteByte(&hNRF, reg); //发送寄存器值(位置),并读取状态值
  for (uint8_t_ctr = 0; uint8_t_ctr < len; uint8_t_ctr++)
  {
    pBuf[uint8_t_ctr] = SPIx_ReadWriteByte(&hNRF, 0XFF); //读出数据
  }
  NRF24L01_SPI_CS_DISABLE(); //关闭SPI传输
  return status;             //返回读到的状态值
}

/**
  * 函数功能: 在指定位置写指定长度的数据
  * 输入参数: 无
  * 返 回 值: 无
  * 说    明：reg:寄存器(位置)  *pBuf:数据指针  len:数据长度
  *           
  */
uint8_t NRF24L01_Write_Buf(uint8_t reg, uint8_t *pBuf, uint8_t len)
{
  uint8_t status, uint8_t_ctr;
  NRF24L01_SPI_CS_ENABLE();                //使能SPI传输
  status = SPIx_ReadWriteByte(&hNRF, reg); //发送寄存器值(位置),并读取状态值
  for (uint8_t_ctr = 0; uint8_t_ctr < len; uint8_t_ctr++)
  {
    SPIx_ReadWriteByte(&hNRF, *pBuf++); //写入数据
  }
  NRF24L01_SPI_CS_DISABLE(); //关闭SPI传输
  return status;             //返回读到的状态值
}

/**
  * 函数功能: 检测24L01是否存在
  * 输入参数: 无
  * 返 回 值: 0，成功;1，失败
  * 说    明：无          
  */
uint8_t NRF24L01_Check(void)
{
  uint8_t buf[5] = {0xA5, 0xA5, 0xA5, 0xA5, 0xA5}; //0xA5  0xC2
  uint8_t i;

  NRF24L01_Write_Buf(NRF_WRITE_REG + TX_ADDR, buf, 5); //写入5个字节的地址.
  NRF24L01_Read_Buf(TX_ADDR, buf, 5);                  //读出写入的地址
  for (i = 0; i < 5; i++)
    if (buf[i] != 0XA5)
      break;
  if (i != 5)
    return 1; //检测24L01错误
  return 0;   //检测到24L01
}

/******************************************************************************
函数原型：	void NRF24L01_Init(uint8_t Channal,uint8_t Mode)
功    能：	NRF24L01初始化
参    数：	Chanal 40，RF通道
Mode   1:TX  2:RX  
*******************************************************************************/

void NRF24L01_Init(uint8_t Channal, uint8_t Mode)
{
  NRF24L01_CE_LOW();
  NRF24L01_Write_Reg(FLUSH_TX, 0xff); //清空发送缓冲区
  NRF24L01_Write_Reg(FLUSH_RX, 0xff); //清空接收缓冲区

  NRF24L01_Write_Buf(NRF_WRITE_REG + TX_ADDR, TX_ADDRESS, 5);    //写TX节点地址
  NRF24L01_Write_Buf(NRF_WRITE_REG + RX_ADDR_P0, RX_ADDRESS, 5); //写RX节点地址

  NRF24L01_Write_Reg(NRF_WRITE_REG + EN_AA, 0x01);      //使能通道0的自动应答
  NRF24L01_Write_Reg(NRF_WRITE_REG + EN_RXADDR, 0x01);  //使能通道0的接收地址
  NRF24L01_Write_Reg(NRF_WRITE_REG + SETUP_RETR, 0x1a); //设置自动重发间隔时间:500us;最大自动重发次数:10次
  NRF24L01_Write_Reg(NRF_WRITE_REG + RF_CH, Channal);   //设置RF通道为CHANAL
  NRF24L01_Write_Reg(NRF_WRITE_REG + RX_PW_P0, 32);     //设置通道0的有效数据宽度
  NRF24L01_Write_Reg(NRF_WRITE_REG + RF_SETUP, 0x0f);   //设置TX发射参数,0db增益,2Mbps,低噪声增益开启

  if (Mode == 1)
    NRF24L01_Write_Reg(NRF_WRITE_REG + CONFIG, 0x0E); //发送
  else if (Mode == 2)
    NRF24L01_Write_Reg(NRF_WRITE_REG + CONFIG, 0x0F); //接收

  NRF24L01_CE_HIGH(); //CE为高,进入接收模式
}

/**
  * 函数功能: 该函数初始化NRF24L01到RX模式
  * 输入参数: 无
  * 返 回 值: 无
  * 说    明：无
  *           
  */
void NRF24L01_RX_Mode(void)
{
  NRF24L01_CE_LOW();
  NRF24L01_Write_Reg(NRF_WRITE_REG + CONFIG, 0x0F); //配置基本工作模式的参数;PWR_UP,EN_CRC,16BIT_CRC
  NRF24L01_CE_HIGH();                               //CE为高,进入接收模式
}

/**
  * 函数功能: 该函数初始化NRF24L01到TX模式
  * 输入参数: 无
  * 返 回 值: 无
  * 说    明：无
  *           
  */
void NRF24L01_TX_Mode(void)
{
  NRF24L01_CE_LOW();
  NRF24L01_Write_Reg(NRF_WRITE_REG + CONFIG, 0x0e); //配置基本工作模式的参数;PWR_UP,EN_CRC,16BIT_CRC,接收模式,开启所有中断
  NRF24L01_CE_HIGH();                               //CE为高,10us后启动发送
}

/**
  * 函数功能: 启动NRF24L01发送一次数据
  * 输入参数: 无
  * 返 回 值: 发送完成状况
  * 说    明：txbuf:待发送数据首地址
  *           
  */
void NRF24L01_TxPacket(uint8_t *txbuf)
{
  //	 uint8_t sta;

  NRF24L01_CE_LOW();
  NRF24L01_Write_Buf(WR_TX_PLOAD, txbuf, TX_PLOAD_WIDTH); //写数据到TX BUF  32个字节
  NRF24L01_CE_HIGH();                                     //启动发送

  //	while( NRF24L01_IRQ_PIN_READ()!=0);//等待发送完成
  //
  //	sta=NRF24L01_Read_Reg(STATUS);  //读取状态寄存器的值
  //	NRF24L01_Write_Reg(NRF_WRITE_REG+STATUS,sta); //清除TX_DS或MAX_RT中断标志
  //	if(sta&MAX_TX)//达到最大重发次数
  //	{
  //		NRF24L01_Write_Reg(FLUSH_TX,0xff);//清除TX FIFO寄存器
  //		return MAX_TX;
  //	}
  //	if(sta&TX_OK)//发送完成
  //	{
  //		return TX_OK;
  //	}
  //	return 0xff;//其他原因发送失败
}

/******************************************************************************
函数原型：	void NRF24L01_IRQ(void)
功    能：	NRF24L01中断
*******************************************************************************/
void NRF24L01_IRQ(uint8_t *txbuf)
{
  uint8_t status = NRF24L01_Read_Reg(NRF_READ_REG + STATUS);

  if (status & (1 << RX_DR)) //接收中断
  {
    uint8_t rx_len = NRF24L01_Read_Reg(R_RX_PL_WID); //收到数据长度
    if (rx_len == 32)
    {
      NRF24L01_Read_Buf(RD_RX_PLOAD, txbuf, rx_len); //读取接收FIFO数据
    }
    else
    {
      NRF24L01_Write_Reg(FLUSH_RX, 0xff); //清空接收缓冲区
    }
  }
  if (status & (1 << MAX_RT)) //达到最多次重发中断
  {
    if (status & (1 << TX_FULL)) //TX FIFO 溢出
    {
      NRF24L01_Write_Reg(FLUSH_TX, 0xff); //清空发送缓冲区
    }
  }
  //	if(status & (1<<TX_DS))//发送完成
  //	{
  NRF24L01_RX_Mode();                                 //设置Nrf2401为接收模式
                                                      //	}
  NRF24L01_Write_Reg(NRF_WRITE_REG + STATUS, status); //清除中断标志位
}

/**
  * 函数功能:启动NRF24L01接收一次数据
  * 输入参数: 无
  * 返 回 值: 无
  * 说    明：无
  *           
  */
void NRF24L01_RxPacket(uint8_t *rxbuf)
{
  //	uint8_t sta;

  //	sta=NRF24L01_Read_Reg(STATUS);  //读取状态寄存器的值
  //	NRF24L01_Write_Reg(NRF_WRITE_REG+STATUS,sta); //清除TX_DS或MAX_RT中断标志

  //	if(sta&RX_OK)//接收到数据
  //	{
  //		NRF24L01_Read_Buf(RD_RX_PLOAD,rxbuf,RX_PLOAD_WIDTH);//读取数据
  //		NRF24L01_Write_Reg(FLUSH_RX,0xff);//清除RX FIFO寄存器
  //		return RX_OK;
  //	}
  //	return 1;//没收到任何数据
}

/******************* (C) COPYRIGHT 2015-2020 硬石嵌入式开发团队 *****END OF FILE****/
