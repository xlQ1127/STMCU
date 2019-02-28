/***********************************************

标题: nrf24l01.c
作者: 秋阳电子
网址：http://qiuyangdz.taobao.com
日期: 2014/05/18
版本：v1.0
功能: nrf24l01+初始化及数据读取
说明：
*************************************************/
#include "stm32f10x.h"
#include "stm32f10x_exti.h"
#include "misc.h"
#include "nrf24l01.h"
#include "spi.h"

/* 寄存器指令 */ 
#define NRF_READ_REG    0x00  // 读寄存器指令 
#define NFR_WRITE_REG   0x20  // 写寄存器指令 
#define RD_RX_PLOAD     0x61  // 读取接收数据指令 
#define WR_TX_PLOAD     0xA0  // 写待发数据指令 
#define FLUSH_TX        0xE1  // 清空发送 FIFO指令 
#define FLUSH_RX        0xE2  // 清空接收 FIFO指令 
#define REUSE_TX_PL     0xE3  // 定义重复装载数据指令 
#define NOP             0xFF  // 保留
 
/* 寄存器地址 */ 
#define CONFIG          0x00  // 配置收发状态，CRC校验模式以及收发状态响应方式 
#define EN_AA           0x01  // 自动应答功能设置 
#define EN_RXADDR       0x02  // 可用信道设置 
#define SETUP_AW        0x03  // 收发地址宽度设置 
#define SETUP_RETR      0x04  // 自动重发功能设置 
#define RF_CH           0x05  // 工作频率设置 
#define RF_SETUP        0x06  // 发射速率、功耗功能设置 
#define NRF_STATUS      0x07  // 状态寄存器 
#define OBSERVE_TX      0x08  // 发送监测功能 
#define CD              0x09  // 地址检测            
#define RX_ADDR_P0      0x0A  // 频道0接收数据地址 
#define RX_ADDR_P1      0x0B  // 频道1接收数据地址 
#define RX_ADDR_P2      0x0C  // 频道2接收数据地址 
#define RX_ADDR_P3      0x0D  // 频道3接收数据地址 
#define RX_ADDR_P4      0x0E  // 频道4接收数据地址 
#define RX_ADDR_P5      0x0F  // 频道5接收数据地址 
#define TX_ADDR         0x10  // 发送地址寄存器 
#define RX_PW_P0        0x11  // 接收频道0接收数据长度 
#define RX_PW_P1        0x12  // 接收频道1接收数据长度 
#define RX_PW_P2        0x13  // 接收频道2接收数据长度 
#define RX_PW_P3        0x14  // 接收频道3接收数据长度 
#define RX_PW_P4        0x15  // 接收频道4接收数据长度 
#define RX_PW_P5        0x16  // 接收频道5接收数据长度 
#define FIFO_STATUS     0x17  // FIFO栈入栈出状态寄存器设置

#define RX_PLOAD_WIDTH	16
#define RX_DR	        0x40
#define RF_CH_VALUE     0x1b

u8 rx_addr[5] = {0x66, 0x88, 0x68, 0x68, 0x68};
/*************************************************

名称：nrf24l01_ce_low(void)
功能：nrf24l01+ ce管脚置低
输入参数：无
输出参数：无
返回值：无
**************************************************/
void nrf24l01_ce_low(void)
{
  GPIO_ResetBits(GPIOA, GPIO_Pin_3);
}
/*************************************************

名称：nrf24l01_ce_high(void)
功能：nrf24l01+ ce管脚置高
输入参数：无
输出参数：无
返回值：无
**************************************************/
void nrf24l01_ce_high(void)
{
  GPIO_SetBits(GPIOA, GPIO_Pin_3);
}
/*************************************************

名称：nrf24l01_rx_mode(void)
功能：nrf24l01+接收模式初始化
输入参数：无
输出参数：无
返回值：无
**************************************************/
void nrf24l01_rx_mode(void)
{
  u8 data_buf = 0;

  /* 进入待机状态 */
  nrf24l01_ce_low();

  /* 接收通道0接收地址 */
  spi_rw(&rx_addr[0], 5, NFR_WRITE_REG + RX_ADDR_P0, WRITE);

  /* 接收通道0有效数据长度 */
  data_buf = RX_PLOAD_WIDTH;
  spi_rw(&data_buf, 1, NFR_WRITE_REG + RX_PW_P0, WRITE);
  
  /* 非自动应答 */
  data_buf = 0;
  spi_rw(&data_buf, 1, NFR_WRITE_REG + EN_AA, WRITE);

  /* 接收通道0允许 */
  data_buf = 0x01;
  spi_rw(&data_buf, 1, NFR_WRITE_REG + EN_RXADDR, WRITE);
   
  /* 工作频率 */
  data_buf = RF_CH_VALUE;
  spi_rw(&data_buf, 1, NFR_WRITE_REG + RF_CH, WRITE);
  
  /* 1Mbps */
  data_buf = 0;
  spi_rw(&data_buf, 1, NFR_WRITE_REG + RF_SETUP, WRITE);
  
  /* 接收模式 16bit CRC 使能管脚触发*/
  data_buf = 0x0f;
  spi_rw(&data_buf, 1, NFR_WRITE_REG + CONFIG, WRITE);

  /* 进入接收状态 */
  nrf24l01_ce_high();			 
}
/*************************************************

名称：nrf24l01_rx_data(u8 *rx_data_buf)
功能：nrf24l01+接收数据
输入参数：
    u8 *rx_data_buf 数组指针  
输出参数：接收到的数据
返回值：无
**************************************************/
u8 nrf24l01_rx_data(u8 *rx_data_buf)
{
  u8 status = 0, data_buf = 0, flag = 0;

  spi_rw(&status, 1, NRF_STATUS, READ);

  if(status & RX_DR)
  {
    spi_rw(rx_data_buf, RX_PLOAD_WIDTH, RD_RX_PLOAD, READ);
    
	/* 清中断*/
	data_buf = RX_DR;
	spi_rw(&data_buf, 1, NFR_WRITE_REG + NRF_STATUS, WRITE);

	flag = 1;
  }

  return flag;
}
/*************************************************

名称：nrf24l01_init(void)
功能：nrf24l01+初始化
输入参数：无
输出参数：无
返回值：无
**************************************************/
void nrf24l01_init(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;

  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOC, ENABLE);
  
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3; 
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; 
  GPIO_Init(GPIOA, &GPIO_InitStructure);

  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10; 
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; 
  GPIO_Init(GPIOC, &GPIO_InitStructure);

  spi_init();
  
  nrf24l01_rx_mode(); 

}
/***************************END OF FILE**********************************************************************/
