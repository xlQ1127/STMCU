#include "NRF24L01.h"
#include "SPI_IO.h"

uint8_t TX_ADDRESS[TX_ADR_WIDTH]= {0x01,0x23,0x45};	
uint8_t RX_ADDRESS[RX_ADR_WIDTH]= {0x01,0x23,0x45};	

//*****************************************************************
//* 写寄存器
//*****************************************************************
uint8_t NRF_Write_Reg(uint8_t reg, uint8_t value)
{
		uint8_t status;
		SPI_CSN_L();					  /* 选通器件 */
		status = Spi_RW(reg);   /* 写寄存器地址 */
		Spi_RW(value);		      /* 写数据 */
		SPI_CSN_H();					  /* 禁止该器件 */
		return 	status;
}
//*****************************************************************
//* 读寄存器
//*****************************************************************
uint8_t NRF_Read_Reg(uint8_t reg)
{
		uint8_t reg_val;
		SPI_CSN_L();					  /* 选通器件 */
		Spi_RW(reg);			      /* 写寄存器地址 */
		reg_val = Spi_RW(0);	  /* 读取该寄存器返回数据 */
		SPI_CSN_H();					  /* 禁止该器件 */
		return 	reg_val;
}
//*****************************************************************
//* 写缓冲区
//*****************************************************************
uint8_t NRF_Write_Buf(uint8_t reg, uint8_t *pBuf, uint8_t uchars)
{
		uint8_t i;
		uint8_t status;
		SPI_CSN_L();				        /* 选通器件 */
		status = Spi_RW(reg);	      /* 写寄存器地址 */
		for(i=0; i<uchars; i++)
		{
			Spi_RW(pBuf[i]);		      /* 写数据 */
		}
		SPI_CSN_H();						    /* 禁止该器件 */
		return 	status;	
}
//*****************************************************************
//* 读缓冲区
//*****************************************************************
uint8_t NRF_Read_Buf(uint8_t reg, uint8_t *pBuf, uint8_t uchars)
{
		uint8_t i;
		uint8_t status;
		SPI_CSN_L();						/* 选通器件 */
		status = Spi_RW(reg);	  /* 写寄存器地址 */
		for(i=0; i<uchars; i++)
		{
			pBuf[i] = Spi_RW(0);  /* 读取返回数据 */ 	
		}
		SPI_CSN_H();						/* 禁止该器件 */
		return 	status;
}
//*****************************************************************
//* 写数据包
//*****************************************************************
void NRF_TxPacket(uint8_t * tx_buf, uint8_t len)
{	
		SPI_CE_L();		 //StandBy I模式	
		nRF24L01_tx(NRFTX,TxBuf);
		NRF_Write_Buf(NRF_WRITE_REG + RX_ADDR_P0, TX_ADDRESS, TX_ADR_WIDTH);  // 装载接收端地址
		NRF_Write_Buf(WR_TX_PLOAD, tx_buf, len); 			                        // 装载数据	
		SPI_CE_H();		                                                        //置高CE，激发数据发送
}
//void NRF_TxPacket_AP(uint8_t * tx_buf, uint8_t len)
//{	
//		SPI_CE_L();		                      //StandBy I模式	
//		NRF_Write_Buf(0xa8, tx_buf, len);   //装载数据
//		SPI_CE_H();		                      //置高CE
//}
uint8_t Nrf24l01_Check(void)
{ 
	u8 buf[5]={0XA5,0XA5,0XA5,0XA5,0XA5};
	u8 i;	 
	NRF_Write_Buf(NRF_WRITE_REG+TX_ADDR,buf,5);//写入5个字节的地址.	
	NRF_Read_Buf(TX_ADDR,buf,5); //读出写入的地址  
	for(i=0;i<5;i++)if(buf[i]!=0XA5)break;	 							   
	if(i!=5)return 1;//检测24L01错误	
	return 0;		 //检测到24L01
}
void Nrf24l01_Init(uint8_t model, uint8_t ch)
{
		SPI_CE_L();
		NRF_Write_Buf(NRF_WRITE_REG+RX_ADDR_P0,RX_ADDRESS,RX_ADR_WIDTH);	//写RX节点地址 
		NRF_Write_Buf(NRF_WRITE_REG+TX_ADDR,TX_ADDRESS,TX_ADR_WIDTH); 		//写TX节点地址  
		NRF_Write_Reg(NRF_WRITE_REG+EN_AA,0x01); 													//使能通道0的自动应答 
		NRF_Write_Reg(NRF_WRITE_REG+EN_RXADDR,0x01);											//使能通道0的接收地址 
		NRF_Write_Reg(NRF_WRITE_REG+SETUP_RETR,0x1f);											//设置自动重发间隔时间:500us;最大自动重发次数:10次 
		NRF_Write_Reg(NRF_WRITE_REG+RF_CH,ch);														//设置RF通道为CHANAL
		NRF_Write_Reg(NRF_WRITE_REG+RF_SETUP,0x0f); 											//设置TX发射参数,0db增益,2Mbps,低噪声增益开启
		//NRF_Write_Reg(NRF_WRITE_REG+RF_SETUP,0x07); 										//设置TX发射参数,0db增益,1Mbps,低噪声增益开启
	  /////////////////////////////////////////////////////////
		if(model==1)				//RX
		{
			NRF_Write_Reg(NRF_WRITE_REG+RX_PW_P0,RX_PLOAD_WIDTH);						//选择通道0的有效数据宽度 
			NRF_Write_Reg(NRF_WRITE_REG + CONFIG, 0x0f);   		              //IRQ收发完成中断开启,16位CRC,主接收
		}
		else if(model==2)		//TX
		{
			NRF_Write_Reg(NRF_WRITE_REG+RX_PW_P0,RX_PLOAD_WIDTH);					  //选择通道0的有效数据宽度 
			NRF_Write_Reg(NRF_WRITE_REG + CONFIG, 0x0e);   		              // IRQ收发完成中断开启,16位CRC,主发送
		}
		else if(model==3)		//RX2
		{
			NRF_Write_Reg(FLUSH_TX,0xff);
			NRF_Write_Reg(FLUSH_RX,0xff);
			NRF_Write_Reg(NRF_WRITE_REG + CONFIG, 0x0f);   		              // IRQ收发完成中断开启,16位CRC,主接收
			
			Spi_RW(0x50);
			Spi_RW(0x73);
			NRF_Write_Reg(NRF_WRITE_REG+0x1c,0x01);
			NRF_Write_Reg(NRF_WRITE_REG+0x1d,0x07);
		}
		else								//TX2
		{
			NRF_Write_Reg(NRF_WRITE_REG + CONFIG, 0x0e);   		              // IRQ收发完成中断开启,16位CRC,主发送
			NRF_Write_Reg(FLUSH_TX,0xff);
			NRF_Write_Reg(FLUSH_RX,0xff);
			
			Spi_RW(0x50);
			Spi_RW(0x73);
			NRF_Write_Reg(NRF_WRITE_REG+0x1c,0x01);
			NRF_Write_Reg(NRF_WRITE_REG+0x1d,0x07);
		}
		SPI_CE_H();
}

unsigned char  TxBuf[24];	  
unsigned char  RxBuf[24];

unsigned int   NRFTX[10];	  
unsigned int   NRFRX[10];

unsigned char  RX_times=0;
unsigned char  TX_times=0;

void nRF24L01_tx(unsigned int *TX,unsigned char *TxBuf)
{
	TxBuf[0]=TX[0]/256;
	TxBuf[1]=TX[0]%256;
	TxBuf[2]=TX[1]/256;
	TxBuf[3]=TX[1]%256;
	TxBuf[4]=TX[2]/256;
	TxBuf[5]=TX[2]%256;
	TxBuf[6]=TX[3]/256;
	TxBuf[7]=TX[3]%256;
	TxBuf[8]=TX[4]/256;
	TxBuf[9]=TX[4]%256;
	TxBuf[10]=TX[5]/256;
	TxBuf[11]=TX[5]%256;
	TxBuf[12]=TX[6]/256;
	TxBuf[13]=TX[6]%256;
	TxBuf[14]=TX[7]/256;
	TxBuf[15]=TX[7]%256;
	TxBuf[16]=TX[8]/256;
	TxBuf[17]=TX[8]%256;
	TxBuf[18]=TX[9]/256;
	TxBuf[19]=TX[9]%256;					
}
void nRF24L01_rx(unsigned int *RX,unsigned char *RxBuf)
{
	RX[0]=(unsigned int)(RxBuf[0]<<8)|RxBuf[1];
	RX[1]=(unsigned int)(RxBuf[2]<<8)|RxBuf[3];
	RX[2]=(unsigned int)(RxBuf[4]<<8)|RxBuf[5];
	RX[3]=(unsigned int)(RxBuf[6]<<8)|RxBuf[7];
	RX[4]=(unsigned int)(RxBuf[8]<<8)|RxBuf[9];
	RX[5]=(unsigned int)(RxBuf[10]<<8)|RxBuf[11];
	RX[6]=(unsigned int)(RxBuf[12]<<8)|RxBuf[13];
	RX[7]=(unsigned int)(RxBuf[14]<<8)|RxBuf[15];
	RX[8]=(unsigned int)(RxBuf[16]<<8)|RxBuf[17];
	RX[9]=(unsigned int)(RxBuf[18]<<8)|RxBuf[19];
}

void Nrf_Check_Event(void)
{
	u8 sta = NRF_Read_Reg(NRF_READ_REG + NRFRegSTATUS);
	if(sta & (1<<RX_DR))
	{
		u8 rx_len = NRF_Read_Reg(R_RX_PL_WID);
		if(rx_len<33)
		{
			NRF_Read_Buf(RD_RX_PLOAD,RxBuf,RX_PLOAD_WIDTH);// read receive payload from RX_FIFO buffer
			nRF24L01_rx(NRFRX,RxBuf);
	    RX_times++;
		}
		else 
		{
			NRF_Write_Reg(FLUSH_RX,0xff);//清空缓冲区
		}
	}
	if(sta & (1<<TX_DS))
	{
   		TX_times++;
	}
	if(sta & (1<<MAX_RT))
	{
		if(sta & 0x01)	//TX FIFO FULL
		{
			NRF_Write_Reg(FLUSH_TX,0xff);
		}
	}
	NRF_Write_Reg(NRF_WRITE_REG + NRFRegSTATUS, sta);
}
