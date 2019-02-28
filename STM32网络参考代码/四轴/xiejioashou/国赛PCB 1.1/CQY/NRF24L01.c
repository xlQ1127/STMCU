/*************************************************
2.4G无线通信模块24L01驱动

MCU：STM32F103CBT6，硬件SPI，库函数版本：3.5.0 KEIL

陈秋阳2013-01-16

*************************************************/

#include "NRF24L01.h"
#include "stm32f10x.h"
#include "delay.h" 
#include "spi.h"
#include "MPU6050.H"
#include "var_global.h"


#define CE1 GPIO_SetBits(GPIOB, GPIO_Pin_2)
#define CE0 GPIO_ResetBits(GPIOB, GPIO_Pin_2)

#define CSN1 GPIO_SetBits(GPIOA, GPIO_Pin_8)
#define CSN0 GPIO_ResetBits(GPIOA, GPIO_Pin_8)

u8 TxDate[32];

u8 TxAddr[]={0x34,0x43,0x10,0x10,0x01};//发送地址

union//无线发送
{
	RF_data   f; 	
  u8        data[32];
} RF;

RC RCun;


void RF_send()
{  
	RF_SendOnec(RF.data);
  while(CheckACK());	//检测是否发送完毕
} 


void Get_RFdata()
{ 
 
//NRFSetRXMode();	

if(NRFRevDate(RCun.RxData)){dir_time=0;}

}

void Send_RFdata()
{
	RF.f.Acc_data[0]=ACC_AVG.X;
	RF.f.Acc_data[1]=ACC_AVG.Y;
 	RF.f.Acc_data[2]=ACC_AVG.Z;
  RF.f.F_GRY_data[0]=GRY_F.X;
  RF.f.F_GRY_data[1]=GRY_F.Y;
	RF.f.F_GRY_data[2]=GRY_F.Z;
	RF.f.F_Cal_data[0]=Q_ANGLE.Pitch;
  RF.f.F_Cal_data[1]=Q_ANGLE.Rool; 
	RF_send(); 
}

void RF_SendOnec(u8 *TxDate) //发送	
{
  CE0; 
	NRFWriteTxDate(W_TX_PAYLOAD,TxDate,TX_DATA_WITDH);//写入数据 
	CE1;
	//delay_us(5);//保持10us秒以上
}

/*****************状态标志*****************************************/
u8 sta;   //状态标志

u8 RX_DR;
u8 TX_DS;
u8 MAX_RT;

/*****************SPI时序函数******************************************/

/**********************NRF24L01初始化函数*******************************/
void NRF24L01Int()
{
	
	CE0; //待机模式1   
	CSN1;  
  GPIO_ResetBits(GPIOA,GPIO_Pin_5);//	SCLK0;
	//IRQ1	 
}
/*****************SPI读寄存器一字节函数*********************************/
u8 NRFReadReg(u8 RegAddr)
{
   u8 BackDate;
   CSN0;//启动时序
   NRFSPI(RegAddr);//写寄存器地址
   BackDate=NRFSPI(0xFF);//写入读寄存器指令  
   CSN1;
   return(BackDate); //返回状态
}
/*****************SPI写寄存器一字节函数*********************************/
u8 NRFWriteReg(u8 RegAddr,u8 date)
{
   u8 BackDate;
   CSN0;//启动时序
   BackDate=NRFSPI(RegAddr);//写入地址
   NRFSPI(date);//写入值
   CSN1;
   return(BackDate);
}
/*****************SPI读取RXFIFO寄存器的值********************************/
u8 NRFReadRxDate(u8 RegAddr,u8 *RxDate,u8 DateLen)
{  //寄存器地址//读取数据存放变量//读取数据长度//用于接收
    u8 BackDate,i;
	CSN0;//启动时序
	BackDate=NRFSPI(RegAddr);//写入要读取的寄存器地址
	for(i=0;i<DateLen;i++) //读取数据
	  {
	     RxDate[i]=NRFSPI(0);
	  } 
   CSN1;
   return(BackDate); 
}
/*****************SPI写入TXFIFO寄存器的值**********************************/
u8 NRFWriteTxDate(u8 RegAddr,u8 *TxDate,u8 DateLen)
{ //寄存器地址//写入数据存放变量//读取数据长度//用于发送
  u8 BackDate,i;
   CSN0;
   BackDate=NRFSPI(RegAddr);//写入要写入寄存器的地址
   for(i=0;i<DateLen;i++)//写入数据
     {
	    NRFSPI(*TxDate++);
	 }   
   CSN1;
   return(BackDate);
}
/*****************NRF设置为发送模式并发送数据******************************/
void NRFSetTxMode(u8 *TxDate)
{//发送模式
	NRF24L01Int();	
	
  CE0; 
  NRFWriteTxDate(W_REGISTER+TX_ADDR,TxAddr,TX_ADDR_WITDH);//写寄存器指令+接收地址使能指令+接收地址+地址宽度
	NRFWriteTxDate(W_REGISTER+RX_ADDR_P0,TxAddr,TX_ADDR_WITDH);//为了应答接收设备，接收通道0地址和发送地址相同
	NRFWriteTxDate(W_TX_PAYLOAD,TxDate,TX_DATA_WITDH);//写入数据 
	/******下面有关寄存器配置**************/
  	NRFWriteReg(W_REGISTER+EN_AA,0x01);       // 使能接收通道0自动应答
  	NRFWriteReg(W_REGISTER+EN_RXADDR,0x01);   // 使能接收通道0
  	NRFWriteReg(W_REGISTER+SETUP_RETR,0x0a);  //自动重发延时等待250us+86us，自动重发1次
  	NRFWriteReg(W_REGISTER+RF_CH,0x40);         // 选择射频通道0x40
  	NRFWriteReg(W_REGISTER+RF_SETUP,0x07);    // 数据传输率1Mbps，发射功率0dBm，低噪声放大器增益
	NRFWriteReg(W_REGISTER+CONFIG_24L01,0x0e);      // CRC使能，16位CRC校验，上电  
	CE1;
	delay_us(30);//保持10us秒以上
}
/*****************NRF设置为接收模式并接收数据******************************/
//主要接收模式
void NRFSetRXMode()
{
	  CE0;  
  	NRFWriteTxDate(W_REGISTER+RX_ADDR_P0,TxAddr,TX_ADDR_WITDH);  // 接收设备接收通道0使用和发送设备相同的发送地址
  	NRFWriteReg(W_REGISTER+EN_AA,0x01);               // 使能接收通道0自动应答
  	NRFWriteReg(W_REGISTER+EN_RXADDR,0x01);           // 使能接收通道0
  	NRFWriteReg(W_REGISTER+RF_CH,0x40);                 // 选择射频通道0x40
  	NRFWriteReg(W_REGISTER+RX_PW_P0,8);  // 接收通道0选择8byte数据宽度
  	NRFWriteReg(W_REGISTER+RF_SETUP,0x07);            // 数据传输率1Mbps，发射功率0dBm，低噪声放大器增益*/
  	NRFWriteReg(W_REGISTER+CONFIG_24L01,0x0f);              // CRC使能，16位CRC校验，上电，接收模式
  	CE1;
    delay_us(20);
}
/****************************检测应答信号******************************/
u8 CheckACK()
{  //用于发射
	sta=NRFReadReg(R_REGISTER+STATUS);                    // 返回状态寄存器
	
RX_DR=(sta&0x40);
TX_DS=(sta&0x20);
MAX_RT=(sta&0x10);
   if(TX_DS||MAX_RT) //发送完毕中断
	{
	   NRFWriteReg(W_REGISTER+STATUS,0xff);  // 清除TX_DS或MAX_RT中断标志
	   CSN0;
	   NRFSPI(FLUSH_TX);//用于清空FIFO ！！关键！！不然会出现意想不到的后果！！！大家记住！！  
     CSN1; 
	   return(0);
	}
	else
	   return(1);
}
/******************判断是否接收收到数据，接到就从RX取出*********************/
//用于接收模式
u8 NRFRevDate(u8 *RevDate)
{
   	 u8 RevFlags=0;
	   sta=NRFReadReg(R_REGISTER+STATUS);//发送数据后读取状态寄存器
    
     RX_DR=(sta&0x40);
     TX_DS=(sta&0x20);
     MAX_RT=(sta&0x10);
         
     if(RX_DR)				// 判断是否接收到数据
	 {
	  CE0; 		   //SPI使能
		NRFReadRxDate(R_RX_PAYLOAD,RevDate,RX_DATA_WITDH);        // 从RXFIFO读取数据
    RevFlags=1;	   //读取数据完成标志
	  }
	
    NRFWriteReg(W_REGISTER+STATUS,0xff); //接收到数据后RX_DR,TX_DS,MAX_PT都置高为1，通过写1来清楚中断标
	
    CSN0;//CSN=0;
	  NRFSPI(FLUSH_RX);//用于清空FIFO
	  
		CSN1;
    CE1;		
    return(RevFlags);
}

// void RF_send(u8 adch,u16 i)
// {
// u8 send;
// u8 id;
// u8 chid;
// if (adch==0)chid=0x01;
// if (adch==1)chid=0x05;
// if (adch==2)chid=0x09;
// if (adch==3)chid=0x0d;

// id= chid<<4;
// send=i & 0x0f;
// send|=id; 
// TxDate[0]=send; 
//              
// id= (chid+1)<<4;
// send=(i>>4) & 0x0f;
// send|=id; 
// TxDate[1]=send; 
//  
// id= (chid+2)<<4;
// send=(i>>8) & 0x0f;
// send|=id; 
// TxDate[2]=send; 
//    
// RF_SendOnec();
// while(CheckACK());	//检测是否发送完毕
// delay_us(200);

// } 

