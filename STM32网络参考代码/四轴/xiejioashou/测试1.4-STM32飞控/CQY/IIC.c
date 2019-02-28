/*************************************************
STM32,硬件IIC支持文件

MCU：STM32F103CBT6，库函数版本：3.5.0 KEIL

陈秋阳2013-01-16

*************************************************/


#include "stm32f10x.h"
#include "IIC.h"

//----------------------------------------//


//-----------------------------------------------------------------//


void I2C_WriteByte(unsigned char id,unsigned char write_address,unsigned char byte)
{
	I2C_GenerateSTART(I2C1,ENABLE);
	//产生起始条件
	while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT));
	//等待ACK
	I2C_Send7bitAddress(I2C1,id,I2C_Direction_Transmitter);
	//向设备发送设备地址
	while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED));
	//等待ACK
	I2C_SendData(I2C1, write_address);
	//寄存器地址
	while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED));
	//等待ACK
	I2C_SendData(I2C1, byte);
	//发送数据
	while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED));
	//发送完成
	I2C_GenerateSTOP(I2C1, ENABLE);
	//产生结束信号
}


//----------------------------------------------------//
unsigned char I2C_ReadByte(unsigned char  id, unsigned char read_address)
{  
	unsigned char temp; 	

	while(I2C_GetFlagStatus(I2C1, I2C_FLAG_BUSY));
  	//等待I2C
  	I2C_GenerateSTART(I2C1, ENABLE);
  	//产生起始信号
  	while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT));
    //EV5
  	I2C_Send7bitAddress(I2C1, id, I2C_Direction_Transmitter);
	//发送地址
  	while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED));
  	//EV6
  	I2C_Cmd(I2C1, ENABLE);
 	//重新设置可以清楚EV6
  	I2C_SendData(I2C1, read_address);  
	//发送读得地址
  	while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED));
  	//EV8 
  	I2C_GenerateSTART(I2C1, ENABLE);
	//重新发送
  	while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT));
  	//EV5
  	I2C_Send7bitAddress(I2C1, id, I2C_Direction_Receiver);
  	//发送读地址
  	while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED));
  	//EV6  
    I2C_AcknowledgeConfig(I2C1, DISABLE);
    I2C_GenerateSTOP(I2C1, ENABLE);
	//关闭应答和停止条件产生
    while(!(I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_RECEIVED)));
	      
    temp = I2C_ReceiveData(I2C1);
   
  	I2C_AcknowledgeConfig(I2C1, ENABLE);
		
    return temp;
}
//-----------------------------------------//
