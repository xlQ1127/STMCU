
#include "iic_imu1.h"

volatile u8 I2C_IMU1_FastMode;

void I2c_IMU1_Soft_delay()
{ 
	__nop();__nop();__nop();
	__nop();__nop();__nop();
	__nop();__nop();__nop();
	
	if(!I2C_IMU1_FastMode)
	{
		u8 i = 15;
		while(i--);
	}
}

void I2c_IMU1_Soft_Init()
{
  GPIO_InitTypeDef  GPIO_InitStructure; 
  RCC_AHB1PeriphClockCmd(ANO_RCC_I2C_IMU1 , ENABLE );
  GPIO_InitStructure.GPIO_Pin =  I2C_IMU1_Pin_SCL | I2C_IMU1_Pin_SDA;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(ANO_GPIO_I2C_IMU1, &GPIO_InitStructure);		
}

int I2c_IMU1_Soft_Start()
{
	SDA_IMU1_H;
	SCL_IMU1_H;
	I2c_IMU1_Soft_delay();
	if(!SDA_IMU1_read)return 0;	//SDA线为低电平则总线忙,退出
	SDA_IMU1_L;
	I2c_IMU1_Soft_delay();
	if(SDA_IMU1_read) return 0;	//SDA线为高电平则总线出错,退出
	SDA_IMU1_L;
	I2c_IMU1_Soft_delay();
	return 1;	

}

void I2c_IMU1_Soft_Stop()
{
	SCL_IMU1_L;
	I2c_IMU1_Soft_delay();
	SDA_IMU1_L;
	I2c_IMU1_Soft_delay();
	SCL_IMU1_H;
	I2c_IMU1_Soft_delay();
	SDA_IMU1_H;
	I2c_IMU1_Soft_delay();
}

void I2c_IMU1_Soft_Ask()
{
	SCL_IMU1_L;
	I2c_IMU1_Soft_delay();
	SDA_IMU1_L;
	I2c_IMU1_Soft_delay();
	SCL_IMU1_H;
	I2c_IMU1_Soft_delay();
	SCL_IMU1_L;
	I2c_IMU1_Soft_delay();
}

void I2c_IMU1_Soft_NoAsk()
{
	SCL_IMU1_L;
	I2c_IMU1_Soft_delay();
	SDA_IMU1_H;
	I2c_IMU1_Soft_delay();
	SCL_IMU1_H;
	I2c_IMU1_Soft_delay();
	SCL_IMU1_L;
	I2c_IMU1_Soft_delay();
}

int I2c_IMU1_Soft_WaitAsk(void) 	 //返回为:=1无ASK,=0有ASK
{
  u8 ErrTime = 0;
	SCL_IMU1_L;
	I2c_IMU1_Soft_delay();
	SDA_IMU1_H;			
	I2c_IMU1_Soft_delay();
	SCL_IMU1_H;
	I2c_IMU1_Soft_delay();
	while(SDA_IMU1_read)
	{
		ErrTime++;
		if(ErrTime>50)
		{
			I2c_IMU1_Soft_Stop();
			return 1;
		}
	}
	SCL_IMU1_L;
	I2c_IMU1_Soft_delay();
	return 0;
}

void I2c_IMU1_Soft_SendByte(u8 SendByte) //数据从高位到低位//
{
    u8 i=8;
    while(i--)
    {
        SCL_IMU1_L;
        I2c_IMU1_Soft_delay();
      if(SendByte&0x80)
        SDA_IMU1_H;  
      else 
        SDA_IMU1_L;   
        SendByte<<=1;
        I2c_IMU1_Soft_delay();
				SCL_IMU1_H;
				I2c_IMU1_Soft_delay();
    }
    SCL_IMU1_L;
}  

//读1个字节，ack=1时，发送ACK，ack=0，发送NACK
u8 I2c_IMU1_Soft_ReadByte(u8 ask)  //数据从高位到低位//
{ 
    u8 i=8;
    u8 ReceiveByte=0;

    SDA_IMU1_H;				
    while(i--)
    {
      ReceiveByte<<=1;      
      SCL_IMU1_L;
      I2c_IMU1_Soft_delay();
			SCL_IMU1_H;
      I2c_IMU1_Soft_delay();	
      if(SDA_IMU1_read)
      {
        ReceiveByte|=0x01;
      }
    }
    SCL_IMU1_L;

	if (ask)
		I2c_IMU1_Soft_Ask();
	else
		I2c_IMU1_Soft_NoAsk();  
    return ReceiveByte;
} 


// IIC写一个字节数据
u8 IIC_IMU1_Write_1Byte(u8 SlaveAddress,u8 REG_Address,u8 REG_data)
{
	I2c_IMU1_Soft_Start();
	I2c_IMU1_Soft_SendByte(SlaveAddress<<1);   
	if(I2c_IMU1_Soft_WaitAsk())
	{
		I2c_IMU1_Soft_Stop();
		return 1;
	}
	I2c_IMU1_Soft_SendByte(REG_Address);       
	I2c_IMU1_Soft_WaitAsk();	
	I2c_IMU1_Soft_SendByte(REG_data);
	I2c_IMU1_Soft_WaitAsk();   
	I2c_IMU1_Soft_Stop(); 
	return 0;
}

// IIC读1字节数据
u8 IIC_IMU1_Read_1Byte(u8 SlaveAddress,u8 REG_Address,u8 *REG_data)
{      		
	I2c_IMU1_Soft_Start();
	I2c_IMU1_Soft_SendByte(SlaveAddress<<1); 
	if(I2c_IMU1_Soft_WaitAsk())
	{
		I2c_IMU1_Soft_Stop();
		return 1;
	}
	I2c_IMU1_Soft_SendByte(REG_Address);     
	I2c_IMU1_Soft_WaitAsk();
	I2c_IMU1_Soft_Start();
	I2c_IMU1_Soft_SendByte(SlaveAddress<<1 | 0x01);
	I2c_IMU1_Soft_WaitAsk();
	*REG_data= I2c_IMU1_Soft_ReadByte(0);
	I2c_IMU1_Soft_Stop();
	return 0;
}	

// IIC写n字节数据
u8 IIC_IMU1_Write_nByte(u8 SlaveAddress, u8 REG_Address, u8 len, u8 *buf)
{	
	I2c_IMU1_Soft_Start();
	I2c_IMU1_Soft_SendByte(SlaveAddress<<1); 
	if(I2c_IMU1_Soft_WaitAsk())
	{
		I2c_IMU1_Soft_Stop();
		return 1;
	}
	I2c_IMU1_Soft_SendByte(REG_Address); 
	I2c_IMU1_Soft_WaitAsk();
	while(len--) 
	{
		I2c_IMU1_Soft_SendByte(*buf++); 
		I2c_IMU1_Soft_WaitAsk();
	}
	I2c_IMU1_Soft_Stop();
	return 0;
}

// IIC读n字节数据
u8 IIC_IMU1_Read_nByte(u8 SlaveAddress, u8 REG_Address, u8 len, u8 *buf)
{	
	I2c_IMU1_Soft_Start();
	I2c_IMU1_Soft_SendByte(SlaveAddress<<1); 
	if(I2c_IMU1_Soft_WaitAsk())
	{
		I2c_IMU1_Soft_Stop();
		return 1;
	}
	I2c_IMU1_Soft_SendByte(REG_Address); 
	I2c_IMU1_Soft_WaitAsk();
	
	I2c_IMU1_Soft_Start();
	I2c_IMU1_Soft_SendByte(SlaveAddress<<1 | 0x01); 
	I2c_IMU1_Soft_WaitAsk();
	while(len) 
	{
		if(len == 1)
		{
			*buf = I2c_IMU1_Soft_ReadByte(0);
		}
		else
		{
			*buf = I2c_IMU1_Soft_ReadByte(1);
		}
		buf++;
		len--;
	}
	I2c_IMU1_Soft_Stop();
	return 0;
}


/******************* (C) COPYRIGHT 2014 ANO TECH *****END OF FILE************/

