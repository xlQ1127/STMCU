#include "myiic.h"

void I2C_delay(void)
{
	delay_us(1);
}

//初始化IIC
void I2C_Init_IO(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC,ENABLE);//开启GPIOC时钟
	 
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11|GPIO_Pin_12; 
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; 
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_OD;//GPIO_Mode_Out_OD;//开漏输出 /*GPIO_Mode_Out_PP;*/
	GPIO_Init(GPIOC, &GPIO_InitStructure);

	GPIO_SetBits(GPIOC, GPIO_Pin_11 | GPIO_Pin_12);//PC 11,12 输出高

}

//产生IIC起始信号
void I2C_Start(void)
{
	SDA_H;
	I2C_delay();
	SCL_H;
	I2C_delay();
	SDA_L;
	I2C_delay();
	SCL_L;
}

//产生IIC停止信号
void I2C_Stop(void)
{
	
	SCL_L;
	I2C_delay();
	SDA_L;
	I2C_delay();
	SCL_H;
    I2C_delay();
	SDA_H;
	I2C_delay();
} 

//等待应答信号到来
//返回为:=1有ACK,=0无ACK
bool I2C_WaitAck(void) 	 
{
	u8 errtime = 0;
	SCL_L;
	I2C_delay();
	SDA_H;//拉高数据总线准备读数据			
	I2C_delay();
	SCL_H;
	I2C_delay();
	while(SDA_read)
	{
		errtime++;
		if(errtime>250)
		{
			I2C_Stop();
			return FALSE;	
		}
	}
	SCL_L;//时钟拉低
	I2C_delay();
	return TRUE;
}

//产生ACK应答
void I2C_Ack(void)
{	
	SCL_L;
	I2C_delay();
	SDA_L;
	I2C_delay();
	SCL_H;
	I2C_delay();
	SCL_L;
	I2C_delay();
}

//不产生ACK应答
void I2C_NoAck(void)
{	
	SCL_L;
	I2C_delay();
	SDA_H;
	I2C_delay();
	SCL_H;
	I2C_delay();
	SCL_L;
	I2C_delay();
}

//IIC发送一个字节
//数据从高位到低位//
void I2C_SendByte(u8 SendByte)
{
    u8 i;
    for(i=0;i<8;i++)
    {
        SCL_L;
        I2C_delay();
      	if(SendByte&(0x80>>i)) SDA_H;  
      	else SDA_L;   
        I2C_delay();
		SCL_H;
        I2C_delay();
    }
    SCL_L;
	I2C_delay();
}

//读1个字节 
//数据从高位到低位// 
u8 I2C_ReadByte(void)  
{ 
    u8 i=0;
    u8 ReceiveByte=0;
				
   	for(i=0;i<8;i++)
    {      
		SCL_L;
		SDA_H;//拉高数据线 准备读取数据
		I2C_delay();
		SCL_H;
		ReceiveByte	<<= 1;
        if(SDA_read)
		{
			ReceiveByte++; 
		}
		else
		{

		}

		I2C_delay();
    }
    SCL_L;
	I2C_delay();
    return ReceiveByte;
}

//******单字节写入**********//
bool Single_Write(u8 REG_Address,u8 REG_data,u8 SlaveAddress)
{
	I2C_Start();
	I2C_SendByte(SlaveAddress);//发送设备地址+写信号
	while(!I2C_WaitAck());
	I2C_SendByte(REG_Address);//设置低起始地址
	while(!I2C_WaitAck());
	I2C_SendByte(REG_data);
	while(!I2C_WaitAck());
	I2C_Stop();
	delay_ms(5);
	return TRUE;	
}

//********单字节读取*********//
u8 Single_Read(u8 REG_Address,u8 SlaveAddress)
{
	u8 data=0;
	I2C_Start();
	I2C_SendByte(SlaveAddress);//发送设备地址+写信号
	while(!I2C_WaitAck());
	I2C_SendByte(REG_Address);   //设置低起始地址
	while(!I2C_WaitAck()); 
	I2C_Start();
	I2C_SendByte(SlaveAddress+1);//发送设备地址+读信号
	while(!I2C_WaitAck());

	data = I2C_ReadByte();
	I2C_NoAck();
    I2C_Stop();

	return data;	
}

//多字节写入//
void Multiple_write(u8 star_addr,u8 num,u8 SlaveAddress,u8* send_buf)
{
	u8 i;
	I2C_Start();
	I2C_SendByte(SlaveAddress);//发送设备地址
	while(!I2C_WaitAck());
	I2C_SendByte(star_addr);//发送起始地址
	while(!I2C_WaitAck());
	for(i=0;i<num;i++)
	{
		I2C_SendByte(send_buf[i]);
		while(!I2C_WaitAck());
	}
	I2C_Stop();
}

//读取多个字节的数据
void Multiple_read(u8 star_addr,u8 num,u8 SlaveAddress,u8* recv_buf)
{
	u8 i;
	
	I2C_Start();
	I2C_SendByte(SlaveAddress);//发送设备地址
	while(!I2C_WaitAck());
	I2C_SendByte(star_addr);//发送起始地址
	while(!I2C_WaitAck());
	
	I2C_Start();
	I2C_SendByte(SlaveAddress+1);//发送设备地址+读信号
	while(!I2C_WaitAck());
	
	for(i=0;i<num;i++)
	{
		recv_buf[i] = I2C_ReadByte();
		if(i == (num-1)) I2C_NoAck();//最后一个数据需要回NOACK
		else I2C_Ack();
	}
	I2C_Stop();//停止信号
}




///////////////////////////////////////////MPU6050接口函数////////////////////////////////////////////////////////
void i2cRead(u8 SlaveAddress,u8 star_addr,u8 num,u8* recv_buf)
{
	Multiple_read(star_addr,num,SlaveAddress,recv_buf);
}

void i2cWrite(u8 SlaveAddress,u8 REG_Address,u8 REG_data)
{
	Single_Write(REG_Address,REG_data,SlaveAddress);
}








