#ifndef __MYIIC_H__
#define __MYIIC_H__

#include "stm32f10x.h"
#include "SysTick.h"


//IO定义
#define SCL_H         GPIOC->BSRR = GPIO_Pin_12
#define SCL_L         GPIOC->BRR  = GPIO_Pin_12 
    
#define SDA_H         GPIOC->BSRR = GPIO_Pin_11
#define SDA_L         GPIOC->BRR  = GPIO_Pin_11

#define SCL_read      GPIOC->IDR  & GPIO_Pin_12 
#define SDA_read      GPIOC->IDR  & GPIO_Pin_11


//IIC所有操作函数
void I2C_Init_IO(void);               //初始化IIC的IO口				 
void I2C_Start(void);				//发送IIC开始信号
void I2C_Stop(void);	  			//发送IIC停止信号

bool I2C_WaitAck(void); 				//IIC等待ACK信号
void I2C_Ack(void);					//IIC发送ACK信号
void I2C_NoAck(void);				//IIC不发送ACK信号

void I2C_SendByte(u8 SendByte);			//IIC发送一个字节
u8 I2C_ReadByte(void);            //IIC读取一个字节

//******单字节写入**********//
bool Single_Write(u8 REG_Address,u8 REG_data,u8 SlaveAddress);
//********单字节读取*********//
u8 Single_Read(u8 REG_Address,u8 SlaveAddress);
//多字节写入//
void Multiple_write(u8 star_addr,u8 num,u8 SlaveAddress,u8* send_buf);
//读取多个字节的数据//
void Multiple_read(u8 star_addr,u8 num,u8 SlaveAddress,u8* recv_buf);

/////////////////////////////mpu6050接口函数///////////////////////////////////////////////////////////
void i2cRead(u8 SlaveAddress,u8 star_addr,u8 num,u8* recv_buf);
void i2cWrite(u8 SlaveAddress,u8 REG_Address,u8 REG_data);

#endif


