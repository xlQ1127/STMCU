#include "i2c.h"

#include "delay.h"


//SDA		PB11
//SCL		PB10
#define SDA_H	GPIO_SetBits(GPIOB, GPIO_Pin_11)
#define SDA_L	GPIO_ResetBits(GPIOB, GPIO_Pin_11)
#define SDA_R	GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_11)

#define SCL_H	GPIO_SetBits(GPIOB, GPIO_Pin_10)
#define SCL_L	GPIO_ResetBits(GPIOB, GPIO_Pin_10)


void IIC_Init(void)
{

	GPIO_InitTypeDef gpioInitStruct;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
	
	gpioInitStruct.GPIO_Mode = GPIO_Mode_Out_OD; //开漏，这样不用去切换输出输入方向
	gpioInitStruct.GPIO_Pin = GPIO_Pin_10 | GPIO_Pin_11;
	gpioInitStruct.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &gpioInitStruct);
	
	SDA_H;
	SCL_H;

}

void IIC_Start(void)
{
	
	SDA_H;
	SCL_H;
	DelayUs(5);
	
	SDA_L;
	DelayUs(5);
	SCL_L; //钳住SCL线，以便发送数据

}

void IIC_Stop(void)
{

	SDA_L;
	SCL_L;
	DelayUs(5);
	
	SCL_H;
	SDA_H;
	DelayUs(5);

}

//等待应答信号
_Bool IIC_WaitAck(unsigned int timeOut)
{
	
	unsigned int ucErrTime = 0;
	
	SDA_H;DelayUs(1);
	SCL_H;DelayUs(1);
	
	while(SDA_R)
	{
		if(++ucErrTime > timeOut)
		{
//			UsartPrintf(USART1, "WaitAck TimeOut\r\n");

			IIC_Stop();
			
			return 1;
		}
		
		DelayUs(1);
	}
	
	SCL_L; //时钟输出0
	
	return 0;
	
}

//产生ACK应答
void IIC_Ack(void)
{
	
	SCL_L;
	SDA_L;
	DelayUs(5);
	SCL_H;
	DelayUs(5);
	SCL_L;
	
}

//不产生ACK应答		    
void IIC_NAck(void)
{
	
	SCL_L;
	SDA_H;
	DelayUs(5);
	SCL_H;
	DelayUs(5);
	SCL_L;
	
}

void IIC_SendByte(unsigned char byte)
{

	unsigned char count = 0;
	
    SCL_L;//拉低时钟开始数据传输
	
    for(; count < 8; count++)
    {
		if(byte & 0x80)
			SDA_H;
		else
			SDA_L;
		
		byte <<= 1;
		
		DelayUs(5);
		SCL_H;
		DelayUs(5);
		SCL_L;
    }

}

//读1个字节，ack=1时，发送ACK，ack=0，发送nACK
unsigned char IIC_RecvByte(void)
{
	
	unsigned char count = 0, receive = 0;
	
	SDA_H;
	
    for(; count < 8; count++ )
	{
		SCL_L;
		DelayUs(5);
		SCL_H;
		
        receive <<= 1;
		
        if(SDA_R)
			receive++;
		
		DelayUs(5);
    }
	
    return receive;
	
}

_Bool I2C_WriteByte(unsigned char slaveAddr, unsigned char regAddr, unsigned char *byte)
{

	unsigned char addr = 0;

	addr = slaveAddr << 1;
	
	IIC_Start(); //起始信号
	
	IIC_SendByte(addr); //发送设备地址
	if(IIC_WaitAck(5000)) //等待应答
		return 1;
	
	IIC_SendByte(regAddr); //发送寄存器地址
	if(IIC_WaitAck(5000)) //等待应答
		return 1;
	
	if(byte)
	{
		IIC_SendByte(*byte); //发送数据
		if(IIC_WaitAck(5000)) //等待应答
			return 1;
	}
	
	IIC_Stop();
	
	return 0;

}

_Bool I2C_ReadByte(unsigned char slaveAddr, unsigned char regAddr, unsigned char *val)
{

	unsigned char addr = 0;

    addr = slaveAddr << 1;
	
	IIC_Start(); //起始信号
	
	IIC_SendByte(addr); //发送设备地址
	if(IIC_WaitAck(5000)) //等待应答
		return 1;
	
	IIC_SendByte(regAddr); //发送寄存器地址
	if(IIC_WaitAck(5000)) //等待应答
		return 1;
	
	IIC_Start(); //起始信号
	
	IIC_SendByte(addr + 1); //发送设备地址
	if(IIC_WaitAck(5000)) //等待应答
		return 1;
	
	*val = IIC_RecvByte();
	IIC_NAck();
	
	IIC_Stop();
	
	return 0;

}

_Bool I2C_WriteBytes(unsigned char slaveAddr, unsigned char regAddr, unsigned char *buf, unsigned char num)
{

	unsigned char addr = 0;

	addr = slaveAddr << 1;
	
	IIC_Start(); //起始信号
	
	IIC_SendByte(addr); //发送设备地址
	if(IIC_WaitAck(5000)) //等待应答
		return 1;
	
	IIC_SendByte(regAddr); //发送寄存器地址
	if(IIC_WaitAck(5000)) //等待应答
		return 1;
	
	while(num--)
	{
		IIC_SendByte(*buf); //发送数据
		if(IIC_WaitAck(5000)) //等待应答
			return 1;
		
		buf++;
		
		DelayUs(10);
	}
	
	IIC_Stop();
	
	return 0;

}

_Bool I2C_ReadBytes(unsigned char slaveAddr, unsigned char regAddr, unsigned char *buf, unsigned char num)
{

	unsigned short addr = 0;

    addr = slaveAddr << 1;
	
	IIC_Start(); //起始信号
	
	IIC_SendByte(addr); //发送设备地址
	if(IIC_WaitAck(5000)) //等待应答
		return 1;
	
	IIC_SendByte(regAddr); //发送寄存器地址
	if(IIC_WaitAck(5000)) //等待应答
		return 1;
	
	IIC_Start(); //起始信号
	
	IIC_SendByte(addr + 1); //发送设备地址
	if(IIC_WaitAck(5000)) //等待应答
		return 1;
	
	while(num--)
	{
		*buf = IIC_RecvByte();
		buf++;
		
		if(num == 0)
        {
           IIC_NAck(); //最后一个数据需要回NOACK
        }
        else
        {
          IIC_Ack(); //回应ACK
		}
	}
	
	IIC_Stop();
	
	return 0;

}
