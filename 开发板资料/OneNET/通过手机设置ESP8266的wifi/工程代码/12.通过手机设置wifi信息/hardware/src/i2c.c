#include "i2c.h"

#include "delay.h"








//SDA		PB10
//SCL		PB11

void IIC_Init(void)
{

	GPIO_InitTypeDef gpioInitStruct;
	I2C_InitTypeDef i2cInitStruct;
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C2, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
	
	gpioInitStruct.GPIO_Mode = GPIO_Mode_AF_OD;
	gpioInitStruct.GPIO_Pin = GPIO_Pin_10 | GPIO_Pin_11;
	gpioInitStruct.GPIO_Speed = GPIO_Speed_50MHz;
	
	GPIO_Init(GPIOB, &gpioInitStruct);
	
	I2C_DeInit(I2C2);
	I2C_Cmd(I2C2, DISABLE);
	
	i2cInitStruct.I2C_Ack = I2C_Ack_Enable;
	i2cInitStruct.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
	i2cInitStruct.I2C_ClockSpeed = I2C_Speed;
	i2cInitStruct.I2C_DutyCycle = I2C_DutyCycle_2;
	i2cInitStruct.I2C_Mode = I2C_Mode_I2C;
	i2cInitStruct.I2C_OwnAddress1 = I2C_SLAVE_ADDRESS7;
	
	I2C_Cmd(I2C2, ENABLE);
	I2C_Init(I2C2, &i2cInitStruct);
	
	I2C_AcknowledgeConfig(I2C2, ENABLE);

}

void I2C_WriteByte(I2C_TypeDef *I2Cx, unsigned short i2c_slave_addr, unsigned char regAddr, unsigned char *byte)
{

	unsigned short tempADD = 0;
	unsigned char timeOut = 50;

	tempADD = i2c_slave_addr << 1;
	
	I2C_AcknowledgeConfig(I2Cx, ENABLE);
	
	/* Test on EV5 and clear it */
	I2C_GenerateSTART(I2Cx, ENABLE);           //*****start  *****//
	while((!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_MODE_SELECT)) && timeOut--)
		DelayUs(10);
	timeOut = 50;
	
	I2C_Send7bitAddress(I2Cx, tempADD, I2C_Direction_Transmitter);  /***device addr*/
	while((!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED)) && timeOut--)
		DelayUs(10);
	timeOut = 50;

	I2C_Cmd(I2Cx, ENABLE);
	I2C_SendData(I2Cx, regAddr);   /*send reg to write *************/
	while(!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_BYTE_TRANSMITTED) && timeOut--)
		DelayUs(10);
	timeOut = 50;

	/* Send the byte to be written */
	if(byte)
	{
		I2C_SendData(I2Cx, *byte);   /******send data*********/
		while(!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_BYTE_TRANSMITTED) && timeOut--)
		DelayUs(10);
		timeOut = 50;
	}
	
	/* Send STOP condition */
	I2C_GenerateSTOP(I2Cx, ENABLE);  /******stop****/

}

void I2C_ReadByte(I2C_TypeDef* I2Cx, unsigned short i2c_slave_addr, unsigned char regAddr, unsigned char *val)
{

	unsigned short addr = 0;
    unsigned char rData = 0;
	unsigned char timeOut = 50;

    addr = i2c_slave_addr << 1;
    I2C_AcknowledgeConfig(I2Cx, ENABLE);

    I2C_GenerateSTART(I2Cx, ENABLE);      /**********start**********/
    while(!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_MODE_SELECT) && timeOut--)
		DelayUs(10);
	timeOut = 50;

    I2C_Send7bitAddress(I2Cx, addr, I2C_Direction_Transmitter); /*******device addr********/
    while (!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED) && timeOut--)
		DelayUs(10);
	timeOut = 50;

    I2C_SendData(I2Cx, regAddr);    /*****reg addr**********/
    while(!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_BYTE_TRANSMITTED) && timeOut--)
		DelayUs(10);
	timeOut = 50;
	
    I2C_GenerateSTART(I2Cx, ENABLE); /***restart *************/
    while(!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_MODE_SELECT))  /* Test on EV5 and clear it */
        I2C_Send7bitAddress(I2Cx, addr, I2C_Direction_Receiver);    /*******device addr******/
	
    while(!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED) && timeOut--)
		DelayUs(10); /* Test on EV6 and clear it */
	timeOut = 50;

    while(!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_BYTE_RECEIVED) && timeOut--)
		DelayUs(10); /* EV7 */
	timeOut = 50;
	
    rData = I2C_ReceiveData(I2Cx);   /***get data****/
    I2C_AcknowledgeConfig(I2Cx, DISABLE);
    I2C_GenerateSTOP(I2Cx, ENABLE);   /********stop******/
	
    *val = rData;

}

void I2C_ReadBytes(I2C_TypeDef* I2Cx, unsigned char i2c_slave_addr, unsigned char regAddr, unsigned char *buf, unsigned char num)
{

	unsigned char addr = 0;
	unsigned char timeOut = 50;
	
    I2C_AcknowledgeConfig(I2Cx, ENABLE);

    /* Send START condition */
    I2C_GenerateSTART(I2Cx, ENABLE);/***start *******/

    addr = i2c_slave_addr << 1;

    while(!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_MODE_SELECT)&& timeOut--) /* Test on EV5 and clear it */
		DelayUs(10);
	timeOut = 50;

    I2C_Send7bitAddress(I2Cx, addr, I2C_Direction_Transmitter); /***device addr**/

    while(!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED)&& timeOut--) /* Test on EV6 and clear it */
		DelayUs(10);
	timeOut = 50;
    I2C_SendData(I2Cx, regAddr);   /* send reg addr */
    while(!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_BYTE_TRANSMITTED)&& timeOut--) /* Test on EV8 and clear it */
		DelayUs(10);
	timeOut = 50;

    I2C_GenerateSTART(I2Cx, ENABLE); /*restart  */
    while(!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_MODE_SELECT)&& timeOut--)  /* Test on EV5 and clear it */
		DelayUs(10);
	timeOut = 50;

    I2C_Send7bitAddress(I2Cx, addr, I2C_Direction_Receiver);  /* re send device addr */
    while(!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED)&& timeOut--) /* Test on EV6 and clear it */
		DelayUs(10);
	timeOut = 50;
	
    /* While there is data to be read */
    while(num)
    {
        if(num == 1)
        {
            /*the last one*/
            I2C_AcknowledgeConfig(I2Cx, DISABLE);/* Disable Acknowledgement */
            I2C_GenerateSTOP(I2Cx, ENABLE);/* Send STOP Condition */
        }

        /* Test on EV7 and clear it */
        if(I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_BYTE_RECEIVED))
        {
            /* Read a byte from the EEPROM */
            *buf = I2C_ReceiveData(I2Cx);
            buf++; /* Point to the next location where the byte read will be saved */
            num--;        /* Decrement the read bytes counter */
        }
		
		DelayUs(10); //不加这个延时的话，获取传感器数据几次后会卡死
    }
    /* Enable Acknowledgement to be ready for another reception */
    I2C_AcknowledgeConfig(I2Cx, ENABLE);

}
