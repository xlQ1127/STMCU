#ifndef _I2C_H_
#define _I2C_H_


#include "stm32f10x.h"




#define I2C_Speed				100000
#define I2C_SLAVE_ADDRESS7		0



void IIC_Init(void);

void I2C_WriteByte(I2C_TypeDef *I2Cx, unsigned short i2c_slave_addr, unsigned char regAddr, unsigned char *byte);

void I2C_ReadByte(I2C_TypeDef* I2Cx, unsigned short i2c_slave_addr, unsigned char regAddr, unsigned char *val);

void I2C_ReadBytes(I2C_TypeDef* I2Cx, unsigned char i2c_slave_addr, unsigned char regAddr, unsigned char *buf, unsigned char num);


#endif
