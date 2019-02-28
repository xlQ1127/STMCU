#ifndef _MPU6050_H_   
#define _MPU6050_H_  

#include "stm32f10x.h"  


#define SCL_H         GPIO_SetBits(GPIOB , GPIO_Pin_10)   
#define SCL_L         GPIO_ResetBits(GPIOB , GPIO_Pin_10) 

#define SDA_H         GPIO_SetBits(GPIOB , GPIO_Pin_11)   
#define SDA_L         GPIO_ResetBits(GPIOB , GPIO_Pin_11) 

#define SDA_read      GPIO_ReadInputDataBit(GPIOB , GPIO_Pin_11)


void  IIC_GPIO_Config(void); 

void Single_WriteI2C(unsigned char SlaveAddress,unsigned char REG_Address,unsigned char REG_data);
unsigned char Single_ReadI2C(unsigned char SlaveAddress,unsigned char REG_Address);
unsigned short int Double_ReadI2C(unsigned char SlaveAddress,unsigned char REG_Address);



#endif 
