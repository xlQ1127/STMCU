#ifndef _SPI_IO_H_   
#define _SPI_IO_H_ 

#include "stm32f10x.h"

 
 
#define SPI_CE_H()   GPIO_SetBits(GPIOA, GPIO_Pin_3) 
#define SPI_CE_L()   GPIO_ResetBits(GPIOA, GPIO_Pin_3)

#define SPI_CSN_H()  GPIO_SetBits(GPIOA, GPIO_Pin_4)
#define SPI_CSN_L()  GPIO_ResetBits(GPIOA, GPIO_Pin_4)

#define SPI_SCK(a)  if (a) GPIO_SetBits(GPIOA,GPIO_Pin_5);  else  GPIO_ResetBits(GPIOA,GPIO_Pin_5);
#define SPI_MOSI(a) if (a) GPIO_SetBits(GPIOA,GPIO_Pin_7);  else  GPIO_ResetBits(GPIOA,GPIO_Pin_7);

#define SPI_MISO_IN  GPIO_ReadInputDataBit(GPIOA , GPIO_Pin_6)



void Spi1_Init(void);
unsigned char Spi_RW(unsigned char uchar);





#endif 
