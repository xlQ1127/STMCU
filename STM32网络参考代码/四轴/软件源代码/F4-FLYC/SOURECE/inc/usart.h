#ifndef __USART_H
#define __USART_H

#include "stm32f4xx.h"
#include <stdio.h>

void usart_x_init(u32 BaudRate);
void USART3_SendData(unsigned char ch);
void SendData(signed int A, signed int B, signed int C, signed int D);

#endif

