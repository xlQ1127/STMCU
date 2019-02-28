#ifndef _USART_H_   
#define _USART_H_  

#include "stm32f10x.h"


void USART1_Config(void);
void usart_send(char ch);
void Printt(char *p);
void UART_Send_int(int16_t Data);
void UART_Send_float(float Data);
void UART1_ReportIMU(int ax,int ay,int az,int gx,int gy,int gz,int hx,int hy,int hz,int yaw,int pitch,int roll);
void UART1_ReportCON(int throt,int yaw,int roll,int pitch,int aux1,int aux2,int aux3,int aux4,int aux5,int pwm1,int pwm2,int pwm3,int pwm4,int votage);

#endif 
