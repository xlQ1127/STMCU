#include "printf_scanf.h"

extern UART_HandleTypeDef hDebug;

int fputc(int ch, FILE *f)  
{  
HAL_UART_Transmit(&hDebug, (uint8_t *)&ch, 1,0xFF);  return ch;  
} 

int fgetc(FILE * f)  
{  
uint8_t ch = 0;  
HAL_UART_Receive(&hDebug,&ch, 1,0xFF);  return ch;  
}
