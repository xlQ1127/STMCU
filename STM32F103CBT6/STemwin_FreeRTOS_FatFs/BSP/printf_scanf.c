#include "printf_scanf.h"

/*
#define hDebuguart
*/

int fputc(int ch, FILE *f)
{
    HAL_UART_Transmit(&hDebuguart, (uint8_t *)&ch, 1, 0xFF);
    return ch;
}

int fgetc(FILE *f)
{
    uint8_t ch = 0;
    HAL_UART_Receive(&hDebuguart, &ch, 1, 0xFF);
    return ch;
}
