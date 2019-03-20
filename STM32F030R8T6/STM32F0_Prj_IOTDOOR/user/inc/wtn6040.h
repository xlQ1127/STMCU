#ifndef WTN6040_H
#define WTN6040_H


#include "common.h"



#define CLKLOW   GPIOB->BSRR = GPIO_BSRR_BR_4
#define CLKHIGH  GPIOB->BSRR = GPIO_BSRR_BS_4
#define DATLOW   GPIOB->BSRR = GPIO_BSRR_BR_3
#define DATHIGH  GPIOB->BSRR = GPIO_BSRR_BS_3



void SendMessage(uint8 num);
#endif