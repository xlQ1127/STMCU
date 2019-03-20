#ifndef _GPIO_INIT_H
#define _GPIO_INIT_H

#include "common.h"
#include "stm32f0xx.h"


///////指示灯
#define LED0_OFF        GPIOA->BSRR = GPIO_BSRR_BS_10
#define LED0_ON		       GPIOA->BSRR = GPIO_BSRR_BR_10
#define LED1_OFF	    GPIOA->BSRR = GPIO_BSRR_BS_9
#define LED1_ON		    GPIOA->BSRR = GPIO_BSRR_BR_9

//////红外灯
/////发射
#define IRLED0_ON       GPIOA->BSRR = GPIO_BSRR_BR_6
#define IRLED1_ON       GPIOC->BSRR = GPIO_BSRR_BR_6
#define IRLED0_OFF      GPIOA->BSRR = GPIO_BSRR_BS_6
#define IRLED1_OFF      GPIOC->BSRR = GPIO_BSRR_BS_6
////接收
#define IRLED0_IN       ((GPIOC->IDR >> 10) & 0x01)
#define IRLED1_IN       ((GPIOA->IDR >> 7) & 0x01)

#define KEY_RST_IN      ((GPIOC->IDR >> 15) & 0x01)
#define KEY_OPEN_IN     ((GPIOC->IDR >> 14) & 0x01)
#define KEY_CLOSE_IN    ((GPIOC->IDR >> 13) & 0x01)

////VDD使能
#define VDD_ON		 GPIOC->BSRR = GPIO_BSRR_BS_8
#define VDD_OFF		GPIOC->BSRR = GPIO_BSRR_BR_8


extern __IO uint32_t AsynchPrediv, SynchPrediv;


void BoardGPIO_init(void);
void uart1_init(void);
void uart2_init(void);
void adc_init(void);
void TIM1_init(uint16 ms);
void rtc_init(void);
void SleepMode(void);
#endif