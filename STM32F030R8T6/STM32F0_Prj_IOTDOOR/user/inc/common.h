#ifndef COMMON_H
#define COMMON_H

#define ARM_MATH_CM0

#include "stm32f0xx.h"
#include "stm32f0xx_adc.h"
#include "stm32f0xx_dma.h"
#include "stm32f0xx_rtc.h"
#include "stm32f0xx_pwr.h"
#include "stm32f0xx_exti.h"
#include "arm_math.h"                   

typedef unsigned char uint8, u8, byte;
typedef signed char int8, sbyte;
typedef unsigned short uint16, u16, word;
typedef signed short int16;
typedef unsigned int uint32, u32;
typedef signed int int32;


#define RTC_CLOCK_SOURCE_LSI
#define EnableIRQ    __enable_irq()
#define DisableIRQ   __disable_irq()


#define ABS(x)   {x >= 0 £¿ x : -x} 
#define TIM1_INT    (TIM1->SR & 0x0001)
#define TIM1_CLEAR  (TIM1->SR &= 0XFFFE)


#define LPWOPEN    1
#define MESSAGEON  1


void delay_ms(uint16 ms);
void delay_us(uint16 us);
void uart1_sendchar(uint8 ch);
void uart1_sendbuff(uint8 * buff, uint16 size);
void uart2_sendchar(uint8 ch);
void uart2_sendbuff(uint8 * buff, uint16 size);
void Delay_Init(void);

#endif