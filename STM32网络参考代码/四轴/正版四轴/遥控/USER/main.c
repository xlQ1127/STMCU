#include "stm32f10x.h"
#include "stm32f10x_conf.h"

#include "SysTick.h"
#include "TIM3_IT.h"
#include "led.h"
#include "adc.h"
#include "usart.h"
#include "SPI_IO.h"
#include "NRF24L01.h"

int ok;

int main(void)   
{	
  SysTick_Init();
	Tim3_Init(5000);
	LED_GPIO_Config();
	ADC1_Init();
	USART1_Config();
	Spi1_Init();
	Delay_us(50000);
	Nrf24l01_Init(MODEL_RX2,100);


  while(1)   
  { 
    LED1(OFF); 
		Delay_us(500000);			
    LED1(ON); 
		Delay_us(500000);	
	}
}
