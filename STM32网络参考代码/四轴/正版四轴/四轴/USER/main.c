#include "stm32f10x.h"
#include "stm32f10x_conf.h"

#include "SysTick.h"
#include "TIM3_IT.h"
#include "led.h"
#include "adc.h"
#include "usart.h"
#include "IIC.h"
#include "MPU6050.h"
#include "TIM4_PWM.h"
#include "SPI_IO.h"
#include "NRF24L01.h"
#include "control.h"

int main(void)   
{	
  SysTick_Init();
	ADC1_Init();	
	USART1_Config();
	IIC_GPIO_Config();	
	Delay_us(10000);
	InitMPU6050();
	Spi1_Init();
	Nrf24l01_Init(MODEL_TX2,100);
	Delay_us(5000);	
	PID_init();
	TIM4_PWM_Init();
	Tim3_Init(5000);

  while(1)   
  { 

	}
}
