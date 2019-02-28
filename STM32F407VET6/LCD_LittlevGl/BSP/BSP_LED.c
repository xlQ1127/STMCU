#include "BSP_LED.h"



void Blink_LED(uint32_t Time_ms,uint32_t Times)
{
uint8_t i;
   for(i=0;i<Times;i++)
	{
	HAL_GPIO_TogglePin(GPIOA, D2_Pin|D3_Pin);
	HAL_Delay(Time_ms);
	}
}

void LED_Erro_Alarm(void)
{
    while(1)
    {
      Blink_LED(1000,2);
    }
	    
}
