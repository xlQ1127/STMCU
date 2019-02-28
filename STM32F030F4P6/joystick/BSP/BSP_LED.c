#include "BSP_LED.h"



void Blink_LED(uint32_t Time_ms,uint32_t Times)
{
uint8_t i;
   for(i=0;i<Times;i++)
	{
	LL_GPIO_TogglePin(GPIOA, LL_GPIO_PIN_4);
	LL_mDelay(Time_ms);
	}
}

void LED_Erro_Alarm(void)
{
    while(1)
    {
      Blink_LED(1000,2);
    }
	    
}
