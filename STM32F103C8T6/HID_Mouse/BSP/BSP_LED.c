#include "BSP_LED.h"
void Blink_LED(uint32_t Time_ms,uint32_t Times)
{
uint8_t i;
   for(i=0;i<Times;i++)
	{
	HAL_GPIO_TogglePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin);
	HAL_Delay(Time_ms);
	}
}

void Lighting_LED(void)
{

	HAL_GPIO_WritePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin, GPIO_PIN_SET)   ;
	HAL_GPIO_WritePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin, GPIO_PIN_RESET)	;	
	HAL_Delay(20);
	HAL_GPIO_WritePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin, GPIO_PIN_SET)   ;
	
}

