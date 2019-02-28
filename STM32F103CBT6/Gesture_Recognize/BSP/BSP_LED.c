#include "BSP_LED.h"

void Blink_LED(uint32_t Time_ms, uint32_t Times)
{
	uint8_t i;

	for (i = 0; i < Times; i++)
	{
		//					HAL_GPIO_TogglePin(LED1_GPIO_Port, LED1_Pin);
		HAL_Delay(Time_ms);
	}
}
