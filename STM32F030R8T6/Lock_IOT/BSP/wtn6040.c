#include "wtn6040.h"

void SendMessage(uint8_t num)
{
	uint8_t i = 0;

	WTN_CLKLOW;
	WTN_DATHIGH;
	
	
	HAL_Delay(5);
	for (i = 0; i < 8; i++)
	{
		WTN_CLKLOW;
		if (num & (1 << i))
		{
			WTN_DATHIGH;
		}
		else
		{
			WTN_DATLOW;
		}
		
		HAL_Delay(1);
		
		WTN_CLKHIGH;
		
		HAL_Delay(1);
		
	}
	WTN_DATHIGH;
	WTN_CLKHIGH;
}
