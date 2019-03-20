#include "wtn6040.h"



void SendMessage(uint8 num)
{
	uint8 i = 0;
#if (MESSAGEON == 0)
        return;
#endif
	CLKLOW;
	DATHIGH;
	delay_ms(5);
	for (i = 0; i < 8; i++)
	{
		CLKLOW;
		if (num & (1 << i))
		{
			DATHIGH;
		}
		else
		{
			DATLOW;
		}
		delay_us(300);
		CLKHIGH;
		delay_us(300);
	}
	DATHIGH;
	CLKHIGH;
}