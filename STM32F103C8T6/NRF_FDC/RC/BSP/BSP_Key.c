#include "BSP_Key.h"

void Key_init(Key_Statu_Typedef *tsKey)
{
	tsKey->Count = 0;

	tsKey->Key_Left_Flag = 0;
	tsKey->Key_Right_Flag = 0;
}
void Scan_Key(Key_Statu_Typedef *tsKey)
{

	tsKey->Key_Left_Flag  =  ReadKey_Left_State;   //低电平按下   拉高
	tsKey->Key_Left_Flag=((tsKey->Key_Left_Flag==GPIO_PIN_SET)?0UL:1UL);
	tsKey->Key_Right_Flag = ReadKey_Right_State;  //高电平按下   拉低
 
#if HAL_Lib
	HAL_Delay(10);
#endif
#if LL_Lib
	LL_mDelay(10);
#endif
#if OS_Lib
	osDelay(10);
#endif
	
	if (tsKey->Key_Left_Flag == 1 && (tsKey->Key_Left_Flag ==   ((tsKey->Key_Left_Flag==GPIO_PIN_SET)?0UL:1UL)) )
	{
		tsKey->Key_Left_Flag = 1;
	}

	if (tsKey->Key_Right_Flag == 1 && (tsKey->Key_Right_Flag == ReadKey_Right_State) )
	{
		tsKey->Key_Right_Flag = 1;
	}
	if (tsKey->Key_Left_Flag  != Key_Pressed &&
	   	tsKey->Key_Right_Flag != Key_Pressed
    )
	{
		tsKey->Count = 0;

		tsKey->Key_Left_Flag = 0;
		tsKey->Key_Right_Flag = 0;
	}
	else
	{
		tsKey->Count++;
	}
	
}


uint8_t Send_Key_Msg(void)
{
	Key_Statu_Typedef sKey;
	//	 uint8_t Key_Msg=0;
	//  sKey=Scan_Key();
	if (sKey.Count > 30 && sKey.Count <= 50)
	{
	}
	else if (sKey.Count > 10 && sKey.Count <= 30)
	{
	}
	else
	{
	}
	return 0;
}
