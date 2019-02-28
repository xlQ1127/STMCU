#include "BSP_Key.h"

void Key_init(Key_Statu_Typedef *tsKey)
{
	tsKey->Count = 0;

	tsKey->Key_Down_Flag = 0;
	tsKey->Key_Left_Flag = 0;
	tsKey->Key_Right_Flag = 0;
	tsKey->Key_Up_Flag = 0;

	tsKey->Key_Special_Flag = 0;
	tsKey->Key_WKUP_Flag = 0;

	tsKey->Key_Special_Down_Flag = 0;
	tsKey->Key_Special_Left_Flag = 0;
	tsKey->Key_Special_Right_Flag = 0;
	tsKey->Key_Special_Up_Flag = 0;
}
void Scan_Key(Key_Statu_Typedef *tsKey)
{

	tsKey->Key_Left_Flag = ReadKey_Left_State;
	tsKey->Key_Right_Flag = ReadKey_Right_State;
	tsKey->Key_Up_Flag = ReadKey_Up_State;
	tsKey->Key_Down_Flag = ReadKey_Down_State;

	tsKey->Key_Special_Flag = ReadKey_Special_State;
	//	  tsKey->Key_WKUP_Flag = ReadKey_WKUP_State ;

#if HAL_Lib
	HAL_Delay(10);
#endif
#if LL_Lib
	LL_mDelay(10);
#endif
#if OS_Lib
	osDelay(10);
#endif
	if (tsKey->Key_Left_Flag == 1 && tsKey->Key_Left_Flag == ReadKey_Left_State)
	{
		tsKey->Key_Left_Flag = 1;
	}

	if (tsKey->Key_Right_Flag == 1 && tsKey->Key_Right_Flag == ReadKey_Right_State)
	{
		tsKey->Key_Right_Flag = 1;
	}

	if (tsKey->Key_Up_Flag == 1 && tsKey->Key_Up_Flag == ReadKey_Up_State)
	{
		tsKey->Key_Up_Flag = 1;
	}

	if (tsKey->Key_Down_Flag == 1 && tsKey->Key_Down_Flag == ReadKey_Down_State)
	{
		tsKey->Key_Down_Flag = 1;
	}

	if (tsKey->Key_Special_Flag == 1 && tsKey->Key_Special_Flag == ReadKey_Special_State)
	{
		tsKey->Key_Special_Flag = 1;
	}

	//			if(  tsKey->Key_WKUP_Flag == 1 && tsKey->Key_WKUP_Flag == ReadKey_WKUP_State )
	//			{
	//     tsKey->Key_WKUP_Flag = 1;
	//			}

	if (tsKey->Key_Left_Flag != Key_Pressed &&
		tsKey->Key_Right_Flag != Key_Pressed &&
		tsKey->Key_Up_Flag != Key_Pressed &&
		tsKey->Key_Down_Flag != Key_Pressed &&
		tsKey->Key_Special_Flag != Key_Pressed)
	{
		tsKey->Count = 0;

		tsKey->Key_Left_Flag = 0;
		tsKey->Key_Right_Flag = 0;
		tsKey->Key_Down_Flag = 0;
		tsKey->Key_Up_Flag = 0;
		tsKey->Key_Special_Flag = 0;
		tsKey->Key_WKUP_Flag = 0;
	}
	else
	{
		tsKey->Count++;
	}
	//			tsKey->Key_Left_Flag
	//	  tsKey->Key_Right_Flag
	//   tsKey->Key_Up_Flag
	//	  tsKey->Key_Down_Flag
	//
	//	  tsKey->Key_Special_Flag
	if (tsKey->Key_Special_Flag == 1)
	{

		if (tsKey->Key_Left_Flag == 1)
		{
			tsKey->Key_Left_Flag = 0;
			tsKey->Key_Special_Flag = 0;
			tsKey->Key_Special_Left_Flag = 1;
		}
		else if (tsKey->Key_Right_Flag == 1)
		{
			tsKey->Key_Right_Flag = 0;
			tsKey->Key_Special_Flag = 0;
			tsKey->Key_Special_Right_Flag = 1;
		}
		else if (tsKey->Key_Up_Flag == 1)
		{
			tsKey->Key_Up_Flag = 0;
			tsKey->Key_Special_Flag = 0;
			tsKey->Key_Special_Up_Flag = 1;
		}
		else if (tsKey->Key_Down_Flag == 1)
		{
			tsKey->Key_Down_Flag = 0;
			tsKey->Key_Special_Flag = 0;
			tsKey->Key_Special_Down_Flag = 1;
		}
		else
		{
			tsKey->Key_Special_Flag = 1;
		}
	}
	else
	{
		tsKey->Key_Special_Flag = 0;

		tsKey->Key_Special_Right_Flag = 0;
		tsKey->Key_Special_Up_Flag = 0;
		tsKey->Key_Special_Left_Flag = 0;
		tsKey->Key_Special_Down_Flag = 0;
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
