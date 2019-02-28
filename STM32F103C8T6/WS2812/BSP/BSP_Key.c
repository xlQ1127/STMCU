#include "BSP_Key.h"

void Key_init(Key_Statu_Typedef *tsKey)
{
	tsKey->Count = 0;
	tsKey->Key1_Flag = 0;
	tsKey->Key2_Flag = 0;
	tsKey->Key3_Flag = 0;
}


void Scan_Key(Key_Statu_Typedef *tsKey)
{

	tsKey->Key1_Flag = ReadKey1;
	tsKey->Key2_Flag = ReadKey2;
	tsKey->Key3_Flag = ReadKey3;

#if HAL_Lib
	HAL_Delay(10);
#endif
#if LL_Lib
	LL_mDelay(10);
#endif
#if OS_Lib
	osDelay(10);
#endif
	if (tsKey->Key1_Flag == 0 && tsKey->Key1_Flag == ReadKey1)
	{
		tsKey->Key1_Flag = 0;
	}

	if (tsKey->Key2_Flag == 0 && tsKey->Key2_Flag == ReadKey2)
	{
		tsKey->Key2_Flag = 0;
	}

	if (tsKey->Key3_Flag == 0 && tsKey->Key3_Flag == ReadKey3)
	{
		tsKey->Key3_Flag = 0;
	}

	if (tsKey->Key1_Flag != Key_Pressed &&
		   tsKey->Key2_Flag != Key_Pressed &&
		   tsKey->Key3_Flag != Key_Pressed )
	{
		tsKey->Count = 0;
		tsKey->Key1_Flag = 1;
		tsKey->Key2_Flag = 1;
		tsKey->Key3_Flag = 1;
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
