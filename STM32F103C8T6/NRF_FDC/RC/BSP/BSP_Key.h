#ifndef __BSP_Key_H__
#define __BSP_Key_H__

#include "stm32f1xx_hal.h"
#include "gpio.h"

#define HAL_GPIO_Lib 1 //1 使用HAL库GPIO API
#define LL_GPIO_Lib 0  //1 使用LL库GPIO  API

#define HAL_Lib 0
#define LL_Lib 0
#define OS_Lib 1

#if OS_Lib
#include "cmsis_os.h"
#endif

typedef struct
{
	uint8_t Key_Left_Flag;
	uint8_t Key_Right_Flag;
	uint8_t Count;
} Key_Statu_Typedef;

/***********************************
键位（高四位）    键消息（低四位）
0000 No key
0001 Key_Down     0000  Key_invalid
0010 Key_Right    0010  Key_Single_Press
0011 Key_Up       0011  Key_Long_Press
0100 Key_Left
0101 Key_Special
***************************************/
typedef enum
{
	Key_invalid = 0,
	Key_Single_Press = 1,
	Key_Long_Press = 2,
	//	Key_VeryLong_Press = 3
} Key_Msg;
typedef struct
{
	uint8_t Key_Left_Msg;
	uint8_t Key_Right_Msg;
	uint8_t Key_Up_Msg;
	uint8_t Key_Down_Msg;

	uint8_t Key_Special_Left_Msg;
	uint8_t Key_Special_Right_Msg;
	uint8_t Key_Special_Up_Msg;
	uint8_t Key_Special_Down_Msg;

	uint8_t Key_Special_Msg;
} Key_Msg_Typedef;

#define Key_Pressed 1
#define Key_NotPressed 0

#if HAL_GPIO_Lib
#define ReadKey_Left_State    HAL_GPIO_ReadPin(KEY_L_GPIO_Port, KEY_L_Pin)
#define ReadKey_Right_State   HAL_GPIO_ReadPin(KEY_R_GPIO_Port, KEY_R_Pin)
#endif

#if LL_GPIO_Lib
#define ReadKey_Left_State    LL_GPIO_IsInputPinSet(Key_Left_GPIO_Port, Key_Left_Pin)
#define ReadKey_Right_State   LL_GPIO_IsInputPinSet(Key_Right_GPIO_Port, Key_Right_Pin)
#endif

void Key_init(Key_Statu_Typedef *tsKey);
void Scan_Key(Key_Statu_Typedef *tsKey);

#endif
