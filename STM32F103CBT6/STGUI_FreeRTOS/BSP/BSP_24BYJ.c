#include "BSP_24BYJ.h"

uint16_t i;

void BYJ_Rotate_Clockwise(uint8_t Speed)
{

	A_In_Set;B_In_Set;C_In_Reset;D_In_Reset;
	HAL_Delay(Speed);
	A_In_Reset;B_In_Set;C_In_Set;D_In_Reset;
	HAL_Delay(Speed);
	A_In_Reset;B_In_Reset;C_In_Set;D_In_Set;
	HAL_Delay(Speed);
	A_In_Set;B_In_Reset;C_In_Reset;D_In_Set;
	HAL_Delay(Speed);

}

void BYJ_Rotate_Anticlockwise(uint8_t Speed)
{
	A_In_Set;B_In_Reset;C_In_Reset;D_In_Set;
	HAL_Delay(Speed);

	A_In_Reset;B_In_Reset;C_In_Set;D_In_Set;
	HAL_Delay(Speed);

	A_In_Reset;B_In_Set;C_In_Set;D_In_Reset;
	HAL_Delay(Speed);

	A_In_Set;B_In_Set;C_In_Reset;D_In_Reset;
	HAL_Delay(Speed);

}


void BYJ_Stop(void)
{
D_In_Reset;
C_In_Reset;
B_In_Reset;
A_In_Reset;
}

