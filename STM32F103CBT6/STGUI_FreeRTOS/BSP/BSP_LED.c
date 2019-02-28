#include "BSP_LED.h"


void Forward_Blink(uint16_t Speed)
{

LD2_ON;LD3_OFF;LD4_OFF;LD5_OFF;
HAL_Delay(Speed);

LD2_OFF;LD3_ON;LD4_OFF;LD5_OFF;
HAL_Delay(Speed);

LD2_OFF;LD3_OFF;LD4_ON;LD5_OFF;
HAL_Delay(Speed);

LD2_OFF;LD3_OFF;LD4_OFF;LD5_ON;
HAL_Delay(Speed);

}



void Backward_Blink(uint16_t Speed)
{

LD2_OFF;LD3_OFF;LD4_OFF;LD5_ON;
HAL_Delay(Speed);


LD2_OFF;LD3_OFF;LD4_ON;LD5_OFF;
HAL_Delay(Speed);

LD2_OFF;LD3_ON;LD4_OFF;LD5_OFF;
HAL_Delay(Speed);

LD2_ON;LD3_OFF;LD4_OFF;LD5_OFF;
HAL_Delay(Speed);

}



