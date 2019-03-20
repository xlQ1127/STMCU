#include "motor.h"

#define MOTORDIR  0

static uint8_t  MotorOverCount = 0;
static uint32_t MotorForceCount = 0;
static uint32_t M_ADC_Data[2];


/////电机正转
void MotorForward(void)
{
#if (MOTORDIR == 1)
	MOTOR_F_HIGH;
#elif (MOTORDIR == 0)
	MOTOR_B_LOW;
	MOTOR_B_HIGH;
	MOTOR_F_LOW;
#endif	
}

//////电机反转
void MotorBackward(void)
{
#if (MOTORDIR == 0)
	MOTOR_F_HIGH;
	MOTOR_B_LOW;
#elif (MOTORDIR == 1)
	MOTOR_B_HIGH;
	MOTOR_F_LOW;
#endif	
}

//电机停
void MotorStop(void)
{
	MOTOR_B_HIGH;
	MOTOR_F_HIGH;
}

//开门
void DoorOpen(void)
{
	MotorOverCount = 0;
	MotorForceCount = 0;
	MotorForward();
	
	HAL_Delay(MOTOR_ELECURDELAY);
	
	while( (MotorOverCount < MOTOR_ELECUROVERCOUNT) && 
    		  (MotorForceCount < MOTOR_MOTORFORCELIMIT) )
	{
		HAL_ADC_Start_DMA(&hadc, M_ADC_Data, 2);
		if( M_ADC_Data[0] > MOTOR_ELECUROVER)
		{
			MotorOverCount++;
		}
		MotorForceCount++;
		HAL_ADC_Stop(&hadc);
	}
	
 MotorStop();
 HAL_Delay(300);
	MotorBackward();
	HAL_Delay(MOTOR_RETURNHALFCOUNT);
	MotorStop();
}

//关门
void DoorClose(void)
{
	MotorOverCount = 0;
	MotorForceCount = 0;
	MotorBackward();
	HAL_Delay(MOTOR_ELECURDELAY);
	while( (MotorOverCount < MOTOR_ELECUROVERCOUNT) &&
    		  (MotorForceCount < MOTOR_MOTORFORCELIMIT)  )
		{
				HAL_ADC_Start_DMA(&hadc, M_ADC_Data, 3);
				if( M_ADC_Data[0] > MOTOR_ELECUROVER)
				{
					MotorOverCount++;
				}
				MotorForceCount++;
				HAL_ADC_Stop(&hadc);
		}
	 MotorStop();
}

