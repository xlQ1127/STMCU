#ifndef __BSP_MOTOR_H__
#define __BSP_MOTOR_H__

#include "tim.h"
#include "arm_math.h"

typedef struct{
float P_Out;
float I_Out;
float D_Out;
float Out;		
}PID_Control_Typedef;  // 10B

typedef struct{
float Kp;
float Ki;
float Kd;		
}PID_Parameter_Typedef;  // 10B


void PWM_Set(TIM_HandleTypeDef *htim,int16_t M1,int16_t M2,int16_t M3,int16_t M4);

void Control_PID_Init( PID_Parameter_Typedef * PID_Par);

void Control_PID_Set( PID_Parameter_Typedef * PID_Par,
	                     float P,float I,float D);
																						
void Control_Function( int16_t Throttle,int8_t Pitch_Set,int8_t Roll_Set,
	                      float	Pitch	,float	Roll,float	Yaw, float  Gyro_X,float  Gyro_Y,float  Gyro_Z,
	                      PID_Parameter_Typedef * Outer_Pitch_Par,PID_Parameter_Typedef * Inner_Pitch_Par,
																							PID_Parameter_Typedef * Outer_Roll_Par,PID_Parameter_Typedef * Inner_Roll_Par,
																							PID_Parameter_Typedef * Yaw_Par );
													
#endif
