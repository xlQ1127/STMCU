#include "stm32f10x.h"
#include "var_global.h"
#include "Cal.h"
#include "Mot_crtl.h"
#include "NRF24L01.H"
#define Moto_PwmMax 999
#include "MPU6050.H"

PID PID_Motor;

float_XYZ EXP_ANGLE ,DIF_ANGLE;


float thr=0;

u8 FLY_Enable=0;

void Mot_output()
{
int16_t Motor1,Motor2,Motor3,Motor4;
	
//--------------------将输出值融合到四个电机--------------------------------//
	
//	Motor1=(int16_t)(thr +PitchOutput  +RollOutput -YawOutput-EXP_ANGLE.Z);   //新机架,全双PID
//	Motor2=(int16_t)(thr -PitchOutput  +RollOutput +YawOutput+EXP_ANGLE.Z);	
//	Motor3=(int16_t)(thr +PitchOutput  -RollOutput +YawOutput+EXP_ANGLE.Z);
//	Motor4=(int16_t)(thr -PitchOutput  -RollOutput -YawOutput-EXP_ANGLE.Z);


	Motor1=(int16_t)(thr -PitchOutput  -RollOutput +YawOutput+EXP_ANGLE.Z);   //新机架,全双PID
	Motor2=(int16_t)(thr -PitchOutput  +RollOutput -YawOutput-EXP_ANGLE.Z);	
	Motor3=(int16_t)(thr +PitchOutput  +RollOutput +YawOutput+EXP_ANGLE.Z);
	Motor4=(int16_t)(thr +PitchOutput  -RollOutput -YawOutput-EXP_ANGLE.Z);

if(FLY_Enable)  
 	 
  PWM_Set(Motor1,Motor2,Motor3,Motor4);  //启动
  else
  
	PWM_Set(0,0,0,0);               //不启动

}

void PWM_Set(int16_t MOTO1_PWM,int16_t MOTO2_PWM,int16_t MOTO3_PWM,int16_t MOTO4_PWM)
{		
	if(MOTO1_PWM>Moto_PwmMax)	MOTO1_PWM = Moto_PwmMax;//PWM限幅
	if(MOTO2_PWM>Moto_PwmMax)	MOTO2_PWM = Moto_PwmMax;
	if(MOTO3_PWM>Moto_PwmMax)	MOTO3_PWM = Moto_PwmMax;
	if(MOTO4_PWM>Moto_PwmMax)	MOTO4_PWM = Moto_PwmMax;
	
  if(MOTO1_PWM<0)	MOTO1_PWM = 0;
	if(MOTO2_PWM<0)	MOTO2_PWM = 0;
	if(MOTO3_PWM<0)	MOTO3_PWM = 0;
	if(MOTO4_PWM<0)	MOTO4_PWM = 0;
	
	TIM2->CCR1 = MOTO1_PWM;
	TIM2->CCR2 = MOTO2_PWM;
	TIM2->CCR3 = MOTO3_PWM;
	TIM2->CCR4 = MOTO4_PWM;
}

 void GET_EXPRAD(void)			//计算期望角度,不加控制时期望角度为0,0
{
	
  DIF_ANGLE.X = EXP_ANGLE.X - Q_ANGLE.Rool;  //【DIF_ANGLE 期望和测量的误差】
	DIF_ANGLE.Y = EXP_ANGLE.Y - Q_ANGLE.Pitch;
  DIF_ANGLE.Z = EXP_ANGLE.Z - Q_ANGLE.Yaw;
	
}







