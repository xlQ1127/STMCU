#include "control.h"
#include "TIM4_PWM.h"


PID PID_ROL,PID_PIT,PID_YAW;

vs16 accelerator;
int  Pitch_ta;
int  Roll_ta;
int  Yaw_ta;

unsigned int  POWER;

void PID_init(void)
{
    PID_ROL.P = 1.0;
		PID_ROL.I = 0;
		PID_ROL.D = 0.1;
	
		PID_PIT.P = 6.0;
		PID_PIT.I = 0;
		PID_PIT.D = 0.8;
	
		PID_YAW.P = 0;
		PID_YAW.I = 0;
		PID_YAW.D = 0;
}
float Get_MxMi(float num,float max,float min)
{
	if(num>max)
		return max;
	else if(num<min)
		return min;
	else
		return num;
}
void CONTROL(float rol_now, float pit_now, float yaw_now, float rol_tar, float pit_tar, float yaw_tar, vs16 throttle, float X_w, float Y_w, float Z_w)
{
	int moto1=0,moto2=0,moto3=0,moto4=0;
	
	float rol = rol_tar + rol_now;
	float pit = pit_tar + pit_now;  
	
	PID_ROL.IMAX = throttle/2;	
	Get_MxMi(PID_ROL.IMAX,1000,0);
	PID_PIT.IMAX = PID_ROL.IMAX;
	
	PID_ROL.pout = PID_ROL.P * rol;
	PID_PIT.pout = PID_PIT.P * pit;
	
	if(rol_tar*rol_tar<0.1 && pit_tar*pit_tar<0.1 && rol_now*rol_now<30 && pit_now*pit_now<30 && throttle>300)
	{
		PID_ROL.iout += PID_ROL.I * rol;
		PID_PIT.iout += PID_PIT.I * pit;
		PID_ROL.iout = Get_MxMi(PID_ROL.iout,PID_ROL.IMAX,-PID_ROL.IMAX);
		PID_PIT.iout = Get_MxMi(PID_PIT.iout,PID_PIT.IMAX,-PID_PIT.IMAX);
	}
	else if(throttle<200)
	{
		PID_ROL.iout = 0;
		PID_PIT.iout = 0;
	}
	
	PID_ROL.dout = PID_ROL.D * X_w;
	PID_PIT.dout = PID_PIT.D * Y_w;
		
	PID_ROL.OUT = PID_ROL.pout + PID_ROL.iout + PID_ROL.dout;
	PID_PIT.OUT = PID_PIT.pout + PID_PIT.iout + PID_PIT.dout;
	
	if(throttle>200)
	{
		moto1 = throttle - PID_ROL.OUT + PID_PIT.OUT ;//+ PID_YAW.OUT;
		moto2 = throttle - PID_ROL.OUT - PID_PIT.OUT ;//- PID_YAW.OUT;
		moto3 = throttle + PID_ROL.OUT - PID_PIT.OUT ;//+ PID_YAW.OUT;
		moto4 = throttle + PID_ROL.OUT + PID_PIT.OUT ;//- PID_YAW.OUT;
	}
	else
	{
		moto1 = 0;
		moto2 = 0;
		moto3 = 0;
		moto4 = 0;
	}
	if(POWER)	Moto_PwmRflash(moto1,moto2,moto3,moto4);
	else			Moto_PwmRflash(0,0,0,0);
}
