#include "stm32f10x.h"
#include "var_global.h"
#include "Cal.h"
#include "Mot_crtl.h"
#include "NRF24L01.H"
#define Moto_PwmMax 999
#include "MPU6050.H"

PID PID_Motor;
float_XYZ EXP_ANGLE ,DIF_ANGLE;

float PWM_X,PWM_Y ;
float thr=0,rool=0,pitch=0,yaw=0;

u8 FLY_Enable=0;


void Get_RFdata()
{ 
  NRFSetRXMode();	
	
	if(NRFRevDate(RxData)) 
{  dir_time=0;
	
	if (RxData[0]==2 && thr<=300) {thr+= 40;return; }
  if (RxData[0]==2 && thr<=800)  thr+= 20;  
  if (RxData[0]==1 && thr>=10)  thr-= 20; 

	if (RxData[0]==7 && !FLY_Enable ) { Get_Offset();} 
	
  if (RxData[0]==8 && FLY_Enable==1) {thr=0; FLY_Enable=0;return;} 
  if (RxData[0]==8 && FLY_Enable==0) {thr=50; FLY_Enable=1;return;} 


//   if (RxData[0]==3 )  EXP_ANGLE.X=  20;
//   if (RxData[0]==4 )  EXP_ANGLE.X= -20; 
//   if (RxData[0]==5 )  EXP_ANGLE.Y=  20;
//   if (RxData[0]==6 )  EXP_ANGLE.Y= -20; 

  if (RxData[0]==3 )  PWM_X=  50;
  if (RxData[0]==4 )  PWM_X= -50; 
  if (RxData[0]==5 )  PWM_Y=  50;
  if (RxData[0]==6 )  PWM_Y= -50;  
 
}
}



void PID_INIT(void)
{
	PID_Motor.P = 5;
	PID_Motor.I = 0.000;
	PID_Motor.D = 2.2;
	
	PID_Motor.POUT = 0;
	PID_Motor.IOUT = 0;
	PID_Motor.DOUT = 0;
	
	PID_Motor.IMAX = 50;
	PID_Motor.LastError = 0;
	PID_Motor.PrerError = 0;
}




void PID_CAL(void)
{
	
	static float rool_i=0,pitch_i=0,yaw_i=0;
	
	int16_t Motor1,Motor2,Motor3,Motor4;
	
	GET_EXPRAD();

//-----------------------------------Rool 处理-------------------------------PID计算
	rool 	= PID_Motor.P * DIF_ANGLE.X; //【进行比例控制，也就是误差乘积】
	
	if(Q_ANGLE.Rool>-0.1 && Q_ANGLE.Rool<0.1)
	{                                                                            
		rool_i = 0;
	}
        
        
	rool_i -= PID_Motor.I * Q_ANGLE.Rool;
	
        PID_Motor.IMAX = DIF_ANGLE.X * 10;//【最大值控制，不能大于误差的10倍】
	
        if(PID_Motor.IMAX<0)//得出限幅最大值	
	{
		PID_Motor.IMAX = (-PID_Motor.IMAX) + 100;
	}
	else
	{
		PID_Motor.IMAX += 100;
	}
	
        if(rool_i>PID_Motor.IMAX) 	rool_i = PID_Motor.IMAX;//I进行限幅保护
	if(rool_i<-PID_Motor.IMAX)	rool_i = -PID_Motor.IMAX;
	
        rool += rool_i;//【进行积分控制，也就是加】
	
	rool -= PID_Motor.D * (GRY_F.X);//【进行微分控制，陀螺仪就是微分量】

//---------------------------------pitch 处理-----------------------------------//	
	pitch 	= PID_Motor.P * DIF_ANGLE.Y;
		
	if(Q_ANGLE.Pitch>-0.1 && Q_ANGLE.Pitch<0.1)
	{
		pitch_i = 0;
	}
        
	pitch_i -= PID_Motor.I * Q_ANGLE.Pitch;
	
        if(PID_Motor.IMAX<0)	
	{
		PID_Motor.IMAX = (-PID_Motor.IMAX) + 100;
	}
	else
	{
		PID_Motor.IMAX += 100;
	}
	
        if(PID_Motor.IMAX<0)	PID_Motor.IMAX = -PID_Motor.IMAX;
	
        if(pitch_i>PID_Motor.IMAX) 	pitch_i = PID_Motor.IMAX;
	if(pitch_i<-PID_Motor.IMAX)	pitch_i = -PID_Motor.IMAX;
	pitch += pitch_i;
	
	pitch -= PID_Motor.D * (GRY_F.Y);

//--------------yaw-----------------------//
  //yaw =0;
	yaw =  DIF_ANGLE.Z*3;	
	
	if(Q_ANGLE.Yaw>-0.1 && Q_ANGLE.Yaw<0.1)
	{
		yaw_i = 0;
	}       
	yaw_i -= PID_Motor.I * Q_ANGLE.Yaw;
        if(PID_Motor.IMAX<0)	
	{
		PID_Motor.IMAX = (-PID_Motor.IMAX) + 100;
	}
	else
	{
		PID_Motor.IMAX += 100;
	}
	
	
  if(PID_Motor.IMAX<0)	PID_Motor.IMAX = -PID_Motor.IMAX;
  if(yaw_i>PID_Motor.IMAX) 	yaw_i = PID_Motor.IMAX;
	if(yaw_i<-PID_Motor.IMAX)	yaw_i = -PID_Motor.IMAX;
	
	
	//yaw += yaw_i;
	yaw -= PID_Motor.D * (GRY_F.Z);
	
	if(yaw> 100) yaw= 200;
	if(yaw<-100) yaw=-200;

//--------------------将输出值融合到四个电机--------------------------------//

  Motor1=(int16_t)(thr + rool - pitch + yaw)-PWM_X-PWM_Y;
	Motor2=(int16_t)(thr - rool - pitch - yaw)+PWM_X-PWM_Y;	
	Motor3=(int16_t)(thr - rool + pitch + yaw)+PWM_X+PWM_Y;
	Motor4=(int16_t)(thr + rool + pitch - yaw)-PWM_X+PWM_Y;

	
	
	
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
// 	EXP_ANGLE.X = (float)(-RC_DATA.ROOL/15);
// 	EXP_ANGLE.Y = (float)(-RC_DATA.PITCH/15);
// 	EXP_ANGLE.Z = (float)(RC_DATA.YAW);
	
	EXP_ANGLE.X =0;
 	EXP_ANGLE.Y =0;
 //	EXP_ANGLE.Z =0;
	
  DIF_ANGLE.X = EXP_ANGLE.X - Q_ANGLE.Rool;  //【DIF_ANGLE 期望和测量的误差】
	DIF_ANGLE.Y = EXP_ANGLE.Y - Q_ANGLE.Pitch;
 // DIF_ANGLE.Z = EXP_ANGLE.Z - Q_ANGLE.Yaw;
	
 DIF_ANGLE.Z = EXP_ANGLE.Z -MAG_angle;
	
}







