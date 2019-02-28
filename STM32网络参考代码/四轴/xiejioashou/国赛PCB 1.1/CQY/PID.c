#include "stm32f10x.h"
#include "delay.h"
#include "math.h"
#include "Cal.h"
#include "Mot_crtl.h"
#include "ALL_config.h"
#include  "PID.h"
#include "var_global.h"

float PID_Dt; 

PID Pitch,Roll,Yaw;
PID PitchRate,RollRate,YawRate;

float PitchOutput,RollOutput,YawOutput;

float rollRateDesired;
float pitchRateDesired;
float yawRateDesired;

void pidInit(PID* pid, const float desired, const float kp,
             const float ki, const float kd)
{
  pid->Err = 0;
  pid->PreErr = 0;
  pid->integ = 0;
  pid->deriv = 0;
  
	pid->desired = desired;
  pid->kp = kp;
  pid->ki = ki;
  pid->kd = kd;
  
  pid->I_Max = 100;
}

void PID_init_Eur(float p,float i,float d)
{
  Pitch.kp=p;
  Roll. kp=p;
	
	
	Pitch.ki=i;
  Roll. ki=i;

	
	Pitch.kd=d;
  Roll. kd=d;

  PID_Dt=0.005f;
	Pitch.I_Max=20;
	Roll.I_Max=20;
}

void PID_init_Rate(float p,float i,float d)
{
  PitchRate.kp=p;
  RollRate. kp=p;
	
	
	PitchRate.ki=i;
  RollRate. ki=i;
	
	PitchRate.kd=d;
  RollRate. kd=d;
	
	PitchRate.I_Max=500;
	RollRate.I_Max =500;
 
}

void PID_Yaw_Rate(float p,float i,float d)
{
	YawRate.  kp=p;

  YawRate.  ki=i;
	
	YawRate.  kd=d;
	
  YawRate. I_Max =500;
	
	
}

float  pidUpdate(PID* pid, const float measured, const u8 updateError)//PID核心
{
  float output;
 
 if (updateError)//是否更新误差，true则即时计算出err
  {
     pid->Err = pid->desired - measured; 
  }
	
  pid->integ += pid->Err * PID_Dt; //积分操作 累加（误差*数据周期）
  if (pid->integ > pid->I_Max)
  {
    pid->integ = pid->I_Max;
  }
  else if (pid->integ < -pid->I_Max)
  {
    pid->integ = -pid->I_Max;
  }
  //积分饱和限制
                                
  pid->deriv = (pid->Err - pid->PreErr) / PID_Dt;//微分计算，这次误差减去上次误差/数据周期

  pid->outP = pid->kp * pid->Err;//比例计算，P*误差
  pid->outI = pid->ki * pid->integ;
  pid->outD = pid->kd * pid->deriv;

  output = (pid->kp * pid->Err) +
           (pid->ki * pid->integ) +
           (pid->kd * pid->deriv);  //PID输出，PID运算的和

  pid->PreErr = pid->Err;//更新前次误差

  return output;
}

//--------------------------------------------------------------------------//
void pidSetDesired(PID* pid, const float desired)
{
  pid->desired = desired;
}

float pidGetDesired(PID* pid)
{
  return pid->desired;
}

void pidSetError(PID* pid, const float error)
{
  pid->Err = error;
}

 void controllerCorrectRatePID(  //角速度PID控制，（主PID）
       float rollRateActual, float pitchRateActual, float yawRateActual,
       float rollRateDesired, float pitchRateDesired, float yawRateDesired)
{
  pidSetDesired(&RollRate, rollRateDesired);//放入PID期望值
  
	RollOutput=pidUpdate(&RollRate, rollRateActual,1);//计算PID，并输出到控制量

  pidSetDesired(&PitchRate, pitchRateDesired);
  PitchOutput=pidUpdate(&PitchRate, pitchRateActual,1);
	
	 pidSetDesired(&YawRate, yawRateDesired);
   YawOutput=pidUpdate(&YawRate, yawRateActual, 1);


}

void controllerCorrectAttitudePID(//姿态环PID
       float eulerRollActual, float eulerPitchActual, float eulerYawActual,
       float eulerRollDesired, float eulerPitchDesired, float eulerYawDesired,
       float* rollRateDesired, float* pitchRateDesired, float* yawRateDesired)
{ 
	float yawError;//航向角需要运算出最短路径 

  pidSetDesired(&Roll, eulerRollDesired);//放入PID期望值（副PID）
  *rollRateDesired = pidUpdate(&Roll, eulerRollActual,1);//计算PID，并把结果放入角速度期望，这样就能强制响应角度变化


  pidSetDesired(&Pitch, eulerPitchDesired);
  *pitchRateDesired = pidUpdate(&Pitch, eulerPitchActual,1);
	
  
  yawError = eulerYawDesired - eulerYawActual;
  if (yawError > 180.0f)
    yawError -= 360.0f;
  else if (yawError < -180.0f)
    yawError += 360.0f;
  pidSetError(&Yaw, yawError);
  *yawRateDesired = pidUpdate(&Yaw, eulerYawActual, 0);



}

void PID_RPY_Rate()
{
	
	yawRateDesired = EXP_ANGLE.Z;
	
 controllerCorrectRatePID(-GRY_F.X, GRY_F.Y, GRY_F.Z,
                          rollRateDesired, pitchRateDesired, yawRateDesired);//计算角速度环PID，使用角速度计算，执行次数更多

}

void PID_Eur()
{
	
 controllerCorrectAttitudePID(Q_ANGLE.Rool, Q_ANGLE.Pitch, Q_ANGLE.Yaw,
                              EXP_ANGLE.X, EXP_ANGLE.Y, 0,
                              &rollRateDesired, &pitchRateDesired, &yawRateDesired);   //计算姿态环PID
                              //主调输出到副调
}
