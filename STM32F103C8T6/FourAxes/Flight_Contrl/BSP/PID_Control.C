#include "PID_Control.h"
#include "BSP_MPU6050.h"
#include "tim.h"
/**********************************************************
           Y(4095)                Pitch Y(0)
           |                            | 
           |                            |
X(4095)---------X(0) YAW   Roll X(0)---------X(4095)        
           |                            |
           |                            |
           Y(0)                         Y(4095)
          throttle

                     Y
                    |
        motor1      |        motor4
                    |
                    |
                    |
--------------------O------------------>X
                    |
                    |
        motor2      |        motor3
                    |


**********************************************************/



PID PID_ROL,PID_PIT,PID_YAW;


extern TIM_HandleTypeDef htim1;


void PID_init(float Roll_P,float Roll_I,float Roll_D,float Pitch_P,float Pitch_I,float Pitch_D)
{
  PID_ROL.P = Roll_P;
		PID_ROL.I = Roll_I;
		PID_ROL.D = Roll_D;//P*TD=D  TD=D/P
	
		PID_PIT.P = Pitch_P;
		PID_PIT.I = Pitch_I;
		PID_PIT.D = Pitch_D;
	
		PID_YAW.P = 0.0;
		PID_YAW.I = 0.0;
		PID_YAW.D = 0.0;
}

void PWM_Set(TIM_HandleTypeDef *htim,uint16_t Motor1,uint16_t Motor2,uint16_t Motor3,uint16_t Motor4)
{
	
      assert_param(IS_TIM_CC1_INSTANCE(htim->Instance));
      htim->Instance->CCR1=Motor1;

      assert_param(IS_TIM_CC2_INSTANCE(htim->Instance));
      htim->Instance->CCR2=Motor2;

      assert_param(IS_TIM_CC3_INSTANCE(htim->Instance));
      htim->Instance->CCR3=Motor3;

      assert_param(IS_TIM_CC4_INSTANCE(htim->Instance));
      htim->Instance->CCR4=Motor4;
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


void PID_CONTROL(float rol_now, float pit_now, float yaw_now, 
                 float rol_tar, float pit_tar, float yaw_tar, 
                 float X_w, float Y_w, float Z_w,
                __IO float throttle, __IO int8_t FlyMode)//角速度
{
	int16_t   moto1=0,
           moto2=0,
           moto3=0,
           moto4=0;
	
	float rol_err,pit_err;
 
  if(FlyMode==2)	 //FlyMode  0: NO Control and Do Nothing;     1:Start Fly and RemoteControl;   2:Stop Fly and Stay where it were ;  3:Stop ALL Motor;
		{
   PWM_Set(&htim1,0,0,0,0);
		}

  
		
		else if( FlyMode==1 || FlyMode==4 )
  {
			
					if(throttle>100)
					{
								rol_err = rol_now - rol_tar;
								pit_err = pit_now - pit_tar; 
										
								PID_ROL.IMAX = throttle/2;	
								PID_PIT.IMAX = PID_ROL.IMAX;
							
		/***********************比例******************************/
								PID_ROL.pout = PID_ROL.P * rol_err;
								PID_PIT.pout = PID_PIT.P * pit_err;

		/***********************积分******************************/					
								PID_ROL.iout += PID_ROL.I * rol_err;
								PID_PIT.iout += PID_PIT.I * pit_err;
								PID_ROL.iout = Get_MxMi(PID_ROL.iout,PID_ROL.IMAX,-PID_ROL.IMAX);
								PID_PIT.iout = Get_MxMi(PID_PIT.iout,PID_PIT.IMAX,-PID_PIT.IMAX);
				
		/***********************微分******************************/													
								PID_ROL.dout = PID_ROL.D * X_w;
								PID_PIT.dout = PID_PIT.D * Y_w;
							
		/***********************输出******************************/	
								PID_ROL.OUT = PID_ROL.pout + PID_ROL.iout + PID_ROL.dout;
								PID_PIT.OUT = PID_PIT.pout + PID_PIT.iout + PID_PIT.dout;

								moto1 =(int16_t) ( throttle - PID_ROL.OUT - PID_PIT.OUT );//+ PID_YAW.OUT;
								moto2 =(int16_t) ( throttle + PID_ROL.OUT - PID_PIT.OUT );//- PID_YAW.OUT;
								moto3 =(int16_t) ( throttle + PID_ROL.OUT + PID_PIT.OUT );//+ PID_YAW.OUT;
								moto4 =(int16_t) ( throttle - PID_ROL.OUT + PID_PIT.OUT );//- PID_YAW.OUT;
																			
								if(moto1>=2000)
									{
										moto1=2000;
									}
								if(moto2>=2000)
									{
										moto2=2000;
									}
								if(moto3>=2000)
									{
										moto3=2000;
									}
								if(moto4>=2000)
									{
										moto4=2000;
									}

								if(moto1<=0)
									{
										moto1=0;
									}
								if(moto2<=0)
									{
										moto2=0;
									}
								if(moto3<=0)
									{
										moto3=0;
									}
								if(moto4<=0)
									{
										moto4=0;
									}
					   PWM_Set(&htim1,moto1,moto2,moto3,moto4);
					}
					
					
					else
					{
						PWM_Set(&htim1,0,0,0,0);
					}
					
  }
		
		else 
		{
				PWM_Set(&htim1,0,0,0,0);
		}
			  
}
