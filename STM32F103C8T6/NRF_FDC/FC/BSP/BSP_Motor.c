#include "BSP_Motor.h"

#include "printf_scanf.h"

#define MAX_PWM 1000

#define INT_PMAX +500.0f
#define INT_NMAX -500.0f

void PWM_Set(TIM_HandleTypeDef *htim,int16_t M1,int16_t M2,int16_t M3,int16_t M4)
{											   
		assert_param(IS_TIM_CC1_INSTANCE(htim->Instance));
		htim->Instance->CCR1=M1;
		assert_param(IS_TIM_CC2_INSTANCE(htim->Instance));
		htim->Instance->CCR2=M2;
		assert_param(IS_TIM_CC3_INSTANCE(htim->Instance));
		htim->Instance->CCR3=M3;
		assert_param(IS_TIM_CC4_INSTANCE(htim->Instance));
		htim->Instance->CCR4=M4;
}
void Control_PID_Init( PID_Parameter_Typedef * PID_Par)
{
  PID_Par->Kp=0.0f;
  PID_Par->Ki=0.0f;
	 PID_Par->Kd=0.0f;
}

void Control_PID_Set( PID_Parameter_Typedef * PID_Par,
	                     float P,float I,float D)
{
  PID_Par->Kp=P;
  PID_Par->Ki=I;
	 PID_Par->Kd=D;
}

float Outer_PID_Alg(PID_Parameter_Typedef *Par , float err ,float err_pre)
{
	 static PID_Control_Typedef Control;
	
  Control.P_Out=Par->Kp * err;
	
	 Control.I_Out+=err;
	 Control.I_Out*=Par->Ki; 
	
	 if( Control.I_Out >= INT_PMAX)
		{
		 Control.I_Out=INT_PMAX;
		}
 	if( Control.I_Out <= INT_NMAX)
		{
		 Control.I_Out=INT_NMAX;
		}
		Control.D_Out=(err - err_pre)*Par->Kd;
		
		Control.Out=Control.P_Out+Control.I_Out;
		
		return Control.Out;	
}




float Inner_PID_Alg(PID_Parameter_Typedef *Par , float err ,float err_pre)
{
	static PID_Control_Typedef Control;
	uint8_t I_Flag=0;
	
	Control.P_Out=Par->Kp * err;
 
	if(err>=150 || err<=-150)
	{
	 I_Flag=0;
	}
	else
	{
	 I_Flag=1;
	}
		
	Control.I_Out+=err;
	Control.I_Out=Control.I_Out*Par->Ki; 
	
	
	if( Control.I_Out >= INT_PMAX)
	{
		Control.I_Out=INT_PMAX;
	}
	if( Control.I_Out <= INT_NMAX)
	{
		Control.I_Out=INT_NMAX;
	}
	
	Control.D_Out=(err - err_pre)*Par->Kd;
	
	Control.Out=Control.P_Out+I_Flag*Control.I_Out+Control.D_Out;
	
	return Control.Out;
}

/*   串级PID                       单位
内环 角速度  x轴     y轴    z轴   度每秒
外环 角度    Roll    Pitch  Yaw   度
*/
void Control_Function( int16_t Throttle,int8_t Pitch_Set,int8_t Roll_Set,
	                      float	Pitch	,float	Roll,float	Yaw, float  Gyro_X,float  Gyro_Y,float  Gyro_Z,
	                      PID_Parameter_Typedef * Outer_Pitch_Par, PID_Parameter_Typedef * Inner_Pitch_Par,
																							PID_Parameter_Typedef * Outer_Roll_Par , PID_Parameter_Typedef * Inner_Roll_Par,
																							PID_Parameter_Typedef * Yaw_Par )
{
	 static float Pitch_out=0,Roll_out=0;
	
	 static float Pitch_Err=0,Pitch_Err_pre=0;
	 static float Roll_Err=0,Roll_Err_pre=0;
	
	 static float Gyro_x_Err=0,Gyro_x_Err_pre=0;
		static	float Gyro_y_Err=0,Gyro_y_Err_pre=0;

	 static int16_t  M1,M2,M3,M4;	
	
   if( Throttle > 100 )
			{
				
				Pitch_Err = Pitch_Set-Pitch;
	   Roll_Err  = Roll_Set-Roll;
				
				/*x轴*/ 
				Roll_out    = Outer_PID_Alg(Outer_Roll_Par ,Roll_Err,Roll_Err_pre);  //外环角度 Roll
				Gyro_x_Err  = Roll_out - Gyro_X;
				Roll_out    = Inner_PID_Alg(Inner_Roll_Par ,Gyro_x_Err,Gyro_x_Err_pre);  //内环角速度 
				
				 
				Pitch_out   = Outer_PID_Alg(Outer_Pitch_Par , Pitch_Err ,Pitch_Err_pre);
				Gyro_y_Err  = Pitch_out - Gyro_Y;
				Pitch_out   = Inner_PID_Alg(Inner_Pitch_Par ,Gyro_y_Err ,Gyro_y_Err_pre);  //内环角速度 				
				
				/*更新误差*/
  	 Pitch_Err_pre  = Pitch_Err;
	 		Gyro_y_Err_pre = Gyro_y_Err;
				
		 	Roll_Err_pre   = Roll_Err;
		 	Gyro_x_Err_pre = Gyro_x_Err;

					M1=(int16_t) ( Throttle + Roll_out - Pitch_out );
					M2=(int16_t) ( Throttle + Roll_out + Pitch_out );
					M3=(int16_t) ( Throttle - Roll_out + Pitch_out );
					M4=(int16_t) ( Throttle - Roll_out - Pitch_out );
			
					if(M1>=MAX_PWM)
					{
					M1=MAX_PWM;
					}
					if(M2>=MAX_PWM)
					{
					M2=MAX_PWM;
					}
					if(M3>=MAX_PWM)
					{
					M3=MAX_PWM;
					}
					if(M4>=MAX_PWM)
					{
					M4=MAX_PWM;
					}
					if(M1<=0)
					{
					M1=0;
					}
					if(M2<=0)
					{
					M2=0;
					}
					if(M3<=0)
					{
					M3=0;
					}
					if(M4<=0)
					{
					M4=0;
					}			
				
				 PWM_Set(  &htim2,M1,M2,M3,M4 );
//					ANO_DT_Send_MotoPWM(M1, M2, M3, M4,0,0,0,0);
//					printf("Roll_Err:%f   Gyro_x_Err%f\r\n",Roll_Err,Gyro_x_Err);
			}
			else
			{
			  PWM_Set(  &htim2,0,0,0,0 );
			}
}
