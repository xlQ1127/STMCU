/**********************************************************************************************************************
******串级PID控制程序、外环采用角度控制，内环采用角速度控制（直接由陀螺仪输出） 外环角速为欧拉角（硬件DMP）*****
***********************************************************************************************************************
******本子程序为四轴飞行器核心算法 ********************************************************************************/
/****************************
					 Y(Roll)
     顺时针转 | 逆时针转
      motor1  |  motor4
       正桨   |   反桨
    --------------------X(Pitch)          
     逆时针转 | 顺时针转 
      motor2  |  motor3
       反桨   |   正桨 
              |
****************************/
#include "control.h" 
#include "led.h"
#include "delay.h"
#include "sys.h"
#include "usart.h"
#include "key.h"
#include "pwm_out.h"
#include "mpu6050.h"
#include "ahrs.h"
#include "inv_mpu.h"
#include "inv_mpu_dmp_motion_driver.h" 
 
//即将要更新的ACC加速度 GYRO陀螺仪
S_INT16_XYZ MPU6050_GYRO_LAST;

T_RC_Dat  Rc_D={0,0,0,1613};//遥控通道数据;
u8 lock=0;   //飞控上锁标志位
extern u8 txbuf[4];  //发送缓冲
extern u8 rxbuf[4];  //接收缓冲
extern u16 test1[3]; //接收到NRf24L01数据

float thr=0;//油门


//PID1 PID_Motor;
/***************************************************************/
float Pitch_i,Roll_i,Yaw_i;              //积分项
float Pitch_old,Roll_old,Yaw_old;        //角度保存
float Pitch_d,Roll_d,Yaw_d;              //微分项

float RC_Pitch=0,RC_Roll=0,RC_Yaw=0;           //期望的姿态角

float Pitch_shell_out,Roll_shell_out,Yaw_shell_out;//外环总输出
        //外环PID参数
float Pitch_shell_kp=140;
float Pitch_shell_ki=0.2;
float Pitch_shell_kd=0.8;
/*********************************/
float Roll_shell_kp=150;              
float Roll_shell_ki=0.2;
float Roll_shell_kd=0.8; 
/*********************************/	
float Yaw_shell_kp=15;//0.87              
float Yaw_shell_ki=0.002;
float Yaw_shell_kd=0.02;   
float Gyro_radian_old_x,Gyro_radian_old_y,Gyro_radian_old_z;//陀螺仪保存
float pitch_core_kp_out,pitch_core_ki_out,pitch_core_kd_out,Roll_core_kp_out,\
			Roll_core_ki_out,Roll_core_kd_out,Roll_core_ki_out,Yaw_core_kp_out,\
			Yaw_core_ki_out,Yaw_core_kd_out;//内环单项输出

float Pitch_core_out,Roll_core_out,Yaw_core_out;//内环总输出       
       
//内环PID参数
float Pitch_core_kp=0.02;
float Pitch_core_ki=0;
float Pitch_core_kd=0.001;//0.001

float Roll_core_kp=0.02;
float Roll_core_ki=0;
float Roll_core_kd=0.001;

float Yaw_core_kp=0.001;
float Yaw_core_ki=0;
float Yaw_core_kd=0.001;


int16_t moto1=1613,moto2=1613,moto3=1613,moto4=1613;//电机的PWM值


/////////////////////////////////////////////////////////////////
//PID 控制函数
//输入：采样的姿态欧拉角数据
//反回：无
//备注：Pitch、Roll_i采用内环、外环PID控制 ，yaw采用内环控制
////////////////////////////////////////////////////////////////
void CONTROL_PID(float pit, float rol, float yaw)
{

  RC_Pitch=(Rc_D.pitch -1499.0f )/50;
  RC_Pitch-=3.3f;
//////////////////外环(PID)/////////////
  Pitch_i+=(pit-RC_Pitch);
//-------------Pitch积分限幅----------------//
  if(Pitch_i>300) Pitch_i=300;
  else if(Pitch_i<-300) Pitch_i=-300;
//-------------Pitch微分--------------------//
  Pitch_d=pit-Pitch_old;
//-------------Pitch  PID-------------------//
  Pitch_shell_out = Pitch_shell_kp*(pit-RC_Pitch) + Pitch_shell_ki*Pitch_i + Pitch_shell_kd*Pitch_d;
//保存角度
  Pitch_old=pit;
/*********************************************************/       
       
	RC_Roll=(Rc_D.roll-1502)/50;
	Roll_i+=(rol-RC_Roll);
//-------------Roll积分限幅----------------//
  if(Roll_i>300) Roll_i=300;
  else if(Roll_i<-300) Roll_i=-300;
//-------------Roll微分--------------------//
  Roll_d=rol-Roll_old;
//-------------Roll  PID-------------------//
  Roll_shell_out  = Roll_shell_kp*(rol-RC_Roll) + Roll_shell_ki*Roll_i + Roll_shell_kd*Roll_d;
//------------Roll保存角度------------------//
  Roll_old=rol;
       
       
  //RC_Yaw=(Rc_D.yaw-1501)*10;
//-------------Yaw微分--------------------//
  Yaw_d=MPU6050_GYRO_LAST.Z-Yaw_old;
//-------------Roll  PID-------------------//
  Yaw_shell_out  = Yaw_shell_kp*(MPU6050_GYRO_LAST.Z-RC_Yaw) + Yaw_shell_ki*Yaw_i + Yaw_shell_kd*Yaw_d;
//------------Roll保存角度------------------//
  Yaw_old=MPU6050_GYRO_LAST.Z;

////////////////////////内环角速度环(PD)/////////////////////////////// 
//陀螺仪	测得是角速度
  pitch_core_kp_out = Pitch_core_kp * (Pitch_shell_out + MPU6050_GYRO_LAST.Y );
  pitch_core_kd_out = Pitch_core_kd * (MPU6050_GYRO_LAST.Y   - Gyro_radian_old_y);

  Roll_core_kp_out  = Roll_core_kp  * (Roll_shell_out  + MPU6050_GYRO_LAST.X );
  Roll_core_kd_out  = Roll_core_kd  * (MPU6050_GYRO_LAST.X   - Gyro_radian_old_x);

  Yaw_core_kp_out  = Yaw_core_kp  * (Yaw_shell_out  + MPU6050_GYRO_LAST.Z);
  Yaw_core_kd_out  = Yaw_core_kd  * (MPU6050_GYRO_LAST.Z   - Gyro_radian_old_z);
       
       
  Pitch_core_out = pitch_core_kp_out + pitch_core_kd_out;
  Roll_core_out  = Roll_core_kp_out  + Roll_core_kd_out;
  Yaw_core_out   = Yaw_core_kp_out   + Yaw_core_kd_out;

  Gyro_radian_old_y = MPU6050_GYRO_LAST.X;
  Gyro_radian_old_x = MPU6050_GYRO_LAST.Y;
  Gyro_radian_old_z = MPU6050_GYRO_LAST.Z;   

       
//--------------------将输出值融合到四个电机--------------------------------//
				       
        if(Rc_D.THROTTLE>1613)
        {
					thr=Rc_D.THROTTLE;               
					moto1=(int16_t)(thr - Roll_core_out - Pitch_core_out- Yaw_core_out);
					moto2=(int16_t)(thr + Roll_core_out - Pitch_core_out+ Yaw_core_out);       
					moto3=(int16_t)(thr + Roll_core_out + Pitch_core_out- Yaw_core_out);
					moto4=(int16_t)(thr - Roll_core_out + Pitch_core_out+ Yaw_core_out);  
						
				}
        else
        {
					moto1 = 1613;
					moto2 = 1613;
					moto3 = 1613;
					moto4 = 1613;
        }
        Motor_PWM_Update(moto1,moto2,moto3,moto4);
}


void ahrs_control_PID_moto(void)//关键字：航姿 控制 串级PID 电机
{
	if(lock==1)
		{

		  if(mpu_dmp_get_data(&Angle.pitch ,&Angle.roll,&Angle.yaw)==0)//得到欧拉角 
			{
				//MPU_Get_Accelerometer(&Acc.X ,&Acc.Y ,&Acc.Z );	//得到加速度传感器数据
        MPU_Get_Gyroscope(&Gyro.X ,&Gyro.Y ,&Gyro.Z);//读取陀螺仪数据
				MPU_Get_GYRO();//陀螺仪数据更新函数
				CONTROL_PID(Angle_SET.pitch,Angle_SET.roll,Angle_SET.yaw);//PID计算pwm值	
			  if(report) Data_Exchange();//航姿数据更新到匿名上位机
			}
				
		}
		else//飞机上锁
		{
			     Motor_PWM_Update(moto1-100,moto2-100,moto3-100,moto4-100);//飞控锁定，还没完善
		}
}














