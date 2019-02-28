#include "stm32f10x.h"

#include "math.h"
#include "var_global.h"
#include "cal.h"                              
#define Gyr_Gain 	0.015267       //角速度500度秒                           
#define ACC_Gain 	0.0011963 
#define AVGtimes  1

u16 Data_time;

u8 AVG_cnt;

float_XYZ ACC_F,GRY_F;

float_XYZ ACC_AVG;

int16_XYZ ACC_Offset,GRY_Offset;

float ACC_X_BUF[AVGtimes],ACC_Y_BUF[AVGtimes],ACC_Z_BUF[AVGtimes];

float time;

float AVGtempX,AVGtempY,AVGtempZ; 

void Cal_TsData() //数据准备已及换算
{ 
	
	u8 i;
	
  
	Data_time=TIM4->CNT;
	TIM4->CNT=0;
	
	time=(float)Data_time/800000;
//--------------数据进行单位转换------------------------//		
  
	ACC_F.X=ACC_RealData.X*ACC_Gain;
  ACC_F.Y=ACC_RealData.Y*ACC_Gain;
	ACC_F.Z=ACC_RealData.Z*ACC_Gain;

  GRY_F.X=GRY_RealData.X*Gyr_Gain;
	GRY_F.Y=GRY_RealData.Y*Gyr_Gain;
	GRY_F.Z=GRY_RealData.Z*Gyr_Gain;

//--------------加速度平均------------------------//	
	ACC_X_BUF[AVG_cnt] = ACC_F.X;
	ACC_Y_BUF[AVG_cnt] = ACC_F.Y;
	ACC_Z_BUF[AVG_cnt] = ACC_F.Z;
	
	for(i=0;i<AVGtimes;i++)
	{
		AVGtempX += ACC_X_BUF[i];
		AVGtempY += ACC_Y_BUF[i];
		AVGtempZ += ACC_Z_BUF[i];
	}
        
	ACC_AVG.X = AVGtempX / AVGtimes;
	ACC_AVG.Y = AVGtempY / AVGtimes;
	ACC_AVG.Z = AVGtempZ / AVGtimes;  // ACC_AVG 加速度平均值
	
	AVGtempX=0;
	AVGtempY=0;
	AVGtempZ=0;
	
	AVG_cnt++;
	if(AVG_cnt>=AVGtimes)	AVG_cnt=0;	
	
}

#define Kp 3.0f   // 比例增益支配率收敛到加速度计                     
#define Ki 0.001f     // 积分增益支配率的陀螺仪偏见的衔接                    

float halfT;     // 采样周期的一半  

float q0=1, q1=0, q2=0, q3=0;   // 四元数的元素，代表估计方向  
float exInt=0,eyInt=0,ezInt=0;  // 按比例缩小积分误差

float_RPY Q_ANGLE;



//四元数卡尔曼计算
void IMUupdate(float gx, float gy, float gz, float ax, float ay, float az)
{ 
  float norm;
  float vx, vy, vz;// wx, wy, wz;
  float ex, ey, ez; 
  float q0q0 = q0*q0;
  float q0q1 = q0*q1;
  float q0q2 = q0*q2;
//float q0q3 = q0*q3;
  float q1q1 = q1*q1;
//float q1q2 = q1*q2;
  float q1q3 = q1*q3;
  float q2q2 = q2*q2;
  float q2q3 = q2*q3;
  float q3q3 = q3*q3;

	halfT=(float)time; 	
	
	gx*=0.0174;gy*=0.0174;gz*=0.0174;
  // 单位化四元数 取模
  norm = sqrtf(ax*ax + ay*ay + az*az);      
  ax = ax /norm;
  ay = ay / norm;
  az = az / norm;
   // 估计方向的重力       
  vx = 2*(q1q3 - q0q2);												
  vy = 2*(q0q1 + q2q3);
  vz = q0q0 - q1q1 - q2q2 + q3q3 ;
 // 错误的领域和方向传感器测量参考方向之间的交叉乘积的总和
  ex = (ay*vz - az*vy) ;                           					
  ey = (az*vx - ax*vz) ;
  ez = (ax*vy - ay*vx) ;
// 积分误差比例积分增益
  exInt = exInt + ex * Ki;								 
  eyInt = eyInt + ey * Ki;
  ezInt = ezInt + ez * Ki;  
 
  // 调整后的陀螺仪测量
  gx = gx + Kp*ex + exInt;					   							
  gy = gy + Kp*ey + eyInt;
  gz = gz + Kp*ez + ezInt;				   							
  
  // 整合四元数率和正常化					   
  q0 = q0 + (-q1*gx - q2*gy - q3*gz)*halfT;
  q1 = q1 + (q0*gx + q2*gz - q3*gy)*halfT;
  q2 = q2 + (q0*gy - q1*gz + q3*gx)*halfT;
  q3 = q3 + (q0*gz + q1*gy - q2*gx)*halfT;
  // 正常化四元数
  norm = sqrt(q0*q0 + q1*q1 + q2*q2 + q3*q3);
  q0 = q0 / norm;
  q1 = q1 / norm;
  q2 = q2 / norm;
  q3 = q3 / norm;
  
  //四元数与欧拉角转换公式
  Q_ANGLE.Pitch  = asin(-2 * q1 * q3 + 2 * q0* q2)* 57.3;
  Q_ANGLE.Rool = atan2(2 * q2 * q3 + 2 * q0 * q1, -2 * q1 * q1 - 2 * q2* q2 + 1)* 57.3; 
  Q_ANGLE.Yaw  = atan2(2 * q2 * q1 + 2 * q0 * q3, -2 * q3 * q3 - 2 * q2* q2 + 1)* 57.3; 
}



