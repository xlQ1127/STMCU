#include "stm32f10x.h"

#include "math.h"
#include "var_global.h"
#include "cal.h"                              
//#define Gyr_Gain 	0.015267       //角速度500度秒 
//#define Gyr_Gain 	0.061       //角速度2000度秒 

#define Gyr_Gain 	0.0305f  

//#define Gyr_Gain 	0.061f   //2000度秒

#define ACC_Gain 	0.0011963f 
#define AVGtimes  10
float invSqrt(float x);
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
	
	time=(float)Data_time/1000000;
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

#define Kp 1.0f   // 比例增益支配率收敛到加速度计                     
#define Ki 0.00005f     // 积分增益支配率的陀螺仪偏见的衔接                    

float halfT;     // 采样周期的一半  

float q0=1, q1=0, q2=0, q3=0;   // 四元数的元素，代表估计方向  
float exInt=0,eyInt=0,ezInt=0;  // 按比例缩小积分误差

float_RPY Q_ANGLE;



//四元数
void IMUupdate(float gx, float gy, float gz, float ax, float ay, float az)
{ 
	
	float qq0=0, qq1=0, qq2=0, qq3=0;  

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

  qq0=q0;qq1=q1;qq2=q2;qq3=q3;

  q0 = qq0 + (-qq1*gx - qq2*gy - qq3*gz)*halfT;
  q1 = qq1 + (qq0*gx + qq2*gz - qq3*gy)*halfT;
  q2 = qq2 + (qq0*gy - qq1*gz + qq3*gx)*halfT;
  q3 = qq3 + (qq0*gz + qq1*gy - qq2*gx)*halfT;
	

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


//New IMU [Bicraze]-----------------------------------------------------------
#define TWO_KP_DEF  (2.0f * 6.0f) // 2 * proportional gain  3.0
//#define TWO_KI_DEF  (2.0f * 0.000005f) // 2 * integral gain  //大了会造成大动作横飞
#define TWO_KI_DEF  (2.0f * 0.000000f) // 2 * integral gain  //大了会造成大动作横飞
  
  float twoKp = TWO_KP_DEF;    // 2 * proportional gain (Kp)
  float twoKi = TWO_KI_DEF;    // 2 * integral gain (Ki)
  float integralFBx = 0.0f;
  float integralFBy = 0.0f;
  float integralFBz = 0.0f;  // integral error terms scaled by Ki


void sensfusion6UpdateQ(float gx, float gy, float gz, float ax, float ay, float az, float dt)
{ //注：dt单位为100us
  float recipNorm;
  float halfvx, halfvy, halfvz;
  float halfex, halfey, halfez;
  float qa, qb, qc;

  gx = gx * 0.174;
  gy = gy * 0.174;
  gz = gz * 0.174;

  // Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
  if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f)))
  {
    // Normalise accelerometer measurement
    recipNorm = invSqrt(ax * ax + ay * ay + az * az);
    ax *= recipNorm;
    ay *= recipNorm;
    az *= recipNorm;

    // Estimated direction of gravity and vector perpendicular to magnetic flux
    halfvx = q1 * q3 - q0 * q2;
    halfvy = q0 * q1 + q2 * q3;
    halfvz = q0 * q0 - 0.5f + q3 * q3;

    // Error is sum of cross product between estimated and measured direction of gravity
    halfex = (ay * halfvz - az * halfvy);
    halfey = (az * halfvx - ax * halfvz);
    halfez = (ax * halfvy - ay * halfvx);

    // Compute and apply integral feedback if enabled
    if(twoKi > 0.0f)
    {
      integralFBx += twoKi * halfex * dt;  // integral error scaled by Ki
      integralFBy += twoKi * halfey * dt;
      integralFBz += twoKi * halfez * dt;
      gx += integralFBx;  // apply integral feedback
      gy += integralFBy;
      gz += integralFBz;
    }
    else
    {
      integralFBx = 0.0f; // prevent integral windup
      integralFBy = 0.0f;
      integralFBz = 0.0f;
    }

    // Apply proportional feedback
    gx += twoKp * halfex;
    gy += twoKp * halfey;
    gz += twoKp * halfez;
  }

  // Integrate rate of change of quaternion
  gx *= (0.5f * dt);   // pre-multiply common factors
  gy *= (0.5f * dt);
  gz *= (0.5f * dt);
  qa = q0;
  qb = q1;
  qc = q2;
  q0 += (-qb * gx - qc * gy - q3 * gz);
  q1 += (qa * gx + qc * gz - q3 * gy);
  q2 += (qa * gy - qb * gz + q3 * gx);
  q3 += (qa * gz + qb * gy - qc * gx);

  // Normalise quaternion
  recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
  q0 *= recipNorm;
  q1 *= recipNorm;
  q2 *= recipNorm;
  q3 *= recipNorm;
}


void sensfusion6GetEulerRPY(float* roll, float* pitch, float* yaw)
{
  float gx, gy, gz; // estimated gravity direction

  gx = 2 * (q1*q3 - q0*q2);
  gy = 2 * (q0*q1 + q2*q3);
  gz = q0*q0 - q1*q1 - q2*q2 + q3*q3;

  *yaw = atan2(2*q1*q2 - 2*q0*q3, 2*q0*q0 + 2*q1*q1 - 1) * 57.3;
  *pitch = atan(gx / sqrt(gy*gy + gz*gz)) * 57.3;
  *roll = atan(gy / sqrt(gx*gx + gz*gz)) * 57.3;
}

//---------------------------------------------------------------------------------------------------
// Fast inverse square-root
// See: http://en.wikipedia.org/wiki/Fast_inverse_square_root
float invSqrt(float x)
{
  float halfx = 0.5f * x;
  float y = x;
  long i = *(long*)&y;
  i = 0x5f3759df - (i>>1);
  y = *(float*)&i;
  y = y * (1.5f - (halfx * y * y));
  return y;
}


