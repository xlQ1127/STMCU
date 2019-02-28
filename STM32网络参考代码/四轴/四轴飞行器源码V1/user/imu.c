/***********************************************

标题: imu.c
作者: 秋阳电子
网址：http://qiuyangdz.taobao.com
日期: 2014/05/18
版本：v1.0
功能: 更新读取机体欧拉角
说明：
*************************************************/
#include "stm32f10x.h"
#include "imu.h"
#include <math.h>

#define sampleFreq	400.0f			// sample frequency in Hz
#define twoKpDef	(2.0f * 0.5f)	// 2 * proportional gain
#define twoKiDef	(0.5f * 0.0f)	// 2 * integral gain

volatile float twoKp = twoKpDef;											// 2 * proportional gain (Kp)
volatile float twoKi = twoKiDef;											// 2 * integral gain (Ki)

volatile float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f;					// quaternion of sensor frame relative to auxiliary frame

volatile float integralFBx = 0.0f,  integralFBy = 0.0f, integralFBz = 0.0f;	// integral error terms scaled by Ki

/*************************************************
名称：float invSqrt(float x) 
功能：平方根倒数
输入参数：输入值
输出参数：
返回值：平方根倒数
**************************************************/
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
/*************************************************

名称：get_euler_angle(float gx, float gy, float gz, float ax, float ay, float az, float *pitch, float *roll)
功能：获取欧拉角
输入参数：3轴陀螺仪 3轴加速度
输出参数：欧拉角
返回值：
**************************************************/
void get_euler_angle(float gx, float gy, float gz, float ax, float ay, float az, float *pitch, float *roll)
{
  float recipNorm;
  float halfvx, halfvy, halfvz;
  float halfex, halfey, halfez;
  float qa, qb, qc;

  if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) 
  {
    gx *= 0.017453f;  
    gy *= 0.017453f;
    gz *= 0.017453f;

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
	  integralFBx += twoKi * halfex * (1.0f / sampleFreq);	// integral error scaled by Ki
	  integralFBy += twoKi * halfey * (1.0f / sampleFreq);
	  integralFBz += twoKi * halfez * (1.0f / sampleFreq);
	  gx += integralFBx;	// apply integral feedback
	  gy += integralFBy;
	  gz += integralFBz;
	}
	else 
	{
	  integralFBx = 0.0f;	// prevent integral windup
	  integralFBy = 0.0f;
	  integralFBz = 0.0f;
	}

	// Apply proportional feedback
	gx += twoKp * halfex;
	gy += twoKp * halfey;
	gz += twoKp * halfez;
  }
	
  // Integrate rate of change of quaternion
  gx *= (0.5f * (1.0f / sampleFreq));		// pre-multiply common factors
  gy *= (0.5f * (1.0f / sampleFreq));
  gz *= (0.5f * (1.0f / sampleFreq));
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
 
  *pitch = -asin(-2 * q1 * q3 + 2 * q0 * q2) * 57.295780f;  
  *roll  =  atan2(2 * q2 * q3 + 2 * q0 * q1, -2 * q1 * q1 - 2 * q2 * q2 + 1) * 57.295780f; 
}
/***************************END OF FILE**********************************************************************/
