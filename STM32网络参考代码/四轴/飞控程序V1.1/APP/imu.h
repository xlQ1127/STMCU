#ifndef __IMU_H_
#define __IMU_H_

#include "stm32f10x.h"

#define Gyro_Gain 		2000/32767;		//陀螺仪量程：+-2000度每秒
#define Gyro_GainR 		0.0010653;		//陀螺仪量程：+-2000弧度每秒
#define Acc_Gain 			2/32767;			//加速度计量程：+-2g

#define DT 	0.002f					//每次滤波的时间片
#define FILTER_A  0.9983f //一阶互补滤波的滤波系数
#define FILTER_K  0.45f//二阶互补滤波的滤波系数

//定义系统的欧拉角
typedef struct
{
	float ROLL;
	float PITCH;
	float YAW;
}ANGLE;

extern ANGLE Q_ANGLE;

float number_to_dps(s16 number);
float number_to_dps1(s16 number);
float number_to_g(s16 number);

void Get_Accel_Angle(s16 x,s16 y,s16 z,float* roll,float* pitch);//使用加速度计数据计算欧拉角
void IMUupdate(s16 gx, s16 gy, s16 gz, s16 ax, s16 ay, s16 az);//数据融合计算欧拉角

void IMUupdate1(float gx, float gy, float gz, float ax, float ay, float az);//陀螺仪 加速度计  四元数姿态解算
void IMUupdate2(float gx, float gy, float gz, float ax, float ay, float az);//陀螺仪 四元数进行姿态解算







#endif


