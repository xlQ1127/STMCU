#ifndef _CONTROL_H
#define _CONTROL_H
#include "sys.h"
#include "mpu6050.h"
#include "pwm_out.h" 
#include <math.h>
#include "usart.h"
extern u8        lock;   //飞控上锁标志位
extern int16_t moto1,moto2,moto3,moto4;

typedef struct
{
	float pitch;//期望姿态
	float roll;
	float yaw;
	float THROTTLE;//油门
}T_RC_Dat;//遥控器接收

extern T_RC_Dat  Rc_D;//遥控通道数据;
//外环PID参数
extern float Pitch_shell_kp;
extern float Pitch_shell_kd;
extern float Pitch_shell_ki;
/*******************************/
extern float Roll_shell_kp;
extern float Roll_shell_kd;            
extern float Roll_shell_ki;
/*******************************/
extern float Yaw_shell_kp;
extern float Yaw_shell_kd;      
extern float Yaw_shell_ki;
//内环PD参数

extern float Pitch_core_kp;
extern float Pitch_core_ki;
extern float Pitch_core_kd;

extern float Roll_core_kp;
extern float Roll_core_ki;
extern float Roll_core_kd;

extern float Yaw_core_kp;
extern float Yaw_core_ki;
extern float Yaw_core_kd;
extern float RC_Pitch,RC_Roll,RC_Yaw;           //期望的姿态角
extern S_INT16_XYZ MPU6050_GYRO_LAST;
void CONTROL_PID(float pit, float rol, float yaw);
void ahrs_control_PID_moto(void);//关键字：航姿 控制 串级PID 电机
#endif




