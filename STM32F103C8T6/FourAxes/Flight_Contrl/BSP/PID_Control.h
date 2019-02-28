#ifndef _CONTROL_H_
#define _CONTROL_H_

#include "main.h"
#include "stm32f1xx_hal.h"


typedef struct PID{
float P,pout,
      I,iout,
      D,dout,
      IMAX,OUT;
}PID;


#define Gyro_Gr	0.0010653				  //角速度变成弧度	此参数对应陀螺2000度每秒

void PID_init(float Roll_P,float Roll_I,float Roll_D,float Pitch_P,float Pitch_I,float Pitch_D);

void PID_CONTROL(float rol_now, float pit_now, float yaw_now, 
                 float rol_tar, float pit_tar, float yaw_tar, 
                 float X_w, float Y_w, float Z_w,
                 __IO float throttle, __IO int8_t Control_Singnal);//角速度

void PWM_Set(TIM_HandleTypeDef *htim,uint16_t Motor1,uint16_t Motor2,uint16_t Motor3,uint16_t Motor4);

#endif
