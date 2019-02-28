#ifndef __CONTROL_H__
#define __CONTROL_H__

#include "stm32f10x.h"


extern u16 moto1,moto2,moto3,moto4;

//定义PID
typedef struct
{
	float P;
	float pout;
	
	float I;
	float IMAX;
	float iout;
	
	float D;
	float dout;
	
	float OUT;
}PID;

extern u8 ARMED;
extern PID PID_ROL,PID_PIT,PID_YAW;


void PID_Init(void);

void CONTROL(float rol_now, float pit_now, float yaw_now, u16 throttle, float rol_tar, float pit_tar, s16 yaw_gyro_tar);


u8 Is_Armed(u16 CH3,u16 CH4);//油门最小 CH4最小
u8 Is_DisArmed(u16 CH3,u16 CH4);//油门最小 CH4最大

void PID_WriteFlash(void);//写入PID值
void PID_ReadFlash(void);//读取Flash里的的PID值

#endif


