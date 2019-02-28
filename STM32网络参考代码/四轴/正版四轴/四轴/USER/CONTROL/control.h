#ifndef _CONTROL_H_
#define _CONTROL_H_

#include "stm32f10x.h"

typedef struct PID{float P,pout,I,iout,D,dout,IMAX,OUT;}PID;

extern PID PID_ROL,PID_PIT,PID_YAW;

extern vs16 accelerator;
extern int  Pitch_ta;
extern int  Roll_ta;
extern int  Yaw_ta;

extern unsigned int  POWER;

void PID_init(void);
void CONTROL(float rol_now, float pit_now, float yaw_now, float rol_tar, float pit_tar, float yaw_tar, vs16 throttle, float X_w, float Y_w, float Z_w);

#endif
