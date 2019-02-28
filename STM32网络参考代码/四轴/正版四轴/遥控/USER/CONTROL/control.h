#ifndef _CONTROL_H_
#define _CONTROL_H_



typedef struct PID{float P,pout,I,iout,D,dout,IMAX,OUT;}PID;

extern PID PID_ROL,PID_PIT,PID_YAW;

extern int accelerator;
extern int Pitch_ta;
extern int Roll_ta;
extern int Yaw_ta;

void PID_init(void);
void CONTROL(float rol_now,float pit_now,float rol_tar,float pit_tar,int X_w,int Y_w,int throttle);

#endif
