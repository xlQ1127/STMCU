#ifndef __pid_H
#define __pid_H

typedef struct 
{
  float kp;
  float ki;
  float kd;

  float err0;
  float err1;
  float err2;
  float sum_err;
  float zero_err;
  float output;
  float ki_max;
  float ki_min;
  float out_max;
  float out_min;
}pid_struct;

void pid_config(void);
void calculate(float pitch, float roll, float yaw, u8 throttle, u8 rudder, u8 elevator, u8 aileron, float gx, float gy, float gz);
#endif
