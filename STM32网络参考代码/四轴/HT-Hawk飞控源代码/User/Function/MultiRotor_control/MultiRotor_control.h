#ifndef _MultiRotor_control_H_
#define _MultiRotor_control_H_
#include "stm32f10x.h"
#include "include.h"

struct _pid{
          float kp;
		  float ki;
	      float kd;
	      float increment;
	      float increment_max;
	      float kp_out;
		  float ki_out;
	      float kd_out;
	      float pid_out;
          };

struct _tache{
    struct _pid shell;
    struct _pid core;	
          };
	
struct _ctrl{
		      u8  ctrlRate;
        struct _tache pitch;    
	    struct _tache roll;  
	    struct _tache yaw;   
            };

struct _target{
        float Pitch;    
	    float Roll;  
	    float Yaw;   
        int32_t Altiude; 
            };

extern struct _ctrl ctrl;
extern struct _target Target;

void Calculate_Target(void);
void CONTROL(struct _target Goal);
void Attitude_RatePID(void);
void Motor_Conter(void);
void Reset_Integral(void);
#endif
