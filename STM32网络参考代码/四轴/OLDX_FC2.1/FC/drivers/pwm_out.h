#ifndef _PWM_OUT_H_
#define _PWM_OUT_H_

#include "stm32f4xx.h"

u8 PWM_Out_Init(uint16_t hz);
u8 PWM_AUX_Out_Init(uint16_t hz);//50Hz
void SetPwm(int16_t pwm[],s16 min,s16 max);
void SetPwm_Fan(int16_t pwm[6]);
void SetPwm_AUX(float pit,float rol);

typedef struct 
{ u16 pwm_tem[2];
	int flag[2];
	u16 init[2];
	u16 max[2];
	u16 min[2];
	float att[2],att_ctrl[2],att_off[2];
	float pwm_per_dig[2];
	float ero[2],ero_reg[2];
}AUX_S;

extern AUX_S aux;

u8 cal_sita_form_pos(float r1,float r2,float x,float y,float z,float *sita1,float *sita2);
#endif

