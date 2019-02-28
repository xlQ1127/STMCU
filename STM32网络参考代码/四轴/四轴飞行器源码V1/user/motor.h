#ifndef __motor_H
#define __motor_H

#define PWM_MOTOR_MIN 0
#define PWM_MOTOR_MED 1400
#define PWM_MOTOR_MAX 2500  

#define PWM_MOTOR1 TIM1->CCR3   
#define PWM_MOTOR2 TIM4->CCR3   
#define PWM_MOTOR3 TIM4->CCR4   
#define PWM_MOTOR4 TIM1->CCR2 

void motor_init(void);
void motor_control(s16 motor1, s16 motor2, s16 motor3, s16 motor4);
#endif
