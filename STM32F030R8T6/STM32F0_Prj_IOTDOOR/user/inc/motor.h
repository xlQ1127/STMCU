#ifndef MOTOR_H
#define MOTOR_H

#include "common.h"
#include "systemstatus.h"

#define MOTORDIR  0

#define MOTOR_ALOW    GPIOC->BSRR = GPIO_BSRR_BR_2
#define MOTOR_AHIGH   GPIOC->BSRR = GPIO_BSRR_BS_2

#define MOTOR_BLOW    GPIOC->BSRR = GPIO_BSRR_BR_3
#define MOTOR_BHIGH   GPIOC->BSRR = GPIO_BSRR_BS_3


#define MOTOR_ELECURDELAY      200
#define MOTOR_ELECUROVER       100        /////堵转电流大小
#define MOTOR_ELECUROVERCOUNT  5
#define MOTOR_MOTORFORCELIMIT  2625000    /////最大正延迟
#define MOTOR_RETURNHALFCOUNT  600        ////最大负延迟



void MotorBackward(void);
void MotorForward(void);
void MotorStop(void);
void DoorClose(SystemDatatypedef * q);
void DoorOpen(SystemDatatypedef * q);
#endif