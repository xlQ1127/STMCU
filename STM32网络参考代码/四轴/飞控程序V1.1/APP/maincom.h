#ifndef __MAINCOM_H__
#define __MAINCOM_H__

#include "SysTick.h"
//#include <stdarg.h>
#include <stdio.h>

extern u8 DMA_DATA_SEND_FLAG;
extern u8 Receive_Complete;
extern u8 recv_dat[32];

void USART1_Config(u32 speed);//USART1初始化
int fputc(int ch, FILE *f);//重定向c库函数printf到USART1

void USART1_Send1Char(u8 c);

void Data_Send_Attitude(short* acc,short* gyro,short* mag,float rool,float pitch,float yaw);//发送姿态角和传感器数据
void Data_Send_Control(u16 *rc_ch,u16 PWM1,u16 PWM2,u16 PWM3,u16 PWM4,u16 votage);//发送遥控数据 和 电机PWM数据
//发送偏移数据
void Data_Send_Offset(s16 acc_offset_x,s16 acc_offset_y,s16 acc_offset_z,s16 gyro_offset_x,s16 gyro_offset_y,s16 gyro_offset_z);
//发送PID数据
void Data_Send_PID(float Roll_P,float Roll_I,float Roll_D,
									 float Pitch_P,float Pitch_I,float Pitch_D,
								   float Yaw_P,float Yaw_I,float Yaw_D);

u8 Recv_Command(void);


//从上位机获取PID值
void Get_Recv_PID(void);
#endif


