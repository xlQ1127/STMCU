/*

1.0.0 STM32飞控基础框架，硬件IIC读取MPU6050，SPI连接NRF24L01

1.1.0 加入 四元数姿态融合，4路PWM ，定时器计算采样周期,IIC升级为连续读取

1.2.0 程序细节优化

1.3.0 加入PID控制马达，加入简单遥控：起落\启动 。

1.4.0 PID参数优化,加入传感器零点校准

1.5.0 加入HMC5883 磁阻传感器

1.6.0 加入简单按键控制方向：改变PID期望角度。

1.6.2 改进方向控制为：直接控制PWM。使用磁阻传感器计算Yaw角。

1.6.4 加强2.4G数据回传,加入新数据融合算法，避免加速度干扰。

1.7 新飞控PCB，新遥控器上线，双回路PID，

*/


#include "stm32f10x.h"
#include "stdlib.h"
#include "math.h"
#include "delay.h" 
#include "NRF24L01.H"
#include "MPU6050.H"
#include "IIC.H"
#include "spi.H"
#include "ALL_config.h"
#include "var_global.h"
#include "Cal.h"
#include "Mot_crtl.h"
#include "HMC5883.H"
#include "PID.H"
//-----------------------------------------------------------------//
u16 dir_time=301;
u8 RC_CON=0;


u16 TO=0;

//---------------------------------------------------------------------//
void Cal_Control(void);

int main()
{

RCC_Configuration();
delay_ms(50);
I2C_Configuration();
GPIO_Configuration();	
SPIx_Init();

NRF24L01Int();
NRFSetTxMode(TxDate);	
//NRFSetRXMode();

Tim2_init();

IIC_Reboot();
IIC_Reboot();

Init_MPU6050() ;	

//Get_GRY_Offset();
Set_Offset();	

TIM3_INT();	
TIM4_INT();	

//串级PID      P   I   D
PID_init_Eur (6, 0.00 ,0.6);  //主调

PID_init_Rate(3, 0,0.12); //副调（陀螺稳定器）  PID_init_Rate(5, 0 ,0.4); //副调（陀螺稳定器）

PID_Yaw_Rate(25,0,0.5);//航向角PID  P小了会自旋

while(1)
{
if(1)	
{
  GPIO_ResetBits(GPIOB,GPIO_Pin_0);
	Delay_s(1);
	GPIO_SetBits(GPIOB,GPIO_Pin_0);	
  Delay_s(8);

  if(RC_CON)
	{		   GPIO_ResetBits(GPIOB,GPIO_Pin_1);
	 Delay_s(1);
	 GPIO_SetBits(GPIOB,GPIO_Pin_1);	
   Delay_s(1);
	}
}
}
}
 
void TIM3_IRQHandler()
{TIM_ClearITPendingBit(TIM3,TIM_FLAG_Update); 

//Get_RFdata();	

READ_MPU6050();	
Cal_TsData();
//Cal_Control();

//IMUupdate(-GRY_F.X,-GRY_F.Y,GRY_F.Z,ACC_AVG.X*5,ACC_AVG.Y*5,ACC_AVG.Z*5);		
//sensfusion6UpdateQ(-GRY_F.X,-GRY_F.Y,GRY_F.Z,ACC_AVG.X*100,ACC_AVG.Y*100,ACC_AVG.Z*100,0.0005f);

MadgwickAHRSupdateIMU(GRY_F.Y,GRY_F.X, GRY_F.Z, ACC_AVG.Y, ACC_AVG.X, ACC_AVG.Z);
sensfusion6GetEulerRPY(&Q_ANGLE.Rool, &Q_ANGLE.Pitch, &Q_ANGLE.Yaw);

//PID_RPY_Rate();   //副调
//PID_Eur();       //主调
 
//Mot_output();

if(dir_time>200){RC_CON=0; FLY_Enable=0;TO++;} else  {dir_time++; RC_CON=1; FLY_Enable=1;}

Send_RFdata(); 
if(Peace>100 && FLY_Enable==0 ) {Get_GRY_Offset(); Peace=0; }
}

//------------------------------------------------------------------//
 void Cal_Control()
{  thr=RCun.RCdata.rc_thr*3;
  if (RCun.RCdata.rc_thr<10) FLY_Enable=0;  if (RCun.RxData[0]!=1 || RCun.RxData[4]!=2) {FLY_Enable=0; RC_CON=0;}
	EXP_ANGLE.Y=RCun.RCdata.rc_pit/4;
	EXP_ANGLE.X=-RCun.RCdata.rc_rol/4;
	EXP_ANGLE.Z=RCun.RCdata.rc_yaw;
}



