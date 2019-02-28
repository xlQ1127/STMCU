#include "stm32f10x.h"
#include "SysTick.h"
#include "led.h"
#include "dma.h"
#include "myiic.h"
#include "stmflash.h"

#include "systime.h"
#include "mpu6050.h"
#include "imu.h"
#include "control.h"
#include "rc.h"
#include "motor.h"
#include "maincom.h"
#include "refreshled.h"




void FlyControlAdjust(void)
{
	u8 x_mode;
	Data_Send_PID(PID_ROL.P,PID_ROL.I,PID_ROL.D,PID_PIT.P,PID_PIT.I,PID_PIT.D,PID_YAW.P,PID_YAW.I,PID_YAW.D);//发送PID数据
	while(1)
	{
		//判读是否要解锁(遥控器控制)
		if(Is_Armed(RC_CH[3-1],RC_CH[4-1]) == 1) break;
	
		//(上位机控制)
		x_mode = Recv_Command();
		if(x_mode == 1)//遥控器通道校准
		{
			Channel_Adjust();
		}
		else if(x_mode == 2)//给上位机发送PID数据
		{
			PID_ReadFlash();
			Data_Send_PID(PID_ROL.P,PID_ROL.I,PID_ROL.D,PID_PIT.P,PID_PIT.I,PID_PIT.D,PID_YAW.P,PID_YAW.I,PID_YAW.D);//发送PID数据
		}
		else if(x_mode == 3)//上位机给飞控发送PID数据
		{
			Get_Recv_PID();//保存PID值到RAM
			PID_WriteFlash();//保存PID值到Flash
		}
		
		LED0(ON);
		delay_ms(10);
		LED0(OFF);
		delay_ms(100);
	
	
		//判断是否进行加速度计校准
		//？？？？？？？？？？？？？
	}
	
}

void FlyControlInit(void)
{
	//RC通道初始化
	if(Channel_Config() == 0)
	{
		while(1)
		{
			LED0(ON);
			delay_ms(500);
			LED0(OFF);
			delay_ms(500);
		}
	}
	
	//ACC_SET_OFFSET();//加速度计水平校准
	GYRO_SET_OFFSET();//陀螺仪零偏校准
	Data_Send_Offset(ACC_OFFSET.X,ACC_OFFSET.Y,ACC_OFFSET.Z,GYRO_OFFSET.X,GYRO_OFFSET.Y,GYRO_OFFSET.Z);//发送加速度计 陀螺仪偏移值
}

void setup(void)// 初始化设置函数
{
	
	/* Setup STM32 system (clock, PLL and Flash configuration) */
	SystemInit();
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);//设置优先级分组 2

	LED_GPIO_Config();//配置LED到IO口
	Delay_Init(72);//延时函数初始化
	USART1_Config(256000);//USART1初始化
	I2C_Init_IO();
	
	delay_ms(1000);
	
	
	MPU6050_INIT();//MPU6050初始化
	MPU6050_INIT();
	PID_Init();//PID数值初始化
	Motor_Init(400);//PWM输出初始化
	RC_Init();//遥控器接收初始化(遥控器数值放在RC_CH[X]数组中)
	
	
	FlyControlAdjust();//判读是否需要进行相应的校准
	
	//--------------------执行到此处说明已经解锁！--------------------
	Led_Refresh_Init();//LED闪烁中断开启
	FlyControlInit();//飞控系统初始化(遥控器通道初始化，水平位置初始化，陀螺仪零偏初始化)
	
	System_Time_Init();
}

int main(void)
{
	setup();
	
	while(1)
	{
		//------------------------------------串口1 DMA发送数据------------------------------------
		if(DMA_DATA_SEND_FLAG == 1)
		{		
			
			//START//////////////////数据上传 4ms////////////////////////////////////////
			
			Data_Send_Attitude(acc,gyro,mag,-Q_ANGLE.ROLL,Q_ANGLE.PITCH,Q_ANGLE.YAW);//上传姿态数据
			Data_Send_Control(RC_CH,(moto1-1000)/10,(moto2-1000)/10,(moto3-1000)/10,(moto4-1000)/10,512);//上传电机、遥控、电压数据
			
			//END////////////////////////////////////////////////////////////////////////
			
			DMA_DATA_SEND_FLAG = 0;
		}
		
		//------------------------------------判断并改变灯闪烁的方式------------------------------------
		if(ARMED == 1)//已经解锁
		{
			led_state = 0;//LED 1次慢闪+2次快闪
		}
		if(ARMED == 0)
		{
			led_state = 2;//LED 1次慢闪
		}
		
	}
}



