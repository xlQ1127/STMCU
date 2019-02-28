#include "systime.h"

#include "led.h"
#include "maincom.h"

#include "mpu6050.h"
#include "imu.h"
#include "control.h"
#include "rc.h"

volatile u32 time5_tick;//Time5计数器


s16 acc[3],gyro[3],mag[3];

void System_Time_Init(void)
{
	MPU6050_READ();//读取
	Get_Accel_Angle(MPU6050_ACC_LAST.X,MPU6050_ACC_LAST.Y,MPU6050_ACC_LAST.Z,&Q_ANGLE.ROLL,&Q_ANGLE.PITCH);//加速度算出的角度为初始角度
	TIM5_Config();
}

/*
 * 函数名：TIM5_Config
 * 描述  ：TIM5配置 NVIC中断配置 2ms中断一次
 * 输入  ：无
 * 输出  ：无
 * 调用  ：外部调用	
 */
void TIM5_Config(void)
{
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

	//设置NVIC中TIM5中断
	NVIC_InitStructure.NVIC_IRQChannel = TIM5_IRQn;//通道设置为TIM5
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;//抢占1
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;//响应1
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;//打开TIM5中断通道
	NVIC_Init(&NVIC_InitStructure);//写入配置
	
	//设置TIM5
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5,ENABLE);//开启TIM5时钟
  //TIM_DeInit(TIM2);//TIM2初始化为缺省值

	TIM_TimeBaseStructure.TIM_Period=2000;//设定计数器自动重装值
	TIM_TimeBaseStructure.TIM_Prescaler= (72 - 1);//时钟预分频数 72M/72
  TIM_TimeBaseStructure.TIM_ClockDivision=TIM_CKD_DIV1;//采样分频 TDTS = Tck_tim
  TIM_TimeBaseStructure.TIM_CounterMode=TIM_CounterMode_Up;//向上计数模式
  TIM_TimeBaseInit(TIM5,&TIM_TimeBaseStructure);//配置TIM5
    
	TIM_ClearFlag(TIM5,TIM_FLAG_Update);//清除溢出中断标志
  TIM_ITConfig(TIM5,TIM_IT_Update,ENABLE);//开启溢出中断
	TIM_Cmd(TIM5,ENABLE);//开启TIM5外设
    //RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2,DISABLE);//先关闭TIM2时钟 等待使用
		
}

/*
 * 函数名：TIM5_Start
 * 描述  ：开启TIM5
 * 输入  ：无
 * 输出  ：无
 * 调用  ：外部调用	
 */
void TIM5_Start(void)
{
	time5_tick = 0;
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5,ENABLE);
	TIM_Cmd(TIM5,ENABLE);
}

/*
 * 函数名：TIM5_Stop
 * 描述  ：关闭TIM5
 * 输入  ：无
 * 输出  ：无
 * 调用  ：外部调用	
 */
void TIM5_Stop(void)
{
	TIM_Cmd(TIM5,DISABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5,DISABLE);
}

//*****TIM5中断函数2ms*****//
void TIM5_IRQHandler(void)
{
	if(TIM_GetITStatus(TIM5,TIM_IT_Update) == SET)//检测TIM5溢出中断是否发生
	{
		TIM_ClearITPendingBit(TIM5,TIM_IT_Update);
		
		time5_tick++;
		time5_tick = time5_tick%1000;//2HZ
		
		//LED0(ON);
		//START////////////////////数据读取+姿态解算+PID控制 1.4ms////////////////////////////
		
		MPU6050_READ();//读取
	
		acc[0] = MPU6050_ACC_LAST.X;
		acc[1] = MPU6050_ACC_LAST.Y;
		acc[2] = MPU6050_ACC_LAST.Z;
		
		gyro[0] = MPU6050_GYRO_LAST.X - GYRO_OFFSET.X;
		gyro[1] = -(MPU6050_GYRO_LAST.Y - GYRO_OFFSET.Y);
		gyro[2] = MPU6050_GYRO_LAST.Z - GYRO_OFFSET.Z;
		if(gyro[2]>=-5 &&gyro[2]<=5) gyro[2] = 0;//Z轴陀螺仪,漂移处理
		
		
		IMUupdate(gyro[0],gyro[1],gyro[2],acc[0],acc[1],acc[2]);//非全姿态角解算
		//IMUupdate1(number_to_dps1(gyro[0]),number_to_dps1(gyro[1]),number_to_dps1(gyro[2]),acc[0],acc[1],acc[2]);
		//IMUupdate2(number_to_dps1(gyro[0]),number_to_dps1(gyro[1]),number_to_dps1(gyro[2]),acc[0],acc[1],acc[2]);
		
		CONTROL(Q_ANGLE.ROLL,Q_ANGLE.PITCH,Q_ANGLE.YAW,Value_2_Thr(),Value_2_Roll(),Value_2_Pitch(),Vaule_2_Gyro());//对电机进行PID控制
		
		//END///////////////////////////////////////////////////////////////////////
		//LED0(OFF);
		
		
		if(time5_tick%5 == 0)//10ms  --  100HZ进行一次
		{
			
		}
		
		if(time5_tick%25 == 0)//50ms	--	20HZ进行一次
		{
			DMA_DATA_SEND_FLAG = 1;//上传数据
		}
		if(time5_tick%50 == 0)//100ms  --  10HZ进行一次
		{
			
		}
		if(time5_tick%500 == 0)//1000ms  --  1HZ进行一次
		{
			if(ARMED == 1)
			{
				Is_DisArmed(RC_CH[3-1],RC_CH[4-1]);//是否加锁
			}
			else
			{
				Is_Armed(RC_CH[3-1],RC_CH[4-1]);//是否解锁
			}
		}
	}		
}










