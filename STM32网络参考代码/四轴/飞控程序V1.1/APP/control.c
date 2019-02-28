#include "control.h"
#include "SysTick.h"
#include "motor.h"
#include "systime.h"//为了包含gyro acc mag
#include "imu.h"
#include "math.h"
#include "stmflash.h"


u16 moto1,moto2,moto3,moto4;

u8 ARMED;
PID PID_ROL,PID_PIT,PID_YAW;


void PID_Init(void)
{
	//P值
	PID_ROL.P = 4.15;
	PID_PIT.P = PID_ROL.P;
	PID_YAW.P = 4.5;
	
	//I值
	PID_ROL.I = 0.008;
	PID_PIT.I = PID_ROL.I;
	PID_YAW.I = PID_ROL.I;
	
	//IMAX
	PID_ROL.IMAX = 25;
	PID_PIT.IMAX = PID_ROL.IMAX;
	PID_YAW.IMAX = PID_ROL.IMAX;
	
	//D值
	PID_ROL.D = 1.1;
	PID_PIT.D = PID_ROL.D;
	PID_YAW.D = PID_ROL.D;
	
	PID_ReadFlash();
}

float Get_MxMi(float num,float max,float min)
{
	if(num>max)
		return max;
	else if(num<min)
		return min;
	else
		return num;
}

void CONTROL(float rol_now, float pit_now, float yaw_now, u16 throttle, float rol_tar, float pit_tar, s16 yaw_gyro_tar)
{
	float rol_error = rol_now - rol_tar;//计算角度差值
	float pit_error = pit_now - pit_tar;
	
	float yaw_error = yaw_now - (yaw_gyro_tar/12);//YAW差值（yaw_gyro_tar/12 = 0-30）
	float yaw_gyro_now = number_to_dps(gyro[2]);//YAW轴的当前角速度
	
	
	//-----------------------Pitch\Roll的PID控制-START-----------------------------
	PID_ROL.pout = PID_ROL.P * rol_error; //输出的P值
	PID_PIT.pout = PID_PIT.P * pit_error;
	
	//输出的I值
	PID_ROL.iout += PID_ROL.I * rol_error;
	PID_ROL.iout = Get_MxMi(PID_ROL.iout,PID_ROL.IMAX,-PID_ROL.IMAX);//判读I是否超出范围
	PID_PIT.iout += PID_PIT.I * pit_error;
	PID_PIT.iout = Get_MxMi(PID_PIT.iout,PID_PIT.IMAX,-PID_PIT.IMAX);
	if(throttle<1100)//当油门小于1100时，I值清零
	{
		PID_ROL.iout = 0;
		PID_PIT.iout = 0;
	}

	PID_ROL.dout = PID_ROL.D * number_to_dps(gyro[0]);//输出的D值（与遥控器的输入无关）
	PID_PIT.dout = PID_PIT.D * number_to_dps(gyro[1]);
	//-----------------------Pitch\Roll的PID控制-END-------------------------------
	
	
	//-------------------YAW的PID控制-START-----------------------------
	if(yaw_gyro_tar>=-5 && yaw_gyro_tar<= 5)//YAW摇杆死区控制
	{
		yaw_gyro_tar = 0;
	}
	else
	{
		Q_ANGLE.YAW = 0;//YAW角度清零
	}
	
	PID_YAW.pout = PID_YAW.P * yaw_error;//P
	PID_YAW.iout = 0;//I
	PID_YAW.dout = PID_YAW.D * (yaw_gyro_now-yaw_gyro_tar);//D 防止YAW轴转动
	//---------------------YAW的PID控制-END-----------------------------
	
	
	PID_ROL.OUT = PID_ROL.pout + PID_ROL.iout + PID_ROL.dout;////PID值相加
	PID_PIT.OUT = PID_PIT.pout + PID_PIT.iout + PID_PIT.dout;
	PID_YAW.OUT = PID_YAW.pout + PID_YAW.iout + PID_YAW.dout;
	
	
	if(ARMED == 1)
	{
		if(throttle>=1100)//对油门进行进行判断<1100的油门电机不转动！！！！！(为了安全很重要！！！)
		{
			moto1 = throttle + PID_ROL.OUT - PID_PIT.OUT + PID_YAW.OUT;  moto1 = Get_MxMi(moto1,2000,1000);
			moto2 = throttle - PID_ROL.OUT - PID_PIT.OUT - PID_YAW.OUT;  moto2 = Get_MxMi(moto2,2000,1000);
			moto3 = throttle - PID_ROL.OUT + PID_PIT.OUT + PID_YAW.OUT;  moto3 = Get_MxMi(moto3,2000,1000);
			moto4 = throttle + PID_ROL.OUT + PID_PIT.OUT - PID_YAW.OUT;  moto4 = Get_MxMi(moto4,2000,1000);
			
			Set_Motor(moto1,moto2,moto3,moto4);//电机PWM输出
		}
		else
		{
			moto1 =1000;moto2 =1000;moto3 =1000;moto4 =1000;
			Set_Motor(moto1,moto2,moto3,moto4);//电机PWM输出
		}
	}
	else
	{
		moto1 =1000;moto2 =1000;moto3 =1000;moto4 =1000;
		Set_Motor(moto1,moto2,moto3,moto4);//电机PWM输出
	}
	
}




//解锁
u8 Is_Armed(u16 CH3,u16 CH4)//油门最小 CH4最小
{
	if( (CH3<1100 && CH3>900) && (CH4<2100 && CH4>1900) )//是否进入解锁程序
	{
		ARMED = 1;
		return 1;//解锁成功
	}
	else 
	{
		return 0;//没有解锁
	}
}

//加锁
u8 Is_DisArmed(u16 CH3,u16 CH4)//油门最小 CH4最大
{
	if( (CH3<1100 && CH3>900) && (CH4<1100 && CH4>900) )
	{
		ARMED = 0;
		return 1;//加锁成功
	}
	else
	{
		return 0;//没有加锁
	}
}


//写入PID值
void PID_WriteFlash(void)
{
	u8 point=29;
	u16 flash_data[39];
	
	//读出Flash数据
	STMFLASH_Read(PAGE1,flash_data,39);
	
	//数据重新写入PID数据！
	flash_data[point++] = ADJUST_FLAG;
	
	flash_data[point++] = (u16)(PID_ROL.P*1000);
	flash_data[point++] = (u16)(PID_ROL.I*1000);
	flash_data[point++] = (u16)(PID_ROL.D*1000);
	
	flash_data[point++] = (u16)(PID_PIT.P*1000);
	flash_data[point++] = (u16)(PID_PIT.I*1000);
	flash_data[point++] = (u16)(PID_PIT.D*1000);
	
	flash_data[point++] = (u16)(PID_YAW.P*1000);
	flash_data[point++] = (u16)(PID_YAW.I*1000);
	flash_data[point++] = (u16)(PID_YAW.D*1000);
	
	//写入Flash数据
	STMFLASH_Write(PAGE1,flash_data,39);
	
}

//读取Flash里的的PID值
void PID_ReadFlash(void)
{
	u8 point=29;
	u16 flash_data[39];
	
	//读出Flash数据
	STMFLASH_Read(PAGE1,flash_data,39);
	
	if(flash_data[point++] == ADJUST_FLAG)//判断PID是否已经进行校准
	{
		//读出当前PID值
		PID_ROL.P = (float)flash_data[point++]/1000;
		PID_ROL.I = (float)flash_data[point++]/1000;
		PID_ROL.D = (float)flash_data[point++]/1000;
		
		PID_PIT.P = (float)flash_data[point++]/1000;
		PID_PIT.I = (float)flash_data[point++]/1000;
		PID_PIT.D = (float)flash_data[point++]/1000;
		
		PID_YAW.P = (float)flash_data[point++]/1000;
		PID_YAW.I = (float)flash_data[point++]/1000;
		PID_YAW.D = (float)flash_data[point++]/1000;
	}
	else
	{
		//写入当前PID值
		PID_WriteFlash();
	}
}








