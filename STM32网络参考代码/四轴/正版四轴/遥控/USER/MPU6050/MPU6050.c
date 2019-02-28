#include "MPU6050.h"
#include "IIC.h"

//**************************************
//MPU6050
//**************************************
void InitMPU6050(void)
{
	Single_WriteI2C(0xD0,PWR_MGMT_1  , 0x00);
	Single_WriteI2C(0xD0,SMPLRT_DIV  , 0x07);
	Single_WriteI2C(0xD0,MPU_CONFIG  , 0x03);
	Single_WriteI2C(0xD0,GYRO_CONFIG , 0x10); 
	Single_WriteI2C(0xD0,ACCEL_CONFIG, 0x08);
}
//**************************************
int16_t GetData(uint8_t REG_Address)
{
	uint8_t Hd,Ld;
	Hd=Single_ReadI2C(0xD0,REG_Address);
	Ld=Single_ReadI2C(0xD0,REG_Address+1);
	return (Hd<<8)+Ld; 
}
//**************************************
//地推均值滤波
//**************************************
#define N 5
float Data_X_g[N];
float Data_Y_g[N];
float Data_Z_g[N];
float GildeAverageValueFilter(float NewValue,float *Data)
{
	unsigned char i;
	float Value;
	float sum;
	sum=0;
	Data[N] = NewValue;
	for(i=0;i<N;i++)
	{
	Data[i]=Data[i+1];
	sum+=Data[i];
	}
	Value=sum/N;
	return(Value);
}
//********************************************************
float  X_g;
float  Y_g;
float  Z_g;

float  X_g_av;
float  Y_g_av;
float  Z_g_av;

float  X_w;
float  Y_w;
float  Z_w;

float  X_g_off = 0;
float  Y_g_off = 0;
float  Z_g_off = 0;

float  X_w_off = 0;
float  Y_w_off = 0;
float  Z_w_off = 0;

void GET_ACC_DATA(void)
{
	unsigned char i;
	for(i=0;i<5;i++)
	{
	X_g    = GetData(ACCEL_XOUT_H) + X_g_off;
	X_g_av = GildeAverageValueFilter(X_g,Data_X_g);
	Y_g    = GetData(ACCEL_YOUT_H) + Y_g_off;
	Y_g_av = GildeAverageValueFilter(Y_g,Data_Y_g);
	Z_g    = GetData(ACCEL_ZOUT_H) + Z_g_off;
	Z_g_av = GildeAverageValueFilter(Z_g,Data_Z_g);
	}
}
void GET_GYRO_DATA(void)
{
	X_w  = GetData(GYRO_XOUT_H) + X_w_off;
	Y_w  = GetData(GYRO_YOUT_H) + Y_w_off;
	Z_w  = GetData(GYRO_ZOUT_H) + Z_w_off;
}
void GET_MPU_DATA(void)
{
  GET_ACC_DATA();
	GET_GYRO_DATA();
}
