#include "MPU6050.h"
#include "usart.h"
#include "IIC.h"
#include  <math.h> 

char TX_DATA[10];  	 //显示据缓存区
char BUF[15];       //接收数据缓存区
short GTRO_X,GTRO_Y,GTRO_Z,ACCEL_X,ACCEL_Y,ACCEL_Z,T_T;		 //X,Y,Z轴，温度


void DATA_printf(char *s,short temp_data)    //转字符串
{
	*s='\0';
	if(temp_data<0){
	temp_data=-temp_data;
    *s='-';
	}
	else *s=' ';
	  *++s =temp_data/1000+0x30;
	  temp_data=temp_data%1000; 
    *++s =temp_data/100+0x30;
    temp_data=temp_data%100;     //取余运算
    *++s =temp_data/10+0x30;
    temp_data=temp_data%10;      //取余运算
    *++s =temp_data+0x30; 
    	
}

void MPU6050_TEST(void)
{
	READ_MPU6050();	         //读取MPU6050数据
	Send_String("GTRO:");
  	DATA_printf(TX_DATA,GTRO_X);//转换X轴数据到数组
	Send_data('X');			        //发送X轴数
	DATA_printf(TX_DATA,GTRO_Y);//转换Y轴数据到数组
	Send_data('Y');			        //发送Y轴数
	DATA_printf(TX_DATA,GTRO_Z);//转换Z轴数据到数组
	Send_data('Z');			        //发送Z轴数
	Send_String("ACCEL:");
  	DATA_printf(TX_DATA,ACCEL_X);//转换X轴数据到数组
	Send_data('X');			        //发送X轴数
	DATA_printf(TX_DATA,ACCEL_Y);//转换Y轴数据到数组
	Send_data('Y');			        //发送Y轴数
	DATA_printf(TX_DATA,ACCEL_Z);//转换Z轴数据到数组
	Send_data('Z');			        //发送Z轴数
	DATA_printf(TX_DATA,T_T);   //转换温度数据到数组
	Send_data('T');			        //发送温度数据
	USART3_SendData(0X0D);	 //换行
	USART3_SendData(0X0A);	 //回车
}

void Init_MPU6050(void)
{
/*
   Single_Write(MPU6050_Addr,PWR_M, 0x80);   //
   Single_Write(MPU6050_Addr,SMPL, 0x07);    //
   Single_Write(MPU6050_Addr,DLPF, 0x1E);    //±2000°
   Single_Write(MPU6050_Addr,INT_C, 0x00 );  //
   Single_Write(MPU6050_Addr,PWR_M, 0x00);   //
*/

  	Single_Write(MPU6050_Addr,PWR_MGMT_1, 0x00);	//解除休眠状态
	Single_Write(MPU6050_Addr,SMPLRT_DIV, 0x02);	//取样率2分频，1k/2=500hz
	Single_Write(MPU6050_Addr,CONFIG, 0x02);	  	//低通滤波94hz
	Single_Write(MPU6050_Addr,GYRO_CONFIG, 0x18);	//2000dps
	Single_Write(MPU6050_Addr,ACCEL_CONFIG, 0x11); 	//+-8g,5Hz高通滤波	 
/*	Single_Write(MPU6050_Addr,SMPLRT_DIV, 0x07);
	Single_Write(MPU6050_Addr,CONFIG, 0x06);
	Single_Write(MPU6050_Addr,GYRO_CONFIG, 0x18);
	Single_Write(MPU6050_Addr,ACCEL_CONFIG, 0x01);				  		*/
}
	
//******读取MPU6050数据****************************************
void READ_MPU6050(void)
{
   BUF[0]=Single_Read(MPU6050_Addr,GYRO_XOUT_L); 
   BUF[1]=Single_Read(MPU6050_Addr,GYRO_XOUT_H);
   GTRO_X=	(BUF[1]<<8)|BUF[0];
   GTRO_X/=16.4; 						   //读取计算X轴数据

   BUF[2]=Single_Read(MPU6050_Addr,GYRO_YOUT_L);
   BUF[3]=Single_Read(MPU6050_Addr,GYRO_YOUT_H);
   GTRO_Y=	(BUF[3]<<8)|BUF[2];
   GTRO_Y/=16.4; 						   //读取计算Y轴数据
   BUF[4]=Single_Read(MPU6050_Addr,GYRO_ZOUT_L);
   BUF[5]=Single_Read(MPU6050_Addr,GYRO_ZOUT_H);
   GTRO_Z=	(BUF[5]<<8)|BUF[4];
   GTRO_Z/=16.4; 					       //读取计算Z轴数据
   
	 BUF[6]=Single_Read(MPU6050_Addr,ACCEL_XOUT_L); 
   BUF[7]=Single_Read(MPU6050_Addr,ACCEL_XOUT_H);
   ACCEL_X=	(BUF[7]<<8)|BUF[6];
   ACCEL_X/=163.84; 						   //读取计算X轴数据

   BUF[8]=Single_Read(MPU6050_Addr,ACCEL_YOUT_L);
   BUF[9]=Single_Read(MPU6050_Addr,ACCEL_YOUT_H);
   ACCEL_Y=	(BUF[9]<<8)|BUF[8];
   ACCEL_Y/=163.84; 						   //读取计算Y轴数据
   BUF[10]=Single_Read(MPU6050_Addr,ACCEL_ZOUT_L);
   BUF[11]=Single_Read(MPU6050_Addr,ACCEL_ZOUT_H);
   ACCEL_Z=	(BUF[11]<<8)|BUF[10];
   ACCEL_Z/=163.84; 					       //读取计算Z轴数据
	 
   BUF[12]=Single_Read(MPU6050_Addr,TEMP_OUT_L); 
   BUF[13]=Single_Read(MPU6050_Addr,TEMP_OUT_H); 
   T_T=(BUF[13]<<8)|BUF[12];
   T_T = (-5)+((double) (T_T + 13200)) / 280;// 读取计算出温度
}
 //********串口发送数据***************************************
void Send_data(unsigned char axis)
{
  USART3_SendData(axis);
  USART3_SendData(':');
  Send_String(TX_DATA);
  USART3_SendData(' ');
  USART3_SendData(' ');
}
 
void Send_String(char *data)
 {
	 while(*data)
	 {
		 USART3_SendData(*data);
		 data++;
	 }
 }
