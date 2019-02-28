/*************************************************
6轴陀螺仪加速度传感器MPU6050驱动

MCU：STM32F103CBT6，硬件IIC，库函数版本：3.5.0 KEIL

陈秋阳2013-01-16

*************************************************/
#include "stm32f10x.h"
#include "IIC.h"
#include "var_global.h"
#include "delay.h"
#include "HMC5883.H"
#include "stdlib.h"

#define	SMPLRT_DIV		0x19	//陀螺仪采样率，典型值：0x07(125Hz)
#define	CONFIG			  0x1A	//低通滤波频率，典型值：0x06(5Hz)
#define	GYRO_CONFIG		0x1B	//陀螺仪自检及测量范围，典型值：0x18(不自检，2000deg/s)
#define	ACCEL_CONFIG	0x1C	//加速计自检、测量范围及高通滤波频率，典型值：0x01(不自检，2G，5Hz)
#define	ACCEL_XOUT_H	0x3B
#define	ACCEL_XOUT_L	0x3C
#define	ACCEL_YOUT_H	0x3D
#define	ACCEL_YOUT_L	0x3E
#define	ACCEL_ZOUT_H	0x3F
#define	ACCEL_ZOUT_L	0x40
#define	TEMP_OUT_H		0x41
#define	TEMP_OUT_L		0x42

#define	GYRO_XOUT_H		0x43
#define	GYRO_XOUT_L		0x44	
#define	GYRO_YOUT_H		0x45
#define	GYRO_YOUT_L		0x46
#define	GYRO_ZOUT_H		0x47
#define	GYRO_ZOUT_L		0x48

#define	PWR_MGMT_1		0x6B	//电源管理，典型值：0x00(正常启用)


#define	MPU6050_Addr   0xD0	  //定义器件在IIC总线中的从地址,根据ALT  ADDRESS地址引脚不同修改 0x68

int16_XYZ ACC_RealData;
int16_XYZ GRY_RealData;
u8 Offset=0;

u16 Peace; //静止校准 ;

void MPU_GetData(u8 id,u8 reg,u8 len,u8 *buff);

void Init_MPU6050(void)
{

  I2C_WriteByte(MPU6050_Addr,PWR_MGMT_1, 0x00);	//解除休眠状态
	I2C_WriteByte(MPU6050_Addr,SMPLRT_DIV, 0x07); //采样分频7
	I2C_WriteByte(MPU6050_Addr,CONFIG, 0x04);     //25Hz低通
	I2C_WriteByte(MPU6050_Addr,GYRO_CONFIG,  2<<3); //陀螺仪量程1000/s
	I2C_WriteByte(MPU6050_Addr,ACCEL_CONFIG, 1<<3); //加速度量程2G
	
}


void READ_MPU6050(void)
{

u8 BUF_6050_buff[14];//6050所有测量参数
int16_XYZ Old_GRY;
u16 turbulen;

Old_GRY.X=GRY_RealData.X ;
Old_GRY.Y=GRY_RealData.Y ;
Old_GRY.Z=GRY_RealData.Z ;
	
MPU_GetData(MPU6050_Addr,ACCEL_XOUT_H,14,BUF_6050_buff);//连续读取14个字节用时500us
	
ACC_RealData.Y=(( ((int16_t)BUF_6050_buff[0]) <<8) |BUF_6050_buff[1]);
ACC_RealData.X=(( ((int16_t)BUF_6050_buff[2]) <<8) |BUF_6050_buff[3]);	 	
ACC_RealData.Z=(( ((int16_t)BUF_6050_buff[4]) <<8) |BUF_6050_buff[5]);	
 		
GRY_RealData.Y=(( ((int16_t)BUF_6050_buff[8]) <<8)  |BUF_6050_buff[9]);
GRY_RealData.X=(( ((int16_t)BUF_6050_buff[10]) <<8) |BUF_6050_buff[11]);
GRY_RealData.Z=(( ((int16_t)BUF_6050_buff[12]) <<8) |BUF_6050_buff[13]);	

if(Offset)
{
 ACC_RealData.X-=ACC_Offset.X;
 ACC_RealData.Y-=ACC_Offset.Y;
 ACC_RealData.Z-=ACC_Offset.Z;		
 GRY_RealData.X-=GRY_Offset.X;
 GRY_RealData.Y-=GRY_Offset.Y;
 GRY_RealData.Z-=GRY_Offset.Z;
}

turbulen=abs(Old_GRY.X-GRY_RealData.X)+abs(Old_GRY.Y-GRY_RealData.Y)+abs(Old_GRY.Z-GRY_RealData.Z);
if(turbulen<15) Peace++;  else Peace=0;  


}

 void Set_Offset()
{
  ACC_Offset.X=161;
	ACC_Offset.Y=0;
  ACC_Offset.Z=1700;
	
	Offset=1;
}


void Get_GRY_Offset()
{ 
	u8 i;
	vs32 GRY_XTemp=0,GRY_YTemp=0,GRY_ZTemp=0;
	
	GPIO_ResetBits(GPIOB,GPIO_Pin_1);
	
	Offset=0;

	for(i=0;i<5;i++)	
	{
	READ_MPU6050();
 
		
	GRY_XTemp+=GRY_RealData.X;
	GRY_YTemp+=GRY_RealData.Y;
	GRY_ZTemp+=GRY_RealData.Z;

	Delay_s(1); 
 	} 	
	
	GRY_XTemp/=5;
	GRY_YTemp/=5;
	GRY_ZTemp/=5;
	
	
	GRY_Offset.X=GRY_XTemp;
	GRY_Offset.Y=GRY_YTemp;
	GRY_Offset.Z=GRY_ZTemp;
		
	Offset=1;

  GPIO_SetBits(GPIOB,GPIO_Pin_1); 
}



void MPU_GetData(u8 id,u8 reg,u8 len,u8 *buff) 

{ //启动传输发送首寄存器地址
  while(I2C_GetFlagStatus(I2C1, I2C_FLAG_BUSY));
  I2C_GenerateSTART(I2C1, ENABLE);
  while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT));
  I2C_Send7bitAddress(I2C1, id, I2C_Direction_Transmitter);
  while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED));
  I2C_Cmd(I2C1, ENABLE);
  I2C_SendData(I2C1,reg);
  while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED));
	
	//开始接受
  I2C_GenerateSTART(I2C1, ENABLE);
  while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT));
  I2C_Send7bitAddress(I2C1, id, I2C_Direction_Receiver);
  while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED));
	

	while (len) 
	{   while(!(I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_RECEIVED)));
      
		  *buff =I2C_ReceiveData(I2C1);    
     
		 if(len==2){ 
		             I2C_AcknowledgeConfig(I2C1, DISABLE);//最后一个不应答
		           }          		  
			buff++;
      len--;
  }
	
	
	
	I2C_AcknowledgeConfig(I2C1, ENABLE);
	I2C_GenerateSTOP(I2C1, ENABLE);

}










