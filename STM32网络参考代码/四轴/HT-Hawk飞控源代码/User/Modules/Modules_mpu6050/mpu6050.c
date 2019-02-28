/******************** (C) COPYRIGHT 2014 Air Nano Team ***************************
 * 文件名  ：mpu6050.c
 * 描述    ：mpu6050配置         
 * 实验平台：Air Nano四轴飞行器
 * 库版本  ：ST3.5.0
 * 作者    ：Air Nano Team 
 * 淘宝    ：http://byd2.taobao.com
**********************************************************************************/
#include "include.h"

u8		 mpu6050_buffer[14];//【八位数组】=================【IIC读取后存放数据】	
struct _sensor sensor;      //此处定义一个二级结构

/*struct _sensor{   
	struct _trans acc;
	struct _trans gyro;
              };
              
  struct _trans{
    struct _int16 origin;  //原始值
    struct _float averag;  //平均值
    struct _float histor;  //历史值
    struct _int16 quiet;   //静态值
    struct _float radian;  //弧度值 
          };                        
*/

/*====================================================================================================*/
/*====================================================================================================*
**函数 : InitMPU6050
**功能 :【初始化MPU6050】
**输入 : None
**輸出 : None
**备注 : None
**====================================================================================================*/
/*====================================================================================================*/
u8 InitMPU6050(void)
{
	u8 ack;
	//MPU6050做从机设备有八位地址  高七位0X68存储在Who am I中 低一位由AD0决定
	ack = Single_Read(MPU6050_ADDRESS, WHO_AM_I);                      //检查MPU6050是否连接汉
	if (!ack)    //正常ack=0X68真    !ack=0假   执行初始化
    return FALSE;
	
	Single_Write(MPU6050_ADDRESS, PWR_MGMT_1, 0x00);  	               //解除休眠状态
    //SMPLRT_DIV	0x19	//陀螺仪采样率，典型值：0x07(125Hz)          //8ms采集一次
	Single_Write(MPU6050_ADDRESS, SMPLRT_DIV, 0x07);     
	Single_Write(MPU6050_ADDRESS, CONFIGL, MPU6050_DLPF);              //低通滤波
	Single_Write(MPU6050_ADDRESS, GYRO_CONFIG, MPU6050_GYRO_FS_1000);  //陀螺仪量程 +-1000
	Single_Write(MPU6050_ADDRESS, ACCEL_CONFIG, MPU6050_ACCEL_FS_4);   //加速度量程 +-4G
	return TRUE;
}

//**************************实现函数********************************************
//  【将iic读取到得数据分拆,放入相应寄存器,更新MPU6050_Last】
//******************************************************************************
void MPU6050_Read(void)
{
	mpu6050_buffer[0]=Single_Read(MPU6050_ADDRESS, 0x3B);
	mpu6050_buffer[1]=Single_Read(MPU6050_ADDRESS, 0x3C);
	mpu6050_buffer[2]=Single_Read(MPU6050_ADDRESS, 0x3D);
	mpu6050_buffer[3]=Single_Read(MPU6050_ADDRESS, 0x3E);
	mpu6050_buffer[4]=Single_Read(MPU6050_ADDRESS, 0x3F);
	mpu6050_buffer[5]=Single_Read(MPU6050_ADDRESS, 0x40);
	mpu6050_buffer[8]=Single_Read(MPU6050_ADDRESS, 0x43);
	mpu6050_buffer[9]=Single_Read(MPU6050_ADDRESS, 0x44);
	mpu6050_buffer[10]=Single_Read(MPU6050_ADDRESS, 0x45);
	mpu6050_buffer[11]=Single_Read(MPU6050_ADDRESS, 0x46);
	mpu6050_buffer[12]=Single_Read(MPU6050_ADDRESS, 0x47);
	mpu6050_buffer[13]=Single_Read(MPU6050_ADDRESS, 0x48);
	
}
/**************************实现函数********************************************
//        【MPU6050数据整合 将iic读取到得数据分拆,放入相应寄存器】
//        【如果没有进行加速度零偏计算则进行计算】
*******************************************************************************/
void MPU6050_Dataanl(void)       //MPU6050输出数据为16位
{
	MPU6050_Read();
	                             //读到的数据进行合并减零偏
	sensor.acc.origin.x = ((((int16_t)mpu6050_buffer[0]) << 8) | mpu6050_buffer[1]) - sensor.acc.quiet.x; // 【加速度计数据】
	sensor.acc.origin.y = ((((int16_t)mpu6050_buffer[2]) << 8) | mpu6050_buffer[3]) - sensor.acc.quiet.y;
	sensor.acc.origin.z = ((((int16_t)mpu6050_buffer[4]) << 8) | mpu6050_buffer[5]);                      //Z轴加速度计未减零偏

	sensor.gyro.origin.x = ((((int16_t)mpu6050_buffer[8]) << 8) | mpu6050_buffer[9]);                     // 【陀螺仪数据】
	sensor.gyro.origin.y = ((((int16_t)mpu6050_buffer[10]) << 8)| mpu6050_buffer[11]);
	sensor.gyro.origin.z = ((((int16_t)mpu6050_buffer[12]) << 8)| mpu6050_buffer[13]);
    //陀螺零偏分开减的好处 因为 下边陀螺零偏计算函数要用到陀螺初始值
	sensor.gyro.radian.x = sensor.gyro.origin.x - sensor.gyro.quiet.x;                                    // 【陀螺仪数据减零偏】
	sensor.gyro.radian.y = sensor.gyro.origin.y - sensor.gyro.quiet.y;
	sensor.gyro.radian.z = sensor.gyro.origin.z - sensor.gyro.quiet.z;

//    	The calibration  of  acc                      //【加速度计零偏计算】
//      此处加速度计是如何实现累加计算的
//      此函数在姿态结算函数中调用 每中断一次执行一次
//      通过中断轮询加静态局部变量实现累加
//*****************************************************************************//	
	 if(flag.calibratingA)                            //【标志位】 如果加速度零偏计算没有完成  flag.calibratingA在rc.c中定义
	 {
		 static int32_t	tempax=0,tempay=0,tempaz=0;   //则计算加速度零偏
		 static uint8_t cnt_a=0;
		 if(cnt_a==0)                                 //首次清零
		 {
				sensor.acc.quiet.x = 0;
				sensor.acc.quiet.y = 0;
				sensor.acc.quiet.z = 0;
				tempax = 0;
				tempay = 0;
				tempaz = 0;
				cnt_a = 1;
		 }
				tempax+= sensor.acc.origin.x;
				tempay+= sensor.acc.origin.y;
				tempaz+= sensor.acc.origin.z;
				if(cnt_a==200)                          //累加200次求平均值
				{
					sensor.acc.quiet.x = tempax/cnt_a;
					sensor.acc.quiet.y = tempay/cnt_a;
					sensor.acc.quiet.z = tempaz/cnt_a;
					cnt_a = 0;
					flag.calibratingA = 0;
					EE_SAVE_ACC_OFFSET();               //把计算的加速度零偏值保存在EEPROM中
					return;
				}
				cnt_a++;		
			}	
}
/*====================================================================================================*/
/*====================================================================================================*
**函数 : Gyro_Calculateoffest
**功能 : 【计算陀螺仪零偏】
**输入 : 
**输出 : None
**使用 : Hto_Gyro_Calculateoffest();
**====================================================================================================*/
/*====================================================================================================*/
void Gyro_Caloffest(s32 x,s32 y,s32 z,u16 amount)
{
     sensor.gyro.quiet.x = x/amount;
	 sensor.gyro.quiet.y = y/amount;
	 sensor.gyro.quiet.z = z/amount;
}


/*====================================================================================================*/
/*====================================================================================================*
**函数 : Gyro_OFFSET    
**功能 :【陀螺仪静态采集】 调用 【计算陀螺仪零偏】
**输入 : None      静置状态下采集陀螺仪输出零偏值
**輸出 : None      50次求平均  单独执行 循环实现
**备注 : None     【开机初始化时执行】
**====================================================================================================*/
/*====================================================================================================*/
//此循环须确保四轴处于完全静止状态    标志位只在函数内使用
void Gyro_OFFSET(void)
{
	static u8 over_flag=0;                           //【u8  over_flag 】
	u8  i,cnt_g = 0;                                 //【u8  i  , cnt_g】
	s32 Integral[3] = {0,0,0};                       //【s32 Integral[3]】
	s32 tempg[3]={0,0,0};                            //【s32   tempg[3] 】
	s16 gx_last=0,gy_last=0,gz_last=0;               //【s16 gx_last=0  , gy_last=0 , gz_last=0】

	while(!over_flag)	                   
	{
		if(cnt_g < 50)
		{
			MPU6050_Dataanl();                //陀螺零偏的初始值问题   

			tempg[0] += sensor.gyro.origin.x; //此处使用的是陀螺初始值 所以没必要考虑零偏初值的问题
			tempg[1] += sensor.gyro.origin.y;
			tempg[2] += sensor.gyro.origin.z;
            
            //absu16( Math_X )  (Math_X<0? -Math_X:Math_X)
			Integral[0] += absu16(gx_last - sensor.gyro.origin.x);  // 前次采样值 减 本次采样值  每次都取正
			Integral[1] += absu16(gy_last - sensor.gyro.origin.y);    
			Integral[2] += absu16(gz_last - sensor.gyro.origin.z);

			gx_last = sensor.gyro.origin.x;
			gy_last = sensor.gyro.origin.y;                  
			gz_last = sensor.gyro.origin.z;
		}
		else{
			// 未校准成功  GYRO_GATHER   100 如果两次采样值差值过大则静态采样失败   此处采样静态机体陀螺仪数据 两次偏差不可能过大
			if(Integral[0]>=GYRO_GATHER || Integral[1]>=GYRO_GATHER || Integral[2]>= GYRO_GATHER){
				cnt_g = 0;
				for(i=0;i<3;i++){
				tempg[i]=Integral[i]=0;  //清空 累加数据
				}
			}
			// 校准成功 
			else{				
				   Gyro_Caloffest(tempg[0],tempg[1],tempg[2],50); //计算陀螺仪零偏值
				   over_flag = 1;
			}
		}
		cnt_g++;
	}
}



