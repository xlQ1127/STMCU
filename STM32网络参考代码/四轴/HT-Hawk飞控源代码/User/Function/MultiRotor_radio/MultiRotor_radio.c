/******************** (C) COPYRIGHT 2014 Air Nano Team ***************************
 * 文件名  ：IMU.c
 * 描述    ：姿态解算         
 * 实验平台：HT_Hawk
 * 库版本  ：ST3.5.0
 * 作者    ：Air Nano Team
 * 论坛    ：http://www.airnano.cn
 * 淘宝    ：http://byd2.taobao.com   
 *           http://hengtuo.taobao.com   
**********************************************************************************/
#include "board_config.h"
#include "MultiRotor_radio.h"

int i,F,sw; 

unsigned char HtoEs_OutPut_Buffer[63] = {0};	   //串口发送缓冲区
unsigned int CHK_SUM;  //校验和


///////////////////////////////////////////////////////////////////
// 15通道数据

float CH1_data  = 0;
float CH2_data  = 0;
float CH3_data  = 0;
float CH4_data  = 0;
float CH5_data  = 0;
float CH6_data  = 0;
float CH7_data  = 0;  //15通道独立数据
float CH8_data  = 0;
float CH9_data  = 0;
float CH10_data = 0;
float CH11_data = 0;
float CH12_data = 0;
float CH13_data = 0;
float CH14_data = 0;
float CH15_data = 0;

///////////////////////////////////////////////////////////////////
//  GPS 数据

float Longitude_val;  //经度数值
float Latitude_Val ;  //纬度数值

float Altitude_Val;  //高度数值
float Dir_Val;       //方位数值
float SPD_Val;       //速度数值

unsigned char Satellite_Val;   //卫星个数
unsigned int  Voltage_Val;     //电池电压
unsigned int  Temperture_Val;  //温度数值

unsigned char Longitude_WE; //标志经度方向，true=W；false=E；
unsigned char Latitude_NS;  //标志纬度方向，true=N；false=S；
unsigned char Location_Sta; //定位状态标识

///////////////////////////////////////////////////////////////////
// PID 参数

int Pitch_PID_P;
int Pitch_PID_I;
int Pitch_PID_D;

int Roll_PID_P;
int Roll_PID_I;
int Roll_PID_D;

int Yaw_PID_P;
int Yaw_PID_I;
int Yaw_PID_D;

int Alt_PID_P;
int Alt_PID_I;
int Alt_PID_D;

int Pos_PID_P;
int Pos_PID_I;
int Pos_PID_D;

//数据缓存数组写入函数                                         //【Float2Byte（）】
//target:目标单精度数据
//buf:待写入数组
//beg:指定从数组第几个元素开始写入
void Float2Byte(float *target,unsigned char *buf,unsigned char beg)
{
    unsigned char *point;
    point = (unsigned char*)target;	  //得到float的地址
    buf[beg]   = point[0];
    buf[beg+1] = point[1];
    buf[beg+2] = point[2];
    buf[beg+3] = point[3];
}

//【生成独立通道数据帧】）））））））））））））））））））  //【HtoEs_Chart_Data_Generate（）】
unsigned char HtoEs_Chart_Data_Generate(void)                   
{
	  unsigned char i;
	
	  HtoEs_OutPut_Buffer[0] = 0x3F; //【帧长度 63字节】
		HtoEs_OutPut_Buffer[1] = 0x01; //【功能码 0x01】
		
	  CH1_data = (float)sensor.acc.origin.x;   //加速度计原始数据
	  CH2_data = (float)sensor.acc.origin.y;
	  CH3_data = (float)sensor.acc.origin.z;
	  CH4_data = (float)sensor.gyro.origin.x;  //陀螺仪原始数据
	  CH5_data = (float)sensor.gyro.origin.y;
	  CH6_data = (float)sensor.gyro.origin.z;
	  CH7_data = (float)MAG[0];                //磁力计原始数据
	  CH8_data = (float)MAG[1];
	  CH9_data = (float)MAG[2];
	  CH10_data = (float)0;
	  CH11_data = (float)0;
	  CH12_data = (float)0;
	
		Float2Byte(&CH1_data ,HtoEs_OutPut_Buffer,2);  //15通道数据写入缓冲数组
		Float2Byte(&CH2_data ,HtoEs_OutPut_Buffer,6);
		Float2Byte(&CH3_data ,HtoEs_OutPut_Buffer,10); //数据写入到【HtoEs_OutPut_Buffer[]】
		Float2Byte(&CH4_data ,HtoEs_OutPut_Buffer,14);
		Float2Byte(&CH5_data ,HtoEs_OutPut_Buffer,18);
		Float2Byte(&CH6_data ,HtoEs_OutPut_Buffer,22);
		Float2Byte(&CH7_data ,HtoEs_OutPut_Buffer,26);
		Float2Byte(&CH8_data ,HtoEs_OutPut_Buffer,30);
		Float2Byte(&CH9_data ,HtoEs_OutPut_Buffer,34);
		Float2Byte(&CH10_data,HtoEs_OutPut_Buffer,38);
		Float2Byte(&CH11_data,HtoEs_OutPut_Buffer,42);
		Float2Byte(&CH12_data,HtoEs_OutPut_Buffer,46);
		Float2Byte(&CH13_data,HtoEs_OutPut_Buffer,50);
		Float2Byte(&CH14_data,HtoEs_OutPut_Buffer,54);
		Float2Byte(&CH15_data,HtoEs_OutPut_Buffer,58);
		
		CHK_SUM =0; 
	
	  for(i = 0 ; i < 62; i++)  //计算和 第63位作为校验位
			CHK_SUM += HtoEs_OutPut_Buffer[i];
		
		HtoEs_OutPut_Buffer[62] = CHK_SUM % 255; //计算校验值
 
	  return 63; 
}

//生成GPS数据帧
unsigned char HtoEs_GPS_Data_Generate(void)
{
	  unsigned char i;
	
	  HtoEs_OutPut_Buffer[0] = 0x1D; //帧长度 29字节
		HtoEs_OutPut_Buffer[1] = 0x02; //功能码
	
	  Float2Byte(&Longitude_val ,HtoEs_OutPut_Buffer,2);  //经度
	  Float2Byte(&Latitude_Val  ,HtoEs_OutPut_Buffer,6);  //纬度
	  Float2Byte(&Altitude_Val  ,HtoEs_OutPut_Buffer,10); //高度
	  Float2Byte(&Dir_Val ,HtoEs_OutPut_Buffer,14);       //方位角
	  Float2Byte(&SPD_Val ,HtoEs_OutPut_Buffer,18);       //速度
	
	  HtoEs_OutPut_Buffer[22] = (Voltage_Val & 0xFF00) >> 8 ;    //取高8位
	  HtoEs_OutPut_Buffer[23] = (Voltage_Val & 0x00FF) ;         //取低8位
	
	  HtoEs_OutPut_Buffer[24] = (Temperture_Val & 0xFF00) >> 8 ; //取高8位
	  HtoEs_OutPut_Buffer[25] = (Temperture_Val & 0x00FF) ;      //取低8位
	
	  HtoEs_OutPut_Buffer[26] = Satellite_Val; //卫星个数
	
	//============================================================================
	  
		HtoEs_OutPut_Buffer[27] = 0; //先将状态标识清除
		
		if( Location_Sta ) //定位模式
			HtoEs_OutPut_Buffer[27] |= 0x01; //置高
		else               //导航模式
			HtoEs_OutPut_Buffer[27] &= 0xFE; //清零
		
		
		if( Longitude_WE == 'W' )  //经度方向
			HtoEs_OutPut_Buffer[27] |= 0x02;  //W,西经
		else if( Longitude_WE == 'E' ) 
			HtoEs_OutPut_Buffer[27] &= 0xFD;  //E,东经
 
		if( Latitude_NS == 'N' )  //纬度方向
			HtoEs_OutPut_Buffer[27] |= 0x04;  //N,北纬
		else if( Latitude_NS == 'S' ) 
			HtoEs_OutPut_Buffer[27] &= 0xFB;  //S,南纬
		
	//============================================================================	
		
		CHK_SUM =0; 
	
	  for(i = 0 ; i < 28; i++)  //计算和
			CHK_SUM += HtoEs_OutPut_Buffer[i];
		
		HtoEs_OutPut_Buffer[28] = CHK_SUM % 255; //计算校验值
	
	  return 29; 
}

//【生成姿态数据帧】 】】】】】】】】】】】】】】】】】】】           //【 HtoEs_Attitude_Data_Generate（）】
unsigned char HtoEs_Attitude_Data_Generate(void)
{
	  unsigned char i;
	
	  HtoEs_OutPut_Buffer[0] = 0x0F; //【帧长度 15字节】
		HtoEs_OutPut_Buffer[1] = 0x03; //【功能码 0x03】
	
	  Float2Byte(&AngE.Pitch,HtoEs_OutPut_Buffer,2);  //俯仰           //目标单精度数据；待写入数组；从数组第几个元素开始写入
	  Float2Byte(&AngE.Roll ,HtoEs_OutPut_Buffer,6);  //横滚           //【&AngE.Roll】【HtoEs_OutPut_Buffer】 【6】
	  Float2Byte(&AngE.Yaw  ,HtoEs_OutPut_Buffer,10); //航向           //数据写入到【HtoEs_OutPut_Buffer[]】
	  
	//============================================================================	
		
		CHK_SUM =0; 
	
	  for(i = 0 ; i < 14; i++)  //计算和
			CHK_SUM += HtoEs_OutPut_Buffer[i];
		
		HtoEs_OutPut_Buffer[14] = CHK_SUM % 255; //计算校验值
		
	  return 15; 
}

//【生成RC通道数据帧】  》》》》》》》》》》》》》》》》》》              //【HtoEs_RC_Data_Generate（）】
unsigned char HtoEs_RC_Data_Generate(void)
{
	  unsigned char i;
	
	  HtoEs_OutPut_Buffer[0] = 0x15; //【帧长度 15字节】                    // 数据存储到
		HtoEs_OutPut_Buffer[1] = 0x04; //【功能码 0x04】                      //【HtoEs_OutPut_Buffer[]】
	 
	  HtoEs_OutPut_Buffer[2] = (RC_Data.rc_data[0] & 0xFF00) >> 8 ; //取高8位
	  HtoEs_OutPut_Buffer[3] = (RC_Data.rc_data[0] & 0x00FF) ;      //取低8位
	
	  HtoEs_OutPut_Buffer[4] = (RC_Data.rc_data[1] & 0xFF00) >> 8 ; //取高8位
	  HtoEs_OutPut_Buffer[5] = (RC_Data.rc_data[1] & 0x00FF) ;      //取低8位
	
	  HtoEs_OutPut_Buffer[6] = (RC_Data.rc_data[2] & 0xFF00) >> 8 ; //取高8位
	  HtoEs_OutPut_Buffer[7] = (RC_Data.rc_data[2] & 0x00FF) ;      //取低8位
	
	  HtoEs_OutPut_Buffer[8] = (RC_Data.rc_data[3] & 0xFF00) >> 8 ; //取高8位
	  HtoEs_OutPut_Buffer[9] = (RC_Data.rc_data[3] & 0x00FF) ;      //取低8位
	                                                                                   //注意16位数据的处理方式
	  HtoEs_OutPut_Buffer[10] = (RC_Data.rc_data[0] & 0xFF00) >> 8 ; //取高8位
	  HtoEs_OutPut_Buffer[11] = (RC_Data.rc_data[0] & 0x00FF) ;      //取低8位
	
	  HtoEs_OutPut_Buffer[12] = (RC_Data.rc_data[0] & 0xFF00) >> 8 ; //取高8位
	  HtoEs_OutPut_Buffer[13] = (RC_Data.rc_data[0] & 0x00FF) ;      //取低8位
		
		HtoEs_OutPut_Buffer[14] = (RC_Data.rc_data[0] & 0xFF00) >> 8 ; //取高8位
	  HtoEs_OutPut_Buffer[15] = (RC_Data.rc_data[0] & 0x00FF) ;      //取低8位
		
		HtoEs_OutPut_Buffer[16] = (RC_Data.rc_data[0] & 0xFF00) >> 8 ; //取高8位
	  HtoEs_OutPut_Buffer[17] = (RC_Data.rc_data[0] & 0x00FF) ;      //取低8位
		
		HtoEs_OutPut_Buffer[18] = (RC_Data.rc_data[0] & 0xFF00) >> 8 ; //取高8位
	  HtoEs_OutPut_Buffer[19] = (RC_Data.rc_data[0] & 0x00FF) ;      //取低8位
	 
	//============================================================================	
		
		CHK_SUM =0; 
	
	  for(i = 0 ; i < 20; i++)  //计算和
			CHK_SUM += HtoEs_OutPut_Buffer[i];
		
		HtoEs_OutPut_Buffer[20] = CHK_SUM % 255; //计算校验值
	
	  return 21; 
}

//【生成PID数据帧】   ++++++++++++++++++++++++++++++++++++++++++             //【HtoEs_PID_Data_Generate（）】
unsigned char HtoEs_PID_Data_Generate(void)
{
	  unsigned char i;                                                         //【内环PID数据】
	
	  Pitch_PID_P = ctrl.pitch.core.kp * 100;   //Pitch
	  Pitch_PID_I = ctrl.pitch.core.ki * 100;
	  Pitch_PID_D = ctrl.pitch.core.kd * 100;
	  Roll_PID_P  = ctrl.roll.core.kp * 100;    //Roll
	  Roll_PID_I  = ctrl.roll.core.ki * 100;
	  Roll_PID_D  = ctrl.roll.core.kd * 100;
	  Yaw_PID_P   = ctrl.yaw.core.kp * 100;     //YAW
	  Yaw_PID_I   = ctrl.yaw.core.ki * 100;
	  Yaw_PID_D   = ctrl.yaw.core.kd * 100;
	
	  HtoEs_OutPut_Buffer[0] = 0x21; //【帧长度 15字节】
		HtoEs_OutPut_Buffer[1] = 0x05; //【功能码 0x05】                         //数据保存到【HtoEs_OutPut_Buffer[]】
	
	  HtoEs_OutPut_Buffer[2] = (Pitch_PID_P & 0xFF00) >> 8 ; //取高8位
	  HtoEs_OutPut_Buffer[3] = (Pitch_PID_P & 0x00FF) ;      //取低8位
	  HtoEs_OutPut_Buffer[4] = (Pitch_PID_I & 0xFF00) >> 8 ; //取高8位
	  HtoEs_OutPut_Buffer[5] = (Pitch_PID_I & 0x00FF) ;      //取低8位
	  HtoEs_OutPut_Buffer[6] = (Pitch_PID_D & 0xFF00) >> 8 ; //取高8位
	  HtoEs_OutPut_Buffer[7] = (Pitch_PID_D & 0x00FF) ;      //取低8位
	
	  HtoEs_OutPut_Buffer[8] = (Roll_PID_P & 0xFF00) >> 8 ; //取高8位
	  HtoEs_OutPut_Buffer[9] = (Roll_PID_P & 0x00FF) ;      //取低8位
	  HtoEs_OutPut_Buffer[10] = (Roll_PID_I & 0xFF00) >> 8 ; //取高8位
	  HtoEs_OutPut_Buffer[11] = (Roll_PID_I & 0x00FF) ;      //取低8位
	  HtoEs_OutPut_Buffer[12] = (Roll_PID_D & 0xFF00) >> 8 ; //取高8位
	  HtoEs_OutPut_Buffer[13] = (Roll_PID_D & 0x00FF) ;      //取低8位
		
		HtoEs_OutPut_Buffer[14] = (Yaw_PID_P & 0xFF00) >> 8 ; //取高8位
	  HtoEs_OutPut_Buffer[15] = (Yaw_PID_P & 0x00FF) ;      //取低8位
	  HtoEs_OutPut_Buffer[16] = (Yaw_PID_I & 0xFF00) >> 8 ; //取高8位
	  HtoEs_OutPut_Buffer[17] = (Yaw_PID_I & 0x00FF) ;      //取低8位
	  HtoEs_OutPut_Buffer[18] = (Yaw_PID_D & 0xFF00) >> 8 ; //取高8位
	  HtoEs_OutPut_Buffer[19] = (Yaw_PID_D & 0x00FF) ;      //取低8位
		
		HtoEs_OutPut_Buffer[20] = (Alt_PID_P & 0xFF00) >> 8 ; //取高8位
	  HtoEs_OutPut_Buffer[21] = (Alt_PID_P & 0x00FF) ;      //取低8位
	  HtoEs_OutPut_Buffer[22] = (Alt_PID_I & 0xFF00) >> 8 ; //取高8位
	  HtoEs_OutPut_Buffer[23] = (Alt_PID_I & 0x00FF) ;      //取低8位
	  HtoEs_OutPut_Buffer[24] = (Alt_PID_D & 0xFF00) >> 8 ; //取高8位
	  HtoEs_OutPut_Buffer[25] = (Alt_PID_D & 0x00FF) ;      //取低8位
		
		HtoEs_OutPut_Buffer[26] = (Pos_PID_P & 0xFF00) >> 8 ; //取高8位
	  HtoEs_OutPut_Buffer[27] = (Pos_PID_P & 0x00FF) ;      //取低8位
	  HtoEs_OutPut_Buffer[28] = (Pos_PID_I & 0xFF00) >> 8 ; //取高8位
	  HtoEs_OutPut_Buffer[29] = (Pos_PID_I & 0x00FF) ;      //取低8位
	  HtoEs_OutPut_Buffer[30] = (Pos_PID_D & 0xFF00) >> 8 ; //取高8位
	  HtoEs_OutPut_Buffer[31] = (Pos_PID_D & 0x00FF) ;      //取低8位
		
	//============================================================================	
		
		CHK_SUM =0; 
	
	  for(i = 0 ; i < 32; i++)  //计算和
			CHK_SUM += HtoEs_OutPut_Buffer[i];
		
		HtoEs_OutPut_Buffer[32] = CHK_SUM % 255; //计算校验值
	
	  return 33; 
}

// //生成传感器数据帧
// unsigned char HtoEs_Senosrs_Data_Generate(void)
// {
// 	  unsigned char i;
// 	
// 	  HtoEs_OutPut_Buffer[0] = 0x27; //帧长度 15字节
// 		HtoEs_OutPut_Buffer[1] = 0x06; //功能码
// 	
// 	  Float2Byte(&Gyro_X  ,HtoEs_OutPut_Buffer,2);
// 		Float2Byte(&Gyro_Y  ,HtoEs_OutPut_Buffer,6);
// 		Float2Byte(&Gyro_Z  ,HtoEs_OutPut_Buffer,10);
// 		Float2Byte(&Accel_X ,HtoEs_OutPut_Buffer,14);
// 		Float2Byte(&Accel_Y ,HtoEs_OutPut_Buffer,18);
// 		Float2Byte(&Accel_Z ,HtoEs_OutPut_Buffer,22);
// 		Float2Byte(&Mag_X   ,HtoEs_OutPut_Buffer,26);
// 		Float2Byte(&Mag_Y   ,HtoEs_OutPut_Buffer,30);
// 		Float2Byte(&Mag_Z   ,HtoEs_OutPut_Buffer,34);
// 	
// //============================================================================	
// 		
// 		CHK_SUM =0; 
// 	
// 	  for(i = 0 ; i < 38; i++)  //计算和
// 			CHK_SUM += HtoEs_OutPut_Buffer[i];
// 		
// 		HtoEs_OutPut_Buffer[38] = CHK_SUM % 255; //计算校验值	
// 	
// 	  return 39; 
// }

s8 fg=10;
/***********************************************************************************************************/
void mavlink(void)     //【飞控板循环发送数据到上位机】
{

	
	switch(sw)           //循环发送各模块数据
	{
			case 1: F = HtoEs_Chart_Data_Generate();    break;  //测试独立通道,返回需发送字节数）））MPU6050原始数据
			case 2: F = HtoEs_RC_Data_Generate();       break;  //测试RC通道，返回需发送字节数 》》》遥控器四通道数据	 
      case 3: F = HtoEs_Attitude_Data_Generate(); break;  //测试姿态,返回需发送字节数	   】】】姿态三个欧拉角
      case 4: if(fg ==10 ) F = HtoEs_PID_Data_Generate(); //调节PID参数（内环）          ++++++++++++++++++++++
				      if(fg<10 && fg >0){
								fg--; 
								F = HtoEs_PID_Data_Generate();            //测试PID参数，返回需发送字节数		
			        }				
							break;  
      case 5: F = HtoEs_GPS_Data_Generate(); 		  break;  //测试GPS,返回需发送字节数
    //case 6: F = HtoEs_Senosrs_Data_Generate();  break;  //测试传感器标定，返回需发送字节数					
			case 6: sw = 0; break; 
	}
	sw++; 
		
	usb_SendDataToHost(&HtoEs_OutPut_Buffer[0], F);
}


/*====================================================================================================*/
/*====================================================================================================*
**函数 : UsbCmdPro
**功能 : USB数据接收
**输入 : None
**輸出 : None
**备注 : None
**====================================================================================================*/
/*====================================================================================================*/
void UsbCmdPro(void)
{
  u8 ucData,ucNum;
  static u16 aCmdBuf[32];
	static u16 usPos;
	static u16 Free_heart=0;
	
	// 空闲心跳
	Free_heart++;  
	if(Free_heart>=60) {
		usPos = 0;
		Free_heart = 60;
 		array_assign(aCmdBuf,0,32);
  }
	
	// 从USB口读取一个字节 ucNum存放读到的字节个数 
	ucData = usb_GetRxByte(&ucNum);	
	
	// 没有接收到输出 退出
	if (ucNum == 0)		return;
	
	Free_heart=0;
	
 	// 接收到的数据放入缓存
 	aCmdBuf[usPos++] = ucData;
  
	if(usPos>=31) usPos=31;
	
	// 数据解析
	if(fg!=10) Data_Parser(aCmdBuf);
	fg=5;
}


