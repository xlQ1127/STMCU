/******************** (C) COPYRIGHT 2014 Air Nano Team ***************************
 * 文件名  ：IMU.c
 * 描述    ：姿态解算         
 * 实验平台：HT_Hawk
 * 库版本  ：ST3.5.0
 * 作者    ：Air Nano Team 
 * 论坛    ：http://www.airnano.cn
 * 淘宝     ：http://byd2.taobao.com   
 *            http://hengtuo.taobao.com   
**********************************************************************************/
#include "board_config.h"
#include "MultiRotor_ahrs.h"
#include "MPU6050.h"

#define KpDef 0.8f
#define KiDef 0.0005f
#define SampleRateHalf 0.001f  

#define  IIR_ORDER     4      //使用IIR滤波器的阶数
double b_IIR[IIR_ORDER+1] ={0.0008f, 0.0032f, 0.0048f, 0.0032f, 0.0008f};  //系数b
double a_IIR[IIR_ORDER+1] ={1.0000f, -3.0176f, 3.5072f, -1.8476f, 0.3708f};//系数a
double InPut_IIR[3][IIR_ORDER+1] = {0};
double OutPut_IIR[3][IIR_ORDER+1] = {0};

// /*	
// 	Q:过程噪声，Q增大，动态响应变快，收敛稳定性变坏
// 	R:测量噪声，R增大，动态响应变慢，收敛稳定性变好	
// */
// #define KALMAN_Q        0.02
// #define KALMAN_R        8.0000

Quaternion NumQ = {1, 0, 0, 0};   //四元数初始值
        /*typedef  struct {
          float Pitch;
          float Roll;
          float Yaw;
        } EulerAngle;
        */
EulerAngle AngE = {0};            //四元数转欧拉角存入该结构体

int16_t MAG[3];


//*******************************************【获取传感器滤波后的数据】*********************************************
void AHRS_getValues(void)
{
	static float x,y,z;
	
	MPU6050_Dataanl();    //【读取6050数据减零偏  】
	
	HMC5883lRead(MAG);    //【读取磁力计数据减零偏】
	
	// 【加速度计IIR滤波】
	sensor.acc.averag.x = IIR_I_Filter(sensor.acc.origin.x, InPut_IIR[0], OutPut_IIR[0], b_IIR, IIR_ORDER+1, a_IIR, IIR_ORDER+1);
	sensor.acc.averag.y = IIR_I_Filter(sensor.acc.origin.y, InPut_IIR[1], OutPut_IIR[1], b_IIR, IIR_ORDER+1, a_IIR, IIR_ORDER+1);
	sensor.acc.averag.z = IIR_I_Filter(sensor.acc.origin.z, InPut_IIR[2], OutPut_IIR[2], b_IIR, IIR_ORDER+1, a_IIR, IIR_ORDER+1);
	
	// 【陀螺仪一阶低通滤波】
 	sensor.gyro.averag.x = LPF_1st(x,sensor.gyro.radian.x * Gyro_G,0.386f);	x = sensor.gyro.averag.x;
 	sensor.gyro.averag.y = LPF_1st(y,sensor.gyro.radian.y * Gyro_G,0.386f);	y = sensor.gyro.averag.y;
 	sensor.gyro.averag.z = LPF_1st(z,sensor.gyro.radian.z * Gyro_G,0.386f);	z = sensor.gyro.averag.z;
}

/*====================================================================================================*/
/*====================================================================================================*
**函数 : AHRS_Update
**功能 : AHRS
**输入 : 四元数初始值 Quaternion NumQ = {1, 0, 0, 0}    
**输出 : None
**使用 : AHRS_Update();
**====================================================================================================*/
/*====================================================================================================*/
void AHRS_GetQ( Quaternion *pNumQ )
{
  fp32 ErrX, ErrY, ErrZ;
  fp32 AccX, AccY, AccZ;
  fp32 GyrX, GyrY, GyrZ;
  fp32 Normalize;
  static fp32 exInt = 0.0f, eyInt = 0.0f, ezInt = 0.0f;
  Gravity V;
	
  // 加速度归一化   Q_rsqurt（） 倒数运算
  Normalize = Q_rsqrt(squa(sensor.acc.averag.x)+ squa(sensor.acc.averag.y) +squa(sensor.acc.averag.z));
  AccX = sensor.acc.averag.x*Normalize;
  AccY = sensor.acc.averag.y*Normalize;
  AccZ = sensor.acc.averag.z*Normalize;

  // 提取重力分量
  V = Quaternion_vectorGravity(&NumQ);
	
  // 向量差乘 求解坐标系误差
  ErrX = (AccY*V.z - AccZ*V.y);
  ErrY = (AccZ*V.x - AccX*V.z);
  ErrZ = (AccX*V.y - AccY*V.x);
 	
  exInt = exInt + ErrX * KiDef;   //误差积分
  eyInt = eyInt + ErrY * KiDef;
  ezInt = ezInt + ErrZ * KiDef;
  //用叉积误差来做PI补偿修正陀螺
  GyrX = Rad(sensor.gyro.averag.x) + KpDef * VariableParameter(ErrX) * ErrX  +  exInt;
  GyrY = Rad(sensor.gyro.averag.y) + KpDef * VariableParameter(ErrY) * ErrY  +  eyInt;
  GyrZ = Rad(sensor.gyro.averag.z) + KpDef * VariableParameter(ErrZ) * ErrZ  +  ezInt;
	
  // 一阶龙格库塔法, 更新四元数
  Quaternion_RungeKutta(&NumQ, GyrX, GyrY, GyrZ, SampleRateHalf);   //调用四元数微分方程函数
	
  // 四元数归一化
  Quaternion_Normalize(&NumQ);
}

/*====================================================================================================*/
/*====================================================================================================*
**函数 : AHRS_Geteuler
**功能 : AHRS获取欧拉角
**输入 : None
**输出 : None
**使用 : AHRS_Geteuler();
**====================================================================================================*/
/*====================================================================================================*/
void AHRS_Geteuler(void)
{
  fp32 sin_pitch,sin_roll,cos_roll,cos_pitch;
  //获取滤波后的数据
  AHRS_getValues();
	
  //姿态解算获取四元数
  //传入参数 Quaternion NumQ = {1, 0, 0, 0}四元数初始值  
  AHRS_GetQ(&NumQ);
	
  //四元数转欧拉角
  //姿态解算是解算出三个欧拉角的只不过解算出的航向角不是很准确
  Quaternion_ToAngE(&NumQ, &AngE);  //传入欧拉角结构体指针  
	
  //计算欧拉角的三角函数值 ROLL 和 PITCH 
  sin_roll  = sin(AngE.Roll);
  sin_pitch = sin(AngE.Pitch);
  cos_roll  = cos(AngE.Roll);
  cos_pitch = cos(AngE.Pitch);
    //姿态解算并不能准确解算偏航
	//如果地磁正常时  
    //否则 就按四元数转化来的航向角为准
	if(!flag.MagIssue && flag.MagExist){     //此处的标志位
		// 地磁倾角补偿
		fp32 hx = MAG[0]*cos_pitch + MAG[1]*sin_pitch*sin_roll - MAG[2]*cos_roll*sin_pitch; 
		fp32 hy = MAG[1]*cos_roll + MAG[2]*sin_roll;
		
		// 【利用地磁解算航向角】
		fp32 mag_yaw = -Degree(atan2((fp64)hy,(fp64)hx));
		 
		// 【陀螺仪积分解算航向角】
		AngE.Yaw += Degree(sensor.gyro.averag.z * Gyro_Gr * 2 * SampleRateHalf);
		
		// 【地磁解算的航向角与陀螺仪积分解算的航向角进行互补融合】
		if((mag_yaw>90 && AngE.Yaw<-90) || (mag_yaw<-90 && AngE.Yaw>90)) 
			AngE.Yaw = -AngE.Yaw * 0.998f + mag_yaw * 0.002f;
		else AngE.Yaw = AngE.Yaw * 0.998f + mag_yaw * 0.002f;  //互补融合
	}
	else 
        //【弧度转化为角度】
		AngE.Yaw = Degree(AngE.Yaw);     // 姿态解算出的三个姿态角  【AngE.Pitch】【此值是在多大的范围之内】
	                                     //                       【AngE.Roll 】【此处数值为度0-360     】
        AngE.Roll = Degree(AngE.Roll);   // roll                  【AngE.Yaw  】
	    AngE.Pitch = Degree(AngE.Pitch); // pitch 
}
