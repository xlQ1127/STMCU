/******************** (C) COPYRIGHT 2014 Air Nano Team ***************************
 * 文件名  ：RC.c
 * 描述    ：接收遥控器数据         
 * 实验平台：HT_Hawk
 * 库版本  ：ST3.5.0
 * 作者    ：Air Nano Team 
 * 论坛    ：http://www.airnano.cn
 * 淘宝    ：http://byd2.taobao.com  
 *           http://hengtuo.taobao.com  
**********************************************************************************/
#include "MultiRotor_rc.h"
#include "include.h"

RC_GETDATA RC_Data;      //遥控器数据结构体定义

////***********************************************************************//
////typedef struct {                     //用于存放提炼的遥控数据
////	            int16_t rc_data[4];  //输入捕获四通道数据存放
////				int16_t ROLL;        //输入捕获数据处理后存放
////				int16_t PITCH;
////				int16_t THROTTLE;
////				int16_t YAW;
////				int16_t SENSITIVITY; //【SENSITIVITY】敏感度参数
////                }RC_GETDATA;   
////***********************************************************************//

////1）typedef int NUM[10];//声明整型数组类型

////    NUM n;//定义n为整型数组变量，其中n[0]--n[9]可用

////2）typedef char* STRING;//声明STRING为字符指针类型

////    STRING p,s[10];//p为字符指针变量，s为指针数组

////3）typedef int (*POINTER)();//声明POINTER为指向函数的指针类型，该函数返回整型值,没有参数

////    POINTER P1,P2;//p1，p2为POINTER类型的指针变量

// typedef void (*rcReadRawData)(void);
//注意此处函数指针的用法
rcReadRawData rcReadRawFunc = RC_Data_Refine;


void RDAU(void)             //【读取遥控器数据】
{
	RC_directive();           //关于上锁解锁
	rcReadRawFunc();          //rcReadRawFunc = RC_Data_Refine
}

/*====================================================================================================*/
/*====================================================================================================*
**函数 : RC_directive
**功能 : 【遥控指令】
**输入 : None
**輸出 : None
**备注 : None
**====================================================================================================*/
/*====================================================================================================*/
void RC_directive(void)
{
    u8 stTmp = 0,i;                    //【stTmp  i】      变量定义
	static u8  rcSticks;               //【rcSticks】      静态局部变量
	static u8  rcDelayCommand;         //【rcDelayCommand 】
    static u16 seltLockCommend;	       //【seltLockCommend】
	
	for (i = 0; i < 4; i++) {          //循环检测四通道遥控数据
			stTmp >>= 2;
			if (RC_Data.rc_data[i] > RC_MINCHECK)     //RC_MINCHECK   1200
					stTmp |= 0x80;  // check for MIN  //置位第八位     1XXX XXXX
            //各通道数值大概标定
			if (RC_Data.rc_data[i] < RC_MAXCHECK)     //RC_MAXCHECK   1800
					stTmp |= 0x40;  // check for MAX  //置位第四位     XXXX 1XXX
	}
    
	if (stTmp == rcSticks)  
    {
		    if (rcDelayCommand < 250)
					rcDelayCommand++;
	} 
    else
			rcDelayCommand = 0;
	        rcSticks = stTmp;
	
	if (rcDelayCommand == 150) 
     {
		if (flag.ARMED)    //电机解锁标志
            {
			 if (rcSticks == THR_LO + YAW_LO + PIT_CE + ROL_CE)   //【上锁】
				  flag.ARMED=0;
		    }
            
		else
            {
            if (rcSticks == THR_LO + YAW_HI + PIT_CE + ROL_CE)    //【解锁】   
					flag.ARMED=1;
            
			if (rcSticks == THR_LO + YAW_LO + PIT_CE + ROL_HI)    //【加速度矫正】  
					flag.calibratingA = 1;
            
			if ((rcSticks == THR_HI + YAW_LO + PIT_LO + ROL_HI) && flag.calibratingM_pre)  //【地磁计矫正】
				  flag.calibratingM = 1;
            
			if (rcSticks == THR_HI + YAW_HI + PIT_LO + ROL_LO)    
					flag.calibratingM_pre = 1;
			else flag.calibratingM_pre = 0;
			
            }
	}
	//武装之后一段时间油门保持最低  则自动解除武装
	if (flag.ARMED){
	   if (rcSticks == THR_LO + YAW_CE + PIT_CE + ROL_CE) {
		    if (seltLockCommend < AUTODISARMDE_TIME)
					 seltLockCommend++;
				else 
					 flag.ARMED=0;
		 }
		 else 
        seltLockCommend = 0;			 
	}
}
/*====================================================================================================*/
/*====================================================================================================*
**函数 : RcData_Refine
**功能 : 【提炼遥控数据】     提炼数据从哪里来  最终存放在哪里  中间经过了什么样的处理
**输入 : None
**输出 : None
**备注 : 无
//六通道捕获值滑动平均值处理
**====================================================================================================*/
/*====================================================================================================*/
void RC_Data_Refine(void)
{
  u8 chan,a;	

	u16 rcDataMax[6], rcDataMin[6];
	static int16_t rcDataCache[6][4], rcDataMean[6];
	static uint8_t rcValuesIndex = 0;

	rcValuesIndex++;
	for (chan = 0; chan < 6; chan++) {
		  //滑动平均值滤波，4次
		  if(RC_Pwm_In[chan]>2800 || RC_Pwm_In[chan]<800) 
              RC_Pwm_In[chan] = RC_Pwm_In_his[chan];
			  rcDataCache[chan][rcValuesIndex % 4] = RC_Pwm_In[chan];		
		      RC_Pwm_In_his[chan] = RC_Pwm_In[chan];
			
			  rcDataMean[chan] = 0;
		      rcDataMax[chan] = 0;
		      rcDataMin[chan] = 25000;
		
			for (a = 0; a < 4; a++) 
            {
				  // 记录缓存中最大值 && 最小值
				  if(rcDataCache[chan][a] > rcDataMax[chan])  rcDataMax[chan] = rcDataCache[chan][a];     
					if(rcDataCache[chan][a] < rcDataMin[chan])	rcDataMin[chan] = rcDataCache[chan][a]; 
				  // 求和
					rcDataMean[chan] += rcDataCache[chan][a];  
              }
			// 剔除缓存中 最大值 && 最小值 
			rcDataMean[chan] = (rcDataMean[chan] - (rcDataMax[chan] + rcDataMin[chan])) / 2;
	} 
//==================================================================================================//
    //数据处理后存放在RC_Data结构体和rcDataMean的数组中
	 RC_Data.YAW   = RC_Data.rc_data[2] =rcDataMean[3];             //【【遥控器最终数据存放】】
	 RC_Data.THROTTLE  = RC_Data.rc_data[3] =rcDataMean[2];
	 RC_Data.ROLL  = RC_Data.rc_data[0] = rcDataMean[0];            //【此值是在多大的范围之内】
	 RC_Data.PITCH = RC_Data.rc_data[1] = rcDataMean[1];            //【1000——2000】
}

////***********************************************************************//
////typedef struct {                     //用于存放提炼的遥控数据
////	            int16_t rc_data[4];  //输入捕获四通道数据存放
////				int16_t ROLL;        //输入捕获数据处理后存放
////				int16_t PITCH;
////				int16_t THROTTLE;
////				int16_t YAW;
////				int16_t SENSITIVITY; //【SENSITIVITY】敏感度参数
////                }RC_GETDATA;   
////***********************************************************************//