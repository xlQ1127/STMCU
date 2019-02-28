/******************** (C) COPYRIGHT 2014 Air Nano Team ***************************
 * 文件名  ：PARAM.c
 * 描述    ：参数加载         
 * 实验平台：Air Nano四轴飞行器
 * 库版本  ：ST3.5.0
 * 作者    ：Air Nano Team 
 * 淘宝    ：http://byd2.taobao.com
**********************************************************************************/
#include "include.h"

uint16_t VirtAddVarTab[NumbOfVar] ={0xAA00, 0xAA01, 0xAA02, 0xAA03, 0xAA04, 0xAA05, 0xAA06, 0xAA07, 0xAA08, 0xAA09, 0xAA0A, 0xAA0B, 0xAA0C, 0xAA0D, 0xAA0E, 0xAA0F,
																		0xAA10, 0xAA11, 0xAA12, 0xAA13, 0xAA14, 0xAA15, 0xAA16, 0xAA17, 0xAA18, 0xAA19, 0xAA1A, 0xAA1B, 0xAA1C, 0xAA1D, 0xAA1E, 0xAA1F,
																		0xAA20, 0xAA21, 0xAA22, 0xAA23, 0xAA24, 0xAA25, 0xAA26, 0xAA27, 0xAA28, 0xAA29, 0xAA2A, 0xAA2B, 0xAA2C, 0xAA2D, 0xAA2E, 0xAA2F,
																		0xAA30, 0xAA31, 0xAA32, 0xAA33, 0xAA34, 0xAA35, 0xAA36, 0xAA37, 0xAA38};

/*====================================================================================================*/
/*====================================================================================================*
**函数 : EE_READ_ACC_OFFSET
**功能 : 【读取加速度零偏】                                               【 读取加速度零偏 】
**输入 : None
**輸出 : None
**备注 : None
**====================================================================================================*/
/*====================================================================================================*/
static void EE_READ_ACC_OFFSET(void)
{
	EE_ReadVariable(VirtAddVarTab[0], &sensor.acc.quiet.x);
	EE_ReadVariable(VirtAddVarTab[1], &sensor.acc.quiet.y);
	EE_ReadVariable(VirtAddVarTab[2], &sensor.acc.quiet.z);
}

/*====================================================================================================*/
/*====================================================================================================*
**函数 : EE_SAVE_ACC_OFFSET                
**功能 : 【保存加速度零偏】                                                【 保存加速度零偏 】
**输入 : None
**輸出 : None
**备注 : None
**====================================================================================================*/
/*====================================================================================================*/
void EE_SAVE_ACC_OFFSET(void)
{
  EE_WriteVariable(VirtAddVarTab[0],sensor.acc.quiet.x);
  EE_WriteVariable(VirtAddVarTab[1],sensor.acc.quiet.y);
	EE_WriteVariable(VirtAddVarTab[2],sensor.acc.quiet.z);
}	
/*====================================================================================================*/
/*====================================================================================================*
**函数 : EE_READ_MAG_OFFSET
**功能 : 【读取磁力计零偏】                                                【 读取磁力计零偏 】
**输入 : None
**輸出 : None
**备注 : None
**====================================================================================================*/
/*====================================================================================================*/
static void EE_READ_MAG_OFFSET(void)
{
	EE_ReadVariable(VirtAddVarTab[3], &HMC58X3_limit[0]);
	EE_ReadVariable(VirtAddVarTab[4], &HMC58X3_limit[1]);
	EE_ReadVariable(VirtAddVarTab[5], &HMC58X3_limit[2]);
	EE_ReadVariable(VirtAddVarTab[6], &HMC58X3_limit[3]);
	EE_ReadVariable(VirtAddVarTab[7], &HMC58X3_limit[4]);
	EE_ReadVariable(VirtAddVarTab[8], &HMC58X3_limit[5]);
}
/*====================================================================================================*/
/*====================================================================================================*
**函数 : EE_SAVE_MAG_OFFSET
**功能 : 【保存磁力计零偏】                                                【 保存磁力计零偏 】
**输入 : None
**輸出 : None
**备注 : None
**====================================================================================================*/
/*====================================================================================================*/
void EE_SAVE_MAG_OFFSET(void)
{
	u8 cy;
	for(cy=0;cy<6;cy++)
    EE_WriteVariable(VirtAddVarTab[3+cy],*(mag_limt+cy));
}
/*====================================================================================================*/
/*====================================================================================================*
**函数 : EE_SAVE_Attitude_PID
**功能 : 【保存姿态PID参数】                                               【 保存姿态PID参数 】
**输入 : None
**輸出 : None
**备注 : None
**====================================================================================================*/
/*====================================================================================================*/
void EE_SAVE_Attitude_PID(void)
{
	u16 _temp;
	
	if(flag.ParamSave){
		flag.ParamSave=0;
		_temp = ctrl.pitch.core.kp * 100;
		EE_WriteVariable(VirtAddVarTab[EE_PID_CORE_PITCH_P],_temp);
		_temp = ctrl.pitch.core.ki * 1000;
		EE_WriteVariable(VirtAddVarTab[EE_PID_CORE_PITCH_I],_temp);
		_temp = ctrl.pitch.core.kd * 100;
		EE_WriteVariable(VirtAddVarTab[EE_PID_CORE_PITCH_D],_temp);
		_temp = ctrl.roll.core.kp * 100;
		EE_WriteVariable(VirtAddVarTab[EE_PID_CORE_ROLL_P],_temp);
		_temp = ctrl.roll.core.ki * 1000;
		EE_WriteVariable(VirtAddVarTab[EE_PID_CORE_ROLL_I],_temp);
		_temp = ctrl.roll.core.kd * 100;
		EE_WriteVariable(VirtAddVarTab[EE_PID_CORE_ROLL_D],_temp);
		_temp = ctrl.yaw.core.kp * 100;
		EE_WriteVariable(VirtAddVarTab[EE_PID_CORE_YAW_P],_temp);
		_temp = ctrl.yaw.core.ki * 1000;
		EE_WriteVariable(VirtAddVarTab[EE_PID_CORE_YAW_I],_temp);
		_temp = ctrl.yaw.core.kd * 100;
		EE_WriteVariable(VirtAddVarTab[EE_PID_CORE_YAW_D],_temp);
	}
}	
/*====================================================================================================*/
/*====================================================================================================*
**函数 : EE_READ_Attitude_PID
**功能 : 【读取姿态PID参数】                                                【 读取姿态PID参数 】
**输入 : None
**輸出 : None
**备注 : None
**====================================================================================================*/
/*====================================================================================================*/
static void EE_READ_Attitude_PID(void)
{
	int16_t _temp;
	EE_ReadVariable(VirtAddVarTab[EE_PID_CORE_PITCH_P],&_temp);
	
	//  参数为0  说明已经清空flash  则使用默认参数
	if(_temp==0)  return;
	ctrl.pitch.core.kp =(float)_temp / 100;	
	EE_ReadVariable(VirtAddVarTab[EE_PID_CORE_PITCH_I],&_temp);
	ctrl.pitch.core.ki =(float)_temp / 1000;
	EE_ReadVariable(VirtAddVarTab[EE_PID_CORE_PITCH_D],&_temp);
	ctrl.pitch.core.kd = (float)_temp / 100;
	EE_ReadVariable(VirtAddVarTab[EE_PID_CORE_ROLL_P],&_temp);
	ctrl.roll.core.kp =(float)_temp / 100;
	EE_ReadVariable(VirtAddVarTab[EE_PID_CORE_ROLL_I],&_temp);
	ctrl.roll.core.ki = (float)_temp / 1000;	
	EE_ReadVariable(VirtAddVarTab[EE_PID_CORE_ROLL_D],&_temp);
	ctrl.roll.core.kd = (float)_temp / 100;
	EE_ReadVariable(VirtAddVarTab[EE_PID_CORE_YAW_P],&_temp);
	ctrl.yaw.core.kp =(float)_temp / 100;
	EE_ReadVariable(VirtAddVarTab[EE_PID_CORE_YAW_I],&_temp);
	ctrl.yaw.core.ki =(float)_temp / 1000;
	EE_ReadVariable(VirtAddVarTab[EE_PID_CORE_YAW_D], &_temp);
	ctrl.yaw.core.kd =(float)_temp / 100;
}



/*====================================================================================================*/
/*====================================================================================================*
**函数 : Data_Parser()
**功能 : 【上位机发送PID解析】                                               【 上位机发送PID解析 】
**输入 : None
**輸出 : None
**备注 : None
**====================================================================================================*/
/*====================================================================================================*/
void Data_Parser(u16 *rxBuffer)
{
	// 寻找帧头
	if(*rxBuffer==0xA5 && *(rxBuffer+1)==0x5A && *(rxBuffer+2)==0x0A)	{
		
		// 寻找帧尾
		if(*(rxBuffer+10) == 0x12 && *(rxBuffer+11) == 0x34){
			
			// 判断功能字节
			switch(*(rxBuffer+3)){
				// PITCH内环参数
				case 0x30:
					ctrl.pitch.core.kp = (fp32)((vs16)(*(rxBuffer+4)<<8)|*(rxBuffer+5))/100;
				    ctrl.pitch.core.ki = (fp32)((vs16)(*(rxBuffer+6)<<8)|*(rxBuffer+7))/100;
					ctrl.pitch.core.kd = (fp32)((vs16)(*(rxBuffer+8)<<8)|*(rxBuffer+9))/100;
				  break;
				// ROLL内环参数
				case 0x31:
					ctrl.roll.core.kp = (fp32)((vs16)(*(rxBuffer+4)<<8)|*(rxBuffer+5))/100;
				    ctrl.roll.core.ki = (fp32)((vs16)(*(rxBuffer+6)<<8)|*(rxBuffer+7))/100;
					ctrl.roll.core.kd = (fp32)((vs16)(*(rxBuffer+8)<<8)|*(rxBuffer+9))/100;
					break;
				// YAW内环参数
				case 0x32:
					ctrl.yaw.core.kp = (fp32)((vs16)(*(rxBuffer+4)<<8)|*(rxBuffer+5))/100;
				    ctrl.yaw.core.ki = (fp32)((vs16)(*(rxBuffer+6)<<8)|*(rxBuffer+7))/100;
					ctrl.yaw.core.kd = (fp32)((vs16)(*(rxBuffer+8)<<8)|*(rxBuffer+9))/100;
					break;
				default: break;
			}
			
			// 参数被调节之后 需要保存
			flag.ParamSave = 1;
		}
  }		 
}

//**************************************************************************
//======================================【参数加载】 ==============================================
//**************************************************************************
void	paramLoad(void)
{
  //The data of pitch
	ctrl.pitch.shell.kp = 4;    //5     外环PI参数
	ctrl.pitch.shell.ki = 0.02;
	
	ctrl.pitch.core.kp = 1.4;   //1.5   内环PID参数
	ctrl.pitch.core.ki = 0.07;   
	ctrl.pitch.core.kd = 0.35;  //0.16
	
	//The data of roll
	ctrl.roll.shell.kp = 4;
	ctrl.roll.shell.ki = 0.02;


	ctrl.roll.core.kp = 1.4;
	ctrl.roll.core.ki = 0.07;
	ctrl.roll.core.kd = 0.35;
	
	//The data of yaw
	ctrl.yaw.shell.kp = 5;      //对偏航角YAW外环使用PD控制 偏航角不需要精准控制
	ctrl.yaw.shell.kd = 0;
	
	ctrl.yaw.core.kp = 1.8;
	ctrl.yaw.core.ki = 0;
	ctrl.yaw.core.kd = 0.1;
	
	//limit for the max increment
	ctrl.pitch.shell.increment_max = 20;        //PID的积分限幅
	ctrl.roll.shell.increment_max = 20;
	                                                           //【加速度计】【磁力计】零偏数据 保存在EEPROM中 【开机读取】
	ctrl.ctrlRate = 0;                                         //【陀螺仪】零偏数据 【开机采集】
	
    EE_READ_ACC_OFFSET();   //读取加速度零偏   校正加速度时 零偏保存在EEPROM中
	EE_READ_MAG_OFFSET();   //读取磁力计零偏
	EE_READ_Attitude_PID(); //读取内环PID参数  串级PID主要调内环
	Gyro_OFFSET();          //采集陀螺仪零偏   四轴上电需校正陀螺仪  四轴静置三四秒  【MPU6050.c】中
}
