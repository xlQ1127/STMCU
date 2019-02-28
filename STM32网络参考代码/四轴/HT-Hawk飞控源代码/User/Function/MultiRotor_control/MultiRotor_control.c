/*
******************* (C) COPYRIGHT 2015 Air Nano Team ***************************
 * 模块名称 : 控制程序
 * 文件名   ：
 * 描述     ：    
 * 实验平台 ：HT_Hawk
 * 库版本   ：ST3.5.0
 * 作者     ：Air Nano Team 
 * 论坛    ：http://www.airnano.cn 
 * 淘宝     ：http://byd2.taobao.com   
 *            http://hengtuo.taobao.com   
*********************************************************************************
*/
#include "include.h"                    //  姿态解算得出三个姿态角单位是度
#include "MultiRotor_control.h"         //  提炼遥控器得到四个通道遥控值范围是1000-2000
                                        //  它们单位不同如何传入PID函数控制
                                        //  需要先对遥控数据进行处理得到可用的目标期望值
struct _ctrl ctrl;                      // 【怎样处理】
struct _target Target;

s16 Moto_duty[MOTOR_NUM];
s16 *motor_array = Moto_duty;

/*====================================================================================================*/
/*====================================================================================================*
**函数 : Calculate_target        【提炼遥控器数据之后经一定处理 】
**功能 : 计算目标量              【方可得到目标量               】
**输入 : None                    【传入PID函数进而得到电机输出量】
**輸出 : None                    【    遥控器数据————目标量     】        
**备注 : None
**====================================================================================================*/
/*====================================================================================================*/
void Calculate_Target(void) //=======此函数修改【target】结构体=======//
{
	int16_t ftemp=0;   //此处并不是需要360°控制  只需要在小角度范围控制  
	                   //遥控原始数据【1000——2000】
	Target.Pitch = (1500-RC_Data.PITCH)/(20 + 7*RC_Data.SENSITIVITY);  // 期望的目标量   【Target.Pitch】
	Target.Roll = (RC_Data.ROLL-1500)/(20 + 7*RC_Data.SENSITIVITY);    //                【Target.Roll 】
                     //±500范围  除以固定数值进一步减小其范围   调节RC_Data.SENSITIVITY值即可调节遥控器灵敏度
	
  //目标航向控制。当油门大于最小检查值时，认为用户希望起飞。那么此时的航向做为目标航向
	
   if(RC_Data.THROTTLE > RC_MINCHECK ) {  // 如果油门值大于最小起飞限定值  此处为油门原始值1200——1800
      if(flag.LockYaw != 1){              // 航向锁定
				 flag.LockYaw = 1;
	       Target.Yaw = AngE.Yaw; //将当前的航向做为目标航向                             【 Target.Yaw 】
      }
   }
   else {
		 flag.LockYaw = 0;	        //起飞锁定航向
		 Target.Yaw = AngE.Yaw;     //将当前的航向做为目标航向                          【航向角的处理较为繁琐】
	 } 
	//航向在中点设置一个死区
	if((RC_Data.YAW > 1600)||(RC_Data.YAW < 1400)){     // 不轻易改变航向角
		ftemp = 1500 - RC_Data.YAW; 
	  Target.Yaw += (ftemp / 200.0f)*0.1f;              // 目标航向角加遥控器数据
		
		//转[-180.0,+180.0]
	  if(Target.Yaw >180.0f) Target.Yaw -= 360.0f;	
	  else if(Target.Yaw <-180.0f)Target.Yaw += 360.0f;
	}
}

/*====================================================================================================*/
/*====================================================================================================*
**函数 : CONTROL(struct _target Goal) 
**功能 : 串级PID 控制
**输入 : Goal
**輸出 : None
**备注 : None
//       PI控制
**====================================================================================================*/
/*====================================================================================================*/
void CONTROL(struct _target Goal)      //传入实参 【target】
{
	float  deviation_pitch,deviation_roll,deviation_yaw;  //偏差定义
	
	if(ctrl.ctrlRate >= 2)             //==================================【内环两次PID控制 外环进行一次PID控制】
	{
		                               //==================================【【外环(角度环)PID】】
		//【俯仰计算】                  //【目标角度——测出角度】
	    deviation_pitch = Goal.Pitch - AngE.Pitch;
		ctrl.pitch.shell.increment += deviation_pitch;   //横滚角误差积分
		
		//limit for the max increment  积分限幅
		ctrl.pitch.shell.increment = data_limit(ctrl.pitch.shell.increment,ctrl.pitch.shell.increment_max,-ctrl.pitch.shell.increment_max);
    //==========================================【横滚角PI输出】==============================================
		ctrl.pitch.shell.pid_out = ctrl.pitch.shell.kp * deviation_pitch + ctrl.pitch.shell.ki * ctrl.pitch.shell.increment;
		
		//【横滚计算】
		deviation_roll = Goal.Roll - AngE.Roll;
		ctrl.roll.shell.increment += deviation_roll;     //俯仰角误差积分
		
		//limit for the max increment  积分限幅
		ctrl.roll.shell.increment = data_limit(ctrl.roll.shell.increment,ctrl.roll.shell.increment_max,-ctrl.roll.shell.increment_max);
    //==========================================【俯仰角PI输出】==============================================
		ctrl.roll.shell.pid_out  = ctrl.roll.shell.kp * deviation_roll + ctrl.roll.shell.ki * ctrl.roll.shell.increment;
		
		//【航向计算】
       if((Goal.Yaw - AngE.Yaw)>180 || (Goal.Yaw - AngE.Yaw)<-180){
       if(Goal.Yaw>0 && AngE.Yaw<0)  deviation_yaw= (-180 - AngE.Yaw) +(Goal.Yaw - 180);
       if(Goal.Yaw<0 && AngE.Yaw>0)  deviation_yaw= (180 - AngE.Yaw) +(Goal.Yaw + 180);
    }
    else  deviation_yaw = Goal.Yaw - AngE.Yaw; //航向角误差       此处对航向角的要求不是很严格
		//===========================================【航向角P输出】==============================================
	  ctrl.yaw.shell.pid_out = ctrl.yaw.shell.kp * deviation_yaw;
      ctrl.ctrlRate = 0; 
	}
	ctrl.ctrlRate ++;
    Attitude_RatePID();
	Motor_Conter();
}

/*====================================================================================================*/
/*====================================================================================================*
**函数 : Attitude_RatePID
**功能 : 角速率控制PID    【【内环】】
**输入 : None
**輸出 : None
**备注 : None
**       PID控制
**====================================================================================================*/
/*====================================================================================================*/
void Attitude_RatePID(void)
{
    fp32 E_pitch,E_roll,E_yaw;
	
	            // 计算偏差  
	E_pitch = ctrl.pitch.shell.pid_out - sensor.gyro.averag.y;
	E_roll  = ctrl.roll.shell.pid_out  - sensor.gyro.averag.x;
	E_yaw   = ctrl.yaw.shell.pid_out   - sensor.gyro.averag.z;
	
	            // 积分
	ctrl.pitch.core.increment += E_pitch;
	ctrl.roll.core.increment  += E_roll;
	ctrl.yaw.core.increment   += E_yaw;
	
	            // 积分限幅
	ctrl.pitch.core.increment = data_limit(ctrl.pitch.core.increment,20,-20);
	ctrl.roll.core.increment  = data_limit(ctrl.roll.core.increment,20,-20);		
	ctrl.yaw.core.increment   = data_limit(ctrl.yaw.core.increment,20,-20);
	//=========================================================================================================//
	                          //比例控制
	ctrl.pitch.core.kp_out = ctrl.pitch.core.kp * E_pitch;
	ctrl.roll.core.kp_out  = ctrl.roll.core.kp  * E_roll;
	ctrl.yaw.core.kp_out   = ctrl.yaw.core.kp   * E_yaw;
	                          //积分控制
	ctrl.pitch.core.ki_out = ctrl.pitch.core.ki * ctrl.pitch.core.increment;
    ctrl.roll.core.ki_out  = ctrl.roll.core.ki  * ctrl.roll.core.increment;
	ctrl.yaw.core.ki_out   = ctrl.yaw.core.ki   * ctrl.yaw.core.increment;
	
	                          //微分控制
	ctrl.pitch.core.kd_out = ctrl.pitch.core.kd * (sensor.gyro.histor.y - sensor.gyro.averag.y)*33;
	ctrl.roll.core.kd_out  = ctrl.roll.core.kd  * (sensor.gyro.histor.x - sensor.gyro.averag.x)*33;
	ctrl.yaw.core.kd_out   = ctrl.yaw.core.kd   * (sensor.gyro.histor.z - sensor.gyro.averag.z)*33;	
	
	sensor.gyro.histor.y = sensor.gyro.averag.y;
	sensor.gyro.histor.x = sensor.gyro.averag.x; 
    sensor.gyro.histor.z = sensor.gyro.averag.z;	
	//=========================================================================================================//
	//内环PID输出
	ctrl.pitch.core.pid_out = ctrl.pitch.core.kp_out + ctrl.pitch.core.ki_out + ctrl.pitch.core.kd_out;
	ctrl.roll.core.pid_out  = ctrl.roll.core.kp_out  + ctrl.roll.core.ki_out  + ctrl.roll.core.kd_out;
	ctrl.yaw.core.pid_out   = ctrl.yaw.core.kp_out   + ctrl.yaw.core.kd_out;
	//=========================================================================================================//
	//总PID输出 内外环PID输出与外环PID输出互补融合
	ctrl.pitch.core.pid_out = ctrl.pitch.core.pid_out*0.8 + ctrl.pitch.shell.pid_out/2;
	ctrl.roll.core.pid_out  = ctrl.roll.core.pid_out *0.8 + ctrl.roll.shell.pid_out/2; 
	ctrl.yaw.core.pid_out   = ctrl.yaw.core.pid_out;
  //《PID输出数值的范围是多大》
}
/*====================================================================================================*/
/*====================================================================================================*
**函数 : Motor_Conter(void)
**功能 : 电机控制
**输入 : None
**輸出 : None
**备注 : None
**====================================================================================================*/
/*====================================================================================================*/
void Motor_Conter(void)
{
	s16 pitch,roll,yaw;
	
	pitch = ctrl.pitch.core.pid_out;
    roll  = ctrl.roll.core.pid_out;    
 	yaw   = -ctrl.yaw.core.pid_out;
    //遥控器输出油门量1000-2000  遥控器输出油门量需经处理后作为电机输出油门量
  if(RC_Data.THROTTLE > RC_MINCHECK) {  //如果油门值大于最小起飞标定值 确定可以起飞 油门原始值1200-1800
		int date_throttle	= (RC_Data.THROTTLE-1000)/cos(AngE.Roll/RtA)/cos(AngE.Pitch/RtA);  //油门曲线
		//原始油门为线性 响应过快 需经一定手段处理使其变为非线性
		#ifdef QUADROTOR           //四轴
			Moto_duty[0] = date_throttle - pitch - roll + yaw;       //【姿态控制是在油门的基础之上的】
			Moto_duty[1] = date_throttle - pitch + roll - yaw;    
			Moto_duty[2] = date_throttle + pitch + roll + yaw;       
			Moto_duty[3] = date_throttle + pitch - roll - yaw;
		#elif defined HEXRCOPTER   //六轴
			Moto_duty[0] = date_throttle - pitch + 0.5*roll - yaw;
			Moto_duty[1] = date_throttle         +     roll + yaw;
			Moto_duty[2] = date_throttle + pitch + 0.5*roll - yaw;
			Moto_duty[3] = date_throttle + pitch - 0.5*roll + yaw;	
			Moto_duty[4] = date_throttle         -     roll - yaw;
			Moto_duty[5] = date_throttle - pitch - 0.5*roll + yaw;	
		#endif 	
	}
	else
	{	                                              //如果油门值小于最小起飞限定值
		array_assign(&Moto_duty[0],IDLING,MOTOR_NUM); //数组写入函数 电机任务数组 都写为电机怠速值
		Reset_Integral();		//积分清零
	}
	if(flag.ARMED)  moto_PwmRflash(&Moto_duty[0]);	//解锁后电机才可以转	
	else            moto_STOP();	                  //不解锁电机不转
}

/*====================================================================================================*/
/*====================================================================================================*
**函数 : Reset_Integral
**功能 : 积分清零  电机未起飞的时候PID积分项清零
**输入 : None
**輸出 : None
**备注 : None
**====================================================================================================*/
/*====================================================================================================*/
void Reset_Integral(void)
{
	ctrl.pitch.shell.increment = 0;  //外环
	ctrl.roll.shell.increment= 0;	
    ctrl.pitch.core.increment = 0;	 //内环
    ctrl.roll.core.increment = 0;		
	ctrl.yaw.core.increment = 0;
}

