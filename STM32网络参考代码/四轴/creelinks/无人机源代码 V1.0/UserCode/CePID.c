/**
  ******************************************************************************
  * @file    CePID.c
  * @author  Creelinks Application Team
  * @version V1.0.0
  * @date    2017-03-26
  * @brief   无人机PID参数调整功能模块
  ******************************************************************************
  * @attention
  *
  *1)输入当前无人机姿态（Pitch/Roll/Yaw）、高度、期望油门。
  *2)输出四个电机的驱动强度，0~1000.
  *3)默认PID参数，需在cePID_initial函数中初始化并赋值
  *
  * <h2><center>&copy; COPYRIGHT 2016 Darcern</center></h2>
  ******************************************************************************
  */
#include "CePID.h"
#include <math.h>
#ifdef __cplusplus
 extern "C" {
#endif //__cplusplus

CePID cePID;                //定义全局PID控制器对象
/**
  * @brief  CePID控制器模块初始化
  * @param  cePackageSend:数据打包并发送使用的结构体
  * @param  cePackageRecv:数据拆包并解析使用的结构体
  */
void cePID_initial(CePackageSend* cePackageSend, CePackageRecv* cePackageRecv)
{
    //当前版本需要在此处配置PID各个参数，下一版本将直接保存到FLASH当中
    cePID.outPitchP = 6.000f;        
    cePID.outPitchI = 0.010f;
    cePID.outPitchD = 0.000f;
    cePID.outPitchError = 0.000f;
    cePID.lastOutPitch = 0.000f;
    cePID.inPitchP = 0.800f;
    cePID.inPitchI = 0.000f;
    cePID.inPitchD = 21.000f;
    cePID.inPitchError = 0.000f;
    cePID.lastInPitchGyrY = 0.000f;

    cePID.outRollP = 6.000f;
    cePID.outRollI = 0.010f;
    cePID.outRollD = 0.000f;
    cePID.outRollError = 0.000f;
    cePID.lastOutRoll = 0.000f;
    cePID.inRollP = 0.800f;
    cePID.inRollI = 0.000f;
    cePID.inRollD = 21.000f;
    cePID.inRollError = 0.000f;
    cePID.lastInRollGyrX = 0.000f;

    cePID.outYawP = 0.000f;                
    cePID.outYawI = 0.000f;                       
    cePID.outYawD = 0.000f;
    cePID.outYawError = 0.000f;                    
    cePID.outPidYaw = 0.000f;            
    cePID.lastOutYaw = 0.000f;
    cePID.inYawP = 6.000f;                       
    cePID.inYawI = 0.000f;                       
    cePID.inYawD = 10.000f;                      
    cePID.inYawError = 0.000f;               
    cePID.inPidYaw = 0.000f;                  
    cePID.lastInYawGyrZ = 0.000f;                  

    cePID.altBase = 400;
    cePID.altKp = 100;
    cePID.altKi = 0.2;
    cePID.altKd = 0;
    cePID.altError = 0;
    cePID.lastAltError = 0;

    cePID.drivePower.driverPower0 = 0;
    cePID.drivePower.driverPower1 = 0;
    cePID.drivePower.driverPower2 = 0;
    cePID.drivePower.driverPower3 = 0;

    cePID.drivePowerZero.driverPower0 = 0;//细调电机驱动零点
    cePID.drivePowerZero.driverPower1 = 0;
    cePID.drivePowerZero.driverPower2 = 0;
    cePID.drivePowerZero.driverPower3 = 0;

    cePID.cePackageSend = cePackageSend;  //保存发送打包属性结构体，用于传输姿态解算中间数值以供观察
    cePID.cePackageRecv = cePackageRecv;
}

/**
  * @brief  根据cePackageRecv中的内容，如果处于PID调节模式下，则更新PID参数
  */
void cePID_updataPIDParment(void)
{
    if ((cePID.cePackageRecv->status & CE_PID_IN_DEBUG) != 0)//如果处于PID调节状态，则更新当前各PID参数
    {
        cePID.outPitchP = (fp32)(cePID.cePackageRecv->outPitchP) / 1000;//发送的数据经过放大1000倍，故这里要再除以1000。放大原因是便于将fp32型数据转换为Int16并进行数据传输
        cePID.outPitchI = (fp32)(cePID.cePackageRecv->outPitchI) / 1000;
        cePID.outPitchD = (fp32)(cePID.cePackageRecv->outPitchD) / 1000;
        cePID.inPitchP = (fp32)(cePID.cePackageRecv->inPitchP) / 1000;
        cePID.inPitchI = (fp32)(cePID.cePackageRecv->inPitchI) / 1000;
        cePID.inPitchD = (fp32)(cePID.cePackageRecv->inPitchD) / 1000;

        cePID.outRollP = (fp32)(cePID.cePackageRecv->outRollP) / 1000;
        cePID.outRollI = (fp32)(cePID.cePackageRecv->outRollI) / 1000;
        cePID.outRollD = (fp32)(cePID.cePackageRecv->outRollD) / 1000;
        cePID.inRollP = (fp32)(cePID.cePackageRecv->inRollP) / 1000;
        cePID.inRollI = (fp32)(cePID.cePackageRecv->inRollI) / 1000;
        cePID.inRollD = (fp32)(cePID.cePackageRecv->inRollD) / 1000;

        cePID.outYawP = (fp32)(cePID.cePackageRecv->outYawP) / 1000;
        cePID.outYawI = (fp32)(cePID.cePackageRecv->outYawI) / 1000;
        cePID.outYawD = (fp32)(cePID.cePackageRecv->outYawD) / 1000;
        cePID.inYawP = (fp32)(cePID.cePackageRecv->inYawP) / 1000;
        cePID.inYawI = (fp32)(cePID.cePackageRecv->inYawI) / 1000;
        cePID.inYawD = (fp32)(cePID.cePackageRecv->inYawD) / 1000;

        cePID.drivePowerZero.driverPower0 = cePID.cePackageRecv->driverPower0Zero;
        cePID.drivePowerZero.driverPower1 = cePID.cePackageRecv->driverPower1Zero;
        cePID.drivePowerZero.driverPower2 = cePID.cePackageRecv->driverPower2Zero;
        cePID.drivePowerZero.driverPower3 = cePID.cePackageRecv->driverPower3Zero;

        cePID.altBase = (fp32)(cePID.cePackageRecv->altBase);
        cePID.altKp = (fp32)(cePID.cePackageRecv->altKp) / 1000;
        cePID.altKi = (fp32)(cePID.cePackageRecv->altKi) / 1000;
        cePID.altKd = (fp32)(cePID.cePackageRecv->altKd) / 1000;
    }
}

/**
  * @brief  根据当前无人机加速度、角速度、姿态角、期望姿态角，直接计算出四个电调应有的驱动强度
  * @param  ceNowAcc:当前无人机三轴加速度数据，单位G
  * @param  ceNowGyr:当前无人机三轴角速度数据，单位度/s
  * @param  ceNowAngles:当前无人机姿态角数据
  * @param  ceHopeAngles:期望无人机处于的姿态角数据
  * @param  dtS:程序执行周期，单位S
  */
CeDrivePower* cePID_calculate(CeAcc* ceNowAcc, CeGyr* ceNowGyr, CeAngles* ceNowAngles, CeAngles* ceHopeAngles,fp32 dtS)
{
    fp32 outPitchErrorNow;
    fp32 inPitchErrorNow;
    fp32 outRollErrorNow;
    fp32 inRollErrorNow;
    fp32 outYawErrorNow;
        
    cePID_updataPIDParment();       //根据cePackageRecv中的内容，如果地面站启动了PID参数调整，则更新PID参数

    ceTaskOp.inCriticalSection(); //进入代码临界段（不清楚的可以百度），在操作系统环境下以下内容执行过程中禁止任务切换，防止其它线程修改当前角、加、姿态等数据。裸奔下则可忽略。

    if(ceNowAngles->accelerator < CE_PID_MAX_DRIVER_POWER*2/10) //当油门小于2/10的强度时，清空积分项，即不进行积分操作
    {
        cePID.outPitchError = 0;
        cePID.inPitchError = 0;
        cePID.outRollError = 0;
        cePID.inRollError = 0;
        cePID.outYawError = 0;
        cePID.inYawError = 0;
    }
     /****Pitch计算*********************************************************************/
    outPitchErrorNow = ceHopeAngles->picth - ceNowAngles->picth;                                    //当前Pitch角度误差=期望角度-当前角度 
    cePID.outPitchError += outPitchErrorNow;                                                        //外环误差累计积分项                                                
    if(cePID.outPitchError > (fp32)(500)) cePID.outPitchError = (fp32)(500);                        //限制外环累积误差最大值
    if(cePID.outPitchError < (fp32)(-500)) cePID.outPitchError = (fp32)(-500);
    cePID.outPidPitch = cePID.outPitchP * outPitchErrorNow + cePID.outPitchI * cePID.outPitchError+ cePID.outPitchD*(ceNowAngles->picth - cePID.lastOutPitch); //外环PID输出=外环Kp * 当前Pitch角度误差 + 外环Ki * 外环累积误差    + 外环Kd * （当前Y轴角速度 - 上一次Y轴角速度）        
    cePID.lastOutPitch = ceNowAngles->picth;

    inPitchErrorNow = -(cePID.outPidPitch - ceNowGyr->y);                                           //内环当前角速度误差=外环PID输出-当前y轴角速度误差,取负值原因为传感器安装位置导致                                                             
    cePID.inPitchError += inPitchErrorNow;                                                          //内环误差累计积分项
    if(cePID.inPitchError > (fp32)(500)) cePID.inPitchError = (fp32)(500);                          //限制内环累积误差最大值
    if(cePID.inPitchError < (fp32)(-500)) cePID.inPitchError = (fp32)(-500);
    cePID.inPidPitch = cePID.inPitchP * inPitchErrorNow + cePID.inPitchI * cePID.inPitchError + cePID.inPitchD * (ceNowGyr->y - cePID.lastInPitchGyrY);//内环PID输出 = 内环Kp*内环当前角速度误差 + 内环Ki*内环角速度误差累积 + 内环Kd*(当前Y轴角速度 - 上一次Y轴角速度)
    cePID.lastInPitchGyrY = ceNowGyr->y;                                                            //保存上一次角速度
                
    /****Roll计算*********************************************************************/
    outRollErrorNow = ceHopeAngles->roll - ceNowAngles->roll;                                       //当前Roll角度误差=期望角度-当前角度
    cePID.outRollError += outRollErrorNow;                                                          //外环误差累计积分项
    if(cePID.outRollError > (fp32)(500)) cePID.outRollError = (fp32)(500);                          //限制外环累积误差最大值
    if(cePID.outRollError < (fp32)(-500)) cePID.outRollError = (fp32)(-500);
    cePID.outPidRoll = cePID.outRollP * outRollErrorNow + cePID.outRollI * cePID.outRollError + cePID.outRollD * (ceNowAngles->roll - cePID.lastOutRoll);      //外环PID输出=外环Kp * 当前Roll角度误差 + 外环Ki * 外环累积误差    + 外环Kd * （当前X轴角速度 - 上一次X轴角速度）    
    cePID.lastOutRoll = ceNowAngles->roll; 


    inRollErrorNow = -(cePID.outPidRoll  - ceNowGyr->x);                                            //内环当前角速度误差=外环PID输出-当前y轴角速度误差,取负值原因为传感器安装位置导致。                                                         
    cePID.inRollError +=inRollErrorNow;                                                             //内环误差累计积分项
    if(cePID.inRollError > (fp32)(500)) cePID.inRollError = (fp32)(500);                            //限制内环累积误差最大值
    if(cePID.inRollError < (fp32)(-500)) cePID.inRollError = (fp32)(-500);
    cePID.inPidRoll = cePID.inRollP * inRollErrorNow + cePID.inRollI * cePID.inRollError + cePID.inRollD * (ceNowGyr->x - cePID.lastInRollGyrX);//内环PID输出 = 内环Kp*内环当前角速度误差 + 内环Ki*内环角速度误差累积 + 内环Kd*(当前y轴角速度误差-上一次角速度误差)
    cePID.lastInRollGyrX = ceNowGyr->x;                                                             //保存上一次角速度    

    /****Yaw计算*********************************************************************/
    outYawErrorNow = ceHopeAngles->yaw - ceNowGyr->z;                                               //yaw角速度误差 = 期望Z轴角速度 - 当前Z轴角速度
    cePID.inYawError += outYawErrorNow;
    if(cePID.inYawError > (fp32)(500)) cePID.inYawError = (fp32)(500);                              //限制内环累积误差最大值
    if(cePID.inYawError < (fp32)(-500)) cePID.inYawError = (fp32)(-500);        
    cePID.inPidYaw = cePID.inYawP * outYawErrorNow + cePID.inYawI * cePID.inYawError+cePID.inYawD * (ceNowGyr->z - cePID.lastInYawGyrZ);//yaw航向角PID输出 = Z轴Kp项 * 当前Z轴角速度误差 + 内环Ki*内环角速度误差累积 + Z轴Kd项 * （当前yaw角速度误差 - 上一次yaw角速度误差）； 
    cePID.lastInYawGyrZ = ceNowGyr->z;

    /****油门计算*********************************************************************/
    if(ceHopeAngles->altitude == -9999)//当前处于自由飞行模式，配置油门为阻尼状态，防止油门突然增加而翻机或突然减小而摔机
    {
        if(ceNowAngles->accelerator < (ceHopeAngles->accelerator-dtS*2000) && ceNowAngles->accelerator < CE_PID_MAX_DRIVER_POWER*7/10 - dtS*2000) //配置油门上升为阻尼工作状态，防止油门突然加大而导致不稳定
            ceNowAngles->accelerator += dtS*2000;//控制油门增加速度为 2/ms
        if(ceNowAngles->accelerator > (ceHopeAngles->accelerator+dtS*1000) && ceNowAngles->accelerator >= dtS*1000 ) //配置油门下降为阴尼工作状态，防止下降速度过快导致摔机
            ceNowAngles->accelerator -= dtS*1000;//控制油门减少速度为 1/ms    
    }else     //当前无人机处于定高飞行状态，即期望高度不为－9999
    {
        fp32 altError = (ceHopeAngles->altitude - ceNowAngles->altitude);                           //海拔误差 = 期望海拔高度 - 当前海拔高度 
        if(altError > 2.0f) altError = 2.0f;                                                        //限制海拔误差最大值
        else if(altError < -2.0f) altError = -2.0f; 
        cePID.altError += altError;                                                                 //海拔误差积分累积
        if(cePID.altError > (fp32)(500)) cePID.altError = (fp32)(500);                              //限制误差累积积分项
        if(cePID.altError < (fp32)(-500)) cePID.altError = (fp32)(-500);        
        cePID.altPid = altError*cePID.altKp + cePID.altError * cePID.altKi;                         //海拔PID输出 = 海拔误差*Kp + 海拔误差积分* Ki
        ceNowAngles->accelerator = cePID.altBase + cePID.altPid;                                    //海拔最终输出 = 海拔PID输出 + 油门基值
        if(ceNowAngles->accelerator < cePID.altBase - 20 ) ceNowAngles->accelerator = cePID.altBase - 20 ;//限制油门最小值，防止因海拔高度突变引起的摔机
        if(ceNowAngles->accelerator > CE_PID_MAX_DRIVER_POWER*6/10 ) ceNowAngles->accelerator = CE_PID_MAX_DRIVER_POWER*6/10;//限制油门最大值，防止无人机突然快速上升
    }

    cePID.drivePower.driverPower0 = ceNowAngles->accelerator + cePID.inPidPitch + cePID.inPidRoll + cePID.inPidYaw + cePID.drivePowerZero.driverPower0;//整合Pitch、Roll、Yaw、油门/海拔 进行PID运算出的结果
    cePID.drivePower.driverPower1 = ceNowAngles->accelerator + cePID.inPidPitch - cePID.inPidRoll - cePID.inPidYaw + cePID.drivePowerZero.driverPower1;
    cePID.drivePower.driverPower2 = ceNowAngles->accelerator - cePID.inPidPitch - cePID.inPidRoll + cePID.inPidYaw + cePID.drivePowerZero.driverPower2;
    cePID.drivePower.driverPower3 = ceNowAngles->accelerator - cePID.inPidPitch + cePID.inPidRoll - cePID.inPidYaw + cePID.drivePowerZero.driverPower3;

    if(cePID.drivePower.driverPower0 > CE_PID_MAX_DRIVER_POWER){cePID.drivePower.driverPower0 = CE_PID_MAX_DRIVER_POWER;}  //限制电机电调驱动强度在允许范围
    if(cePID.drivePower.driverPower1 > CE_PID_MAX_DRIVER_POWER){cePID.drivePower.driverPower1 = CE_PID_MAX_DRIVER_POWER;};
    if(cePID.drivePower.driverPower2 > CE_PID_MAX_DRIVER_POWER){cePID.drivePower.driverPower2 = CE_PID_MAX_DRIVER_POWER;};
    if(cePID.drivePower.driverPower3 > CE_PID_MAX_DRIVER_POWER){cePID.drivePower.driverPower3 = CE_PID_MAX_DRIVER_POWER;};

    if(cePID.drivePower.driverPower0 < CE_PID_MIN_DRIVER_POWER){cePID.drivePower.driverPower0 = CE_PID_MIN_DRIVER_POWER;};  //限制电机电调驱动强度在允许范围
    if(cePID.drivePower.driverPower1 < CE_PID_MIN_DRIVER_POWER){cePID.drivePower.driverPower1 = CE_PID_MIN_DRIVER_POWER;};
    if(cePID.drivePower.driverPower2 < CE_PID_MIN_DRIVER_POWER){cePID.drivePower.driverPower2 = CE_PID_MIN_DRIVER_POWER;};
    if(cePID.drivePower.driverPower3 < CE_PID_MIN_DRIVER_POWER){cePID.drivePower.driverPower3 = CE_PID_MIN_DRIVER_POWER;};

    ceTaskOp.outCriticalSection();                                      //退出代码临界段

    cePID.cePackageSend->driverPower0 = cePID.drivePower.driverPower0;  //将四个电机的驱动强度数据发送给地面站，以观察波形数据
    cePID.cePackageSend->driverPower1 = cePID.drivePower.driverPower1;
    cePID.cePackageSend->driverPower2 = cePID.drivePower.driverPower2;
    cePID.cePackageSend->driverPower3 = cePID.drivePower.driverPower3;

    return &(cePID.drivePower);
}

/**
  * @brief  CePID模块操作对象定义
  */
const CePIDOp cePIDOp = {cePID_initial,cePID_calculate};

#ifdef __cplusplus
 }
#endif //__cplusplus
