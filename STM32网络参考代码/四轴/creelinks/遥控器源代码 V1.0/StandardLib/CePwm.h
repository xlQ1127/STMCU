/**
  ******************************************************************************
  * @file    CePwm.h
  * @author  Creelinks Application Team
  * @version V1.0.0
  * @date    2016-08-05
  * @brief   Creelinks平台CePwm头文件
  ******************************************************************************
  * @attention
  *
  *1)由于受限于各个处理器平台，Creelinks不能保证所有的Pwm资源均单独配置周期及占空比，部分处理器平台可能会出现多个
  *  Pwm资源周期必须相同的约定，详细可查看CePwm.c中的描述
  *2)
  * <h2><center>&copy; COPYRIGHT 2016 Darcern</center></h2>
  ******************************************************************************
  */
#ifndef __CE_PWM_H__
#define __CE_PWM_H__

#include "CeMcu.h"

#ifdef __cplusplus
 extern "C" {
#endif //__cplusplus

/**
  * @brief  结构体，Pwm对象可用属性集合
  */
typedef struct
{
    CE_RESOURCE     ceResource;                     /*!< Pwm对应的资源号*/
    uint32          cycleNs;                        /*!< Pwm的周期，单位ns*/
    uint32          dutyNs;                         /*!< Pwm的高电平时间，单位ns*/
    CeExPwmPar      ceExPwmPar;                     /*!< 与处理器平台相关的额外参数结构体，用以提高代码效率，用户无须关注*/
}CePwm;

/**
  * @brief  结构体，Pwm对象可用操作集合
  */
typedef struct
{
    CE_STATUS   (*initial)(CePwm* cePwm);           /*!< @brief 初始化Pwm
                                                         @param cePwm:Pwm属性对象*/

    void        (*start)(CePwm* cePwm);             /*!< @brief 开始Pwm输出
                                                         @param cePwm:Pwm属性对象*/

    void        (*updata)(CePwm* cePwm);            /*!< @brief 更新Pwm参数
                                                         @param cePwm:Pwm属性对象*/

    void        (*stop)(CePwm* cePwm);              /*!< @brief 停止Pwm输出
                                                         @param cePwm:Pwm属性对象*/

    void        (*setBit)(CePwm* cePwm);            /*!< @brief 设置Pwm口的输出为高电平。注意：只能在Pwm没有输出时进行此操作！
                                                         @param cePwm:Pwm属性对象*/

    void        (*resetBit)(CePwm* cePwm);          /*!< @brief 设置Pwm口的输出为低电平。注意：只能在Pwm没有输出时进行此操作！
                                                         @param cePwm:Pwm属性对象*/
}CePwmOp;
extern const CePwmOp cePwmOp;                       /*!< 所有与Pwm相关的操作*/

#ifdef __cplusplus
 }
#endif //__cplusplus
#endif  //__CE_PWM_H__

/**
******************************************************************************
* @brief  使用流程及示例程序(基于前后台非操作系统环境)
* @function 使用一路Pwm，并输出1Khz的，占空比为25%的方波
******************************************************************************
#include "Creelinks.h"
CePwm myPwm;                                    //定义Pwm属性对象
int main(void)
{
    ceSystemOp.initial();                       //Creelinks环境初始化
    ceDebugOp.initial(Uartx);                        //通过Uart串口输出Debug信息到上位机
    //TODO:请在此处插入模块初始化等操作
    myPwm.ceResource = RxP;                     //指定Pwm属性对象使用的资源号
    myPwm.cycleNs = 1000000;                    //指定Pwm输出的周期，单位纳秒
    myPwm.dutyNs = myPwm.cycleNs / 4;           //指定Pwm输出的占空比高电平持续时间，单位纳秒
    cePwmOp.initial(&myPwm);                    //初始化Pwm输出
    cePwmOp.start(&myPwm);                      //开始Pwm输出波形
    while (1)
    {
        ceTaskOp.mainTask();                    //Creelinks环境主循环任务，请保证此函数能够被周期调用
        //TODO:请在此处插入用户操作
    };
}
******************************************************************************
*/
