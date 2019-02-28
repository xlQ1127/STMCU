/**
  ******************************************************************************
  * @file    CeMD.h
  * @author  Creelinks Application Team
  * @version V1.0.0
  * @date    2017-03-26
  * @brief   电机电调驱动文件，即将0~1000的驱动为转化为0~100%占空比的PWM输出
  ******************************************************************************
  * @attention
  *
  *1)输入0~1000的驱动强度，输出对应为0~100%的占空比
  *
  * <h2><center>&copy; COPYRIGHT 2016 Darcern</center></h2>
  ******************************************************************************
  */
#ifndef __CE_MD_H__
#define __CE_MD_H__
#ifdef __cplusplus
 extern "C" {
#endif //__cplusplus
#include "Creelinks.h"
#define __CE_MD_VERSION__ 1                                         /*!< 此驱动文件的版本号*/
#define __CE_MD_NEED_CREELINKS_VERSION__ 1                          /*!< 需要Creelinks平台库的最低版本*/
#if (__CE_CREELINKS_VERSION__ < __CE_MD_NEED_CREELINKS_VERSION__)   /*!< 检查Creelinks平台库的版本是否满足要求*/
#error "驱动文件CeMD.h需要高于1.0以上版本的Creelink库，请登陆www.creelinks.com下载最新版本的Creelinks库。"
#else

//#define CE_MD_REVERSE                                             /*!< 根据驱动的电调类型，定义Pwm是否采用反向输出方式，即有效电平为低电平*/

#define CE_MD_MAX_PWM_CYCLE_NS  50000                               /*!< 根据驱动的电调类型，定义Pwm输出的最大周期，单位Ns*/
#define CE_MD_MIN_PWM_CYCLE_NS  0                                   /*!< 根据驱动的电调类型，定义Pwm输出的最小周期，单位Ns*/

#define CE_MD_MAX_PWM_DUTY_NS   50000                               /*!< 根据驱动的电调类型，定义Pwm输出的最大占空比，单位Ns*/
#define CE_MD_MIN_PWM_DUTY_NS   0                                   /*!< 根据驱动的电调类型，定义Pwm输出的最小占空比，单位Ns*/

/*
 *CeMD属性对像
 */
typedef struct
{
    CePwm cePwm;                                                    /*!< 模块使用到的处理器Pwm资源*/
}CeMD;
/*
 *CeMD操作对像
 */
typedef struct
{
    CE_STATUS (*initial)(CeMD* ceMD, CE_RESOURCE cePwm);            /*!< @brief CeMD模块初始化
                                                                         @param ceMD:CeMD属性对象指针
                                                                         @param ceXX:CeMD模块使用的资源号*/
                                                                         
    void      (*setDriverPower)(CeMD* ceMD, uint16 driverPower);    /*!< @brief 设置Pwm的驱动强度，0~1000，对应占空比为0%~100%
                                                                         @param ceMD:CeMD属性对象指针
                                                                         @param driverPower:Pwm的驱动强度，0~1000，对应占空比为0%~100%*/
}CeMDOp;
/*
 *CeMD操作对象实例
 */
extern const CeMDOp ceMDOp;

#endif // (__CE_CREELINKS_VERSION__ < __CE_MD_NEED_CREELINKS_VERSION__)
#ifdef __cplusplus
 }
#endif //__cplusplus
#endif //__CE_MD_H__

