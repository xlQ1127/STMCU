/**
  ******************************************************************************
  * @file    CeSystem.h
  * @author  Creelinks Application Team
  * @version V1.0.0
  * @date    2016-08-05
  * @brief   Creelinks平台所有有关系统的函数
  ******************************************************************************
  * @attention
  *
  *1)
  *
  * <h2><center>&copy; COPYRIGHT 2016 Darcern</center></h2>
  ******************************************************************************
  */
#ifndef __CE_SYSTEM_H__
#define __CE_SYSTEM_H__

#include "CeMcu.h"
#include "CeTimer.h"
#ifdef __cplusplus
 extern "C" {
#endif //__cplusplus

/**
  * @brief 用于计算时间花费的属性对象
  */
typedef struct
{
     uint32 nowTickUs;
}CeTimeCost;


typedef struct
{
    CeTimer ceTimer;
    uint32 tickLoopNow;
}CeSystem;
/**
  * @brief  系统操作相关的函数集合结构体对象
  */
typedef struct
{
    CE_STATUS   (*initial)(void);       /*!< @brief CreeLinks环境初始化，需要在main最开始时调用*/

    void        (*delayNs)(uint32 ns);  /*!< @brief 微秒延时函数，1s = 1 000ms = 1 000 000us = 1 000 000 000 ns
                                             @param ns:设定的延时时间，范围0~60129542144ns，精度正负14ns*/

    void        (*delayUs)(uint32 us);  /*!< @brief 微秒延时函数，1s = 1 000ms = 1 000 000us = 1 000 000 000 ns
                                             @param us:设定的微秒延时时间，范围0~59652323us，精度正负14us*/

    void        (*delayMs)(uint32 ms);  /*!< @brief 毫秒秒延时函数，1s = 1 000ms = 1 000 000us = 1 000 000 000 ns
                                             @param ms:设定的毫秒延时时间，范围0~4294967296ms，精度正负1us*/

    uint64      (*getSystemTickUs)(void);/*!<@brief 获取系统从开机到现在所经过的时间,精度1Us
                                             @return 获取系统从开机到现在的运行时间，单位Us*/

    uint64      (*getSystemTickMs)(void);/*!<@brief 获取系统从开机到现在所经过的时间,精度1Ms
                                             @return 获取系统从开机到现在的运行时间，单位Ms*/

    const char* (*getErrorMsg)(CE_STATUS ceStatus);/*!<
                                             @brief 根据返回的错误码，得到char*类型的错误文字信息
                                             @param ceStatus:状态码
                                             @return 以字符串方式返回状态码*/
}CeSystemOp;
extern const CeSystemOp ceSystemOp; /*!< 所有与系统平台相关的操作，如打印、延时、初始化等*/

#ifdef __cplusplus
 }
#endif //__cplusplus
#endif //__CE_SYSTEM_H__
