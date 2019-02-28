/**
  ******************************************************************************
  * @file    CeTimer.h
  * @author  Creelinks Application Team
  * @version V1.0.0
  * @date    2016-08-05
  * @brief   此驱动头文件，包含所有精确定时器相关的操作
  ******************************************************************************
  * @attention
  *
  *1)
  *
  * <h2><center>&copy; COPYRIGHT 2016 Darcern</center></h2>
  ******************************************************************************
  */
#ifndef __CE_TIMER_H__
#define __CE_TIMER_H__

#include "CeMcu.h"
#ifdef __cplusplus
 extern "C" {
#endif //__cplusplus

/**
  * @brief  结构体，Timer对象可用属性集合
  */
typedef struct
{
    CE_RESOURCE ceResource;
    uint32      intervalNs;                                 /*!< 定时器定时间隔*/
    void*       pAddPar;                                    /*!< 空指针*/
    void        (*callBack)(void* pAddPar);                 /*!< 到达定时时间后，需要执行的函数*/

    CeExTimerPar   ceExTimerPar;                            /*!< 与处理器平台相关的额外参数结构体，用以提高代码效率，用户无须关注*/
}CeTimer;

/**
  * @brief  结构体，Timer对象可用操作集合
  */
typedef struct
{
    CE_STATUS   (*initial)(CeTimer* ceTimer);              /*!< @brief 由系统调用的定时器初始化程序
                                                                 @param ceTimer:定时器指针*/

    void        (*start)(CeTimer* ceTimer);                 /*!< @brief 开始一个定时任务
                                                                 @param ceTimer:定时器指针*/

    void        (*upData)(CeTimer* ceTimer);                /*!< @brief 更新一个定时任务的周期
                                                                 @param ceTimer:定时器指针*/

    void        (*stop)(CeTimer* ceTimer);                  /*!< @brief 停止一个定时任务
                                                                 @param ceTimer:定时器指针*/

    uint32      (*getTimerMaxCnt)(CeTimer* ceTimer);        /*!< @brief 获取定时器计数器的最大值
                                                                 @param ceTimer:定时器指针*/

    uint32      (*getTimerNowCnt)(CeTimer* ceTimer,uint8 isStopInt); /*!< 
                                                                 @brief 获取定时器计数器的值
                                                                 @param ceTimer:定时器指针
                                                                 @param isStopInt:0x01:获取是需停止定时器中断；0x00:获取时无需停止定时器中断*/
}CeTimerOp;
extern const CeTimerOp ceTimerOp;                           /*!< 所有与Timer关的操作*/

#ifdef __cplusplus
 }
#endif //__cplusplus
#endif //__CE_TIMER_H__
