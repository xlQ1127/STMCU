/**
  ******************************************************************************
  * @file    CeTicker.h
  * @author  Creelinks Application Team
  * @version V1.0.0
  * @date    2016-08-05
  * @brief   此驱动头文件，包含所有精确心跳回调相关的操作
  ******************************************************************************
  * @attention
  *
  *1)定时器最小间隔为1ms（有些处理器平台可能为其它值）,精度根据不同的处理器平台而不同，基本为正负10us以内。
  *2)针对无操作系统的平台，注册的定时器函数运行在系统中断内；针对有操作系统的平台，则运行在一个高优先级的线程内，故尽量勿在回调函数内进行内进行耗时操作。
  *
  * <h2><center>&copy; COPYRIGHT 2016 Darcern</center></h2>
  ******************************************************************************
  */
#ifndef __CE_TICKER_H__
#define __CE_TICKER_H__

#include "CeMcu.h"

#ifdef __cplusplus
 extern "C" {
#endif //__cplusplus

/**
  * @brief  结构体，Ticker对象可用属性集合
  */
typedef struct CeTickerBase
{
    uint32      ID;                                         /*!< 定时器任务ID*/
    uint32      intervalMs;                                 /*!< 定时器定时间隔*/
    void*       pAddPar;                                    /*!< 空指针*/
    void        (*callBack)(void* pAddPar);                 /*!< 到达定时时间后，需要执行的函数*/
    struct      CeTickerBase* nextCeTicker;                 /*!< 链表，保存下一个定时任务结构体*/

    CeExTickerPar  ceExTickerPar;                           /*!< 与处理器平台相关有额外参数*/
}CeTicker;

/**
  * @brief  结构体，Ticker对象可用操作集合
  */
typedef struct
{
    CE_STATUS   (*registerTicker)(CeTicker* ceTicker);      /*!< @brief 注册一个定时任务
                                                                 @param ceTicker:定时器指针*/

    CE_STATUS   (*start)(CeTicker* ceTicker);               /*!< @brief 开始一个定时任务
                                                                 @param ceTicker:定时器指针*/

    CE_STATUS   (*stop)(CeTicker* ceTicker);                /*!< @brief 停止一个定时任务
                                                                 @param ceTicker:定时器指针*/

    CE_STATUS   (*unRegister)(CeTicker* ceTicker);          /*!< @brief 取消注册定时任务
                                                                 @param ceTicker:定时器指针*/

    CE_STATUS   (*callBySystem)(void);                      /*!< @brief 由系统在定时器中断内调用的函数*/
}CeTickerOp;
extern const CeTickerOp ceTickerOp;                     /*!< 所有与Ticker相关的操作*/

#ifdef __cplusplus
 }
#endif //__cplusplus
#endif //__CE_TICKER_H__

/**
******************************************************************************
* @brief  使用流程及示例程序(基于前后台非操作系统环境)
* @function 注册一个滴答定时器，让其每500ms调用一次回调函数
******************************************************************************
#include "Creelinks.h"
CeTicker myTicker;                              //定义Ticker属性对象
void tickCallBack(void* pAddPar)
{
    ceDebugOp.printf("Tick is running, ID=%d\n", ((CeTicker*)(pAddPar))->ID);
}
int main(void)
{
    ceSystemOp.initial();                       //Creelinks环境初始化
    ceDebugOp.initial(Uartx);                        //通过Uart串口输出Debug信息到上位机
    //TODO:请在此处插入模块初始化等操作
    myTicker.ID = 0x01;                         //指定Ticker的ID号
    myTicker.intervalMs = 500;                  //调用的周期间隔
    myTicker.pAddPar = &myTicker;               //设置属性对象中空指针指向自己，即为回调函数传入的参数
    myTicker.callBack = tickCallBack;           //指定回调函数
    ceTickerOp.registerTicker(&myTicker);       //注册一个Ticker滴答定时器任务
    ceTickerOp.start(&myTicker);                //开始此任务
    while (1)
    {
        ceTaskOp.mainTask();                    //Creelinks环境主循环
        //TODO:请在此处插入用户操作
    };
}
******************************************************************************
*/
