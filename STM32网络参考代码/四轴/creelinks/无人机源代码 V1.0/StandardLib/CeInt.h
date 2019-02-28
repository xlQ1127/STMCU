/**
  ******************************************************************************
  * @file    CeInt.h
  * @author  Creelinks Application Team
  * @version V1.0.0
  * @date    2016-08-05
  * @brief   Creelinks平台CeInt库头文件，主要用于外部中断的响应与处理
  ******************************************************************************
  * @attention
  *
  *1)不同的Int资源，有不同的中断优先级，用户可以CeInt.c中的宏定义中配置
  *2)每个Int的回调均在中断内进行，因此请勿在回调中执行耗时操作
  *
  * <h2><center>&copy; COPYRIGHT 2016 Darcern</center></h2>
  ******************************************************************************
  */
#ifndef __CE_INT_H__
#define __CE_INT_H__

#include "CeMcu.h"

#ifdef __cplusplus
 extern "C" {
#endif //__cplusplus

/**
  * @brief  枚举，外部中断Int的触发方式
  */
typedef enum
{
    CE_INT_MODE_TRIGGER_FALLING = 0x00,             /*!< 中断触发方式，下降沿触发*/
    CE_INT_MODE_TRIGGER_RISING,                     /*!< 中断触发方式 ，上升沿触发*/
}CE_INT_MODE;

/**
  * @brief  结构体，外部中断Int对象可用属性集合
  */
typedef struct
{
    CE_RESOURCE     ceResource;                     /*!< 外部中断Int对应的资源号*/
    CE_INT_MODE     ceIntMode;                      /*!< 外部中断Int的工作模式*/
    void*           pAddPar;                        /*!< 空指针，传递给外部中断Int触发时的回调*/
    void            (*callBack)(void* pAddPar);     /*!< 中断事件发生时，需要执行的函数
                                                         callBack:外部中断发生时，需要执行的回调；pAddPar:空指针，可用于传送额外参数*/

    CeExIntPar      ceExIntPar;                     /*!< 与处理器平台相关的额外参数结构体，用以提高代码效率，用户无须关注*/
}CeInt;

/**
  * @brief  结构体，外部中断Int对象可用操作集合
  */
typedef struct
{
    CE_STATUS   (*initial)(CeInt* ceInt);           /*!< @brief 初始化外部中断Int
                                                         @param ceInt:外部中断Int属性对象指针*/

    void        (*setMode)(CeInt* ceInt, CE_INT_MODE ceIntMode);/*!<
                                                         @brief 配置中断方式，是上升还是下降
                                                         @param ceInt:外部中断Int属性对象指针
                                                         @param ceIntMode:需要配置的中断方式*/

    void        (*start)(CeInt* ceInt);             /*!< @brief 开始外部中断Int监测
                                                         @param ceInt:外部中断Int属性对象指针*/

    void        (*stop)(CeInt* ceInt);              /*!< @brief 停止外部中断Int监测
                                                         @param ceInt:外部中断Int属性对象指针*/

    uint8       (*getBit)(CeInt* ceInt);            /*!< @brief 获取外部中断Int口对应的Gpio的电平值，0x01和0x00
                                                         @param ceInt:外部中断Int属性对象指针
                                                         @reutrn 中断对应引脚的电平状态*/
}CeIntOp;
extern const CeIntOp ceIntOp;                       /*!< 所有与外部中Int断相关的操作*/

#ifdef __cplusplus
 }
#endif //__cplusplus
#endif //__CE_INT_H__

/**
******************************************************************************
* @brief  使用流程及示例程序(基于前后台非操作系统环境)
* @function 外部中断测试函数，下降沿触发
******************************************************************************
#include "Creelinks.h"
CeInt myInt;

void intEventCallBack(void* pAddPar)
{
    ceDebugOp.printf("Enter the interrupt, Gpio status:%d\n", ceIntOp.getBit((CeInt*)(pAddPar)));//进入中断事件后，打印调试信息
}
int main(void)
{
    ceSystemOp.initial();                           //Creelinks环境初始化
    ceDebugOp.initial(Uartx);                            //通过Uart串口输出Debug信息到上位机
    //TODO:请在此处插入模块初始化等操作
    myInt.ceResource = RxI;                         //指定资源中断功能模块使用的资源号
    myInt.ceMode = CE_INT_MODE_TRIGGER_FALLING;          //指定外部中断是下降沿中断
    myInt.pAddPar = &myInt;                         //设置空指针指向自己，在中断回调里使用
    myInt.callBack = intEventCallBack;              //指定事件回调函数
    ceIntOp.initial(&myInt);                        //初始化外部中断
    ceIntOp.start(&myInt);                          //使能外部中断
    while (1)
    {
        ceTaskOp.mainTask();                        //Creelinks环境主循环任务，请保证此函数能够被周期调用
        //TODO:请在此处插入用户操作
    };
}
******************************************************************************
*/
