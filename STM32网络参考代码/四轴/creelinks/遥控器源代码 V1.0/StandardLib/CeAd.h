/**
  ******************************************************************************
  * @file   CeAd.h
  * @author Creelinks Application Team
  * @version V1.0.0
  * @date   2017-03-26
  * @brief  Creelinks平台Ad对象的操作头文件,包含有关处理器平台Ad内容的相关操作
  ****************************************************************************** 
  * @attention
  *
  *1)
  *
  * <h2><center>&copy; COPYRIGHT 2016 Darcern</center></h2>
  ******************************************************************************
  */
#ifndef __CE_AD_H__
#define __CE_AD_H__

#include "CeMcu.h"

#ifdef __cplusplus
 extern "C" {
#endif //__cplusplus
/**
  * @brief  结构体，AD对象可用属性集合
  */
typedef struct
{
    CE_RESOURCE     ceResource;                     /*!< Ad对应的资源号*/

    CeExAdPar       ceExPar;                        /*!< 与处理器平台相关的额外参数结构体，用以提高代码效率，用户无须关注*/
}CeAd;

/**
  * @brief  结构体，AD对象可用操作集合
  */
typedef struct
{
    CE_STATUS   (*initial)(CeAd* ceAd);             /*!< @brief 初始化Ad转换
                                                         @param ceAd:Ad属性对象指针*/

    uint32      (*getConvertValue)(CeAd* ceAd);     /*!< @brief 获得Ad转换结果
                                                         @param ceAd:Ad属性对象指针
                                                         @return AD转换结果*/
}CeAdOp;
extern const CeAdOp ceAdOp;                         /*!< 所有与Ad相关的操作*/

#endif //__CE_USE_AD__

#ifdef __cplusplus
 }
#endif //__cplusplus

/**
******************************************************************************
* @brief  使用流程及示例程序(基于前后台非操作系统环境)
* @function 读取Ad口转换值，并通过Uart口传输给串口调试助手
******************************************************************************
#include "Creelinks.h"
CeAd myAd;                                      //定义Ad属性对象
uint32 convertVal;                              //转换结果保存的临时变量
int main(void)
{
    ceSystemOp.initial();                       //Creelinks环境初始化
    ceDebugOp.initial(Uartx);                   //通过Uart串口输出Debug信息到上位机
    //TODO:请在此处插入模块初始化等操作
    myAd.ceResource = PAxA;                     //定义Ad资源号
    ceAdOp.initial(&myAd);
    while (1)
    {
        ceTaskOp.mainTask();                    //Creelinks环境主循环任务，请保证此函数能够被周期调用
        //TODO:请在此处插入用户操作       
        ceDebugOp.printf("ConvertVal = %d",ceAdOp.getConvertValue(&myAd)); //打印Ad转换结果
        ceSystemOp.delayMs(500);                //延时500ms
    };
}
******************************************************************************
*/


