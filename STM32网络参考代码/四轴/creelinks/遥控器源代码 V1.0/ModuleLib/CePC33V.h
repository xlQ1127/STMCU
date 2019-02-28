/**
  ******************************************************************************
  * @file     CePC33V.h
  * @author   Creelinks Application Team
  * @version  V1.0.0
  * @date    2017-01-06
  * @brief    适用于CePC33V模块的驱动头文件
  ******************************************************************************
  * @attention
  *
  *1)
  *
  * <h2><center>&copy; COPYRIGHT 2016 Darcern</center></h2>
  ******************************************************************************
  */
#ifndef __CE_PC_33V_H__
#define __CE_PC_33V_H__
#ifdef __cplusplus
 extern "C" {
#endif //__cplusplus
#include "Creelinks.h"
#define __CE_PC_33V_VERSION__ 1                                             /*!< 此驱动文件的版本号*/
#define __CE_PC_33V_NEED_CREELINKS_VERSION__ 1                              /*!< 需要Creelinks平台库的最低版本*/
#if (__CE_PC_33V_VERSION__ > __CE_PC_33V_NEED_CREELINKS_VERSION__)       /*!< 检查Creelinks平台库的版本是否满足要求*/
#error "驱动文件CePC33V.h需要高于1.0以上版本的Creelink库，请登陆www.creelinks.com下载最新版本的Creelinks库。"
#else
/*
 *CePC33V属性对像
 */
typedef struct
{
    CeAd ceAd;                                                              /*!< Ad资源接口属性对象*/
}CePC33V;
/*
 *CePC33V操作对像
 */
typedef struct
{
    CE_STATUS   (*initial)(CePC33V* cePC33V, CE_RESOURCE ceAd);             /*!< @brief  CePC33V模块初始化
                                                                                 @param  cePC33V:CePC33V属性对象指针
                                                                                 @param  ceXX:CePC33V模块使用的资源号*/

    fp32        (*getVoltage)(CePC33V* cePC33V);                            /*!< @brief  获得电压值
                                                                                 @param  cePC33V:CePC33V属性对象指针
                                                                                 @return 采集到的电压值*/
}CePC33VOpBase;

/*
 *CePC33V操作对象实例
 */
extern const CePC33VOpBase cePC33VOp;

#endif // (__CE_CREELINKS_VERSION__ < __CE_PC_33V_NEED_CREELINKS_VERSION__)
#ifdef __cplusplus
}
#endif //__cplusplus
#endif //__CE_PC_33V_H__

/**
******************************************************************************
* @brief  使用流程及示例程序(基于前后台非操作系统环境) 
* @function 采集电压，并通过串口将信息显示到上位机
******************************************************************************
#include "Creelinks.h"
#include "CePC33V.h"
CePC33V myPc;
int main(void)
{
    ceSystemOp.initial();                       //Creelinks环境初始化
    ceDebugOp.initial(R9Uart);                  //通过Uart串口输出Debug信息到上位机
    //TODO:请在此处插入模块初始化等操作
    cePC33VOp.initial(&myPc,R1AGP);             //使用R1AGP的Ad功能功能初始化模块
    while (1)
    {
        ceTaskOp.mainTask();                    //Creelinks环境主循环任务，请保证此函数能够被周期调用
        //TODO:请在此处插入用户操
        ceDebugOp.printf("The voltage is:%fV\n",cePC33VOp.getVoltage(&myPc));
        ceSystemOp.delayMs(500);
    };
}
******************************************************************************
*/
