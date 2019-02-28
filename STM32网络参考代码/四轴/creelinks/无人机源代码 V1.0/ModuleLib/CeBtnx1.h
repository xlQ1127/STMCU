/**
  ******************************************************************************
  * @file    CeBtnx1.h
  * @author  Creelinks Application Team
  * @version V1.0.0
  * @date    2017-01-06
  * @brief   适用于CeBtnx1模块的驱动头文件
  ******************************************************************************
  * @attention
  *
  *1)
  *
  * <h2><center>&copy; COPYRIGHT 2016 Darcern</center></h2>
  ******************************************************************************
  */
#ifndef __CE_BTN_X1_H__
#define __CE_BTN_X1_H__
#ifdef __cplusplus
 extern "C" {
#endif //__cplusplus
#include "Creelinks.h"
#define __CE_BTN_X1_VERSION__ 1                                             /*!< 此驱动文件的版本号*/
#define __CE_BTN_X1_NEED_CREELINKS_VERSION__ 1                              /*!< 需要Creelinks平台库的最低版本*/
#if (__CE_CREELINKS_VERSION__ < __CE_BTN_X1_NEED_CREELINKS_VERSION__)       /*!< 检查Creelinks平台库的版本是否满足要求*/
#error "驱动文件CeBtnx1.h需要高于1.0以上版本的Creelink库，请登陆www.creelinks.com下载最新版本的Creelinks库。"
#else
/*
 *CeBtnx1属性对像
 */
typedef struct
{
    CeInt       ceInt;                              /*!< 模块使用外部中断Int完成初始化用到的外部中断Int资源属性对象*/
    CeGpio      ceGpio;                             /*!< 模块使用Gpio完成初始化用到的Gpio资源属性对象*/
    CeTicker    ceTicker;                           /*!< 模块使用的滴答毫秒定时器属性对象*/
    uint8       btnStatus;                          /*!< 按钮当前状态*/
    void        (*callBackPressEvent)(void);        /*!< 按键事件回调函数*/
}CeBtnx1;

/*
 *CeBtnx1操作对像
 */
typedef struct
{
    CE_STATUS   (*initialByGpio)(CeBtnx1* ceBtnx1, CE_RESOURCE ceGpio, void (*callBackPressEvent)(void));/*!<
                                                         @brief CeBtnx1模块使用Gpio口来完成初始化
                                                         @param CeBtnx1:CeBtnx1属性对象指针
                                                         @param ceGpio:CeBtnx1模块使用的资源号
                                                         @param callBackPressEvent:按键按下时的回调函数，不需要回调传CE_NULL即可*/

    CE_STATUS   (*initialByInt)(CeBtnx1* ceBtnx1, CE_RESOURCE ceInt, void (*callBackPressEvent)(void));/*!<
                                                         @brief CeBtnx1模块使用外部中断Int来完成初始化
                                                         @param CeBtnx1:CeBtnx1属性对象指针
                                                         @param ceInt:CeBtnx1模块使用的资源号
                                                         @param callBackPressEvent:按键按下时的回调函数，不需要回调传CE_NULL即可*/

    uint8       (*getStatus)(CeBtnx1* ceBtnx1);     /*!< @brief 获取CeBtnx1状态，返回1表明已按下，返回0表明未按下
                                                         @param ceBtnx1:CeBtnx1属性对象指针*/

    CE_STATUS   (*waitForPressDown)(CeBtnx1* ceBtnx1, uint32 outTimeMs);/*!<
                                                         @brief 等待按键按下(如果是外部中断Int方式初始化，请谨慎使用此方法)
                                                         @param ceBtnx1:CeBtnx1属性对象指针
                                                         @param outTimeMs:等待的超时时间，Ms*/

    CE_STATUS   (*waitForPressUp)(CeBtnx1* ceBtnx1, uint32 outTimeMs);/*!<
                                                         @brief 等待按键弹起
                                                         @param ceBtnx1:CeBtnx1属性对象指针
                                                         @param outTimeMs:等待的超时时间，Ms*/
}CeBtnx1OpBase;

/*
 *CeBtnx1操作对象实例
 */
extern const CeBtnx1OpBase ceBtnx1Op;

#endif //(__CE_CREELINKS_VERSION__ < __CE_BTN_X1_NEED_CREELINKS_VERSION__)
#ifdef __cplusplus
 }
#endif //__cplusplus
#endif //__CE_BTN_X1_H__

/**
******************************************************************************
* @brief  使用流程及示例程序(基于前后台非操作系统环境) 
* @function 按钮每按下一次，便在上位机上打印调试信息
******************************************************************************
#include "Creelinks.h"
#include "CeBtnx1.h"
CeBtnx1 myBtn;                                  //定义CeBtnx1属性对象
void callBackPress(void)
{
    ceDebugOp.printf("myBtn is Press down.\n");//按下按键执行事件回调，打印调试信息
}
int main(void)
{
    ceSystemOp.initial();                       //Creelinks环境初始化
    ceDebugOp.initial(R9Uart);                  //通过Uart串口输出Debug信息到上位机
    //TODO:请在此处插入模块初始化等操作
    ceBtnx1Op.initialByGpio(&myBtn, R1AGP, callBackPress);  //使用资源号R1AGP初始化myBtn
    while (1)
    {
        ceTaskOp.mainTask();                    //Creelinks环境主循环任务，请保证此函数能够被周期调用
        //TODO:请在此处插入用户操

    };
}
******************************************************************************
*/
