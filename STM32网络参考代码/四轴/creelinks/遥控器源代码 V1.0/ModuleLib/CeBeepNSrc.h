/**
  ******************************************************************************
  * @file    CeBeepNSrc.h
  * @author  Creelinks Application Team
  * @version V1.0.0
  * @date    2017-01-06
  * @brief   适用于CeBeepNSrc模块的驱动头文件
  ******************************************************************************
  * @attention
  *
  *1)无
  *
  * <h2><center>&copy; COPYRIGHT 2016 Darcern</center></h2>
  ******************************************************************************
  */
#ifndef __CE_BEEP_NSRC_H__
#define __CE_BEEP_NSRC_H__
#ifdef __cplusplus
 extern "C" {
#endif //__cplusplus
#include "Creelinks.h"
#define __CE_BEEP_NSRC_VERSION__ 1                                             /*!< 此驱动文件的版本号*/
#define __CE_BEEP_NSRC_NEED_CREELINKS_VERSION__ 1                               /*!< 需要Creelinks平台库的最低版本*/
#if (__CE_CREELINKS_VERSION__ < __CE_BEEP_NSRC_NEED_CREELINKS_VERSION__)       /*!< 检查Creelinks平台库的版本是否满足要求*/
#error "驱动文件CeBeepNSrc.h需要高于1.0以上版本的Creelink库，请登陆www.creelinks.com下载最新版本的Creelinks库。"
#else
/*
 *CeBeepNSrc属性对像
 */
typedef struct
{
    CeGpio ceGpio;                                                          /*!< */
}CeBeepNSrc;
/*
 *CeBeepNSrc操作对像
 */
typedef struct
{
    CE_STATUS   (*initialByGpio)(CeBeepNSrc* ceBeepNSrc, CE_RESOURCE ceGpio);   /*!< @brief CeBeepNSrc模块初始化，使用Gpio资源
                                                                                     @param ceBeepNSrc:CeBeepNSrc属性对象指针
                                                                                     @param ceGpio:CeBeepNSrc模块使用的资源号*/

    CE_STATUS   (*initialByPwm)(CeBeepNSrc* ceBeepNSrc, CE_RESOURCE cePwm);     /*!< @brief CeBeepNSrc模块初始化，使用Pwm资源
                                                                                     @param ceBeepNSrc:CeBeepNSrc属性对象指针
                                                                                     @param cePwm:CeBeepNSrc模块使用的资源号*/

    void        (*say)(CeBeepNSrc* ceBeepNSrc,uint16 sayMs,uint16 sleepMs, uint8 beepTimes);
                                                                                /*!< @brief CeBeepNSrc发声，由于是无源，所以线程将会被堵塞，直到发声完成
                                                                                     @param ceBeepNSrc:CeBeepNSrc属性对象指针
                                                                                     @param durationMs:发声时间，单位毫秒
                                                                                     @param sleepMs:停止发声时间
                                                                                     @param beepTimes:发声次数*/

}CeBeepNSrcOp;
/*
 *CeBeepNSrc操作对象实例
 */
extern const CeBeepNSrcOp ceBeepNSrcOp;

#endif // (__CE_CREELINKS_VERSION__ < __CE_BEEP_NSRC_NEED_CREELINKS_VERSION__)
#ifdef __cplusplus
 }
#endif //__cplusplus
#endif //__CE_BEEP_NSRC_H__

/**
******************************************************************************
* @brief  使用流程及示例程序(基于前后台非操作系统环境) 
* @function xxxxxzzzz
******************************************************************************
#include "Creelinks.h"
int main(void)
{
    ceSystemOp.initial();                       //Creelinks环境初始化
    ceDebugOp.initial(Uartx);                        //通过Uart串口输出Debug信息到上位机
    //TODO:请在此处插入模块初始化等操作

    while (1)
    {
        ceTaskOp.mainTask();                    //Creelinks环境主循环任务，请保证此函数能够被周期调用
        //TODO:请在此处插入用户操作

    };
}
******************************************************************************
*/
