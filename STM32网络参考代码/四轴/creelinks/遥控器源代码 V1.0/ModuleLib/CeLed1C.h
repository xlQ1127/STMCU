/**
  ******************************************************************************
  * @file    CeLed1C.h
  * @author  Creelinks Application Team
  * @version V1.0.0
  * @date    2017-01-06
  * @brief   适用于CeLed1C模块的驱动头文件
  ******************************************************************************
  * @attention
  *
  *1)
  *
  * <h2><center>&copy; COPYRIGHT 2016 Darcern</center></h2>
  ******************************************************************************
  */
#ifndef __CE_LED_1C_H__
#define __CE_LED_1C_H__
#ifdef __cplusplus
 extern "C" {
#endif //__cplusplus
#include "Creelinks.h"
#define __CE_LED_1C_VERSION__ 1                                             /*!< 此驱动文件的版本号*/
#define __CE_LED_1C_NEED_CREELINKS_VERSION__ 1                              /*!< 需要Creelinks平台库的最低版本*/
#if (__CE_CREELINKS_VERSION__ < __CE_LED_1C_NEED_CREELINKS_VERSION__)       /*!< 检查Creelinks平台库的版本是否满足要求*/
#error "驱动文件CeLed1C.h需要高于1.0以上版本的Creelink库，请登陆www.creelinks.com下载最新版本的Creelinks库。"
#else
/*
 *CeLed1C属性对象
 */
typedef struct
{
    CeGpio      ceGpio;                                             /*!< 模块使用到的Gpio资源对象*/
    CeTicker    ceTicker;                                           /*!< CeLed1C闪烁用到的滴答毫秒定时器任务对象*/
    CePwm       cePwm;                                              /*!< 模块使用到的Pwm资源对象*/
    int32       breathAdd;                                          /*!< 模块工作在呼吸灯模式下的中间变量*/
    uint8       isBreathUp;                                         /*!< 模块工作在呼吸灯模式下的中间变量*/
    uint8       ledMode;                                            /*!< 标志模块工作在闪烁模式下，还是呼吸模式下*/
    uint8       isGpio;                                             /*!< 标志使用的是Pwm资源，还是Gpio资源*/
    uint16      flashUpMs;                                          /*!< 模块闪烁的发光持续时间*/
    uint16      falshDownMs;                                        /*!< 模块闪烁的熄灭持续时间*/
}CeLed1C;

/*
 *CeLed1C操作对象
 */
typedef struct
{
    void    (*initialByGpio)(CeLed1C* ceLed1C,CE_RESOURCE ceGpio);  /*!< @brief CeLed1C模块初始化
                                                                         @param ceLed1C:CeLed1C属性对象指针
                                                                         @param ceGpio:CeLed1C模块使用的资源号*/

    void    (*initialByPwm)(CeLed1C* ceLed1C, CE_RESOURCE cePwm);   /*!< @brief CeLed1C模块初始化
                                                                         @param ceLed1C:CeLed1C属性对象指针
                                                                         @param ceGpio:CeLed1C模块使用的资源号*/

    void    (*setOn)(CeLed1C* ceLed1C);                             /*!< @brief 设置Led状态为开
                                                                         @param ceLed1C:CeLed1C属性对象指针*/

    void    (*setOff)(CeLed1C* ceLed1C);                            /*!< @brief 设置Led状态为关
                                                                         @param ceLed1C:CeLed1C属性对象指针
                                                                         @param ceGpio:CeLed1C模块使用的资源号*/

    void    (*setFlash)(CeLed1C* ceLed1C, uint16 flashUpMs,uint16 flashDownMs);/*!<
                                                                         @brief 设置Led状态为闪烁
                                                                         @param ceLed1C:CeLed1C属性对象指针
                                                                         @param flashUpMs:发光持续时间
                                                                         @param flashDownMs:熄灭持续时间*/

    void    (*setBreath)(CeLed1C* ceLed1C, uint16 flashMs);         /*!< @brief 设置Led为呼吸状态,只有使用Pwm资源接口进行初始化才能够实现此功能
                                                                         @param ceLed1C:CeLed1C属性对象指针
                                                                         @param flashMs:呼吸周期，即完成一次亮与灭的时间间隔*/
}CeLed1COpBase;
/*
 *CeLed1C操作对象实例
 */
extern const CeLed1COpBase ceLed1COp;

#endif //(__CE_CREELINKS_VERSION__ < __CE_LED_1C_NEED_CREELINKS_VERSION__)
#ifdef __cplusplus
 }
#endif //__cplusplus
#endif //__CE_LED_1C_H__

/**
******************************************************************************
* @brief  使用流程及示例程序(基于前后台非操作系统环境) 
* @function 使led以1HZ的频度闪烁
******************************************************************************
#include "Creelinks.h"
#include "CeLed1C.h"
CeLed1C myLed;                                  //定义CeLed1C结构体
int main(void)
{
    ceSystemOp.initial();                       //Creelinks环境初始化
    ceDebugOp.initial(R9Uart);                  //通过Uart串口输出Debug信息到上位机
    //TODO:请在此处插入模块初始化等操作
    ceLed1COp.initialByGpio(&myLed, R1AGP);     //使用R1AGP资源号来初始化myLed
    while (1)
    {
        ceTaskOp.mainTask();                    //Creelinks环境主循环任务，请保证此函数能够被周期调用
        //TODO:请在此处插入用户操
        ceLed1COp.setOn(&myLed);                //设置Led状态为发光
        ceSystemOp.delayMs(500);                //延时500ms
        ceLed1COp.setOff(&myLed);               //设置Led状态为熄灭
        ceSystemOp.delayMs(500);                //延时500ms
    };
}
******************************************************************************
*/
