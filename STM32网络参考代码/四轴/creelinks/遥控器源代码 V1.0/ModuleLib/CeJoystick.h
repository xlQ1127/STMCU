/**
  ******************************************************************************
  * @file    CeJoystick.h
  * @author  Creelinks Application Team
  * @version V1.0.0
  * @date    2017-01-06
  * @brief   适用于CeJoystick模块的驱动头文件
  ******************************************************************************
  * @attention
  *
  *1)无
  *
  * <h2><center>&copy; COPYRIGHT 2016 Darcern</center></h2>
  ******************************************************************************
  */
#ifndef __CE_JOY_STICK_H__
#define __CE_JOY_STICK_H__
#ifdef __cplusplus
 extern "C" {
#endif //__cplusplus
#include "Creelinks.h"
#define __CE_JOY_STICK_VERSION__ 1                                             /*!< 此驱动文件的版本号*/
#define __CE_JOY_STICK_NEED_CREELINKS_VERSION__ 1                              /*!< 需要Creelinks平台库的最低版本*/
#if (__CE_CREELINKS_VERSION__ < __CE_JOY_STICK_NEED_CREELINKS_VERSION__)       /*!< 检查Creelinks平台库的版本是否满足要求*/
#error "驱动文件CeJoystick.h需要高于1.0以上版本的Creelink库，请登陆www.creelinks.com下载最新版本的Creelinks库。"
#else

#define  CE_JOY_STICK_SLIDER_SIZE   3   /*!< 对采集到的数据进行滑动平均滤波的深度*/

typedef struct
{
     int16 x;                           /*!< 当前摇杆的x坐标,范围-1000 到 1000*/
     int16 y;                           /*!< 当前摇杆的y坐标,范围-1000 到 1000*/
}CeJoystickAxis;

/*
 *CeJoystick属性对像
 */
typedef struct
{
    CeAd ceAdX;                             /*!< x坐标检测使用到的Ad资源*/
    CeAd ceAdY;                             /*!< y坐标检测使用到的Ad资源*/
    CeGpio ceGpio;                          /*!< 用于检测按钮是否按下的Gpio资源*/
    int32 sliderX[CE_JOY_STICK_SLIDER_SIZE];/*!< X轴滑动平均滤波器*/
    int32 sliderY[CE_JOY_STICK_SLIDER_SIZE];/*!< Y轴滑动平均滤波器*/
    CeJoystickAxis ceJoystickAxis;          /*!< 缓存获取到的坐标信息*/
    CeJoystickAxis ceCalibrationZero;       /*!< 存放校准零点时的坐标信息*/
}CeJoystick;
/*
 *CeJoystick操作对像
 */
typedef struct
{
    CE_STATUS       (*initial)(CeJoystick* ceJoystick, CE_RESOURCE ceAd1, CE_RESOURCE ceAd2, CE_RESOURCE ceGpio);/*!<
                                                                         @brief CeJoystick模块初始化
                                                                         @param ceJoystick:CeJoystick属性对象指针
                                                                         @param ceAd1:CeJoystick模块使用的Ad1资源号
                                                                         @param ceAd1:CeJoystick模块使用的Ad2资源号
                                                                         @param ceAd1:CeJoystick模块使用的Gpio资源号*/

    void            (*calibrationZero)(CeJoystick* ceJoystick);     /*!< @brief 校准零点
                                                                         @param ceJoystick:CeJoystick属性对象指针
                                                                         @param ceXX:CeJoystick模块使用的资源号*/

    CeJoystickAxis* (*getAxis)(CeJoystick* ceJoystick);             /*!< @brief 获取坐标
                                                                         @param ceJoystick:CeJoystick属性对象指针
                                                                         @param ceXX:CeJoystick模块使用的资源号*/

    uint8           (*getBtnStatus)(CeJoystick* ceJoystick);        /*!< @brief 获取按钮的状态
                                                                         @param ceJoystick:CeJoystick属性对象指针*/


}CeJoystickOp;
/*
 *CeJoystick操作对象实例
 */
extern const CeJoystickOp ceJoystickOp;

#endif // (__CE_CREELINKS_VERSION__ < __CE_JOY_STICK_NEED_CREELINKS_VERSION__)
#ifdef __cplusplus
 }
#endif //__cplusplus
#endif //__CE_JOY_STICK_H__

/**
******************************************************************************
* @brief  使用流程及示例程序(基于前后台非操作系统环境) 
* @function 将遥杆当前的位置坐标通过Uart在上位机中显示
******************************************************************************
#include "Creelinks.h"
#include "CeJoystick.h"
CeJoystick myJoystick;                          //定义摇杆属性对象
CeJoystickAxis* joystickAxis;                   //定义摇杆坐标临时保存对象指针
int main(void)
{
    ceSystemOp.initial();                       //Creelinks环境初始化
    ceDebugOp.initial(R9Uart);                  //通过Uart串口输出Debug信息到上位机
    //TODO:请在此处插入模块初始化等操作
    ceJoystickOp.initial(&myJoystick, R1AGP, R5ACGPW, R3GI);
    ceJoystickOp.calibrationZero(&myJoystick);
    while (1)
    {
        ceTaskOp.mainTask();                    //Creelinks环境主循环任务，请保证此函数能够被周期调用
        //TODO:请在此处插入用户操作
        joystickAxis = ceJoystickOp.getAxis(&myJoystick);//获取当前摇杆所有坐标
        ceDebugOp.printf("Joystick Axis is: x=%d, y=%d\n",joystickAxis->x,joystickAxis->y);
        ceDebugOp.printf("Joystick btn status is: %d\n",ceJoystickOp.getBtnStatus(&myJoystick));
        ceSystemOp.delayMs(100);
    };
}
******************************************************************************
*/
