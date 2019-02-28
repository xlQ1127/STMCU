/**
  ******************************************************************************
  * @file    CeGpio.h
  * @author  Creelinks Application Team
  * @version V1.0.0
  * @date    2016-08-05
  * @brief   Creelinks平台CeGpio库头文件
  ******************************************************************************
  * @attention
  *
  *1)因受处理器平台的约束，不同的处理器有不同的IO口配置模式，为保证兼容性，Creelinks会根据用户设置的IO口模式自动匹
  *  配对应处理器的IO口模式
  *
  * <h2><center>&copy; COPYRIGHT 2016 Darcern</center></h2>
  ******************************************************************************
  */
#ifndef __CE_GPIO_H__
#define __CE_GPIO_H__

#include "CeMcu.h"

#ifdef __cplusplus
 extern "C" {
#endif //__cplusplus

/**
  * @brief  结构体，GPIO对象可用属性集合
  */
typedef struct
{
    CE_RESOURCE     ceResource;                                 /*!< GPIO对应的资源号*/
    CE_GPIO_MODE    ceGpioMode;                                 /*!< 所设定的GPIO引脚模式*/

    CeExGpioPar     ceExGpioPar;                                /*!< 与处理器平台相关的额外参数结构体，用以提高代码效率，用户列须关注*/
}CeGpio;

/**
  * @brief  结构体，GPIO对象可用操作集合
  */
typedef struct
{
    CE_STATUS   (*initial)(CeGpio* ceGpio);                     /*!< @brief 初始化一个GPIO
                                                                     @param ceGpio:GPIO属性对象集合指针*/

    void        (*setBit)(CeGpio* ceGpio);                      /*!< @brief 设置GPIO口的值为1
                                                                     @param ceGpio:GPIO属性对象集合指针*/

    void        (*resetBit)(CeGpio* ceGpio);                    /*!< @brief 设置GPIO口的值为0
                                                                     @param ceGpio:GPIO属性对象集合指针*/

    uint8       (*getBit)(CeGpio* ceGpio);                      /*!< @brief 获取GPIO口的值，0x01和0x00
                                                                     @param ceGpio:GPIO属性对象集合指针
                                                                     @return 当前Gpio口的电平状态*/

    void        (*setMode)(CeGpio* ceGpio,CE_GPIO_MODE ceGpioMode);/*!<
                                                                     @brief 配置Gpio口的工作方式
                                                                     @param ceGpio:GPIO属性对象集合指针
                                                                     @param ceGpioMode:GPIO工作模式*/
}CeGpioOp;
extern const CeGpioOp ceGpioOp;                                 /*!< 所有与GPIO相关的操作*/

#ifdef __cplusplus
 }
#endif //__cplusplus
#endif //__CE_GPIO_H__

/**
******************************************************************************
* @brief  使用流程及示例程序(基于前后台非操作系统环境)
* @function 设定Gpio每500ms进行一次电平翻转
******************************************************************************
#include "Creelinks.h"
CeGpio myGpio;                                          //定义Gpio属性对象
int main(void)
{
    ceSystemOp.initial();                               //Creelinks环境初始化
    ceDebugOp.initial(Uartx);                                //通过Uart串口输出Debug信息到上位机
    //TODO:请在此处插入模块初始化等操作
    myGpio.ceResource = RxG;                            //指定Gpio使用的资源号
    myGpio.ceGpioMode = CE_GPIO_MODE_OUT_OD;            //配置Gpio的工作模式
    ceGpioOp.initial(&myGpio);                          //初始化Gpio
    while (1)
    {
        ceTaskOp.mainTask();                            //Creelinks环境主循环任务，请保证此函数能够被周期调用
        //TODO:请在此处插入用户操作
        ceGpioOp.setBit(&myGpio);                   //设定Gpio为高电平
        ceDebugOp.printf("Gpio status: up.\n");        //向上位机输出调试信息
        ceSystemOp.delayMs(500);                        //延时500ms
        ceGpioOp.resetBit(&myGpio);                 //设定Gpio为高电平
        ceDebugOp.printf("Gpio status: down.\n");      //向上位机输出调试信息
        ceSystemOp.delayMs(500);                        //延时500ms
    };
}
******************************************************************************
*/
