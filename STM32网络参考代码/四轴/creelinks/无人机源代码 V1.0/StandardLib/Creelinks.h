/**
  ******************************************************************************
  * @file   Creelinks.h
  * @author Creelinks Application Team
  * @version V1.0.0
  * @date   2017-03-26
  * @brief  Creelinks平台主库入口，即各个子模块都要包含此头文件。
  ******************************************************************************
  * @attention
  *
  *1)有关与处理器平台相关的定义、及资源号的指定等内容，可查看CeMcu.h文件
  *
  * <h2><center>&copy; COPYRIGHT 2016 Darcern</center></h2>
  ******************************************************************************
  */
#ifndef __CE_CREELINKS_H__
#define __CE_CREELINKS_H__

/*Creelinks平台硬件属性****************************************/
#define __CE_CREELINKS_VERSION__    1           /*!< 当前Creelinks.h文件的版本号*/

#include "CeMcu.h"                              /*!< 与处理器平台相关的内容及宏开关、资源号定义等*/
#include "CeSystem.h"                           /*!< 与系统相关内容，如调试打印、获得时间戳、RTOS进入和退出临界段等*/
#include "CeGpio.h"                             /*!< Gpio输入输出口资源相关*/
#include "CeAd.h"                               /*!< Ad模数转换资源相关*/
#include "CeUart.h"                             /*!< Uart串口资源相关*/
#include "CeSpi.h"                              /*!< Spi串行总线资源相关*/
#include "CeDebug.h"                            /*!< L36通用8080总线LCD显示内容相关*/
#include "CeInt.h"                              /*!< Int外部中断资源相关*/
#include "CePwm.h"                              /*!< Pwm脉宽调制资源相关*/
#include "CeDa.h"                               /*!< Da数模转换资源相关*/
#include "CeTimer.h"                            /*!< Timer内部硬件定时器资源相关*/
#include "CeCcp.h"                              /*!< Ccp计数器资源相关*/
#include "CeTicker.h"                           /*!< Ticker 1ms软件周期定时器相关*/
#include "CeTask.h"                             /*!< Task任务相关*/
#include "CeI2c.h"                              /*!< I2c总线资源相关*/
#include "CeFlash.h"                            /*!< 处理器片内FLASH资源相关*/


#endif //__CE_CREELINKS_H__
