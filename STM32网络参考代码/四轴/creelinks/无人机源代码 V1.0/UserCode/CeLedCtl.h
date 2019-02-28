/**
  ******************************************************************************
  * @file    CeLedCtl.h
  * @author  Creelinks Application Team
  * @version V1.0.0
  * @date    2017-01-06
  * @brief   适用于CeLedCtl模块的驱动头文件
  ******************************************************************************
  * @attention
  *
  *1)无
  *
  * <h2><center>&copy; COPYRIGHT 2016 Darcern</center></h2>
  ******************************************************************************
  */
#ifndef __CE_LED_CTL_H__
#define __CE_LED_CTL_H__
#ifdef __cplusplus
 extern "C" {
#endif //__cplusplus
#include "Creelinks.h"
#include "CeLed1C.h"
     
/*
 *枚举，定义四个LED的闪烁方式
 */
typedef enum 
{
    CE_LED_CTL_MODE_OFF = 0x00,             /*!< 所有LED均不亮状态*/
    CE_LED_CTL_MODE_IN_CFG,                 /*!< 处于初始化及参数配置状态*/
    CE_LED_CTL_MODE_IN_NORMAL,              /*!< 正常工作状态*/
    CE_LED_CTL_MODE_IN_ERROR,               /*!< 故障状态*/
    CE_LED_CTL_MODE_FLASH_CYCLE_P,          /*!< 偏航正*/
    CE_LED_CTL_MODE_FLASH_CYCLE_N,          /*!< 偏航负*/
    CE_LED_CTL_MODE_GOTO_FRONT,             /*!< 俯*/
    CE_LED_CTL_MODE_GOTO_BACK,              /*!< 仰*/
    CE_LED_CTL_MODE_GOTO_LEFT,              /*!< 左翻滚*/
    CE_LED_CTL_MODE_GOTO_RIGHT,             /*!< 右翻滚*/
}CE_LED_CTL_MODE;


/*
 *CeLedCtl属性对像
 */
typedef struct
{
    CeLed1C ceLed0;                    
    CeLed1C ceLed1;                  
    CeLed1C ceLed2;                 
    CeLed1C ceLed3;   
    CE_LED_CTL_MODE ctlMode;
    int16 tick;
    CeTicker ceTicker;
}CeLedCtl;
/*
 *CeLedCtl操作对像
 */
typedef struct
{
    CE_STATUS   (*initial)(CE_RESOURCE ceGpioM0,CE_RESOURCE ceGpioM1,CE_RESOURCE ceGpioM2,CE_RESOURCE ceGpioM3);    /*!< @brief CeLedCtl模块初始化
                                                                                                                         @param ceGpioM0-3:四个LED使用的Gpio资源号*/

    void        (*setMode)(CE_LED_CTL_MODE ctlMode);                                                                /*!< @brief 配置四个LED闪烁的方式
                                                                                                                         @param ctlMode:四个LED闪烁的方式*/

    CE_LED_CTL_MODE (*getMode)(void);                                                                               /*!< @brief 获取当前四个LED闪烁的方式
                                                                                                                         @return 当前四个LED闪烁的方式*/
}CeLedCtlOp;
/*
 *CeLedCtl操作对象实例
 */
extern const CeLedCtlOp ceLedCtlOp;

#ifdef __cplusplus
 }
#endif //__cplusplus
#endif //__CE_LED_CTL_H__
