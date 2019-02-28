/**
  ******************************************************************************
  * @file    CeLedCtl.c
  * @author  Creelinks Application Team
  * @version V1.0.0
  * @date    2017-01-06
  * @brief   适用于CeLedCtl模块的驱动库文件
  ******************************************************************************
  * @attention
  *
  *1)无
  *
  * <h2><center>&copy; COPYRIGHT 2016 Darcern</center></h2>
  ******************************************************************************
  */
#include "CeLedCtl.h"
#ifdef __cplusplus
 extern "C" {
#endif //__cplusplus
CeLedCtl ceLedCtl;      /*!< 定义全局变量*/
/**
  * @brief  定时器任务CeTicker的回调函数，详细可参考CREELINKS平台有关CeTicker相关文档
  * @param  pAddPar:ceLedCtl对象指针
  */
void ceLedCtl_callBack(void* pAddPar)
{
    if(ceLedCtl.ctlMode == CE_LED_CTL_MODE_FLASH_CYCLE_P)//顺时针从Led0到Led3循环点亮
    {
        switch (ceLedCtl.tick)
        {
        case 2:
        case 12:
        case 22:
        case 32:
            ceLed1COp.setOff(&(ceLedCtl.ceLed0));
            ceLed1COp.setOff(&(ceLedCtl.ceLed1));
            ceLed1COp.setOff(&(ceLedCtl.ceLed2));
            ceLed1COp.setOff(&(ceLedCtl.ceLed3));
            break;
        case 0:
            ceLed1COp.setOn(&(ceLedCtl.ceLed0));
            ceLed1COp.setOff(&(ceLedCtl.ceLed1));
            ceLed1COp.setOff(&(ceLedCtl.ceLed2));
            ceLed1COp.setOff(&(ceLedCtl.ceLed3));
            break;
        case 10:
            ceLed1COp.setOff(&(ceLedCtl.ceLed0));
            ceLed1COp.setOn(&(ceLedCtl.ceLed1));
            ceLed1COp.setOff(&(ceLedCtl.ceLed2));
            ceLed1COp.setOff(&(ceLedCtl.ceLed3));
            break;
        case 20:
            ceLed1COp.setOff(&(ceLedCtl.ceLed0));
            ceLed1COp.setOff(&(ceLedCtl.ceLed1));
            ceLed1COp.setOn(&(ceLedCtl.ceLed2));
            ceLed1COp.setOff(&(ceLedCtl.ceLed3));
            break;
        case 30:
            ceLed1COp.setOff(&(ceLedCtl.ceLed0));
            ceLed1COp.setOff(&(ceLedCtl.ceLed1));
            ceLed1COp.setOff(&(ceLedCtl.ceLed2));
            ceLed1COp.setOn(&(ceLedCtl.ceLed3));
            break;
        case 60:
            ceLedCtl.tick = -1;
            break;
        default:
            break;
        }
    }if(ceLedCtl.ctlMode == CE_LED_CTL_MODE_FLASH_CYCLE_N)//顺时针从Led3到Led0循环点亮
    {
        switch (ceLedCtl.tick)
        {
        case 2:
        case 12:
        case 22:
        case 32:
            ceLed1COp.setOff(&(ceLedCtl.ceLed0));
            ceLed1COp.setOff(&(ceLedCtl.ceLed1));
            ceLed1COp.setOff(&(ceLedCtl.ceLed2));
            ceLed1COp.setOff(&(ceLedCtl.ceLed3));
            break;
        case 0:
            ceLed1COp.setOff(&(ceLedCtl.ceLed0));
            ceLed1COp.setOff(&(ceLedCtl.ceLed1));
            ceLed1COp.setOff(&(ceLedCtl.ceLed2));
            ceLed1COp.setOn(&(ceLedCtl.ceLed3));
            break;
        case 10:
            ceLed1COp.setOff(&(ceLedCtl.ceLed0));
            ceLed1COp.setOff(&(ceLedCtl.ceLed1));
            ceLed1COp.setOn(&(ceLedCtl.ceLed2));
            ceLed1COp.setOff(&(ceLedCtl.ceLed3));
            break;
        case 20:
            ceLed1COp.setOff(&(ceLedCtl.ceLed0));
            ceLed1COp.setOn(&(ceLedCtl.ceLed1));
            ceLed1COp.setOff(&(ceLedCtl.ceLed2));
            ceLed1COp.setOff(&(ceLedCtl.ceLed3));
            break;
        case 30:
            ceLed1COp.setOn(&(ceLedCtl.ceLed0));
            ceLed1COp.setOff(&(ceLedCtl.ceLed1));
            ceLed1COp.setOff(&(ceLedCtl.ceLed2));
            ceLed1COp.setOff(&(ceLedCtl.ceLed3));
            break;
        case 60:
            ceLedCtl.tick = -1;
            break;
        default:
            break;
        }
    }else if(ceLedCtl.ctlMode == CE_LED_CTL_MODE_GOTO_FRONT)//由后向前闪亮
    {
        switch (ceLedCtl.tick)
        {
        case 2:
        case 12:
            ceLed1COp.setOff(&(ceLedCtl.ceLed0));
            ceLed1COp.setOff(&(ceLedCtl.ceLed1));
            ceLed1COp.setOff(&(ceLedCtl.ceLed2));
            ceLed1COp.setOff(&(ceLedCtl.ceLed3));
            break;
        case 0:
            ceLed1COp.setOff(&(ceLedCtl.ceLed0));
            ceLed1COp.setOff(&(ceLedCtl.ceLed1));
            ceLed1COp.setOn(&(ceLedCtl.ceLed2));
            ceLed1COp.setOn(&(ceLedCtl.ceLed3));

            break;
        case 10:
            ceLed1COp.setOn(&(ceLedCtl.ceLed0));
            ceLed1COp.setOn(&(ceLedCtl.ceLed1));
            ceLed1COp.setOff(&(ceLedCtl.ceLed2));
            ceLed1COp.setOff(&(ceLedCtl.ceLed3));
            break;
        case 60:
            ceLedCtl.tick = -1;
            break;
        default:
            break;
        }
    }else if(ceLedCtl.ctlMode == CE_LED_CTL_MODE_GOTO_BACK)//由前向后闪亮
    {
        switch (ceLedCtl.tick)
        {
        case 2:
        case 12:
            ceLed1COp.setOff(&(ceLedCtl.ceLed0));
            ceLed1COp.setOff(&(ceLedCtl.ceLed1));
            ceLed1COp.setOff(&(ceLedCtl.ceLed2));
            ceLed1COp.setOff(&(ceLedCtl.ceLed3));
            break;
        case 0:
            ceLed1COp.setOn(&(ceLedCtl.ceLed0));
            ceLed1COp.setOn(&(ceLedCtl.ceLed1));
            ceLed1COp.setOff(&(ceLedCtl.ceLed2));
            ceLed1COp.setOff(&(ceLedCtl.ceLed3));
            break;
        case 10:
            ceLed1COp.setOff(&(ceLedCtl.ceLed0));
            ceLed1COp.setOff(&(ceLedCtl.ceLed1));
            ceLed1COp.setOn(&(ceLedCtl.ceLed2));
            ceLed1COp.setOn(&(ceLedCtl.ceLed3));
            break;
        case 60:
            ceLedCtl.tick = -1;
            break;
        default:
            break;
        }
    }else if(ceLedCtl.ctlMode == CE_LED_CTL_MODE_GOTO_LEFT)//从右向左闪亮
    {
        switch (ceLedCtl.tick)
        {
        case 2:
        case 12:
            ceLed1COp.setOff(&(ceLedCtl.ceLed0));
            ceLed1COp.setOff(&(ceLedCtl.ceLed1));
            ceLed1COp.setOff(&(ceLedCtl.ceLed2));
            ceLed1COp.setOff(&(ceLedCtl.ceLed3));
            break;
        case 0:
            ceLed1COp.setOff(&(ceLedCtl.ceLed0));
            ceLed1COp.setOn(&(ceLedCtl.ceLed1));
            ceLed1COp.setOn(&(ceLedCtl.ceLed2));
            ceLed1COp.setOff(&(ceLedCtl.ceLed3));
            break;
        case 10:
            ceLed1COp.setOn(&(ceLedCtl.ceLed0));
            ceLed1COp.setOff(&(ceLedCtl.ceLed1));
            ceLed1COp.setOff(&(ceLedCtl.ceLed2));
            ceLed1COp.setOn(&(ceLedCtl.ceLed3));
            break;
        case 60:
            ceLedCtl.tick = -1;
            break;
        default:
            break;
        }

    }else if(ceLedCtl.ctlMode == CE_LED_CTL_MODE_GOTO_RIGHT)//从左向右闪亮
    {
        switch (ceLedCtl.tick)
        {
        case 2:
        case 12:
            ceLed1COp.setOff(&(ceLedCtl.ceLed0));
            ceLed1COp.setOff(&(ceLedCtl.ceLed1));
            ceLed1COp.setOff(&(ceLedCtl.ceLed2));
            ceLed1COp.setOff(&(ceLedCtl.ceLed3));
            break;
        case 0:
            ceLed1COp.setOn(&(ceLedCtl.ceLed0));
            ceLed1COp.setOff(&(ceLedCtl.ceLed1));
            ceLed1COp.setOff(&(ceLedCtl.ceLed2));
            ceLed1COp.setOn(&(ceLedCtl.ceLed3));
            break;
        case 10:
            ceLed1COp.setOff(&(ceLedCtl.ceLed0));
            ceLed1COp.setOn(&(ceLedCtl.ceLed1));
            ceLed1COp.setOn(&(ceLedCtl.ceLed2));
            ceLed1COp.setOff(&(ceLedCtl.ceLed3));
            break;
        case 60:
            ceLedCtl.tick = -1;
            break;
        default:
            break;
        }

    }else if(ceLedCtl.ctlMode == CE_LED_CTL_MODE_IN_CFG)//处在配置及初始化状态，快闪
    {
        switch (ceLedCtl.tick)
        {
        case 20:
            ceLed1COp.setOff(&(ceLedCtl.ceLed0));
            ceLed1COp.setOff(&(ceLedCtl.ceLed1));
            ceLed1COp.setOff(&(ceLedCtl.ceLed2));
            ceLed1COp.setOff(&(ceLedCtl.ceLed3));
            break;
        case 0:
            ceLed1COp.setOn(&(ceLedCtl.ceLed0));
            ceLed1COp.setOn(&(ceLedCtl.ceLed1));
            ceLed1COp.setOn(&(ceLedCtl.ceLed2));
            ceLed1COp.setOn(&(ceLedCtl.ceLed3));
            break;
        case 40:
            ceLedCtl.tick = -1;
            break;
        default:
            break;
        }

    }else if(ceLedCtl.ctlMode == CE_LED_CTL_MODE_IN_NORMAL)//正常状态
    {
        switch (ceLedCtl.tick)
        {
        case 2:
            ceLed1COp.setOff(&(ceLedCtl.ceLed0));
            ceLed1COp.setOff(&(ceLedCtl.ceLed1));
            ceLed1COp.setOff(&(ceLedCtl.ceLed2));
            ceLed1COp.setOff(&(ceLedCtl.ceLed3));
            break;
        case 0:
            ceLed1COp.setOn(&(ceLedCtl.ceLed0));
            ceLed1COp.setOn(&(ceLedCtl.ceLed1));
            ceLed1COp.setOn(&(ceLedCtl.ceLed2));
            ceLed1COp.setOn(&(ceLedCtl.ceLed3));
            break;
        case 100:
            ceLedCtl.tick = -1;
            break;
        default:
            break;
        }

    }else if(ceLedCtl.ctlMode == CE_LED_CTL_MODE_IN_ERROR)//错误状态，所有LED常亮
    {
        switch (ceLedCtl.tick)
        {
        case 2:
            ceLed1COp.setOn(&(ceLedCtl.ceLed0));
            ceLed1COp.setOn(&(ceLedCtl.ceLed1));
            ceLed1COp.setOn(&(ceLedCtl.ceLed2));
            ceLed1COp.setOn(&(ceLedCtl.ceLed3));

            break;
        case 0:
            ceLed1COp.setOff(&(ceLedCtl.ceLed0));
            ceLed1COp.setOff(&(ceLedCtl.ceLed1));
            ceLed1COp.setOff(&(ceLedCtl.ceLed2));
            ceLed1COp.setOff(&(ceLedCtl.ceLed3));
            break;
        case 60:
            ceLedCtl.tick = -1;
            break;
        default:
            break;
        }

    }
    ceLedCtl.tick++;
}

/**
  * @brief  CeLedCtl模块初始化
  * @param  @param ceGpioM0-3:四个LED使用的Gpio资源号
  * @return 系统状态码
  */
CE_STATUS ceLedCtl_initial(CE_RESOURCE ceGpioM0,CE_RESOURCE ceGpioM1,CE_RESOURCE ceGpioM2,CE_RESOURCE ceGpioM3)
{
    ceLedCtl.ctlMode = CE_LED_CTL_MODE_OFF;
    ceLedCtl.tick = 0;

    ceLed1COp.initialByGpio(&(ceLedCtl.ceLed0),ceGpioM0);
    ceLed1COp.initialByGpio(&(ceLedCtl.ceLed1),ceGpioM1);
    ceLed1COp.initialByGpio(&(ceLedCtl.ceLed2),ceGpioM2);
    ceLed1COp.initialByGpio(&(ceLedCtl.ceLed3),ceGpioM3);

    ceLedCtl.ceTicker.callBack = ceLedCtl_callBack;
    ceLedCtl.ceTicker.ID = (uint16)ceGpioM0;
    ceLedCtl.ceTicker.intervalMs = 10;
    ceLedCtl.ceTicker.pAddPar = &ceLedCtl;
    ceTickerOp.registerTicker(&(ceLedCtl.ceTicker)); 
    ceTickerOp.start(&(ceLedCtl.ceTicker));

    return CE_STATUS_SUCCESS;
}

/**
  * @brief 配置四个LED闪烁的方式
  * @param ctlMode:四个LED闪烁的方式
  */
void ceLedCtl_setMode(CE_LED_CTL_MODE ctlMode)
{
    if(ctlMode == ceLedCtl.ctlMode) return;//需要配置的状态与当前状态相同，则直接返回
    ceLedCtl.ctlMode = ctlMode;
    ceLedCtl.tick = 0;  

    ceLed1COp.setOff(&(ceLedCtl.ceLed0));
    ceLed1COp.setOff(&(ceLedCtl.ceLed1));
    ceLed1COp.setOff(&(ceLedCtl.ceLed2));
    ceLed1COp.setOff(&(ceLedCtl.ceLed3));
}
/**
  * @brief 获取当前四个LED闪烁的方式
  * @return 当前四个LED闪烁的方式
  */
CE_LED_CTL_MODE ceLedCtl_getMode()
{
    return ceLedCtl.ctlMode;
}
/**
  * @brief  初始化CeLedCtl模块操作对象
  */
const CeLedCtlOp ceLedCtlOp = {ceLedCtl_initial,ceLedCtl_setMode,ceLedCtl_getMode};

#ifdef __cplusplus
 }
#endif //__cplusplus
