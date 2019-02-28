/**
  ******************************************************************************
  * @file    CeBeepNSrc.c
  * @author  Creelinks Application Team
  * @version V1.0.0
  * @date    2017-01-06
  * @brief   适用于CeBeepNSrc模块的驱动库文件
  ******************************************************************************
  * @attention
  *
  *1)无
  *
  * <h2><center>&copy; COPYRIGHT 2016 Darcern</center></h2>
  ******************************************************************************
  */
#include "CeBeepNSrc.h"
#ifdef __cplusplus
 extern "C" {
#endif //__cplusplus

/**
  * @brief  CeBeepNSrc模块初始化
  * @param  ceBeepNSrc:CeBeepNSrc属性对象
  * @param  ceXX:CeBeepNSrc模块使用的资源号
  * @return 系统状态码
  */
CE_STATUS ceBeepNSrc_initial(CeBeepNSrc* ceBeepNSrc, CE_RESOURCE ceGpio)
{
    ceBeepNSrc->ceGpio.ceResource = ceGpio;
    ceBeepNSrc->ceGpio.ceGpioMode = CE_GPIO_MODE_OUT_PP;
    ceGpioOp.initial(&(ceBeepNSrc->ceGpio));
    ceGpioOp.resetBit(&(ceBeepNSrc->ceGpio));
    return CE_STATUS_SUCCESS;
}

CE_STATUS ceBeepNSrc_initialByPwm(CeBeepNSrc* ceBeepNSrc, CE_RESOURCE cePwm)
{
        return CE_STATUS_SUCCESS;
}

/**
  * @brief CeBeepNSrc发声，由于是无源，所以线程将会被堵塞，直到发声完成
  * @param ceBeepNSrc:CeBeepNSrc属性对象指针
  * @param durationMs:发声时间，单位毫秒
  * @param sleepMs:停止发声时间
  * @param beepTimes:发声次数
  */
void ceBeepNSrc_say(CeBeepNSrc* ceBeepNSrc, uint16 sayMs,uint16 sleepMs, uint8 beepTimes)
{
    int loop,i;
    for(i=0;i<beepTimes;i++)
    {
         ceTaskOp.inCriticalSection();
         for(loop = 0;loop<sayMs*5;loop++)//5Khz   
         {
             CE_SET_GPIO_BIT(&(ceBeepNSrc->ceGpio));
             ceSystemOp.delayUs(100);
             CE_RESET_GPIO_BIT(&(ceBeepNSrc->ceGpio));    
             ceSystemOp.delayUs(100);
         }
         ceTaskOp.outCriticalSection();
         ceSystemOp.delayMs(sleepMs);
    }
}


/**
  * @brief  CeBeepNSrc模块操作对象定义
  */
const CeBeepNSrcOp ceBeepNSrcOp = {ceBeepNSrc_initial,ceBeepNSrc_initialByPwm,ceBeepNSrc_say};

#ifdef __cplusplus
 }
#endif //__cplusplus
