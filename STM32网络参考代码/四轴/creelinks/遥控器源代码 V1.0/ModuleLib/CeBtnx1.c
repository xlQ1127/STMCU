/**
  ******************************************************************************
  * @file    CeBtnx1.c
  * @author  Creelinks Application Team
  * @version V1.0.0
  * @date    2017-01-06
  * @brief   适用于CeBtnx1模块的驱动库文件
  ******************************************************************************
  * @attention
  *
  *1)
  *
  * <h2><center>&copy; COPYRIGHT 2016 Darcern</center></h2>
  ******************************************************************************
  */
#include "CeBtnx1.h"
#ifdef __cplusplus
 extern "C" {
#endif //__cplusplus
/**
  * @brief  Gpio方式按键按下时的回调处理
  * @param  pAddPar:CeBtnx1属性对象
  * @return None
  */
void ceBtnx1_callBackTick(void* pAddPar)
{
    CeBtnx1* ceBtnx1 = (CeBtnx1*)pAddPar;
    if(ceGpioOp.getBit(&ceBtnx1->ceGpio) == 0x00)//如果按键被按下
    {
        if((ceBtnx1->btnStatus & 0x01) == 0x00)
        {
            if(ceBtnx1->callBackPressEvent != CE_NULL)
            {
                ceBtnx1->callBackPressEvent();
            }
            ceBtnx1->btnStatus |= 0x01;              //防止重复响应
        }
    }
    else
    {
        ceBtnx1->btnStatus &= (~0x01);
    }
}

/**
  * @brief  外部中断Int方式按键按下时的回调处理
  * @return None
  */
void ceBtnx1_callBackInt(void* pAddPar)
{
    if(((CeBtnx1*)pAddPar)->callBackPressEvent != CE_NULL)
    {
        ((CeBtnx1*)pAddPar)->callBackPressEvent();
    }
}

/**
  * @brief  CeBtnx1模块使用Gpio口来完成初始化
  * @param  ceBtnx1:CeBtnx1属性对象
  * @param  ceGpio:CeBtnx1模块使用的资源号
  * @param  callBackPressEvent:按键按下时的回调函数，不需要回调传CE_NULL即可
  * @return 系统状态码
  */
CE_STATUS ceBtnx1_initialByGpio(CeBtnx1* ceBtnx1, CE_RESOURCE ceGpio, void (*callBackPressEvent)(void))
{
    ceBtnx1->btnStatus = 0x00;
    ceBtnx1->callBackPressEvent = callBackPressEvent;
    ceBtnx1->ceGpio.ceResource = ceGpio;
    ceBtnx1->ceGpio.ceGpioMode = CE_GPIO_MODE_IPU;
    ceGpioOp.initial(&(ceBtnx1->ceGpio));

    if (ceBtnx1->callBackPressEvent != CE_NULL)
    {
        ceBtnx1->ceTicker.ID = ceGpio;
        ceBtnx1->ceTicker.callBack = ceBtnx1_callBackTick;
        ceBtnx1->ceTicker.intervalMs = 100;
        ceBtnx1->ceTicker.pAddPar = ceBtnx1;

        ceTickerOp.registerTicker(&(ceBtnx1->ceTicker));
        ceTickerOp.start(&(ceBtnx1->ceTicker));
    }
    else if(ceBtnx1->ceTicker.ID == ceGpio)
    {
        ceTickerOp.unRegister(&(ceBtnx1->ceTicker));
    }
    return CE_STATUS_SUCCESS;
}

/**
  * @brief  CeBtnx1模块使用外部中断Int来完成初始化
  * @param  ceBtnx1:CeBtnx1属性对象
  * @param  ceInt:CeBtnx1模块使用的资源号
  * @param  callBackPressEvent:按键按下时的回调函数，不需要回调传CE_NULL即可
  * @return 系统状态码
  */
CE_STATUS ceBtnx1_initialByInt(CeBtnx1* ceBtnx1, CE_RESOURCE ceInt, void (*callBackPressEvent)(void))
{
    ceBtnx1->btnStatus = 0x80;
    ceBtnx1->callBackPressEvent = callBackPressEvent;
    ceBtnx1->ceInt.ceResource = ceInt;
    ceBtnx1->ceInt.callBack = ceBtnx1_callBackInt;
    ceBtnx1->ceInt.ceIntMode = CE_INT_MODE_TRIGGER_FALLING;//常态高电平，下降沿触发
    ceBtnx1->ceInt.pAddPar = ceBtnx1;
    ceIntOp.initial(&(ceBtnx1->ceInt));
    if (ceBtnx1->callBackPressEvent != CE_NULL)
    {
        ceIntOp.start(&(ceBtnx1->ceInt));
    }
    return CE_STATUS_SUCCESS;
}

/**
  * @brief  获取CeBtnx1状态，返回1表明已按下，返回0表明未按下
  * @param  ceBtnx1:CeBtnx1属性对象
  * @return Gpio的电平值，0x01和0x00
  */
uint8 ceBtnx1_getStatus(CeBtnx1* ceBtnx1)
{
    if((ceBtnx1->btnStatus & 0x80) == 0x80)
    {
        return (ceIntOp.getBit(&(ceBtnx1->ceInt)) == 0x00)? 0x01:0x00;
    }
    else
    {
        return (ceGpioOp.getBit(&(ceBtnx1->ceGpio)) == 0x00)? 0x01:0x00;
    }
}

/**
  * @brief  等待按键按下(如果是外部中断Int方式初始化，请谨慎使用此方法)
  * @param  ceBtnx1:CeBtnx1属性对象
  * @param  outTimeMs:等待的超时时间，Ms
  * @return 系统状态码，CE_STATUS_SUCCESS或CE_STATUS_OUT_TIME
  */
CE_STATUS ceBtnx1_waitForPressDown(CeBtnx1* ceBtnx1, uint32 outTimeMs)
{
    uint32 temp = ceSystemOp.getSystemTickMs();
    if((ceBtnx1->btnStatus & 0x80) == 0x80)
    {
        while (ceIntOp.getBit(&(ceBtnx1->ceInt)) == 0x01)
        {
            if((ceSystemOp.getSystemTickMs() - temp) > outTimeMs)
            {
                return CE_STATUS_OUT_TIME;
            }
#ifdef CE_USE_RTOS          //如果是在操作系统环境下，进行任务切换
            ceSystemOp.delayMs(0);
#endif
        };
    }
    else
    {
        while (ceGpioOp.getBit(&(ceBtnx1->ceGpio)) == 0x01)
        {
            if((ceSystemOp.getSystemTickMs() - temp) > outTimeMs)
            {
                return CE_STATUS_OUT_TIME;
            }
        };
#ifdef CE_USE_RTOS          //如果是在操作系统环境下，进行任务切换
        ceSystemOp.delayMs(0);
#endif
    }
    return CE_STATUS_SUCCESS;
}

/**
  * @brief  等待按键弹起
  * @param  ceBtnx1:CeBtnx1属性对象
  * @param  outTimeMs:等待的超时时间，Ms
  * @return 系统状态码，CE_STATUS_SUCCESS或CE_STATUS_OUT_TIME
  */
CE_STATUS ceBtnx1_waitForPressUp(CeBtnx1* ceBtnx1, uint32 outTimeMs)
{
    uint32 temp = ceSystemOp.getSystemTickMs();
    if((ceBtnx1->btnStatus & 0x80) == 0x80)
    {
        while (ceIntOp.getBit(&(ceBtnx1->ceInt)) == 0x00)
        {
            if((ceSystemOp.getSystemTickMs() - temp) > outTimeMs)
            {
                return CE_STATUS_OUT_TIME;
            }
        };
    }
    else
    {
        while (ceGpioOp.getBit(&(ceBtnx1->ceGpio)) == 0x00)
        {
            if((ceSystemOp.getSystemTickMs() - temp) > outTimeMs)
            {
                return CE_STATUS_OUT_TIME;
            }
        };
    }
    return CE_STATUS_SUCCESS;
}

const CeBtnx1OpBase ceBtnx1Op = {ceBtnx1_initialByGpio, ceBtnx1_initialByInt, ceBtnx1_getStatus, ceBtnx1_waitForPressDown, ceBtnx1_waitForPressUp};

#ifdef __cplusplus
 }
#endif //__cplusplus
