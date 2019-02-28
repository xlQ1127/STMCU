/**
  ******************************************************************************
  * @file    CeLed1C.c
  * @author  Creelinks Application Team
  * @version V1.0.0
  * @date    2017-01-06
  * @brief   适用于CeLed7C模块的驱动库文件
  ******************************************************************************
  * @attention
  *
  *1)
  *
  * <h2><center>&copy; COPYRIGHT 2016 Darcern</center></h2>
  ******************************************************************************
  */
#include"CeLed1C.h"
#ifdef __cplusplus
 extern "C" {
#endif //__cplusplus

void ceLed1C_setOn(CeLed1C* ceLed1C);
void ceLed1C_setOff(CeLed1C* ceLed1C);
/**
  * @brief  CeLed1C闪烁回调函数
  * @param  pAddPar:CeLed1C属性对象
  * @return None
  */
void ceLed1C_TickCallBack(void* pAddPar)
{
    CeLed1C* ceLed1C = (CeLed1C*)pAddPar;
    if (ceLed1C->ledMode == 0x00)
    {
        if (ceLed1C->isBreathUp == 0x01)
        {
            ceLed1C->cePwm.dutyNs += ceLed1C->breathAdd;
            if (ceLed1C->cePwm.dutyNs >= ceLed1C->cePwm.cycleNs)
            {
                ceLed1C->cePwm.dutyNs = ceLed1C->cePwm.cycleNs-1;
                ceLed1C->isBreathUp = 0x00;
            }
        }
        else
        {
            if (ceLed1C->cePwm.dutyNs < ceLed1C->breathAdd + CE_PWM_MIN_CYCLE_NS)
            {
                ceLed1C->cePwm.dutyNs = CE_PWM_MIN_CYCLE_NS;
                ceLed1C->isBreathUp = 0x01;
            }
            else
            {
                ceLed1C->cePwm.dutyNs -= ceLed1C->breathAdd;
            }
        }
        cePwmOp.updata(&(ceLed1C->cePwm));
    }
    else
    {
        if (ceLed1C->isGpio == 0x01)
        {
            if(ceLed1C->flashUpMs == ceLed1C->ceTicker.intervalMs)
            {
                ceLed1C->ceTicker.intervalMs = ceLed1C->falshDownMs;
                ceGpioOp.resetBit(&(ceLed1C->ceGpio));
            }
            else
            {
                ceLed1C->ceTicker.intervalMs = ceLed1C->flashUpMs;
                ceGpioOp.setBit(&(ceLed1C->ceGpio));
            }
        }
        else
        {
            if(ceLed1C->flashUpMs == ceLed1C->ceTicker.intervalMs)
            {
                ceLed1C->ceTicker.intervalMs = ceLed1C->falshDownMs;
                cePwmOp.resetBit(&(ceLed1C->cePwm));
            }
            else
            {
                ceLed1C->ceTicker.intervalMs = ceLed1C->flashUpMs;
                cePwmOp.setBit(&(ceLed1C->cePwm));
            }
        }
    }
}

/**
  * @brief  CeLed1C模块初始化
  * @param  ceLed1C:CeLed1C属性对象
  * @param  ceGpio:CeLed1C模块使用的资源号
  * @return None
  */
void ceLed1C_initialByGpio(CeLed1C* ceLed1C, CE_RESOURCE ceGpio)
{
    ceLed1C->isGpio = 0x01;

    ceLed1C->ceGpio.ceResource = ceGpio;
    ceLed1C->ceGpio.ceGpioMode = CE_GPIO_MODE_OUT_PP;
    ceGpioOp.initial(&(ceLed1C->ceGpio));
    ceGpioOp.resetBit(&(ceLed1C->ceGpio));

    ceLed1C->ceTicker.ID = ceGpio;
    ceLed1C->ceTicker.intervalMs = 1000;
    ceLed1C->ceTicker.pAddPar = ceLed1C;
    ceLed1C->ceTicker.callBack = ceLed1C_TickCallBack;
    ceTickerOp.registerTicker(&(ceLed1C->ceTicker));
}

/**
  * @brief  CeLed1C模块初始化
  * @param  ceLed1C:CeLed1C属性对象
  * @param   ceGpio:CeLed1C模块使用的资源号
  * @return None
  */
void ceLed1C_initialByPwm(CeLed1C* ceLed1C, CE_RESOURCE cePwm)
{
    ceLed1C->isGpio = 0x00;

    ceLed1C->cePwm.ceResource = cePwm;
    ceLed1C->cePwm.cycleNs = 6000000;
    ceLed1C->cePwm.dutyNs = ceLed1C->cePwm.cycleNs / 2;
    cePwmOp.initial(&(ceLed1C->cePwm));
    cePwmOp.resetBit(&(ceLed1C->cePwm));

    ceLed1C->ceTicker.ID = cePwm;
    ceLed1C->ceTicker.intervalMs = 1000;
    ceLed1C->ceTicker.pAddPar = ceLed1C;
    ceLed1C->ceTicker.callBack = ceLed1C_TickCallBack;
    ceTickerOp.registerTicker(&(ceLed1C->ceTicker));
}


/**
  * @brief  设置Led状态为开
  * @param  ceLed1C:CeLed1C属性对象
  * @return None
  */
void ceLed1C_setOn(CeLed1C* ceLed1C)
{
    if (ceLed1C->isGpio == 0x01)
    {
        ceTickerOp.stop(&(ceLed1C->ceTicker));
        ceGpioOp.setBit(&(ceLed1C->ceGpio));
    }
    else
    {
        ceTickerOp.stop(&(ceLed1C->ceTicker));
        cePwmOp.stop(&(ceLed1C->cePwm));
        cePwmOp.setBit(&(ceLed1C->cePwm));
    }
}

/**
  * @brief  设置Led状态为关
  * @param  ceLed1C:CeLed1C属性对象
  * @return None
  */
void ceLed1C_setOff(CeLed1C* ceLed1C)
{
    if (ceLed1C->isGpio == 0x01)
    {
        ceTickerOp.stop(&(ceLed1C->ceTicker));
        ceGpioOp.resetBit(&(ceLed1C->ceGpio));
    }
    else
    {
        ceTickerOp.stop(&(ceLed1C->ceTicker));
        cePwmOp.stop(&(ceLed1C->cePwm));
        cePwmOp.resetBit(&(ceLed1C->cePwm));
    }
}

/**
  * @brief  设置Led状态为闪烁
  * @param  ceLed1C:CeLed1C属性对象
  * @param  flashMs:闪烁周期，即完成一次亮与灭的时间间隔
  * @return None
  */
void ceLed1C_setFlash(CeLed1C* ceLed1C, uint16 flashUpMs,uint16 flashDownMs)
{
    ceLed1C->ledMode = 0x01;
    ceLed1C->flashUpMs = flashUpMs;
    ceLed1C->falshDownMs = flashDownMs;
    ceLed1C->ceTicker.intervalMs = flashDownMs;
    ceTickerOp.start(&(ceLed1C->ceTicker));
}

/**
  * @brief  设置Led状态为呼吸
  * @param  ceLed1C:CeLed1C属性对象
  * @param  flashMs:呼吸周期，即完成一次亮与灭的时间间隔
  * @return None
  */
void ceLed1C_setBreath(CeLed1C* ceLed1C, uint16 flashMs)
{
    if (ceLed1C->isGpio == 0x01)
    {
        ceDebugOp.printf("Can not set breath is Giop resource!\n");
        return;
    }
    ceLed1C->ledMode = 0x00;
    ceLed1C->isBreathUp = 0x01;

    ceLed1C->breathAdd = ceLed1C->cePwm.cycleNs / (flashMs / 50 + 1);
    ceLed1C->ceTicker.intervalMs = 20;
    ceTickerOp.start(&(ceLed1C->ceTicker));
    cePwmOp.start(&(ceLed1C->cePwm));
}

const CeLed1COpBase ceLed1COp = {ceLed1C_initialByGpio, ceLed1C_initialByPwm, ceLed1C_setOn, ceLed1C_setOff, ceLed1C_setFlash, ceLed1C_setBreath};

#ifdef __cplusplus
 }
#endif //__cplusplus
