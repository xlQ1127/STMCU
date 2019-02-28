/**
  ******************************************************************************
  * @file    CeMD.c
  * @author  Creelinks Application Team
  * @version V1.0.0
  * @date    2017-03-26
  * @brief   电机电调驱动文件，即将0~1000的驱动为转化为0~100%占空比的PWM输出
  ******************************************************************************
  * @attention
  *
  *1)输入0~1000的驱动强度，输出对应为0~100%的占空比
  *
  * <h2><center>&copy; COPYRIGHT 2016 Darcern</center></h2>
  ******************************************************************************
  */
#include "CeMD.h"
#ifdef __cplusplus
extern "C" {
#endif //__cplusplus

/**
  * @brief  CeMD模块初始化
  * @param  ceMD:CeMD属性对象
  * @param  ceXX:CeMD模块使用的资源号
  * @return 系统状态码
  */
CE_STATUS ceMD_initial(CeMD *ceMD, CE_RESOURCE cePwm)
{
    ceMD->cePwm.ceResource = cePwm;
    ceMD->cePwm.cycleNs = CE_MD_MAX_PWM_CYCLE_NS;
    #ifdef CE_MD_REVERSE
    ceMD->cePwm.dutyNs = CE_MD_MAX_PWM_DUTY_NS;
    #else
    ceMD->cePwm.dutyNs = CE_MD_MIN_PWM_DUTY_NS;
    #endif
    cePwmOp.initial(&(ceMD->cePwm));
        cePwmOp.resetBit(&(ceMD->cePwm));    
    cePwmOp.start(&(ceMD->cePwm));
    return CE_STATUS_SUCCESS;
}

/**
  * @brief  设置Pwm的驱动强度，0~1000，对应占空比为0%~100%
  * @param  ceMD:CeMD属性对象
  * @param  driverPower:Pwm的驱动强度，0~1000，对应占空比为0%~100%
  */
void ceMD_setDriverPower(CeMD *ceMD, uint16 driverPower)
{
    if(driverPower > 1000)
    {
        driverPower = 1000;
    }
    #ifdef CE_MD_REVERSE
    ceMD->cePwm.dutyNs = CE_MD_MAX_PWM_CYCLE_NS - CE_MD_MIN_PWM_DUTY_NS + (uint32)(((uint64)(CE_MD_MAX_PWM_DUTY_NS - CE_MD_MIN_PWM_DUTY_NS)*driverPower)/1000);
    #else
    ceMD->cePwm.dutyNs = CE_MD_MIN_PWM_DUTY_NS + (uint32)(((uint64)(CE_MD_MAX_PWM_DUTY_NS - CE_MD_MIN_PWM_DUTY_NS)*driverPower)/1000);
    #endif
        
    cePwmOp.updata(&(ceMD->cePwm));//更新Pwm输出
}

/**
  * @brief  初始化CeMD模块操作对象
  */
const CeMDOp ceMDOp = {ceMD_initial,ceMD_setDriverPower};


#ifdef __cplusplus
}
#endif //__cplusplus
