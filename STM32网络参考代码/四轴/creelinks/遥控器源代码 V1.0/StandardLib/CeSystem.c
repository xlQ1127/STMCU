/**
  ******************************************************************************
  * @file    CeSystem.c
  * @author  Creelinks Application Team
  * @version V1.0.0
  * @date    2016-08-05
  * @brief   基于STM32F103RET6处理器平台的CeSystem资源函数实现库文件
  ******************************************************************************
  * @attention
  *
  *1)Debug模式下，printf函数在所有数据去发送完成后，才返回
  *2)请尽量勿在各种中断中调用printf函数
  *
  * <h2><center>&copy; COPYRIGHT 2016 Darcern</center></h2>
  ******************************************************************************
  */
#include "CeSystem.h"
#include "CeTicker.h"
#include "CeDebug.h"

#ifdef __cplusplus
 extern "C" {
#endif //__cplusplus
const char ceStatusString[9][30] = {{"CE_STATUS_SUCCESS"},{"CE_STATUS_FAILE"},{"CE_STATUS_RESOURCE_ERROR"},{"CE_STATUS_INITIAL_FALSE"},{"CE_STATUS_NULL_POINTER"},{"CE_STATUS_MALLOC_FALSE"},{"CE_STATUS_PAR_ERROR"},{"CE_STATUS_OUT_TIME"},{"UnknowError"}};

#define CE_PRIORITY_GROUP_CONFIG NVIC_PriorityGroup_2//基于STM32F103的中断组

CeSystem ceSystem;

#ifdef __CE_CHECK_PAR__

#ifdef  USE_FULL_ASSERT
extern void assert_failed(uint8_t* file, uint32_t line);
#endif //USE_FULL_ASSERT

/**
  * @brief   断言失败的处理函数，若使用到UART进行代码调式，通过UART输出错误信息，上位机接收此错误信息，通过此信息可以定位到断言失败的代码行
  * @param   file:断言失败的文件路径
  * @param   line:断言失败的代码在文件中的行数
  * @param   ceStatus:断言失败的状态码
  * @return  状态指示码
  */
void ce_assert_failed(uint8_t* file, uint32_t line, CE_STATUS ceStatus)
{
    if (ceStatus == CE_STATUS_SUCCESS)
    {
        return;
    }
    while (1)
    {
        ceDebugOp.printf("There is an error occurred:\n\tFile:%s\nLine:%d\nCE_STATUS:%s\n\n", file, line, ceSystemOp.getErrorMsg(ceStatus));
        ceSystemOp.delayMs(1000);
    }
}
#endif //__CE_CHECK_PAR__

/**
  * @brief    Creelinks平台中断初始化主函数，系统启动前需调用
  * @param    None
  * @return  状态指示码
  */
CE_STATUS ceSystem_GolbalInterruptInitial(void)
{
    NVIC_SetVectorTable(NVIC_VectTab_FLASH, 0x0);
    NVIC_PriorityGroupConfig(CE_PRIORITY_GROUP_CONFIG);
    return CE_STATUS_SUCCESS;
}

/**
  * @brief   System使用到的定时器中断回调函数
  * @param   pAddPar:空指针
  */
void ceSystem_callBackTickTimer(void* pAddPar)
{
    ceSystem.tickLoopNow++;
    ceTickerOp.callBySystem();
}

/**
  * @brief   初始化1Us精确定时器
  * @param   None
  * @return  状态指示码
  */
CE_STATUS ceSystem_initialTickTimer()
{
    ceSystem.ceTimer.callBack = ceSystem_callBackTickTimer;
    ceSystem.ceTimer.ceResource = CE_SYSTEM_DELAY_USE_TIMX;
    ceSystem.ceTimer.intervalNs = 5000000;//重要：定义定时器器益出中断周期的时间间隔，单位ns
    ceSystem.ceTimer.pAddPar = &ceSystem;
    ceSystem.tickLoopNow = 0;
    ceTimerOp.initial(&(ceSystem.ceTimer));
    ceTimerOp.start(&(ceSystem.ceTimer));
    return CE_STATUS_SUCCESS;
}

/**
  * @brief   获取系统从开机到现在所经过的时间,精度1Us
  * @param   None
  * @return  定时器运行的总时间
  */
uint64 ceSystem_getSystemTickUs(void)
{
    uint32 maxCnt,nowCnt;
    maxCnt = (uint32)(ceTimerOp.getTimerMaxCnt(&(ceSystem.ceTimer)));
    nowCnt = (uint32)(ceTimerOp.getTimerNowCnt(&(ceSystem.ceTimer),0x01)); 
    return ((uint64)maxCnt*ceSystem.tickLoopNow+nowCnt)/72;
}

/**
  * @brief   获取系统从开机到现在所经过的时间,精度1Ms
  * @param   None
  * @return  定时器运行的总时间
  */
uint64 ceSystem_getSystemTickMs(void)
{
    #ifdef __CE_USE_RTOS__
    return OSTimeGet();
    #else
    return ceSystem_getSystemTickUs()/1000;
    #endif
}

/**
  * @brief   Creelinks平台延时函数
  * @param   us:需要延时的时间，单位微秒
  * @return  None
  */
void ceSystem_delayUs(uint32 us)
{
    if(us == 0)
    {
        return;
    }else
    {
    uint64 maxCnt,nowCnt;    
    uint64 usTemp;    
      usTemp = ceSystem_getSystemTickUs();
    maxCnt = (uint64)(ceTimerOp.getTimerMaxCnt(&(ceSystem.ceTimer)));
    nowCnt = (uint64)(ceTimerOp.getTimerNowCnt(&(ceSystem.ceTimer),0x00)); 

    while((maxCnt*ceSystem.tickLoopNow+nowCnt)/72 - usTemp < us)
        nowCnt = (uint64)(ceTimerOp.getTimerNowCnt(&(ceSystem.ceTimer),0x00)); 
    }
}

/**
  * @brief   Creelinks平台延时函数
  * @param   ns:需要延时的时间，单位纳秒
  * @return  None
  */
void ceSystem_delayNs(uint32 ns)
{
   ceSystem_delayUs(ns / 1000);
}


/**
  * @brief   Creelinks平台延时函数
  * @param   us:需要延时的时间，单位毫秒
  * @return  None
  */
void ceSystem_delayMs(uint32 ms)
{
#ifdef __CE_USE_RTOS__
    if (OSRunning == 0x00)
        for (; ms > 0; ms--)
            ceSystem_delayUs(1000);
    else
        if (ms == 0)
            ceSystem_taskSchedule();//直接进行任务调度
        else
            OSTimeDly (ms);
#else
    uint64 usTemp = ceSystem_getSystemTickUs();
    uint64 maxCnt,nowCnt;
    maxCnt = (uint64)(ceTimerOp.getTimerMaxCnt(&(ceSystem.ceTimer)));
    nowCnt = (uint64)(ceTimerOp.getTimerNowCnt(&(ceSystem.ceTimer),0x00)); 

    while((maxCnt*ceSystem.tickLoopNow+nowCnt)/72 - usTemp < (uint64)ms*1000)
        nowCnt = (uint64)(ceTimerOp.getTimerNowCnt(&(ceSystem.ceTimer),0x00)); 
#endif
}

/**
  * @brief   根据返回的错误码，得到char*类型的错误文字信息
  * @param   ceStatus:状态码
  * @return  以字符串方式返回状态码
  */
const char* ceSystem_getErrorMsg(CE_STATUS ceStatus)
{
    return ceStatusString[(uint8)ceStatus];
}


/**
  * @brief   Creelinks平台初始化主函数，系统启动前需调用
  * @param   None
  * @return  状态指示码
  */
CE_STATUS ceSystem_initial(void)
{
    ceSystem_GolbalInterruptInitial();
    ceSystem_initialTickTimer();

    #ifdef __CE_USE_RTOS__
    OSInit();
    #endif
    return CE_STATUS_SUCCESS;
}

const CeSystemOp ceSystemOp = {ceSystem_initial,
                               ceSystem_delayNs,ceSystem_delayUs, ceSystem_delayMs, ceSystem_getSystemTickUs, ceSystem_getSystemTickMs, 
                               ceSystem_getErrorMsg};
#ifdef __cplusplus
 }
#endif //__cplusplus
