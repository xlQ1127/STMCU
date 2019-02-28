/**
  ******************************************************************************
  * @file    CeTicker.c
  * @author  Creelinks Application Team
  * @version V1.0.0
  * @date    2016-08-05
  * @brief   基于STM32F103RET6处理器平台的CeTicker资源函数实现库文件
  ******************************************************************************
  * @attention
  *
  *1)定时器最小间隔为1ms,精度根据不同的处理器平台而不同，针对STM32F103，约为正负2us。
  *2)针对无操作系统的平台，注册的定时器函数运行在系统中断内；针对有操作系统的平台，则运行在一个高优先级的线程内，故尽量别在定时器进行内进行耗时操作。
  *
  * <h2><center>&copy; COPYRIGHT 2016 Darcern</center></h2>
  ******************************************************************************
  */
#include "CeTicker.h"
#include "CeTask.h"
#include "CeSystem.h"
#ifdef __cplusplus
 extern "C" {
#endif //__cplusplus

CeTicker* ceTickerList = CE_NULL;
uint32 ceTickerLastMs = 0;

#ifdef __CE_CHECK_PAR__
/**
  * @brief   检查用户传递的ceTicker指针是否正确
  * @param   ceTicker:ceTickert属性对象指针
  * @return  系统状态码，可能的值:CE_STATUS_SUCCESS、CE_STATUS_INITIAL_FALSE、CE_STATUS_NULL_POINTER、CE_STATUS_PAR_ERROR
  */
CE_STATUS ceCheckCeTicker(CeTicker* ceTicker)
{
    if (ceTicker == CE_NULL)
    {
        return CE_STATUS_NULL_POINTER;
    }
    if (ceTicker->intervalMs == 0)
    {
        return CE_STATUS_PAR_ERROR;
    }
    if (ceTicker->callBack == CE_NULL)
    {
        return CE_STATUS_INITIAL_FALSE;
    }
    return CE_STATUS_SUCCESS;
}
#endif //__CE_CHECK_PAR__

/**
  * @brief   由系统调每1ms调用一次的函数
  * @param   ceTicker:ceTickert属性对象指针
  * @return  None
  */
CE_STATUS ceTicker_callBySystem()
{
    uint64 ttt = ceSystemOp.getSystemTickMs();
    
    if(ttt - ceTickerLastMs < CE_TICKER_CALL_TIME_MS) return CE_STATUS_SUCCESS;
    ceTickerLastMs = ceSystemOp.getSystemTickMs();
    if(ceTickerList != CE_NULL)
    {
        CeTicker* ceTickerTemp = ceTickerList;
        while(ceTickerTemp != CE_NULL)
        {
            if(ceTickerTemp->ceExTickerPar.isRunning == 0x01)
            {
                ceTickerTemp->ceExTickerPar.nowTick += CE_TICKER_CALL_TIME_MS;
                if(ceTickerTemp->ceExTickerPar.nowTick >= ceTickerTemp->intervalMs)
                {
                    ceTickerTemp->callBack(ceTickerTemp->pAddPar);
                    ceTickerTemp->ceExTickerPar.nowTick = 0x0000;
                }
            }
            ceTickerTemp = ceTickerTemp->nextCeTicker;
        }
    }
    return CE_STATUS_SUCCESS;
}

/**
  * @brief   注册一个定时器
  * @param   ceTicker:ceTickert属性对象指针
  * @return  系统状态码，可能的值:CE_STATUS_SUCCESS
  */
CE_STATUS ceTicker_register(CeTicker* ceTicker)
{
#ifdef __CE_CHECK_PAR__
    ce_assert_failed((uint8*)__FILE__, __LINE__, ceCheckCeTicker(ceTicker));
#endif //__CE_CHECK_PAR__

    ceTicker->nextCeTicker = CE_NULL;
    ceTicker->ceExTickerPar.isRunning = 0x00;

    if (ceTickerList == CE_NULL)
    {
        ceTickerList = ceTicker;
    } else
    {
        CeTicker* ceTickerTemp = ceTickerList;
        while (1)
        {
            if (ceTicker->ID == ceTickerTemp->ID)
            {
                break;
            }
            if (ceTickerTemp->nextCeTicker == CE_NULL)
            {
                ceTickerTemp->nextCeTicker = ceTicker;
                break;
            }
            ceTickerTemp = ceTickerTemp->nextCeTicker;
        }
    }
    return CE_STATUS_SUCCESS;
}

/**
  * @brief   开始一个定时器任务
  * @param   ceTicker:ceTickert属性对象指针
  * @return  系统状态码，可能的值:CE_STATUS_SUCCESS
  */
CE_STATUS ceTicker_start(CeTicker* ceTicker)
{
    if (ceTickerList == CE_NULL)
    {
        return CE_STATUS_FAILE;
    } else
    {
        CeTicker* ceTickerTemp = ceTickerList;

#ifdef __CE_CHECK_PAR__
        ce_assert_failed((uint8*)__FILE__, __LINE__, ceCheckCeTicker(ceTicker));
#endif //__CE_CHECK_PAR__

        while (1)
        {
            if (ceTicker->ID == ceTickerTemp->ID)
            {
                ceTickerTemp->ceExTickerPar.isRunning = 0x01;
                ceTickerTemp->ceExTickerPar.nowTick = 0x0000;
                break;
            }
            ceTickerTemp = ceTickerTemp->nextCeTicker;
        }
    }
    return CE_STATUS_SUCCESS;
}

/**
  * @brief   停止一个定时器任务
  * @param   ceTicker:ceTickert属性对象指针
  * @return  系统状态码，可能的值:CE_STATUS_SUCCESS
  */
CE_STATUS cTicker_stop(CeTicker* ceTicker)
{
    if (ceTickerList == CE_NULL)
    {
        return CE_STATUS_FAILE;
    } else
    {
        CeTicker* ceTickerTemp = ceTickerList;
        #ifdef __CE_CHECK_PAR__
        ce_assert_failed((uint8*)__FILE__, __LINE__, ceCheckCeTicker(ceTicker));
        #endif //__CE_CHECK_PAR__

        while (1)
        {
            if (ceTicker->ID == ceTickerTemp->ID)
            {
                ceTickerTemp->ceExTickerPar.isRunning = 0x00;
                break;
            }
            ceTickerTemp = ceTickerTemp->nextCeTicker;
        }
    }
    return CE_STATUS_SUCCESS;
}

/**
  * @brief   删除一个定时器任务
  * @param   ceTicker:ceTickert属性对象指针
  * @return  系统状态码，可能的值:CE_STATUS_SUCCESS
  */
CE_STATUS ceTicker_unRegister(CeTicker* ceTicker)
{
    if (ceTickerList == CE_NULL)
    {
        return CE_STATUS_FAILE;
    } else
    {
        CeTicker* ceTickerBefore = CE_NULL;
        CeTicker* ceTickerTemp = ceTickerList;
        #ifdef __CE_CHECK_PAR__
        ce_assert_failed((uint8*)__FILE__, __LINE__, ceCheckCeTicker(ceTicker));
        #endif //__CE_CHECK_PAR__

        while (1)
        {
            if (ceTicker->ID == ceTickerTemp->ID)
            {
                if (ceTickerBefore == CE_NULL)
                {
                    ceTickerList = CE_NULL;
                } else
                {
                    if (ceTickerTemp->nextCeTicker == CE_NULL)
                    {
                        ceTickerBefore->nextCeTicker = CE_NULL;
                    } else
                    {
                        ceTickerBefore->nextCeTicker = ceTickerTemp->nextCeTicker;
                    }
                }
                break;
            }
            ceTickerBefore = ceTickerTemp;
            ceTickerTemp = ceTickerTemp->nextCeTicker;
        }
    }
    return CE_STATUS_SUCCESS;
}

const CeTickerOp ceTickerOp = {ceTicker_register, ceTicker_start, cTicker_stop, ceTicker_unRegister,ceTicker_callBySystem};

#ifdef __cplusplus
}
#endif //__cplusplus
