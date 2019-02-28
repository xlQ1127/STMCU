/**
  ******************************************************************************
  * @file    CeTask.h
  * @author  Creelinks Application Team
  * @version V1.0.0
  * @date    2016-08-05
  * @brief   基于STM32F103RET6处理器平台的CeTask资源函数实现库文件
  ******************************************************************************
  * @attention
  *
  *1)针对前后台操作无操作系统软件平台，为主函数中主循环mainTask函数，故用户请保证mainTask函数会周期调用而不被阻塞。
  *
  * <h2><center>&copy; COPYRIGHT 2016 Darcern</center></h2>
  ******************************************************************************
  */
#include "CeTask.h"
#include "CeSystem.h"
#ifdef __cplusplus
 extern "C" {
#endif //__cplusplus

#ifdef __CE_USE_RTOS__
#define CE_TASK_PRIORITY_START_BASE 5                               /*!< uCOSII 可创建64个任务，其中0、1、2、3/4任务为保留，故这里从5开始，HH级任务的起始ID，可创建11个HH级任务*/
#define CE_TASK_PRIORITY_H_BASE     5 + OS_LOWEST_PRIO / 5          /*!< H级任务的起始ID，可创建11个H级任务*/
#define CE_TASK_PRIORITY_M_BASE     5 + OS_LOWEST_PRIO * 2 / 5      /*!< M任务的起始ID，可创建11个M级任务*/
#define CE_TASK_PRIORITY_L_BASE     5 + OS_LOWEST_PRIO * 3 / 5      /*!< L级任务的起始ID，可创建11个H级任务*/
#define CE_TASK_PRIORITY_LL_BASE    5 + OS_LOWEST_PRIO * 4 / 5      /*!< LL级任务的起始ID，可创建11个H级任务*/
#define CE_TASK_PRIORITY_END_BASE   OS_LOWEST_PRIO                  /*!< uCOSII 可创建64个任务，其中63、62、61、60任务为保留，故这里到59节止*/
uint8 ceTask_priorityHHIndex    =   CE_TASK_PRIORITY_START_BASE;
uint8 ceTask_priorityHIndex     =   CE_TASK_PRIORITY_H_BASE;
uint8 ceTask_priorityMIndex     =   CE_TASK_PRIORITY_M_BASE;
uint8 ceTask_priorityLIndex     =   CE_TASK_PRIORITY_L_BASE;
uint8 ceTask_priorityLLIndex    =   CE_TASK_PRIORITY_LL_BASE;

#define CE_MAIN_TASK_STACK_SIZE  2048
CE_STK  CE_MAIN_TASK_STACK[CE_MAIN_TASK_STACK_SIZE];
CeTask ceTaskRtos;
#endif //CE_USE_RTOS

CeTask* ceTaskList = CE_NULL;
uint16 ceTask_criticalTimes = 0;//进行临界段的次数

#ifdef __CE_CHECK_PAR__
/**
  * @brief   检查ceTask参数的正确性
  * @param   ceTask:ceTask属性对象指针
  * @return  系统状态码，可能的值:CE_STATUS_SUCCESS、CE_STATUS_RESOURCE_ERROR、CE_STATUS_NULL_POINTER
  */
CE_STATUS ceCheckCeTask(CeTask* ceTask)
{
    if (ceTask == CE_NULL)
    {
        return CE_STATUS_NULL_POINTER;
    }
    if (ceTask->callBack == CE_NULL)
    {
        return CE_STATUS_INITIAL_FALSE;
    }
#ifdef __CE_USE_RTOS__
    if (ceTask->taskPriority > 4)
    {
        return CE_STATUS_PAR_ERROR;
    }
    if (ceTask->isNewThread != 0x01 && ceTask->isNewThread != 0x00)
    {
        return CE_STATUS_PAR_ERROR;
    }
    if ( ceTask->taskPriority > CE_TASK_PRIORITY_LL || ceTask->taskStackBuf == CE_NULL || ceTask->taskStackBufSize == 0)
    {
        return CE_STATUS_PAR_ERROR;
    }
#endif //CE_USE_RTOS
    return CE_STATUS_SUCCESS;
}
#endif //__CE_CHECK_PAR__


#ifdef __CE_USE_RTOS__
/**
  * @brief   由于不能保证用户提供的任务函数是否为死循环，所有这里给出死循环来调用用户的任务函数
  * @param   pAddPar:ceTask属性对象指针
  */
void ceTask_uCOSIITaskPlam(void* pAddPar)
{
    while (1)
    {
        ((CeTask*)(pAddPar))->callBack(((CeTask*)(pAddPar))->pAddPar);
    };
}
#endif //CE_USE_RTOS


/**
  * @brief  系统主循环中调用的函数
  * @param  ceTask:ceTask属性对象指针
  */
CE_STATUS ceTask_mainTask(void)
{
    #ifdef __CE_USE_RTOS__
    OS_CPU_SysTickInit();
    //OSStatInit();             //初始化统计任务
    OSStart();                  //main函数执行到这里后，就不再向下执行了，也就是说程序在这里停止执行了。
    #else 
    if (ceTaskList != CE_NULL)
    {
        CeTask* ceTaskTemp = ceTaskList;
        while (ceTaskTemp != CE_NULL)
        {
            if (ceTaskTemp->ceExTaskPar.isRunning == 0x01)
            {
                ceTaskTemp->callBack(ceTaskTemp->pAddPar);
            }
            ceTaskTemp = ceTaskTemp->nextCeTask;
        }
    }
    #endif
    return CE_STATUS_SUCCESS;
}

/**
  * @brief   注册一个CeTask
  * @param   ceTask:ceTask属性对象指针
  * @return  系统状态码，可能的值:CE_STATUS_SUCCESS、CE_STATUS_RESOURCE_ERROR、CE_STATUS_NULL_POINTER
  */
CE_STATUS ceTask_register(CeTask* ceTask)
{
#ifdef __CE_CHECK_PAR__
    ce_assert_failed((uint8*)__FILE__, __LINE__, ceCheckCeTask(ceTask));
#endif //__CE_CHECK_PAR__
    ceTask->nextCeTask = CE_NULL;
    ceTask->ceExTaskPar.isRunning = 0x00;

#ifdef __CE_USE_RTOS__
    switch (ceTask->taskPriority)//自动进行优化级的分配，先注册的任务的优先级更高
    {
    case CE_TASK_PRIORITY_HH:
        if (ceTask_priorityHHIndex != CE_TASK_PRIORITY_H_BASE)
        {
            ceTask->ceExTaskPar.ceExTaskPriority = ceTask_priorityHHIndex;
            ceTask_priorityHHIndex++;
        }
        else if(ceTask_priorityHIndex != CE_TASK_PRIORITY_M_BASE)
        {
            ceTask->ceExTaskPar.ceExTaskPriority = ceTask_priorityHIndex;
            ceTask_priorityHIndex++;
        }else if (ceTask_priorityMIndex != CE_TASK_PRIORITY_L_BASE)
        {
            ceTask->ceExTaskPar.ceExTaskPriority = ceTask_priorityMIndex;
            ceTask_priorityMIndex++;
        }
        else if (ceTask_priorityLIndex != CE_TASK_PRIORITY_LL_BASE)
        {
            ceTask->ceExTaskPar.ceExTaskPriority = ceTask_priorityLIndex;
            ceTask_priorityLIndex++;
        }
        else if (ceTask_priorityLLIndex != CE_TASK_PRIORITY_END_BASE)
        {
            ceTask->ceExTaskPar.ceExTaskPriority = ceTask_priorityLLIndex;
            ceTask_priorityLLIndex++;
        }
        break;
    case CE_TASK_PRIORITY_H:
        if (ceTask_priorityHIndex != CE_TASK_PRIORITY_M_BASE)
        {
            ceTask->ceExTaskPar.ceExTaskPriority = ceTask_priorityHIndex;
            ceTask_priorityHIndex++;
        }
        else if (ceTask_priorityMIndex != CE_TASK_PRIORITY_L_BASE)
        {
            ceTask->ceExTaskPar.ceExTaskPriority = ceTask_priorityMIndex;
            ceTask_priorityMIndex++;
        }
        else if (ceTask_priorityLIndex != CE_TASK_PRIORITY_LL_BASE)
        {
            ceTask->ceExTaskPar.ceExTaskPriority = ceTask_priorityLIndex;
            ceTask_priorityLIndex++;
        }
        else if (ceTask_priorityLLIndex != CE_TASK_PRIORITY_END_BASE)
        {
            ceTask->ceExTaskPar.ceExTaskPriority = ceTask_priorityLLIndex;
            ceTask_priorityLLIndex++;
        }
        break;
    case CE_TASK_PRIORITY_M:
        if (ceTask_priorityMIndex != CE_TASK_PRIORITY_L_BASE)
        {
            ceTask->ceExTaskPar.ceExTaskPriority = ceTask_priorityMIndex;
            ceTask_priorityMIndex++;
        }
        else if (ceTask_priorityLIndex != CE_TASK_PRIORITY_LL_BASE)
        {
            ceTask->ceExTaskPar.ceExTaskPriority = ceTask_priorityLIndex;
            ceTask_priorityLIndex++;
        }
        else if (ceTask_priorityLLIndex != CE_TASK_PRIORITY_END_BASE)
        {
            ceTask->ceExTaskPar.ceExTaskPriority = ceTask_priorityLLIndex;
            ceTask_priorityLLIndex++;
        }
        break;
    case CE_TASK_PRIORITY_L:
        if (ceTask_priorityLIndex != CE_TASK_PRIORITY_LL_BASE)
        {
            ceTask->ceExTaskPar.ceExTaskPriority = ceTask_priorityLIndex;
            ceTask_priorityLIndex++;
        }
        else if (ceTask_priorityLLIndex != CE_TASK_PRIORITY_END_BASE)
        {
            ceTask->ceExTaskPar.ceExTaskPriority = ceTask_priorityLLIndex;
            ceTask_priorityLLIndex++;
        }
        break;
    case CE_TASK_PRIORITY_LL:
        if (ceTask_priorityLLIndex != CE_TASK_PRIORITY_END_BASE)
        {
            ceTask->ceExTaskPar.ceExTaskPriority = ceTask_priorityLLIndex;
            ceTask_priorityLLIndex++;
        }
        break;
    default:
        break;
    }
    if (ceTask->isNewThread == 0x01)//如果模块的驱动需要新建一个任务
    {
        uint8_t os_err;
        //ceDebugOp.printf("CeTask Create Task,ID=%u,ceExTaskPriority=%u.\n", ceTask->ID, ((ceTask->ceExTaskPar).ceExTaskPriority));//调试用，注意，不要忽略注册Debug前创建的任务
        OSTaskCreateExt(
            ceTask_uCOSIITaskPlam,
            ceTask,
            (OS_STK*)(ceTask->taskStackBuf + ceTask->taskStackBufSize - 1),
            ceTask->ceExTaskPar.ceExTaskPriority,
            ceTask->ceExTaskPar.ceExTaskPriority,
            (OS_STK *)(ceTask->taskStackBuf),
            ceTask->taskStackBufSize,
            (void *)0,
            OS_TASK_OPT_STK_CHK | OS_TASK_OPT_STK_CLR);
        OSTaskNameSet(ceTask->ceExTaskPar.ceExTaskPriority, (uint8*)(ceTask->taskName),&os_err);//设定任务的名称
    }
    else
    {
        if (ceTaskList == CE_NULL)
        {
            ceTaskList = ceTask;
        }
        else
        {
            CeTask* ceTaskTemp = ceTaskList;
            while (1)
            {
                if (ceTask->ID == ceTaskTemp->ID)
                {
                    break;
                }
                if (ceTaskTemp->nextCeTask == CE_NULL)
                {
                    ceTaskTemp->nextCeTask = ceTask;
                    break;
                }
                ceTaskTemp = ceTaskTemp->nextCeTask;
            }
        }
    }
#else
    if (ceTaskList == CE_NULL)
    {
        ceTaskList = ceTask;
    }
    else
    {
        CeTask* ceTaskTemp = ceTaskList;
        while (1)
        {
            if (ceTask->ID == ceTaskTemp->ID)
            {
                break;
            }
            if (ceTaskTemp->nextCeTask == CE_NULL)
            {
                ceTaskTemp->nextCeTask = ceTask;
                break;
            }
            ceTaskTemp = ceTaskTemp->nextCeTask;
        }
    }
#endif //CE_USE_RTOS
    return CE_STATUS_SUCCESS;
}

/**
  * @brief   开始一个CeTask
  * @param   ceTask:ceTask属性对象指针
  * @return  系统状态码，可能的值:CE_STATUS_SUCCESS
  */
CE_STATUS ceTask_start(CeTask* ceTask)
{
    if (ceTaskList == CE_NULL)
    {
        return CE_STATUS_PAR_ERROR;
    } else
    {
        CeTask* ceTaskTemp = ceTaskList;
        while (1)
        {
            if (ceTask->ID == ceTaskTemp->ID)
            {
                ceTaskTemp->ceExTaskPar.isRunning = 0x01;
                break;
            }
            ceTaskTemp = ceTaskTemp->nextCeTask;
        }
    }
    return CE_STATUS_SUCCESS;
}

/**
  * @brief   停止一个CeTask
  * @param   ceTask:ceTask属性对象指针
  * @return  系统状态码，可能的值:CE_STATUS_SUCCESS
  */
CE_STATUS ceTask_stop(CeTask* ceTask)
{
    if (ceTaskList == CE_NULL)
    {
        return CE_STATUS_PAR_ERROR;
    } else
    {
        CeTask* ceTaskTemp = ceTaskList;
        while (1)
        {
            if (ceTask->ID == ceTaskTemp->ID)
            {
                ceTaskTemp->ceExTaskPar.isRunning = 0x00;
                break;
            }
            ceTaskTemp = ceTaskTemp->nextCeTask;
        }
    }
    return CE_STATUS_SUCCESS;
}

/**
  * @brief   删除一个CeTask
  * @param   ceTask:ceTask属性对象指针
  * @return  系统状态码，可能的值:CE_STATUS_SUCCESS
  */
CE_STATUS ceTask_unRegister(CeTask* ceTask)
{
    if (ceTaskList == CE_NULL)
    {
        return CE_STATUS_PAR_ERROR;
    } else
    {
        CeTask* ceTaskBefore = CE_NULL;
        CeTask* ceTaskTemp = ceTaskList;
        while (1)
        {
            if (ceTask->ID == ceTaskTemp->ID)
            {
                if (ceTaskBefore == CE_NULL)
                {
                    ceTaskList = CE_NULL;
                } else
                {
                    if (ceTaskTemp->nextCeTask == CE_NULL)
                    {
                        ceTaskBefore->nextCeTask = CE_NULL;
                    } else
                    {
                        ceTaskBefore->nextCeTask = ceTaskTemp->nextCeTask;
                    }
                }
                break;
            }
            ceTaskBefore = ceTaskTemp;
            ceTaskTemp = ceTaskTemp->nextCeTask;
        }
    }
    return CE_STATUS_SUCCESS;
}


/**
  * @brief   进行临界段代码，此后禁止任务切换
  * @param   None
  * @return  状态指示码
  */
void ceTask_inCriticalSection(void)
{
    ceTask_criticalTimes++;
    //非RTOS环境，此函数为空
#ifdef __CE_USE_RTOS__
    //OS_ENTER_CRITICAL();
#endif
}

/**
  * @brief   跳出临界段代码，此后任务切换可以正常进行
  * @param   None
  */
void ceTask_outCriticalSection(void)
{

    if (ceTask_criticalTimes > 0)
    {
        ceTask_criticalTimes--;
    }
    if (ceTask_criticalTimes == 0)
    {
        #ifdef __CE_USE_RTOS__
        //OS_EXIT_CRITICAL();
        #else
        //非RTOS环境，为打开中断
        #endif
    }
}
/**
  * @brief 验证是否处于临界状态
  * @return 返回0x01:正处于临界阶；0x00：未处于临界段码
  */
uint8 ceTask_getCriticalStatus(void) 
{
    if(ceTask_criticalTimes > 0)
        return 0x01;
    else return 0x00;
}


/**
  * @brief   通知操作系统，进行任务调度
  * @param   None
  * @retur   状态指示码
  */
void ceTask_taskSchedule(void)
{
    //非RTOS环境，此函数为空
#ifdef __CE_USE_RTOS__
    ceSystemOp.delayMs(1);
#endif
}



const CeTaskOp ceTaskOp = {ceTask_mainTask,ceTask_register, ceTask_start, ceTask_stop, ceTask_unRegister,ceTask_inCriticalSection,ceTask_outCriticalSection,ceTask_getCriticalStatus,ceTask_taskSchedule};

#ifdef __cplusplus
 }
#endif //__cplusplus
