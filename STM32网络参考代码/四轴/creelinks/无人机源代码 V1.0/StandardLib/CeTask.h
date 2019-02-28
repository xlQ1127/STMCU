/**
  ******************************************************************************
  * @file    CeTask.h
  * @author  Creelinks Application Team
  * @version V1.0.0
  * @date    2016-08-05
  * @brief   Creelinks平台CeTask库头文件
  ******************************************************************************
  * @attention
  *
  *1)针对前后无操作系统软件（裸奔）平台，所有注册过的任务，均在主函数中的mainTask函数内，按注册顺序周期调用，
  *  故用户必须保证mainTask函数不被阻塞，并且注册的任务函数（callBack）也必须是可短时间内可执行完并返回的。
  *2)针对任意操作系统环境（RTOS），可以根据isNewThread来指定此任务是独立任务，还是在后台中调用的任务。
  *3)如果是新任务(isNewThread = 0x01)，则会使用RTOS的创建任务函数进行任务创建，为了保证其它任务能够正常运行，
  *  请在注册的函数内适当添加延时操作。
  *4)如果是后台中调用的任务，Creelinks平台将注册的函数与其它同样是后台任务的注册函数，统一在一个单一的由系统
  *  创建的任务中轮循调用，故注册的函数中尽量不要有延时操作。
  *
  * <h2><center>&copy; COPYRIGHT 2016 Darcern</center></h2>
  ******************************************************************************
  */
#ifndef __CE_LOOP_H__
#define __CE_LOOP_H__

#include "CeMcu.h"

#ifdef __cplusplus
 extern "C" {
#endif //__cplusplus

#ifdef __CE_USE_RTOS__
/**
  * @brief  RTOS模式下，任务的优先级，这里暂定5个优先级
  */
 typedef enum
 {
     CE_TASK_PRIORITY_HH = 0,
     CE_TASK_PRIORITY_H,
     CE_TASK_PRIORITY_M,
     CE_TASK_PRIORITY_L,
     CE_TASK_PRIORITY_LL,
 }CE_TASK_PRIORITY;
#endif //__CE_USE_RTOS__

/**
  * @brief  结构体，Task对象可用操作集合
  */
typedef struct CeTaskBase
{
    uint32      ID;                                 /*!< 当前线程的ID号*/
    void*       pAddPar;                            /*!< 额外的指针，执行回调时传入此指针*/
    void        (*callBack)(void* pAddPar);         /*!< 线程的主循环函数*/

    #ifdef __CE_USE_RTOS__
    CE_TASK_PRIORITY taskPriority;                  /*!< 如果此任务在一个新线程中建立，则需要指定任务的优先级*/
    CE_STK*     taskStackBuf;                       /*!< 指定任务的堆栈首地址*/
    uint32      taskStackBufSize;                   /*!< 指定任务堆栈的大小*/
    char*       taskName;                           /*!< 指定任务的名称*/
    #endif

    struct  CeTaskBase* nextCeTask;                 /*!< 链表，用于保存下一个线程*/
    CeExTaskPar    ceExTaskPar;
}CeTask;

/**
 * @brief  结构体，Task对象可用操作集合
 */
typedef struct
{

    CE_STATUS   (*mainTask)();                      /*!< @brief 在main入口函数的主while循环中调用的任务循环处理函数*/

    CE_STATUS   (*registerTask)(CeTask* ceTask);    /*!< @brief 注册一个单独运行的线程
                                                         @param ceTask:线程对象的指针*/

    CE_STATUS   (*start)(CeTask* ceTask);           /*!< @brief 开始一个注册后的线程
                                                         @param ceTask:线程对象的指针*/

    CE_STATUS   (*stop)(CeTask* ceTask);            /*!< @brief 停止一个注册后的线程
                                                         @param ceTask:线程对象的指针*/

    CE_STATUS   (*unRegister)(CeTask* ceTask);      /*!< @brief 取消线程的注册
                                                         @param ceTask:线程对象的指针*/
  
    void        (*inCriticalSection)(void);         /*!<@brief 进行临界段代码，此后禁止任务切换*/

    void        (*outCriticalSection)(void);        /*!<@brief 跳出临界段代码，此后任务切换可以正常进行*/

    uint8       (*getCriticalStatus)(void);         /*!<@brief 验证是否处于临界状态
                                                        @return 返回0x01:正处于临界阶；0x00：未处于临界段*/

    void        (*taskSchedule)(void);              /*!< @brief 通知操作系统，进行任务调度*/
} CeTaskOp;
extern const CeTaskOp ceTaskOp;                     /*!< 所有与Task相关的操作*/

#ifdef __cplusplus
 }
#endif //__cplusplus
#endif //__CE_LOOP_H__


/**
******************************************************************************
* @brief  使用流程及示例程序(基于前后台非操作系统环境，即裸奔环境)
* @function 开始一个任务，每次执行时通过Uart向上位机发送调试信息
******************************************************************************
#include "Creelinks.h"
CeTask myTask;                                  //定义Task属性对象
void myTaskCallBack(void * pAddPar)             //任务主函数，注意：在裸奔系统中，此函数必须能够短时间执行完毕，且一定不能为while(1)死循环
{
    ceDebugOp.printf("Task ID %d is running!\n", ((CeTask*)(pAddPar))->ID);
}
int main(void)
{
    ceSystemOp.initial();                       //Creelinks环境初始化
    ceDebugOp.initial(Uartx);                        //通过Uart串口输出Debug信息到上位机
    //TODO:请在此处插入模块初始化等操作             
    myTask.ID = 0x01;                           //指定任务ID   
    myTask.pAddPar = &myTask;                   //指定Task属性中的空指针，任务函数调用时传入回调函数
    myTask.callBack = myTaskCallBack;           //指定任务的回调函数
    ceTaskOp.registerTask(&myTask);             //注册此任务
    ceTaskOp.start(&myTask);                    //开始此任务
    while (1)
    {
        ceTaskOp.mainTask();                    //Creelinks环境主循环任务，请保证此函数能够被周期调用
        //TODO:请在此处插入用户操作
        ceSystemOp.delayMs(200);                //延时函数
    };
}
******************************************************************************
*/

/**
******************************************************************************
* @brief  使用流程及示例程序(基于实时操作系统(RTOS)环境)
* @function 新创建两个独立的任务，并向上位机打印任务信息
****************************************************************************** 
#include "Creelinks.h"
#define MY_TASK1_STACK_BUF_SIZE  1024                   //任务1堆栈缓存大小
CE_STK MY_TASK1_STACK_BUF[MY_TASK1_STACK_BUF_SIZE];     //任务1使用堆栈缓存
CeTask myTask1;                                         //定义任务1的属性对象实例
void myTask1CallBack(void* pAddPar)                     //示例任务1函数，注意：因Creelinks环境将任务创建和调用进行简化处理，故任务函数可以不是while(1)循环，也能保证周期调用
{
    ceDebugOp.printf("Task 1 is running...\n");
    ceSystemOp.delayMs(1000);                           //适当延时，让处理器处理其它任务
}

#define MY_TASK2_STACK_BUF_SIZE  1024                   //任务2堆栈缓存大小
CE_STK MY_TASK2_STACK_BUF[MY_TASK2_STACK_BUF_SIZE];     //任务2使用堆栈缓存
CeTask myTask2;                                         //定义任务2的属性对象实例
void myTask2CallBack(void* pAddPar)                     //示例任务2函数，注意：因Creelinks环境将任务创建和调用进行简化处理，故任务函数可以不是while(1)循环，也能保证周期调用
{
    ceDebugOp.printf("Task 2 is running...\n");
    ceSystemOp.delayMs(3000);                           //适当延时，让处理器处理其它任务     
}

int main(void)
{
    ceSystemOp.initial();                               //Creelinks环境初始化
    ceDebugOp.initial(R9Uart);                          //通过Uart串口输出Debug信息到上位机
    //TODO:请在此处插入模块初始化、任务创建等操作

    myTask1.ID = 0x00001;                               //指定任务1的ID号
    myTask1.taskName = "My Task1";                      //设定任务1的名称，注册长度不要超过CeMcu.h中定义的CE_TASK_NAME_LENGTH值
    myTask1.callBack = myTask1CallBack;                 //指定具体任务1，一个函数即为一个任务，系统会周期调用任务函数
    myTask1.pAddPar = &myTask1;                         //指定给任务1函数传递的pAddPar参数
    myTask1.isNewThread = 0x01;                         //0x01:此任务1为一个独立于其它任务的新任务；0x00:所有此参数为0x00的注册任务，均由一个系统后台M级任务轮循调用
    myTask1.taskPriority = CE_TASK_PRIORITY_M;          //基于Creelinks平台的任务1优先级，共5个，分为为HH、H、M、L、LL，同等优先级的任务，在RTOS环境下，先注册的任务高于后注册的任务
    myTask1.taskStackBuf = MY_TASK1_STACK_BUF;          //任务使用堆栈缓存
    myTask1.taskStackBufSize = MY_TASK1_STACK_BUF_SIZE; //任务堆栈缓存大小
    ceTaskOp.registerTask(&myTask1);                    //注册此任务1
    ceTaskOp.start(&myTask1);                           //开始运行

    myTask2.ID = 0x00002;                               //指定任务2的ID号
    myTask2.taskName = "My Task2";                      //设定任务2的名称，注册长度不要超过CeMcu.h中定义的CE_TASK_NAME_LENGTH值
    myTask2.callBack = myTask2CallBack;                 //指定具体任务2，一个函数即为一个任务，系统会周期调用任务函数
    myTask2.pAddPar = &myTask2;                         //指定给任务函数传递的pAddPar参数
    myTask2.isNewThread = 0x01;                         //0x01:此任务2为一个独立于其它任务的新任务；0x00:所有此参数为0x00的注册任务，均由一个系统后台M级任务轮循调用
    myTask2.taskPriority = CE_TASK_PRIORITY_HH;         //基于Creelinks平台的任务2优先级，共5个，分为为HH、H、M、L、LL，同等优先级的任务，在RTOS环境下，先注册的任务高于后注册的任务
    myTask2.taskStackBuf = MY_TASK2_STACK_BUF;          //任务2使用堆栈缓存
    myTask2.taskStackBufSize = MY_TASK2_STACK_BUF_SIZE; //任务2堆栈缓存大小
    ceTaskOp.registerTask(&myTask2);                    //注册此任务2
    ceTaskOp.start(&myTask2);                           //开始运行

    ceTaskOp.mainTask();                                //在RTOS操作系统（uCOS II）环境下，执行到此函数后，将不再向下执行。
    return 0;
}
******************************************************************************
*/
