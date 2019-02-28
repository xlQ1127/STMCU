/**
  ******************************************************************************
  * @file    CeCcp.h
  * @author  Creelinks Application Team
  * @version V1.0.0
  * @date    2017-03-26
  * @brief   Creelinks平台外部脉冲计数器CeCcp（脉冲计数器）库头文件
  ******************************************************************************
  * @attention
  *
  *1)
  *
  * <h2><center>&copy; COPYRIGHT 2016 Darcern</center></h2>
  ******************************************************************************
  */
#ifndef __CE_CCP_H__
#define __CE_CCP_H__

#include "CeMcu.h"

#ifdef __cplusplus
 extern "C" {
#endif //__cplusplus

/**
  * @brief  结构体，CCP对象可用属性集合
  */
typedef struct
{
    CE_RESOURCE ceResource;                             /*!< Ccp对应的资源号*/
    uint32      ceCntVal;                               /*!< 用户设定的Ccp计数临界值*/
    void*       pAddPar;                                /*!< 空指针，可用于传递额外参数*/
    void        (*callBackReachCntVal)(void* pAddPar);  /*!< Ccp计数到达用户设置的临界值后，需要执行的回调函数*/

    CeExCcpPar  ceExCcpPar;                             /*!< 与处理器平台相关的额外参数结构体，用以提高代码效率，用户无须关注*/
}CeCcp;

/**
  * @brief  结构体，CCP对象可用操作集合
  */
typedef struct
{
    CE_STATUS   (*initial)(CeCcp* ceCcp);               /*!< @brief 初始化Ccp计数器
                                                             @param ceCcp:ceCcp属性对象指针*/

    void        (*start)(CeCcp* ceCcp);                 /*!< @brief 开始Ccp计数
                                                             @param ceCcp:ceCcp属性对象指针*/

    void        (*stop)(CeCcp* ceCcp);                  /*!< @brief 停止Ccp计数
                                                             @param ceCcp:ceCcp属性对象指针*/

    uint32      (*getNowCcpCnt)(CeCcp* ceCcp);          /*!< @brief 获得当前Ccp计数的值，此值一定小于等于ceMaxCnt
                                                             @param ceCcp:ceCcp属性对象指针
                                                             @return 获取本次计数周期的计数值*/

    uint32      (*getAllCcpCnt)(CeCcp* ceCcp);          /*!< @brief 获得从开始计数起(调用startCcp时开始)，到现在一共的计数值
                                                             @param ceCcp:ceCcp属性对象指针
                                                             @return 获取从开始计数到目前为止总的计数值*/

    void        (*clearCcpCnt)(CeCcp* ceCcp);           /*!< @brief 清除计数，从0开始重新计数
                                                             @param ceCcp:ceCcp属性对象指针*/
}CeCcpOp;
extern const CeCcpOp ceCcpOp;                       /*!< 所有与Ccp相关的操作*/

#ifdef __cplusplus
 }
#endif //__cplusplus
#endif //__CE_CCP_H__
/**
******************************************************************************
* @brief  使用流程及示例程序(基于前后台非操作系统环境)
* @function 设定Ccp的最大计数值为10，并每500ms读取当前值并通过Uart输出到上位机
******************************************************************************
#include "Creelinks.h"
CeCcp myCcp;                                       //Ccp属性对象
uint8 ccpCount;                                    //当前Ccp的计数值

// @brief  Ccp计数器达到设置值后的事件回调
// @param  pAddPar:Ccp对像指针里的pAddPar参数
void ceCcpReachCntCallBack(void* pAddPar)
{
    ceDebugOp.printf("Ccp is reach setting count!\n");
}

int main(void)
{
    ceSystemOp.initial();                          //Creelinks环境初始化
    ceDebugOp.initial(Uartx);                     //通过Uart串口输出Debug信息到上位机
    //TODO:请在此处插入模块初始化等操作
    myCcp.ceResource = RxC;                        //指定计数器资源号
    myCcp.ceCntVal = 10;                           //指定计数到何值后，进入中断回调
    myCcp.callBackReachCntVal = ceCcpReachCntCallBack;//指定回调函数
    myCcp.pAddPar = &myCcp;                        //指定属性中空指针为本身，中断回调中传入此指针
    ceCcpOp.initial(&myCcp);                       //初始化计数器
    ceCcpOp.start(&myCcp);                         //开始计数
    while (1)
    {
        ceTaskOp.mainTask();                       //Creelinks环境主循环任务，请保证此函数能够被周期调用
        //TODO:请在此处插入用户操作
        ccpCount = ceCcpOp.getNowCcpCnt(&myCcp);   //获得当前计数值
        ceDebugOp.printf("Ccp count is %d\n", ccpCount);//打印当前计数值
        ceSystemOp.delayMs(500);                   //延时500ms
    };
}
******************************************************************************
*/



