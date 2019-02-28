/**
  ******************************************************************************
  * @file    CeDa.h
  * @author  Creelinks Application Team
  * @version V1.0.0
  * @date    2017-03-26
  * @brief   Creelinks平台CeDa库头文件
  ******************************************************************************
  * @attention
  *
  *1)
  *
  * <h2><center>&copy; COPYRIGHT 2016 Darcern</center></h2>
  ******************************************************************************
  */
#ifndef __CE_DA_H__
#define __CE_DA_H__
#include "CeMcu.h"
#ifdef __cplusplus
 extern "C" {
#endif
/**
  * @brief  结构体，DA对象可用属性集合
  */
typedef struct
{
    CE_RESOURCE             ceResource;                             /*!< Da对应的资源号*/
    uint32                  convertIntervalNs;                      /*!< 连续两个需要Da转换的数据相隔的时间，单位ns*/
    void*                   pAddPar;                                /*!< 空指针，可用于传递额外对数*/
    void                    (*callBackConvertFinish)(void* pAddPar);/*!< 完成一次DA转换后，需要执行的函数*/

    CeExDaPar               ceExDaPar;                              /*!< 与处理器平台相关的额外参数结构体，用以提高代码效率，用户无须关注*/
}CeDa;

/**
  * @brief  结构体，DA对象可用操作集合
  */
typedef struct
{
    CE_STATUS   (*initial)(CeDa* ceDa);                             /*!< @brief 初始化一个Da
                                                                         @param ceDa:Da属性对象指针*/

    void        (*start)(CeDa* ceDa, const uint16* dataBuf, uint32 dataBufSize);/*!<
                                                                         @brief 开始DA转换
                                                                         @param ceDa:Da属性对象指针
                                                                         @param dataBuf:需要转换的数据缓冲区
                                                                         @param dataBufSize:需要转换的数据个数，应小于数据缓冲区的有效个数*/

    void        (*startFixedVoltage)(CeDa* ceDa, uint16 Val);       /*!< @brief Da输出固定值
                                                                         @param ceDa:Da属性对象指针*/

    void        (*stop)(CeDa* ceDa);                                /*!< @brief 停止Da转换
                                                                         @param ceDa:Da属性对象指针*/

    void        (*updata)(CeDa* ceDa);                              /*!< @brief 更新Da参数
                                                                         @param ceDa:Da属性对象*/


}CeDaOp;
extern const CeDaOp ceDaOp;                                     /*!< 所有与Da相关的操作*/


#ifdef __cplusplus
 }
#endif //__cplusplus
#endif //__CE_DA_H__

/**
******************************************************************************
* @brief   使用流程及示例程序(基于前后台非操作系统环境)
* @function 通过Da输出100Hz的三角波
******************************************************************************
#include "Creelinks.h"

#define DA_BUF_SIZE     1000                    //Da待转换的数组长度
CeDa myDa;                                      //Da属性对象指针
uint16 outVal[DA_BUF_SIZE];                     //Da待转换的数组
void daConvertFinishCallBack(void* pAddPar)
{
    ceDaOp.start((CeDa*)(pAddPar), outVal, DA_BUF_SIZE);   //完成一次转换后，再进行一次转换以达到连续波长的输出
}

int main(void)
{
    int i;
    ceSystemOp.initial();                       //Creelinks环境初始化
    ceDebugOp.initial(R23Uart);                 //通过Uart串口输出Debug信息到上位机
    //TODO:请在此处插入模块初始化等操作
    for (i = 0; i < DA_BUF_SIZE; i++)
    {
        outVal[i] = i;//数组初始化
    }
    myDa.ceResource = R8ADG;                    //指定资源号
    myDa.convertIntervalNs = 10000;             //设定每两个点之间的转换间隔，10000ns*1000个点＝10ms，即输出三角波频率为1s/10ms = 100Hz
    myDa.pAddPar = &myDa;                       //设置属性中空指针指向自己，方便区分多个不同的Da对象
    myDa.callBackConvertFinish = daConvertFinishCallBack;//指定转换完成的回调函数
    ceDaOp.initial(&myDa);                      //初始化Da
    ceDaOp.start(&myDa, outVal, DA_BUF_SIZE);   //开始转换
    while (1)
    {
        ceTaskOp.mainTask();                    //Creelinks环境主循环任务，请保证此函数能够被周期调用
        //TODO:请在此处插入用户操作
    };
}
******************************************************************************
*/


