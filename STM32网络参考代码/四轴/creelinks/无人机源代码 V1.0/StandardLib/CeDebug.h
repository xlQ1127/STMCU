/**
  ******************************************************************************
  * @file    CeDeBug.h
  * @author  Creelinks Application Team
  * @version V1.0.0
  * @date    2017-01-06
  * @brief   适用于CeDeBug模块的驱动头文件
  ******************************************************************************
  * @attention
  *
  *1)无
  *
  * <h2><center>&copy; COPYRIGHT 2016 Darcern</center></h2>
  ******************************************************************************
  */
#ifndef __CE_DEBUG_H__
#define __CE_DEBUG_H__

#ifdef __cplusplus
 extern "C" {
#endif //__cplusplus
#include "Creelinks.h"

#define CE_DEBUG_RECV_BUF_SIZE      32                  /*!< 串口调试使用的Uart接收缓存大小，如果要从上位机接收大量数据，请修改此值*/
#define CE_DEBUG_APPEND_BUF_SIZE    256                 /*!< AppendString 使用的缓存*/
/*
 *CeDeBug属性对像
 */
typedef struct
{
    CeUart ceUart;                                      /*!< 串口调试使用的Uard对象*/
    uint8 recvBuf[CE_DEBUG_RECV_BUF_SIZE];              /*!< 串口使用到的接收缓存*/
    void(*appendString)(const char* msg);               /*!< 调试时，需要追加CMD信息的回调*/
    char appendStringBuf[CE_DEBUG_APPEND_BUF_SIZE];     /*!< 用于追加CMD信息时，sprintf使用的缓存*/
    uint16 appendStringBufIndex;                        /*!< */
    uint8 isPrintfFinish ;                              /*!< */
}CeDebug;
/*
 *CeDeBug操作对像
 */
typedef struct
{

    CE_STATUS   (*initial)(CE_RESOURCE ceUart);/*!<
                                             @brief 若使用到UART进行代码调式，需在initial后执行此函数。ceUart:使用哪个UART资源做为调式口
                                             @param ceUart:打印调试信息使用的Uart资源号*/

    void        (*registerAppendString)(void (appendString)(const char* msg));/*!<
                                             @brief 当用户使用额外的显示设备时，可以将显示设备的appendString使用此函数注册，注册完成后可达到显示设备显示调试信息的作用
                                             @param appendString:需要注册的appendString函数*/

    void        (*unRegisterAppendString)(void);/*!<
                                             @brief 取消在额外显示设备上显示调试信息函数*/


    void        (*printf)(const char* msg, ...);/*!<
                                             @brief 通过UART口打印调式信息到外设，需在调用initialDebug后，才可以正常使用
                                             @param msg:需打印的信息*/

    int         (*sprintf)(char *buffer, const char *msg, ...);/*!<
                                             @brief 通过UART口打印调式信息到内存
                                             @param buffer:内存缓存
                                             @param msg:需打印的信息*/


    uint8       (*getRecvDataCount)(void);/*!<
                                             @brief 使用Uart进行调试时，获得上位机发送来的可用调试数据数量
                                             @return 可用的数据数量 */

    uint8       (*getRecvData)(uint8* dataOutBuf, uint8 readCount);/*!<
                                             @brief 使用Uart进行调试时，获得上位机发送来的调试数据
                                             @param dataOutBuf:保存读取数据的缓存
                                             @param readCount:需要读取的数据长度
                                             @return 实际读取到的数据长度*/

}CeDebugOp;
/*
 *CeDeBug操作对象实例
 */
extern const CeDebugOp ceDebugOp;

#ifdef __cplusplus
 }
#endif //__cplusplus
#endif //__CE_DEBUG_H__

/**
******************************************************************************
* @brief  使用流程及示例程序(基于前后台非操作系统环境) 
* @function xxxxxzzzz
******************************************************************************
#include "Creelinks.h"
int main(void)
{
    ceSystemOp.initial();                       //Creelinks环境初始化
    ceDebugOp.initial(Uartx);                        //通过Uart串口输出Debug信息到上位机
    //TODO:请在此处插入模块初始化等操作

    while (1)
    {
        ceTaskOp.mainTask();                    //Creelinks环境主循环任务，请保证此函数能够被周期调用
        //TODO:请在此处插入用户操作

    };
}
******************************************************************************
*/
