/**
  ******************************************************************************
  * @file   CeUart.h
  * @author Creelinks Application Team
  * @version V1.0.0
  * @date   2016-08-05
  * @brief  Creelinks平台Uart库头文件
  ******************************************************************************
  * @attention
  *
  *1)如果针对多线程操作，针对同一个Uart资源，尽量不要多个纯种同时访问
  *2)有关每个Uart接收及发送中断的优先级，可在CeUart.c文件中的宏定义中配置
  *
  * <h2><center>&copy; COPYRIGHT 2016 Darcern</center></h2>
  ******************************************************************************
  */
#ifndef __CE_UART_H__
#define __CE_UART_H__

#include "CeMcu.h"
#include "CeExTra.h"

#ifdef __cplusplus
 extern "C" {
#endif //__cplusplus
/**
  * @brief  枚举，Uart对象波特率
  */
typedef enum
{
    CE_UART_BAUD_RATE_2400 = (uint32)2400,            /*!< 波特率2400*/
    CE_UART_BAUD_RATE_4800 = (uint32)4800,            /*!< 波特率4800*/
    CE_UART_BAUD_RATE_9600 = (uint32)9600,            /*!< 波特率9600*/
    CE_UART_BAUD_RATE_19200 = (uint32)19200,          /*!< 波特率19200*/
    CE_UART_BAUD_RATE_38400 = (uint32)38400,          /*!< 波特率38400*/
    CE_UART_BAUD_RATE_43000 = (uint32)43000,          /*!< 波特率4300*/
    CE_UART_BAUD_RATE_56000 = (uint32)56000,          /*!< 波特率56000*/
    CE_UART_BAUD_RATE_57600 = (uint32)57600,          /*!< 波特率576000*/
    CE_UART_BAUD_RATE_115200 = (uint32)115200,        /*!< 波特率115200*/
}CE_UART_BAUD_RATE;

/**
  * @brief  枚举，Uart对象数据位
  */
typedef enum
{
    CE_UART_WORD_LENGTH_8B = 0x01,     /*!< 数据位8位*/
    CE_UART_WORD_LENGTH_9B,            /*!< 数据位9位*/
}CE_UART_WORD_LENGTH;

/**
  * @brief  枚举，Uart对象停止位
  */
typedef enum
{
    CE_UART_STOP_BITS_1 = 0x01,        /*!< 1位停止位*/
    CE_UART_STOP_BITS_0_5,             /*!< 半位停止位*/
    CE_UART_STOP_BITS_2,               /*!< 2位停止位*/
    CE_UART_STOP_BITS_1_5,             /*!< 1位半位停止位*/
}CE_UART_STOP_BITS;

/**
  * @brief  枚举，Uart对象奇偶校验位
  */
typedef enum
{
    CE_UART_PARITY_NO = 0x01,          /*!< 无奇偶校验位*/
    CE_UART_PARITY_EVEN,               /*!< 偶校验位*/
    CE_UART_PARITY_ODD,                /*!< 奇校验位*/
}CE_UART_PARITY;

/**
  * @brief  结构体，Uart对象可用属性集合
  */
typedef struct
{
    CE_RESOURCE             ceResource;                 /*!< Uart对应的资源号*/
    CE_UART_BAUD_RATE       uartBaudRate;               /*!< Uart的波特率*/
    CE_UART_WORD_LENGTH     uartWordLength;             /*!< Uart的数据宽度*/
    CE_UART_STOP_BITS       uartStopBits;               /*!< Uart的停止位*/
    CE_UART_PARITY          uartParity;                 /*!< Uart的奇偶校验位*/

    uint8*                  recvBuf;                    /*!< Uart接收缓存*/
    uint16                  recvBufSize;                /*!< Uart接收缓存的容量*/
    void*                   pAddPar;

    CeFifo                  ceExFifo;                   /*!< Uart接收使用的缓存Fifo*/
    CeExUartPar             ceExUartPar;                /*!< 与处理器平台相关的额外参数结构体，用以提高代码效率，用户无须关注*/
}CeUart;

/**
  * @brief  结构体，Uart对象可用操作集合
  */
typedef struct
{
    CE_STATUS   (*initial)(CeUart* ceUart);             /*!< @brief 初始化Uart
                                                             @param ceUart:ceUart属性对象指针*/

    void        (*start)(CeUart* ceUart);               /*!< @brief 开始Uart
                                                             @param ceUart:ceUart属性对象指针*/

    CE_STATUS   (*sendData)(CeUart* ceUart,uint8* dataBuf, uint16 dataBufSize);/*!<
                                                             @brief 发送数据
                                                             @param ceUart:ceUart属性对象指针
                                                             @param dataBuf:待发送的数据
                                                             @param dataBufSize:待发送的数据长度*/

    uint16      (*getRecvDataCount)(CeUart* ceUart);    /*!< @brief 获得接收缓存中的可用数据
                                                             @param ceUart:ceUart属性对象指针
                                                             @return 返回可读取的数据长度*/

    uint16      (*readData)(CeUart* ceUart, uint8* dataBuf, uint16 readCount);/*!<
                                                             @brief 接收数据
                                                             @param ceUart:ceUart属性对象指针
                                                             @param dataBuf:接收到的数据存放的位置
                                                             @param readCount:要读取的数据个数
                                                             @return 返回实际读取到的数据长度*/

    void        (*stop)(CeUart* ceUart);                /*!< @brief 停止Uart
                                                             @param ceUart:ceUart属性对象指针*/

    void        (*clearRecvBuf)(CeUart* ceUart);        /*!< @brief 清空接收缓存
                                                             @param ceUart:ceUart属性对象指针*/
}CeUartOp;
extern const CeUartOp ceUartOp;                         /*!< 所有与Uart相关的操作*/

#ifdef __cplusplus
 }
#endif //__cplusplus
#endif //__CE_UART_H__

/**
******************************************************************************
* @brief  使用流程及示例程序(基于前后台非操作系统环境)
* @function 使用Uart与上位机通讯，将上位机发送来的数据，再原样发送给上位机
******************************************************************************
#include "Creelinks.h"
#define UART_RECV_BUF_SIZE  1024                    //Uart接收缓存大小
CeUart myUart;                                      //定义Uart属性对象
uint8 recvBuf[UART_RECV_BUF_SIZE];                  //Uart接收缓存数组
int main(void)
{
    ceSystemOp.initial();                           //Creelinks环境初始化
    ceDebugOp.initial(R9Uart);                      //通过Uart串口输出Debug信息到上位机
    //TODO:请在此处插入模块初始化等操作
    myUart.ceResource = R18Uart;                     //指定Uart使用的资源号
    myUart.recvBuf = recvBuf;                       //指定Uart接收缓存
    myUart.recvBufSize = UART_RECV_BUF_SIZE;        //指定Uart接收缓存大小
    myUart.uartBaudRate = CE_UART_BAUD_RATE_115200; //指定Uart的工作速率为115200
    myUart.uartWordLength = CE_UART_WORD_LENGTH_8B; //指定Uart数据长度为8bit
    myUart.uartStopBits = CE_UART_STOP_BITS_1;      //指定Uart的停止位为1
    myUart.uartParity = CE_UART_PARITY_NO;          //指定Uart的奇偶检验位为无
    ceUartOp.initial(&myUart);                      //初始化Uart
    ceUartOp.start(&myUart);                        //开始Uart接收与发送
    while (1)
    {
        ceTaskOp.mainTask();                        //Creelinks环境主循环
        //TODO:请在此处插入用户操作
        while (ceUartOp.getRecvDataCount(&myUart) > 0)//检测Uart是否收到数据
        {
            uint8 recvTemp[128];                    //定义缓存，用于读取接收到的数据
            uint16 tureRecvCount = ceUartOp.readData(&myUart, recvTemp, 128);//读取接收到的数据
            ceUartOp.sendData(&myUart, recvTemp, tureRecvCount);             //将读取到的数据，再通过Uart口发回上位机
        }
        ceSystemOp.delayMs(10);                     //延时10ms
    };
}
******************************************************************************
*/
