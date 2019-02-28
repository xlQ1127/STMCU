/**
  ******************************************************************************
  * @file    CeWlsNrf.h
  * @author  Creelinks Application Team
  * @version V1.0.0
  * @date    2017-01-06
  * @brief   适用于CeWlsNrf模块的驱动头文件
  ******************************************************************************
  * @attention
  *
  *1)无
  *
  * <h2><center>&copy; COPYRIGHT 2016 Darcern</center></h2>
  ******************************************************************************
  */
#ifndef __CE_WLS_NRF_H__
#define __CE_WLS_NRF_H__
#ifdef __cplusplus
 extern "C" {
#endif //__cplusplus
#include "Creelinks.h"
#define __CE_WLS_NRF_VERSION__ 1                                             /*!< 此驱动文件的版本号*/
#define __CE_WLS_NRF_NEED_CREELINKS_VERSION__ 1                              /*!< 需要Creelinks平台库的最低版本*/
#if (__CE_CREELINKS_VERSION__ < __CE_WLS_NRF_NEED_CREELINKS_VERSION__)       /*!< 检查Creelinks平台库的版本是否满足要求*/
#error "驱动文件CeWlsNrf.h需要高于1.0以上版本的Creelink库，请登陆www.creelinks.com下载最新版本的Creelinks库。"
#else

#define CE_WLS_NRF_RECV_BUF_SIZE    128             /*!< 接收缓存的数组长度*/

#define CE_WLS_NRF_PACKET_LENGTH    32              /*!< 设置接收与发送单个Pack包的长度*/

typedef struct
{
    CeSpiMaster ceSpiMaster;                        /*!< 模块使用的Spi资源*/
    CeGpio      ceGpio;                             /*!< 模块使用的Gpio资源*/
    CeInt       ceInt;                              /*!< 模块使用的Int资源*/
    void        (*callBackRecv[6])(uint8* dataBuf, uint16 dataBufSize);/*!< 接收通道*/
    uint8       recvAddress[6][5];                  /*!< 接收地址缓存*/
    uint8       recvBuf[CE_WLS_NRF_RECV_BUF_SIZE];  /*!< 接收缓存*/
    uint16      recvBufSize;                        /*!< 接收缓冲区中有效数据的字节数*/
    uint8       status;                             /*!< 工作状态，是处于发送状态还是接收状态*/
}CeWlsNrf;
/*
*CeWlsNrf操作对像
*/
typedef struct
{
    CE_STATUS   (*initial)(CeWlsNrf* ceWlsNrf, CE_RESOURCE ceSpi, CE_RESOURCE ceGpio,CE_RESOURCE ceInt);/*!<
                                                         @brief CeWlsNrf模块初始化
                                                         @param ceWlsNrf:CeWlsNrf属性对象指针
                                                         @param ceSpi:模块使用的Spi资源号
                                                         @param ceGpio:模块使用的ceGpio资源号
                                                         @param ceInt:模块使用的ceInt资源号
                                                         @param 此模块的有效地址，共5个字节，一个模块对应一个地址*/

    CE_STATUS   (*send)(CeWlsNrf* ceWlsNrf, uint8* sendAddress, uint8* dataBuf, uint16 dataBufSize);/*!<
                                                         @brief 进入发送模式，发送操作完成后，函数才返回
                                                         @param ceWlsNrf:CeWlsNrf属性对象指针
                                                         @param sendAddress:发送地址，与接收端6个接收通道中的一个相同
                                                         @param dataBuf:要发送的数据缓存区
                                                         @param dataBufSize:要发送的数据长度，注意：一定要为CE_WLS_NRF_PACKET_LENGTH的整数倍
                                                         @return 返回CE_STATUS_SUCCESS则表明发送成功，其它则表明发送失败*/

    CE_STATUS   (*recv)(CeWlsNrf* ceWlsNrf, uint8 pipeIndex, uint8* recvAddress, void(callBackRecv)(uint8* dataBuf, uint16 dataBufSize));/*!<
                                                         @brief 进入接收状态，开始接收数据。注意：当调用send函数后，一定要再次调用此函数才能实现数据接收，异步执行，函数直接返回。
                                                         @param ceWlsNrf:CeWlsNrf属性对象指针
                                                         @param pipeIndex:模块共有6个可用接收通道，此值指定使用哪个接收通道接收数据，如果要使用多个接收通道接收数据，则可重复调用此函数多次
                                                         @param recvAddress:接收通道对应的接收地址，接收地址有详细规则，详细阅读模块手册
                                                         @param callBackRecv:当对应通道接收到数据后，调用的回调函数，每个通道的回调函数均独立
                                                         @return 返回CE_STATUS_SUCCESS则表明配置成功，其它则表明配置失败*/
}CeWlsNrfOpBase;
/*
*CeWlsNrf操作对象实例
*/
extern const CeWlsNrfOpBase ceWlsNrfOp;

#endif // (__CE_CREELINKS_VERSION__ < __CE_WLS_NRF_NEED_CREELINKS_VERSION__)
#ifdef __cplusplus
 }
#endif //__cplusplus
#endif //__CE_WLS_NRF_H__

/**
******************************************************************************
* @brief  使用流程及示例程序(基于前后台非操作系统环境) 
* @function 使用CeWlsNrf模块建立发送或接收端，通过宏定义来选择。
          发送方1S发送一次数据，并将发送的数据及发送的状态通过串口打印
          接收方将接收到的数据和接收的次数通过串口打印
******************************************************************************
#include "Creelinks.h"
#include "CeWlsNrf.h"

CeWlsNrf myWlsNrf;

CE_STATUS ceStatus = CE_STATUS_SUCCESS;               //操作的姿态
uint8 datBuffer[CE_WLS_NRF_PACKET_LENGTH] = { 0 };    //数据缓冲区
#define WLS_NRF_SEND                                  //选择是发送还是接收，如果是接收请屏蔽此宏！

#ifdef WLS_NRF_SEND
uint8 sendAddress[5] = {0x34, 0xC3, 0x10, 0xc1, 0x00};//设置发送地址
uint32 sendCount     = 0;                             //发送计数
uint32 sendOkCount   = 0;                             //发送成功计数
#else
uint8 recvAddress[5] = {0x34, 0xC3, 0x10, 0xc1, 0x00};//设置接收地址，接收方只要知道自己的地址即可
uint32 recvCount     = 0;                             //接收计数
#endif

#ifndef WLS_NRF_SEND
void callBackNrfRecv(uint8* dataBuf, uint16 dataBufSize)
{
    ceDebugOp.printf((char*)dataBuf);          //通过串口打印数据
    recvCount++;
    ceDebugOp.printf( "Recvount=%u.", recvCount);//打印接收计数
}
#endif

int main(void)
{
    ceSystemOp.initial();                       //Creelinks环境初始化
    ceDebugOp.initial(R9Uart);                  //通过Uart串口输出Debug信息到上位机
    //TODO:请在此处插入模块初始化等操作

    while(ceWlsNrfOp.initial(&myWlsNrf, R7Spi, R2TI2c) != CE_STATUS_SUCCESS)//使用R7SPI和R2TI2c初始化CeWlsNrf模块，并等待与模块正常连接初始化成功！
    {
        ceDebugOp.printf("CeWlsNrf initial return %s!\n", ceSystemOp.getErrorMsg(ceStatus));
        ceSystemOp.delayMs(100);
    };

#ifndef WLS_NRF_SEND
    ceStatus = ceWlsNrfOp.recv(&myWlsNrf, 0, recvAddress, callBackNrfRecv);//接收模式下，注册接收回调函数，等待
    if(ceStatus != CE_STATUS_SUCCESS)
    {
        ceDebugOp.printf("CeWlsNrf recv return %s\n", ceSystemOp.getErrorMsg(ceStatus));
    }
#endif

    while (1)
    {
        ceTaskOp.mainTask();                    //Creelinks环境主循环任务，请保证此函数能够被周期调用
        //TODO:请在此处插入用户操作

#ifdef WLS_NRF_SEND
        ceDebugOp.sprintf((char*)datBuffer, "SendCount=%u.", sendCount);//准备发送数据，数据的长度不能超过 CE_WLS_NRF_PACKET_LENGTH
        ceStatus = ceWlsNrfOp.send(&myWlsNrf, sendAddress, datBuffer, CE_WLS_NRF_PACKET_LENGTH);//发送数据并获取发送状态
        if (ceStatus == CE_STATUS_SUCCESS)
            sendOkCount++;                      //记录发送成功的次数
        sendCount ++;                           //发送计数增加
        ceDebugOp.printf("SendData: %s SendtCount: %u SendOkCount: %u Status: %s.\n", datBuffer, sendCount, sendOkCount, ceSystemOp.getErrorMsg(ceStatus));//打印发送的数据和发送的状态
#endif
        ceSystemOp.delayMs(1000);
    };
}
******************************************************************************
*/

