/**
  ******************************************************************************
  * @file    CeTMU.h
  * @author  Creelinks Application Team
  * @version V1.0.0
  * @date    2017-03-26
  * @brief   数据传输管理，用于WIFI、蓝牙、2.4G模块的初始化；数据发送接收等处理
  ******************************************************************************
  * @attention
  *
  *1)移植请注意：请在initial函数中，定义各个模块使用到的资源。
  *2)发送数据调用send函数，输入Byte数组即可；
  *3)接收到数据后自动调用初始化时提供的回调，传入未经任何处理的Byte数组。
  *4)接收到数据后，调用的回调函数，在ceTaskOp.mainTask()中执行，请保证主main函数中的ceTaskOp.mainTask()能够被周期调用 
  * 
  * <h2><center>&copy; COPYRIGHT 2016 Darcern</center></h2>
  ******************************************************************************
  */
#ifndef __CE_TMU_H__
#define __CE_TMU_H__
#ifdef __cplusplus
 extern "C" {
#endif //__cplusplus
#include "Creelinks.h"

#include "CeWifiEsp.h"
#include "CeBlueHc.h"
#include "CeWlsNrf.h"

#define CE_TMU_WIFI_SSID        "Darcern"           /*!< 配置无人机需要连接WIFI的SSID*/
#define CE_TMU_WIFI_PWD         "Dxwzh178"          /*!< 配置无人机需要连接WIFI的密码*/
#define CE_TMU_WIFI_SERVER_IP   "192.168.1.211"     /*!< 配置无人机需要连接服务器的IP*/
#define CE_TMU_WIFI_SERVER_PORT  2121               /*!< 配置无人机需要连接服务器的端口*/



/**
  * @brief  枚举，当前无人机通讯方式
  */
typedef enum 
{
    CE_TMU_USE_WIFI,                     /*!< 使用Wifi传输方式*/
    CE_TMU_USE_BLUE,                     /*!< 使用蓝牙传输方式*/
    CE_TMU_USE_NRF,                      /*!< 使用无线射频传输方式*/
}CE_TUM_USE;

/*
 *CeTMU属性对像
 */
typedef struct
{
    CeWifiEsp   ceWifiEsp;              /*!< CeWifiEsp模块对象，基于ESP8266-12E*/
    CeBlueHc    ceBlueHc;               /*!< CeBlueHc模块对象，基于HC-05*/
    CeWlsNrf    ceWlsNrf;               /*!< CeWlsNrf模块对象，基于NRF24L01+*/
    uint32      sendPackCount;          /*!< 发送到数据的次数，用于断连检测*/
    uint32      recvPackCount;          /*!< 接收到数据的次数，用于断连检测*/
    void        (*recvCallBack)(uint8* recvBuf, uint16 recvCount);/*!< 接收到数据后，直接TMU对象需调用的回调*/
    uint32      lastSendTime;           /*!< 用于计算两次发送时间间隔是否大于sendIntervalMs*/
    CE_TUM_USE  useType;                /*!< 当前无人机通讯方式*/
}CeTMU;
/*
 *CeTMU操作对像
 */
typedef struct
{
    CE_STATUS (*initial)(CE_TUM_USE useType, void (*recvCallBack)(uint8* recvBuf, uint16 recvCount)); /*!< 
                                                                     @brief CeTMU模块初始化
                                                                     @param useType:配置TMU模块使用什么通讯方式进行数据传输
                                                                     @param recvCallBack:用户需提供的回调函数*/

    CE_STATUS (*sendData)(uint8* dataBuf, uint16 dataCount);    /*!< @brief 发送数据，注意：函数内部会检测距离上一次发送数据的时间是否大于intervalMs，如果小于则直接返回
                                                                     @param dataBuf:发送缓存地址
                                                                     @param dataCount:发送缓存数据长度*/

    CE_STATUS (*checkConnectStatus)(void);                      /*!< @brief 检测是否通讯中断
                                                                     @return CE_STATUS_SUCCESS：通讯正常； 其它：通讯中断*/

    uint32    (*getSendIntervalMs)(void);                       /*!< @brief 检测距离上一次发送数据到此时的时间间隔
                                                                     @return 距离上一次发送数据到现在的时间间隔，单位ms*/
}CeTMUOp;
/*
 *CeTMU操作对象实例
 */
extern const CeTMUOp ceTMUOp;

#ifdef __cplusplus
 }
#endif //__cplusplus
#endif //__CE_TMU_H__


