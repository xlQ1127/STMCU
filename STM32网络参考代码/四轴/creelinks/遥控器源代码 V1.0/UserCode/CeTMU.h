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

#define CE_TMU_WIFI_SSID        "Darcern"              /*!< 配置无人机需要连接WIFI的SSID*/
#define CE_TMU_WIFI_PWD         "Dxwzh178"          /*!< 配置无人机需要连接WIFI的密码*/
#define CE_TMU_WIFI_SERVER_IP   "192.168.1.244"     /*!< 配置无人机需要连接服务器的IP*/
#define CE_TMU_WIFI_SERVER_PORT  2121               /*!< 配置无人机需要连接服务器的端口*/

#define CE_TMU_NRF_FIFO_SIZE    256

/**
  * @brief  枚举，当前无人机通讯方式
  */
typedef enum 
{
    CE_TMU_USE_WIFI,                     /*!< 使用Wifi传输方式*/
    CE_TMU_USE_BLUE,                     /*!< 使用蓝牙传输方式*/
    CE_TMU_USE_NRF,                      /*!< 使用无线射频传输方式*/
}CE_TUM_USE;

/**
  * @brief  枚举，无人机当前的受控方式
  */
typedef enum 
{
    CE_TMU_CTL_TYPE_NONE=0x00,          /*!< 未受控制*/
    CE_TMU_CTL_TYPE_STATION=0x01,       /*!< 受地面站点控制*/
    CE_TMU_CTL_TYPE_CONTROL=0x02,       /*!< 摇控器控制*/
    CE_TMU_CTL_TYPE_PHONE=0x03,         /*!< 受手机控制*/
}CE_TUM_CTL_TYPE;
/*
 *CeTMU属性对像
 */
typedef struct
{
    CeWifiEsp   ceWifiEsp;              /*!< CeWifiEsp模块对象，基于ESP8266-12E*/
    CeBlueHc    ceBlueHc;               /*!< CeBlueHc模块对象，基于HC-05*/
    CeWlsNrf    ceWlsNrf;               /*!< CeWlsNrf模块对象，基于NRF24L01+*/
    CeTask      recvTask;               /*!< 由于nrf24l01使用中断接收数据，回调也是在中断中执行，故这里建立任务，只让nrf接收中断写数据到fifo，然后再在在任务中处理*/
    CeFifo      ceFifo;                 /*!< 接收数据缓存FiFO*/

    uint8       stackBuf[CE_TMU_NRF_FIFO_SIZE];
    uint8       fifoBuf[CE_TMU_NRF_FIFO_SIZE];
    uint8       useBuf[CE_TMU_NRF_FIFO_SIZE];
    uint32      sendPackCount;          /*!< 发送到数据的次数，用于断连检测*/
    uint32      recvPackCount;          /*!< 接收到数据的次数，用于断连检测*/
    void        (*recvCallBack)(uint8* recvBuf, uint16 recvCount);/*!< 接收到数据后，直接TMU对象需调用的回调*/
    void        (*sendCallBack)(void);
    CE_TUM_USE  useType;                /*!< 当前无人机通讯方式*/
}CeTMU;
/*
 *CeTMU操作对像
 */
typedef struct
{
    CE_STATUS (*initialByWifi)(void (*recvCallBack)(uint8* recvBuf, uint16 recvCount)); /*!< 
                                                                     @brief CeTMU模块初始化
                                                                     @param intervalMs:定义发送时间间隔
                                                                     @param recvCallBack:用户需提供的回调函数*/

    CE_STATUS (*initialByNrf)(void (*recvCallBack)(uint8* recvBuf, uint16 recvCount), void (*sendCallBack)(void)); /*!< 
                                                                     @brief CeTMU模块初始化
                                                                     @param intervalMs:定义发送时间间隔
                                                                     @param recvCallBack:用户需提供的回调函数
                                                                     @param sendCallBack:由于NRF24L01的数据接收是在中断中，而为达到收到数据后，立刻回传数据的一应一答功能，中断收到数据并写入收FIFO后，会直接调用sendCallBack，用户在内填写发送相关操作*/
 
    CE_STATUS (*initialByBlue)(void (*recvCallBack)(uint8* recvBuf, uint16 recvCount)); /*!< 
                                                                     @brief CeTMU模块初始化
                                                                     @param intervalMs:定义发送时间间隔
                                                                     @param recvCallBack:用户需提供的回调函数*/

    CE_STATUS (*sendData)(uint8* dataBuf, uint16 dataCount);    /*!< @brief 发送数据，注意：函数内部会检测距离上一次发送数据的时间是否大于intervalMs，如果小于则直接返回
                                                                     @param dataBuf:发送缓存地址
                                                                     @param dataCount:发送缓存数据长度*/

    CE_STATUS (*getConnectStatus)(void);                        /*!< @brief 检测是否通讯中断
                                                                     @return CE_STATUS_SUCCESS：通讯正常； 其它：通讯中断*/

}CeTMUOp;
/*
 *CeTMU操作对象实例
 */
extern const CeTMUOp ceTMUOp;

#ifdef __cplusplus
 }
#endif //__cplusplus
#endif //__CE_TMU_H__


