/**
  ******************************************************************************
  * @file    CeSpi.h
  * @author  Creelinks Application Team
  * @version V1.0.0
  * @date    2016-08-05
  * @brief   Creelinks平台CeSpi头文件
  ******************************************************************************
  * @attention
  *
  *1)
  *
  * <h2><center>&copy; COPYRIGHT 2016 Darcern</center></h2>
  ******************************************************************************
  */
#ifndef __CE_SPI_H__
#define __CE_SPI_H__

#include "CeMcu.h"

#ifdef __cplusplus
 extern "C" {
#endif //__cplusplus
/**
  * @brief  枚举，SpiMaster对象的速率
  */
typedef enum
{
    CE_SPI_MASTER_SPEED_1GBPS = 0x00,
    CE_SPI_MASTER_SPEED_500MBPS,
    CE_SPI_MASTER_SPEED_100MBPS,
    CE_SPI_MASTER_SPEED_50MBPS,
    CE_SPI_MASTER_SPEED_20MBPS,
    CE_SPI_MASTER_SPEED_10MBPS,
    CE_SPI_MASTER_SPEED_5MBPS,
    CE_SPI_MASTER_SPEED_1MBPS,
    CE_SPI_MASTER_SPEED_500KBPS,
    CE_SPI_MASTER_SPEED_100KBPS,
    CE_SPI_MASTER_SPEED_50KBPS,
    CE_SPI_MASTER_SPEED_10KBPS,
    CE_SPI_MASTER_SPEED_5KBPS,
    CE_SPI_MASTER_SPEED_1KBPS
}CE_SPI_MASTER_SPEED;

/**
  * @brief  枚举，SpiMaster对象时钟在空闲时的状态
  */
typedef enum
{
    CE_SPI_MASTER_CLOCK_POLARITY_HIGH = 0x00,           /*!< 串行同步时钟的空闲状态为高电平，大多设备都为高电平，所以默认为此值*/
    CE_SPI_MASTER_CLOCK_POLARITY_LOW                    /*!< 串行同步时钟的空闲状态为低电平*/
}CE_SPI_MASTER_CLOCK_POLARITY;

/**
  * @brief  枚举，SpiMaster对象时钟在空闲时的状态
  */
typedef enum
{
    CE_SPI_MASTER_CLOCK_PHASE_2Edge = 0x00,             /*!< 串行同步时钟的第二个跳变沿（上升或下降）数据被采样，大多设备都为第二个跳变沿，所以默认为此值*/
    CE_SPI_MASTER_CLOCK_PHASE_1Edge                     /*!< 串行同步时钟的第一个跳变沿（上升或下降）数据被采样，用于特殊模块，例如: nRf24L01*/
}CE_SPI_MASTER_CLOCK_PHASE;

/**
  * @brief  结构体，SpiMaster对象可用属性集合
  */
typedef struct
{
    CE_RESOURCE         ceResource;                     /*!< SPI对应的资源号*/
    CE_SPI_MASTER_SPEED ceSpiMasterSpeed;               /*!< SPI工作的速率*/
    CE_SPI_MASTER_CLOCK_POLARITY ceSpiMasterClockPolarity;/*!< SPI工作时同步时钟的空闲状态*/
    CE_SPI_MASTER_CLOCK_PHASE    ceSpiMasterClockPhase;/*!< SPI工作时同步时钟在指定跳变沿时采样数据*/
    CeExSpiMasterPar    ceExSpiMasterPar;               /*!< 与处理器平台相关的额外参数结构体，用以提高代码效率，用户无须关注*/
}CeSpiMaster;

/**
  * @brief  结构体，SpiMaster对象可用操作集合
  */
typedef struct
{
    CE_STATUS   (*initial)(CeSpiMaster* ceSpiMaster);   /*!< @brief 初始化SPI
                                                             @param ceSpicMaster:SpiMaster属性对象指针*/

    void        (*start)(CeSpiMaster* ceSpiMaster);     /*!< @brief 开始SPI
                                                             @param ceSpicMaster:SpiMaster属性对象指针*/

    void        (*stop)(CeSpiMaster* ceSpiMaster);      /*!< @brief 停止SPI
                                                             @param ceSpicMaster:SpiMaster属性对象指针*/

    uint8       (*writeReadByte)(CeSpiMaster* ceSpiMaster, uint8 writeVal);/*!<
                                                             @brief 发送并接收数据
                                                             @param ceSpicMaster:SpiMaster属性对象指针
                                                             @param writeVal:要发送的数据，单位字节
                                                             @return 读到的数据，单位字节*/

    void        (*setNSSBit)(CeSpiMaster* ceSpiMaster); /*!< @brief 设置SpiMaster的NSS引脚电平为高
                                                             @param ceSpicMaster:SpiMaster属性对象指针*/

    void        (*resetNSSBit)(CeSpiMaster* ceSpiMaster);/*!<
                                                             @brief 设置SpiMaster的NSS引脚电平为低
                                                             @param ceSpicMaster:SpiMaster属性对象指针*/

    CE_STATUS   (*lockBus)(CeSpiMaster* ceSpiMaster);   /*!< @brief 获取Spi总线控制权
                                                             @param ceSpicMaster:SpiMaster属性对象指针*/

    void        (*unlockBus)(CeSpiMaster* ceSpiMaster); /*!< @brief 释放Spi总线控制权
                                                             @param ceSpicMaster:SpiMaster属性对象指针*/

}CeSpiMasterOp;
extern const CeSpiMasterOp ceSpiMasterOp;           /*!< 所有与SpiMaster相关的操作*/
#ifdef __cplusplus
 }
#endif //__cplusplus
#endif //__CE_SPI_H__
