/**
  ******************************************************************************
  * @file    CeI2c.h
  * @author  Creelinks Application Team
  * @version V1.0.0
  * @date    2016-08-05
  * @brief   Creelinks平台CeI2c库头文件
  ******************************************************************************
  * @attention
  *
  *1)因平台不同，部分I2c总线采用软件模拟的方式实现，请参考CeI2c.c中的描述
  *
  * <h2><center>&copy; COPYRIGHT 2016 Darcern</center></h2>
  ******************************************************************************
  */
#ifndef __CE_I2C_H__
#define __CE_I2C_H__

#include "CeMcu.h"

#ifdef __cplusplus
 extern "C" {
#endif //__cplusplus

/**
  * @brief  枚举，I2cMaster对象速率
  */
typedef enum
{
    CE_I2C_SPEED_100KBPS,                                                   /*!< I2C总线速率：100Kbps*/
    CE_I2C_SPEED_400KBPS,                                                   /*!< I2C总线速率：400Kbps*/
    CE_I2C_SPEED_3_4MBPS,                                                   /*!< I2C总线速率：3.4Mbps*/
}CE_I2C_MASTER_SPEED;

/**
  * @brief  结构体，I2cMaster对象可用属性集合
  */
typedef struct
{
    CE_RESOURCE         ceResource;                                         /*!< I2C对应的资源号*/
    CE_I2C_MASTER_SPEED ceI2cMasterSpeed;                                   /*!< I2C配置的总线速率*/

    CeExI2cMasterPar    ceExI2cMasterPar;                                   /*!< 与处理器平台相关的额外参数结构体，用以提高代码效率，用户无须关注*/
}CeI2cMaster;

/**
  * @brief  结构体，I2cMaster对象可用操作集合
  */
typedef struct
{
    CE_STATUS   (*initial)(CeI2cMaster* ceI2cMaster);                       /*!< @brief 初始化I2C总线
                                                                                 @param ceI2cMaster:I2cMaster属性对象指针*/

    void        (*start)(CeI2cMaster* ceI2cMaster);                         /*!< @brief 开始I2C总线
                                                                                 @param ceI2cMaster:I2cMaster属性对象指针*/

    void        (*stop)(CeI2cMaster* ceI2cMaster);                          /*!< @brief 停止I2C总线
                                                                                 @param ceI2cMaster:I2cMaster属性对象指针*/

    void        (*writeByte)(CeI2cMaster* ceI2cMaster,uint8 val);   /*!< @brief 发送一个字节到I2C总线
                                                                                 @param ceI2cMaster:I2cMaster属性对象指针
                                                                                 @param val:要发送的数据，单位字节*/

    uint8       (*readByte)(CeI2cMaster* ceI2cMaster,uint8 isAck); /*!< @brief 接收一个字节到I2C总线
                                                                                 @param ceI2cMaster:I2cMaster属性对象指针
                                                                                 @param isAck:是否在接收完成后，发送应答信号
                                                                                 @return 返回收到的数据*/

    CE_STATUS   (*waitAck)(CeI2cMaster* ceI2cMaster);              /*!< @brief 等待从设备的应答信号
                                                                                 @param ceI2cMaster:I2cMaster属性对象指针*/

    CE_STATUS   (*lockBus)(CeI2cMaster* ceI2cMaster);                       /*!< @brief 获取I2c总线控制权
                                                                                 @param ceI2cMaster:I2cMaster属性对象指针*/

    void        (*unlockBus)(CeI2cMaster* ceI2cMaster);                     /*!< @brief 释放I2c总线控制权
                                                                                 @param ceI2cMaster:I2cMaster属性对象指针*/

}CeI2cMasterOp;
extern const CeI2cMasterOp ceI2cMasterOp;                               /*!< 所有与I2C相关的操作*/

#ifdef __cplusplus
 }
#endif //__cplusplus
#endif //__CE_I2C_H__

/**
******************************************************************************
* @brief  使用流程及示例程序(基于前后台非操作系统环境)
* @function 操作I2c读、写数据（详细示例可参考使用到I2c接口的模块）
******************************************************************************
#include "Creelinks.h"
CeI2cMaster myI2c;                                  //定义I2c属性对像
uint8 recvData;                                         //接收数据缓存
int main(void)
{
    ceSystemOp.initial();                           //Creelinks环境初始化
    ceDebugOp.initial(Uartx);                            //通过Uart串口输出Debug信息到上位机
                                                    //TODO:请在此处插入模块初始化等操作
    myI2c.ceResource = RxI2c;                       //指定I2c使用的资源号
    myI2c.ceI2cMasterSpeed = CE_I2C_SPEED_3_4MBPS;  //设定I2c的工作速率
    if (ceI2cMasterOp.initia(&myI2c) != CE_STATUS_SUCCESS)  //初始化I2c
    {
        ceDebugOp.printf("I2c initial fail!\n");
    }
    ceDebugOp.printf("I2c initial success!\n");
    while (1)
    {
        ceTaskOp.mainTask();                        //Creelinks环境主循环任务，请保证此函数能够被周期调用
                                                    //TODO:请在此处插入用户操作
        ceI2cMasterOp.lockBus(&myI2c);              //获得I2c总线控制权（因为I2c总线上可能挂有多个从设备，避免多个进程同时使用I2c总线）
        ceI2cMasterOp.start(&myI2c);                //开始总线
        ceI2cMasterOp.writeByte(&myI2c, 0x21);//发送数据
        recvData = ceI2cMasterOp.readByte(&myI2c, 0x00);//读取数据，0x00表示不需要从设备应答信号
        ceDebugOp.printf("I2c recv data:%d\n", recvData);//向上位机发送调试信息
        ceI2cMasterOp.stop(&myI2c);                 //停止I2c总线
        ceI2cMasterOp.unlockBus(&myI2c);            //释放总线控制权
    };
}
******************************************************************************
*/
