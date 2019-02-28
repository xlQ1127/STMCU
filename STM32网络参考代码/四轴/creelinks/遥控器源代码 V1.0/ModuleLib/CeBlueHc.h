/**
  ******************************************************************************
  * @file    CeBlueHc.h
  * @author  Creelinks Application Team
  * @version V1.0.0
  * @date    2017-01-06
  * @brief   适用于CeBlueHc模块的驱动头文件
  ******************************************************************************
  * @attention
  *
  *1)无
  *
  * <h2><center>&copy; COPYRIGHT 2016 Darcern</center></h2>
  ******************************************************************************
  */
#ifndef __CE_BLUE_HC_H__
#define __CE_BLUE_HC_H__
#ifdef __cplusplus
 extern "C" {
#endif //__cplusplus
#include "Creelinks.h"
#define __CE_BLUE_HC_VERSION__ 1                                             /*!< 此驱动文件的版本号*/
#define __CE_BLUE_HC_NEED_CREELINKS_VERSION__ 1                              /*!< 需要Creelinks平台库的最低版本*/
#if (__CE_CREELINKS_VERSION__ < __CE_BLUE_HC_NEED_CREELINKS_VERSION__)       /*!< 检查Creelinks平台库的版本是否满足要求*/
#error "驱动文件CeBlueHc.h需要高于1.0以上版本的Creelink库，请登陆www.creelinks.com下载最新版本的Creelinks库。"
#else

#define CE_BLUE_HC_RECV_BUF_SIZE        256                            /*!< 蓝牙进行接收的缓存长度*/
#define CE_BLUE_HC_DEV_LIST_SIZE        3                               /*!< 指示查找设备时，最多可查找到的从设备数量*/
#define CE_BLUE_HC_FIND_DEV_OUT_TIME    48                              /*!< 指示查找设备的超时时间*/


#define  CE_BLUE_HC_DEV_TYPE_NO_STANDARD    "0x1F1F"                           /*!< 非标准蓝牙设备类*/
#define  CE_BLUE_HC_DEV_TYPE_IPHONE         "0x7A020C"                          /*!< 苹果手机蓝牙设备类*/
#define  CE_BLUE_HC_DEV_TYPE_ANDROID        "0x5A020C"                          /*!< 安卓手机蓝牙设备类（魅族，华为等）*/


/**
  * @brief  枚举，CeBlueHc对象的工作模式
  */
typedef enum
{
    CE_BLUE_HC_WORK_MODE_MASTER = 1,                                    /*!< 主设备，即主动查找其它设备并发出连接请求*/
    CE_BLUE_HC_WORK_MODE_SLAVE = 0,                                     /*!< 从设备，即被其它设备查找，并接收其它设备的连接请求*/
    CE_BLUE_HC_WORK_MODE_LOOP = 2,                                      /*!< 回环角色，被动连接，接收远程蓝牙主设备数据并将数据原样返回给远程蓝牙主设备*/
}CE_BLUE_HC_WORK_MODE;


typedef struct
{
    char devName[32];                                                   /*!< 模块使用的Uart资源*/
    char devAdress[32];                                                 /*!< 模块使用的Uart资源*/
}CeBlueHcDevInfo;

/*
 *CeBlueHc属性对像
 */
typedef struct
{
    CeUart  ceUart;                                                     /*!< 模块使用的Uart资源*/
    CeGpio  ceGpio0;                                                    /*!< 模块使用的Gpio资源*/
    CeGpio  ceGpio1;                                                    /*!< 模块使用的Gpio资源*/
    CeGpio  ceGpio2;                                                    /*!< 模块使用的Gpio资源*/
    CeBlueHcDevInfo ceBlueHcDevInfoList[CE_BLUE_HC_DEV_LIST_SIZE];      /*!< 当在主模式下，查找从设备时，查找到从设备对象的存放地*/
    uint8   ceBlueHcDevInfoFindDevCount;                                /*!< 存放查找到的从设备数量*/
    uint8   uartRecvBuf[CE_BLUE_HC_RECV_BUF_SIZE];                      /*!< Uart工作使用到的接收缓存*/
    uint8   isLockRecvBuf;                                              /*!< 锁定接收缓存，防止发送时与接收时的数据相冲突*/
}CeBlueHc;
/*
 *CeBlueHc操作对像
 */
typedef struct
{
    CE_STATUS           (*initial)(CeBlueHc* ceBlueHc, CE_RESOURCE ceUart, CE_RESOURCE ceGpio0,CE_RESOURCE ceGpio1,CE_RESOURCE ceGpio2);/*!<
                                                                             @brief CeBlueHc模块初始化
                                                                             @param ceBlueHc:CeBlueHc属性对象指针
                                                                             @param ceUart:CeBlueHc模块使用的Uart资源号
                                                                             @param ceGpio0-2:CeBlueHc模块使用的Gpio资源号*/

    CE_STATUS           (*parmentConfig)(CeBlueHc* ceBlueHc, CE_BLUE_HC_WORK_MODE ceBlueHcWorkMode, const char* ceBlueHcDevType,const char* devName, const char* password);/*!<
                                                                             @brief CeBlueHc模块参数配置
                                                                             @param ceBlueHc:CeBlueHc属性对象指针
                                                                             @param ceBlueHcWorkMode:CeBlueHc模块的工作方式，即主模块和从模块
                                                                             @param ceBlueHcDevType:当模块工作在主模式时，查找从设备时只查找此类型的；当模块工作在从模式时，为此模块的类型
                                                                             @param devName:此设备的名称
                                                                             @param password:此设备的配对密码*/

    void                (*outParmentConfig)(CeBlueHc* ceBlueHc);        /*!< @brief 退出参数配置状态（AT状态），模块重新上电，并进入正常工作模式
                                                                             @param ceBlueHc:CeBlueHc属性对象指针*/


    CeBlueHcDevInfo*    (*getCanConnectDevInfo)(CeBlueHc* ceBlueHc);    /*!< @brief 模式工作在主模式时，查找周围中可连接的蓝牙信息
                                                                             @param ceBlueHc:CeBlueHc属性对象指针
                                                                             @return 返回连接的蓝牙信息数组*/

    uint8               (*getCanConnectDevCount)(CeBlueHc* ceBlueHc);   /*!< @brief 模式工作在主模式时，查找周围中可连接的蓝牙设备数量
                                                                             @param ceBlueHc:CeBlueHc属性对象指针
                                                                             @return 返回可连接的蓝牙设备数量*/

    CE_STATUS           (*checkDevIsExist)(CeBlueHc* ceBlueHc, const char* devBlueName);/*!<
                                                                             @brief 模式工作在主模式时，查找指定蓝牙名称的设备是否存在并处于可连接状态
                                                                             @param ceBlueHc:CeBlueHc属性对象指针
                                                                             @param devBlueName:需要检查的从设备名称
                                                                             @return 返回CE_STATUS_SUCCESS表示可连接， 返回其它表示不可连接*/

    CE_STATUS           (*connectDevByName)(CeBlueHc* ceBlueHc, const char* devBlueName);/*!<
                                                                             @brief 模式工作在主模式时，使用设备名称来连接一个从设备
                                                                             @param ceBlueHc:CeBlueHc属性对象指针
                                                                             @param devBlueName:需要连接的从设备名称
                                                                             @return 返回CE_STATUS_SUCCESS表示连接成功， 返回其它表示连接失败*/

    uint8               (*getConnectStatus)(CeBlueHc* ceBlueHc);        /*!< @brief 获取模块的连接状态
                                                                             @param ceBlueHc:CeBlueHc属性对象指针
                                                                             @return 返回0x00:模块处于未配对状态；返回0x01:模块处于配对成功状态，可以进行数据传输*/

    void               (*sendData)(CeBlueHc* ceBlueHc, uint8* dataInBuf, uint16 sendCount);/*!<
                                                                             @brief 发送数据
                                                                             @param ceBlueHc:CeBlueHc属性对象指针
                                                                             @param dataInBuf:需要发送的数据缓存
                                                                             @param sendCount:需要发送的数据长度
                                                                             @return 实际发送完成的数据长度*/

    uint16              (*getRecvDataCount)(CeBlueHc* ceBlueHc);        /*!< @brief 获取接收缓存中的可读取的数据长度
                                                                             @param ceBlueHc:CeBlueHc属性对象指针
                                                                             @return 接收缓存中的可读取的数据数量*/

    uint16              (*readData)(CeBlueHc* ceBlueHc, uint8* dataOutBuf, uint16 readCount);/*!<
                                                                             @brief 从接收缓存中读取数据
                                                                             @param ceBlueHc:CeBlueHc属性对象指针
                                                                             @param dataOutBuf:读取数据存放的缓存
                                                                             @param readCount:需要读取的数据长度
                                                                             @return 实际读取到的数据长度*/
}CeBlueHcOpBase;
/*
 *CeBlueHc操作对象实例
 */
extern const CeBlueHcOpBase ceBlueHcOp;

#endif // (__CE_CREELINKS_VERSION__ < __CE_BLUE_HC_NEED_CREELINKS_VERSION__)
#ifdef __cplusplus
}
#endif //__cplusplus
#endif //__CE_BLUE_HC_H__

/**
******************************************************************************
* @brief  使用流程及示例程序(基于前后台非操作系统环境) 
* @function xxxxxzzzz
******************************************************************************

#include "Creelinks.h"
#include "CeBlueHc.h"
#include "CeTft320Nt.h"

CeTft320Nt ceTft320Nt;
CeBlueHc ceBlueHc;


int main(void)
{
    ceSystemOp.initial();                       //Creelinks环境初始化
    ceDebugOp.initial(R9Uart);                  //通过Uart串口输出Debug信息到上位机
    //TODO:请在此处插入模块初始化等操作
    ceTft320NtOp.initial(&ceTft320Nt,R26L36);
    ceDebugOp.registerAppendString(ceTft320NtOp.appendString);
    ceBlueHcOp.initial(&ceBlueHc,R18Uart,R10TI2c);

    while (1)
    {
        ceTaskOp.mainTask();                    //Creelinks环境主循环任务，请保证此函数能够被周期调用
        //TODO:请在此处插入用户操作
        while(CE_STATUS_SUCCESS != ceBlueHcOp.parmentConfig(&ceBlueHc,CE_BLUE_HC_WORK_MODE_MASTER,CE_BLUE_HC_DEV_TYPE_NO_STANDARD,"haha","1234"))
        {
            ceSystemOp.delayMs(100);
            ceDebugOp.printf("parmentConfig Faile \n");
        }

        while(CE_STATUS_SUCCESS != ceBlueHcOp.checkDevIsExist(&ceBlueHc,"CC Technology"))
        {
            ceSystemOp.delayMs(100);
            ceDebugOp.printf("checkDevIsExist Faile \n");
        }

        while(CE_STATUS_SUCCESS != ceBlueHcOp.connectDevByName(&ceBlueHc,"CC Technology"))
        {
            ceSystemOp.delayMs(100);
            ceDebugOp.printf("connectDevByName Faile \n");
        }

        ceBlueHcOp.outParmentConfig(&ceBlueHc);

        while(1)
        {
            ceSystemOp.delayMs(1000);
            ceDebugOp.printf("status %d\n",ceBlueHcOp.getConnectStatus(&ceBlueHc));
        }
    };
}

******************************************************************************
*/

