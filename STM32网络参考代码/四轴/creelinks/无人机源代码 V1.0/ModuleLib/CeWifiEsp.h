/**
  ******************************************************************************
  * @file    CeWifiEsp.h
  * @author  Creelinks Application Team
  * @version V1.0.0
  * @date    2017-01-06
  * @brief   适用于CeWifiEsp模块的驱动头文件
  ******************************************************************************
  * @attention
  *
  *1)
  *
  * <h2><center>&copy; COPYRIGHT 2016 Darcern</center></h2>
  ******************************************************************************
  */
#ifndef __CE_WIFI_ESP_H__
#define __CE_WIFI_ESP_H__
#ifdef __cplusplus
 extern "C" {
#endif //__cplusplus
#include "Creelinks.h"
#define __CE_WIFI_ESP_VERSION__ 1                                       /*!< 此驱动文件的版本号*/
#define __CE_WIFI_ESP_NEED_CREELINKS_VERSION__ 1                        /*!< 需要Creelinks平台库的最低版本*/
#if (__CE_CREELINKS_VERSION__ < __CE_WIFI_ESP_NEED_CREELINKS_VERSION__) /*!< 检查Creelinks平台库的版本是否满足要求*/
#error "驱动文件CeWifiEsp.h需要高于1.0以上版本的Creelink库，请登陆www.creelinks.com下载最新版本的Creelinks库。"
#else
#define CE_WIFI_ESP_UT                                                /*!< ESP8266是否启用透传(默认关闭)。注意：必须是STA的单连接模式才可以启用透传，即如果启用，则只能作为客户端连接一个服务器*/

#define CE_WIFI_ESP_RECV_BUF_SIZE       256                            /*!< UART使用的收fifo的容量，可根据实际情况增加或减少，但需要保证接收一次的数据量小于此值*/
#define CE_WIFI_ESP_MAX_CONNECT_BUF     10                              /*!< 查看从设备或客户端连接列表的最大值*/
/*
 *枚举 此模式的工作方式
*/
typedef enum
{
    CE_WIFI_ESP_MODE_STA = 1,                                           /*!< STA模式: Station, 类似于无线终端，sta本身并不接受无线的接入，它可以连接到AP，一般无线网卡即工作在该模式*/
    CE_WIFI_ESP_MODE_AP = 2,                                            /*!< AP模式: Access Point，提供无线接入服务，允许其它无线设备接入，提供数据访问，一般的无线路由/网桥工作在该模式下*/
    CE_WIFI_ESP_MODE_STA_AP=3,                                          /*!< AP+STA*/
}CE_WIFI_ESP_MODE;

/*
 *枚举 AP模式下，创建新Wifi时的加密方式
*/
 typedef enum
{
    CE_WIFI_ESP_ECN_OPEN = 0,                                           /*!< 开放式*/
    CE_WIFI_ESP_ECN_WEP = 1,                                            /*!< WEP（有线等效加密）——采用WEP 64位或者128位数据加密*/
    CE_WIFI_ESP_ECN_WPA_PSK = 2,                                        /*!< WPA-PSK[TKIP]——采用预共享密钥的Wi-Fi保护访问，采用WPA-PSK标准加密技术，加密类型为TKIP*/
    CE_WIFI_ESP_ECN_WPA2_PSK = 3,                                       /*!< WPA2-PSK[AES]——采用预共享密钥的Wi-Fi保护访问（版本2），采用WPA2-PSK标准加密技术，加密类型为AES*/
    CE_WIFI_ESP_ECN_WPA_WPA2_PSK = 4,                                   /*!< WPA-PSK[TKIP] + WPA2-PSK [AES]——允许客户端使用WPA-PSK [TKIP]或者WPA2-PSK [AES]*/
}CE_WIFI_ESP_ECN;

/*
 *枚举 SOCKET的连接方式
*/
typedef enum
{
    CE_WIFI_ESP_SOCKET_MODE_TCP=0,                                      /*!< TCP连接方式*/
    CE_WIFI_ESP_SOCKET_MODE_UDP=1                                       /*!< UDP连接方式*/
}CE_WIFI_ESP_SOCKET_MODE;
/*
 *枚举 AP模式入，与此模块连接拉设备信息
*/
typedef struct
{
    char ip[16];                                                        /*!< IP地址，字符串"192.168.1.1"方式*/
    char mac[18];                                                       /*!< MAC地址，字符串"8F:34:32:A4:56:76"方式*/
}CeWifiEspAPLinkDevInfo;

/*
    *枚举 已经创建的SOCKET连接详细信息
*/
typedef struct
{
    CE_WIFI_ESP_SOCKET_MODE socketMode;                                 /*!< 是TCP还是UDP*/
    char                    ip[16];                                     /*!< IP地址，字符串"192.168.1.1"方式*/
    uint16                  port;                                       /*!< 端口号*/
    uint8                   linkID;                                     /*!< 此条连接的ID，ESP8266一共可以有5条连接同时存在*/
    uint8                   moduleLinkType;                             /*!< 指此连接，模块在其中是服务器的角色，还是客户端的角色。1做为服务器，0做为客户端*/
}CeWifiEspLinkInfo;

/*
 *枚举 环境中可用Wifi的详细信息例表
*/
typedef struct
{
    CE_WIFI_ESP_ECN ceWifiEpsEcn;                                       /*!< 加密方式*/
    char ssid[64];                                                      /*!< wifi的ssid名称*/
    int signal;                                                         /*!< 信号强度*/
    char mac[18];                                                       /*!< Mac地址*/
    uint16 other1;                                                      /*!< */
    uint16 other2;                                                      /*!< */
}CeWifiEspCanConnectWifiInfo;

/*
 *CeWifiEsp属性对像
 */
typedef struct
{
    CeUart              ceUart;                                                     /*!< 模块使用到的UART资源接口对象*/
    CeTask              ceTask;                                                     /*!< 此模块使用到的任务函数，主要起到接收数据的检查，与执行用户提供的接收回调函数*/
    CE_WIFI_ESP_MODE    wifiEspMode;                                                /*!< 此模块的工作方式，AP与STA两种*/
    CeWifiEspAPLinkDevInfo      apLinkDevInfoList[CE_WIFI_ESP_MAX_CONNECT_BUF];     /*!< AP模式下，连接到此模块的Wifi下的设备例表*/
    CeWifiEspLinkInfo           linkInfoList[CE_WIFI_ESP_MAX_CONNECT_BUF];          /*!< 服务器或客户端条件下建立的SOCKET连接例表*/
    CeWifiEspCanConnectWifiInfo staCanConnectWifiList[CE_WIFI_ESP_MAX_CONNECT_BUF]; /*!< 模块对周围的可用Wifi进行搜索，查找到的可用Wifi例表*/
    void    (*callBackClientRecv[5])(uint8* recvBuf, uint16 recvCount);             /*!< 处于客户端模式时，可建的5个客户端分别对应的接收回调*/
    void    (*callBackServerRecv)(uint8 linkID,uint8* recvBuf, uint16 recvCount);   /*!< 处于服务器模式时，服务器收到数据后执行的回调,linkID:第几个客户端发送来的数据*/
    uint8   isLockRecvBuf;                                                          /*!< 锁定接收缓存，防止发送时与接收时的数据相冲突*/
    uint8   isServerMode;                                                           /*!< 当前环境是处于服务器，还是客户端状态，用于判断调用服务器回调，还是客户端回调*/
    uint8   apLinkDeviceCount;                                                      /*!< AP模式下，有多少个设备连接到此模块的Wifi上*/
    uint8   linkInfoCount;                                                          /*!< AP或STA模式下，有多少个SOCKET连接存在，UDP与TCP*/
    uint8   staCanConnectwifiCount;                                                 /*!< 在STA模式下，查找到的环境中的可用Wifi数量*/
    uint8   isInUTSend;                                                             /*!< 客房端条件下，是否处于透传模式下*/
    uint8   uartRecvBuf[CE_WIFI_ESP_RECV_BUF_SIZE];                                 /*!< Uart工作使用到的接收缓存*/
    uint8   recvData[CE_WIFI_ESP_RECV_BUF_SIZE];                                    /*!< 存放接收到的数据，用于用户直接调用处理，数据处理回调处理完成后，此中的数据才能够被刷新，否则数据均将保存到fifo中*/
}CeWifiEsp;

/*
 *CeWifiEsp操作对象
 */
typedef struct
{

    CE_STATUS   (*initial)(CeWifiEsp* ceWifiEsp,CE_RESOURCE ceUart);                                    /*!< @brief  使用一个UART资源来初始化CeWifiEsp模块
                                                                                                             @param  ceWifiEsp:CeWifiEsp属性对象
                                                                                                             @param  ceUart:此模块使用到的Uart资源接口*/

    CE_STATUS   (*setWorkMode)(CeWifiEsp* ceWifiEsp,CE_WIFI_ESP_MODE ceWifiEspMode);                    /*!< @brief  以AP模式或STA模式初始化WIFI
                                                                                                             @param  ceWifiEsp:CeWifiEsp属性对象
                                                                                                             @param  ceWifiEspMode:此模块的工作方式，即AP或STA方式，
                                                                                                                    以AP模式（模块做为热点，其它设备连接模块）进行模块配置，
                                                                                                                    以STA（模块做为从设备，连接其它已存在的热点，如家用路由器、手机上建立的热点等）进行模块配置*/


    CE_STATUS   (*createWifi)(CeWifiEsp* ceWifiEsp,const char* ip,const char* ssid, const char* passWord,uint8 channelNumber,CE_WIFI_ESP_ECN ceWifiEspEcn);
                                                                                                        /*!< @brief  创建一个AP热点
                                                                                                             @param  ceWifiEsp:CeWifiEsp属性对象
                                                                                                             @param  ip:此AP热点的IP地址，其它连接到此模块的设备在IP在此基础上增加
                                                                                                             @param  ssid:此AP热点的Wifi名称
                                                                                                             @param  channelNumber:Wifi工作信道，范围1－13，初始化时给范围内任意值即可。
                                                                                                             @param  ceWifiEspEcn:此AP热点创建的Wifi加密方式，建议CE_WIFI_ESP_ECN_WPA2_PSK*/

    CeWifiEspAPLinkDevInfo* (*getConnectedDeviceList)(CeWifiEsp* ceWifiEsp);                            /*!< @brief  AP工作方式下，获得与此AP据点连接的从设备的IP与MAC地址例表
                                                                                                             @param  ceWifiEsp:CeWifiEsp属性对象
                                                                                                             @return 从设备信息例表*/

    uint8       (*getConnectedDeviceCount)(CeWifiEsp* ceWifiEsp);                                       /*!< @brief  AP工作方式下，获得与此AP据点连接的从设备的数量，可用于判断有无设备连接到此Wifi之上
                                                                                                             @param  ceWifiEsp:CeWifiEsp属性对象
                                                                                                             @return 从设备的数量*/

    CeWifiEspCanConnectWifiInfo* (*getCanConnectWifiList)(CeWifiEsp* ceWifiEsp);                        /*!< @brief  STA工作方式下，查找周围环境中可连接的Wifi信号
                                                                                                             @param  ceWifiEsp:CeWifiEsp属性对象
                                                                                                             @return 可用Wifi的例表指针*/

    uint8       (*getCanConnectWifiCount)(CeWifiEsp* ceWifiEsp);                                        /*!< @brief  STA工作方式下，查找周围环境中可连接的Wifi信号数量
                                                                                                             @param  ceWifiEsp:CeWifiEsp属性对象
                                                                                                             @return 可用Wifi的数量*/

    CE_STATUS   (*checkCanConnectSsidIsExist)(CeWifiEsp* ceWifiEsp, const char* ssid);                  /*!< @brief  STA工作方式下，查找周围环境中是否有给出的ssid的Wifi信号
                                                                                                             @param  ceWifiEsp:CeWifiEsp属性对象*/

    CE_STATUS   (*connectWifi)(CeWifiEsp* ceWifiEsp, const char* ssid, const char* passWord);           /*!< @brief  STA工作方式，连接一个已经存在的热点，参数为热点的SSID与PWD
                                                                                                             @param  ceWifiEsp:CeWifiEsp属性对象
                                                                                                             @param  ssid:需要连接的Wifi名称
                                                                                                             @param  passWord:Wifi密码*/

    CE_STATUS   (*setStaIp)(CeWifiEsp* ceWifiEsp, const char* ip);                                      /*!< @brief  STA工作方式下，连接到热点后的IP，注意此时配置的IP应与热点分配的ID相匹配。
                                                                                                             @param  ceWifiEsp:CeWifiEsp属性对象
                                                                                                             @param  ip:需要设定的IP地址*/

    CE_STATUS   (*createServer)(CeWifiEsp* ceWifiEsp,uint16 serVerPortNum,CE_WIFI_ESP_SOCKET_MODE ceWifiEspSocketMode, void (*callBackServerRecv)(uint8 linkID,uint8* recvBuf, uint16 recvCount));
                                                                                                        /*!< @brief  创建一个服务器，用来接收与发送数据，AP与STA模式均可创建
                                                                                                             @param  ceWifiEsp:CeWifiEsp属性对象
                                                                                                             @param  serVerPortNum:需创建的服务器端口号
                                                                                                             @param  linkID:由于是创建了服务器，则此服务器最大只允许5个客户端与其连接，linkID指的就是这5个客户端的编号，最先连到服务器的客户端为0。回调中LinkID表示服务器接收到了数据，此数据是哪个客户端发送来的
                                                                                                             @param  callBackServerRecv:服务器接收到数据后的回调*/

    CE_STATUS   (*connectServer)(CeWifiEsp* ceWifiEsp, uint8 linkID, const char* serverIP,uint16 serVerPortNum,CE_WIFI_ESP_SOCKET_MODE ceWifiEspSocketMode, void (*callBackClientRecv)(uint8* recvBuf, uint16 recvCount));
                                                                                                        /*!< @brief  连接一个已存在的服务器，用来接收与发送数据，AP与STA模式均可创建
                                                                                                             @param  ceWifiEsp:CeWifiEsp属性对象
                                                                                                             @param  linkID:ESP8266可以创建5个客户端（编号0－4）分别连接各自的服务器，LinkID指使用哪个客户端编号号连接服务器
                                                                                                             @param  serverIP:需要连接的服务器的IP
                                                                                                             @param  serVerPortNum:需要连接的服务器的端口
                                                                                                             @param ceWifiEspSocketMode:连接方式，TCP或UDP
                                                                                                             @param callBackClientRecv:此客户端接收到数据后的回调*/

    CE_STATUS           (*startUTSendOnClient)(CeWifiEsp* ceWifiEsp);                                   /*!< @brief  启动透传，只有模块是单连接，且为客户端才可以有此功能
                                                                                                             @param  ceWifiEsp:CeWifiEsp属性对象*/

    CE_STATUS           (*stopUTSendOnClient)(CeWifiEsp* ceWifiEsp);                                    /*!< @brief  退出透传，只有模块是单连接，且为客户端才可以有此功能
                                                                                                             @param  ceWifiEsp:CeWifiEsp属性对象*/

    CeWifiEspLinkInfo*  (*getLinkList)(CeWifiEsp* ceWifiEsp);                                           /*!< @brief  获得本模块所建立的连接列表，包含服务器模式下，与此服务器连接的客户端数量；客户端模式下，有多个少客户端正在建立并且与服务端连接（最大可5个）
                                                                                                             @param  ceWifiEsp:CeWifiEsp属性对象
                                                                                                             @return SOCKET连接例表*/

    uint8               (*getLinkCount)(CeWifiEsp* ceWifiEsp);                                          /*!< @brief  获得本模块所建立的连接数量，可以用来判断：服务器工作方式下，有无客户端连接。客户端方式下，是否正常连接到服务器。
                                                                                                             @param  ceWifiEsp:CeWifiEsp属性对象
                                                                                                             @return SOCKET连接数量*/

    CE_STATUS           (*sendData)(CeWifiEsp* ceWifiEsp,uint8 linkID, uint8* dataBuf,uint16 dataSize); /*!< @brief  向一个连接号linkID发送数据。
                                                                                                             @param  ceWifiEsp:CeWifiEsp属性对象
                                                                                                             @param  linkID:服务器模式下，此连接号指的是与其连接的客户端序号；客户端模式下，此连接号指使用哪个客户端（编号0－4）
                                                                                                             @param  dataBuf:需要发送的数据缓存
                                                                                                             @param  dataSize:需要发送的数据长度*/

}CeWifiEspOpBase;
/*
 *CeWifiEsp操作对象实例
 */
extern const CeWifiEspOpBase ceWifiEspOp;

#endif //(__CE_CREELINKS_VERSION__ < __CE_WIFI_ESP_NEED_CREELINKS_VERSION__)
#ifdef __cplusplus
 }
#endif //__cplusplus
#endif //__CE_WIFI_ESP_H__

/**
******************************************************************************
* @brief  使用流程及示例程序(基于前后台非操作系统环境) 
* @function 使用已连接家用无线路的电脑或手机创建一个服务器（可下载安装网络调试助手），
          由用CeWifiEsp模块连接此热点（Wifi），并连接刚创建的服务器。接收服务器发送来
          的数据，并通过串口打印；同时每1S向服务器发送一次数据包。
******************************************************************************
#include "Creelinks.h"
#include "CeWifiEsp.h"
CeWifiEsp myWifiEsp;

void callBackClient0(uint8* recvBuf, uint16 recvCount)
{
    uint16 i;
    ceDebugOp.printf("CeWifiEsp has recv data from server, DataCount:%d, Data:\n",recvCount);
    for(i=0;i<recvCount;i++)
    {
        ceDebugOp.printf("%c",recvBuf[i]);
    }
    ceDebugOp.printf("\n");
}

int main(void)
{
    ceSystemOp.initial();                                        //Creelinks环境初始化
    ceDebugOp.initial(R9Uart);                                   //通过Uart串口输出Debug信息到上位机
    //TODO:请在此处插入模块初始化等操作
    ceWifiEspOp.initial(&myWifiEsp,R14Uart,CE_WIFI_ESP_MODE_STA);//使用R14Uart资源，并使用STA工作方式（即模块作为终端，连接一个Wifi）
    while(CE_STATUS_SUCCESS !=  ceWifiEspOp.checkCanConnectSsidIsExist(&myWifiEsp,"Creelinks"))//循环检查环境中是否存在Creelinks此Wifi信号，用户根据自已的Wifi名称来修改SSID。
    {
        ceDebugOp.printf("Cannot find wifi signal by ssid: Creelinks");
        ceSystemOp.delayMs(1000);
    }
    //ceWifiEspOp.setStaIp(&myWifiEsp,"192.168.1.188");//配置模块的IP地址，需要与路由器保持一致；也可不用配置，模块将会自动获取一个IP。
    while(CE_STATUS_SUCCESS != ceWifiEspOp.connectServer(&myWifiEsp,0,"192.168.1.156",2121,CE_WIFI_ESP_SOCKET_MODE_TCP,callBackClient0))//连接一个由网络调试助手配置好的服务器，需提供服务器IP与端口。
    {
        ceDebugOp.printf("Cannot connect server,IP:192.168.1.156, Port:2121\n");
        ceSystemOp.delayMs(1000);
    }

    while (1)
    {
        ceTaskOp.mainTask();                           //Creelinks环境主循环任务，请保证此函数能够被周期调用
        //TODO:请在此处插入用户操作
        ceWifiEspOp.sendData(&myWifiEsp,0,(uint8*)("Hello server!\n"),ceStringOp.strlen("Hello server!\n"));//发送数据至服务器
        ceSystemOp.delayMs(1000);
    };
}
******************************************************************************
*/

/**
******************************************************************************
* @brief  使用流程及示例程序(基于前后台非操作系统环境) 
* @function 使用CeWifiEsp建立一个独立的热点，并建立一个服务器；通过手机或笔记本电脑连接此wifi信号，并互传数据包。
******************************************************************************
#include "Creelinks.h"
#include "CeWifiEsp.h"
CeWifiEsp myWifiEsp;

void callBackServerRecv(uint8 linkID,uint8* recvBuf, uint16 recvCount)
{
    uint16 i;
    ceDebugOp.printf("CeWifiEsp has recv data from client, linkID:%d,  DataCount:%d, Data:\n",linkID,recvCount);
    for(i=0;i<recvCount;i++)
    {
        ceDebugOp.printf("%c",recvBuf[i]);
    }
    ceDebugOp.printf("\n");
}

int main(void)
{
    ceSystemOp.initial();                       //Creelinks环境初始化
    ceDebugOp.initial(R9Uart);                  //通过Uart串口输出Debug信息到上位机
    //TODO:请在此处插入模块初始化等操作
    ceWifiEspOp.initial(&myWifiEsp,R14Uart,CE_WIFI_ESP_MODE_AP);//使用R14Uart资源，并使用AP工作方式（即模块作为路由器，允许其它设备与其相联）
    while(CE_STATUS_SUCCESS !=  ceWifiEspOp.createWifi(&myWifiEsp,"192.168.1.188","Creelinks","12345678",1,CE_WIFI_ESP_ECN_WPA2_PSK))//创建一个指定IP、Wifi名、密码、加密方式的Wifi热点，使用手机或笔记本电脑与之相联
    {
        ceDebugOp.printf("Cannot create wifi!\n");
        ceSystemOp.delayMs(1000);
    }

    while(ceWifiEspOp.getConnectedDeviceCount(&myWifiEsp) == 0)                              //检查并等待有无设备与之相连接
    {
        ceDebugOp.printf("Wait for device connect wifi...\n");
        ceSystemOp.delayMs(1000);
    }

    while(CE_STATUS_SUCCESS !=  ceWifiEspOp.createServer(&myWifiEsp,2121,callBackServerRecv))//创建一个端口为2121的服务器
    {
        ceDebugOp.printf("Cannot create server!\n");
        ceSystemOp.delayMs(1000);
    }

    while(ceWifiEspOp.getLinkCount(&myWifiEsp) == 0)//检查并等待客户端与之连接
    {
        ceDebugOp.printf("Wait for client connect server...\n");
        ceSystemOp.delayMs(1000);
    }
    while (1)
    {
        ceTaskOp.mainTask();                       //Creelinks环境主循环任务，请保证此函数能够被周期调用
        //TODO:请在此处插入用户操作
        ceWifiEspOp.sendData(&myWifiEsp,0,(uint8*)("Hello server!\n"),ceStringOp.strlen("Hello server!\n"));//发送数据至客户端
        ceSystemOp.delayMs(1000);
    };
}
******************************************************************************
*/


