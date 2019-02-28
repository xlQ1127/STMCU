/**
  ******************************************************************************
  * @file    CeTMU.c
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
#include "CeTMU.h"
#ifdef __cplusplus
 extern "C" {
#endif //__cplusplus


CeTMU ceTMU;
uint8 nrfAddress[5] = {0x01,0x13,0x5C,0x0C,0x03};//设置发送地址
/**
  * @brief  供通讯模块接收数据后调用的回调函数
  * @param  recvBuf:接收到数据的缓存地址
  * @param  recvCount:接收到数据的长度
  */
void ceTMU_recvCallBack(uint8* recvBuf, uint16 recvCount)
{
    ceTMU.recvPackCount++;
    ceFifoOp.write(&(ceTMU.ceFifo),recvBuf,recvCount);
}

/**
  * @brief  供通讯模块接收数据后调用的回调函数
  * @param  recvBuf:接收到数据的缓存地址
  * @param  recvCount:接收到数据的长度
  */
void ceTMU_taskCallBack(void*  pAddPar)
{
    uint16 recvCount = ceFifoOp.getCanReadSize(&(ceTMU.ceFifo));
    if( recvCount > 0)
        {
        ceTMU.sendCallBack();
        ceFifoOp.read(&(ceTMU.ceFifo),ceTMU.useBuf,recvCount);
                ceTMU.recvCallBack(ceTMU.useBuf,recvCount);            
        }
}

/**
  * @brief  供通讯模块接收数据后调用的回调函数
  * @param  recvBuf:接收到数据的缓存地址
  * @param  recvCount:接收到数据的长度
  */
void ceTMU_recvCallBackByWifi(uint8 linkID, uint8* recvBuf, uint16 recvCount)
{
    ceTMU.recvPackCount++;
    ceTMU.recvCallBack(recvBuf,recvCount);
}

/**
  * @brief CeTMU模块初始化
  * @param intervalMs:定义发送时间间隔
  * @param recvCallBack:用户需提供的回调函数
  */
CE_STATUS ceTMU_initialByWifi(void (*recvCallBack)(uint8* recvBuf, uint16 recvCount))
{
    ceTMU.sendPackCount = 0;
    ceTMU.recvPackCount = 0;
    ceTMU.recvCallBack = recvCallBack;
    ceTMU.useType = CE_TMU_USE_WIFI;

    ceWifiEspOp.initial(&(ceTMU.ceWifiEsp),Uart1);
    ceDebugOp.printf("Set wifi work mode...\n");
    while(CE_STATUS_SUCCESS != ceWifiEspOp.setWorkMode(&(ceTMU.ceWifiEsp),CE_WIFI_ESP_MODE_AP))
    {
        ceDebugOp.printf("Set Error, Try again...\n");
        ceSystemOp.delayMs(500);
    };
    ceDebugOp.printf("Create wifi:/nSSID:%s\nPWD:%s\nIP:%s\n...\n",CE_TMU_WIFI_SSID,CE_TMU_WIFI_PWD,CE_TMU_WIFI_SERVER_IP);
    while(CE_STATUS_SUCCESS != ceWifiEspOp.createWifi(&(ceTMU.ceWifiEsp),CE_TMU_WIFI_SERVER_IP, CE_TMU_WIFI_SSID,CE_TMU_WIFI_PWD,1,CE_WIFI_ESP_ECN_WPA2_PSK))
    {
        ceDebugOp.printf("Create wifi Error, Try again...\n");
        ceSystemOp.delayMs(500);            
    };

    ceDebugOp.printf("Create server:/nIP:%s\nMODE:TCP\nPORT:%d\n...\n",CE_TMU_WIFI_SERVER_IP,CE_TMU_WIFI_SERVER_PORT);        
    while(CE_STATUS_SUCCESS != ceWifiEspOp.createServer(&(ceTMU.ceWifiEsp),CE_TMU_WIFI_SERVER_PORT,CE_WIFI_ESP_SOCKET_MODE_TCP,ceTMU_recvCallBackByWifi))
    {
        ceDebugOp.printf("Create server Error, Try again...\n");
        ceSystemOp.delayMs(500);            
    }

    ceDebugOp.printf("Wifi initial finish, wait for client to connect...",CE_TMU_WIFI_SERVER_IP,CE_TMU_WIFI_SERVER_PORT);    

    return CE_STATUS_SUCCESS;
}

/**
  * @brief CeTMU模块初始化
  * @param intervalMs:定义发送时间间隔
  * @param recvCallBack:用户需提供的回调函数
  */
CE_STATUS ceTMU_initialByNrf(void (*recvCallBack)(uint8* recvBuf, uint16 recvCount), void (*sendCallBack)(void))
{
    ceTMU.sendPackCount = 0;
    ceTMU.recvPackCount = 0;
    ceTMU.recvCallBack = recvCallBack;
    ceTMU.sendCallBack = sendCallBack;
    ceTMU.useType = CE_TMU_USE_NRF;

    ceTMU.ceFifo.buff =ceTMU.fifoBuf;
    ceTMU.ceFifo.buffSize = CE_TMU_NRF_FIFO_SIZE;
    ceFifoOp.initial(&(ceTMU.ceFifo));

    ceTMU.recvTask.callBack = ceTMU_taskCallBack;
    ceTMU.recvTask.ID = 100;

    #ifdef __CE_USE_RTOS__
    ceTMU.recvTask.isNewThread = 0x01;
    ceTMU.recvTask.pAddPar = &ceTMU;
    ceTMU.recvTask.taskName = "TMU Task";
    ceTMU.recvTask.taskPriority = CE_TASK_PRIORITY_H;
    ceTMU.recvTask.taskStackBuf = ceTMU.stackBuf;
    ceTMU.recvTask.taskStackBufSize = CE_TMU_NRF_FIFO_SIZE;
    #endif

    ceTaskOp.registerTask(&(ceTMU.recvTask));
    ceTaskOp.start(&(ceTMU.recvTask));

    ceWlsNrfOp.initial(&(ceTMU.ceWlsNrf),Spi1,PA11GIP,PA12CGI);
        
        ceTMU.sendCallBack ();
    ceWlsNrfOp.recv(&(ceTMU.ceWlsNrf),0,nrfAddress,ceTMU_recvCallBack);//启动接收
        return CE_STATUS_SUCCESS;
}

/**
  * @brief CeTMU模块初始化
  * @param intervalMs:定义发送时间间隔
  * @param recvCallBack:用户需提供的回调函数
  */
CE_STATUS ceTMU_initialByBlue(void (*recvCallBack)(uint8* recvBuf, uint16 recvCount))
{
    ceTMU.sendPackCount = 0;
    ceTMU.recvPackCount = 0;
    ceTMU.recvCallBack = recvCallBack;
    ceTMU.useType = CE_TMU_USE_BLUE;
    return CE_STATUS_SUCCESS;
}

/**
  * @brief 发送数据，注意：函数内部会检测距离上一次发送数据的时间是否大于intervalMs，如果小于则直接返回
  * @param dataBuf:发送缓存地址
  * @param dataCount:发送缓存数据长度
  */
CE_STATUS ceTMU_sendData(uint8* dataBuf, uint16 dataCount)
{
    ceTMU.sendPackCount++;
    if(ceTMU.useType == CE_TMU_USE_WIFI)//如果处于WIFI工作模式
    {
        ceWifiEspOp.sendData(&(ceTMU.ceWifiEsp),0,dataBuf,dataCount);
    }else if(ceTMU.useType == CE_TMU_USE_NRF)//如果处于2.4G无线工作方式
    {
        ceWlsNrfOp.send(&(ceTMU.ceWlsNrf),nrfAddress,dataBuf,dataCount);//发送数据给控制器
        ceWlsNrfOp.recv(&(ceTMU.ceWlsNrf),0,nrfAddress,ceTMU_recvCallBack);//启动接收
    }else if(ceTMU.useType == CE_TMU_USE_BLUE)//如果处于蓝牙工作方式
    {
        //当前软件版本暂不支持蓝牙通讯方式
    }
    return     CE_STATUS_SUCCESS;
} 
/**
  * @brief 检测是否通讯中断
  * @return CE_STATUS_SUCCESS：通讯正常； 其它：通讯中断
  */
CE_STATUS ceTMU_checkConnectStatus()
{
    CE_STATUS ceStatus = CE_STATUS_SUCCESS;
    if(ceTMU.sendPackCount >= 10)
    {
        if(ceMathOp.abs((int)(ceTMU.recvPackCount) - (int)(ceTMU.sendPackCount)) >= 10)//连续发送10帧，但没有收到回复，则认为与控制器断连
            ceStatus = CE_STATUS_FAILE;    
        ceTMU.sendPackCount=0;
        ceTMU.recvPackCount=0;    
    }
    return ceStatus;
}

/**
  * @brief  CeTMU模块操作对象定义
  */
const CeTMUOp ceTMUOp = {ceTMU_initialByWifi,ceTMU_initialByNrf,ceTMU_initialByBlue,ceTMU_sendData,ceTMU_checkConnectStatus};

#ifdef __cplusplus
 }
#endif //__cplusplus
