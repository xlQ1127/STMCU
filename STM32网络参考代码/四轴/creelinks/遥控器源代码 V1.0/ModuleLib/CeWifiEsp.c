/**
  ******************************************************************************
  * @file    CeWifiEsp.c
  * @author  Creelinks Application Team
  * @version V1.0.0
  * @date    2017-01-06
  * @brief   适用于CeWifiEsp模块的驱动库文件
  ******************************************************************************
  * @attention
  *
  *1)无
  *
  * <h2><center>&copy; COPYRIGHT 2016 Darcern</center></h2>
  ******************************************************************************
  */
#include"CeWifiEsp.h"
#ifdef __cplusplus
 extern "C" {
#endif //__cplusplus

#ifdef CE_WIFI_ESP_UT
#define CE_WIFI_ESP_CONNECT_MUL         0                               /*!< ESP8266连接数量，0为单连接，1为多连接*/
#else
#define CE_WIFI_ESP_CONNECT_MUL         1                               /*!< ESP8266连接数量，0为单连接，1为多连接*/
#endif

/**
  * @brief  复制数据
  * @param  desBuf:目的缓存
  * @param  srcBuf:源缓存
  * @param  cpCount:需复制的数据长度
  * @return 系统状态码，可能的值：CE_STATUS_SUCCESS,CE_STATUS_OUT_TIME
  */
void ceWifiEsp_cpData(uint8* desBuf, uint8* srcBuf, uint16 cpCount)
{
    uint16 i;
    for (i = 0; i < cpCount; i++)
    {
        desBuf[i] = srcBuf[i];
    }
}

/**
  * @brief  从Uart中读取数据，以endChar结尾结尾
  * @param  ceWifiEsp:CeWifiEsp属性对象
  * @param  buf:读出的数据需放置的缓存
  * @param  bufSize:缓存大小
  * @param  endChar:读数据，直到以此字符串结尾
  * @param  outTimeMs:超时时间
  * @return 系统状态码，可能的值：CE_STATUS_SUCCESS,CE_STATUS_OUT_TIME
  */
uint16 ceWifiEsp_readStringByEndChar(CeWifiEsp* ceWifiEsp, uint8* buf,uint16 bufSize, char* endChar, uint16 outTimeMs)
{
    uint16 tickMs = 0;
    uint16 checkIndex = 0;
    uint16 bufIndex = 0;
    char temp[2];
    temp[1] = '\0';
    buf[0] = '\0';

    //ceDebugOp.printf("endChar:%s\n ,",endChar);
    
    while (1)
    {
        if (ceUartOp.getRecvDataCount(&(ceWifiEsp->ceUart)) <= 0)   //如果没有收到数
        {
            if (tickMs >= outTimeMs)
            {
                buf[0] = '\0';
                return 0;
            }
            tickMs += 10;
            ceSystemOp.delayMs(10);
        }
        else   //如果有收到数
        {
            ceUartOp.readData(&(ceWifiEsp->ceUart), (uint8*)(temp), 1);//读取一个字节

            if (ceWifiEsp->isServerMode != 0x00 || ceWifiEsp->isInUTSend == 0x01)
            {
//ceDebugOp.printf("%s,",temp[0]);//把收到的数据打出来，调试时使用
            }
                        

            buf[bufIndex] = temp[0];
            bufIndex++;
            if (bufIndex == bufSize)//读取到的数据超过了接收缓存的最大长度，此时应该是接收的数据有误，所有应该直接返回。
            {
                buf[0] = '\0';            
//ceDebugOp.printf("ddd\n");                            
                return 0;
            }
            if (temp[0] == endChar[checkIndex])        //比较读取到的字节与需要对比的内容是否相同
            {
                checkIndex++;                       //如果相同，则让比较的索引增加1
                if (endChar[checkIndex] == '\0')    //如果增加1后，即下一个要比较的字节是空，则表明比较完成，返回成功
                {
                    buf[bufIndex - ceStringOp.strlen(endChar)] = '\0';
                    return bufIndex - ceStringOp.strlen(endChar)+1;
                }
            }
            else                                    //如果不相同，则将索引设置为0，再从0重新开始比较
            {
                checkIndex = 0;
            }
        }
    }

}

/**
  * @brief  从Uart中发数据给模块，并等待模块返回结果
  * @param  ceWifiEsp:CeWifiEsp属性对象
  * @param  sendMsg:要发送出去的内容
  * @param  recvMsg:期望接收到的内容,这里只检测是否包含
  * @param  outTimeMs:超时时间
  * @return 系统状态码，可能的值：CE_STATUS_SUCCESS,CE_STATUS_OUT_TIME
  */
CE_STATUS ceWifiEsp_sendDataAndCheck(CeWifiEsp* ceWifiEsp, char* sendMsg, char* recvMsg, uint16 outTimeMs)
{
    if (sendMsg == CE_NULL && ceWifiEsp->isInUTSend == 0x01)
    {
        return CE_STATUS_SUCCESS;
    }
    else
    {
        uint16 tickMs = 0;
        uint16 checkIndex = 0;
        uint16 checkEIndex = 0;
        uint16 timeOutMs = 0;
        char temp[2];
        char* tempError = "ERROR";
        temp[1] = '\0';


        while (ceWifiEsp->isLockRecvBuf == 0x01)
        {
            ceSystemOp.delayMs(1);
            timeOutMs++;
            if (timeOutMs >= 2000)
            {
                break;
            }
        }
        ceWifiEsp->isLockRecvBuf = 0x01;

        if (sendMsg != CE_NULL)
        {
            ceUartOp.clearRecvBuf(&(ceWifiEsp->ceUart));
            ceUartOp.sendData(&(ceWifiEsp->ceUart), (uint8*)sendMsg, ceStringOp.strlen(sendMsg));
        }
        while (1)
        {
            if (ceUartOp.getRecvDataCount(&(ceWifiEsp->ceUart)) <= 0)   //如果没有收到数
            {
                ceSystemOp.delayMs(10);
                tickMs += 10;
                if (tickMs >= outTimeMs)
                {
                    ceWifiEsp->isLockRecvBuf = 0x00;
                    return CE_STATUS_OUT_TIME;
                }
            }
            else   //如果有收到数
            {
                ceUartOp.readData(&(ceWifiEsp->ceUart), (uint8*)(temp), 1);//读取一个字节

                //ceDebugOp.printf(temp);//把收到的数据打出来，调试时使用

                if (temp[0] == tempError[checkEIndex])
                {
                    checkEIndex++;
                    if (tempError[checkEIndex] == '\0')
                    {
                        return CE_STATUS_FAILE;
                    }
                }

                if (temp[0] == recvMsg[checkIndex])        //比较读取到的字节与需要对比的内容是否相同
                {
                    checkIndex++;                       //如果相同，则让比较的索引增加1
                    if (recvMsg[checkIndex] == '\0')    //如果增加1后，即下一个要比较的字节是空，则表明比较完成，返回成功
                    {
                        ceWifiEsp->isLockRecvBuf = 0x00;
                        return CE_STATUS_SUCCESS;
                    }
                }
                else                                    //如果不相同，则将索引设置为0，再从0重新开始比较
                {
                    checkIndex = 0;
                }
            }
        }
    }
}

/**
  * @brief  设置模块超时时间
  * @param  ceWifiEsp:CeWifiEsp属性对象
  * @param  outTime:超时时间
  * @return 系统状态码，可能的值：CE_STATUS_SUCCESS,CE_STATUS_OUT_TIME
  */
CE_STATUS ceWifiEsp_atCIpSto(CeWifiEsp* ceWifiEsp,uint16 outTime)
{
    char sendBuf[30];
    CE_STATUS ceStatus;
    ceDebugOp.sprintf(sendBuf, "AT+CIPSTO=%d\r\n", outTime);
    ceStatus = ceWifiEsp_sendDataAndCheck(ceWifiEsp, sendBuf, "\r\nOK\r\n",500);
    #ifdef __CE_CHECK_PAR__
    ceDebugOp.printf("Run CMD:%s, run result:%s\n", sendBuf, ceSystemOp.getErrorMsg(ceStatus));
    #endif
    return ceStatus;
}
/**
  * @brief  非透传模式下，发送数据前的命令
  * @param  ceWifiEsp:CeWifiEsp属性对象
  * @param  id:linkID号
  * @param  dataSize:数据长度
  * @return 系统状态码，可能的值：CE_STATUS_SUCCESS,CE_STATUS_OUT_TIME
  */
CE_STATUS ceWifiEsp_atCIpSend(CeWifiEsp* ceWifiEsp,uint8 id, uint16 dataSize)
{
    if ((ceWifiEsp->isInUTSend == 0x01 || CE_WIFI_ESP_CONNECT_MUL == 0 )&& ceWifiEsp->isServerMode != 0x01)
    {
        return CE_STATUS_SUCCESS;
    }
    else
    {
        char sendBuf[30];
        CE_STATUS ceStatus;
        if (CE_WIFI_ESP_CONNECT_MUL == 0)
        {
                      if(ceWifiEsp->isServerMode == 0x01)
                            ceDebugOp.sprintf(sendBuf, "AT+CIPSEND=%d,%d\r\n", id, dataSize);
             else
                ceDebugOp.sprintf(sendBuf, "AT+CIPSEND=%d\r\n", dataSize);
        }
        else
        {
            ceDebugOp.sprintf(sendBuf, "AT+CIPSEND=%d,%d\r\n", id, dataSize);                    
        }
        ceStatus = ceWifiEsp_sendDataAndCheck(ceWifiEsp, sendBuf, ">",1000);
#ifdef __CE_CHECK_PAR__
        ceDebugOp.printf("Run CMD:%sRun result:%s\n\n", sendBuf, ceSystemOp.getErrorMsg(ceStatus));
#endif
        return ceStatus;
    }
}

/**
  * @brief  向Uart发灵气
  * @param  ceWifiEsp:CeWifiEsp属性对象
  * @param  dataBuf:要发送的数据缓存
  * @param  dataSize:要发送的数据长度
  */
void ceWifiEsp_sendUartData(CeWifiEsp* ceWifiEsp, uint8* dataBuf, uint16 dataSize)
{
    uint16 timeOutMs = 0;
    while (ceWifiEsp->isLockRecvBuf == 0x01)
    {
        ceSystemOp.delayMs(1);
        timeOutMs++;
        if (timeOutMs >= 2000)
        {
            break;
        }
    }
    ceWifiEsp->isLockRecvBuf = 0x01;
    ceUartOp.sendData(&(ceWifiEsp->ceUart), dataBuf, dataSize);
    ceWifiEsp->isLockRecvBuf = 0x00;
}

/**
  * @brief  任务Task，用于周期检查接收数据，并调用用户回调
  * @param  ceWifiEsp:CeWifiEsp属性对象
  * @param  pAddPar:CeWifiEsp指针
  */
void ceWifiEsp_taskCallBack(void* pAddPar)
{
    CeWifiEsp* ceWifiEsp = (CeWifiEsp*)(pAddPar);
    if (ceWifiEsp->isLockRecvBuf == 0x01)
        return;

    if ((ceWifiEsp->isInUTSend == 0x00 &&ceUartOp.getRecvDataCount(&ceWifiEsp->ceUart) <= 9) || (ceWifiEsp->isInUTSend == 0x01 &&ceUartOp.getRecvDataCount(&ceWifiEsp->ceUart) == 0))
        return;

    else
    {
        uint16 recvDataCount; 
        uint8 linkID = 0;
        if (ceWifiEsp->isInUTSend == 0x01)
        {
            recvDataCount = ceUartOp.getRecvDataCount(&ceWifiEsp->ceUart);
        }
        else
        {
            uint8 dataBuf[30];
            uint16 outTimeMs = 0;
            recvDataCount = ceWifiEsp_readStringByEndChar(ceWifiEsp, (uint8*)dataBuf, 30, "+IPD,",500);//
            if (recvDataCount == 0)         //如果没有读到数据，此时uart缓存的数据已读完，并且也发生超时了
                return;

            if(ceWifiEsp->isServerMode == 1 || CE_WIFI_ESP_CONNECT_MUL == 1)//服务器模式下一定为多连接
            {
                recvDataCount = ceWifiEsp_readStringByEndChar(ceWifiEsp, dataBuf, 30, ",",500);//读取是哪个客户端连接发送来的数据
                if (recvDataCount == 0 || recvDataCount > 2 || (dataBuf[0] - 0x30) > 4)
                    return;

                linkID = dataBuf[0] - 0x30;
            }else if(CE_WIFI_ESP_CONNECT_MUL == 0)
                linkID = 0;                                            

            recvDataCount = ceWifiEsp_readStringByEndChar(ceWifiEsp, dataBuf, 30, ":",500);//读取数据长度
            if (recvDataCount == 0 || recvDataCount > 6)
                return;


            recvDataCount = ceStringOp.atoi((char*)dataBuf);
            if (recvDataCount <= 0 || recvDataCount >= 1500)//检测读到的数据长度是否满足要求
                return;

            while (ceUartOp.getRecvDataCount(&ceWifiEsp->ceUart) < recvDataCount)//Uart接收缓存中的数据，不等于需要读取的数据长度
            {
                ceSystemOp.delayMs(1);
                outTimeMs++;
                if (outTimeMs > 2000)
                    return;
            }
        }
        if (recvDataCount != ceUartOp.readData(&(ceWifiEsp->ceUart), ceWifiEsp->recvData, recvDataCount))
            return;

        ceWifiEsp->recvData[recvDataCount] = '\0';

        if (ceWifiEsp->isServerMode == 0x00)
        {
            if (ceWifiEsp->callBackClientRecv[linkID] == CE_NULL)
                return;

            ceWifiEsp->callBackClientRecv[linkID](ceWifiEsp->recvData, recvDataCount);//这里的回调有问题，因该是函数指针数组的问题
        }
        else
        {
            if (ceWifiEsp->callBackServerRecv == CE_NULL)
            {
                return;
            }
            ceWifiEsp->callBackServerRecv(linkID, ceWifiEsp->recvData, recvDataCount);
        }
    }
}

/**
  * @brief  使用一个UART资源来初始化CeWifiEsp模块
  * @param  ceWifiEsp:CeWifiEsp属性对象
  * @param  ceUart:此模块使用到的Uart资源接口
  * @param  ceWifiEspMode:此模块的工作方式，即AP或STA方式
  *         以AP模式（模块做为热点，其它设备连接模块）进行模块配置
  *         以STA（模块做为从设备，连接其它已存在的热点，如家用路由器、手机上建立的热点等）进行模块配置
  * @return 系统状态码，可能的值：CE_STATUS_SUCCESS,CE_STATUS_OUT_TIME
  */
CE_STATUS ceWifiEsp_initial(CeWifiEsp* ceWifiEsp, CE_RESOURCE ceUart)
{
   
    ceWifiEsp->isLockRecvBuf = 0x00;
    ceWifiEsp->isInUTSend = 0x00;
    ceWifiEsp->isServerMode = 0x00;
    ceWifiEsp->linkInfoCount = 0;
    ceWifiEsp->apLinkDeviceCount = 0;
    ceWifiEsp->staCanConnectwifiCount = 0;
    ceWifiEsp->callBackServerRecv = CE_NULL;
    ceWifiEsp->callBackClientRecv[0] = CE_NULL;
    ceWifiEsp->callBackClientRecv[1] = CE_NULL;
    ceWifiEsp->callBackClientRecv[2] = CE_NULL;
    ceWifiEsp->callBackClientRecv[3] = CE_NULL;
    ceWifiEsp->callBackClientRecv[4] = CE_NULL;

    ceWifiEsp->ceUart.ceResource = ceUart;
    ceWifiEsp->ceUart.uartBaudRate = CE_UART_BAUD_RATE_115200;
    ceWifiEsp->ceUart.uartParity = CE_UART_PARITY_NO;
    ceWifiEsp->ceUart.uartStopBits = CE_UART_STOP_BITS_1;
    ceWifiEsp->ceUart.uartWordLength = CE_UART_WORD_LENGTH_8B;
    ceWifiEsp->ceUart.recvBufSize = CE_WIFI_ESP_RECV_BUF_SIZE;
    ceWifiEsp->ceUart.recvBuf = ceWifiEsp->uartRecvBuf;
    ceWifiEsp->ceUart.pAddPar = ceWifiEsp;
    ceUartOp.initial(&(ceWifiEsp->ceUart));
    ceUartOp.start(&(ceWifiEsp->ceUart));

    ceWifiEsp->ceTask.ID = ceUart;
    ceWifiEsp->ceTask.callBack = ceWifiEsp_taskCallBack;
    ceWifiEsp->ceTask.pAddPar = ceWifiEsp;
#ifdef CE_USE_RTOS
    ceWifiEsp->ceTask.isNewThread = 0x01;
    ceWifiEsp->ceTaskPriority = CE_TASK_PRIORITY_H;
#endif
    ceTaskOp.registerTask(&(ceWifiEsp->ceTask));
    ceTaskOp.start(&(ceWifiEsp->ceTask));

    return CE_STATUS_SUCCESS;
}


CE_STATUS   ceWifiEsp_setWorkMode(CeWifiEsp* ceWifiEsp,CE_WIFI_ESP_MODE ceWifiEspMode)
{
    char sendBuf[128];
    CE_STATUS ceStatus;

    ceStatus = ceWifiEsp_sendDataAndCheck(ceWifiEsp, "AT\r\n", "OK", 2000);       //AT+RST
#ifdef __CE_CHECK_PAR__
    ceDebugOp.printf("Run CMD:%s\nRun result:%s\n\n", "AT",ceSystemOp.getErrorMsg(ceStatus));
#endif
    if (ceStatus != CE_STATUS_SUCCESS)
    {
        ceWifiEspOp.stopUTSendOnClient(ceWifiEsp);//如果发送AT指令没有回应，则可能是当前正处于透传模式下，故先尝试退出透传模式
        ceStatus = ceWifiEsp_sendDataAndCheck(ceWifiEsp, "AT\r\n", "OK", 2000);       //AT+RST
        if(ceStatus != CE_STATUS_SUCCESS)
            return ceStatus;
    }

    ceDebugOp.sprintf(sendBuf, "AT+CWMODE=%d\r\n", ceWifiEspMode);                     //AT+CWMODE=? 工作方式，1：STA； 2：AP； 3：STA+AP
    ceStatus = ceWifiEsp_sendDataAndCheck(ceWifiEsp, sendBuf, "OK", 2000);
#ifdef __CE_CHECK_PAR__
    ceDebugOp.printf("Run CMD:%sRun result:%s\n\n", sendBuf, ceSystemOp.getErrorMsg(ceStatus));
#endif
    if (ceStatus != CE_STATUS_SUCCESS)
    {
        return ceStatus;
    }

    ceStatus = ceWifiEsp_sendDataAndCheck(ceWifiEsp, "AT+RST\r\n", "ready", 8000);       //AT+RST
#ifdef __CE_CHECK_PAR__
    ceDebugOp.printf("Run CMD:%s\nRun result:%s\n\n", "AT+RST",ceSystemOp.getErrorMsg(ceStatus));
#endif
    if (ceStatus != CE_STATUS_SUCCESS)
    {
        return ceStatus;
    }

    ceDebugOp.sprintf(sendBuf, "AT+CIPMUX=%d\r\n", CE_WIFI_ESP_CONNECT_MUL);           //AT+CIPMUX=? 是否多连接0：单连接； 1：多连接
    ceStatus = ceWifiEsp_sendDataAndCheck(ceWifiEsp, sendBuf, "OK", 2000);
#ifdef __CE_CHECK_PAR__
    ceDebugOp.printf("Run CMD:%sRun result:%s\n\n", sendBuf, ceSystemOp.getErrorMsg(ceStatus));
#endif
    if (ceStatus != CE_STATUS_SUCCESS)
    {
        return ceStatus;
    }
    return ceStatus;
}


/**
  * @brief  创建一个AP热点
  * @param  ceWifiEsp:CeWifiEsp属性对象
  * @param  ip:此AP热点的IP地址，其它连接到此模块的设备在IP在此基础上增加
  * @param  ssid:此AP热点的Wifi名称
  * @param  channelNumber:Wifi工作信道，范围1－13，初始化时给范围内任意值即可。
  * @param  ceWifiEspEcn:此AP热点创建的Wifi加密方式，建议CE_WIFI_ESP_ECN_WPA2_PSK
  * @return 系统状态码，可能的值：CE_STATUS_SUCCESS,CE_STATUS_OUT_TIME
  */
CE_STATUS ceWifiEsp_createWifi(CeWifiEsp* ceWifiEsp, const char* ip, const char* ssid, const char* passWord,uint8 channelNumber,CE_WIFI_ESP_ECN ceWifiEspEcn)
{
    char sendBuf[128];
    CE_STATUS ceStatus;

    ceDebugOp.sprintf(sendBuf, "AT+CWSAP=\"%s\",\"%s\",%d,%d\r\n", ssid, passWord, channelNumber, ceWifiEspEcn);
    ceStatus = ceWifiEsp_sendDataAndCheck(ceWifiEsp, sendBuf, "OK", 2000);
#ifdef __CE_CHECK_PAR__
    ceDebugOp.printf("Run CMD:%sRun result:%s\n\n", sendBuf, ceSystemOp.getErrorMsg(ceStatus));
#endif
    if (ceStatus != CE_STATUS_SUCCESS)
    {
        return ceStatus;
    }

    ceDebugOp.sprintf(sendBuf, "AT+CIPAP=\"%s\"\r\n", ip);
    ceStatus = ceWifiEsp_sendDataAndCheck(ceWifiEsp, sendBuf, "OK", 2000);
#ifdef __CE_CHECK_PAR__
    ceDebugOp.printf("Run CMD:%sRun result:%s\n\n", sendBuf, ceSystemOp.getErrorMsg(ceStatus));
#endif
    if (ceStatus != CE_STATUS_SUCCESS)
    {
        return ceStatus;
    }

    return ceStatus;
}

/**
  * @brief  AP工作方式下，获得与此AP据点连接的从设备的IP与MAC地址例表
  * @param  ceWifiEsp:CeWifiEsp属性对象
  * @return 从设备信息例表
  */
CeWifiEspAPLinkDevInfo* ceWifiEsp_getConnectedDeviceList(CeWifiEsp* ceWifiEsp)
{
    uint8 dataBuf[30];
    uint8 i;
    uint16 recvDataCount;
    CE_STATUS ceStatus;

    for (i = 0; i < CE_WIFI_ESP_MAX_CONNECT_BUF; i++)
    {
        ceWifiEsp->apLinkDevInfoList[i].ip[0] = '\0';
        ceWifiEsp->apLinkDevInfoList[i].mac[0] = '\0';
    }

#ifdef __CE_CHECK_PAR__
    ceDebugOp.printf("CeWifiEsp try to find connect device...\n");
#endif

    ceStatus = ceWifiEsp_sendDataAndCheck(ceWifiEsp, "AT+CWLIF\r\n", "AT+CWLIF\r\r\n", 1000);
#ifdef __CE_CHECK_PAR__
    ceDebugOp.printf("Run CMD:%sRun result:%s\n\n", "AT+CWLIF\r\n", ceSystemOp.getErrorMsg(ceStatus));
#endif
    if (ceStatus != CE_STATUS_SUCCESS)
    {
        ceWifiEsp->apLinkDeviceCount = 0;
        return ceWifiEsp->apLinkDevInfoList;
    }

    for (i = 0; i < CE_WIFI_ESP_MAX_CONNECT_BUF; i++)
    {
        recvDataCount = ceWifiEsp_readStringByEndChar(ceWifiEsp, dataBuf, 30, ",",2000);//读取ip
        if (recvDataCount == 0 || recvDataCount > 16)
        {
            ceWifiEsp->apLinkDeviceCount = i;
            return  ceWifiEsp->apLinkDevInfoList;
        }
        ceWifiEsp_cpData((uint8*)(ceWifiEsp->apLinkDevInfoList[i].ip), dataBuf, recvDataCount);

        recvDataCount = ceWifiEsp_readStringByEndChar(ceWifiEsp, dataBuf, 30, "\r\n", 2000);//读取MAC地址
        if (recvDataCount == 0 || recvDataCount > 19)
        {
            ceWifiEsp->apLinkDevInfoList[i].ip[0] = '\0';
            ceWifiEsp->apLinkDeviceCount = i;
            return  ceWifiEsp->apLinkDevInfoList;
        }
        ceWifiEsp_cpData((uint8*)(ceWifiEsp->apLinkDevInfoList[i].mac), dataBuf, recvDataCount);

#ifdef __CE_CHECK_PAR__
        ceDebugOp.printf("CeWifiEsp find connect device on AP Mode:\nIP:%s    MAC:%s\n", ceWifiEsp->apLinkDevInfoList[i].ip, ceWifiEsp->apLinkDevInfoList[i].mac);
#endif
    }
    return ceWifiEsp->apLinkDevInfoList;
}

/**
  * @brief  AP工作方式下，获得与此AP据点连接的从设备的数量，可用于判断有无设备连接到此Wifi之上
  * @param  ceWifiEsp:CeWifiEsp属性对象
  * @return 从设备的数量
  */
uint8 ceWifiEsp_getConnectedDeviceCount(CeWifiEsp* ceWifiEsp)
{
    ceWifiEsp_getConnectedDeviceList(ceWifiEsp);
#ifdef __CE_CHECK_PAR__
    ceDebugOp.printf("CeWifiEsp find connect device count: %d\n", ceWifiEsp->apLinkDeviceCount);
#endif
    return ceWifiEsp->apLinkDeviceCount;
}

/**
  * @brief  STA工作方式下，查找周围环境中可连接的Wifi信号
  * @param  ceWifiEsp:CeWifiEsp属性对象
  * @return 可用Wifi的例表指针
  */
CeWifiEspCanConnectWifiInfo*  ceWifiEsp_getCanConnectWifiList(CeWifiEsp* ceWifiEsp)
{
    uint16 i;
    uint8 dataBuf[50];
    uint16 recvDataCount;
    CE_STATUS ceStatus;
    for (i = 0; i < CE_WIFI_ESP_MAX_CONNECT_BUF; i++)//先清空例表中的内容
    {
        ceWifiEsp->staCanConnectWifiList[i].ssid[0] = '\0';
        ceWifiEsp->staCanConnectWifiList[i].mac[0] = '\0';
    }

    ceStatus = ceWifiEsp_sendDataAndCheck(ceWifiEsp, "AT+CWLAP\r\n", "AT+CWLAP", 2000);
#ifdef __CE_CHECK_PAR__
    ceDebugOp.printf("Run CMD:%sRun result:%s\n\n", "AT+CWLAP\r\n", ceSystemOp.getErrorMsg(ceStatus));
#endif
    if (ceStatus != CE_STATUS_SUCCESS)
    {
        ceWifiEsp->staCanConnectwifiCount = 0;
        return ceWifiEsp->staCanConnectWifiList;
    }

    for (i = 0; i < CE_WIFI_ESP_MAX_CONNECT_BUF; i++)//
    {
        recvDataCount = ceWifiEsp_readStringByEndChar(ceWifiEsp, dataBuf, 50, "+CWLAP:(", 8000);//查找可用wifi最花时间，这里多设置些超时时间
        if (recvDataCount <= 0)
        {
            ceWifiEsp->staCanConnectwifiCount = i;
            return  ceWifiEsp->staCanConnectWifiList;
        }

        recvDataCount = ceWifiEsp_readStringByEndChar(ceWifiEsp, dataBuf, 50, ",\"", 2000);//ecn
        if (recvDataCount != 2 || dataBuf[0] - 0x30 > 5)
        {
            ceWifiEsp->staCanConnectwifiCount = i;
            return  ceWifiEsp->staCanConnectWifiList;
        }
        ceWifiEsp->staCanConnectWifiList[i].ceWifiEpsEcn = (CE_WIFI_ESP_ECN)(dataBuf[0] - 0x30);

        recvDataCount = ceWifiEsp_readStringByEndChar(ceWifiEsp, dataBuf, 50, "\",", 2000);//ssid
        if (recvDataCount <= 1)
        {
            ceWifiEsp->staCanConnectwifiCount = i;
            return  ceWifiEsp->staCanConnectWifiList;
        }
        ceWifiEsp_cpData((uint8*)(ceWifiEsp->staCanConnectWifiList[i].ssid), dataBuf, recvDataCount);

        recvDataCount = ceWifiEsp_readStringByEndChar(ceWifiEsp, dataBuf, 50, ",\"", 2000);//signal
        if (recvDataCount <= 1 || recvDataCount >= 6)
        {
            ceWifiEsp->staCanConnectwifiCount = i;
            return  ceWifiEsp->staCanConnectWifiList;
        }
        ceWifiEsp->staCanConnectWifiList[i].signal = ceStringOp.atoi((char*)dataBuf);

        recvDataCount = ceWifiEsp_readStringByEndChar(ceWifiEsp, dataBuf, 50, "\",", 2000);//MAC
        if (recvDataCount != 18)
        {
            ceWifiEsp->staCanConnectwifiCount = i;
            return  ceWifiEsp->staCanConnectWifiList;
        }
        ceWifiEsp_cpData((uint8*)(ceWifiEsp->staCanConnectWifiList[i].mac), dataBuf, recvDataCount);

        recvDataCount = ceWifiEsp_readStringByEndChar(ceWifiEsp, dataBuf, 50, ",", 2000);//other1
        if (recvDataCount <= 0 || recvDataCount > 6)
        {
            ceWifiEsp->staCanConnectwifiCount = i;
            return  ceWifiEsp->staCanConnectWifiList;
        }
        ceWifiEsp->staCanConnectWifiList[i].other1 = ceStringOp.atoi((char*)dataBuf);

        recvDataCount = ceWifiEsp_readStringByEndChar(ceWifiEsp, dataBuf, 50, ")", 2000);//other2
        if (recvDataCount <= 0 || recvDataCount > 6)
        {
            ceWifiEsp->staCanConnectwifiCount = i;
            return  ceWifiEsp->staCanConnectWifiList;
        }
        ceWifiEsp->staCanConnectWifiList[i].other2 = ceStringOp.atoi((char*)dataBuf);

#ifdef __CE_CHECK_PAR__
        ceDebugOp.printf("CeWifiEsp find can connect wifi on STA Mode:\n ECN:%d, SSID:%s, Signal:%d, MAC:%s, O1:%d, O2:%d\n",
            ceWifiEsp->staCanConnectWifiList[i].ceWifiEpsEcn, ceWifiEsp->staCanConnectWifiList[i].ssid,
            ceWifiEsp->staCanConnectWifiList[i].signal,ceWifiEsp->staCanConnectWifiList[i].mac,
            ceWifiEsp->staCanConnectWifiList[i].other1, ceWifiEsp->staCanConnectWifiList[i].other2);
#endif
    }
    return ceWifiEsp->staCanConnectWifiList;
}

/**
  * @brief  STA工作方式下，查找周围环境中可连接的Wifi信号数量
  * @param  ceWifiEsp:CeWifiEsp属性对象
  * @return 可用Wifi的数量
  */
uint8  ceWifiEsp_getCanConnectWifiCount(CeWifiEsp* ceWifiEsp)
{
    ceWifiEsp_getCanConnectWifiList(ceWifiEsp);
#ifdef __CE_CHECK_PAR__
    ceDebugOp.printf("CeWifiEsp find can connect wifi count: %d\n", ceWifiEsp->staCanConnectwifiCount);
#endif
    return ceWifiEsp->staCanConnectwifiCount;
}

/**
  * @brief  STA工作方式下，查找周围环境中是否有给出的ssid的Wifi信号
  * @param  ceWifiEsp:CeWifiEsp属性对象
  * @return 系统状态码，可能的值：CE_STATUS_SUCCESS,CE_STATUS_OUT_TIME
  */
CE_STATUS ceWifiEsp_checkCanConnectSsidIsExist(CeWifiEsp* ceWifiEsp, const char* ssid)
{
    uint8  i;
    CeWifiEspCanConnectWifiInfo* canConnectWifiInfo = ceWifiEsp_getCanConnectWifiList(ceWifiEsp);
    if (ceWifiEsp->staCanConnectwifiCount == 0)
    {
#ifdef __CE_CHECK_PAR__
        ceDebugOp.printf("CeWifiEsp cannot find wifi by name: %s\n", ssid);
#endif
        return CE_STATUS_FAILE;
    }
    for (i = 0; i < CE_WIFI_ESP_MAX_CONNECT_BUF; i++)
    {
        if (ceStringOp.strcmp(ceWifiEsp->staCanConnectWifiList[i].ssid, ssid) == 0)
        {
#ifdef __CE_CHECK_PAR__
            ceDebugOp.printf("CeWifiEsp has find wifi by name: %s\n ECN:%d, SSID:%s, Signal:%d, MAC:%s, O1:%d, O2:%d\n", ssid,
                ceWifiEsp->staCanConnectWifiList[i].ceWifiEpsEcn, ceWifiEsp->staCanConnectWifiList[i].ssid,
                ceWifiEsp->staCanConnectWifiList[i].signal, ceWifiEsp->staCanConnectWifiList[i].mac,
                ceWifiEsp->staCanConnectWifiList[i].other1, ceWifiEsp->staCanConnectWifiList[i].other2);
#endif
            return CE_STATUS_SUCCESS;
        }
    }
#ifdef __CE_CHECK_PAR__
    ceDebugOp.printf("CeWifiEsp cannot find wifi by name: %s\n", ssid);
#endif
    return CE_STATUS_FAILE;
}

/**
  * @brief  STA工作方式，连接一个已经存在的热点，参数为热点的SSID与PWD
  * @param  ceWifiEsp:CeWifiEsp属性对象
  * @param  ssid:需要连接的Wifi名称
  * @param  passWord:Wifi密码
  * @return 系统状态码，可能的值：CE_STATUS_SUCCESS,CE_STATUS_OUT_TIME
  */
CE_STATUS ceWifiEsp_connectWifi(CeWifiEsp* ceWifiEsp, const char* ssid, const char* passWord)
{
    CE_STATUS ceStatus;
    char sendBuf[100];

    ceDebugOp.sprintf(sendBuf, "AT+CWJAP=\"%s\",\"%s\"\r\n", ssid, passWord);
    ceStatus = ceWifiEsp_sendDataAndCheck(ceWifiEsp, sendBuf, "OK", 12000);
#ifdef __CE_CHECK_PAR__
    ceDebugOp.printf("Run CMD:%sRun result:%s\n\n", sendBuf, ceSystemOp.getErrorMsg(ceStatus));
#endif

#ifdef CE_WIFI_ESP_UT
    ceDebugOp.sprintf(sendBuf, "AT+CIPMODE=%d\r\n", 1);
    ceStatus = ceWifiEsp_sendDataAndCheck(ceWifiEsp, sendBuf, "OK", 2000);
#ifdef __CE_CHECK_PAR__
    ceDebugOp.printf("Run CMD:%s Run result:%s\n\n", sendBuf, ceSystemOp.getErrorMsg(ceStatus));
#endif

#endif

    return ceStatus;
}

/**
  * @brief  STA工作方式下，连接到热点后的IP，注意此时配置的IP应与热点分配的ID相匹配。
  * @param  ceWifiEsp:CeWifiEsp属性对象
  * @param  ip:需要设定的IP地址
  * @return 系统状态码，可能的值：CE_STATUS_SUCCESS,CE_STATUS_OUT_TIME
  */
CE_STATUS ceWifiEsp_setStaIp(CeWifiEsp* ceWifiEsp, const char* ip)
{
    char sendBuf[64];
    CE_STATUS ceStatus;
    ceDebugOp.sprintf(sendBuf, "AT+CIPAP=\"%s\"\r\n", ip);
    ceStatus = ceWifiEsp_sendDataAndCheck(ceWifiEsp, sendBuf, "OK", 2000);
#ifdef __CE_CHECK_PAR__
    ceDebugOp.printf("Run CMD:%sRun result:%s\n\n", sendBuf, ceSystemOp.getErrorMsg(ceStatus));
#endif
    return ceStatus;
}

/**
  * @brief  创建一个服务器，用来接收与发送数据，AP与STA模式均可创建
  * @param  ceWifiEsp:CeWifiEsp属性对象
  * @param  serVerPortNum:需创建的服务器端口号
  * @param  linkID:由于是创建了服务器，则此服务器最大只允许5个客户端与其连接，linkID指的就是这5个客户端的编号，最先连到服务器的客户端为0。回调中LinkID表示服务器接收到了数据，此数据是哪个客户端发送来的
  * @param  callBackServerRecv:服务器接收到数据后的回调
  * @return 系统状态码，可能的值：CE_STATUS_SUCCESS,CE_STATUS_OUT_TIME
  */
CE_STATUS    ceWifiEsp_createServer(CeWifiEsp* ceWifiEsp,uint16 serverPortNum, CE_WIFI_ESP_SOCKET_MODE ceWifiEspSocketMode, void (*callBackServerRecv)(uint8 linkID,uint8* recvBuf, uint16 recvCount))
{
    char sendBuf[30];
    CE_STATUS ceStatus;
    ceWifiEsp->isServerMode = 0x01;
    ceWifiEsp->callBackServerRecv = callBackServerRecv;
    
      ceDebugOp.sprintf(sendBuf, "AT+CIPMUX=%d\r\n", 1);           //AT+CIPMUX=? 是否多连接0：单连接； 1：多连接
    ceStatus = ceWifiEsp_sendDataAndCheck(ceWifiEsp, sendBuf, "OK", 2000);
#ifdef __CE_CHECK_PAR__
    ceDebugOp.printf("Run CMD:%sRun result:%s\n\n", sendBuf, ceSystemOp.getErrorMsg(ceStatus));
#endif
    if (ceStatus != CE_STATUS_SUCCESS)
    {
        return ceStatus;
    }
    
    
    ceDebugOp.sprintf(sendBuf, "AT+CIPSERVER=%d,%d\r\n", 1, serverPortNum);
    ceStatus = ceWifiEsp_sendDataAndCheck(ceWifiEsp, sendBuf, "OK", 2000);
#ifdef __CE_CHECK_PAR__
    ceDebugOp.printf("Run CMD:%sRun result:%s\n\n", sendBuf, ceSystemOp.getErrorMsg(ceStatus));
#endif
    return ceStatus;
}

/**
  * @brief  连接一个已存在的服务器，用来接收与发送数据，AP与STA模式均可创建
  * @param  ceWifiEsp:CeWifiEsp属性对象
  * @param  linkID:ESP8266可以创建5个客户端（编号0－4）分别连接各自的服务器，LinkID指使用哪个客户端编号号连接服务器
  * @param  serverIP:需要连接的服务器的IP
  * @param  serVerPortNum:需要连接的服务器的端口
  * @param ceWifiEspSocketMode:连接方式，TCP或UDP
  * @param callBackClientRecv:此客户端接收到数据后的回调
  * @return 系统状态码，可能的值：CE_STATUS_SUCCESS,CE_STATUS_OUT_TIME
  */
CE_STATUS ceWifiEsp_connectServer(CeWifiEsp* ceWifiEsp,uint8 linkID, const char* serverIP,uint16 serVerPortNum,CE_WIFI_ESP_SOCKET_MODE ceWifiEspSocketMode, void (*callBackServerRecv)(uint8* recvBuf, uint16 recvCount))
{

    char sendBuf[50];
    CE_STATUS ceStatus;    
    
    ceWifiEsp->isServerMode = 0x00;
    ceWifiEsp->callBackClientRecv[linkID] = callBackServerRecv;

    if (CE_WIFI_ESP_CONNECT_MUL == 0)
        ceDebugOp.sprintf(sendBuf, "AT+CIPSTART=\"%s\",\"%s\",%d\r\n", ((ceWifiEspSocketMode == CE_WIFI_ESP_SOCKET_MODE_TCP) ? "TCP" : "UDP"), serverIP, serVerPortNum);
    else
        ceDebugOp.sprintf(sendBuf, "AT+CIPSTART=%d,\"%s\",\"%s\",%d\r\n", linkID, ((ceWifiEspSocketMode == CE_WIFI_ESP_SOCKET_MODE_TCP) ? "TCP" : "UDP"), serverIP, serVerPortNum);

    ceStatus = ceWifiEsp_sendDataAndCheck(ceWifiEsp, sendBuf, "OK", 2000);

#ifdef __CE_CHECK_PAR__
    ceDebugOp.printf("Run CMD:%sRun result:%s\n\n", sendBuf, ceSystemOp.getErrorMsg(ceStatus));
#endif
    return ceStatus;
}

/**
  * @brief  获得本模块所建立的连接列表，包含服务器模式下，与此服务器连接的客户端数量；客户端模式下，有多个少客户端正在建立并且与服务端连接（最大可5个）
  * @param  ceWifiEsp:CeWifiEsp属性对象
  * @return SOCKET连接例表
  */
CeWifiEspLinkInfo* ceWifiEsp_getLinkList(CeWifiEsp* ceWifiEsp)
{
    uint16 i;
    uint8 dataBuf[30];
    uint16 recvDataCount;
    CE_STATUS ceStatus;
    for (i = 0; i < CE_WIFI_ESP_MAX_CONNECT_BUF; i++)
    {
        ceWifiEsp->linkInfoList[i].ip[0] = '\0';
    }

    ceStatus = ceWifiEsp_sendDataAndCheck(ceWifiEsp, "AT+CIPSTATUS\r\n", "+CIPSTATUS:", 2000);
#ifdef __CE_CHECK_PAR__
    ceDebugOp.printf("Run CMD:%sRun result:%s\n\n", "AT+CIPSTATUS\r\n", ceSystemOp.getErrorMsg(ceStatus));
#endif
    if (ceStatus != CE_STATUS_SUCCESS)
    {
#ifdef __CE_CHECK_PAR__
        ceDebugOp.printf("CeWifiEsp cannot find link on server or client mode.\n");
#endif
        ceWifiEsp->linkInfoCount = 0;
        return ceWifiEsp->linkInfoList;
    }
    for (i = 0; i < CE_WIFI_ESP_MAX_CONNECT_BUF; i++)
    {
        recvDataCount = ceWifiEsp_readStringByEndChar(ceWifiEsp, dataBuf, 30, ",\"", 2000);//连接号，0－4
        if (recvDataCount == 0 || recvDataCount > 2 || (dataBuf[0] - 0x30) > 4)
        {
            ceWifiEsp->linkInfoCount = i;
            return  ceWifiEsp->linkInfoList;
        }
        ceWifiEsp->linkInfoList[i].linkID = (dataBuf[0] - 0x30);


        recvDataCount = ceWifiEsp_readStringByEndChar(ceWifiEsp, dataBuf, 30, "\",\"", 2000);//读取连接方式，TCP或UDP
        if (recvDataCount == 0 || recvDataCount > 4)
        {
            ceWifiEsp->linkInfoCount = i;
            return  ceWifiEsp->linkInfoList;
        }
        if (dataBuf[0] == 'T' && dataBuf[1] == 'C' && dataBuf[2] == 'P')
        {
            ceWifiEsp->linkInfoList[i].socketMode = CE_WIFI_ESP_SOCKET_MODE_TCP;
        }
        else if (dataBuf[0] == 'U' && dataBuf[1] == 'D' && dataBuf[2] == 'P')
        {
            ceWifiEsp->linkInfoList[i].socketMode = CE_WIFI_ESP_SOCKET_MODE_UDP;
        }
        else
        {
            ceWifiEsp->linkInfoList[i].ip[0] = '\0';
            ceWifiEsp->linkInfoCount = i;
            return  ceWifiEsp->linkInfoList;
        }

        recvDataCount = ceWifiEsp_readStringByEndChar(ceWifiEsp, dataBuf, 30, "\",", 2000);//读取IP
        if (recvDataCount <7 || recvDataCount > 20)
        {
            ceWifiEsp->linkInfoList[i].ip[0] = '\0';
            ceWifiEsp->linkInfoCount = i;
            return  ceWifiEsp->linkInfoList;
        }
        ceWifiEsp_cpData((uint8*)(ceWifiEsp->linkInfoList[i].ip), dataBuf, recvDataCount);

        recvDataCount = ceWifiEsp_readStringByEndChar(ceWifiEsp, dataBuf, 30, ",", 2000);//读取端口号
        if (recvDataCount == 0 || recvDataCount > 6)
        {
            ceWifiEsp->linkInfoList[i].ip[0] = '\0';
            ceWifiEsp->linkInfoCount = i;
            return  ceWifiEsp->linkInfoList;
        }
        ceWifiEsp->linkInfoList[i].port = (uint16)(ceStringOp.atoi((char*)dataBuf));


        recvDataCount = ceWifiEsp_readStringByEndChar(ceWifiEsp, dataBuf, 30, "\r\n", 2000);//读取此模块是做为服务器，还是客户端连接，1做为服务器，0做为客户端
        if (recvDataCount == 0 || recvDataCount > 2)
        {
            ceWifiEsp->linkInfoList[i].ip[0] = '\0';
            ceWifiEsp->linkInfoCount = i;
            return  ceWifiEsp->linkInfoList;
        }
        ceWifiEsp->linkInfoList[i].moduleLinkType = (uint16)(ceStringOp.atoi((char*)dataBuf));

#ifdef __CE_CHECK_PAR__
        ceDebugOp.printf("CeWifiEsp find one link:\n LinkID:%d, Socket mode:%s, IP:%s, Port:%d, LinkProperty:Module as %s\n",
            ceWifiEsp->linkInfoList[i].linkID, ((ceWifiEsp->linkInfoList[i].socketMode == CE_WIFI_ESP_SOCKET_MODE_TCP)?"TCP":"UDP"),
            ceWifiEsp->linkInfoList[i].ip, ceWifiEsp->linkInfoList[i].port,((ceWifiEsp->linkInfoList[i].moduleLinkType == 1)?"Server":"Client"));
#endif
    }
    return ceWifiEsp->linkInfoList;
}

/**
  * @brief  获得本模块所建立的连接数量，可以用来判断：服务器工作方式下，有无客户端连接。客户端方式下，是否正常连接到服务器。
  * @param  ceWifiEsp:CeWifiEsp属性对象
  * @return SOCKET连接数量
  */
uint8 ceWifiEsp_getLinkCount(CeWifiEsp* ceWifiEsp)
{
    ceWifiEsp_getLinkList(ceWifiEsp);
#ifdef __CE_CHECK_PAR__
    ceDebugOp.printf("CeWifiEsp find link count: %d\n", ceWifiEsp->linkInfoCount);
#endif
    return ceWifiEsp->linkInfoCount;
}

/**
  * @brief  启动透传，只有模块是单连接，且为客户端才可以有此功能
  * @param  ceWifiEsp:CeWifiEsp属性对象
  * @return 系统状态码，可能的值：CE_STATUS_SUCCESS,CE_STATUS_OUT_TIME
  */
CE_STATUS ceWifiEsp_startUTSendOnClient(CeWifiEsp* ceWifiEsp)
{
    CE_STATUS ceStatus;
    ceStatus = ceWifiEsp_sendDataAndCheck(ceWifiEsp, "AT+CIPSEND\r\n", ">",8000);
#ifdef __CE_CHECK_PAR__
    ceDebugOp.printf("Run CMD:%sRun result:%s\n\n", "AT+CIPSEND\r\n", ceSystemOp.getErrorMsg(ceStatus));
#endif
    ceWifiEsp->isInUTSend = 0x01;
    return ceStatus;
}

/**
  * @brief  停止透传，只有模块是单连接，且为客户端才可以有此功能
  * @param  ceWifiEsp:CeWifiEsp属性对象
  * @return 系统状态码，可能的值：CE_STATUS_SUCCESS,CE_STATUS_OUT_TIME
  */
CE_STATUS ceWifiEsp_stopUTSendOnClient(CeWifiEsp* ceWifiEsp)
{
    ceWifiEsp_sendDataAndCheck(ceWifiEsp, "+++", CE_NULL,8000);
#ifdef __CE_CHECK_PAR__
    ceDebugOp.printf("Run CMD:%s\n\n", "+++");
#endif
    ceWifiEsp->isInUTSend = 0x00;
    return CE_STATUS_SUCCESS;
}

/**
  * @brief  向一个连接号linkID发送数据。
  * @param  ceWifiEsp:CeWifiEsp属性对象
  * @param  linkID:服务器模式下，此连接号指的是与其连接的客户端序号；客户端模式下，此连接号指使用哪个客户端（编号0－4）
  * @param  dataBuf:需要发送的数据缓存
  * @param  dataSize:需要发送的数据长度
  * @return 系统状态码，可能的值：CE_STATUS_SUCCESS,CE_STATUS_OUT_TIME
  */
CE_STATUS ceWifiEsp_sendData(CeWifiEsp* ceWifiEsp, uint8 linkID, uint8* dataBuf, uint16 dataSize)
{
    if (dataSize <= 1400)//socket 一次最大传送1480个字节，这里给1400个字节
    {
        if (ceWifiEsp_atCIpSend(ceWifiEsp, linkID, dataSize) != CE_STATUS_SUCCESS)
        {
            return CE_STATUS_FAILE;
        }
        ceWifiEsp_sendUartData(ceWifiEsp, dataBuf, dataSize);
        if (ceWifiEsp->isInUTSend)
            return CE_STATUS_SUCCESS;
        else
            return ceWifiEsp_sendDataAndCheck(ceWifiEsp, CE_NULL, "OK",1000);
    }
    else
    {
        uint8 loop = dataSize / 1400;
        uint16 lastData = dataSize % 1400;
        uint8 i;
        for (i = 0; i < loop; i++)
        {
            if (ceWifiEsp_atCIpSend(ceWifiEsp, linkID, 1400) != CE_STATUS_SUCCESS)
            {
                return CE_STATUS_FAILE;
            }
            ceWifiEsp_sendUartData(ceWifiEsp, (uint8*)(dataBuf + i * 1400), dataSize);
            if (CE_STATUS_SUCCESS != ceWifiEsp_sendDataAndCheck(ceWifiEsp, CE_NULL, "OK",1000))
            {
                return CE_STATUS_FAILE;
            }
        }
        if (lastData != 0)
        {
            if (ceWifiEsp_atCIpSend(ceWifiEsp, linkID, lastData) != CE_STATUS_SUCCESS)
            {
                return CE_STATUS_FAILE;
            }
            ceWifiEsp_sendUartData(ceWifiEsp,  (uint8*)(dataBuf + loop * 1400), lastData);
            if (CE_STATUS_SUCCESS != ceWifiEsp_sendDataAndCheck(ceWifiEsp, CE_NULL, "OK",1000))
            {
                return CE_STATUS_FAILE;
            }
        }
    }
    return CE_STATUS_SUCCESS;
}

const CeWifiEspOpBase ceWifiEspOp = { ceWifiEsp_initial, ceWifiEsp_setWorkMode, ceWifiEsp_createWifi, ceWifiEsp_getConnectedDeviceList, ceWifiEsp_getConnectedDeviceCount,
                                        ceWifiEsp_getCanConnectWifiList, ceWifiEsp_getCanConnectWifiCount, ceWifiEsp_checkCanConnectSsidIsExist,
                                        ceWifiEsp_connectWifi, ceWifiEsp_setStaIp, ceWifiEsp_createServer, ceWifiEsp_connectServer,
                                        ceWifiEsp_startUTSendOnClient, ceWifiEsp_stopUTSendOnClient,
                                        ceWifiEsp_getLinkList, ceWifiEsp_getLinkCount, ceWifiEsp_sendData
                                    };

#ifdef __cplusplus
 }
#endif //__cplusplus
