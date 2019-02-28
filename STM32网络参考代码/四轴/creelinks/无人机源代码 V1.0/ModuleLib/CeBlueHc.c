/**
  ******************************************************************************
  * @file    CeBlueHc.c
  * @author  Creelinks Application Team
  * @version V1.0.0
  * @date    2017-01-06
  * @brief   适用于CeBlueHc模块的驱动库文件
  ******************************************************************************
  * @attention
  *
  *1)无
  *
  * <h2><center>&copy; COPYRIGHT 2016 Darcern</center></h2>
  ******************************************************************************
  */
#include "CeBlueHc.h"
#ifdef __cplusplus
extern "C" {
#endif //__cplusplus

#define CE_BLUE_HC_BUFFER_LENGTH 256

/**
  * @brief  复制数据
  * @param  desBuf:目的缓存
  * @param  srcBuf:源缓存
  * @param  cpCount:需复制的数据长度
  * @return 系统状态码，可能的值：CE_STATUS_SUCCESS,CE_STATUS_OUT_TIME
*/
void ceBlueHc_cpData(uint8* desBuf, uint8* srcBuf, uint16 cpCount)
{
    uint16 i;
    for(i = 0; i < cpCount; i++)
    {
        desBuf[i] = srcBuf[i];
    }
    desBuf[i] = '\0';
}

/**
  * @brief  从Uart中读取数据，以endChar结尾结尾
  * @param  ceBlueHc:CeBlueHc属性对象
  * @param  buf:读出的数据需放置的缓存
  * @param  bufSize:缓存大小
  * @param  endChar:读数据，直到以此字符串结尾
  * @param  isCheckOK:是否检查"OK\0"字符串
  * @param  checkOkStatus:用于获取是否出现"OK\0"字符，0x00表示没有，0x01表示有
  * @param  outTimeMs:超时时间
  * @return 系统状态码，可能的值：CE_STATUS_SUCCESS,CE_STATUS_OUT_TIME
*/
uint16 ceBlueHc_readStringByEndChar(CeBlueHc* ceBlueHc, uint8* buf, uint16 bufSize, char* endChar, uint8 isCheckOK, uint8* checkOkStatus, uint16 outTimeMs)
{
    uint16 tickMs = 0;
    uint16 checkIndex = 0;
    uint16 bufIndex = 0;
    uint16 checkOKIndex = 0;
    char* tempOK = "OK\0";
    char temp[2];
    temp[1] = '\0';
    buf[0] = '\0';
    *checkOkStatus = 0x00;
    while(1)
    {
        if(ceUartOp.getRecvDataCount(&(ceBlueHc->ceUart)) <= 0)    //如果没有收到数
        {
            if(tickMs >= outTimeMs)
            {
                buf[0] = '\0';
                return 0;
            }
            tickMs += 10;
            ceSystemOp.delayMs(10);
        }
        else   //如果有收到数
        {
            ceUartOp.readData(&(ceBlueHc->ceUart), (uint8*)(temp), 1);//读取一个字节
            //ceDebugOp.printf(temp);//把收到的数据打出来，调试时使用
            buf[bufIndex] = temp[0];
            bufIndex++;
            if(bufIndex == bufSize) //读取到的数据超过了接收缓存的最大长度，此时应该是接收的数据有误，所有应该直接返回。
            {
                buf[0] = '\0';
                return 0;
            }
            if(isCheckOK == 0x01)
            {
                if(temp[0] == tempOK[checkOKIndex])
                {
                    checkOKIndex++;
                    if(tempOK[checkOKIndex] == '\0')
                    {
                        *checkOkStatus = 0x01;
                        if(bufIndex > 5)
                        {
                            buf[bufIndex - ceStringOp.strlen(tempOK)] = '\0';
                            return bufIndex - ceStringOp.strlen(tempOK) + 1;
                        }
                        return 0;
                    }
                }
            }
            if(temp[0] == endChar[checkIndex])         //比较读取到的字节与需要对比的内容是否相同
            {
                checkIndex++;                       //如果相同，则让比较的索引增加1
                if(endChar[checkIndex] == '\0')     //如果增加1后，即下一个要比较的字节是空，则表明比较完成，返回成功
                {
                    buf[bufIndex - ceStringOp.strlen(endChar)] = '\0';
                    return bufIndex - ceStringOp.strlen(endChar) + 1;
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
  * @param  ceBlueHc:CeBlueHc属性对象
  * @param  sendMsg:要发送出去的内容
  * @param  recvMsg:期望接收到的内容,这里只检测是否包含
  * @param  outTimeMs:超时时间
  * @return 系统状态码，可能的值：CE_STATUS_SUCCESS,CE_STATUS_OUT_TIME
*/
CE_STATUS ceBlueHc_sendDataAndCheck(CeBlueHc* ceBlueHc, char* sendMsg, char* recvMsg, uint16 outTimeMs)
{
    if(sendMsg == CE_NULL)
    {
        return CE_STATUS_FAILE;
    }
    else
    {
        uint32 tickMs = 0;
        uint16 checkIndex = 0;
        uint16 checkFIndex = 0;
        char* tempFalse = "FAIL\0";
        char temp[2];
        temp[1] = '\0';
        while(ceBlueHc->isLockRecvBuf == 0x01)
        {
            ceSystemOp.delayMs(1);
            tickMs++;
            if(tickMs >= 4000)
            {
                break;
            }
        }
        ceBlueHc->isLockRecvBuf = 0x01;
        if(sendMsg != CE_NULL)
        {
            ceUartOp.clearRecvBuf(&(ceBlueHc->ceUart));
            ceUartOp.sendData(&(ceBlueHc->ceUart), (uint8*)sendMsg, ceStringOp.strlen(sendMsg));
            ceSystemOp.delayMs(500);//要么是Fifo有问题，要么是Uart有问题，发送完了不延时，会导致收不到数。
            //ceDebugOp.printf("Recv Count:%d \n",ceUartOp.getRecvDataCount(&(ceBlueHc->ceUart)));
        }
        while(1)
        {
            if(ceUartOp.getRecvDataCount(&(ceBlueHc->ceUart)) <= 0)    //如果没有收到数
            {
                ceSystemOp.delayMs(10);
                tickMs += 10;
                if(tickMs >= outTimeMs)
                {
                    ceBlueHc->isLockRecvBuf = 0x00;
                    return CE_STATUS_OUT_TIME;
                }
            }
            else   //如果有收到数
            {
                ceUartOp.readData(&(ceBlueHc->ceUart), (uint8*)(temp), 1);//读取一个字节
                ceDebugOp.printf(temp);//把收到的数据打出来，调试时使用
                if(temp[0] == tempFalse[checkFIndex])
                {
                    checkFIndex++;
                    if(tempFalse[checkFIndex] == '\0')
                    {
                        return CE_STATUS_FAILE;
                    }
                }
                if(temp[0] == recvMsg[checkIndex])         //比较读取到的字节与需要对比的内容是否相同
                {
                    checkIndex++;                       //如果相同，则让比较的索引增加1
                    if(recvMsg[checkIndex] == '\0')     //如果增加1后，即下一个要比较的字节是空，则表明比较完成，返回成功
                    {
                        ceBlueHc->isLockRecvBuf = 0x00;
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
  * @brief  CeBlueHc模块检查波特率
  * @param  ceBlueHc:CeBlueHc属性对象
  * @return 系统状态码
  */
CE_STATUS ceBlueHc_checkUartRate(CeBlueHc* ceBlueHc)
{
    CE_STATUS ceStatus;
    ceStatus = ceBlueHc_sendDataAndCheck(ceBlueHc, "AT\r\n", "OK", 2000);//AT
#ifdef __CE_CHECK_PAR__
    ceDebugOp.printf("Run CMD:%s\nRun result:%s\n\n", "AT", ceSystemOp.getErrorMsg(ceStatus));
#endif
    return ceStatus;
}

/**
  * @brief  CeBlueHc模块初始化
  * @param  ceBlueHc:CeBlueHc属性对象
  * @param  ceXX:CeBlueHc模块使用的资源号
  * @return 系统状态码
  */
CE_STATUS ceBlueHc_initial(CeBlueHc* ceBlueHc, CE_RESOURCE ceUart, CE_RESOURCE ceGpio0,CE_RESOURCE ceGpio1,CE_RESOURCE ceGpio2)
{
    uint8 retry = 0;
    CE_STATUS ceStatus = CE_STATUS_FAILE;
    //Gpio0输出控制模块上下电，0上电1掉电；Gpio1控制AT模式还是工作模式，1为AT模式； Gpio2输入检测连接状态，1未连接，0已建立连接。
    ceBlueHc->ceGpio0.ceResource = ceGpio0;
    ceBlueHc->ceGpio0.ceGpioMode = CE_GPIO_MODE_OUT_PP;
    ceGpioOp.initial(&(ceBlueHc->ceGpio0));

    ceBlueHc->ceGpio1.ceResource = ceGpio1;
    ceBlueHc->ceGpio1.ceGpioMode = CE_GPIO_MODE_OUT_PP;
    ceGpioOp.initial(&(ceBlueHc->ceGpio1));

    ceBlueHc->ceGpio2.ceResource = ceGpio2;
    ceBlueHc->ceGpio2.ceGpioMode = CE_GPIO_MODE_IN_FLOATING;
    ceGpioOp.initial(&(ceBlueHc->ceGpio2));

    ceBlueHc->ceUart.ceResource = ceUart;
    ceBlueHc->ceUart.uartBaudRate = CE_UART_BAUD_RATE_38400;//CE_UART_BAUD_RATE_115200
    ceBlueHc->ceUart.uartParity = CE_UART_PARITY_NO;
    ceBlueHc->ceUart.uartStopBits = CE_UART_STOP_BITS_1;
    ceBlueHc->ceUart.uartWordLength = CE_UART_WORD_LENGTH_8B;
    ceBlueHc->ceUart.recvBufSize = CE_BLUE_HC_RECV_BUF_SIZE;
    ceBlueHc->ceUart.recvBuf = ceBlueHc->uartRecvBuf;
    ceBlueHc->ceUart.pAddPar = ceBlueHc;
    ceUartOp.initial(&(ceBlueHc->ceUart));
    ceUartOp.start(&(ceBlueHc->ceUart));

    ceGpioOp.setBit(&(ceBlueHc->ceGpio0));//先进入AT状态，检查模块连接是否正常
    ceGpioOp.setBit(&(ceBlueHc->ceGpio1));
    ceSystemOp.delayMs(20);
    ceGpioOp.resetBit(&(ceBlueHc->ceGpio0));

    while(ceStatus != CE_STATUS_SUCCESS)
    {
        ceStatus = ceBlueHc_checkUartRate(ceBlueHc);
        retry++;
        if (retry > 10)
        {
            break;
        }
        ceSystemOp.delayMs(100);
    }
    ceGpioOp.setBit(&(ceBlueHc->ceGpio0));//默认进入正常工作模式，仅在参数配置函数内，才进入AT模式
    ceGpioOp.resetBit(&(ceBlueHc->ceGpio1));
    ceSystemOp.delayMs(50);
    ceGpioOp.resetBit(&(ceBlueHc->ceGpio0));
    return ceStatus;
}

/**
  * @brief  CeBlueHc模块参数配置
  * @param  ceBlueHc:CeBlueHc属性对象
  * @param  ceBlueHcWorkMode:CeBlueHc模块的工作方式，即主模块和从模块
  * @param  devName:此设备的名称
  * @param  ceBlueHcDevType:当模块工作在主模式时，查找从设备时只查找此类型的；当模块工作在从模式时，为此模块的类型
  * @return 系统状态码
  */
CE_STATUS ceBlueHc_parmentConfig(CeBlueHc* ceBlueHc, CE_BLUE_HC_WORK_MODE ceBlueHcWorkMode, const char*  ceBlueHcDevType, const char* devName, const char* password)
{
    char sendBuf[64];
    CE_STATUS ceStatus;
    uint8 i=0;

    ceGpioOp.setBit(&(ceBlueHc->ceGpio1)); //模块重新上一下电，进入AT环境
    ceGpioOp.setBit(&(ceBlueHc->ceGpio0));
    ceSystemOp.delayMs(50);
    ceGpioOp.resetBit(&(ceBlueHc->ceGpio0));

    for(i=0;i<10;i++)//至少进行两次以上复位才能成功
    {
        ceDebugOp.sprintf(sendBuf, "AT+RESET\r\n");
        ceStatus = ceBlueHc_sendDataAndCheck(ceBlueHc, sendBuf, "OK", 2000);
#ifdef __CE_CHECK_PAR__
        ceDebugOp.printf("Run CMD:%sRun result:%s\n\n", sendBuf, ceSystemOp.getErrorMsg(ceStatus));
#endif
        if(ceStatus != CE_STATUS_SUCCESS) continue;
        else break;
    }



    ceStatus = ceBlueHc_sendDataAndCheck(ceBlueHc, "AT+INIT\r\n", "OK", 2000);//初始化SPP 规范库，只能进行一次初始化，多次初始化返回ERROR：17
    #ifdef __CE_CHECK_PAR__
    ceDebugOp.printf("Run CMD:%sRun result:%s\n\n", "AT+INIT\r\n", ceSystemOp.getErrorMsg(ceStatus));
    #endif
    if(ceStatus != CE_STATUS_SUCCESS) return ceStatus;


    ceDebugOp.sprintf(sendBuf, "AT+NAME=%s\r\n", devName);
    ceStatus = ceBlueHc_sendDataAndCheck(ceBlueHc, sendBuf, "OK", 3000);
    #ifdef __CE_CHECK_PAR__
    ceDebugOp.printf("Run CMD:%sRun result:%s\n\n", sendBuf, ceSystemOp.getErrorMsg(ceStatus));
    #endif
    if(ceStatus != CE_STATUS_SUCCESS) return ceStatus;

    /*
    ceDebugOp.sprintf(sendBuf, "AT+RMAAD\r\n");//从蓝牙配对列表中删除所有认证设备
    retry = 0;
    do
    {
        ceStatus = ceBlueHc_sendDataAndCheck(ceBlueHc, sendBuf, "OK", 3000);
#ifdef __CE_CHECK_PAR__
        ceDebugOp.printf("Run CMD:%sRun result:%s\n\n", sendBuf, ceSystemOp.getErrorMsg(ceStatus));
#endif
        retry++;
    }
    while(ceStatus != CE_STATUS_SUCCESS && retry < 10);
    */


    ceDebugOp.sprintf(sendBuf, "AT+ROLE=%d\r\n", ceBlueHcWorkMode);
    ceStatus = ceBlueHc_sendDataAndCheck(ceBlueHc, sendBuf, "OK", 2000);
    #ifdef __CE_CHECK_PAR__
    ceDebugOp.printf("Run CMD:%sRun result:%s\n\n", sendBuf, ceSystemOp.getErrorMsg(ceStatus));
    #endif
    if(ceStatus != CE_STATUS_SUCCESS) return ceStatus;

    ceDebugOp.sprintf(sendBuf, "AT+PSWD=%s\r\n", password);//设置/查询-配对码
    ceStatus = ceBlueHc_sendDataAndCheck(ceBlueHc, sendBuf, "OK", 4000);
    #ifdef __CE_CHECK_PAR__
    ceDebugOp.printf("Run CMD:%sRun result:%s\n\n", sendBuf, ceSystemOp.getErrorMsg(ceStatus));
    #endif
    if(ceStatus != CE_STATUS_SUCCESS) return ceStatus;


   /* if(ceBlueHcWorkMode == CE_BLUE_HC_WORK_MODE_SLAVE)
    {
        return CE_STATUS_SUCCESS;
    }
    */

    ceDebugOp.sprintf(sendBuf, "AT+IAC=%s\r\n", "9e8b33");//设置查询任意访问码的蓝牙设备
    ceStatus = ceBlueHc_sendDataAndCheck(ceBlueHc, sendBuf, "OK", 2000);
    #ifdef __CE_CHECK_PAR__
    ceDebugOp.printf("Run CMD:%sRun result:%s\n\n", sendBuf, ceSystemOp.getErrorMsg(ceStatus));
    #endif
    if(ceStatus != CE_STATUS_SUCCESS) return ceStatus;

    ceDebugOp.sprintf(sendBuf, "AT+CLASS=%s\r\n", ceBlueHcDevType);//设置蓝牙设备类型
    ceStatus = ceBlueHc_sendDataAndCheck(ceBlueHc, sendBuf, "OK", 2000);
    #ifdef __CE_CHECK_PAR__
    ceDebugOp.printf("Run CMD:%sRun result:%s\n\n", sendBuf, ceSystemOp.getErrorMsg(ceStatus));
    #endif
    if(ceStatus != CE_STATUS_SUCCESS) return ceStatus;

    ceDebugOp.sprintf(sendBuf, "AT+INQM=0,%d,%d\r\n", CE_BLUE_HC_DEV_LIST_SIZE, CE_BLUE_HC_FIND_DEV_OUT_TIME);//查询模式：查询一般的设备，超过CE_BLUE_HC_DEV_LIST_SIZE个蓝牙设备响应则终止查询，设定超时为CE_BLUE_HC_FIND_DEV_OUT_TIME
    ceStatus = ceBlueHc_sendDataAndCheck(ceBlueHc, sendBuf, "OK", 4000);
    #ifdef __CE_CHECK_PAR__
    ceDebugOp.printf("Run CMD:%sRun result:%s\n\n", sendBuf, ceSystemOp.getErrorMsg(ceStatus));
    #endif
    return ceStatus;
}

/**
  * @brief  退出参数配置状态（AT状态），模块重新上电，并进入正常工作模式
  * @param  ceBlueHc:CeBlueHc属性对象
  */
void    ceBlueHc_outParmentConfig(CeBlueHc* ceBlueHc)
{
    ceSystemOp.delayMs(50);
    ceGpioOp.setBit(&(ceBlueHc->ceGpio0));     //模块复位，让蓝牙自动连接设备
    ceGpioOp.resetBit(&(ceBlueHc->ceGpio1));
    ceSystemOp.delayMs(50);
    ceGpioOp.resetBit(&(ceBlueHc->ceGpio0));
}

/**
  * @brief  模式工作在主模式时，查找周围中可连接的蓝牙信息
  * @param  ceBlueHc:CeBlueHc属性对象
  * @return 返回连接的蓝牙信息数组
  */
CeBlueHcDevInfo*  ceBlueHc_getCanConnectDevInfo(CeBlueHc* ceBlueHc)
{
    uint8 dataBuf[CE_BLUE_HC_BUFFER_LENGTH];
    char sendBuf[64];
    CE_STATUS ceStatus;
    uint16 recvDataCount;
    uint16 i;
    uint8 checkOkStatus = 0x00;
    for(i = 0; i < CE_BLUE_HC_DEV_LIST_SIZE; i++)
    {
        ceBlueHc->ceBlueHcDevInfoList[i].devAdress[0] = '\0';
        ceBlueHc->ceBlueHcDevInfoList[i].devName[0] = '\0';
    }
    ceBlueHc->ceBlueHcDevInfoFindDevCount = 0;

    ceStatus = ceBlueHc_sendDataAndCheck(ceBlueHc, "AT+INQ\r\n", "+INQ:", CE_BLUE_HC_FIND_DEV_OUT_TIME * 1300);//
    #ifdef __CE_CHECK_PAR__
    ceDebugOp.printf("Run CMD:%sRun result:%s\n\n", "AT+INQ\r\n", ceSystemOp.getErrorMsg(ceStatus));
    #endif
    if(ceStatus != CE_STATUS_SUCCESS)
    {
        ceBlueHc->ceBlueHcDevInfoFindDevCount = 0;
        return ceBlueHc->ceBlueHcDevInfoList;
    }

    while(1)
    {

        recvDataCount = ceBlueHc_readStringByEndChar(ceBlueHc, dataBuf, CE_BLUE_HC_BUFFER_LENGTH, "\r\n", 0x01, &checkOkStatus, (CE_BLUE_HC_DEV_LIST_SIZE * 4000));//ecn
        if (recvDataCount < 2)
        {
            break;
        }
        ceBlueHc_cpData((uint8*)ceBlueHc->ceBlueHcDevInfoList[ceBlueHc->ceBlueHcDevInfoFindDevCount].devAdress, dataBuf, 14);//地址12位，然后附加两个:，一共14个
        ceBlueHc->ceBlueHcDevInfoList[ceBlueHc->ceBlueHcDevInfoFindDevCount].devAdress[14] = ('\0');
        ceBlueHc->ceBlueHcDevInfoFindDevCount++;

        recvDataCount = ceBlueHc_readStringByEndChar(ceBlueHc, dataBuf, CE_BLUE_HC_BUFFER_LENGTH, "+INQ:", 0x01, &checkOkStatus, (CE_BLUE_HC_DEV_LIST_SIZE * 4000));//ecn
        if (recvDataCount < 4)
        {
            break;
        }
    }

    for(i = 0; i < ceBlueHc->ceBlueHcDevInfoFindDevCount; i++)
    {
        uint8 j;
        for(j = 0; j < 32; j++)
        {
            if(ceBlueHc->ceBlueHcDevInfoList[i].devAdress[j] == ':')
            {
                ceBlueHc->ceBlueHcDevInfoList[i].devAdress[j] = ',';
            }
            else if(ceBlueHc->ceBlueHcDevInfoList[i].devAdress[j] == '\0')
            {
                break;
            }else if(ceBlueHc->ceBlueHcDevInfoList[i].devAdress[j] == ',')
            {
                ceBlueHc->ceBlueHcDevInfoList[i].devAdress[j] = '\0';
            }
        }
    }

    for(i = 0; i < ceBlueHc->ceBlueHcDevInfoFindDevCount; i++)
    {
        ceDebugOp.sprintf(sendBuf, "AT+RNAME? %s\r\n", ceBlueHc->ceBlueHcDevInfoList[i].devAdress);
        ceStatus = ceBlueHc_sendDataAndCheck(ceBlueHc, sendBuf, "+RNAME:", 5000);//
        #ifdef __CE_CHECK_PAR__
        ceDebugOp.printf("Run CMD:%sRun result:%s\n\n", sendBuf, ceSystemOp.getErrorMsg(ceStatus));
        #endif
        if(ceStatus != CE_STATUS_SUCCESS)
        {
            ceBlueHc_cpData((uint8*)ceBlueHc->ceBlueHcDevInfoList[i].devName, (uint8*)"Unknow device", ceStringOp.strlen("Unknow device"));
            continue;
        }
        recvDataCount = ceBlueHc_readStringByEndChar(ceBlueHc, dataBuf, CE_BLUE_HC_BUFFER_LENGTH, "OK", 0x01, &checkOkStatus, 2000);
        if(recvDataCount < 1)
        {
            ceBlueHc_cpData((uint8*)ceBlueHc->ceBlueHcDevInfoList[i].devName, (uint8*)"Unknow device", ceStringOp.strlen("Unknow device"));
            continue;
        }
        ceBlueHc_cpData((uint8*)ceBlueHc->ceBlueHcDevInfoList[i].devName, dataBuf, recvDataCount - (ceStringOp.strlen("\r\n") + 1));//不拷贝后面的\r\n
        ceBlueHc->ceBlueHcDevInfoList[i].devName[recvDataCount - 2 + 1] = '\0';
        #ifdef __CE_CHECK_PAR__
        ceDebugOp.printf("devName: %s\n", ceBlueHc->ceBlueHcDevInfoList[i].devName);
        #endif

    }
    return  ceBlueHc->ceBlueHcDevInfoList;
}

/**
  * @brief  模式工作在主模式时，查找周围中可连接的蓝牙设备数量
  * @param  ceBlueHc:CeBlueHc属性对象
  * @return 返回可连接的蓝牙设备数量
  */
uint8 ceBlueHc_getCanConnectDevCount(CeBlueHc* ceBlueHc)
{
    ceBlueHc_getCanConnectDevInfo(ceBlueHc);
    return ceBlueHc->ceBlueHcDevInfoFindDevCount;
}

/**
  * @brief  模式工作在主模式时，查找指定蓝牙名称的设备是否存在并处于可连接状态
  * @param  ceBlueHc:CeBlueHc属性对象
  * @param  devBlueName:需要检查的从设备名称
  * @return 返回CE_STATUS_SUCCESS表示可连接， 返回其它表示不可连接
  */
CE_STATUS ceBlueHc_checkDevIsExist(CeBlueHc* ceBlueHc, const char* devBlueName)
{
    int i;
    ceBlueHc_getCanConnectDevInfo(ceBlueHc);
    for(i = 0; i < ceBlueHc->ceBlueHcDevInfoFindDevCount; i++)
    {
        //ceDebugOp.printf("devBlueName: %s devName: %s\n", devBlueName, ceBlueHc->ceBlueHcDevInfoList[i].devName);
        if(ceStringOp.strcmp(devBlueName, ceBlueHc->ceBlueHcDevInfoList[i].devName) == 0)
        {
            return CE_STATUS_SUCCESS;
        }
    }
    return CE_STATUS_FAILE;
}

/**
  * @brief  模式工作在主模式时，使用设备名称来连接一个从设备
  * @param  ceBlueHc:CeBlueHc属性对象
  * @param  devBlueName:需要连接的从设备名称
  * @return 返回CE_STATUS_SUCCESS表示连接成功，
  */
CE_STATUS ceBlueHc_connectDevByName(CeBlueHc* ceBlueHc, const char* devBlueName)
{
    char sendBuf[64];
    uint8 i;
    CE_STATUS ceStatus;

    ceBlueHc_getCanConnectDevInfo(ceBlueHc);

    for(i = 0; i < ceBlueHc->ceBlueHcDevInfoFindDevCount; i++)
    {
        if(ceStringOp.strcmp(devBlueName, ceBlueHc->ceBlueHcDevInfoList[i].devName) == 0)
        {
            //先配对，也可以直接连接
            ceDebugOp.sprintf(sendBuf, "AT+PAIR=%s,%d\r\n", ceBlueHc->ceBlueHcDevInfoList[i].devAdress, 20);
            ceStatus = ceBlueHc_sendDataAndCheck(ceBlueHc, sendBuf, "OK", 30000);
#ifdef __CE_CHECK_PAR__
            ceDebugOp.printf("Run CMD:%s\nRun result:%s\n\n", sendBuf, ceSystemOp.getErrorMsg(ceStatus));
#endif
            if(ceStatus != CE_STATUS_SUCCESS)
            {
                return CE_STATUS_FAILE;
            }

            ceDebugOp.sprintf(sendBuf, "AT+LINK=%s\r\n", ceBlueHc->ceBlueHcDevInfoList[i].devAdress);
            ceStatus = ceBlueHc_sendDataAndCheck(ceBlueHc, sendBuf, "OK", 20000);//如果未配对直接连接，超时时间应设置长一些，应该需要等待从设备输入确认码
#ifdef __CE_CHECK_PAR__
            ceDebugOp.printf("Run CMD:%s\nRun result:%s\n\n", sendBuf, ceSystemOp.getErrorMsg(ceStatus));
#endif
            if(ceStatus != CE_STATUS_SUCCESS)
            {
                //如果收到对应错误码，应该进行一些处理
                return ceStatus;
            }
        }
    }
    return ceStatus;
}

/**
  * @brief  获取模块的连接状态
  * @param  ceBlueHc:CeBlueHc属性对象
  * @return 返回0x00:模块处于未配对状态；返回0x01:模块处于配对成功状态，可以进行数据传输
  */
uint8 getConnectStatus(CeBlueHc* ceBlueHc)
{
    /*
    //获取状态
    ceDebugOp.sprintf(sendBuf, "AT+STATE?\r\n");
    ceStatus = ceBlueHc_sendDataAndCheck(ceBlueHc, sendBuf, "OK", 20000);
#ifdef __CE_CHECK_PAR__
    ceDebugOp.printf("Run CMD:%s\nRun result:%s\n\n", sendBuf, ceSystemOp.getErrorMsg(ceStatus));
#endif
    if(ceStatus == CE_STATUS_SUCCESS)
    {
        return CE_STATUS_SUCCESS;
    }
    return CE_STATUS_FAILE;*/
    return ceGpioOp.getBit(&(ceBlueHc->ceGpio2));//如有必要，可以采用上面的方式
}

/**
  * @brief  发送数据
  * @param  ceBlueHc:CeBlueHc属性对象
  * @param  dataInBuf:需要发送的数据缓存
  * @param  sendCount:需要发送
  * @return 实际发送完成的数据长度
  */
void ceBlueHc_sendData(CeBlueHc* ceBlueHc, uint8* dataInBuf, uint16 sendCount)
{
    uint16 timeOutMs = 0;
    while(ceBlueHc->isLockRecvBuf == 0x01)
    {
        ceSystemOp.delayMs(1);
        timeOutMs++;
        if(timeOutMs >= 2000)
        {
            break;
        }
    }
    ceBlueHc->isLockRecvBuf = 0x01;
    ceUartOp.sendData(&(ceBlueHc->ceUart), dataInBuf, sendCount);
    ceBlueHc->isLockRecvBuf = 0x00;
}

/**
  * @brief  获取接收缓存中的可读取的数据长度
  * @param  ceBlueHc:CeBlueHc属性对象
  * @return 接收缓存中的可读取的数据数量
  */
uint16 ceBlueHc_getRecvDataCount(CeBlueHc* ceBlueHc)
{
    return ceUartOp.getRecvDataCount(&(ceBlueHc->ceUart));
}

/**
  * @brief  从接收缓存中读取数据
  * @param  ceBlueHc:CeBlueHc属性对象
  * @param  dataOutBuf:读取数据存放的缓存
  * @param  readCount:需要读取的数据长度
  * @return 实际读取到的数据长度
  */
uint16 ceBlueHc_readData(CeBlueHc* ceBlueHc, uint8* dataOutBuf, uint16 readCount)
{
    return ceUartOp.readData(&(ceBlueHc->ceUart), dataOutBuf, readCount);
}

/**
  * @brief  CeBlueHc模块操作对象定义
  */
const CeBlueHcOpBase ceBlueHcOp = {ceBlueHc_initial, ceBlueHc_parmentConfig, ceBlueHc_outParmentConfig,ceBlueHc_getCanConnectDevInfo, ceBlueHc_getCanConnectDevCount,
                                   ceBlueHc_checkDevIsExist, ceBlueHc_connectDevByName, getConnectStatus,
                                   ceBlueHc_sendData, ceBlueHc_getRecvDataCount, ceBlueHc_readData
                                  };

#ifdef __cplusplus
}
#endif //__cplusplus
