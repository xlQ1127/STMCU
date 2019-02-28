/**
  ******************************************************************************
  * @file    CePackage.c
  * @author  Creelinks Application Team
  * @version V1.0.0
  * @date    2017-03-26
  * @brief   适用于CePackage模块的驱动头文件,用于数据的打包拆包操作
  ******************************************************************************
  * @attention
  *
  *1)添加额外参数量，直接CePackageSend和CePackageRecv中添加即可，系统自动计算结构体长度
  *2)添加的变量一定为int32，否则会出现无法预测的故障
  *
  * <h2><center>&copy; COPYRIGHT 2016 Darcern</center></h2>
  ******************************************************************************
  */
#include "CePackage.h"
#ifdef __cplusplus
extern "C" {
#endif //__cplusplus

uint8 cePackageSendBuf[CE_PACKAGE_SEND_BUF_SIZE];//发送数据缓存

/**
  * @brief  cePackageSend模块初始化，对结构体中的数据进行清0操作
  * @param cePackageSend:CePackageSend属性对象指针
  */
CE_STATUS   cePackage_initialSend(CePackageSend* cePackageSend)
{
    int i;
    int32* temp = (int32*)(cePackageSend);
    for(i=0;i<sizeof(CePackageRecv)/sizeof(int32);i++)
    {
        *temp = 0;
        temp++;
    }

    for(i=0;i<CE_PACKAGE_SEND_BUF_SIZE;i++)
        cePackageSendBuf[i] = 0x00;

    return CE_STATUS_SUCCESS;
}
/**
  * @brief cePackageRecv模块初始化，对结构体中的数据进行清0操作
  * @param cePackageRecv:CePackageRecv属性对象指针
  */
CE_STATUS   cePackage_initialRecv(CePackageRecv* cePackageRecv)
{
    int i;
    int32* temp = (int32*)(cePackageRecv);
    for(i=0;i<sizeof(CePackageRecv)/sizeof(int32);i++)
    {
        *temp = 0;
        temp++;
    }
    return CE_STATUS_SUCCESS;
}
/**
  * @brief 对cePackageSend结构体进行打包，反回打包后可直接发送byte数组
  * @param cePackageSend:CePackageSend属性对象指针
  * @param ctlStatus:无人机当前受控状态，对应cePackageRecv里的status
  * @return 打包后可直接发送的byte数组，长度为CE_PACKAGE_SEND_BUF_SIZE
  */
uint8*  cePackage_dealSend(CePackageSend* cePackageSend, uint32 ctlStatus)
{
    int i,j;
    int32* temp = (int32*)(cePackageSend);
    uint8 sum=0;
    uint16 packCount = 0;
    if((ctlStatus & CE_CTL_TYPE_STATION)  != 0)//如果是地面站点控制方式，则发送全部数据；其它控制方式则只发送一包数据即可。
        packCount = (CE_PACKAGE_SEND_BUF_SIZE / CE_PACKAGE_PACK_SIZE);
    else 
        packCount = 1;
    for(j=0;j< packCount;j++)
    {
        cePackageSendBuf[0+j*CE_PACKAGE_PACK_SIZE] = 0x55;  //帧头0x55
        cePackageSendBuf[1+j*CE_PACKAGE_PACK_SIZE] = j;     //第N包
        sum = 0x55+j;
        for(i=2;i< CE_PACKAGE_PACK_SIZE-2;i+=4)
        {
            if(temp >= (int32*)(cePackageSend) + sizeof(CePackageSend)/sizeof(int32))
                break;
            cePackageSendBuf[i+0+j*CE_PACKAGE_PACK_SIZE] = (*temp)&0xFF;
            cePackageSendBuf[i+1+j*CE_PACKAGE_PACK_SIZE] = ((*temp) >> 8)&0xFF;
            cePackageSendBuf[i+2+j*CE_PACKAGE_PACK_SIZE] = ((*temp) >> 16)&0xFF;
            cePackageSendBuf[i+3+j*CE_PACKAGE_PACK_SIZE] = ((*temp) >> 24)&0xFF;
            temp++;
            sum = (sum + cePackageSendBuf[i+j*CE_PACKAGE_PACK_SIZE] + cePackageSendBuf[i+1+j*CE_PACKAGE_PACK_SIZE]) + cePackageSendBuf[i+2+j*CE_PACKAGE_PACK_SIZE]+cePackageSendBuf[i+3+j*CE_PACKAGE_PACK_SIZE] ;//检验合累加
        }
        cePackageSendBuf[CE_PACKAGE_PACK_SIZE+j*CE_PACKAGE_PACK_SIZE-2] = sum;//前面数据的合，用于校验数据的正确性
        cePackageSendBuf[CE_PACKAGE_PACK_SIZE+j*CE_PACKAGE_PACK_SIZE-1] = 0xFE;//帧尾

    }
    return cePackageSendBuf;
}
/**
  * @brief 对recvBuf中的数据进行拆包解析处理，解析后的结果更新到结构体cePackageRecv
  * @param cePackageRecv:CePackageRecv属性对象指针
  * @param recvBuf:接收数据缓存地址
  * @param recvCount:接收数据缓存长度
  * @return 返回CE_STATUS_SUCCESS则解析成功，否则失败
  */
CE_STATUS    cePackage_dealRecv(CePackageRecv* cePackageRecv, uint8* recvBuf, uint16 recvCount)
{
    int i=0,j;
    int packIndex = 0;
    int32* temp = (int32*)(cePackageRecv);
    uint8 sum=0;
    if( recvCount % CE_PACKAGE_PACK_SIZE != 0 )
        return CE_STATUS_FAILE;
    for(j=0;j< recvCount/CE_PACKAGE_PACK_SIZE;j++)
    {
        if(recvBuf[0+j*CE_PACKAGE_PACK_SIZE] != 0x55 || recvBuf[CE_PACKAGE_PACK_SIZE-1+j*CE_PACKAGE_PACK_SIZE] != 0xFE)//检测帧头与帧尾
            return CE_STATUS_FAILE;//帧头不符合，直接返回，不进行处理

        sum = 0;
        for(i=j*CE_PACKAGE_PACK_SIZE;i<j*CE_PACKAGE_PACK_SIZE+CE_PACKAGE_PACK_SIZE-2;i++)        //进行检验合检查
            sum += recvBuf[i];

        if(sum != recvBuf[j*CE_PACKAGE_PACK_SIZE+CE_PACKAGE_PACK_SIZE-2])
            return CE_STATUS_FAILE;//校验合不符合，直接返回，不进行处理

        packIndex = recvBuf[j*CE_PACKAGE_PACK_SIZE+1];

        if(packIndex >= CE_PACKAGE_RECV_BUF_SIZE/CE_PACKAGE_PACK_SIZE)
            break;

        temp = (int32*)(cePackageRecv) + packIndex * (CE_PACKAGE_PACK_SIZE-4)/4;

        for(i=j*CE_PACKAGE_PACK_SIZE+2;i<j*CE_PACKAGE_PACK_SIZE+CE_PACKAGE_PACK_SIZE-2;i+=4)
        {
            if(temp >= (int32*)(cePackageRecv) + sizeof(CePackageRecv)/sizeof(int32))
                break;

            *temp = (int32)(((0x000000FF & recvBuf[i+3]) << 24) |((0x000000FF & recvBuf[i+2]) << 16) |((0x000000FF & recvBuf[i+1]) << 8) | (0x000000FF & recvBuf[i]));
            temp++;
        }
    }
    return CE_STATUS_SUCCESS;
}

/**
  * @brief  CePackage模块操作对象定义
  */
const CePackageOp cePackageOp = {cePackage_initialSend,cePackage_initialRecv,cePackage_dealSend,cePackage_dealRecv};

#ifdef __cplusplus
}
#endif //__cplusplus
