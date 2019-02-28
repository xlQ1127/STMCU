/**
  ******************************************************************************
  * @file    CeWlsNrf.c
  * @author  Creelinks Application Team
  * @version V1.0.0
  * @date    2017-01-06
  * @brief   适用于CeWlsNrf模块的驱动库文件
  ******************************************************************************
  * @attention
  *
  *1)24L01的最大SPI时钟为10Mhz
  *
  * <h2><center>&copy; COPYRIGHT 2016 Darcern</center></h2>
  ******************************************************************************
  */
#include "CeWlsNrf.h"
#ifdef __cplusplus
 extern "C" {
#endif //__cplusplus

/*!< NRF24L01相关的宏定义*/
#define CE_WLS_NRF_TX_ADR_WIDTH    5         /*!< 5 uints TX address width*/
#define CE_WLS_NRF_RX_ADR_WIDTH    5         /*!< 5 uints RX address width*/

#define CE_WLS_NRF_RX_PLOAD_WIDTH  32        /*!< 32 uints TX payload*/
#define CE_WLS_NRF_TX_PLOAD_WIDTH  32        /*!< 32 uints TX payload*/
/*!< NRF24L01寄存器指令*/
#define CE_WLS_NRF_NRF_READ_REG    0x00      /*!< 读寄存器指令*/
#define CE_WLS_NRF_NRF_WRITE_REG   0x20     /*!< 写寄存器指令*/
#define CE_WLS_NRF_RD_RX_PLOAD     0x61      /*!< 读取接收数据指令*/
#define CE_WLS_NRF_WR_TX_PLOAD     0xA0      /*!< 写待发数据指令*/
#define CE_WLS_NRF_FLUSH_TX        0xE1     /*!< 冲洗发送 FIFO指令*/
#define CE_WLS_NRF_FLUSH_RX        0xE2      /*!< 冲洗接收 FIFO指令*/
#define CE_WLS_NRF_REUSE_TX_PL     0xE3      /*!< 定义重复装载数据指令*/
#define CE_WLS_NRF_NOP             0xFF     /*!< 保留*/
/*!< SPI(nRF24L01)寄存器地址*/
#define CE_WLS_NRF_CONFIG          0x00     /*!< 配置寄存器，CRC校验模式以及收发状态响应方式*/
#define CE_WLS_NRF_EN_AA           0x01     /*!< 使能自动应答功能*/
#define CE_WLS_NRF_EN_RXADDR       0x02     /*!< 接收数据通道允许*/
#define CE_WLS_NRF_SETUP_AW        0x03     /*!< 收发地址宽度设置*/
#define CE_WLS_NRF_SETUP_RETR      0x04     /*!< 自动重发功能设置*/
#define CE_WLS_NRF_RF_CH           0x05     /*!< 射频通道*/
#define CE_WLS_NRF_RF_SETUP        0x06     /*!< 射频寄存器*/
#define CE_WLS_NRF_STATUS          0x07     /*!< 状态寄存器*/
#define CE_WLS_NRF_OBSERVE_TX      0x08     /*!< 发送检测寄存器*/
#define CE_WLS_NRF_CD              0x09     /*!< 载波检测*/
#define CE_WLS_NRF_RX_ADDR_P0      0x0A     /*!< 频道0接收数据地址*/
#define CE_WLS_NRF_RX_ADDR_P1      0x0B     /*!< 频道1接收数据地址*/
#define CE_WLS_NRF_RX_ADDR_P2      0x0C     /*!< 频道2接收数据地址*/
#define CE_WLS_NRF_RX_ADDR_P3      0x0D     /*!< 频道3接收数据地址*/
#define CE_WLS_NRF_RX_ADDR_P4      0x0E     /*!< 频道4接收数据地址*/
#define CE_WLS_NRF_RX_ADDR_P5      0x0F     /*!< 频道5接收数据地址*/
#define CE_WLS_NRF_TX_ADDR         0x10     /*!< 发送地址寄存器*/
#define CE_WLS_NRF_RX_PW_P0        0x11     /*!< 接收频道0有效数据宽度设置寄存器*/
#define CE_WLS_NRF_RX_PW_P1        0x12     /*!< 接收频道1有效数据宽度设置寄存器*/
#define CE_WLS_NRF_RX_PW_P2        0x13     /*!< 接收频道2有效数据宽度设置寄存器*/
#define CE_WLS_NRF_RX_PW_P3        0x14     /*!< 接收频道3有效数据宽度设置寄存器*/
#define CE_WLS_NRF_RX_PW_P4        0x15     /*!< 接收频道4有效数据宽度设置寄存器*/
#define CE_WLS_NRF_RX_PW_P5        0x16     /*!< 接收频道5有效数据宽度设置寄存器*/
#define CE_WLS_NRF_FIFO_STATUS     0x17     /*!< FIFO栈入栈出状态寄存器设置*/
/*!< 中断标志位*/
#define CE_WLS_NRF_RX_DR            6       /*!< 接收到数据中断。中断有效时为1，写1清除中断*/
#define CE_WLS_NRF_TX_DS            5       /*!< 数据发送完成中断*/
#define CE_WLS_NRF_MAX_RT           4       /*!< 重发次数超过设定值中断。中断有效时为1，写1清除中断*/

#define CE_WLS_NRF_MAX_TX      0x10  
#define CE_WLS_NRF_TX_OK       0x20  
#define CE_WLS_NRF_RX_OK       0x40 

#define CE_WLS_NRF_IDLE     0x00      /*!< 空闲标志*/
#define CE_WLS_NRF_RECV     0x01      /*!< 接收状态*/
#define CE_WLS_NRF_SEND     0x02      /*!< 发送状态*/


uint8 ceWlsNrf_rxPacket(CeWlsNrf* ceWlsNrf);
/**
  * @brief  Task任务回调函数，用于处理接收数据
  * @param  pAddr:CeWlsNrf属性对象指针
  * @return None
  */
void ceWlsNrf_intCallBack(void* pAddr)
{
    int8 pipiIndex = 0x00;
    CeWlsNrf* ceWlsNrf = (CeWlsNrf*)pAddr;
    pipiIndex = ceWlsNrf_rxPacket(ceWlsNrf);    
    if (pipiIndex >= 0x01 && pipiIndex < 0x07)//判断是否正常接收数据
    {
        pipiIndex -= 1;
        if(ceWlsNrf->callBackRecv[pipiIndex] != CE_NULL)
        {
            ceWlsNrf->callBackRecv[pipiIndex](ceWlsNrf->recvBuf, CE_WLS_NRF_RX_PLOAD_WIDTH);
        }
    }
}
/**
  * @brief  通过SPI向NRF指定寄存器写入数据
  * @param  ceWlsNrf:CeWlsNrf属性对象
  * @param  reg:欲写入数据的指定寄存器地址
  * @return 返回写入寄存器前寄存器的内容
  */
uint8 ceWlsNrf_writeReg(CeWlsNrf* ceWlsNrf, uint8 reg, uint8 val)
{
    uint8 status;
    ceSpiMasterOp.resetNSSBit(&(ceWlsNrf->ceSpiMaster));
    status = ceSpiMasterOp.writeReadByte(&(ceWlsNrf->ceSpiMaster), reg);
    ceSpiMasterOp.writeReadByte(&(ceWlsNrf->ceSpiMaster), val);
    ceSpiMasterOp.setNSSBit(&(ceWlsNrf->ceSpiMaster));
    return status;
}

/**
  * @brief  通过SPI从NRF读取指定寄存器的数据
  * @param  ceWlsNrf:CeWlsNrf属性对象
  * @param  reg:欲读取的指定寄存器地址
  * @return 返回读取到的寄存器的内容
  */
uint8 ceWlsNrf_readReg(CeWlsNrf* ceWlsNrf, uint8 reg)
{
    uint8 reg_val;
    ceSpiMasterOp.resetNSSBit(&(ceWlsNrf->ceSpiMaster));
    ceSpiMasterOp.writeReadByte(&(ceWlsNrf->ceSpiMaster), reg);
    reg_val = ceSpiMasterOp.writeReadByte(&(ceWlsNrf->ceSpiMaster), 0x00);
    ceSpiMasterOp.setNSSBit(&(ceWlsNrf->ceSpiMaster));
    return reg_val;
}

/**
  * @brief  通过SPI向NRF写入数据
  * @param  ceWlsNrf:CeWlsNrf属性对象
  * @param  reg:欲写入数据的寄存器地址
  * @param  pBuf:存放欲写入的数据的缓冲区
  * @param  bytes:指定欲写入数据的字节数
  * @return 返回操作的状态
  */
uint8 ceWlsNrf_writeBuf(CeWlsNrf* ceWlsNrf, uint8 reg, uint8 *pBuf, uint8 bytes)
{
    uint8 status, i;
    ceSpiMasterOp.resetNSSBit(&(ceWlsNrf->ceSpiMaster));
    status = ceSpiMasterOp.writeReadByte(&(ceWlsNrf->ceSpiMaster), reg);
    for (i = 0; i<bytes; i++)
    {
        ceSpiMasterOp.writeReadByte(&(ceWlsNrf->ceSpiMaster), *pBuf);
        pBuf++;
    }
    ceSpiMasterOp.setNSSBit(&(ceWlsNrf->ceSpiMaster));
    return status;
}

/**
  * @brief  通过SPI从NRF中读取数据
  * @param  ceWlsNrf:CeWlsNrf属性对象
  * @param  reg:欲读取数据的寄存器地址
  * @param  pBuf:存放欲读取数据的缓冲区
  * @param  bytes:指定欲读取数据的字节数
  * @return 返回操作的状态
  */
uint8 ceWlsNrf_readBuf(CeWlsNrf* ceWlsNrf, uint8 reg, uint8 *pBuf, uint8 bytes)
{
    uint8 status, i;
    ceSpiMasterOp.resetNSSBit(&(ceWlsNrf->ceSpiMaster));
    status = ceSpiMasterOp.writeReadByte(&(ceWlsNrf->ceSpiMaster), reg);
    for (i = 0; i < bytes; i++)
    {
        pBuf[i] = ceSpiMasterOp.writeReadByte(&(ceWlsNrf->ceSpiMaster), 0xFF);
    }
    ceSpiMasterOp.setNSSBit(&(ceWlsNrf->ceSpiMaster));
    return status;
}

/**
  * @brief  发生数据包
  * @param  ceWlsNrf:CeWlsNrf属性对象
  * @param  sendBuf:欲发生的数据包缓冲区
  * @param  bufSize:欲发生数据的字节长度
  * @return 返回操作的状态
  */
CE_STATUS ceWlsNrf_txPacket(CeWlsNrf* ceWlsNrf, uint8* sendBuf, uint8 bufSize)
{
    u8 sta;
    uint8 tick = 0;
    ceIntOp.stop(&(ceWlsNrf->ceInt));

    ceGpioOp.resetBit(&(ceWlsNrf->ceGpio));
    ceSystemOp.delayUs(5);
    ceWlsNrf_writeBuf(ceWlsNrf,CE_WLS_NRF_WR_TX_PLOAD,sendBuf,CE_WLS_NRF_TX_PLOAD_WIDTH);
    ceWlsNrf_writeReg(ceWlsNrf,CE_WLS_NRF_NRF_WRITE_REG+CE_WLS_NRF_CONFIG,0x0e);
    ceGpioOp.setBit(&(ceWlsNrf->ceGpio));//置高CE，激发数据发送   
    ceSystemOp.delayUs(20);
    while(ceIntOp.getBit(&(ceWlsNrf->ceInt)) == 0x01)//等待Irq拉低，数据发送完成
    {
        tick++;
        ceSystemOp.delayMs(1);
        if(tick >= 10)
            break;
    }
    sta=ceWlsNrf_readReg(ceWlsNrf,CE_WLS_NRF_STATUS);    
    ceWlsNrf_writeReg(ceWlsNrf,CE_WLS_NRF_NRF_WRITE_REG+CE_WLS_NRF_STATUS,sta);

    ceIntOp.start(&(ceWlsNrf->ceInt));    
        
    if((sta&CE_WLS_NRF_MAX_TX) != 0)
        {
        ceWlsNrf_writeReg(ceWlsNrf,CE_WLS_NRF_FLUSH_TX,0xff);
            return CE_STATUS_FAILE;
        }



    if(sta&CE_WLS_NRF_TX_OK)
    {
        return CE_STATUS_SUCCESS;//发送成功
    }
    return CE_STATUS_FAILE;
}

/**
  * @brief  接收数据包
  * @param  ceWlsNrf:CeWlsNrf属性对象
  * @return 返回操作的状态
  */
uint8 ceWlsNrf_rxPacket(CeWlsNrf* ceWlsNrf)
{
    uint8 status = 0x00;
    status = ceWlsNrf_readReg(ceWlsNrf, CE_WLS_NRF_STATUS);//读取状态寄存其来判断数据接收状况
    ceWlsNrf_writeReg(ceWlsNrf, CE_WLS_NRF_NRF_WRITE_REG + CE_WLS_NRF_STATUS, status);//清除RX_DR中断
    if(status & (0x01 << CE_WLS_NRF_RX_DR))//接收数据中断
    {
        //读数据前应该先判断RX FIFO中是否存在有效数据，FIFO满和不满要分别处理。这样就不会读取到多余的数据了
        ceWlsNrf_readBuf(ceWlsNrf, CE_WLS_NRF_RD_RX_PLOAD, ceWlsNrf->recvBuf, CE_WLS_NRF_RX_PLOAD_WIDTH);//读取数据，当读RX有效数据后，FIFO寄存器中有效数据被清空
        /*status = ceWlsNrf_readReg(ceWlsNrf, CE_WLS_NRF_OBSERVE_TX);//不知这里的操作是否有用，可以检测数据包是否丢失。By Lizhiyang 2016-11-16
        ceWlsNrf_writeReg(ceWlsNrf, CE_WLS_NRF_OBSERVE_TX, status & 0x0F);//数据包丢失计数器清0
        (status & 0xF0);//数据包丢失计数器*/
        status = ((status & 0x0E) >> 1) + 1;//取出通道号
    }
    return status;
}



/**
  * @brief  设置发生模式
  * @param  ceWlsNrf:CeWlsNrf属性对象
  * @param  localAddress:指定本地接收使能ACK的节点地址
  * @param  sendToAddress:指定发生至的节点地址
  * @return 系统状态码
  */
void  ceWlsNrf_setSendMode(CeWlsNrf* ceWlsNrf, uint8* localAddress, uint8* sendToAddress)
{
    ceGpioOp.resetBit(&(ceWlsNrf->ceGpio));//Ce拉低，StandBy I模式
    ceWlsNrf_writeReg(ceWlsNrf, CE_WLS_NRF_NRF_WRITE_REG + CE_WLS_NRF_CONFIG, 0x08);//设置为初始状态
    ceWlsNrf_writeReg(ceWlsNrf, CE_WLS_NRF_FLUSH_TX, 0xFF);                                            //清除TX FIFO寄存器
    ceWlsNrf_writeBuf(ceWlsNrf, CE_WLS_NRF_NRF_WRITE_REG + CE_WLS_NRF_TX_ADDR, sendToAddress, CE_WLS_NRF_TX_ADR_WIDTH);        //写TX节点地址
    ceWlsNrf_writeBuf(ceWlsNrf, CE_WLS_NRF_NRF_WRITE_REG + CE_WLS_NRF_RX_ADDR_P0, localAddress, CE_WLS_NRF_RX_ADR_WIDTH);     //设置RX节点地址,主要为了使能ACK
    ceWlsNrf_writeReg(ceWlsNrf, CE_WLS_NRF_NRF_WRITE_REG + CE_WLS_NRF_EN_AA, 0x3f);     //使能所有通道自动应答
    ceWlsNrf_writeReg(ceWlsNrf, CE_WLS_NRF_NRF_WRITE_REG + CE_WLS_NRF_EN_RXADDR, 0x3f); //使能所有通道的接收地址(这两项定义的通道数不得小于当前使用的通道数)
    ceWlsNrf_writeReg(ceWlsNrf, CE_WLS_NRF_NRF_WRITE_REG + CE_WLS_NRF_SETUP_RETR, 0x1A);//设置自动重发间隔时间:500us + 86us;最大自动重发次数:10次
    ceWlsNrf_writeReg(ceWlsNrf, CE_WLS_NRF_NRF_WRITE_REG + CE_WLS_NRF_RF_CH, 40);       //设置RF通道为40,收发必须一致！

    ceWlsNrf_writeReg(ceWlsNrf, CE_WLS_NRF_NRF_WRITE_REG + CE_WLS_NRF_RX_PW_P0, CE_WLS_NRF_RX_PLOAD_WIDTH); 

    ceWlsNrf_writeReg(ceWlsNrf, CE_WLS_NRF_NRF_WRITE_REG + CE_WLS_NRF_RF_SETUP, 0x0F);  //设置TX发射参数,0db增益,2Mbps,低噪声增益开启,收发必须一致！
    ceWlsNrf_writeReg(ceWlsNrf, CE_WLS_NRF_NRF_WRITE_REG + CE_WLS_NRF_CONFIG, 0x0E);    //配置基本工作模式的参数;PWR_UP,EN_CRC,16BIT_CRC,接收模式,开启所有中断
    ceGpioOp.setBit(&(ceWlsNrf->ceGpio));
    ceSystemOp.delayUs(20);
}

/**
  * @brief  设置接收模式
  * @param  ceWlsNrf:CeWlsNrf属性对象
  * @param  pipeIndex:欲操作的接收节点索引，0-5
  * @param  recvAddress:欲操作的接收节点的地址
  * @return 系统状态码
  */
void  ceWlsNrf_setRecvMode(CeWlsNrf* ceWlsNrf, uint8 pipeIndex, uint8* recvAddress)
{
    ceGpioOp.resetBit(&(ceWlsNrf->ceGpio));//Ce拉低，StandBy I模式
    ceWlsNrf_writeReg(ceWlsNrf, CE_WLS_NRF_NRF_WRITE_REG + CE_WLS_NRF_CONFIG, 0x08);//设置为初始状态
    ceWlsNrf_writeReg(ceWlsNrf, CE_WLS_NRF_FLUSH_RX, 0xFF);

    ceWlsNrf_writeBuf(ceWlsNrf, CE_WLS_NRF_NRF_WRITE_REG + CE_WLS_NRF_TX_ADDR, recvAddress, CE_WLS_NRF_TX_ADR_WIDTH);

    if(pipeIndex <= 0x01)
    {
        ceWlsNrf_writeBuf(ceWlsNrf, CE_WLS_NRF_NRF_WRITE_REG + CE_WLS_NRF_RX_ADDR_P0 + pipeIndex, recvAddress, CE_WLS_NRF_RX_ADR_WIDTH);//写接收数据的地址
    }
    else
    {
        ceWlsNrf_writeBuf(ceWlsNrf, CE_WLS_NRF_NRF_WRITE_REG + CE_WLS_NRF_RX_ADDR_P1, recvAddress, CE_WLS_NRF_RX_ADR_WIDTH); //设置RX节点地址,主要为了使能ACK
        ceWlsNrf_writeBuf(ceWlsNrf, CE_WLS_NRF_NRF_WRITE_REG + (CE_WLS_NRF_RX_ADDR_P0 + pipeIndex), recvAddress, 1);  //设置RX节点地址,主要为了使能ACK(地址长度只能是1，不能多写，写一样的值也不行)
    }
    ceWlsNrf_writeReg(ceWlsNrf, CE_WLS_NRF_NRF_WRITE_REG + CE_WLS_NRF_EN_AA, 0x3F << pipeIndex);    //使能通道的自动应答
    ceWlsNrf_writeReg(ceWlsNrf, CE_WLS_NRF_NRF_WRITE_REG + CE_WLS_NRF_EN_RXADDR, 0x3F << pipeIndex);//使能通道的接收地址
    ceWlsNrf_writeReg(ceWlsNrf, CE_WLS_NRF_NRF_WRITE_REG + CE_WLS_NRF_RF_CH, 40);                   //设置RF通信频率
    ceWlsNrf_writeReg(ceWlsNrf, CE_WLS_NRF_NRF_WRITE_REG + (CE_WLS_NRF_RX_PW_P0 + pipeIndex), CE_WLS_NRF_RX_PLOAD_WIDTH);      //选择通道0的有效数据宽度
    ceWlsNrf_writeReg(ceWlsNrf, CE_WLS_NRF_NRF_WRITE_REG + CE_WLS_NRF_RF_SETUP, 0x0F);//设置TX发射参数,0db增益,2Mbps,低噪声增益开启
    ceWlsNrf_writeReg(ceWlsNrf, CE_WLS_NRF_NRF_WRITE_REG + CE_WLS_NRF_CONFIG, 0x0F);//配置基本工作模式的参数;PWR_UP,EN_CRC,16BIT_CRC,接收模式,开启所有中断
    ceGpioOp.setBit(&(ceWlsNrf->ceGpio));//进入接收模式(进入接收模式后130us后才开始检测空中信号，所有接收模式时应一直保持CE为高)
}

/**
  * @brief  通过SPI读写寄存器来判断NRF芯片是否正常连接
  * @param  ceWlsNrf:CeWlsNrf属性对象
  * @return 系统状态码
  */
CE_STATUS ceWlsNrf_check(CeWlsNrf* ceWlsNrf)
{
    uint8 writeBuf[5] = {0xC2, 0xC3, 0xC4, 0xC5, 0xC6};
    uint8 readBuf[5] = { 0 };
    uint8 i = 0;
    ceWlsNrf_writeBuf(ceWlsNrf, CE_WLS_NRF_NRF_WRITE_REG + CE_WLS_NRF_TX_ADDR, writeBuf, 5);//写入5个字节的地址
    ceWlsNrf_readBuf(ceWlsNrf, CE_WLS_NRF_TX_ADDR, readBuf, 5);//读出写入的地址        
    for (i = 0; i < 5; i++)//比较
    {
        if (writeBuf[i] != readBuf[i] && i == 4)
        {
            return CE_STATUS_INITIAL_FALSE;
        }
    }
    return CE_STATUS_SUCCESS;
}

/**
  * @brief  CeWlsNrf模块初始化
  * @param  ceWlsNrf:CeWlsNrf属性对象
  * @param  ceSpi:CeWlsNrf模块使用的Spi资源号
  * @param ceGpio:模块使用的ceGpio资源号
  * @param ceInt:模块使用的ceInt资源号
  * @return 返回CE_STATUS_SUCCESS表示初始化成功，返回CE_STATUS_INITIAL_FALSE表示未检测到NRF芯片
  */
CE_STATUS ceWlsNrf_initial(CeWlsNrf* ceWlsNrf, CE_RESOURCE ceSpi,CE_RESOURCE ceGpio,CE_RESOURCE ceInt)
{
    ceWlsNrf->status = CE_WLS_NRF_IDLE;
    ceWlsNrf->callBackRecv[0] = CE_NULL;

    ceWlsNrf->ceSpiMaster.ceResource = ceSpi;
    ceWlsNrf->ceSpiMaster.ceSpiMasterSpeed = CE_SPI_MASTER_SPEED_10MBPS;
    ceWlsNrf->ceSpiMaster.ceSpiMasterClockPhase = CE_SPI_MASTER_CLOCK_PHASE_1Edge;
    ceWlsNrf->ceSpiMaster.ceSpiMasterClockPolarity = CE_SPI_MASTER_CLOCK_POLARITY_LOW;
    ceSpiMasterOp.initial(&(ceWlsNrf->ceSpiMaster));
    ceSpiMasterOp.start(&(ceWlsNrf->ceSpiMaster));

    ceWlsNrf->ceGpio.ceResource = ceGpio;
    ceWlsNrf->ceGpio.ceGpioMode = CE_GPIO_MODE_OUT_PP;
    ceGpioOp.initial(&(ceWlsNrf->ceGpio));
    ceGpioOp.resetBit(&(ceWlsNrf->ceGpio));//Ce拉低，StandBy I模式

    ceWlsNrf->ceInt.ceResource = ceInt;
    ceWlsNrf->ceInt.callBack=ceWlsNrf_intCallBack;
    ceWlsNrf->ceInt.ceIntMode = CE_INT_MODE_TRIGGER_FALLING;
    ceWlsNrf->ceInt.pAddPar = ceWlsNrf;
    ceIntOp.initial(&(ceWlsNrf->ceInt));//IRQ，低电平标志中断有效
    ceIntOp.start(&(ceWlsNrf->ceInt));
    ceIntOp.stop(&(ceWlsNrf->ceInt));
    ceIntOp.start(&(ceWlsNrf->ceInt));

    return ceWlsNrf_check(ceWlsNrf);
}

/**
  * @brief  进入发送模式，发送操作完成后，函数才返回
  * @param  ceWlsNrf:CeWlsNrf属性对象指针
  * @param  sendAddress:发送地址，与接收端6个接收通道中的一个相同
  * @param  dataBuf:要发送的数据缓存区
  * @param  dataBufSize:要发送的数据长度，注意：一定要为CE_WLS_NRF_PACKET_LENGTH的整数倍
  * @return 返回CE_STATUS_SUCCESS则表明发送成功，其它则表明发送失败
  */
CE_STATUS ceWlsNrf_send(CeWlsNrf* ceWlsNrf, uint8* sendAddress, uint8* dataBuf, uint16 dataBufSize)
{
    if (ceWlsNrf->status != CE_WLS_NRF_SEND)
    {
        ceWlsNrf->status = CE_WLS_NRF_SEND;
    }
    ceWlsNrf_setSendMode(ceWlsNrf, sendAddress, sendAddress);
    return ceWlsNrf_txPacket(ceWlsNrf, dataBuf, CE_WLS_NRF_PACKET_LENGTH);
}

/**
  * @brief  进入接收状态，开始接收数据。注意：当调用send函数后，一定要再次调用此函数才能实现数据接收，异步执行，函数直接返回。
  * @param  ceWlsNrf:CeWlsNrf属性对象指针
  * @param pipeIndex:模块共有6个可用接收通道，范围0-5，此值指定使用哪个接收通道接收数据，如果要使用多个接收通道接收数据，则可重复调用此函数多次
  * @param recvAddress:接收通道对应的接收地址，接收地址有详细规则，详细阅读模块手册
  * @param callBackRecv:当对应通道接收到数据后，调用的回调函数，每个通道的回调函数均独立
  * @return 返回CE_STATUS_SUCCESS则表明配置成功，返回CE_STATUS_PAR_ERROR表示参数错误，其它则表明配置失败
  */
CE_STATUS ceWlsNrf_recv(CeWlsNrf* ceWlsNrf, uint8 pipeIndex, uint8* recvAddress, void(callBackRecv)(uint8* dataBuf, uint16 dataBufSize))
{
    if(pipeIndex > 5)
    {
        return CE_STATUS_PAR_ERROR;
    }
    ceWlsNrf->status = CE_WLS_NRF_RECV;
    ceWlsNrf->callBackRecv[pipeIndex] = callBackRecv;
    ceWlsNrf_setRecvMode(ceWlsNrf, pipeIndex, recvAddress);
    return CE_STATUS_SUCCESS;
}

/**
  * @brief    CeWlsNrf模块操作对象定义
  */
const CeWlsNrfOpBase ceWlsNrfOp = {ceWlsNrf_initial, ceWlsNrf_send, ceWlsNrf_recv};

#ifdef __cplusplus
}
#endif //__cplusplus
