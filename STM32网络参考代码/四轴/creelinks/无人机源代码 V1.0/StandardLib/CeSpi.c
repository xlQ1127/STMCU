/**
  ******************************************************************************
  * @file    CeSpi.c
  * @author  Creelinks Application Team
  * @version V1.0.0
  * @date    2016-08-05
  * @brief   Creelinks平台CeSpi库文件
  ******************************************************************************
  * @attention
  *
  *1)
  *
  * <h2><center>&copy; COPYRIGHT 2016 Darcern</center></h2>
  ******************************************************************************
  */
#include "CeSpi.h"
#include "CeSystem.h"
#ifdef __cplusplus
 extern "C" {
#endif //__cplusplus
#define CE_SPIMASTER_NOT_INITIAL  0x00          /*!< SpiMaster未始化标志*/
#define CE_SPIMASTER_IS_IDLE      0x01          /*!< SpiMaster空闲标志*/
#define CE_SPIMASTER_IS_BUSY      0x02          /*!< SpiMaster忙碌标志*/

uint8   ceSpiMaster_status1 = 0x00;             /*!< SpiMaster总线1状态，值等于CE_SPIMASTER_NOT_INITIAL表示未始化
                                                                         值等于CE_SPIMASTER_IS_IDLE表示空闲
                                                                         值等于CE_SPIMASTER_IS_BUSY表示忙碌*/

uint8   ceSpiMaster_status2 = 0x00;             /*!< SpiMaster总线2状态，值等于CE_SPIMASTER_NOT_INITIAL表示未始化
                                                                         值等于CE_SPIMASTER_IS_IDLE表示空闲
                                                                         值等于CE_SPIMASTER_IS_BUSY表示忙碌*/

uint8   ceSpiMaster_status3 = 0x00;             /*!< SpiMaster总线3状态，值等于CE_SPIMASTER_NOT_INITIAL表示未始化
                                                                         值等于CE_SPIMASTER_IS_IDLE表示空闲
                                                                         值等于CE_SPIMASTER_IS_BUSY表示忙碌*/

#ifdef __CE_CHECK_PAR__
/**
  * @brief   SpiMaster指针对象正确性检查，在调试打开__CE_CHECK_PAR__使用
  * @param   ceSpicMaster:SpiMaster属性对象指针
  * @return  系统状态码，可能的返回值:CE_STATUS_NULL_POINTER、CE_STATUS_RESOURCE_ERROR、CE_STATUS_SUCCESS
  */
CE_STATUS ceCheckSpiMaster(CeSpiMaster* ceSpiMaster)
{
    if (ceSpiMaster == CE_NULL)
    {
        return CE_STATUS_NULL_POINTER;
    }
    if (((ceSpiMaster->ceResource & CE_RES_MARK_SPI) != CE_RES_MARK_SPI))
    {
        return CE_STATUS_RESOURCE_ERROR;
    }
    return CE_STATUS_SUCCESS;
}
#endif //__CE_CHECK_PAR__

/**
  * @brief   SpiMaster对象，发送并接收一字节数据
  * @param   ceSpicMaster:SpiMaster属性对象指针
  * @param   writeVal:要发送的数据，单位字节
  * @return  读到的数据，单位字节
  */
uint8 ceSpiMasterByte_writeRead(CeSpiMaster* ceSpiMaster, uint8 writeVal)
{
    uint8 retry = 0;
#ifdef __CE_CHECK_PAR__
    ce_assert_failed((uint8*)__FILE__, __LINE__, ceCheckSpiMaster(ceSpiMaster));
#endif //__CE_CHECK_PAR__
    while (SPI_I2S_GetFlagStatus(ceSpiMaster->ceExSpiMasterPar.ceExSPIx, SPI_I2S_FLAG_TXE) == RESET) //检查指定的SPI标志位设置与否:发送缓存空标志位
    {
        retry++;
        if (retry > 200) //超时退出
            return 0;
    }
    SPI_I2S_SendData(ceSpiMaster->ceExSpiMasterPar.ceExSPIx, writeVal); //通过外设SPIx发送一个数据
    retry = 0;

    while (SPI_I2S_GetFlagStatus(ceSpiMaster->ceExSpiMasterPar.ceExSPIx, SPI_I2S_FLAG_RXNE) == RESET) //检查指定的SPI标志位设置与否:接受缓存非空标志位
    {
        retry++;
        if (retry > 200) //超时退出
            return 0;
    }
    return SPI_I2S_ReceiveData(ceSpiMaster->ceExSpiMasterPar.ceExSPIx); //返回通过SPIx最近接收的数据

}

/**
  * @brief   初始化SpiMaster对象，需要完全执行，还需在本函数执行完后调用 ceSpiMaster_start
  * @param   ceSpicMaster:SpiMaster属性对象指针
  * @return  系统状态码，可能的返回值:CE_STATUS_SUCCESS
  */
CE_STATUS ceSpiMaster_initial(CeSpiMaster* ceSpiMaster)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    SPI_InitTypeDef SPI_InitStructure;
    uint16 SPI_BaudRatePrescaler;
#ifdef __CE_CHECK_PAR__
    ce_assert_failed((uint8*)__FILE__, __LINE__, ceCheckSpiMaster(ceSpiMaster));
#endif //__CE_CHECK_PAR__



    switch ((uint32)(ceSpiMaster->ceResource))//SPI1时钟接在APB2上，是72MHz。SPI2和SPI3时钟接在APB1上，是36MHz。
    {
    case Spi1://SPI1
        ceSpiMaster->ceExSpiMasterPar.ceExSpiMasterStatusx = &ceSpiMaster_status1;
        ceSpiMaster->ceExSpiMasterPar.ceExSPIx = SPI1;
        ceSpiMaster->ceExSpiMasterPar.ceExNSSGpiox = GPIOA;
        ceSpiMaster->ceExSpiMasterPar.ceExNSSGpioPinx = GPIO_Pin_4;//PA4 SPI2 NSS
        ceSpiMaster->ceExSpiMasterPar.ceExSCKGpiox = GPIOA;
        ceSpiMaster->ceExSpiMasterPar.ceExSCKGpioPinx = GPIO_Pin_5;//PA5 SPI2 SCK
        ceSpiMaster->ceExSpiMasterPar.ceExMOSIGpiox = GPIOA;
        ceSpiMaster->ceExSpiMasterPar.ceExMOSIGpioPinx = GPIO_Pin_7;//PA7 SPI2 MOSI
        ceSpiMaster->ceExSpiMasterPar.ceExMISOGpiox = GPIOA;
        ceSpiMaster->ceExSpiMasterPar.ceExMISOGpioPinx = GPIO_Pin_6;//PA6 SPI2 MISO
        if(ceSpiMaster_status1 != CE_SPIMASTER_NOT_INITIAL)
        {
            return CE_STATUS_SUCCESS;
        }
        ceSpiMaster_status1 = CE_SPIMASTER_IS_IDLE;
        RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1, ENABLE);
        RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
        break;
    case Spi2://SPI2
        ceSpiMaster->ceExSpiMasterPar.ceExSpiMasterStatusx = &ceSpiMaster_status2;
        ceSpiMaster->ceExSpiMasterPar.ceExSPIx = SPI2;
        ceSpiMaster->ceExSpiMasterPar.ceExNSSGpiox = GPIOB;
        ceSpiMaster->ceExSpiMasterPar.ceExNSSGpioPinx = GPIO_Pin_12;//PB12 SPI2 NSS
        ceSpiMaster->ceExSpiMasterPar.ceExSCKGpiox = GPIOB;
        ceSpiMaster->ceExSpiMasterPar.ceExSCKGpioPinx = GPIO_Pin_13;//PB13 SPI2 SCK
        ceSpiMaster->ceExSpiMasterPar.ceExMOSIGpiox = GPIOB;
        ceSpiMaster->ceExSpiMasterPar.ceExMOSIGpioPinx = GPIO_Pin_15;//PB15 SPI2 MOSI
        ceSpiMaster->ceExSpiMasterPar.ceExMISOGpiox = GPIOB;
        ceSpiMaster->ceExSpiMasterPar.ceExMISOGpioPinx = GPIO_Pin_14;//PB14 SPI2 MISO
        if(ceSpiMaster_status2 != CE_SPIMASTER_NOT_INITIAL)
        {
            return CE_STATUS_SUCCESS;
        }
        ceSpiMaster_status2 = CE_SPIMASTER_IS_IDLE;
        RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI2, ENABLE);
        RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);       
        break;
    case Spi3://SPI3
        ceSpiMaster->ceExSpiMasterPar.ceExSpiMasterStatusx = &ceSpiMaster_status3;
        ceSpiMaster->ceExSpiMasterPar.ceExSPIx = SPI3;
        ceSpiMaster->ceExSpiMasterPar.ceExNSSGpiox = GPIOA;
        ceSpiMaster->ceExSpiMasterPar.ceExNSSGpioPinx = GPIO_Pin_15;//PA15 SPI3 NSS
        ceSpiMaster->ceExSpiMasterPar.ceExSCKGpiox = GPIOB;
        ceSpiMaster->ceExSpiMasterPar.ceExSCKGpioPinx = GPIO_Pin_3;//PB3 SPI3 SCK
        ceSpiMaster->ceExSpiMasterPar.ceExMOSIGpiox = GPIOB;
        ceSpiMaster->ceExSpiMasterPar.ceExMOSIGpioPinx = GPIO_Pin_5;//PB5 SPI3 MOSI
        ceSpiMaster->ceExSpiMasterPar.ceExMISOGpiox = GPIOB;
        ceSpiMaster->ceExSpiMasterPar.ceExMISOGpioPinx = GPIO_Pin_4;//PB4 SPI3 MISO
        if(ceSpiMaster_status3 != CE_SPIMASTER_NOT_INITIAL)
        {
            return CE_STATUS_SUCCESS;
        }
        ceSpiMaster_status3 = CE_SPIMASTER_IS_IDLE;
        //为了利用串行调试接口来释放一些普通I/O口，用户软件必须在复位后设置SWJ_CFG=010，从而释放PA15，PB3和PB4用做普通I/O口。(拷贝于STM32中文参考手册 29.4.4)
        RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
        GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable, ENABLE);
        RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI3, ENABLE);
        RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB, ENABLE);        
        break;
    default:
        return CE_STATUS_RESOURCE_ERROR;
    }

    ceSpiMaster->ceExSpiMasterPar.ceExSPIx->CR1 &= 0XFFC7; //设置SPI速度
    if (ceSpiMaster->ceSpiMasterSpeed <= CE_SPI_MASTER_SPEED_50MBPS)
        SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_2;
    else if (ceSpiMaster->ceSpiMasterSpeed == CE_SPI_MASTER_SPEED_20MBPS)
        SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_4;
    else if (ceSpiMaster->ceSpiMasterSpeed == CE_SPI_MASTER_SPEED_10MBPS)
        SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_8;
    else if (ceSpiMaster->ceSpiMasterSpeed == CE_SPI_MASTER_SPEED_5MBPS)
        SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_16;
    else if (ceSpiMaster->ceSpiMasterSpeed == CE_SPI_MASTER_SPEED_1MBPS)
        SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_32;
    else if (ceSpiMaster->ceSpiMasterSpeed == CE_SPI_MASTER_SPEED_500KBPS)
        SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_128;
    else if (ceSpiMaster->ceSpiMasterSpeed >= CE_SPI_MASTER_SPEED_100KBPS)
        SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_256;

    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Pin = ceSpiMaster->ceExSpiMasterPar.ceExNSSGpioPinx;
    GPIO_Init(ceSpiMaster->ceExSpiMasterPar.ceExNSSGpiox, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Pin = ceSpiMaster->ceExSpiMasterPar.ceExSCKGpioPinx;
    GPIO_Init(ceSpiMaster->ceExSpiMasterPar.ceExSCKGpiox, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Pin = ceSpiMaster->ceExSpiMasterPar.ceExMOSIGpioPinx;
    GPIO_Init(ceSpiMaster->ceExSpiMasterPar.ceExMOSIGpiox, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_InitStructure.GPIO_Pin = ceSpiMaster->ceExSpiMasterPar.ceExMISOGpioPinx;
    GPIO_Init(ceSpiMaster->ceExSpiMasterPar.ceExMISOGpiox, &GPIO_InitStructure);

    GPIO_SetBits(ceSpiMaster->ceExSpiMasterPar.ceExNSSGpiox, ceSpiMaster->ceExSpiMasterPar.ceExNSSGpioPinx);//片选拉高，低电平有效，不确定所有设备都是这个样子
    GPIO_ResetBits(ceSpiMaster->ceExSpiMasterPar.ceExSCKGpiox, ceSpiMaster->ceExSpiMasterPar.ceExSCKGpioPinx);
    GPIO_ResetBits(ceSpiMaster->ceExSpiMasterPar.ceExMOSIGpiox, ceSpiMaster->ceExSpiMasterPar.ceExMOSIGpioPinx);
    GPIO_ResetBits(ceSpiMaster->ceExSpiMasterPar.ceExMISOGpiox, ceSpiMaster->ceExSpiMasterPar.ceExMISOGpioPinx);

    SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;  //设置SPI单向或者双向的数据模式:SPI设置为双线双向全双工
    SPI_InitStructure.SPI_Mode = SPI_Mode_Master;                       //设置SPI工作模式:设置为主SPI
    SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;                   //设置SPI的数据大小:SPI发送接收8位帧结构
    SPI_InitStructure.SPI_CPOL = (ceSpiMaster->ceSpiMasterClockPolarity == CE_SPI_MASTER_CLOCK_POLARITY_LOW) ? SPI_CPOL_Low : SPI_CPOL_High;//串行同步时钟的空闲状态为高电平
    SPI_InitStructure.SPI_CPHA = (ceSpiMaster->ceSpiMasterClockPhase == CE_SPI_MASTER_CLOCK_PHASE_1Edge) ? SPI_CPHA_1Edge : SPI_CPHA_2Edge;//串行同步时钟的第二个跳变沿（上升或下降）数据被采样
    SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;                            //NSS信号由硬件（NSS管脚）还是软件（使用SSI位）管理:内部NSS信号有SSI位控制
    SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler;     //设置指定波特率预分频值，最高设置为2分频，SPI2和SPI3属于APB1的外设，时钟频率最大为36M，SPI1属于APB2的外设，时钟频率最大为36M
    SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;                   //指定数据传输从MSB位还是LSB位开始:数据传输从MSB位开始
    SPI_InitStructure.SPI_CRCPolynomial = 7;                             //CRC值计算的多项式
    SPI_Init(ceSpiMaster->ceExSpiMasterPar.ceExSPIx, &SPI_InitStructure);
    return CE_STATUS_SUCCESS;
}

/**
  * @brief   开始SpiMaster，需在ceSpiMaster_initial之后调用
  * @param   ceSpicMaster:SpiMaster属性对象指针
  * @return  系统状态码，可能的返回值:CE_STATUS_SUCCESS
  */
void ceSpiMaster_start(CeSpiMaster* ceSpiMaster)
{
#ifdef __CE_CHECK_PAR__
    ce_assert_failed((uint8*)__FILE__, __LINE__, ceCheckSpiMaster(ceSpiMaster));
#endif //__CE_CHECK_PAR__
    if (ceSpiMaster->ceExSpiMasterPar.ceExSPIx != CE_NULL)
    {
        SPI_Cmd(ceSpiMaster->ceExSpiMasterPar.ceExSPIx, ENABLE);//使能SPI
    }
}

/**
  * @brief   停止SpiMaster
  * @param   ceSpicMaster:SpiMaster属性对象指针
  * @return  系统状态码，可能的返回值:CE_STATUS_SUCCESS
  */
void ceSpiMaster_stop(CeSpiMaster* ceSpiMaster)
{
#ifdef __CE_CHECK_PAR__
    ce_assert_failed((uint8*)__FILE__, __LINE__, ceCheckSpiMaster(ceSpiMaster));
#endif //__CE_CHECK_PAR__
    if (ceSpiMaster->ceExSpiMasterPar.ceExSPIx != CE_NULL)
    {
        SPI_Cmd(ceSpiMaster->ceExSpiMasterPar.ceExSPIx, DISABLE);
    }
}

/**
  * @brief    设置SpiMaster的NSS引脚电平为高
  * @param    ceSpicMaster:SpiMaster属性对象指针
  * @return   None
  */
void ceSpiMaster_setNSSBit(CeSpiMaster* ceSpiMaster)
{
#ifdef __CE_CHECK_PAR__
    ce_assert_failed((uint8*)__FILE__, __LINE__, ceCheckSpiMaster(ceSpiMaster));
#endif //__CE_CHECK_PAR__
    GPIO_SetBits(ceSpiMaster->ceExSpiMasterPar.ceExNSSGpiox, ceSpiMaster->ceExSpiMasterPar.ceExNSSGpioPinx);    
}

/**
  * @brief    设置SpiMaster的NSS引脚电平为低
  * @param    ceSpicMaster:SpiMaster属性对象指针
  * @return   None
  */
void ceSpiMaster_resetNSSBit(CeSpiMaster* ceSpiMaster)
{
#ifdef __CE_CHECK_PAR__
    ce_assert_failed((uint8*)__FILE__, __LINE__, ceCheckSpiMaster(ceSpiMaster));
#endif //__CE_CHECK_PAR__
    GPIO_ResetBits(ceSpiMaster->ceExSpiMasterPar.ceExNSSGpiox, ceSpiMaster->ceExSpiMasterPar.ceExNSSGpioPinx);
}

/**
  * @brief   获取Spi总线控制权
  * @param   ceSpiMaster:SpiMaster属性对象指针
  * @return  系统状态码，可能的值:CE_STATUS_OUT_TIME、CE_STATUS_SUCCESS
  */
CE_STATUS ceSpiMaster_lockBus(CeSpiMaster* ceSpiMaster)
{
    uint16 temp = 0x0000;
    uint8* pSpiMaster_statusx = (uint8*)(ceSpiMaster->ceExSpiMasterPar.ceExSpiMasterStatusx);

    while((*pSpiMaster_statusx) == CE_SPIMASTER_IS_BUSY)
    {
        ceSystemOp.delayMs (1);
        temp++;
        if(temp > 0x1770)//超时时间为6m
        {
            return CE_STATUS_OUT_TIME;
        }
    };
    *pSpiMaster_statusx = CE_SPIMASTER_IS_BUSY;//如果获取到使用权，立刻将状态位标志为忙碌状态，然后返回获取成功
    return CE_STATUS_SUCCESS;
}

/**
  * @brief   释放Spi总线控制权
  * @param   ceSpiMaster:SpiMaster属性对象指针
  * @return  None
  */
void ceSpiMaster_unlockBus(CeSpiMaster* ceSpiMaster)
{
    *(ceSpiMaster->ceExSpiMasterPar.ceExSpiMasterStatusx) = CE_SPIMASTER_IS_IDLE;
}

const CeSpiMasterOp ceSpiMasterOp = {ceSpiMaster_initial, ceSpiMaster_start, ceSpiMaster_stop, ceSpiMasterByte_writeRead, 
                                            ceSpiMaster_setNSSBit, ceSpiMaster_resetNSSBit, ceSpiMaster_lockBus, ceSpiMaster_unlockBus};

#ifdef __cplusplus
 }
#endif //__cplusplus
