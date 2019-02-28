/**
  ******************************************************************************
  * @file    CeI2c.c
  * @author  Creelinks Application Team
  * @version V1.0.0
  * @date    2016-08-05
  * @brief   Creelinks平台CeI2c库文件
  ******************************************************************************
  * @attention
  *
  *1)因Stm32f103中的硬件I2c Bug，为保证程序的稳定性，这里采用软件模拟方式实现所有的资源I2c
  *
  * <h2><center>&copy; COPYRIGHT 2016 Darcern</center></h2>
  ******************************************************************************
  */
#include "CeI2c.h"
#include "CeSystem.h"
#ifdef __cplusplus
 extern "C" {
#endif //__cplusplus

#define CE_I2CMASTER_NOT_INITIAL  0x00      /*!< I2cMaster未始化标志*/
#define CE_I2CMASTER_IS_IDLE      0x01      /*!< I2cMaster空闲标志*/
#define CE_I2CMASTER_IS_BUSY      0x02      /*!< I2cMaster忙碌标志*/

uint8   ceI2cMaster_status1 = 0x00;         /*!< I2cMaster总线1状态，值等于CE_I2CMASTER_NOT_INITIAL表示未始化
                                                 值等于CE_I2CMASTER_IS_IDLE表示空闲
                                                 值等于CE_I2CMASTER_IS_BUSY表示忙碌*/

uint8   ceI2cMaster_status2 = 0x00;         /*!< I2cMaster总线2状态，值等于CE_I2CMASTER_NOT_INITIAL表示未始化
                                                 值等于CE_I2CMASTER_IS_IDLE表示空闲
                                                 值等于CE_I2CMASTER_IS_BUSY表示忙碌*/

/**
  * @brief   获取SDA引脚的电平
  * @param   ceI2cMaster:I2cMaster属性对象指针
  * @return  0x00:SDA引脚为低电平，0x01:SDA引脚为高电平
  */
uint8 ceGetI2cMasterSDABit(CeI2cMaster* ceI2cMaster)
{
    if ((ceI2cMaster->ceExI2cMasterPar.ceExSDAGpiox->IDR & ceI2cMaster->ceExI2cMasterPar.ceExSDAGpioPinx) != (uint32)Bit_RESET)
    {
        return (uint8)Bit_SET;
    }
    else
    {
        return (uint8)Bit_RESET;
    }
}

/**
  * @brief   直接配置SDA引脚的工作模式
  * @param   ceI2cMaster:I2cMaster属性对象指针
  * @param   ceGpioMode:SDA引脚的配置模式
  * @return  None
  */
void ceSetI2cMasterSDAMode(CeI2cMaster* ceI2cMaster, uint8 isOut)
{
    GPIO_InitTypeDef GPIO_InitStructure;

    if (isOut == 0x01)
    {
        GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    }
    else
    {
        //GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
        GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;//这里不应该用浮空输入，这个有时间要测试一下.
    }
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Pin = ceI2cMaster->ceExI2cMasterPar.ceExSDAGpioPinx;
    GPIO_Init(ceI2cMaster->ceExI2cMasterPar.ceExSDAGpiox, &GPIO_InitStructure);
}

/**
  * @brief   向从设备发送非应答信号
  * @param   ceI2cMaster:I2cMaster属性对象指针
  * @return  None
  */
void ceI2cMasterNAck(CeI2cMaster* ceI2cMaster)
{
    ceI2cMaster->ceExI2cMasterPar.ceExSCLGpiox->BRR = ceI2cMaster->ceExI2cMasterPar.ceExSCLGpioPinx;
    ceSetI2cMasterSDAMode(ceI2cMaster, 0x01);    //sda线输出
    ceI2cMaster->ceExI2cMasterPar.ceExSDAGpiox->BSRR = ceI2cMaster->ceExI2cMasterPar.ceExSDAGpioPinx;
    ceSystemOp.delayNs(ceI2cMaster->ceExI2cMasterPar.ceExDelayNs);
    ceI2cMaster->ceExI2cMasterPar.ceExSCLGpiox->BSRR = ceI2cMaster->ceExI2cMasterPar.ceExSCLGpioPinx;
    ceSystemOp.delayNs(ceI2cMaster->ceExI2cMasterPar.ceExDelayNs);
    ceI2cMaster->ceExI2cMasterPar.ceExSCLGpiox->BRR = ceI2cMaster->ceExI2cMasterPar.ceExSCLGpioPinx;
}

/**
  * @brief   向从设备发送应答信号
  * @param   ceI2cMaster:I2cMaster属性对象指针
  * @return  None
  */
void ceI2cMasterAck(CeI2cMaster* ceI2cMaster)
{
    ceI2cMaster->ceExI2cMasterPar.ceExSCLGpiox->BRR = ceI2cMaster->ceExI2cMasterPar.ceExSCLGpioPinx;
    ceSetI2cMasterSDAMode(ceI2cMaster, 0x01);     //sda线输出
    ceI2cMaster->ceExI2cMasterPar.ceExSDAGpiox->BRR = ceI2cMaster->ceExI2cMasterPar.ceExSDAGpioPinx;
    ceSystemOp.delayNs(ceI2cMaster->ceExI2cMasterPar.ceExDelayNs);
    ceI2cMaster->ceExI2cMasterPar.ceExSCLGpiox->BSRR = ceI2cMaster->ceExI2cMasterPar.ceExSCLGpioPinx;
    ceSystemOp.delayNs(ceI2cMaster->ceExI2cMasterPar.ceExDelayNs);
    ceI2cMaster->ceExI2cMasterPar.ceExSCLGpiox->BRR = ceI2cMaster->ceExI2cMasterPar.ceExSCLGpioPinx;
}

#ifdef __CE_CHECK_PAR__
/**
  * @brief   对I2cMaster指针对象进行检验
  * @param   ceI2cMaster:I2cMaster属性对象指针
  * @return  系统状态码，可能的值:CE_STATUS_SUCCESS、CE_STATUS_NULL_POINTER、CE_STATUS_RESOURCE_ERROR
  */
CE_STATUS ceCheckCeI2cMaster(CeI2cMaster* ceI2cMaster)
{
    if (ceI2cMaster == CE_NULL)
    {
        return CE_STATUS_NULL_POINTER;
    }
    if (((ceI2cMaster->ceResource & CE_RES_MARK_I2C) != CE_RES_MARK_I2C))
    {
        return CE_STATUS_RESOURCE_ERROR;
    }
    return CE_STATUS_SUCCESS;
}
#endif //__CE_CHECK_PAR__

/**
  * @brief   初始化I2cMaster对象
  * @param   ceI2cMaster:I2cMaster属性对象指针
  * @return  系统状态码，可能的值:CE_STATUS_SUCCESS
  */
CE_STATUS ceI2cMaster_initial(CeI2cMaster* ceI2cMaster)
{
    GPIO_InitTypeDef GPIO_InitStructure;
#ifdef __CE_CHECK_PAR__
    ce_assert_failed((uint8*)__FILE__, __LINE__, ceCheckCeI2cMaster(ceI2cMaster));
#endif //__CE_CHECK_PAR__

    switch ((uint32)(ceI2cMaster->ceResource))
    {
    case I2c1://I2c1,PB0,PB6,PB7
        ceI2cMaster->ceExI2cMasterPar.ceExI2cMasterStatusx = &ceI2cMaster_status1;
        ceI2cMaster->ceExI2cMasterPar.ceExSCLGpiox = GPIOB;
        ceI2cMaster->ceExI2cMasterPar.ceExSCLGpioPinx = GPIO_Pin_6;
        ceI2cMaster->ceExI2cMasterPar.ceExSDAGpiox = GPIOB;
        ceI2cMaster->ceExI2cMasterPar.ceExSDAGpioPinx = GPIO_Pin_7;
        if(ceI2cMaster_status1 != CE_I2CMASTER_NOT_INITIAL)
        {
            return CE_STATUS_SUCCESS;
        }
        ceI2cMaster_status1 = CE_I2CMASTER_IS_IDLE;
        RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
        break;
    case I2c2://I2c2,PA8,PB10,PB11
        ceI2cMaster->ceExI2cMasterPar.ceExI2cMasterStatusx = &ceI2cMaster_status2;
        ceI2cMaster->ceExI2cMasterPar.ceExSCLGpiox = GPIOB;
        ceI2cMaster->ceExI2cMasterPar.ceExSCLGpioPinx = GPIO_Pin_10;
        ceI2cMaster->ceExI2cMasterPar.ceExSDAGpiox = GPIOB;
        ceI2cMaster->ceExI2cMasterPar.ceExSDAGpioPinx = GPIO_Pin_11;
        if(ceI2cMaster_status2 != CE_I2CMASTER_NOT_INITIAL)
        {
            return CE_STATUS_SUCCESS;
        }
        ceI2cMaster_status2 = CE_I2CMASTER_IS_IDLE;
        RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB, ENABLE);
        break;
    default:
        return CE_STATUS_RESOURCE_ERROR;
    }

    switch (ceI2cMaster->ceI2cMasterSpeed)
    {
    case CE_I2C_SPEED_100KBPS:
        ceI2cMaster->ceExI2cMasterPar.ceExDelayNs = 10000;
        break;
    case CE_I2C_SPEED_400KBPS:
        ceI2cMaster->ceExI2cMasterPar.ceExDelayNs = 2500;
        break;
    case CE_I2C_SPEED_3_4MBPS:
        ceI2cMaster->ceExI2cMasterPar.ceExDelayNs = 250;
        break;
    default:
        break;
    }

    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;

    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Pin = ceI2cMaster->ceExI2cMasterPar.ceExSCLGpioPinx;
    GPIO_Init(ceI2cMaster->ceExI2cMasterPar.ceExSCLGpiox, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;//这里不应该用浮空输入，这个有时间要测试一下.
    GPIO_InitStructure.GPIO_Pin = ceI2cMaster->ceExI2cMasterPar.ceExSDAGpioPinx;
    GPIO_Init(ceI2cMaster->ceExI2cMasterPar.ceExSDAGpiox, &GPIO_InitStructure);
    return CE_STATUS_SUCCESS;
}

/**
  * @brief   开始I2cMaster对象操作
  * @param   ceI2cMaster:I2cMaster属性对象指针
  * @return  系统状态码，可能的值:CE_STATUS_SUCCESS
  */
void ceI2cMaster_start(CeI2cMaster* ceI2cMaster)
{
#ifdef __CE_CHECK_PAR__
    ce_assert_failed((uint8*)__FILE__, __LINE__, ceCheckCeI2cMaster(ceI2cMaster));
#endif //__CE_CHECK_PAR__
    ceSetI2cMasterSDAMode(ceI2cMaster, 0x01);     //sda线输出
    ceI2cMaster->ceExI2cMasterPar.ceExSDAGpiox->BSRR = ceI2cMaster->ceExI2cMasterPar.ceExSDAGpioPinx;//发送起始条件的数据信号
    ceI2cMaster->ceExI2cMasterPar.ceExSCLGpiox->BSRR = ceI2cMaster->ceExI2cMasterPar.ceExSCLGpioPinx;
    ceSystemOp.delayNs(2 * ceI2cMaster->ceExI2cMasterPar.ceExDelayNs);
    ceI2cMaster->ceExI2cMasterPar.ceExSDAGpiox->BRR = ceI2cMaster->ceExI2cMasterPar.ceExSDAGpioPinx;//发送起始信号
    ceSystemOp.delayNs(2 * ceI2cMaster->ceExI2cMasterPar.ceExDelayNs);
    ceI2cMaster->ceExI2cMasterPar.ceExSCLGpiox->BRR = ceI2cMaster->ceExI2cMasterPar.ceExSCLGpioPinx;//钳住I2C总线，准备发送或接收数据
}

/**
  * @brief   停止I2cMaster对象操作
  * @param   ceI2cMaster:I2cMaster属性对象指针
  * @return  系统状态码，可能的值:CE_STATUS_SUCCESS
  */
void ceI2cMaster_stop(CeI2cMaster* ceI2cMaster)
{
#ifdef __CE_CHECK_PAR__
    ce_assert_failed((uint8*)__FILE__, __LINE__, ceCheckCeI2cMaster(ceI2cMaster));
#endif //__CE_CHECK_PAR__
    
    ceSetI2cMasterSDAMode(ceI2cMaster, 0x01);//sda线输出
    ceI2cMaster->ceExI2cMasterPar.ceExSCLGpiox->BRR = ceI2cMaster->ceExI2cMasterPar.ceExSCLGpioPinx;
    ceI2cMaster->ceExI2cMasterPar.ceExSDAGpiox->BRR = ceI2cMaster->ceExI2cMasterPar.ceExSDAGpioPinx;//发送结束条件的数据信号
    ceSystemOp.delayNs(2 * ceI2cMaster->ceExI2cMasterPar.ceExDelayNs);
    ceI2cMaster->ceExI2cMasterPar.ceExSCLGpiox->BSRR = ceI2cMaster->ceExI2cMasterPar.ceExSCLGpioPinx;//结束条件建立时间大于4μs
    ceI2cMaster->ceExI2cMasterPar.ceExSDAGpiox->BSRR = ceI2cMaster->ceExI2cMasterPar.ceExSDAGpioPinx;//发送I2C总线结束信号
    ceSystemOp.delayNs(2 * ceI2cMaster->ceExI2cMasterPar.ceExDelayNs);
}

/**
  * @brief   发送一个字节
  * @param   ceI2cMaster:I2cMaster属性对象指针
  * @param   val:要发送的字节
  * @return  系统状态码，可能的值:CE_STATUS_SUCCESS
  */
void ceI2cMaster_sendByte(CeI2cMaster* ceI2cMaster, uint8 val)
{
    u8 BitCnt;
#ifdef __CE_CHECK_PAR__
    ce_assert_failed((uint8*)__FILE__, __LINE__, ceCheckCeI2cMaster(ceI2cMaster));
#endif //__CE_CHECK_PAR__
    ceSetI2cMasterSDAMode(ceI2cMaster, 0x01);     //sda线输出
    ceI2cMaster->ceExI2cMasterPar.ceExSCLGpiox->BRR = ceI2cMaster->ceExI2cMasterPar.ceExSCLGpioPinx;//拉低时钟开始数据传输
    //条件 一定要开启总线 保持SCL处于0状态 才能进行写入
    for (BitCnt = 0; BitCnt < 8; BitCnt++)//要传送的数据长度为8位
    {
        if ((val << BitCnt) & 0x80)//判断发送位  发送是由高位开始发送
        {
            ceI2cMaster->ceExI2cMasterPar.ceExSDAGpiox->BSRR = ceI2cMaster->ceExI2cMasterPar.ceExSDAGpioPinx;
        }
        else
        {
            ceI2cMaster->ceExI2cMasterPar.ceExSDAGpiox->BRR = ceI2cMaster->ceExI2cMasterPar.ceExSDAGpioPinx;
        }
        ceSystemOp.delayNs(ceI2cMaster->ceExI2cMasterPar.ceExDelayNs);
        ceI2cMaster->ceExI2cMasterPar.ceExSCLGpiox->BSRR = ceI2cMaster->ceExI2cMasterPar.ceExSCLGpioPinx;//置时钟线为高，通知被控器开始接收数据位
        ceSystemOp.delayNs(ceI2cMaster->ceExI2cMasterPar.ceExDelayNs);
        ceI2cMaster->ceExI2cMasterPar.ceExSCLGpiox->BRR = ceI2cMaster->ceExI2cMasterPar.ceExSCLGpioPinx;
        ceSystemOp.delayNs(ceI2cMaster->ceExI2cMasterPar.ceExDelayNs);
    }
}

/**
  * @brief   接收一个字节操作
  * @param   ceI2cMaster:I2cMaster属性对象指针
  * @param   isAck:接收完成后，是否发送应答信息，0x00:不发送，0x01:发送
  * @return  读取到的字节
  */
uint8 ceI2cMaster_recvByte(CeI2cMaster* ceI2cMaster, uint8 isAck)
{
    u8 retc = 0, i;
#ifdef __CE_CHECK_PAR__
    ce_assert_failed((uint8*)__FILE__, __LINE__, ceCheckCeI2cMaster(ceI2cMaster));
#endif //__CE_CHECK_PAR__
    ceSetI2cMasterSDAMode(ceI2cMaster, 0x00);            //SDA设置为输入
    for (i = 0; i < 8; i++)
    {
        ceI2cMaster->ceExI2cMasterPar.ceExSCLGpiox->BRR = ceI2cMaster->ceExI2cMasterPar.ceExSCLGpioPinx;//置时钟线为低，准备接收数据位
        ceSystemOp.delayNs(ceI2cMaster->ceExI2cMasterPar.ceExDelayNs);
        ceI2cMaster->ceExI2cMasterPar.ceExSCLGpiox->BSRR = ceI2cMaster->ceExI2cMasterPar.ceExSCLGpioPinx;//置时钟线为高使数据线上数据有效
        ceSystemOp.delayNs(ceI2cMaster->ceExI2cMasterPar.ceExDelayNs / 2);
        retc <<= 1;
        if (GPIO_ReadInputDataBit(ceI2cMaster->ceExI2cMasterPar.ceExSDAGpiox, ceI2cMaster->ceExI2cMasterPar.ceExSDAGpioPinx) > 0x00)
        {
            retc++;//读数据位,接收的数据位放入retc中
        }
        ceSystemOp.delayNs(ceI2cMaster->ceExI2cMasterPar.ceExDelayNs / 2);
    }
    if (isAck <= 0x00)
        ceI2cMasterNAck(ceI2cMaster);//发送nACK
    else
        ceI2cMasterAck(ceI2cMaster);//发送ACK
    return retc;
}

/**
  * @brief   等待I2cMaster对象返回应答信息，如果250个时钟周期内无应答，则返回超时状态码，
  * @param   ceI2cMaster:I2cMaster属性对象指针
  * @return  系统状态码，可能的值:CE_STATUS_OUT_TIME、CE_STATUS_SUCCESS
  */
CE_STATUS ceI2cMaster_waitAck(CeI2cMaster* ceI2cMaster)
{
    u8 Time = 0;
#ifdef __CE_CHECK_PAR__
    ce_assert_failed((uint8*)__FILE__, __LINE__, ceCheckCeI2cMaster(ceI2cMaster));
#endif //__CE_CHECK_PAR__
    ceSetI2cMasterSDAMode(ceI2cMaster, 0x00);//配置SDA为输入
    ceI2cMaster->ceExI2cMasterPar.ceExSDAGpiox->BSRR = ceI2cMaster->ceExI2cMasterPar.ceExSDAGpioPinx;//准备接收应答位
    ceSystemOp.delayNs(ceI2cMaster->ceExI2cMasterPar.ceExDelayNs / 2);
    ceI2cMaster->ceExI2cMasterPar.ceExSCLGpiox->BSRR = ceI2cMaster->ceExI2cMasterPar.ceExSCLGpioPinx;
    ceSystemOp.delayNs(ceI2cMaster->ceExI2cMasterPar.ceExDelayNs / 2);
    while (GPIO_ReadInputDataBit(ceI2cMaster->ceExI2cMasterPar.ceExSDAGpiox, ceI2cMaster->ceExI2cMasterPar.ceExSDAGpioPinx))
    {
        Time++;
        if (Time > 250)
        {
            ceI2cMaster_stop(ceI2cMaster);
            return CE_STATUS_OUT_TIME;//无应答返回超时
        }
    }
    ceI2cMaster->ceExI2cMasterPar.ceExSCLGpiox->BRR = ceI2cMaster->ceExI2cMasterPar.ceExSCLGpioPinx;//时钟输出0
    return CE_STATUS_SUCCESS;
}

/**
  * @brief   获取I2c总线控制权
  * @param   ceI2cMaster:I2cMaster属性对象指针
  * @return  系统状态码，可能的值:CE_STATUS_OUT_TIME、CE_STATUS_SUCCESS
  */
CE_STATUS ceI2cMaster_lockBus(CeI2cMaster* ceI2cMaster)
{
    uint16 temp = 0x0000;
    uint8* pI2cMaster_statusx = (uint8*)(ceI2cMaster->ceExI2cMasterPar.ceExI2cMasterStatusx);
    while((*pI2cMaster_statusx) == CE_I2CMASTER_IS_BUSY)
    {
        ceSystemOp.delayNs (1);
        temp++;
        if(temp > 0x1770)//超时时间为6m
        {
            return CE_STATUS_OUT_TIME;
        }
    };
    *pI2cMaster_statusx = CE_I2CMASTER_IS_BUSY;//如果获取到使用权，立刻将状态位标志为忙碌状态，然后返回获取成功
    return CE_STATUS_SUCCESS;
}

/**
  * @brief   释放I2c总线控制权
  * @param   ceI2cMaster:I2cMaster属性对象指针
  * @return  None
  */
void ceI2cMaster_unlockBus(CeI2cMaster* ceI2cMaster)
{
    *(ceI2cMaster->ceExI2cMasterPar.ceExI2cMasterStatusx) = CE_I2CMASTER_IS_IDLE;
}

const CeI2cMasterOp ceI2cMasterOp = {ceI2cMaster_initial, ceI2cMaster_start, ceI2cMaster_stop, ceI2cMaster_sendByte, ceI2cMaster_recvByte,
                                            ceI2cMaster_waitAck,ceI2cMaster_lockBus, ceI2cMaster_unlockBus};
#ifdef __cplusplus
 }
#endif //__cplusplus

