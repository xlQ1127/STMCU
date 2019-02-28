/**
  ******************************************************************************
  * @file   CeUart.c
  * @author Creelinks Application Team
  * @version V1.0.0
  * @date   2016-08-05
  * @brief  基于STM32F103RET6处理器平台的CeUart资源函数实现库文件
  ******************************************************************************
  * @attention
  *
  *1)可以此文件中配置各个Uart资源的中断优先级，有关优先级的内容请参考STM32F103RET6手册
  *2)由于使用DMA传输方式，需要为DMA提供一个缓冲用来保存数据，用户需保证Uart通道每次连续接收到的数据包长度小于
  *  CE_UART_Rx_DMA_BUF_SIZE。默认为512，如果需要，用户可修改为更大的值。
  *
  * <h2><center>&copy; COPYRIGHT 2016 Darcern</center></h2>
  ******************************************************************************
  */
#include "CeUart.h"
#ifdef __cplusplus
 extern "C" {
#endif //__cplusplus

const uint16 ceUartIntPriority[] = {0x2131,0x2131,0x2131,0x2121,0x2121};/*!< 对应Uart发送与接收的中断优先级，录0x2131表示发送中断抢占式优先级为2，响应式优先级为1；接收的抢占式优先级为3，响应式优先级为1*/

#define CE_UART1_DMA_BUF_SIZE            512 /*!< 资源R14DMA通道使用的缓存长度*/
#define CE_UART2_DMA_BUF_SIZE            512 /*!< 资源R24DMA通道使用的缓存长度*/
#define CE_UART3_DMA_BUF_SIZE            512 /*!< 资源R9DMA通道使用的缓存长度*/
#define CE_UART4_DMA_BUF_SIZE            0   /*!< 资源R31未使用DMA方式传输，故不需要提供DMA缓存*/
#define CE_UART5_DMA_BUF_SIZE            0   /*!< 资源R18未使用DMA方式传输，故不需要提供DMA缓存*/

typedef struct
{
    uint16 txIndex;
    uint16 rxIndex;
    uint8* sendBuf;
    uint32 sendBufSize;
} CeUartBase;

CeUartBase ceUartBase4;    /*!< Uart4未使用DMA，这里敬贺对待，进行二次封装*/
CeUartBase ceUartBase5;    /*!< Uart5未使用DMA，这里敬贺对待，进行二次封装*/


CeUart* ceUartList[]={CE_NULL,CE_NULL,CE_NULL,CE_NULL,CE_NULL};      /*!< 用于保存STM32F103对应的5个Uart属性对象*/
uint8 ceUartDmaTemp[1];

uint8 ceUart1DmaBuf[CE_UART1_DMA_BUF_SIZE];
uint8 ceUart2DmaBuf[CE_UART2_DMA_BUF_SIZE];
uint8 ceUart3DmaBuf[CE_UART3_DMA_BUF_SIZE];


/**
  * @brief   Uart1的接收空闲中断，即接收到一定数量数据后，一段时间没有新数据被润滑剂地，则触发此中断
  * @param   None
  * @return  None
  */
void USART1_IRQHandler(void)
{
    if (USART_GetITStatus(USART1, USART_IT_IDLE) != RESET)// 空闲中断
    {
        uint32 recvCount = ceUartList[0]->recvBufSize - DMA_GetCurrDataCounter(DMA1_Channel5);
        DMA_Cmd(DMA1_Channel5, DISABLE);// 关闭DMA ，防止干扰
        DMA_ClearFlag( DMA1_FLAG_GL5);// 清DMA标志位
        DMA1_Channel5->CNDTR = ceUartList[0]->recvBufSize;
        ceFifoOp.write(&(ceUartList[0]->ceExFifo),ceUart1DmaBuf,recvCount);//Uart内部使用的Fifo，可以保证同一时刻只有一个线程在写

        USART_ReceiveData( USART1);         // Clear IDLE interrupt flag bit
        DMA_Cmd(DMA1_Channel5, ENABLE);
    }
}

/**
  * @brief   Uart2的接收空闲中断，即接收到一定数量数据后，一段时间没有新数据被润滑剂地，则触发此中断
  * @param   None
  * @return  None
  */
void USART2_IRQHandler(void)
{
    if (USART_GetITStatus(USART2, USART_IT_IDLE) != RESET)// 空闲中断
    {
        uint32 recvCount = ceUartList[1]->recvBufSize - DMA_GetCurrDataCounter(DMA1_Channel6);
        DMA_Cmd(DMA1_Channel6, DISABLE);// 关闭DMA ，防止干扰
        DMA_ClearFlag( DMA1_FLAG_GL6);// 清DMA标志位
        DMA1_Channel6->CNDTR = ceUartList[1]->recvBufSize;

        ceFifoOp.write(&(ceUartList[1]->ceExFifo),ceUart2DmaBuf,recvCount);//Uart内部使用的Fifo，可以保证同一时刻只有一个线程在写

        USART_ReceiveData(USART2);// Clear IDLE interrupt flag bit
        DMA_Cmd(DMA1_Channel6, ENABLE);
    }
}

/**
  * @brief   Uart3的接收空闲中断，即接收到一定数量数据后，一段时间没有新数据被润滑剂地，则触发此中断
  * @param   None
  * @return  None
  */
void USART3_IRQHandler(void)
{

    if (USART_GetITStatus(USART3, USART_IT_IDLE) != RESET)// 空闲中断
    {
        uint32 recvCount = ceUartList[2]->recvBufSize - DMA_GetCurrDataCounter(DMA1_Channel3);
        DMA_Cmd(DMA1_Channel3, DISABLE);// 关闭DMA ，防止干扰
        DMA_ClearFlag( DMA1_FLAG_GL3);// 清DMA标志位
        DMA1_Channel3->CNDTR = ceUartList[2]->recvBufSize;
        ceFifoOp.write(&(ceUartList[2]->ceExFifo),ceUart3DmaBuf,recvCount);//Uart内部使用的Fifo，可以保证同一时刻只有一个线程在写
        USART_ReceiveData(USART3);// Clear IDLE interrupt flag bit
        DMA_Cmd(DMA1_Channel3, ENABLE);
    }

}

/**
  * @brief   Uar4的发送完成，接收到数据中断函数，未使用Dma方式，故仅推荐用于Debug调试串口
  * @param   None
  * @return  None
  */
void UART4_IRQHandler(void)
{
    if (USART_GetITStatus(UART4, USART_IT_TC) != RESET)// Transmit the string in a loop
    {
        if (ceUartBase4.txIndex >= ceUartBase4.sendBufSize)
        {
            USART_ITConfig(UART4, USART_IT_TC, DISABLE);
            ceUartList[3]->ceExUartPar.ceExIsSendFinishFlag = 0x01;
            ceUartBase4.txIndex = 0;
        } else
        {
            USART_SendData(UART4, ceUartBase4.sendBuf[ceUartBase4.txIndex]);
            ceUartBase4.txIndex++;
        }
    }

    if (USART_GetITStatus(UART4, USART_IT_RXNE) != RESET)// Received characters modify string
    {
        uint8 temp = USART_ReceiveData(UART4);
        //ceUartList.ceUart4->recvBuf[ceUart4Base.rxIndex] = USART_ReceiveData(UART4);
        //ceUart4Base.rxIndex++;
        ceFifoOp.write(&(ceUartList[3]->ceExFifo), &temp, 1);//Uart内部使用的Fifo，可以保证同一时刻只有一个线程在写
    }

    if (USART_GetITStatus(UART4, USART_IT_IDLE) != RESET)// 空闲中断
    {
        USART_ReceiveData(UART4);// Clear IDLE interrupt flag bit
    }
}

/**
  * @brief   Uart5的发送完成，接收到数据中断函数，未使用Dma方式，故仅推荐用于Debug调试串口
  * @param   None
  * @return  None
  */
void UART5_IRQHandler(void)
{
    if (USART_GetITStatus(UART5, USART_IT_TC) != RESET)// Transmit the string in a loop
    {
        if (ceUartBase5.txIndex >= ceUartBase5.sendBufSize)
        {
            USART_ITConfig(UART5, USART_IT_TC, DISABLE);
            ceUartList[4]->ceExUartPar.ceExIsSendFinishFlag = 0x01;
            ceUartBase5.txIndex = 0;
        } else
        {
            USART_SendData(UART5, ceUartBase5.sendBuf[ceUartBase5.txIndex]);
            ceUartBase5.txIndex++;
        }
    }

    if (USART_GetITStatus(UART5, USART_IT_RXNE) != RESET)// Received characters modify string
    {
        uint8 temp = USART_ReceiveData(UART5);
        //ceUartList.ceUart5->recvBuf[ceUart5Base.rxIndex] = USART_ReceiveData(UART5);
        //ceUart5Base.rxIndex++;
        ceFifoOp.write(&(ceUartList[4]->ceExFifo), &temp, 1);//Uart内部使用的Fifo，可以保证同一时刻只有一个线程在写
    }

    if (USART_GetITStatus(UART5, USART_IT_IDLE) != RESET)// 空闲中断
    {
        USART_ReceiveData(UART5);// Clear IDLE interrupt flag bit
    }
}

/**
  * @brief   DMA1,CH4通道传输完成中断函数，对应Uart1的发送完成
  * @param   None
  * @return  None
  */
void DMA1_Channel4_IRQHandler(void)
{
    if (DMA_GetITStatus(DMA1_FLAG_TC4))
    {
        DMA_ClearFlag( DMA1_FLAG_GL4);// 清除标志
        DMA_Cmd(DMA1_Channel4, DISABLE);// 关闭DMA通道
    }
    ceUartList[0]->ceExUartPar.ceExIsSendFinishFlag = 0x01;
}

/**
  * @brief   DMA1,CH7通道传输完成中断函数，对应Uart2的发送完成
  * @param   None
  * @return  None
  */
void DMA1_Channel7_IRQHandler(void)
{
    if (DMA_GetITStatus(DMA1_FLAG_TC7))
    {
        DMA_ClearFlag( DMA1_FLAG_GL7);// 清除标志
        DMA_Cmd(DMA1_Channel7, DISABLE);// 关闭DMA通道
    }
    ceUartList[1]->ceExUartPar.ceExIsSendFinishFlag = 0x01;
}


/**
  * @brief   DMA1,CH2通道传输完成中断函数，对应Uart3的发送完成
  * @param   None
  * @return  None
  */
void DMA1_Channel2_IRQHandler(void)
{
    if (DMA_GetITStatus(DMA1_FLAG_TC2))
    {
        DMA_ClearFlag(DMA1_FLAG_GL2);// 清除标志
        DMA_Cmd(DMA1_Channel2, DISABLE);// 关闭DMA通道
    }
    ceUartList[2]->ceExUartPar.ceExIsSendFinishFlag = 0x01;
}


#ifdef __CE_CHECK_PAR__
/**
  * @brief   检验Uart指针参数
  * @param   ceUart:CeUart属性对象指针
  * @return  系统状态码，可能的值:CE_STATUS_SUCCESS、CE_STATUS_RESOURCE_ERROR、CE_STATUS_NULL_POINTER
  */
CE_STATUS ceCheckCeUart(CeUart* ceUart)
{
    if (ceUart == CE_NULL)
    {
        return CE_STATUS_NULL_POINTER;
    }
    if ((ceUart->ceResource & CE_RES_MARK_UART) != CE_RES_MARK_UART)
    {
        return CE_STATUS_RESOURCE_ERROR;
    }
    return CE_STATUS_SUCCESS;
}
#endif //__CE_CHECK_PAR__

/**
  * @brief   初始化Uart
  * @param   ceUart:CeUart属性对象指针
  * @return  系统状态码，可能的值:CE_STATUS_SUCCESS、CE_STATUS_RESOURCE_ERROR、CE_STATUS_NULL_POINTER
  */
CE_STATUS ceUart_initial(CeUart* ceUart)
{
    GPIO_InitTypeDef GPIO_InitStructureTx;
    GPIO_InitTypeDef GPIO_InitStructureRx;
    NVIC_InitTypeDef NVIC_InitStructureTx;
    NVIC_InitTypeDef NVIC_InitStructureRx;
    USART_InitTypeDef USART_InitStructure;
    DMA_InitTypeDef DMA_InitStructure;
    GPIO_TypeDef * GPIOx;
    uint8 UART_IRQN;
    uint8 uartIndex;
#ifdef __CE_CHECK_PAR__
    ce_assert_failed((uint8*)(uint8*)__FILE__, __LINE__, ceCheckCeUart(ceUart));
#endif //__CE_CHECK_PAR__

    uartIndex = ceUart->ceResource & 0x0000000F;

    ceUart->ceExFifo.buff = ceUart->recvBuf;
    ceUart->ceExFifo.buffSize = ceUart->recvBufSize;
    ceFifoOp.initial(&(ceUart->ceExFifo));

    ceUart->ceExUartPar.ceExIsSendFinishFlag = 0x00;   //防止未完全初始化便开始发送数据

    switch (ceUart->ceResource)
    {
    case Uart1://Uart1
        RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);
        RCC_APB2PeriphClockCmd( RCC_APB2Periph_GPIOA | RCC_APB2Periph_AFIO, ENABLE);
        GPIO_InitStructureTx.GPIO_Pin = GPIO_Pin_9;
        GPIO_InitStructureRx.GPIO_Pin = GPIO_Pin_10;
        ceUart->ceExUartPar.ceExUartx = USART1;
        ceUart->ceExUartPar.ceExDMAChannelTx = DMA1_Channel4;
        ceUart->ceExUartPar.ceExDMAChannelRx = DMA1_Channel5;
        UART_IRQN = USART1_IRQn;
        ceUart->ceExUartPar.ceExDMAx_FLAG_GLx_Tx = DMA1_FLAG_GL4;
        ceUart->ceExUartPar.ceExDMAx_FLAG_GLx_Rx = DMA1_FLAG_GL5;
        ceUart->ceExUartPar.ceExDMAx_Channelx_IRQn = DMA1_Channel4_IRQn;
        ceUartList[0] = ceUart;
        GPIOx = GPIOA;
        break;
    case Uart2://Uart2
        RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);
        RCC_APB2PeriphClockCmd( RCC_APB2Periph_GPIOA | RCC_APB2Periph_AFIO, ENABLE);
        GPIO_InitStructureTx.GPIO_Pin = GPIO_Pin_2;
        GPIO_InitStructureRx.GPIO_Pin = GPIO_Pin_3;
        ceUart->ceExUartPar.ceExUartx = USART2;
        ceUart->ceExUartPar.ceExDMAChannelTx = DMA1_Channel7;
        ceUart->ceExUartPar.ceExDMAChannelRx = DMA1_Channel6;
        UART_IRQN = USART2_IRQn;
        ceUart->ceExUartPar.ceExDMAx_FLAG_GLx_Tx = DMA1_FLAG_GL7;
        ceUart->ceExUartPar.ceExDMAx_FLAG_GLx_Rx = DMA1_FLAG_GL6;
        ceUart->ceExUartPar.ceExDMAx_Channelx_IRQn = DMA1_Channel7_IRQn;
        ceUartList[1] = ceUart;
        GPIOx = GPIOA;
        break;
    case Uart3://Uart3
        RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);
        RCC_APB2PeriphClockCmd( RCC_APB2Periph_GPIOB | RCC_APB2Periph_AFIO, ENABLE);
        GPIO_InitStructureTx.GPIO_Pin = GPIO_Pin_10;
        GPIO_InitStructureRx.GPIO_Pin = GPIO_Pin_11;
        ceUart->ceExUartPar.ceExUartx = USART3;
        ceUart->ceExUartPar.ceExDMAChannelTx = DMA1_Channel2;
        ceUart->ceExUartPar.ceExDMAChannelRx = DMA1_Channel3;
        UART_IRQN = USART3_IRQn;
        ceUart->ceExUartPar.ceExDMAx_FLAG_GLx_Tx = DMA1_FLAG_GL2;
        ceUart->ceExUartPar.ceExDMAx_FLAG_GLx_Rx = DMA1_FLAG_GL3;
        ceUart->ceExUartPar.ceExDMAx_Channelx_IRQn = DMA1_Channel2_IRQn;
        ceUartList[2] = ceUart;
        GPIOx = GPIOB;
        break;
    case Uart4://Uart4
        RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART4, ENABLE);
        RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
        GPIO_InitStructureTx.GPIO_Pin = GPIO_Pin_10;
        GPIO_InitStructureRx.GPIO_Pin = GPIO_Pin_11;
        ceUart->ceExUartPar.ceExUartx = UART4;
        UART_IRQN = UART4_IRQn;
        ceUartList[3] = ceUart;
        GPIOx = GPIOC;
        break;
    case Uart5://Uart5
        RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART5, ENABLE);
        RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC | RCC_APB2Periph_GPIOD, ENABLE);
        GPIO_InitStructureTx.GPIO_Pin = GPIO_Pin_12;
        GPIO_InitStructureRx.GPIO_Pin = GPIO_Pin_2;
        ceUart->ceExUartPar.ceExUartx = UART5;
        UART_IRQN = UART5_IRQn;// 接收和发送中断
        ceUartList[4] = ceUart;
        GPIOx = GPIOC;
        break;
    default:
        return CE_STATUS_RESOURCE_ERROR;
    }

    NVIC_InitStructureTx.NVIC_IRQChannel = ceUart->ceExUartPar.ceExDMAx_Channelx_IRQn;//发送DMA通道的中断配置
    NVIC_InitStructureRx.NVIC_IRQChannel = UART_IRQN;//串口发送或接收中断配置
    NVIC_InitStructureTx.NVIC_IRQChannelPreemptionPriority = (ceUartIntPriority[uartIndex]>>12)&0x000F;//优先级设置
    NVIC_InitStructureTx.NVIC_IRQChannelSubPriority = (ceUartIntPriority[uartIndex]>>8)&0x000F;
    NVIC_InitStructureRx.NVIC_IRQChannelPreemptionPriority = (ceUartIntPriority[uartIndex]>>4)&0x000F;
    NVIC_InitStructureRx.NVIC_IRQChannelSubPriority = (ceUartIntPriority[uartIndex]>>0)&0x000F;
    NVIC_InitStructureTx.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructureTx);//TX

    NVIC_InitStructureRx.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructureRx);//RX

    GPIO_InitStructureTx.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructureTx.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOx, &GPIO_InitStructureTx);
    GPIO_InitStructureRx.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init((UART_IRQN == UART5_IRQn ? GPIOD : GPIOx), &GPIO_InitStructureRx);//注意，只有Uart5的Tx与Rx端口不是一样的！

    if (UART_IRQN != UART4_IRQn && UART_IRQN != UART5_IRQn)
    {
        RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);// 开启DMA1时钟

        DMA_Cmd(ceUart->ceExUartPar.ceExDMAChannelTx, DISABLE);                 // 关DMA通道
        DMA_DeInit(ceUart->ceExUartPar.ceExDMAChannelTx);                       // 恢复缺省值
        DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32) (&ceUart->ceExUartPar.ceExUartx->DR);// 设置串口发送数据寄存器
        DMA_InitStructure.DMA_MemoryBaseAddr = (uint32) ceUartDmaTemp;          // 设置发送缓冲区首地址
        DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralDST;                      // 设置外设位目标，内存缓冲区 ->外设寄存器
        DMA_InitStructure.DMA_BufferSize = (uint16) 1;                          // 需要发送的字节数，这里其实可以设置为0，因为在实际要发送的时候，会重新设置次值
        DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;        // 外设地址不做增加调整，调整不调整是DMA自动实现的
        DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;                 // 内存缓冲区地址增加调整
        DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte; // 外设数据宽度8位，1个字节
        DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;         // 内存数据宽度8位，1个字节
        DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;                           // 单次传输模式
        DMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh;                 // 优先级设置
        DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;                            // 关闭内存到内存的DMA模式
        DMA_Init(ceUart->ceExUartPar.ceExDMAChannelTx, &DMA_InitStructure);     // 写入配置
        DMA_ClearFlag(ceUart->ceExUartPar.ceExDMAx_FLAG_GLx_Tx);                // 清除DMA所有标志
        DMA_Cmd(ceUart->ceExUartPar.ceExDMAChannelTx, DISABLE);                 // 关闭DMA
        DMA_ITConfig(ceUart->ceExUartPar.ceExDMAChannelTx, DMA_IT_TC, ENABLE);  // 开启发送DMA通道中断

        DMA_Cmd(ceUart->ceExUartPar.ceExDMAChannelRx, DISABLE);                 // 关DMA通道
        DMA_DeInit(ceUart->ceExUartPar.ceExDMAChannelRx);                       // 恢复缺省值
        DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32) (&ceUart->ceExUartPar.ceExUartx->DR);// 设置串口接收数据寄存器
        switch (UART_IRQN)
        {
        case USART2_IRQn:
             DMA_InitStructure.DMA_MemoryBaseAddr = (uint32)ceUart2DmaBuf;      // 设置接收缓冲区首地址
            break;
        case USART1_IRQn:
            DMA_InitStructure.DMA_MemoryBaseAddr = (uint32)ceUart1DmaBuf;       // 设置接收缓冲区首地址
            break;
        case USART3_IRQn:
            DMA_InitStructure.DMA_MemoryBaseAddr = (uint32)ceUart3DmaBuf;       // 设置接收缓冲区首地址
            break;
        default:
            break;
        }

        DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;                      // 设置外设为数据源，外设寄存器 -> 内存缓冲区
        DMA_InitStructure.DMA_BufferSize = (uint16)ceUart->recvBufSize;        // 需要最大可能接收到的字节数
        DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;        // 外设地址不做增加调整，调整不调整是DMA自动实现的
        DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;                 // 内存缓冲区地址增加调整
        DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte; // 外设数据宽度8位，1个字节
        DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;         // 内存数据宽度8位，1个字节
        DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;                           // 单次传输模式
        DMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh;                 // 优先级设置
        DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;                            // 关闭内存到内存的DMA模式
        DMA_Init(ceUart->ceExUartPar.ceExDMAChannelRx, &DMA_InitStructure);     // 写入配置
        DMA_ClearFlag(ceUart->ceExUartPar.ceExDMAx_FLAG_GLx_Rx);                // 清除DMA所有标志
        DMA_Cmd(ceUart->ceExUartPar.ceExDMAChannelRx, ENABLE);                  // 开启接收DMA通道，等待接收数据
    }
    USART_InitStructure.USART_BaudRate = ceUart->uartBaudRate;
    USART_InitStructure.USART_WordLength = ceUart->uartWordLength;
    USART_InitStructure.USART_WordLength = ceUart->uartWordLength == CE_UART_WORD_LENGTH_9B ? USART_WordLength_9b : USART_WordLength_8b;
    switch (ceUart->uartStopBits)
    {
    case CE_UART_STOP_BITS_1:
        USART_InitStructure.USART_StopBits = USART_StopBits_1;
        break;
    case CE_UART_STOP_BITS_0_5:
        USART_InitStructure.USART_StopBits = USART_StopBits_0_5;
        break;
    case CE_UART_STOP_BITS_2:
        USART_InitStructure.USART_StopBits = USART_StopBits_2;
        break;
    case CE_UART_STOP_BITS_1_5:
        USART_InitStructure.USART_StopBits = USART_StopBits_1_5;
        break;
    default:
        USART_InitStructure.USART_StopBits = USART_StopBits_1;
        break;
    }
    switch(ceUart->uartParity)
    {
    case CE_UART_PARITY_NO:
        USART_InitStructure.USART_Parity = USART_Parity_No;
        break;
    case CE_UART_PARITY_EVEN:
        USART_InitStructure.USART_Parity = USART_Parity_Even;
        break;
    case CE_UART_PARITY_ODD:
        USART_InitStructure.USART_Parity = USART_Parity_Odd;
        break;
    default:
        USART_InitStructure.USART_Parity = USART_Parity_No;
        break;
    }

    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
    USART_Init(ceUart->ceExUartPar.ceExUartx, &USART_InitStructure);

    USART_ITConfig(ceUart->ceExUartPar.ceExUartx, USART_IT_IDLE, ENABLE);
    if (UART_IRQN != UART4_IRQn && UART_IRQN != UART5_IRQn)
    {
        USART_DMACmd(ceUart->ceExUartPar.ceExUartx, USART_DMAReq_Tx, ENABLE);
        USART_DMACmd(ceUart->ceExUartPar.ceExUartx, USART_DMAReq_Rx, ENABLE);
    }
    else
    {
        //USART_ITConfig(ceUart->ceExUartPar.ceExUartx, USART_IT_TXE, ENABLE);//在进行接收的时候再开启对应的TX中断
        USART_ITConfig(ceUart->ceExUartPar.ceExUartx, USART_IT_RXNE, ENABLE);
    }
    ceUart->ceExUartPar.ceExIsSendFinishFlag = 0x01;

    return CE_STATUS_SUCCESS;
}

/**
  * @brief   开始Uart
  * @param   eUart:ceUart属性对象指针
  * @return  系统状态码，可能的值:CE_STATUS_SUCCESS
  */
void ceUart_start(CeUart* ceUart)
{
#ifdef __CE_CHECK_PAR__
    ce_assert_failed((uint8*)(uint8*)__FILE__, __LINE__, ceCheckCeUart(ceUart));
#endif //__CE_CHECK_PAR__
    USART_Cmd(ceUart->ceExUartPar.ceExUartx, ENABLE);  // 开启串口
}

/**
  * @brief   通过Uart对象发送数据，DMA方式
  * @param   ceUart:CeUart属性对象指针
  * @param   dataBuf:待发送的数据
  * @param   dataCount:待发送的数据长度
  * @return  系统状态码，可能的值:CE_STATUS_SUCCESS
  */
CE_STATUS ceUart_sendData(CeUart* ceUart, uint8* dataBuf, uint16 dataCount)
{
#ifdef __CE_CHECK_PAR__
    ce_assert_failed((uint8*)(uint8*)(uint8*)__FILE__, __LINE__, ceCheckCeUart(ceUart));
#endif //__CE_CHECK_PAR__
    if (dataCount == 0)
    {
        return CE_STATUS_SUCCESS;
    }
    while (ceUart->ceExUartPar.ceExIsSendFinishFlag == 0x00);
    ceUart->ceExUartPar.ceExIsSendFinishFlag = 0x00;
    while (USART_GetFlagStatus(ceUart->ceExUartPar.ceExUartx, USART_FLAG_TC) == RESET);

    if (ceUart->ceResource == Uart4)
    {
        ceUartBase4.sendBuf = dataBuf;
        ceUartBase4.sendBufSize = dataCount;
        ceUartBase4.rxIndex = 0;
        USART_ITConfig(ceUart->ceExUartPar.ceExUartx, USART_IT_TC, ENABLE);
    }
    else if (ceUart->ceResource == Uart5)
    {
        ceUartBase5.sendBuf = dataBuf;
        ceUartBase5.sendBufSize = dataCount;
        ceUartBase5.rxIndex = 0;
        USART_ITConfig(ceUart->ceExUartPar.ceExUartx, USART_IT_TC, ENABLE);
    }
    else
    {
        ceUart->ceExUartPar.ceExDMAChannelTx->CMAR = (uint32)dataBuf;
        ceUart->ceExUartPar.ceExDMAChannelTx->CNDTR = (uint16)dataCount;// 设置要发送的字节数目
        DMA_Cmd(ceUart->ceExUartPar.ceExDMAChannelTx, ENABLE);//开始DMA发送
    }
    return CE_STATUS_SUCCESS;
}

/**
  * @brief   获得接收缓存中的可用数据
  * @param   ceUart:CeUart属性对象指针
  * @return  返回可读取的数据长度
  */
uint16 ceUart_getRecvDataCount(CeUart* ceUart)
{
    return ceFifoOp.getCanReadSize(&(ceUart->ceExFifo));
}

/**
  * @brief   接收数据
  * @param   ceUart:CeUart属性对象指针
  * @return  返回实际读取到的数据长度
  */
uint16 ceUart_readData(CeUart* ceUart, uint8* dataBuf, uint16 readCount)
{
    return ceFifoOp.read(&(ceUart->ceExFifo),dataBuf, readCount);
}

/**
  * @brief   停止Uart
  * @param   ceUart:CeUart属性对象指针
  * @return  系统状态码，可能的值:CE_STATUS_SUCCESS
  */
void ceUart_stop(CeUart* ceUart)
{
#ifdef __CE_CHECK_PAR__
    ce_assert_failed((uint8*)(uint8*)__FILE__, __LINE__, ceCheckCeUart(ceUart));
#endif //__CE_CHECK_PAR__
    USART_Cmd(ceUart->ceExUartPar.ceExUartx, DISABLE);
}

void ceUart_clearRecvBuf(CeUart* ceUart)
{
    ceFifoOp.clear(&(ceUart->ceExFifo));
}

const CeUartOp ceUartOp = {ceUart_initial, ceUart_start, ceUart_sendData, ceUart_getRecvDataCount, ceUart_readData, ceUart_stop, ceUart_clearRecvBuf};

#ifdef __cplusplus
 }
#endif //__cplusplus
