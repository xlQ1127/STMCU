/**
  ******************************************************************************
  * @file    CeDa.c
  * @author  Creelinks Application Team
  * @version V1.0.0
  * @date    2017-03-26
  * @brief   基于STM32F103RET6处理器平台的CeDa资源函数实现库文件
  ******************************************************************************
  * @attention
  *
  *1)Da转换完成后的回调，是基于DMA发送完成中断回调的，故勿在回调中执行耗时操作
  *2)Da执行还是有些Bug，有时间在进行整理，屏蔽的代码不要删除！
  *
  * <h2><center>&copy; COPYRIGHT 2016 Darcern</center></h2>
  ******************************************************************************
  */
#include "CeDa.h"
#ifdef __cplusplus
 extern "C" {
#endif //__cplusplus

#define CE_DA_PA4_PREEMPTION_PRIORITY   2                               /*!< 资源R10对应DMA中断的抢占式优先级*/
#define CE_DA_PA4_SUB_PRIORITY          1                               /*!< 资源R10对应DMA中断的响应式优先级*/

#define CE_DA_PA5_PREEMPTION_PRIORITY   2                               /*!< 资源R12对应DMA中断的抢占式优先级*/
#define CE_DA_PA5_SUB_PRIORITY          1                               /*!< 资源R12对应DMA中断的响应式优先级*/

#define CE_DAC1_Address                 0x40007408                      /*!< Da1通道的DMA对应地址*/
#define CE_DAC2_Address                 0x40007414                      /*!< Da2通道的DMA对应地址*/
#define CE_DA_MID2_INTERVAL_NS          (uint32)7456540444//65536       /*!< 两个数据转换时间间隔中间值，定时器65536分频*/
#define CE_DA_MID1_INTERVAL_NS          (uint32)910222//8192            /*!< 两个数据转换时间间隔中间值，定时器8192分频*/

typedef struct
{
    uint16 timPrescaler;
    uint16 timPeriod;
} CeDaTimBaseData;

typedef struct
{
    CeDa* ceDa1;
    CeDa* ceDa2;
} CeDaList;

CeDaList ceDaList = {CE_NULL, CE_NULL};
const uint16 ceDaTemp[1] = { 0x000};           /*!< 初始化DMA通道时的地址*/

/**
  * @brief   根据Da两个数据之前的转换间隔，计划TIM定时器的寄存器值
  * @param   ceDaTimBaseData
  * @param   intervalNs
  * @return  系统状态码，可能的值:CE_STATUS_SUCCESS
  */
void ceCalDaPrescalerAndPeriod(CeDaTimBaseData* ceDaTimBaseData, uint32 intervalNs)
{
    uint16 timPrescaler = 0xFFFF;
    uint16 timPeriod = 0xFFFF;
    if (intervalNs > CE_DA_MAX_INTERVAL_NS)
    {
        intervalNs = CE_DA_MAX_INTERVAL_NS;
    }
    if (intervalNs < CE_DA_MIN_INTERVAL_NS)
    {
        intervalNs = CE_DA_MIN_INTERVAL_NS;
    }
    if (intervalNs >= CE_DA_MID2_INTERVAL_NS)
    {
        timPrescaler = 0xFFFF;
    } 
    else if (intervalNs >= CE_DA_MID1_INTERVAL_NS)
    {
        timPrescaler = 0x1FFF;
    } 
    else
    {
        timPrescaler = 0x0000;
    }
    timPeriod = ((uint64) intervalNs * 72) / (1000 * (timPrescaler + 1));
    if (timPeriod < 7)
    {
        timPeriod = 7;
    }
    ceDaTimBaseData->timPrescaler = timPrescaler;
    ceDaTimBaseData->timPeriod = timPeriod;
}

/**
  * @brief   DMA2 CH3通道数据传送完成中断，对应Da1
  * @return  None
  */
void DMA2_Channel3_IRQHandler()
{
    if (DMA_GetITStatus(DMA2_FLAG_TC3) != RESET)
    {
        DMA_ClearFlag(DMA2_FLAG_GL3);//清除中断标志
        TIM_Cmd(TIM6, DISABLE);//失能定时器
        DMA_Cmd(DMA2_Channel3, DISABLE);//失能DAC通道的DMA
        DAC_DMACmd(DAC_Channel_1, DISABLE);//失能DAC通道的DMA
        DMA_ITConfig(DMA2_Channel3, DMA_IT_TC, DISABLE);//失能发送DMA通道中断
        DAC_Cmd(DAC_Channel_1, DISABLE);//失能DAC通道
        ceDaList.ceDa1->ceExDaPar.ceExIsConvertFinish = 0x01;//设置Da为未工作状态
        if (ceDaList.ceDa1->callBackConvertFinish != CE_NULL)
        {
            ceDaList.ceDa1->callBackConvertFinish(ceDaList.ceDa1->pAddPar);
        }
    }
}

/**
  * @brief   DMA2 CH4通道数据传送完成中断，对应Da2
  * @return  None
  */
void DMA2_Channel4_5_IRQHandler()
{
    if (DMA_GetITStatus(DMA2_FLAG_TC4) != RESET)
    {
        TIM_Cmd(TIM7, DISABLE);//失能定时器
        DMA_ClearFlag(DMA2_FLAG_TC4);//清除中断标志
        DMA_Cmd(DMA2_Channel4, DISABLE);//失能DAC通道的DMA
        DAC_DMACmd(DAC_Channel_2, DISABLE);//失能DAC通道的DMA
        DMA_ITConfig(DMA2_Channel4, DMA_IT_TC, DISABLE);//失能发送DMA通道中断
        DAC_Cmd(DAC_Channel_2, DISABLE);//失能DAC通道
        ceDaList.ceDa2->ceExDaPar.ceExIsConvertFinish = 0x01;//设置Da为未工作状态
        if (ceDaList.ceDa2->callBackConvertFinish != CE_NULL)
        {
            ceDaList.ceDa2->callBackConvertFinish(ceDaList.ceDa2->pAddPar);
        }
    }
}

#ifdef __CE_CHECK_PAR__
/**
  * @brief   检验Da属性指针是否正确
  * @param   ceDa:Da属性对象指针
  * @return  系统状态码，可能的值:CE_STATUS_SUCCESS、CE_STATUS_RESOURCE_ERROR、CE_STATUS_NULL_POINTER
  */
CE_STATUS ceCheckCeDa(CeDa* ceDa)
{
    if (ceDa == CE_NULL)
    {
        return CE_STATUS_NULL_POINTER;
    }
    // if(ceDa->callBackConvertFinish == CE_NULL)//可能用户不使用DA的回调
    // {
    //     return CE_STATUS_NULL_POINTER;
    // }
    if (ceDa->ceResource & CE_RES_MARK_DA != CE_RES_MARK_DA)
    {
        return CE_STATUS_RESOURCE_ERROR;
    }
    return CE_STATUS_SUCCESS;
}
#endif //__CE_CHECK_PAR__

/**
  * @brief   初始化一个Da
  * @param   ceDa:Da属性对象指针
  * @return  系统状态码，可能的值:CE_STATUS_SUCCESS、CE_STATUS_RESOURCE_ERROR、CE_STATUS_NULL_POINTER
  */
CE_STATUS ceDa_initial(CeDa* ceDa)
{
    DMA_InitTypeDef DMA_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;
    GPIO_InitTypeDef GPIO_InitStructure;
    CeDaTimBaseData ceDaTimBaseData;
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
#ifdef __CE_CHECK_PAR__
    ce_assert_failed((uint8*)__FILE__, __LINE__, ceCheckCeDa(ceDa));
#endif //__CE_CHECK_PAR__

    ceDa->ceExDaPar.ceExIsConvertFinish = 0x01;
    if (ceDa->ceResource == PA4ADGI)
    {
        ceDa->ceExDaPar.ceExDMAChannel = DMA2_Channel3;
        ceDa->ceExDaPar.ceExTIMx = TIM6;
        ceDa->ceExDaPar.ceExDAC_Channel_x = DAC_Channel_1;
        ceDa->ceExDaPar.ceExGpiox = GPIOA;
        ceDa->ceExDaPar.ceExGpioPinx = GPIO_Pin_4;
        ceDaList.ceDa1 = ceDa;

        RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM6, ENABLE);
        NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = CE_DA_PA4_PREEMPTION_PRIORITY;// 优先级设置
        NVIC_InitStructure.NVIC_IRQChannelSubPriority = CE_DA_PA4_SUB_PRIORITY;
        NVIC_InitStructure.NVIC_IRQChannel = DMA2_Channel3_IRQn;//发送DMA通道的中断配置
    }
    else if (ceDa->ceResource == PA5ADGI)
    {
        ceDa->ceExDaPar.ceExDMAChannel = DMA2_Channel4;
        ceDa->ceExDaPar.ceExTIMx = TIM7;
        ceDa->ceExDaPar.ceExDAC_Channel_x = DAC_Channel_2;
        ceDa->ceExDaPar.ceExGpiox = GPIOA;
        ceDa->ceExDaPar.ceExGpioPinx = GPIO_Pin_5;
        ceDaList.ceDa2 = ceDa;

        RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM7, ENABLE);
        NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = CE_DA_PA5_PREEMPTION_PRIORITY;// 优先级设置
        NVIC_InitStructure.NVIC_IRQChannelSubPriority = CE_DA_PA5_SUB_PRIORITY;
        NVIC_InitStructure.NVIC_IRQChannel = DMA2_Channel4_5_IRQn;
    }
    else
    {
        return CE_STATUS_RESOURCE_ERROR;
    }
    
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA2, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_DAC, ENABLE);

    //中断配置
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    //Gpio配置
    GPIO_InitStructure.GPIO_Pin = ceDa->ceExDaPar.ceExGpioPinx; //PA4 DAC1
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(ceDa->ceExDaPar.ceExGpiox, &GPIO_InitStructure);
    
    //定时器配置
    ceCalDaPrescalerAndPeriod(&ceDaTimBaseData, ceDa->convertIntervalNs);
    TIM_TimeBaseStructure.TIM_Prescaler = ceDaTimBaseData.timPrescaler;
    TIM_TimeBaseStructure.TIM_Period = ceDaTimBaseData.timPeriod;
    TIM_TimeBaseStructure.TIM_ClockDivision = 0; // 时钟分割
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;//计数方向向上计数
    TIM_DeInit(ceDa->ceExDaPar.ceExTIMx);
    TIM_TimeBaseInit(ceDa->ceExDaPar.ceExTIMx, &TIM_TimeBaseStructure);
    TIM_PrescalerConfig(ceDa->ceExDaPar.ceExTIMx, ceDaTimBaseData.timPrescaler, TIM_PSCReloadMode_Update);
    TIM_SetAutoreload(ceDa->ceExDaPar.ceExTIMx, ceDaTimBaseData.timPeriod);
    TIM_SelectOutputTrigger(ceDa->ceExDaPar.ceExTIMx, TIM_TRGOSource_Update);

    //DMA配置
    DMA_DeInit(ceDa->ceExDaPar.ceExDMAChannel);
    DMA_InitStructure.DMA_PeripheralBaseAddr = ((ceDa->ceResource == PA4ADGI) ? CE_DAC1_Address : CE_DAC2_Address);
    DMA_InitStructure.DMA_MemoryBaseAddr = (uint32) &ceDaTemp;
    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralDST;
    DMA_InitStructure.DMA_BufferSize = 1;
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
    DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
    DMA_InitStructure.DMA_Priority = DMA_Priority_High;
    DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
    DMA_Init(ceDa->ceExDaPar.ceExDMAChannel, &DMA_InitStructure);

    return CE_STATUS_SUCCESS;
}

/**
  * @brief   开始DA转换
  * @param   ceDa:Da属性对象指针
  * @param   dataBuf:需要转换的数据缓存
  * @param   dataBufSize:需要转换的数据长度
  * @return  系统状态码，可能的值:CE_STATUS_SUCCESS
  */
void ceDa_start(CeDa* ceDa, const uint16* dataBuf, uint32 dataBufSize)
{
     DAC_InitTypeDef DAC_InitStructure;
#ifdef __CE_CHECK_PAR__
    ce_assert_failed((uint8*)__FILE__, __LINE__, ceCheckCeDa(ceDa));
#endif //__CE_CHECK_PAR__
    while (ceDa->ceExDaPar.ceExIsConvertFinish == 0x00);
    ceDa->ceExDaPar.ceExIsConvertFinish = 0x00;
    
    //DAC配置
    if (ceDa->ceResource == PA4ADGI)
    {
        DAC_InitStructure.DAC_Trigger = DAC_Trigger_T6_TRGO;//设置TIMx为触发源
        DAC_SetChannel1Data(DAC_Align_12b_R, 0);//设置ADC指定通道的数据位数、对其方式，以及具体输出值。输出值小于0xFFF是12位右对齐
    }
    else
    {
        DAC_InitStructure.DAC_Trigger = DAC_Trigger_T7_TRGO;
        DAC_SetChannel2Data(DAC_Align_12b_R, 0);
    }
    DAC_InitStructure.DAC_WaveGeneration = DAC_WaveGeneration_None;//设置产生的波形，噪声、三角波或者不产生前面两个波形
    DAC_InitStructure.DAC_OutputBuffer = DAC_OutputBuffer_Disable;//设置输出是否需要缓冲
    DAC_Init(ceDa->ceExDaPar.ceExDAC_Channel_x, &DAC_InitStructure);

    //重新配置DMA缓冲区
    ceDa->ceExDaPar.ceExDMAChannel->CMAR = (uint32) dataBuf;//设置DMA通道操作的缓冲区
    ceDa->ceExDaPar.ceExDMAChannel->CNDTR = dataBufSize;//设置缓冲区长度
    //ceDa->ceExDaPar.ceExTIMx->CNT = 0x0000;
    //ceDa->ceExDaPar.ceExTIMx->PSC = 0x0000;

    DMA_ITConfig(ceDa->ceExDaPar.ceExDMAChannel, DMA_IT_TC, ENABLE);//使能DMA通道中断    
    DAC_Cmd(ceDa->ceExDaPar.ceExDAC_Channel_x, ENABLE);//使能DAC通道
    DAC_DMACmd(ceDa->ceExDaPar.ceExDAC_Channel_x, ENABLE);//使能DAC通道的DMA
    DMA_Cmd(ceDa->ceExDaPar.ceExDMAChannel, ENABLE);//使能DMA
    TIM_Cmd(ceDa->ceExDaPar.ceExTIMx, ENABLE);//使能定时器
}

/**
  * @brief   停止Da转换
  * @param   ceDa:Da属性对象指针
  * @return  系统状态码，可能的值:CE_STATUS_SUCCESS
  */
void ceDa_stop(CeDa* ceDa)
{
#ifdef __CE_CHECK_PAR__
    ce_assert_failed((uint8*)__FILE__, __LINE__, ceCheckCeDa(ceDa));
#endif //__CE_CHECK_PAR__

    TIM_Cmd(ceDa->ceExDaPar.ceExTIMx, DISABLE);//失能定时器
    DAC_SoftwareTriggerCmd(ceDa->ceExDaPar.ceExDAC_Channel_x, DISABLE);//失能软件触发
    DMA_Cmd(ceDa->ceExDaPar.ceExDMAChannel, DISABLE);//失能DAC通道的DMA
    DAC_DMACmd(ceDa->ceExDaPar.ceExDAC_Channel_x, DISABLE);//失能DAC通道的DMA
    DMA_ITConfig(ceDa->ceExDaPar.ceExDMAChannel, DMA_IT_TC, DISABLE);//失能发送DMA通道中断
    DAC_Cmd(ceDa->ceExDaPar.ceExDAC_Channel_x, DISABLE);//失能DAC通道

    if (ceDa->ceResource == PA4ADGI)
    {
        DMA_ClearFlag(DMA2_FLAG_GL3);//清除中断标志
    }
    else //if (ceDa->ceResource == R8ADIG)
    {
        DMA_ClearFlag(DMA2_FLAG_GL4);//清除中断标志
    }
    
    ceDa->ceExDaPar.ceExIsConvertFinish = 0x01;
}

/**
  * @brief   Da输出固定值
  * @param   ceDa:Da属性对象指针
  * @return  None
  */
void ceDa_startFixedVoltage(CeDa* ceDa, uint16 val)
{
    DAC_InitTypeDef DAC_InitStructure;
#ifdef __CE_CHECK_PAR__
    ce_assert_failed((uint8*)__FILE__, __LINE__, ceCheckCeDa(ceDa));
#endif //__CE_CHECK_PAR__

    while (ceDa->ceExDaPar.ceExIsConvertFinish == 0x00);
    ceDa->ceExDaPar.ceExIsConvertFinish = 0x00;

    if (val > 0x0FFF)
    {
        val = 0x0FFF;
    }

    //DAC配置
    if (ceDa->ceResource == PA4ADGI)
    {
        DAC_InitStructure.DAC_Trigger = DAC_Trigger_None;//设置无触发源
        DAC_SetChannel1Data(DAC_Align_12b_R, val);//设置ADC指定通道的数据位数、对其方式，以及具体输出值。输出值小于0xFFF是12位右对齐
    }
    else //if (ceDa->ceResource == R8ADIG)
    {
        DAC_InitStructure.DAC_Trigger = DAC_Trigger_None;
        DAC_SetChannel2Data(DAC_Align_12b_R, val);
    }
    DAC_InitStructure.DAC_WaveGeneration = DAC_WaveGeneration_None;//设置产生的波形，噪声、三角波或者不产生前面两个波形
    DAC_InitStructure.DAC_OutputBuffer = DAC_OutputBuffer_Disable;//设置输出是否需要缓冲
    DAC_Init(ceDa->ceExDaPar.ceExDAC_Channel_x, &DAC_InitStructure);

    TIM_Cmd(ceDa->ceExDaPar.ceExTIMx, ENABLE);
    DAC_Cmd(ceDa->ceExDaPar.ceExDAC_Channel_x, ENABLE);//使能DAC通道
    DAC_SoftwareTriggerCmd(ceDa->ceExDaPar.ceExDAC_Channel_x, ENABLE);//使能软件触发
}

/**
  * @brief   更新DA参数
  * @param   ceDa:Da属性对象指针
  */
void ceDa_update(CeDa* ceDa)
{
     CeDaTimBaseData ceDaTimBaseData;
#ifdef __CE_CHECK_PAR__
    ce_assert_failed((uint8*)__FILE__, __LINE__, ceCheckCeDa(ceDa));
#endif //__CE_CHECK_PAR__
    if (ceDa->ceExDaPar.ceExIsConvertFinish == 0x00)//如果Da在工作状态则不进行任何操作
    {
        return;
    }

    //重新配置TIM定时器
    TIM_Cmd(ceDa->ceExDaPar.ceExTIMx, DISABLE);
    ceCalDaPrescalerAndPeriod(&ceDaTimBaseData, ceDa->convertIntervalNs);
    /* Set the Autoreload value */
    ceDa->ceExDaPar.ceExTIMx->ARR = ceDaTimBaseData.timPeriod;
    /* Set the Prescaler value */
    ceDa->ceExDaPar.ceExTIMx->PSC = ceDaTimBaseData.timPrescaler;

    TIM_SetCounter(ceDa->ceExDaPar.ceExTIMx, 0);
    TIM_ClearFlag(ceDa->ceExDaPar.ceExTIMx, TIM_FLAG_Update);//Clear TIMx update pending flag  清除TIMx溢出中断标志
    TIM_ITConfig(ceDa->ceExDaPar.ceExTIMx, TIM_IT_Update, ENABLE);
}

const CeDaOp ceDaOp = {ceDa_initial, ceDa_start, ceDa_startFixedVoltage, ceDa_stop, ceDa_update};

#ifdef __cplusplus
 }
#endif //__cplusplus
