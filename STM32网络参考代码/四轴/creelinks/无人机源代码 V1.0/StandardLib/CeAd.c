/**
  ******************************************************************************
  * @file   CeAd.c
  * @author Creelinks Application Team
  * @version V1.0.0
  * @date   2017-03-26
  * @brief  Creelinks平台的Ad对象库实现函数，基于STM32F103x平台
  ******************************************************************************
  * @attention
  *
  *1)所有Ad共用的同一个Ad转换模块，采取循环采集方式转换
  *
  * <h2><center>&copy; COPYRIGHT 2016 Darcern</center></h2>
  ******************************************************************************
  */
#include "CeAd.h"
#include "CeSystem.h"

#ifdef __cplusplus
 extern "C" {
#endif  //__cplusplus

#define CE_AD_ADC1_DR_Address   ((uint32)0x4001244C)
#define CE_AD_NOT_INITIAL       0x00    /*!< AD未始化标志*/
#define CE_AD_IS_IDLE           0x01    /*!< AD空闲标志*/
#define CE_AD_IS_BUSY           0x02    /*!< AD忙标志*/

uint8   ceAd_status = 0x00;             /*!< AD的状态，值等于CE_AD_NOT_INITIAL表示未始化*/
GPIO_TypeDef*   ceAdGpioxArray[] = {GPIOA,GPIOB,GPIOC,GPIOD
#ifdef GPIOE
,GPIOE
#endif

#ifdef GPIOF
,GPIOF
#endif

#ifdef GPIOG
,GPIOG
#endif
}; 

const uint8 ceAdChannelxArray[][16] =
{
    {0x00,0x01,0x02,0x03,0x04,0x05,0x06,0x07,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00},
    {0x08,0x09,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00},
    {0x0A,0x0B,0x0C,0x0D,0x0E,0x0F,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00},
    {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00},

#ifdef GPIOE
    {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00},
#endif

#ifdef GPIOF
    {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00},
#endif

#ifdef GPIOG
    {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00},
#endif
};



#ifdef __CE_CHECK_PAR__
/**
  * @brief   检验Ad属性对象的值是否正确
  * @param   ceAd:Ad属性对象指针
  * @return  系统状态码，可能的返回值:CE_STATUS_SUCCESS、CE_STATUS_RESOURCE_ERROR、CE_STATUS_NULL_POINTER
  */
CE_STATUS ceCheckCeAd(CeAd* ceAd)
{
    if (ceAd == CE_NULL)
    {
        return CE_STATUS_NULL_POINTER;
    }
    if ((ceAd->ceResource & CE_RES_MARK_AD) != CE_RES_MARK_AD)
    {
        return CE_STATUS_RESOURCE_ERROR;
    }
    return CE_STATUS_SUCCESS;
}
#endif //__CE_CHECK_PAR__

/**
  * @brief   Ad采集完成后，DMA传输完成后的回调
  * @param   None
  * @return  None
  */
void DMA1_Channel1_IRQHandler()
{
}

/**
  * @brief   初始化Ad转换
  * @param   ceAd:Ad属性对象指针
  * @return  系统状态码，可能的返回值:CE_STATUS_SUCCESS、CE_STATUS_RESOURCE_ERROR、CE_STATUS_NULL_POINTER
  */
CE_STATUS ceAd_initial(CeAd* ceAd)
{
    GPIO_InitTypeDef GPIO_InitStructure;
#ifdef __CE_CHECK_PAR__
    ce_assert_failed((uint8*)__FILE__, __LINE__, ceCheckCeAd(ceAd));
#endif //__CE_CHECK_PAR__

    RCC_APB2PeriphClockCmd(0x00000004 << ((ceAd->ceResource>>4) & 0x0000000F) , ENABLE);
    ceAd->ceExPar.ceExGpiox = ceAdGpioxArray[(ceAd->ceResource>>4) & 0x0000000F];
    ceAd->ceExPar.ceExGpioPinx = (uint16)(0x0001) << (ceAd->ceResource & 0x0000000F);
    ceAd->ceExPar.ceAdChannelx =ceAdChannelxArray[(ceAd->ceResource & 0x000000F0) >> 4][ceAd->ceResource & 0x0000000F];

    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;//获取对应的Ad通道的Goio口，并初始化Gpio
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Pin = ceAd->ceExPar.ceExGpioPinx;
    GPIO_Init(ceAd->ceExPar.ceExGpiox, &GPIO_InitStructure);

    if(ceAd_status == CE_AD_NOT_INITIAL)
    {
        ADC_InitTypeDef ADC_InitStructure;
        ceAd_status = CE_AD_IS_IDLE;

        RCC_ADCCLKConfig(RCC_PCLK2_Div4);//设置ADC工作时钟
        RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);//使能ADC时钟

        ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;
        ADC_InitStructure.ADC_ScanConvMode = DISABLE;//单次转换模式
        ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;
        ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;//转换不受外界决定
        ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;//数据右对齐
        ADC_InitStructure.ADC_NbrOfChannel = 1;//扫描通道输
        ADC_Init(ADC1, &ADC_InitStructure);//初始化ADC
        ADC_Cmd(ADC1, ENABLE);//使能或者失能指定的ADC

        ADC_ResetCalibration(ADC1);
        while (ADC_GetResetCalibrationStatus(ADC1));
        ADC_StartCalibration(ADC1);
        while (ADC_GetCalibrationStatus(ADC1));
    }
    return CE_STATUS_SUCCESS;
}

/**
  * @brief   开始Ad转换并立即获得Ad转换结果
  * @param   None
  * @return  返回Ad转换结果
  */
uint32 ceAd_getConvertValue(CeAd* ceAd)
{
    uint32 adConvertValue = 0x00000000;
    while(ceAd_status == CE_AD_IS_BUSY)
    {
        ceSystemOp.delayMs(0);
    }
    ceAd_status = CE_AD_IS_BUSY;

    ADC_RegularChannelConfig(ADC1, ceAd->ceExPar.ceAdChannelx, 1, ADC_SampleTime_1Cycles5);//配置转换通道等
    ADC_ClearFlag(ADC1, ADC_FLAG_EOC);//清除标志
    ADC_SoftwareStartConvCmd(ADC1, ENABLE);//使能或者失能指定的ADC的软件转换启动功能
    while(ADC_GetFlagStatus(ADC1,ADC_FLAG_EOC) == RESET);//等待转换结束
    adConvertValue = ADC_GetConversionValue(ADC1);//获取ADC转换值
    ADC_SoftwareStartConvCmd(ADC1, DISABLE);//使能或者失能指定的ADC的软件转换启动功能
    ceAd_status = CE_AD_IS_IDLE;
    return adConvertValue;
}

const CeAdOp ceAdOp = {ceAd_initial, ceAd_getConvertValue};

#ifdef __cplusplus
 }
#endif  //__cplusplus
