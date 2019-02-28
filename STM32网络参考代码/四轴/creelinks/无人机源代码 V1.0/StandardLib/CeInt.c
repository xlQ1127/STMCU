/**
  ******************************************************************************
  * @file    CeInt.c
  * @author  Creelinks Application Team
  * @version V1.0.0
  * @date    2016-08-05
  * @brief   基于STM32F103RET6处理器平台的CeUart资源函数实现库文件
  ******************************************************************************
  * @attention
  *
  *1)不同的Int资源，有不同的中断优先级，用户可以宏定义中配置,详细配置规则请参考Stm32f103数据手删
  *2)每个Int的回调均在中断内进行，因此请勿在回调中执行耗时操作
  *
  * <h2><center>&copy; COPYRIGHT 2016 Darcern</center></h2>
  ******************************************************************************
  */
#include "CeInt.h"
#ifdef __cplusplus
 extern "C" {
#endif //__cplusplus

GPIO_TypeDef*   ceIntGpioxArray[] = {GPIOA,GPIOB,GPIOC,GPIOD
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

const uint8 ceIntPreemptionAndSubPriority[][16] = {     /*!< 资源号对应的中断优先级，如PB5对应[1][5],0x31高四位表示抢占式优先级，低四位表响应式优先级*/
    {0x31,0x31,0x31,0x31,0x31,0x31,0x31,0x31,0x31,0x31,0x31,0x31,0x31,0x31,0x31,0x31}
    ,{0x31,0x31,0x31,0x31,0x31,0x31,0x31,0x31,0x31,0x31,0x31,0x31,0x31,0x31,0x31,0x31}
    ,{0x31,0x31,0x31,0x31,0x31,0x31,0x31,0x31,0x31,0x31,0x31,0x31,0x31,0x31,0x31,0x31}
    ,{0x31,0x31,0x31,0x31,0x31,0x31,0x31,0x31,0x31,0x31,0x31,0x31,0x31,0x31,0x31,0x31}
    #ifdef GPIOE
    ,{0x31,0x31,0x31,0x31,0x31,0x31,0x31,0x31,0x31,0x31,0x31,0x31,0x31,0x31,0x31,0x31}
    #endif

    #ifdef GPIOF
    ,{0x31,0x31,0x31,0x31,0x31,0x31,0x31,0x31,0x31,0x31,0x31,0x31,0x31,0x31,0x31,0x31}
    #endif

    #ifdef GPIOG
    ,{0x31,0x31,0x31,0x31,0x31,0x31,0x31,0x31,0x31,0x31,0x31,0x31,0x31,0x31,0x31,0x31}
    #endif
};

/**
  * @brief  结构体，外部中断Int暂存集合
  */
CeInt* ceIntList[] = {CE_NULL, CE_NULL, CE_NULL, CE_NULL, CE_NULL, CE_NULL, CE_NULL, CE_NULL, CE_NULL, CE_NULL, CE_NULL, CE_NULL, CE_NULL, CE_NULL, CE_NULL, CE_NULL};


#ifdef __CE_CHECK_PAR__
/**
  * @brief   检验ceInt指针参数
  * @param   ceInt:外部中断Int属性对象指针
  * @return  系统状态码，可能的值:CE_STATUS_SUCCESS、CE_STATUS_RESOURCE_ERROR、CE_STATUS_NULL_POINTER
  */
CE_STATUS ceCheckCeInt(CeInt* ceInt)
{
    if (ceInt == CE_NULL)
    {
        return CE_STATUS_NULL_POINTER;
    }
    if (ceInt->callBack == CE_NULL)
    {
        return CE_STATUS_NULL_POINTER;
    }
    if (((ceInt->ceResource & CE_RES_MARK_INT) != CE_RES_MARK_INT))
    {
        return CE_STATUS_RESOURCE_ERROR;
    }
    return CE_STATUS_SUCCESS;
}
#endif //__CE_CHECK_PAR__

/**
  * @brief   外部中断Int中断线0的入口函数
  * @param   None
  * @return  None
  */
void EXTI0_IRQHandler(void)
{
    if (EXTI_GetFlagStatus(EXTI_Line0) != RESET)
    {
        EXTI_ClearFlag(EXTI_Line0);
        if(ceIntList[0]->ceExIntPar.ceExIsStart)
        {
            ceIntList[0]->callBack( ceIntList[0]->pAddPar);
        }
    }
}

/**
  * @brief   外部中断Int中断线1的入口函数
  * @param   None
  * @return  None
  */
void EXTI1_IRQHandler(void)
{
    if (EXTI_GetFlagStatus(EXTI_Line1) != RESET)
    {
        EXTI_ClearFlag(EXTI_Line1);
        if(ceIntList[1]->ceExIntPar.ceExIsStart)
        {
            ceIntList[1]->callBack(ceIntList[1]->pAddPar);
        }
    }
}

/**
  * @brief   外部中断Int中断线2的入口函数
  * @param   None
  * @return  None
  */
void EXTI2_IRQHandler(void)
{
    if (EXTI_GetFlagStatus(EXTI_Line2) != RESET)
    {
        EXTI_ClearFlag(EXTI_Line2);
        if(ceIntList[2]->ceExIntPar.ceExIsStart)
        {
            ceIntList[2]->callBack(ceIntList[2]->pAddPar);
        }
    }
}

/**
  * @brief   外部中断Int中断线3的入口函数
  * @param   None
  * @return  None
  */
void EXTI3_IRQHandler(void)
{
    if (EXTI_GetFlagStatus(EXTI_Line3) != RESET)
    {
        EXTI_ClearFlag(EXTI_Line3);
        if(ceIntList[3]->ceExIntPar.ceExIsStart)
        {
            ceIntList[3]->callBack( ceIntList[3]->pAddPar);
        }
    }
}

/**
  * @brief   外部中断Int中断线4的入口函数
  * @param   None
  * @return  None
  */
void EXTI4_IRQHandler(void)
{
    if (EXTI_GetFlagStatus(EXTI_Line4) != RESET)
    {
        EXTI_ClearFlag(EXTI_Line4);
        if(ceIntList[4]->ceExIntPar.ceExIsStart)
        {
            ceIntList[4]->callBack(ceIntList[4]->pAddPar);
        }
    }
}

/**
  * @brief   外部中断Int中断线5、6、7、8、9的入口函数
  * @param   None
  * @return  None
  */
void EXTI9_5_IRQHandler(void)
{
    if (EXTI_GetFlagStatus(EXTI_Line5) != RESET)
    {
        EXTI_ClearFlag(EXTI_Line5);
        if(ceIntList[5]->ceExIntPar.ceExIsStart)
            ceIntList[5]->callBack( ceIntList[5]->pAddPar);
    } 
    else if (EXTI_GetFlagStatus(EXTI_Line6) != RESET)
    {
        EXTI_ClearFlag(EXTI_Line6);
        if(ceIntList[6]->ceExIntPar.ceExIsStart)
            ceIntList[6]->callBack(ceIntList[6]->pAddPar);
    } 
    else if (EXTI_GetFlagStatus(EXTI_Line7) != RESET)
    {
        EXTI_ClearFlag(EXTI_Line7);//即使没有用到此中断，但还是保留判断中断标志及清除中断标志的代码，以防未知原因导致中断过来而又没有清除中断导致的中断多次到来
        if(ceIntList[7]->ceExIntPar.ceExIsStart)
            ceIntList[7]->callBack(ceIntList[7]->pAddPar);    
    } 
    else if (EXTI_GetFlagStatus(EXTI_Line8) != RESET)
    {
        EXTI_ClearFlag(EXTI_Line8);
        if(ceIntList[8]->ceExIntPar.ceExIsStart)
            ceIntList[8]->callBack(ceIntList[8]->pAddPar);
    } 
    else if (EXTI_GetFlagStatus(EXTI_Line9) != RESET)
    {
        EXTI_ClearFlag(EXTI_Line9);       
        if(ceIntList[9]->ceExIntPar.ceExIsStart)
            ceIntList[9]->callBack(ceIntList[9]->pAddPar);
    }
}

/**
  * @brief   外部中断Int中断线10、11、12、13、14、15的入口函数
  * @param   None
  * @return  None
  */
void EXTI15_10_IRQHandler(void)
{
    if (EXTI_GetFlagStatus(EXTI_Line10) != RESET)
    {
        EXTI_ClearFlag(EXTI_Line10);
        if(ceIntList[10]->ceExIntPar.ceExIsStart)
        {
            ceIntList[10]->callBack( ceIntList[10]->pAddPar);
        }
    } 
    else if (EXTI_GetFlagStatus(EXTI_Line11) != RESET)
    {
        EXTI_ClearFlag(EXTI_Line11);
        if(ceIntList[11]->ceExIntPar.ceExIsStart)
        {
            ceIntList[11]->callBack(ceIntList[11]->pAddPar);
        }
    } 
    else if (EXTI_GetFlagStatus(EXTI_Line12) != RESET)
    {
        EXTI_ClearFlag(EXTI_Line12);
        if(ceIntList[12]->ceExIntPar.ceExIsStart)
        {
            ceIntList[12]->callBack(ceIntList[12]->pAddPar);
        }
    } 
    else if (EXTI_GetFlagStatus(EXTI_Line13) != RESET)
    {
        EXTI_ClearFlag(EXTI_Line13);//虽然没有用到此中断，但还是保留判断中断标志及清除中断标志的代码，以防未知原因导致中断过来而又没有清除中断导致的中断多次到来
        if(ceIntList[13]->ceExIntPar.ceExIsStart)
        {
            ceIntList[13]->callBack(ceIntList[13]->pAddPar);
        }
    } 
    else if (EXTI_GetFlagStatus(EXTI_Line14) != RESET)
    {
        EXTI_ClearFlag(EXTI_Line14);//同上
        if(ceIntList[14]->ceExIntPar.ceExIsStart)
        {
            ceIntList[14]->callBack(ceIntList[14]->pAddPar);
        }
    } 
    else if (EXTI_GetFlagStatus(EXTI_Line15) != RESET)
    {
        EXTI_ClearFlag(EXTI_Line15);//同上
        if(ceIntList[15]->ceExIntPar.ceExIsStart)
        {
            ceIntList[15]->callBack(ceIntList[15]->pAddPar);
        }
    }
}

/**
  * @brief   初始化外部中断Int
  * @param   ceInt:外部中断Int属性对象指针
  * @return  系统状态码，可能的值:CE_STATUS_SUCCESS、CE_STATUS_RESOURCE_ERROR、CE_STATUS_NULL_POINTER
  */
CE_STATUS ceInt_initial(CeInt* ceInt)
{
    NVIC_InitTypeDef* NVIC_InitStructure = &(ceInt->ceExIntPar.ceExNVIC_InitStructure);
    GPIO_InitTypeDef GPIO_InitStructure;
#ifdef __CE_CHECK_PAR__
    ce_assert_failed((uint8*)__FILE__, __LINE__, ceCheckCeInt(ceInt));
#endif //__CE_CHECK_PAR__


    GPIO_InitStructure.GPIO_Pin = (uint16)(0x0001) << (ceInt->ceResource & 0x0000000F);
    ceInt->ceExIntPar.ceExGpiox  = ceIntGpioxArray[(ceInt->ceResource>>4) & 0x0000000F];
    ceInt->ceExIntPar.ceExEXTI_InitStructure.EXTI_Line = 0x00000001 << ((ceInt->ceResource & 0x0000000F));
    ceIntList[ceInt->ceResource & 0x0000000F] = ceInt;
    NVIC_InitStructure->NVIC_IRQChannelPreemptionPriority = ceIntPreemptionAndSubPriority[(ceInt->ceResource>>4) & 0x0000000F][ceInt->ceResource & 0x0000000F] >>4;
    NVIC_InitStructure->NVIC_IRQChannelSubPriority = ceIntPreemptionAndSubPriority[(ceInt->ceResource>>4) & 0x0000000F][ceInt->ceResource & 0x0000000F] &0x0F;
    if((ceInt->ceResource) & (0x0000000F <= 4))
    {
        NVIC_InitStructure->NVIC_IRQChannel = (IRQn_Type)((ceInt->ceResource & 0x0000000F)+6);
    }else if(ceInt->ceResource & (0x0000000F <= 9))
    {
        NVIC_InitStructure->NVIC_IRQChannel = EXTI9_5_IRQn;
    }else
    {
        NVIC_InitStructure->NVIC_IRQChannel = EXTI15_10_IRQn;
    }

    switch (ceInt->ceIntMode)//配置中断触发的模式
    {
    case CE_INT_MODE_TRIGGER_FALLING:
        ceInt->ceExIntPar.ceExEXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;
        break;
    case CE_INT_MODE_TRIGGER_RISING:
        ceInt->ceExIntPar.ceExEXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;
        break;
    default:
        return CE_STATUS_INITIAL_FALSE;
    }

    ceInt->ceExIntPar.ceExGpioPinx = GPIO_InitStructure.GPIO_Pin;//获取之前设置的GpioPin
    ceInt->ceExIntPar.ceExIsStart = 0x00;//初始化时设置外部中断Int为非工作状态

    RCC_APB2PeriphClockCmd(0x00000004 << ((ceInt->ceResource>>4) & 0x0000000F) , ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;

    GPIO_Init(ceInt->ceExIntPar.ceExGpiox, &GPIO_InitStructure);
    GPIO_EXTILineConfig((ceInt->ceResource>>4) & 0x0000000F, ceInt->ceResource & 0x0000000F);

    ceInt->ceExIntPar.ceExEXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
    ceInt->ceExIntPar.ceExEXTI_InitStructure.EXTI_LineCmd = ENABLE;
    EXTI_Init(&(ceInt->ceExIntPar.ceExEXTI_InitStructure));
    EXTI_ClearFlag((uint32)(ceInt->ceExIntPar.ceExGpioPinx));

    NVIC_InitStructure->NVIC_IRQChannelCmd = DISABLE;//开始的时候关闭外部中断Int
    NVIC_Init(NVIC_InitStructure);

    EXTI_GenerateSWInterrupt(ceInt->ceExIntPar.ceExEXTI_InitStructure.EXTI_Line);//产生一个软件中断，这也能解释为什么之前第一次使用外部中断Int总是先来一次中断的现象了。此代码可以删除，但开始和停止外部中断Int操作前后务必清除中断标志

    return CE_STATUS_SUCCESS;
}

/**
  * @brief  配置中断方式
  * @param  ceInt:外部中断Int属性对象指针
  * @param  ceIntMode:中断方式
  * @return None
  */
void ceInt_setMode(CeInt* ceInt,  CE_INT_MODE ceIntMode)
{
    uint32 tmp = 0;
#ifdef __CE_CHECK_PAR__
    ce_assert_failed((uint8*)__FILE__, __LINE__, ceCheckCeInt(ceInt));
#endif //__CE_CHECK_PAR__

    if (ceIntMode == CE_INT_MODE_TRIGGER_FALLING)
    {
        tmp = (uint32_t)EXTI_BASE;
        tmp += EXTI_Trigger_Falling;
        *(__IO uint32 *) tmp |= ceInt->ceExIntPar.ceExEXTI_InitStructure.EXTI_Line;
    }
    else
    {
        tmp = (uint32_t)EXTI_BASE;
        tmp += EXTI_Trigger_Rising;

        *(__IO uint32 *) tmp |= ceInt->ceExIntPar.ceExEXTI_InitStructure.EXTI_Line;
    }
}

/**
  * @brief   开始外部中断Int监测
  * @param   ceInt:外部中断Int属性对象指针
  * @return  None
  */
void ceInt_start(CeInt* ceInt)
{
#ifdef __CE_CHECK_PAR__
    ce_assert_failed((uint8*)__FILE__, __LINE__, ceCheckCeInt(ceInt));
#endif //__CE_CHECK_PAR__
    ceInt->ceExIntPar.ceExIsStart = 0x01;//设置外部中断Int为工作状态
    EXTI_ClearFlag((uint32)(ceInt->ceExIntPar.ceExGpioPinx));//在开中断前和中断后务必清一下中断（预防管脚上的信号抖动），不然初始化后第一次开中断会来一次中断。由于EXTI_LineX和GPIO_Pin_X都是用X位为1表示某个Line或Pin，所有这里直接用的GPIO_Pin_X，省了1个成员变量。
    ceInt->ceExIntPar.ceExNVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&(ceInt->ceExIntPar.ceExNVIC_InitStructure));
}

/**
  * @brief   停止外部中断Int监测
  * @param   ceInt:外部中断Int属性对象指针
  * @return  None
  */
void ceInt_stop(CeInt* ceInt)
{
#ifdef __CE_CHECK_PAR__
    ce_assert_failed((uint8*)__FILE__, __LINE__, ceCheckCeInt(ceInt));
#endif //__CE_CHECK_PAR__

    ceInt->ceExIntPar.ceExIsStart = 0x00;//设置外部中断Int为不工作状态

    if(ceInt->ceExIntPar.ceExGpioPinx > GPIO_Pin_4 && ceInt->ceExIntPar.ceExGpioPinx <= GPIO_Pin_9)//EXTI9_5_IRQn
    {
        if(ceIntList[5]->ceExIntPar.ceExIsStart || ceIntList[6]->ceExIntPar.ceExIsStart || ceIntList[7]->ceExIntPar.ceExIsStart || ceIntList[8]->ceExIntPar.ceExIsStart || ceIntList[9]->ceExIntPar.ceExIsStart)      
        {
            return;//如果至少有1个外部中断Int在开始状态，则不做任何操作
        }
    }
    else if(ceInt->ceExIntPar.ceExGpioPinx >= GPIO_Pin_10)//EXTI15_10_IRQn
    {
        if(ceIntList[10]->ceExIntPar.ceExIsStart || ceIntList[11]->ceExIntPar.ceExIsStart || ceIntList[12]->ceExIntPar.ceExIsStart
            || ceIntList[13]->ceExIntPar.ceExIsStart || ceIntList[14]->ceExIntPar.ceExIsStart || ceIntList[15]->ceExIntPar.ceExIsStart)
        {
            return;//如果至少有1个外部中断Int在开始状态，则不做任何操作，否则执行函数最后的代码关闭此中断
        }
    }

    ceInt->ceExIntPar.ceExNVIC_InitStructure.NVIC_IRQChannelCmd = DISABLE;
    NVIC_Init(&(ceInt->ceExIntPar.ceExNVIC_InitStructure));
    EXTI_ClearFlag((uint32)(ceInt->ceExIntPar.ceExGpioPinx));//在开中断前和中断后务必清一下中断（预防管脚上的信号抖动），不然初始化后第一次开中断会来一次中断。由于EXTI_LineX和GPIO_Pin_X都是用X位为1表示某个Line或Pin，所有这里直接用的GPIO_Pin_X，省了1个成员变量。
}

/**
  * @brief   获取外部中断Int口对应的Gpio的电平值，0x01和0x00
  * @param   ceInt:外部中断Int属性对象指针
  * @return  Gpio的电平值，0x01和0x00
  */
uint8 ceInt_getBit(CeInt* ceInt)
{
#ifdef __CE_CHECK_PAR__
    ce_assert_failed((uint8*)__FILE__, __LINE__, ceCheckCeInt(ceInt));
#endif //__CE_CHECK_PAR__
    return GPIO_ReadInputDataBit(ceInt->ceExIntPar.ceExGpiox, ceInt->ceExIntPar.ceExGpioPinx);
}

const CeIntOp ceIntOp = {ceInt_initial, ceInt_setMode, ceInt_start, ceInt_stop, ceInt_getBit};

#ifdef __cplusplus
 }
#endif //__cplusplus
