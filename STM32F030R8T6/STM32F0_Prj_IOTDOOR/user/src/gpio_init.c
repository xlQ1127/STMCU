#include "gpio_init.h"
#include "systemstatus.h"

static __IO uint32_t AsynchPrediv, SynchPrediv;


void LPW_GPIO_Set(void)
{
         GPIO_InitTypeDef GPIO_InitStructure;
         RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);
         RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOB, ENABLE);
         RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOC, ENABLE);
         RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOD, ENABLE);
         RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOF, ENABLE);
        
         GPIO_InitStructure.GPIO_Pin = GPIO_Pin_All;
         GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
         GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
         GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN; 
         GPIO_Init(GPIOA, &GPIO_InitStructure);
         GPIO_Init(GPIOB, &GPIO_InitStructure);
         GPIO_Init(GPIOC, &GPIO_InitStructure);
         GPIO_Init(GPIOD, &GPIO_InitStructure);
         GPIO_Init(GPIOF, &GPIO_InitStructure);
         
         GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;
	 GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	 GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	 GPIO_InitStructure.GPIO_Speed = GPIO_Speed_Level_3;
	 GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	 GPIO_Init(GPIOC, &GPIO_InitStructure);
         VDD_OFF;
}


void BoardGPIO_init(void)
{
	GPIO_InitTypeDef      GPIO_InitStruct;
	EXTI_InitTypeDef        EXTI_InitStructure;
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);
        RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOB, ENABLE);
        RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOC, ENABLE);
        RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOD, ENABLE);
        RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);

	/////指示灯   A9  A10
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_9 | GPIO_Pin_10;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_Level_2;
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOA, &GPIO_InitStruct);
        LED0_OFF;
        LED1_OFF;

        
	/////电机使能  C2 C3
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_2 | GPIO_Pin_3;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_Level_2;
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOC, &GPIO_InitStruct);

        
	/////红外灯发射 C6 A6
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_6;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_Level_2;
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOC, &GPIO_InitStruct);       
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_6;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_Level_2;
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOA, &GPIO_InitStruct);
        //////红外接收
        GPIO_InitStruct.GPIO_Pin = GPIO_Pin_7;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IN;
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_DOWN;
	GPIO_Init(GPIOA, &GPIO_InitStruct);
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_10;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IN;
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_DOWN;
	GPIO_Init(GPIOC, &GPIO_InitStruct);

        
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IN;
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOC, &GPIO_InitStruct);
        
	/////声音模块 B3 B4 B5
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_3 | GPIO_Pin_4 | GPIO_Pin_5 |GPIO_Pin_8;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_Level_3;
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOB, &GPIO_InitStruct);
        
        
        GPIO_InitStruct.GPIO_Pin = GPIO_Pin_8;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_Level_3;
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOB, &GPIO_InitStruct);

        
	////usart 2
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource2, GPIO_AF_1);
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource3, GPIO_AF_1);
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_2;//TX
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(GPIOA, &GPIO_InitStruct);
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_3;//RX
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;
	GPIO_Init(GPIOA, &GPIO_InitStruct);

        
	////usart 1
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource2, GPIO_AF_1);
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource3, GPIO_AF_1);
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_6;//TX
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(GPIOB, &GPIO_InitStruct);
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_7;//RX
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;
	GPIO_Init(GPIOB, &GPIO_InitStruct);

        
	////ADC A1 A4 A5
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_1 | GPIO_Pin_4 | GPIO_Pin_5;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AN;
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOA, &GPIO_InitStruct);
        
        
        /////外部唤醒中断   C0 A8
        GPIO_InitStruct.GPIO_Pin = GPIO_Pin_8;
        GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
        GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IN;
        GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_UP;
        GPIO_Init(GPIOA, &GPIO_InitStruct);
        
        
        GPIO_InitStruct.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15;
        GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
        GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IN;
        GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_UP;
        GPIO_Init(GPIOC, &GPIO_InitStruct);
       
      
        
        SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOC, EXTI_PinSource0 | EXTI_PinSource13);
        SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOA, EXTI_PinSource8);
        
        
        
        EXTI_InitStructure.EXTI_Line = EXTI_Line0 | EXTI_Line8 | EXTI_Line13;
        EXTI_InitStructure.EXTI_LineCmd = ENABLE;
        EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
        EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;
        EXTI_Init(&EXTI_InitStructure);
        
        SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOC, EXTI_PinSource14);
        EXTI_InitStructure.EXTI_Line = EXTI_Line14;
        EXTI_InitStructure.EXTI_LineCmd = ENABLE;
        EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
        EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;
        EXTI_Init(&EXTI_InitStructure);
        
        SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOC, EXTI_PinSource15);
        EXTI_InitStructure.EXTI_Line = EXTI_Line15;
        EXTI_InitStructure.EXTI_LineCmd = ENABLE;
        EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
        EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;
        EXTI_Init(&EXTI_InitStructure);
	return;
}


void uart1_init(void)
{
	RCC->APB2ENR |= RCC_APB2ENR_USART1EN;
	RCC->APB2RSTR |= RCC_APB2RSTR_USART1RST;
	RCC->APB2RSTR &= ~(RCC_APB2RSTR_USART1RST);
	if (USART1->CR1 & (1 << 15))
		USART1->BRR = 0x340;
	else
		USART1->BRR = 0x1A1;
	USART1->CR1 |= (1 << 0) | (1 << 2) | (1 << 3)
		| (1 << 5);
	return;
}


void uart2_init(void)
{
	RCC->APB1ENR |= RCC_APB1ENR_USART2EN;
	RCC->APB1RSTR |= RCC_APB1RSTR_USART2RST;
	RCC->APB1RSTR &= ~(RCC_APB1RSTR_USART2RST);
	if (USART2->CR1 & (1 << 15))
		USART2->BRR = 0x2710;
	else
		USART2->BRR = 0x1388;
	USART2->CR1 |= (1 << 0) | (1 << 2) | (1 << 3)
		| (1 << 5);
	return;
}


void adc_init(void)
{
	DMA_InitTypeDef     DMA_InitStructure;
	ADC_InitTypeDef     ADC_InitStructure;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);

	ADC_DeInit(ADC1);
	DMA_DeInit(DMA1_Channel1);
	DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)(&ADC1->DR);
	DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)SystemData.ADBuff;
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;
	DMA_InitStructure.DMA_BufferSize = 3;
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
	DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
	DMA_InitStructure.DMA_Priority = DMA_Priority_High;
	DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
	DMA_Init(DMA1_Channel1, &DMA_InitStructure);

	DMA_Cmd(DMA1_Channel1, ENABLE);
	ADC_DMARequestModeConfig(ADC1, ADC_DMAMode_Circular);
	ADC_DMACmd(ADC1, ENABLE);

	ADC_StructInit(&ADC_InitStructure);
	ADC_InitStructure.ADC_Resolution = ADC_Resolution_8b;
	ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;
	ADC_InitStructure.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_None;
	ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
	ADC_InitStructure.ADC_ScanDirection = ADC_ScanDirection_Backward;
	ADC_Init(ADC1, &ADC_InitStructure);

	ADC_ChannelConfig(ADC1, ADC_Channel_1, ADC_SampleTime_239_5Cycles);
	ADC_ChannelConfig(ADC1, ADC_Channel_4, ADC_SampleTime_239_5Cycles);
	ADC_ChannelConfig(ADC1, ADC_Channel_5, ADC_SampleTime_239_5Cycles);

	ADC_GetCalibrationFactor(ADC1);
	ADC_Cmd(ADC1, ENABLE);
	while (!ADC_GetFlagStatus(ADC1, ADC_FLAG_ADEN));
	ADC_StartOfConversion(ADC1);
	return;
}


void TIM1_init(uint16 ms)
{
	RCC->APB2ENR |= RCC_APB2ENR_TIM1EN;
	TIM1->PSC = 4799;
	TIM1->ARR = 10 * ms;
	TIM1->DIER |= 1 << 0;
	TIM1->CR1 |= 0X01;
}


static void rtc_config(void)
{
#if defined (RTC_CLOCK_SOURCE_LSI)

	RCC_LSICmd(ENABLE);                    /*启动LSI晶振*/

	while (RCC_GetFlagStatus(RCC_FLAG_LSIRDY) == RESET); /*等待LSI就绪*/

	RCC_RTCCLKConfig(RCC_RTCCLKSource_LSI);  /*选择LSI时钟*/

	/*RTC_LSI 为40kHz   RTC_LSI = SynchPrediv * AsynchPrediv*/
	SynchPrediv = 0x18F;
	AsynchPrediv = 0x63;

#elif defined (RTC_CLOCK_SOURCE_LSE)

	RCC_LSEConfig(RCC_LSE_ON);                          /*启动LSE晶振*/

	while (RCC_GetFlagStatus(RCC_FLAG_LSERDY) == RESET); /*等待LSE就绪*/

	RCC_RTCCLKConfig(RCC_RTCCLKSource_LSE);             /*选择LSE时钟*/

	/*RTC_LSE 为32.768kHz   RTC_LSI = SynchPrediv * AsynchPrediv*/
	SynchPrediv = 0xFF;
	AsynchPrediv = 0x7F;

#else
#error Please select the RTC Clock source inside the main.c file
#endif

	RCC_RTCCLKCmd(ENABLE);
	RTC_WaitForSynchro();
}


void rtc_init(void)
{
#define RTC_FLAG_BKP 0x0505  

	RTC_DateTimeTypeDef RTC_DateTimeStructure;
	RTC_InitTypeDef RTC_InitStructure;

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR, ENABLE);
	PWR_BackupAccessCmd(ENABLE);
	if (RTC_ReadBackupRegister(RTC_BKP_DR0) != RTC_FLAG_BKP)
	{
		rtc_config();
		RTC_InitStructure.RTC_AsynchPrediv = AsynchPrediv;
		RTC_InitStructure.RTC_SynchPrediv = SynchPrediv;
		RTC_InitStructure.RTC_HourFormat = RTC_HourFormat_24;
		if (RTC_Init(&RTC_InitStructure) == ERROR)
		{
			while (1);
		}
		RTC_DateTimeStructure.Year = 18;
		RTC_DateTimeStructure.Month = 8;
		RTC_DateTimeStructure.Date = 18;
		RTC_DateTimeStructure.Week = 6;
		RTC_DateTimeStructure.Hour = 0;
		RTC_DateTimeStructure.Minute = 0;
		RTC_DateTimeStructure.Second = 0;
		RTC_SetDateTime(RTC_DateTimeStructure);
		RTC_WriteBackupRegister(RTC_BKP_DR0, RTC_FLAG_BKP);
	}
	else
	{
#ifdef RTC_CLOCK_SOURCE_LSI
		RCC_LSICmd(ENABLE);
#endif
		RTC_WaitForSynchro();                            /*等待RTC与RTC_APB时钟同步*/
	}
	return;
}


void SleepMode(void)
{
  LPW_GPIO_Set();
  ADC_Cmd(ADC1, DISABLE);
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR, ENABLE);  
  PWR_EnterSTOPMode(PWR_Regulator_LowPower, PWR_STOPEntry_WFI);
}