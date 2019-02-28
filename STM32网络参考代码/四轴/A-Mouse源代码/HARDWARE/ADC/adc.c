#include "adc.h"   
#include <stm32f10x.h>
u16 __IO ADCValues[5];
void Adc_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	ADC_InitTypeDef ADC_InitStruct;
	DMA_InitTypeDef DMA_InitStruct;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_ADC1 | RCC_APB2Periph_GPIOB, ENABLE);
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1,ENABLE);
								 
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_4 | GPIO_Pin_5 | GPIO_Pin_6;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
  GPIO_Init(GPIOA, &GPIO_InitStructure);

	RCC_ADCCLKConfig(RCC_PCLK2_Div6 );

	 DMA_DeInit(DMA1_Channel1);
	 DMA_InitStruct.DMA_PeripheralBaseAddr = (uint32_t) & (ADC1->DR); 
   DMA_InitStruct.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
   DMA_InitStruct.DMA_MemoryBaseAddr = (u32)&ADCValues;
   DMA_InitStruct.DMA_DIR = DMA_DIR_PeripheralSRC;
	 DMA_InitStruct.DMA_M2M = DMA_M2M_Disable;
	 DMA_InitStruct.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
	 DMA_InitStruct.DMA_MemoryInc = DMA_MemoryInc_Disable;
   DMA_InitStruct.DMA_MemoryInc = DMA_MemoryInc_Enable; 
	 DMA_InitStruct.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	 DMA_InitStruct.DMA_Mode  = DMA_Mode_Circular;
	 DMA_InitStruct.DMA_Priority = DMA_Priority_High;
	 DMA_InitStruct.DMA_BufferSize = 5;
	 DMA_Init(DMA1_Channel1,&DMA_InitStruct);
   DMA_Cmd(DMA1_Channel1, ENABLE);
	
	ADC_DeInit(ADC1);
	ADC_InitStruct.ADC_Mode = ADC_Mode_Independent;//定义ADC初始化结构体变量
  ADC_InitStruct.ADC_ScanConvMode = ENABLE;//使能扫描
 	ADC_InitStruct.ADC_ContinuousConvMode = ENABLE;//ADC转换工作在连续模式
  ADC_InitStruct.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;//由软件控制转换
  ADC_InitStruct.ADC_DataAlign = ADC_DataAlign_Right;//转换数据右对齐
  ADC_InitStruct.ADC_NbrOfChannel = 5;//转换序列长度为1
	ADC_Init(ADC1,&ADC_InitStruct);
	ADC_RegularChannelConfig(ADC1,ADC_Channel_0, 1, ADC_SampleTime_239Cycles5);
	ADC_RegularChannelConfig(ADC1,ADC_Channel_1, 2, ADC_SampleTime_239Cycles5);
	ADC_RegularChannelConfig(ADC1,ADC_Channel_4, 3, ADC_SampleTime_239Cycles5);
	ADC_RegularChannelConfig(ADC1,ADC_Channel_5, 4, ADC_SampleTime_239Cycles5);
	ADC_RegularChannelConfig(ADC1,ADC_Channel_6, 5, ADC_SampleTime_239Cycles5);
	
	ADC_DMACmd(ADC1,ENABLE);
	ADC_Cmd(ADC1, ENABLE);//使能ADC1   
  ADC_ResetCalibration(ADC1);//重置ADC1校准寄存器
  while(ADC_GetResetCalibrationStatus(ADC1));//等待ADC1校准重置完成
	ADC_StartCalibration(ADC1);//开始ADC1校准
  while(ADC_GetCalibrationStatus(ADC1));//等待ADC1校准完成   							    
}

void Get_Adc_Average(u16 adc[],u8 times)
{
   u32 temp2[5] = {0,0,0,0,0};
	 //u16 temp1[5] = {0,0,0,0,0};
   u8 t,i;
	 
   for(t=0;t<times;t++)
   {
		  for(i=0;i<times;i++)
			{
				temp2[i] += ADCValues[i];
			}
			Delay(5);
   }
	 for(i=0;i<times;i++)
	 {
			adc[i] = temp2[i]/times;
	 }
}

void Delay(unsigned int i)
{
	unsigned char j;
 	for(;i>0;i--)
		for(j = 255; j>0 ; j--);
}
