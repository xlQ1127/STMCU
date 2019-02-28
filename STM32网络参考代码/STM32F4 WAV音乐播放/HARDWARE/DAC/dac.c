#include "dac.h"

extern u16 digital;
void MyDAC_Init(void)//DAC channel1 Configuration
{
  	unsigned int tmpreg1=0,tmpreg2=0;
 	RCC->APB2ENR|=1<<2;//使能PORTA时钟
	RCC->APB1ENR|=RCC_APB1Periph_DAC;//使能DAC时钟
 	GPIOA->CRL&=0XFF00FFFF; 
	GPIOA->CRL|=0X00440000;//PA4,5 浮空输入   	 

  	tmpreg1=DAC->CR;//Get the DAC CR value  
  	tmpreg1&=~(CR_CLEAR_Mask<<DAC_Channel_1);//Clear BOFFx, TENx, TSELx, WAVEx and MAMPx bits  
  	tmpreg2=(DAC_Trigger_Software|DAC_WaveGeneration_None|DAC_LFSRUnmask_Bits8_0|DAC_OutputBuffer_Enable); 
  	tmpreg1|=tmpreg2<<DAC_Channel_1;//Calculate CR register value depending on DAC_Channel 
  	DAC->CR=tmpreg1;//Write to DAC CR 
	DAC->CR|=CR_EN_Set<<DAC_Channel_1;//DAC Channel1使能,PA4自动连接到DAC
	DAC1_SetData(0x000);

  	tmpreg1=DAC->CR;//Get the DAC CR value  
  	tmpreg1&=~(CR_CLEAR_Mask<<DAC_Channel_2);//Clear BOFFx, TENx, TSELx, WAVEx and MAMPx bits  
  	tmpreg2=(DAC_Trigger_Software|DAC_WaveGeneration_None|DAC_LFSRUnmask_Bits8_0|DAC_OutputBuffer_Enable); 
  	tmpreg1|=tmpreg2<<DAC_Channel_2;//Calculate CR register value depending on DAC_Channel 
  	DAC->CR=tmpreg1;//Write to DAC CR 
	DAC->CR|=CR_EN_Set<<DAC_Channel_2;//DAC Channel2使能,PA5自动连接到DAC
	DAC2_SetData(0x000);
}

void DAC1_SetData(u16 data)
{
	DAC->DHR12R1=data;//通道1的12位右对齐数据
	DAC->SWTRIGR|=0x01;//软件启动转换
}

void DAC2_SetData(u16 data)
{
	DAC->DHR12R2=data;//通道2的12位右对齐数据
	DAC->SWTRIGR|=0x02;//软件启动转换
}






