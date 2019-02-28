#include "adc.h"



/*
 * 函数名：ADC1_Init
 * 描述  ：ADC1初始化 (默认将开启通道0~3，单次转换模式)
 * 输入  ：无
 * 输出  ：无
 * 调用  ：外部调用
 */
void ADC1_Init(void)
{
	ADC_InitTypeDef ADC_InitStructure; 
	GPIO_InitTypeDef GPIO_InitStructure;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA|RCC_APB2Periph_ADC1,ENABLE);//开始ADC1和相应的IO口时钟
	RCC_ADCCLKConfig(RCC_PCLK2_Div6);//ADC时钟频率为系统频率的6分频 72/6=12MHz

	/* ADC1 IO口配置(PA0.1.2.3模拟输入) */                         
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0|GPIO_Pin_1|GPIO_Pin_2|GPIO_Pin_3;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;		//模拟输入引脚
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	/* ADC1 模式配置 */
	ADC_DeInit(ADC1);//将外设 ADC1 的全部寄存器重设为缺省值
	 
	ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;//ADC工作模式:ADC1和ADC2工作在独立模式 
	ADC_InitStructure.ADC_ScanConvMode = DISABLE;//模数转换工作在单通道模式 
	ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;//模数转换工作在单次转换模式 
	ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;//转换由软件而不是外部触发启动 
	ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;//ADC 数据右对齐  
	ADC_InitStructure.ADC_NbrOfChannel = 1;//顺序进行规则转换的ADC通道的数目 
	ADC_Init(ADC1, &ADC_InitStructure);//配置ADC1
	
	/*  */
	ADC_Cmd(ADC1,ENABLE);//开启ADC1转换
	
	ADC_ResetCalibration(ADC1);//重置指定的ADC1的校准寄存器
	while(ADC_GetResetCalibrationStatus(ADC1) != RESET);//获取ADC1重置校准寄存器的状态,设置状态则等待
	
	ADC_StartCalibration(ADC1);//开始指定ADC1的校准状态
	while(ADC_GetCalibrationStatus(ADC1) != RESET);//获取指定ADC1的校准程序,设置状态则等待
	
	//ADC_SoftwareStartConvCmd(ADC1, ENABLE);//使能指定的ADC1的软件转换启动功能	
}



/*
 * 函数名：Get_Adc
 * 描述  ：获得ADC值
 * 输入  ：ch:通道值 0~3
 * 输出  ：无
 * 调用  ：外部调用
 */
u16 Get_Adc(u8 ch)   
{
  	//设置指定ADC的规则组通道，设置它们的转化顺序和采样时间
	ADC_RegularChannelConfig(ADC1,ch,1,ADC_SampleTime_239Cycles5);	//ADC1,ADC通道ch,规则采样顺序值为1,采样时间为239.5周期
	ADC_SoftwareStartConvCmd(ADC1,ENABLE);		//使能指定的ADC1的软件转换启动功能	
	 
	while(ADC_GetFlagStatus(ADC1,ADC_FLAG_EOC) != 1);//等待转换结束
	return ADC_GetConversionValue(ADC1);//返回最近一次ADC1规则组的转换结果
}







