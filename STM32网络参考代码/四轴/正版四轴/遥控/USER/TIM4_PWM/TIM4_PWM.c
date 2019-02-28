#include "TIM4_PWM.h"

static void TIM4_GPIO_Config(void) 
{
		GPIO_InitTypeDef GPIO_InitStructure;
		/* TIM4 clock enable */
		//PCLK1经过2倍频后作为TIM4的时钟源等于36MHz
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE); 
		/*GPIOB clock enable */
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE); 
		/*GPIOA Configuration: TIM4 channel 1 2 3 and 4 as alternate function push-pull */
		GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_6 | GPIO_Pin_7 | GPIO_Pin_0 | GPIO_Pin_1;
		GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF_PP;		    // 复用推挽输出
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
		
		GPIO_Init(GPIOB, &GPIO_InitStructure);
} 
void TIM4_Mode_Config(u16 CCR1_Val,u16 CCR2_Val,u16 CCR3_Val,u16 CCR4_Val)   
{   
     TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;   
     TIM_OCInitTypeDef        TIM_OCInitStructure;          
     /* Time base configuration */
     TIM_TimeBaseStructure.TIM_Period = 999;    
     TIM_TimeBaseStructure.TIM_Prescaler = 0;      
     TIM_TimeBaseStructure.TIM_ClockDivision = 0;    
     TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up; 
	
     TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure); 
	
     /* PWM1 Mode configuration: Channel1 */  
     TIM_OCInitStructure.TIM_OCMode      = TIM_OCMode_PWM1;    
     TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;  
     TIM_OCInitStructure.TIM_Pulse       = CCR1_Val;    
     TIM_OCInitStructure.TIM_OCPolarity  = TIM_OCPolarity_High; 
	
     TIM_OC1Init(TIM4, &TIM_OCInitStructure);    
     TIM_OC1PreloadConfig(TIM4, TIM_OCPreload_Enable); 
	
     /* PWM1 Mode configuration: Channel2 */  
     TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
     TIM_OCInitStructure.TIM_Pulse       = CCR2_Val; 
		 
     TIM_OC2Init(TIM4, &TIM_OCInitStructure);    
     TIM_OC2PreloadConfig(TIM4, TIM_OCPreload_Enable);
		 
     /* PWM1 Mode configuration: Channel3 */  
     TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;  
     TIM_OCInitStructure.TIM_Pulse       = CCR3_Val; 
		 
     TIM_OC3Init(TIM4, &TIM_OCInitStructure);    
     TIM_OC3PreloadConfig(TIM4, TIM_OCPreload_Enable); 
		 
     /* PWM1 Mode configuration: Channel4 */  
     TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;  
     TIM_OCInitStructure.TIM_Pulse       = CCR4_Val; 
		 
     TIM_OC4Init(TIM4, &TIM_OCInitStructure);     
     TIM_OC4PreloadConfig(TIM4, TIM_OCPreload_Enable);
		 
     TIM_ARRPreloadConfig(TIM4, ENABLE);      
     /* TIM4 enable counter */  
     TIM_Cmd(TIM3, ENABLE);     
}
void TIM4_PWM_Init(void)   
{   
     TIM4_GPIO_Config();   
     TIM4_Mode_Config(0,0,0,0);    
}
