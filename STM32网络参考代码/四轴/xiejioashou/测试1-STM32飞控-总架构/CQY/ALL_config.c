#include "stm32f10x.h"

#include "ALL_config.h"
NVIC_InitTypeDef NVIC_InitStructure;

void RCC_Configuration(void)
{
	FLASH_PrefetchBufferCmd(FLASH_PrefetchBuffer_Enable);

  FLASH_SetLatency(FLASH_Latency_2);//设置flash延时
	/*  使用内部RC晶振 */
   RCC_HSICmd(ENABLE);//使能内部高速晶振 ;    
	 RCC_PLLConfig(RCC_PLLSource_HSI_Div2, RCC_PLLMul_16);//PLL倍率=16
   RCC_PLLCmd(ENABLE);
   RCC_SYSCLKConfig(RCC_SYSCLKSource_PLLCLK);//选择PLL时钟作为系统时钟SYSCLOCK=64MHZ 
   RCC_HCLKConfig(RCC_SYSCLK_Div1);//选择HCLK时钟源为系统时钟SYYSCLOCK
   RCC_PCLK1Config(RCC_HCLK_Div4);//APB1时钟64/4
   RCC_PCLK2Config(RCC_HCLK_Div4);//APB2时钟为64/4

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOE, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C1, ENABLE);
}



void Tim2_init()//PWM产生
{
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);	
	
	TIM2->ARR=1600;//设定计数器自动重装值
	TIM2->PSC=0;//预分频器不分频
	TIM2->CCMR1|=6<<4; //CH1 PWM2模式
	TIM2->CCMR1|=1<<3; //CH1预装载使能
	TIM2->CCMR1|=6<<12; //CH2 PWM2模式
	TIM2->CCMR1|=1<<11; //CH2预装载使能
	TIM2->CCMR2|=6<<4; //CH3 PWM2模式
	TIM2->CCMR2|=1<<3; //CH3预装载使能
	TIM2->CCMR2|=6<<12; //CH4 PWM2模式
	TIM2->CCMR2|=1<<11; //CH4预装载使能
	TIM2->CCER|=0x1111; //CH1234 输出使能
	TIM2->CR1|=0x01; //使能定时器2			
	
	TIM2->CCR1=10; //PWM占空比
	TIM2->CCR2=10;
	TIM2->CCR3=10;
	TIM2->CCR4=10;
	
}

 void TIM4_INT(void)
{    
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);
  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);  //用第2组分法 				
	NVIC_SetVectorTable(NVIC_VectTab_FLASH, 0x0); 	   
  NVIC_InitStructure.NVIC_IRQChannel = TIM4_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3; //抢占级别0-3   （嵌套）
	NVIC_InitStructure.NVIC_IRQChannelSubPriority =0;        //响应优先级0-3 （排队）
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure); 

	TIM4->ARR=5000; //设定计数器自动重装值  10000-20ms
	TIM4->PSC=31;   //预分频器64，得到500khz的计数时钟 
	TIM4->DIER|=1<<0;   //允许更新中断     
	TIM4->DIER|=1<<6;   //允许触发中断 
	TIM4->CR1|=0x01;    //使能定时器4

} 
void TIM3_INT(void)//1ms定时

{ 
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);    
  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);  //用第2组分法 				
	NVIC_SetVectorTable(NVIC_VectTab_FLASH, 0x0); 	   
  NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3; //抢占级别0-3   （嵌套）
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;        //响应优先级0-3 （排队）
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure); 
	TIM3->ARR=5000; //设定计数器自动重装值  
	TIM3->PSC=31;   //预分频器32，得到1Mhz的计数时钟  31-1000=1ms 	 
	TIM3->DIER|=1<<0;   //允许更新中断     
	TIM3->DIER|=1<<6;   //允许触发中断 
	TIM3->CR1|=0x01;    //使能定时器3
	
} 


void I2C_Configuration(void) //硬件iic初始化
{
  I2C_InitTypeDef  I2C_InitStructure; 
  I2C_InitStructure.I2C_Mode = I2C_Mode_I2C;
  I2C_InitStructure.I2C_DutyCycle = I2C_DutyCycle_2 ;
  I2C_InitStructure.I2C_OwnAddress1 =  0xA0;
  I2C_InitStructure.I2C_Ack = I2C_Ack_Enable;
  I2C_InitStructure.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
  I2C_InitStructure.I2C_ClockSpeed = 100000; 
  I2C_Cmd(I2C1, ENABLE);
  I2C_Init(I2C1, &I2C_InitStructure);
	I2C_AcknowledgeConfig(I2C1, ENABLE);
}

void GPIO_Configuration(void) //各个模块GPIO初始化
{
	GPIO_InitTypeDef GPIO_InitStructure;
	
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;//  PB0-LED
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; 
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 
  GPIO_Init(GPIOB, &GPIO_InitStructure); 
	
	GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_6 | GPIO_Pin_7;//iic管脚：SCL and SDA
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_OD;
  GPIO_Init(GPIOB, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_11 | GPIO_Pin_10;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_Init(GPIOB, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_4;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_Init(GPIOA, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin =GPIO_Pin_0|GPIO_Pin_1|GPIO_Pin_2|GPIO_Pin_3;//配置管脚
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP; 
  GPIO_Init(GPIOA, &GPIO_InitStructure); 
}

