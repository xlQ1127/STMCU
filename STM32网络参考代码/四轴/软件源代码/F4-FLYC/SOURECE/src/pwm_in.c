/**********************************************************************************
 * 文件名  ：incap.c
 * 描述    ：四通道输入捕获	PC6-PC9	TIM3_CH1-CH4

 * 实验平台：四轴飞行器STM33F4主控板
 * 库版本  ：
 * 作者    ：
 * 时间    ：
**********************************************************************************/	
#include "pwm_in.h"


u8  TIM3CH1_CAPTURE_STA=0;	//通道1输入捕获状态		  用高两位做捕获标志，低六位做溢出计数  				
u16	TIM3CH1_CAPTURE_UPVAL;	//通道1输入捕获值
u16	TIM3CH1_CAPTURE_DOWNVAL;	//通道1输入捕获值

u8  TIM3CH2_CAPTURE_STA=0;	//通道2输入捕获状态		    				
u16	TIM3CH2_CAPTURE_UPVAL;	//通道2输入捕获值
u16	TIM3CH2_CAPTURE_DOWNVAL;	//通道2输入捕获值

u8  TIM3CH3_CAPTURE_STA=0;	//通道3输入捕获状态		    				
u16	TIM3CH3_CAPTURE_UPVAL;	//通道3输入捕获值
u16	TIM3CH3_CAPTURE_DOWNVAL;	//通道3输入捕获值

u8  TIM3CH4_CAPTURE_STA=0;	//通道4输入捕获状态		    				
u16	TIM3CH4_CAPTURE_UPVAL;	//通道4输入捕获值
u16	TIM3CH4_CAPTURE_DOWNVAL;	//通道4输入捕获值


u32 tempup1=0;//捕获总高电平的时间us单位
u32 tempup2=0;//捕获总高电平的时间us单位
u32 tempup3=0;//捕获总高电平的时间us单位
u32 tempup4=0;//捕获总高电平的时间us单位
u32 tim3_T;

int pwmout1,pwmout2,pwmout3,pwmout4; 				//输出占空比


void pwm_in_init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_ICInitTypeDef  TIM3_ICInitStructure;
//	EXTI_InitTypeDef EXTI_InitStructure;

   	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);	//使能TIM3时钟
 	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);  //使能GPIOC时钟
	
	GPIO_InitStructure.GPIO_Pin  = GPIO_Pin_6|GPIO_Pin_7|GPIO_Pin_8|GPIO_Pin_9;  //PA0 清除之前设置  
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(GPIOC, &GPIO_InitStructure);
	

	GPIO_PinAFConfig(GPIOC, GPIO_PinSource6, GPIO_AF_TIM3);  
  	GPIO_PinAFConfig(GPIOC, GPIO_PinSource7, GPIO_AF_TIM3);
	GPIO_PinAFConfig(GPIOC, GPIO_PinSource8, GPIO_AF_TIM3);  
  	GPIO_PinAFConfig(GPIOC, GPIO_PinSource9, GPIO_AF_TIM3);
		
/*--------------------------------------------------配置定时器5输入捕获--------------------------------------------*/	
	//初始化定时器3 TIM3	 
	TIM_TimeBaseStructure.TIM_Period = 0XFFFF; //设定计数器自动重装值 
	TIM_TimeBaseStructure.TIM_Prescaler =84-1; 	//预分频器   
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1; //设置时钟分割:TDTS = Tck_tim
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  //TIM向上计数模式
	TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure); //根据TIM_TimeBaseInitStruct中指定的参数初始化TIMx的时间基数单位
 
	//初始化TIM3输入捕获参数
	TIM3_ICInitStructure.TIM_Channel = TIM_Channel_1; //CC1S=01 	选择输入端 IC1映射到TI1上
  	TIM3_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;	//上升沿捕获
  	TIM3_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI; //映射到TI1上
  	TIM3_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;	 //配置输入分频,不分频 
  	TIM3_ICInitStructure.TIM_ICFilter = 0x01;//IC1F=0000 配置输入滤波器 不滤波
  	TIM_ICInit(TIM3, &TIM3_ICInitStructure);

	
	TIM3_ICInitStructure.TIM_Channel = TIM_Channel_2; //CC1S=01 	选择输入端 IC1映射到TI1上
  	TIM3_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;	//上升沿捕获
  	TIM3_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI; //映射到TI1上
  	TIM3_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;	 //配置输入分频,不分频 
  	TIM3_ICInitStructure.TIM_ICFilter = 0x01;//IC1F=0000 配置输入滤波器 不滤波
  	TIM_ICInit(TIM3, &TIM3_ICInitStructure);
			
	TIM3_ICInitStructure.TIM_Channel = TIM_Channel_3; //CC1S=01 	选择输入端 IC1映射到TI1上
  	TIM3_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;	//上升沿捕获
  	TIM3_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI; //映射到TI1上
  	TIM3_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;	 //配置输入分频,不分频 
  	TIM3_ICInitStructure.TIM_ICFilter = 0x01;//IC1F=0000 配置输入滤波器 不滤波
  	TIM_ICInit(TIM3, &TIM3_ICInitStructure);

	TIM3_ICInitStructure.TIM_Channel = TIM_Channel_4; //CC1S=01 	选择输入端 IC1映射到TI1上
  	TIM3_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;	//上升沿捕获
  	TIM3_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI; //映射到TI1上
  	TIM3_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;	 //配置输入分频,不分频 
  	TIM3_ICInitStructure.TIM_ICFilter = 0x01;//IC1F=0000 配置输入滤波器 不滤波
  	TIM_ICInit(TIM3, &TIM3_ICInitStructure);
									  
	TIM_ITConfig(TIM3,TIM_IT_CC1,ENABLE);//允许更新中断 ,允许CC1IE捕获中断	 
	TIM_ITConfig(TIM3,TIM_IT_CC2,ENABLE);//允许更新中断 ,允许CC1IE捕获中断	
	TIM_ITConfig(TIM3,TIM_IT_CC3,ENABLE);//允许更新中断 ,允许CC1IE捕获中断	
	TIM_ITConfig(TIM3,TIM_IT_CC4,ENABLE);//允许更新中断 ,允许CC1IE捕获中断	
					 
   	TIM_Cmd(TIM3,ENABLE ); 	//使能定时器5
}

//定时器3中断服务程序	 
void TIM3_IRQHandler(void)
{   
 	if((TIM3CH1_CAPTURE_STA&0X80)==0)//CH1 还未成功捕获	
	{  		
		if (TIM_GetITStatus(TIM3, TIM_IT_CC1) != RESET)//捕获1发生捕获事件
		{	
			 TIM_ClearITPendingBit(TIM3, TIM_IT_CC1);

			if(TIM3CH1_CAPTURE_STA&0X40)		//捕获到一个下降沿为真 		   发生上升沿中断后再次发生中断为下降沿
			{	  			
				TIM3CH1_CAPTURE_DOWNVAL=TIM_GetCapture1(TIM3);

				if(TIM3CH1_CAPTURE_DOWNVAL<TIM3CH1_CAPTURE_UPVAL)
				{
					 tim3_T=65535;
			    }
			    else tim3_T=0;	

			   tempup1=TIM3CH1_CAPTURE_DOWNVAL-TIM3CH1_CAPTURE_UPVAL+tim3_T;//得到总的高电平时间

			   pwmout1=tempup1;

			   TIM3CH1_CAPTURE_STA=0;
					
			   TIM_OC1PolarityConfig(TIM3,TIM_ICPolarity_Rising); //CC1P=0 设置为上升沿捕获			  
			}
			else  								//发生捕获事件但不是下降沿，第一次捕获上升沿，清零，定时器开始计数
			{
				TIM3CH1_CAPTURE_UPVAL=TIM_GetCapture1(TIM3);			//获取上升沿的数据
				TIM3CH1_CAPTURE_STA|=0X40;		//标记以捕获到了上升沿
			   	TIM_OC1PolarityConfig(TIM3,TIM_ICPolarity_Falling);		//CC1P=1 设置为下降沿捕获
			
			}		    
		}			     	    					   
 	}
	
  
	if((TIM3CH2_CAPTURE_STA&0X80)==0)//CH1 还未成功捕获	
	{  
		if (TIM_GetITStatus(TIM3, TIM_IT_CC2) != RESET)//捕获1发生捕获事件
		{	
			TIM_ClearITPendingBit(TIM3,TIM_IT_CC2);
			if(TIM3CH2_CAPTURE_STA&0X40)		//捕获到一个下降沿为真 		   发生上升沿中断后再次发生中断为下降沿
			{	  			

			   TIM3CH2_CAPTURE_DOWNVAL=TIM_GetCapture2(TIM3);

				if(TIM3CH2_CAPTURE_DOWNVAL<TIM3CH2_CAPTURE_UPVAL)
				{
					tim3_T=65535;
				}
				else tim3_T=0;	

			   tempup2=TIM3CH2_CAPTURE_DOWNVAL-TIM3CH2_CAPTURE_UPVAL+tim3_T;//得到总的高电平时间			

			   pwmout2=tempup2;
	  
			   TIM3CH2_CAPTURE_STA=0;	

			   TIM_OC2PolarityConfig(TIM3,TIM_ICPolarity_Rising); //CC1P=0 设置为上升沿捕获
			   
			}
			else  								//发生捕获事件但不是下降沿，第一次捕获上升沿，清零，定时器开始计数
			{
				TIM3CH2_CAPTURE_UPVAL=TIM_GetCapture2(TIM3);			//获取上升沿的数据
				TIM3CH2_CAPTURE_STA|=0X40;		//标记以捕获到了上升沿
			   	TIM_OC2PolarityConfig(TIM3,TIM_ICPolarity_Falling);		//CC1P=1 设置为下降沿捕获	
			}		    
		}			     	    					   
 	}		 
	

	if((TIM3CH3_CAPTURE_STA&0X80)==0)//CH3 还未成功捕获	
	{	
	  
		if (TIM_GetITStatus(TIM3, TIM_IT_CC3) != RESET)//捕获3发生捕获事件
		{	
			TIM_ClearITPendingBit(TIM3, TIM_IT_CC3);

			if(TIM3CH3_CAPTURE_STA&0X40)		//捕获到一个下降沿为真 		   发生上升沿中断后再次发生中断为下降沿
			{	  			
				TIM3CH3_CAPTURE_DOWNVAL=TIM_GetCapture3(TIM3);

				if(TIM3CH3_CAPTURE_DOWNVAL<TIM3CH3_CAPTURE_UPVAL)
				{
					tim3_T=65535;
				}
				else tim3_T=0;	

				tempup3=TIM3CH3_CAPTURE_DOWNVAL-TIM3CH3_CAPTURE_UPVAL+tim3_T;//得到总的高电平时间

			   pwmout3=tempup3;

			   TIM3CH3_CAPTURE_STA=0;
				
			   TIM_OC3PolarityConfig(TIM3,TIM_ICPolarity_Rising); //CC3P=0 设置为上升沿捕获
			
			}
			else  								//发生捕获事件但不是下降沿，第一次捕获上升沿，清零，定时器开始计数
			{
				TIM3CH3_CAPTURE_UPVAL=TIM_GetCapture3(TIM3);			//获取上升沿的数据
				TIM3CH3_CAPTURE_STA|=0X40;		//标记以捕获到了上升沿
			   	TIM_OC3PolarityConfig(TIM3,TIM_ICPolarity_Falling);		//CC1P=1 设置为下降沿捕获
			}		    
		}			     	    					   
 	}

	if((TIM3CH4_CAPTURE_STA&0X80)==0)//CH4 还未成功捕获	
	{	  
		if (TIM_GetITStatus(TIM3, TIM_IT_CC4) != RESET)//捕获1发生捕获事件
		{	
			TIM_ClearITPendingBit(TIM3,TIM_IT_CC4);

			if(TIM3CH4_CAPTURE_STA&0X40)		//捕获到一个下降沿为真 		   发生上升沿中断后再次发生中断为下降沿
			{	  			

				TIM3CH4_CAPTURE_DOWNVAL=TIM_GetCapture4(TIM3);
				if(TIM3CH4_CAPTURE_DOWNVAL<TIM3CH4_CAPTURE_UPVAL)
				{
					tim3_T=65535;
				}
				else tim3_T=0;	

				tempup4=TIM3CH4_CAPTURE_DOWNVAL-TIM3CH4_CAPTURE_UPVAL+tim3_T;//得到总的高电平时间

			   	pwmout4=tempup4;

			    TIM3CH4_CAPTURE_STA=0;	
   				
			   	TIM_OC4PolarityConfig(TIM3,TIM_ICPolarity_Rising); //CC1P=0 设置为上升沿捕获				
			}
			else  								//发生捕获事件但不是下降沿，第一次捕获上升沿，清零，定时器开始计数
			{
				TIM3CH4_CAPTURE_UPVAL=TIM_GetCapture4(TIM3);			//获取上升沿的数据
				TIM3CH4_CAPTURE_STA|=0X40;		//标记以捕获到了上升沿
			   	TIM_OC4PolarityConfig(TIM3,TIM_ICPolarity_Falling);		//CC4P=1 设置为下降沿捕获
			
			}		    
		}			     	    					   
 	}							  			 		 
}


