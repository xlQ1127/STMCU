/******************** (C) COPYRIGHT 2014 Air Nano Team ***************************
 * 文件名  ：PWM_IN.c
 * 描述    ：PWM输入捕获      
 * 实验平台：Air Nano四轴飞行器
 * 库版本  ：ST3.5.0
 * 作者    ：Air Nano Team 
 * 淘宝    ：http://byd2.taobao.com
**********************************************************************************/
#include "pwm_in.h"


u16  Rise[7],Drop[7];   //上升下降沿捕获值存于此
u16  RC_Pwm_In[8];      //捕获的PWM值存于此处
u16  RC_Pwm_In_his[8];





/*====================================================================================================*/
/*====================================================================================================*
**函数 : PWM_IN_Config
**功能 : 配置PWM输入捕获
**输入 : None
**輸出 : None
**备注 : None
**====================================================================================================*/
/*====================================================================================================*/
//TIM2的四个通道PA-0123.TIM3的两个通道PA-67.

void PWM_IN_Config(void)
{   //【时钟及引脚配置】
	  GPIO_InitTypeDef         GPIO_InitStructure;
	  TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	  TIM_ICInitTypeDef  TIM2_ICInitStructure;            //通道模式的结构体不同的结构体可以共用

	  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2 | RCC_APB1Periph_TIM3, ENABLE);	 //使能TIM2时钟
 	  RCC_APB2PeriphClockCmd(RCC_GPIO_TIM2 | RCC_GPIO_TIM3 |RCC_APB2Periph_GPIOB, ENABLE);  

	  GPIO_InitStructure.GPIO_Pin  = TIM2_CH1 | TIM2_CH2 | TIM2_CH3 | TIM2_CH4;      //  【TIM2】****【GPIOA-0-1-2-3】        
	  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD;            
	  GPIO_Init(GPIO_TIM2, &GPIO_InitStructure);	

      GPIO_InitStructure.GPIO_Pin  = TIM3_CH1 | TIM3_CH2;	
	  GPIO_Init(GPIO_TIM3, &GPIO_InitStructure);	                                 //  【TIM3】****【GPIOA-6-7】
	
	  GPIO_InitStructure.GPIO_Pin  = GPIO_Pin_0;	
	  GPIO_Init(GPIOB, &GPIO_InitStructure);	

	  //【时基的初始化】
	  TIM_TimeBaseStructure.TIM_Period = 0XFFFF;      //【0XFFFF】     //设定计数器自动重装值 
	  TIM_TimeBaseStructure.TIM_Prescaler =71; 	                       //预分频器   计数频率 【 1HZ 】
	  TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;          //设置时钟分割:TDTS = Tck_tim
	  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;      //TIM向上计数模式
      
	  TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure); //TIM2           //根据TIM_TimeBaseInitStruct中指定的参数初始化TIMx的时间基数单位
      TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure); //TIM3
      
	
	  //【通道模式的初始化】
	  TIM2_ICInitStructure.TIM_Channel = TIM_Channel_1;                //CC1S=01 	选择输入端 IC1映射到TI1上
  	  TIM2_ICInitStructure.TIM_ICPolarity =TIM_ICPolarity_Rising;	   //上升沿捕获
  	  TIM2_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI; //映射到TI1上
  	  TIM2_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;	       //配置输入分频,不分频 
  	  TIM2_ICInitStructure.TIM_ICFilter = 0x00;                        //IC1F=0000 配置输入滤波器 不滤波
  	  TIM_ICInit(TIM2, &TIM2_ICInitStructure);
	  TIM_ICInit(TIM3, &TIM2_ICInitStructure);
	
	  TIM2_ICInitStructure.TIM_Channel = TIM_Channel_2;                //CC1S=01 	选择输入端 IC1映射到TI1上
  	  TIM2_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;	   //上升沿捕获
  	  TIM2_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI; //映射到TI1上
  	  TIM2_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;	       //配置输入分频,不分频 
  	  TIM2_ICInitStructure.TIM_ICFilter = 0x00;                        //IC1F=0000 配置输入滤波器 不滤波
  	  TIM_ICInit(TIM2, &TIM2_ICInitStructure);
	  TIM_ICInit(TIM3, &TIM2_ICInitStructure);
		
	  TIM2_ICInitStructure.TIM_Channel = TIM_Channel_3;                //CC1S=01 	选择输入端 IC1映射到TI1上
  	  TIM2_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;	   //上升沿捕获
  	  TIM2_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI; //映射到TI1上
  	  TIM2_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;	       //配置输入分频,不分频 
  	  TIM2_ICInitStructure.TIM_ICFilter = 0x00;                        //IC1F=0000 配置输入滤波器 不滤波
  	  TIM_ICInit(TIM2, &TIM2_ICInitStructure);
	   
		
	  TIM2_ICInitStructure.TIM_Channel = TIM_Channel_4;                //CC1S=01 	选择输入端 IC1映射到TI1上
  	  TIM2_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;	   //上升沿捕获
  	  TIM2_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI; //映射到TI1上
  	  TIM2_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;	       //配置输入分频,不分频 
  	  TIM2_ICInitStructure.TIM_ICFilter = 0x00;                        //IC1F=0000 配置输入滤波器 不滤波
  	   
	
	  TIM_Cmd(TIM2,ENABLE ); //使能TIM2 TIM3
	  TIM_Cmd(TIM3,ENABLE );
		
	  TIM_ITConfig(TIM2, TIM_IT_CC1, ENABLE);        //允许更新中断 ,允许CC1IE捕获中断	
	  TIM_ITConfig(TIM2, TIM_IT_CC2, ENABLE);
	  TIM_ITConfig(TIM2, TIM_IT_CC3, ENABLE);
	  TIM_ITConfig(TIM2, TIM_IT_CC4, ENABLE);	
		
	  TIM_ITConfig(TIM3, TIM_IT_CC1, ENABLE);        //允许更新中断 ,允许CC1IE捕获中断	
	  TIM_ITConfig(TIM3, TIM_IT_CC2, ENABLE);
	  TIM_ITConfig(TIM3, TIM_IT_CC3, ENABLE);
}
/*====================================================================================================*/
/*====================================================================================================*
**函数 : TIM2_IRQHandler
**功能 : TIM2中断服务
**输入 : None
**輸出 : None
**备注 : None
//RC_Pwm_In[8] 各通道捕获值
**====================================================================================================*/
/*====================================================================================================*/
//上升沿捕获 下降沿捕获加计算
void TIM2_IRQHandler(void)
{ 
    if (TIM_GetITStatus(TIM2, TIM_IT_CC1) != RESET)                             //捕获1发生捕获事件
		{	
			TIM_ClearITPendingBit(TIM2, TIM_IT_CC1);                            //清除中断标志位
			if(GPIO_ReadInputDataBit(GPIO_TIM2,TIM2_CH1) == 1)                  //读取IO状态   若TIM2 通道1为高
			{
				  TIM_OC1PolarityConfig(TIM2,TIM_ICPolarity_Falling);	        //CC1P=1 设置为下降沿捕获
                  Rise[0]=TIM_GetCapture1(TIM2);
             }     //【Rise[0]保存信号高电平起时间】
			else 
			{     //【Drop[0]保存低电平起时间】
				  TIM_OC1PolarityConfig(TIM2,TIM_ICPolarity_Rising);            //CC1P=0 设置为上升沿捕获
                  Drop[0]=TIM_GetCapture1(TIM2);
				  if(Rise[0]>Drop[0])  RC_Pwm_In[0] = 65535-Rise[0] + Drop[0];  //计算两次差值考虑是否溢出
					else 	               RC_Pwm_In[0] = Drop[0] - Rise[0];
             }			                                                        //【最终遥控器原始数据被保存到 RC_Pwm_In[0]】 
		}	
	  
	if (TIM_GetITStatus(TIM2, TIM_IT_CC2) != RESET)   //捕获2发生捕获事件
		{	
			TIM_ClearITPendingBit(TIM2, TIM_IT_CC2); //清除中断标志位
			if(GPIO_ReadInputDataBit(GPIO_TIM2,TIM2_CH2) == 1) 
			{
				  TIM_OC2PolarityConfig(TIM2,TIM_ICPolarity_Falling);		//CC1P=1 设置为下降沿捕获
          Rise[1]=TIM_GetCapture2(TIM2);
      }
			else 
			{
				  TIM_OC2PolarityConfig(TIM2,TIM_ICPolarity_Rising); //CC1P=0 设置为上升沿捕获
          Drop[1]=TIM_GetCapture2(TIM2);
				  if(Rise[1]>Drop[1])  RC_Pwm_In[1] = 65535-Rise[1] + Drop[1];
					else 	               RC_Pwm_In[1] = Drop[1] - Rise[1];
      }			
		}	
		
    if (TIM_GetITStatus(TIM2, TIM_IT_CC3) != RESET)            //捕获3发生捕获事件
		{	
			TIM_ClearITPendingBit(TIM2, TIM_IT_CC3); //清除中断标志位
			if(GPIO_ReadInputDataBit(GPIO_TIM2,TIM2_CH3) == 1) 
			{
				  TIM_OC3PolarityConfig(TIM2,TIM_ICPolarity_Falling);		//CC1P=1 设置为下降沿捕获
          Rise[2]=TIM_GetCapture3(TIM2);
      }
			else 
			{
				  TIM_OC3PolarityConfig(TIM2,TIM_ICPolarity_Rising); //CC1P=0 设置为上升沿捕获
          Drop[2]=TIM_GetCapture3(TIM2);
				  if(Rise[2]>Drop[2]) RC_Pwm_In[2] = 65535-Rise[2] + Drop[2];
					else 	              RC_Pwm_In[2] = Drop[2] - Rise[2];
      }	 
		}	

    if (TIM_GetITStatus(TIM2, TIM_IT_CC4) != RESET)            //捕获4发生捕获事件
		{	
			TIM_ClearITPendingBit(TIM2, TIM_IT_CC4); //清除中断标志位
		  if(GPIO_ReadInputDataBit(GPIO_TIM2,TIM2_CH4) == 1) 
			{
				  TIM_OC4PolarityConfig(TIM2,TIM_ICPolarity_Falling);		//CC1P=1 设置为下降沿捕获
          Rise[3]=TIM_GetCapture4(TIM2);
      }
			else 
			{
				  TIM_OC4PolarityConfig(TIM2,TIM_ICPolarity_Rising); //CC1P=0 设置为上升沿捕获
          Drop[3]=TIM_GetCapture4(TIM2);
				  if(Rise[3]>Drop[3])  RC_Pwm_In[3] = 65535-Rise[3] + Drop[3];
					else 	               RC_Pwm_In[3] = Drop[3] - Rise[3];
      }	  
		}		
}


vu16 US100_Alt_Temp=0,Alt_Last=0; 
float Alt_CuntTmep1=0,Alt_CuntTmep2=0;
float Alt_V_CuntTmep1=0,Alt_V_CuntTmep2=0;
float US100_Alt_Last=0;
/*====================================================================================================*/
/*====================================================================================================*
**函数 : TIM3_IRQHandler
**功能 : TIM3中断服务
**输入 : None
**輸出 : None
**备注 : None
**====================================================================================================*/
/*====================================================================================================*/
void TIM3_IRQHandler(void)
{ 
    if (TIM_GetITStatus(TIM3, TIM_IT_CC1) != RESET)          //捕获1发生捕获事件
		{	
			TIM_ClearITPendingBit(TIM3, TIM_IT_CC1);         //清除中断标志位
			if(GPIO_ReadInputDataBit(GPIO_TIM3,TIM3_CH1) == 1) 
			{
				  TIM_OC1PolarityConfig(TIM3,TIM_ICPolarity_Falling);		//CC1P=1 设置为下降沿捕获
          Rise[4]=TIM_GetCapture1(TIM3);
      }
			else 
			{
				  TIM_OC1PolarityConfig(TIM3,TIM_ICPolarity_Rising);        //CC1P=0 设置为上升沿捕获
          Drop[4]=TIM_GetCapture1(TIM3);
				  if(Rise[4]>Drop[4])  RC_Pwm_In[4] = 65535-Rise[4] + Drop[4];
					else 	               RC_Pwm_In[4] = Drop[4] - Rise[4];
      }			
		}	
	  
	if (TIM_GetITStatus(TIM3, TIM_IT_CC2) != RESET)    //捕获2发生捕获事件
		{	
			TIM_ClearITPendingBit(TIM3, TIM_IT_CC2);   //清除中断标志位
			if(GPIO_ReadInputDataBit(GPIO_TIM3,TIM3_CH2) == 1) 
			{
				  TIM_OC2PolarityConfig(TIM3,TIM_ICPolarity_Falling);		//CC1P=1 设置为下降沿捕获
          Rise[5]=TIM_GetCapture2(TIM3);
      }
			else 
			{
				  TIM_OC2PolarityConfig(TIM3,TIM_ICPolarity_Rising);        //CC1P=0 设置为上升沿捕获
          Drop[5]=TIM_GetCapture2(TIM3);
				  if(Rise[5]>Drop[5])  RC_Pwm_In[5] = 65535-Rise[5] + Drop[5];
					else 	               RC_Pwm_In[5] = Drop[5] - Rise[5];
      }			
		}	
		
	 if (TIM_GetITStatus(TIM3, TIM_IT_CC3) != RESET)   //捕获3发生捕获事件
		{	
			TIM_ClearITPendingBit(TIM3, TIM_IT_CC3);   //清除中断标志位
			if(GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_0) == 1) 
			{
				TIM_OC3PolarityConfig(TIM3,TIM_ICPolarity_Falling);		//CC1P=1 设置为下降沿捕获
				Rise[6]=TIM_GetCapture3(TIM3);
      }
			else 
			{
				TIM_OC3PolarityConfig(TIM3,TIM_ICPolarity_Rising);      //CC1P=0 设置为上升沿捕获
				Drop[6]=TIM_GetCapture3(TIM3);
				if(Rise[6]>Drop[6])  US100_Alt_Temp = 65535-Rise[6] + Drop[6];
				else 	               US100_Alt_Temp = Drop[6] - Rise[6];

				if(US100_Alt_Temp>10000)   US100_Alt_Temp=Alt_Last; 
				else    					       Alt_Last=US100_Alt_Temp; 

				Alt_CuntTmep2=Alt_CuntTmep1;        //滑动平均滤波
				Alt_CuntTmep1=US100_Alt_Temp;//*((float)cos(angle.pitch/57.295779f))*((float)cos(angle.roll/57.295779f));        //姿态补偿

				//US100_Alt=((Alt_CuntTmep1+Alt_CuntTmep2)/2)*34/2000;  //转化为CM

// 				Alt_V_CuntTmep2=Alt_V_CuntTmep1;//滑动平均滤波
// 				Alt_V_CuntTmep1=(US100_Alt-US100_Alt_Last)/ 2;   //除以0.05s获得速度单位：m/S 

// 				US100_Alt_V= (Alt_V_CuntTmep1+Alt_V_CuntTmep2)/2;   
// 				US100_Alt_Last=US100_Alt; 
// 				fff=1;
      }			
		}	
}




