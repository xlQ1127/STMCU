#include "rc.h"
#include "Systick.h"
#include "led.h"
#include "maincom.h"
#include "stmflash.h"

//TIM4 PWM  输入通道：
//channel1  channel2  channel3  channel4
//PB6       PB7       PB8       PB9



//定时器4通道输入捕获配置
u8  TIM4CH1_CAPTURE_STA=0;	//通道1输入捕获状态		  用高两位做捕获标志，低六位做溢出计数  				
u16	TIM4CH1_CAPTURE_UPVAL;	//通道1输入捕获值
u16	TIM4CH1_CAPTURE_DOWNVAL;//通道1输入捕获值

u8  TIM4CH2_CAPTURE_STA=0;	//通道2输入捕获状态		    				
u16	TIM4CH2_CAPTURE_UPVAL;	//通道2输入捕获值
u16	TIM4CH2_CAPTURE_DOWNVAL;//通道2输入捕获值

u8  TIM4CH3_CAPTURE_STA=0;	//通道3输入捕获状态		    				
u16	TIM4CH3_CAPTURE_UPVAL;	//通道3输入捕获值
u16	TIM4CH3_CAPTURE_DOWNVAL;//通道3输入捕获值

u8  TIM4CH4_CAPTURE_STA=0;	//通道4输入捕获状态		    				
u16	TIM4CH4_CAPTURE_UPVAL;	//通道4输入捕获值
u16	TIM4CH4_CAPTURE_DOWNVAL;//通道4输入捕获值

u16 RC_CH[8];

u16 CH1_MxMi[2] = {1500,1500};
u16 CH2_MxMi[2] = {1500,1500};
u16 CH3_MxMi[2] = {1500,1500};
u16 CH4_MxMi[2] = {1500,1500};
	
u16 CH5_Mode[6] = {0,0,0,0,0,0};
u16 CH6_MxMi[2] = {1500,1500};
u16 CH7_MxMi[2] = {1500,1500};
u16 CH8_MxMi[2] = {1500,1500};



void RC_Init(void)
{
	TIM4_Init();
}


void TIM4_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_ICInitTypeDef  TIM4_ICInitStructure;
	
	NVIC_InitTypeDef NVIC_InitStructure;
	
	//设置NVIC中TIM4中断
	NVIC_InitStructure.NVIC_IRQChannel = TIM4_IRQn;//通道设置为TIM4
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;//抢占0
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;//响应3
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;//打开中断通道
	NVIC_Init(&NVIC_InitStructure);//写入配置
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);//使能TIM4时钟
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB,ENABLE);//GPIOB 时钟开启
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6|GPIO_Pin_7|GPIO_Pin_8|GPIO_Pin_9;
	//GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; 
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;//上拉输入
	GPIO_Init(GPIOB, &GPIO_InitStructure);//GPIOB 6 7 8 9 口配置
	GPIO_ResetBits(GPIOB,GPIO_Pin_6|GPIO_Pin_7|GPIO_Pin_8|GPIO_Pin_9);
	
	
	TIM_TimeBaseStructure.TIM_Period = 20000-1;
  TIM_TimeBaseStructure.TIM_Prescaler = (72 - 1);//时钟预分频数 72M/72(1M/20000) = 50HZ
 	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;//设置时钟分频系数：不分频
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;//向上计数模式(0->?)
	TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure);//配置TIM4
	
	//初始化TIM4输入捕获参数
	TIM4_ICInitStructure.TIM_Channel = TIM_Channel_1; //CC1S=01 	选择输入端 IC1映射到TI1上
  TIM4_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;	//上升沿捕获
  TIM4_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI; //映射到TI1上
  TIM4_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;	 //配置输入分频,不分频 
  TIM4_ICInitStructure.TIM_ICFilter = 0x00;//IC1F=0000 配置输入滤波器 不滤波
  TIM_ICInit(TIM4, &TIM4_ICInitStructure);
	
	TIM4_ICInitStructure.TIM_Channel = TIM_Channel_2; //CC1S=01 	选择输入端 IC1映射到TI1上
  TIM4_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;	//上升沿捕获
  TIM4_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI; //映射到TI1上
  TIM4_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;	 //配置输入分频,不分频 
  TIM4_ICInitStructure.TIM_ICFilter = 0x00;//IC1F=0000 配置输入滤波器 不滤波
  TIM_ICInit(TIM4, &TIM4_ICInitStructure);
	
	TIM4_ICInitStructure.TIM_Channel = TIM_Channel_3; //CC1S=01 	选择输入端 IC1映射到TI1上
  TIM4_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;	//上升沿捕获
  TIM4_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI; //映射到TI1上
  TIM4_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;	 //配置输入分频,不分频 
  TIM4_ICInitStructure.TIM_ICFilter = 0x00;//IC1F=0000 配置输入滤波器 不滤波
  TIM_ICInit(TIM4, &TIM4_ICInitStructure);
	
	TIM4_ICInitStructure.TIM_Channel = TIM_Channel_4; //CC1S=01 	选择输入端 IC1映射到TI1上
  TIM4_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;	//上升沿捕获
  TIM4_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI; //映射到TI1上
  TIM4_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;	 //配置输入分频,不分频 
  TIM4_ICInitStructure.TIM_ICFilter = 0x00;//IC1F=0000 配置输入滤波器 不滤波
  TIM_ICInit(TIM4, &TIM4_ICInitStructure);
	
	
	TIM_ITConfig(TIM4,TIM_IT_CC1,ENABLE);//允许CC1IE捕获中断	 
	TIM_ITConfig(TIM4,TIM_IT_CC2,ENABLE);//允许CC2IE捕获中断	
	TIM_ITConfig(TIM4,TIM_IT_CC3,ENABLE);//允许CC3IE捕获中断	
	TIM_ITConfig(TIM4,TIM_IT_CC4,ENABLE);//允许CC4IE捕获中断
	
	TIM_Cmd(TIM4,ENABLE );//使能定时器4
}

//定时器4中断服务程序	 
void TIM4_IRQHandler(void)
{
	
	if(TIM_GetITStatus(TIM4, TIM_IT_CC1) == SET)//捕获1发生捕获事件
	{
		TIM_ClearITPendingBit(TIM4, TIM_IT_CC1);//清除捕获1标志位
		
		if(TIM4CH1_CAPTURE_STA == 0)//捕获到上升沿
		{
			TIM4CH1_CAPTURE_UPVAL = TIM_GetCapture1(TIM4);//获取上升沿的数据
			
			TIM4CH1_CAPTURE_STA = 1;		//标记以捕获到了上升沿
			TIM_OC1PolarityConfig(TIM4,TIM_ICPolarity_Falling);//设置为下降沿捕获
		}
		else                        //捕获到下降沿 (已经捕获到一个完整的高电平脉冲！)
		{
			TIM4CH1_CAPTURE_DOWNVAL = TIM_GetCapture1(TIM4);//获取下降沿的数据
			
			//判读是否超出发生了溢出,计算高电平脉冲时间us
			if(TIM4CH1_CAPTURE_DOWNVAL<TIM4CH1_CAPTURE_UPVAL)
			{
				RC_CH[0] = 20000 - TIM4CH1_CAPTURE_UPVAL + TIM4CH1_CAPTURE_DOWNVAL;
			}
			else
			{
				RC_CH[0] = TIM4CH1_CAPTURE_DOWNVAL- TIM4CH1_CAPTURE_UPVAL;
			}
			
			TIM4CH1_CAPTURE_STA = 0;
			TIM_OC1PolarityConfig(TIM4,TIM_ICPolarity_Rising); //设置为上升沿捕获
		}
	}	


	if(TIM_GetITStatus(TIM4, TIM_IT_CC2) == SET)//捕获2发生捕获事件
	{
		TIM_ClearITPendingBit(TIM4, TIM_IT_CC2);//清除捕获2标志位
		
		if(TIM4CH2_CAPTURE_STA == 0)//捕获到上升沿
		{
			TIM4CH2_CAPTURE_UPVAL = TIM_GetCapture2(TIM4);//获取上升沿的数据
			
			TIM4CH2_CAPTURE_STA = 1;		//标记以捕获到了上升沿
			TIM_OC2PolarityConfig(TIM4,TIM_ICPolarity_Falling);//设置为下降沿捕获
		}
		else                        //捕获到下降沿 (已经捕获到一个完整的高电平脉冲！)
		{
			TIM4CH2_CAPTURE_DOWNVAL = TIM_GetCapture2(TIM4);//获取下降沿的数据
			
			//判读是否超出发生了溢出,计算高电平脉冲时间us
			if(TIM4CH2_CAPTURE_DOWNVAL<TIM4CH2_CAPTURE_UPVAL)
			{
				RC_CH[1] = 20000 - TIM4CH2_CAPTURE_UPVAL + TIM4CH2_CAPTURE_DOWNVAL;
			}
			else
			{
				RC_CH[1] = TIM4CH2_CAPTURE_DOWNVAL- TIM4CH2_CAPTURE_UPVAL;
			}
			TIM4CH2_CAPTURE_STA = 0;
			TIM_OC2PolarityConfig(TIM4,TIM_ICPolarity_Rising); //设置为上升沿捕获
		}
	}
	
	if(TIM_GetITStatus(TIM4, TIM_IT_CC3) == SET)//捕获3发生捕获事件
	{
		TIM_ClearITPendingBit(TIM4, TIM_IT_CC3);//清除捕获3标志位
		
		if(TIM4CH3_CAPTURE_STA == 0)//捕获到上升沿
		{
			TIM4CH3_CAPTURE_UPVAL = TIM_GetCapture3(TIM4);//获取上升沿的数据
			
			TIM4CH3_CAPTURE_STA = 1;		//标记以捕获到了上升沿
			TIM_OC3PolarityConfig(TIM4,TIM_ICPolarity_Falling);//设置为下降沿捕获
		}
		else                        //捕获到下降沿 (已经捕获到一个完整的高电平脉冲！)
		{
			TIM4CH3_CAPTURE_DOWNVAL = TIM_GetCapture3(TIM4);//获取下降沿的数据
			
			//判读是否超出发生了溢出,计算高电平脉冲时间us
			if(TIM4CH3_CAPTURE_DOWNVAL<TIM4CH3_CAPTURE_UPVAL)
			{
				RC_CH[2] = 20000 - TIM4CH3_CAPTURE_UPVAL + TIM4CH3_CAPTURE_DOWNVAL;
			}
			else
			{
				RC_CH[2] = TIM4CH3_CAPTURE_DOWNVAL- TIM4CH3_CAPTURE_UPVAL;
			}
			
			TIM4CH3_CAPTURE_STA = 0;
			TIM_OC3PolarityConfig(TIM4,TIM_ICPolarity_Rising); //设置为上升沿捕获
			
		}
	}
	
	
	if(TIM_GetITStatus(TIM4, TIM_IT_CC4) == SET)//捕获4发生捕获事件
	{
		TIM_ClearITPendingBit(TIM4, TIM_IT_CC4);//清除捕获4标志位
		
		if(TIM4CH4_CAPTURE_STA == 0)//捕获到上升沿
		{
			TIM4CH4_CAPTURE_UPVAL = TIM_GetCapture4(TIM4);//获取上升沿的数据
			
			TIM4CH4_CAPTURE_STA = 1;		//标记以捕获到了上升沿
			TIM_OC4PolarityConfig(TIM4,TIM_ICPolarity_Falling);//设置为下降沿捕获
		}
		else                        //捕获到下降沿 (已经捕获到一个完整的高电平脉冲！)
		{
			TIM4CH4_CAPTURE_DOWNVAL = TIM_GetCapture4(TIM4);//获取下降沿的数据
			
			//判读是否超出发生了溢出,计算高电平脉冲时间us
			if(TIM4CH4_CAPTURE_DOWNVAL<TIM4CH4_CAPTURE_UPVAL)
			{
				RC_CH[3] = 20000 - TIM4CH4_CAPTURE_UPVAL + TIM4CH4_CAPTURE_DOWNVAL;
			}
			else
			{
				RC_CH[3] = TIM4CH4_CAPTURE_DOWNVAL- TIM4CH4_CAPTURE_UPVAL;
			}
			
			TIM4CH4_CAPTURE_STA = 0;
			TIM_OC4PolarityConfig(TIM4,TIM_ICPolarity_Rising); //设置为上升沿捕获
			
		}
	}
	
}
//////////////////////////////////////////////////////////RC具体功能实现///////////////////////////////////////////////////////////


u16 Get_RightCH_Value(u16 num,u16 max,u16 min)
{
	if(num>max)
		return max;
	else if(num<min)
		return min;
	else
		return num;
}



//把CH3通道捕获的油门信号转换成校准后的油门值
u16 Value_2_Thr(void)
{
	return (1000+RC_CH[3-1]-CH3_MxMi[1]);
}

//把CH1通道的PWM值转换成期望的Roll角度
float Value_2_Roll(void)
{
	s16 value_half;
	
	value_half = (CH3_MxMi[0]+CH3_MxMi[1])/2;
	
	return -(float)MAX_ANGLE*(float)((s16)RC_CH[1-1]-value_half)/(float)(value_half-CH3_MxMi[0]);
	
}


//把CH2通道的PWM值转换成期望的Pitch角度
float Value_2_Pitch(void)
{
	s16 value_half;
	
	value_half = (CH2_MxMi[0]+CH2_MxMi[1])/2;//计算中点值
	
	return (float)MAX_ANGLE*(float)((s16)RC_CH[2-1]-value_half)/(float)(value_half-CH2_MxMi[0]);
	
}

//把CH4通道的PWM值转换成期望的Yaw转动角速度
s16 Vaule_2_Gyro(void)
{
	s16 value_half;
	
	value_half = (CH4_MxMi[0]+CH4_MxMi[1])/2;//计算中点值
	
	return MAX_GYRO*((s16)RC_CH[4-1]-value_half)/(value_half-CH4_MxMi[0]);
	
}


//遥控器RC各通道校准(进入遥控器校准程序，校准完毕后必须复位才能退出！)
void Channel_Adjust(void)
{
	u16 flash_data[39];
	u16 i = 0;
	u8 point;
	
	for(i=0;i<1000;i++)//20s内完成遥控器校准
	{
		if(RC_CH[1-1]>CH1_MxMi[0] && RC_CH[1-1]<=2500) CH1_MxMi[0] = RC_CH[1-1];//得到最大值
		if(RC_CH[1-1]<CH1_MxMi[1] && RC_CH[1-1]>=500) CH1_MxMi[1] = RC_CH[1-1];//得到最小值
	
		if(RC_CH[2-1]>CH2_MxMi[0] && RC_CH[2-1]<=2500) CH2_MxMi[0] = RC_CH[2-1];//得到最大值
		if(RC_CH[2-1]<CH2_MxMi[1] && RC_CH[2-1]>=500) CH2_MxMi[1] = RC_CH[2-1];//得到最小值
	
		if(RC_CH[3-1]>CH3_MxMi[0] && RC_CH[3-1]<=2500) CH3_MxMi[0] = RC_CH[3-1];//得到最大值
		if(RC_CH[3-1]<CH3_MxMi[1] && RC_CH[3-1]>=500) CH3_MxMi[1] = RC_CH[3-1];//得到最小值
	
		if(RC_CH[4-1]>CH4_MxMi[0] && RC_CH[4-1]<=2500) CH4_MxMi[0] = RC_CH[4-1];//得到最大值
		if(RC_CH[4-1]<CH4_MxMi[1] && RC_CH[4-1]>=500) CH4_MxMi[1] = RC_CH[4-1];//得到最小值
		
		LED0(ON);
		delay_ms(20);
		LED0(OFF);
	}
	
	for(i=0;i<3;i++)//LED快闪烁3下表示校准完毕
	{
		LED0(ON);
		delay_ms(100);
		LED0(OFF);
		delay_ms(900);
	}
	
	STMFLASH_Read(PAGE1,flash_data,39);//把flash里的数据先读出来
	
	//再赋值
	point = 0;
	
	flash_data[point++] = ADJUST_FLAG;//写入校准标记位
	//1-4通道赋值
	flash_data[point++] = CH1_MxMi[0];  flash_data[point++] = CH1_MxMi[1];
	flash_data[point++] = CH2_MxMi[0];  flash_data[point++] = CH2_MxMi[1];
	flash_data[point++] = CH3_MxMi[0];  flash_data[point++] = CH3_MxMi[1];
	flash_data[point++] = CH4_MxMi[0];  flash_data[point++] = CH4_MxMi[1];
	
	//5-8通道赋值
	//???????????????????????????
	
	STMFLASH_Write(PAGE1,flash_data,39);//新数据写入flash
	
	//读出校准数据 并 发送到上位机
	STMFLASH_Read(PAGE1,flash_data,39);
	
	printf("CH1 = %d,%d\n",flash_data[1],flash_data[2]);
	printf("CH2 = %d,%d\n",flash_data[3],flash_data[4]);
	printf("CH3 = %d,%d\n",flash_data[5],flash_data[6]);
	printf("CH4 = %d,%d\n",flash_data[7],flash_data[8]);
	
	LED0(ON);
	while(1);
}


//读取Flash里的的RC通道校准值
u8 Channel_Config(void)
{
	u16 flash_data[39];
	u8 point=0;
	
	//读出Flash数据
	STMFLASH_Read(PAGE1,flash_data,29);
	
	if(flash_data[point++] == ADJUST_FLAG)//判断是否已经进行遥控器校准
	{
		CH1_MxMi[0] = flash_data[point++];CH1_MxMi[1] = flash_data[point++];
		CH2_MxMi[0] = flash_data[point++];CH2_MxMi[1] = flash_data[point++];
		CH3_MxMi[0] = flash_data[point++];CH3_MxMi[1] = flash_data[point++];
		CH4_MxMi[0] = flash_data[point++];CH4_MxMi[1] = flash_data[point++];
		
		return 1;
	}
	else
	{
		CH1_MxMi[0] = 2000;CH1_MxMi[1] = 1000;
		CH2_MxMi[0] = 2000;CH2_MxMi[1] = 1000;
		CH3_MxMi[0] = 2000;CH3_MxMi[1] = 1000;
		CH4_MxMi[0] = 2000;CH4_MxMi[1] = 1000;
		
		return 0;
	}
}

