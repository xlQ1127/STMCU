#include  "TIM3_IT.h"
#include  "NRF24L01.h"
#include  "usart.h"
#include  "adc.h"
#include "SysTick.h"

/**************************实现函数********************************************
*函数原型:		
*功　　能:1ms中断一次,计数器为1000		
*******************************************************************************/
void Tim3_Init(u16 period_num)
{
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3,ENABLE);
	
	TIM_DeInit(TIM3);

	TIM_TimeBaseStructure.TIM_Period        = period_num;   //装载值	//prescaler is 1200,that is 72000000/72/500=2000Hz;
	TIM_TimeBaseStructure.TIM_Prescaler     = 72-1;         //分频系数
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1; //or TIM_CKD_DIV2 or TIM_CKD_DIV4
	TIM_TimeBaseStructure.TIM_CounterMode   = TIM_CounterMode_Up;
	
	TIM_TimeBaseInit(TIM3,&TIM_TimeBaseStructure);	
	
	TIM_ITConfig(TIM3,TIM_IT_Update,ENABLE );   //使能指定的TIM3中断,允许更新中断
	
	//中断优先级NVIC设置	
	NVIC_InitStructure.NVIC_IRQChannel                   = TIM3_IRQn;  //TIM3中断
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;          //先占优先级0级
	NVIC_InitStructure.NVIC_IRQChannelSubPriority        = 3;          //从优先级3级
	NVIC_InitStructure.NVIC_IRQChannelCmd                = ENABLE;     //IRQ通道被使能
	NVIC_Init(&NVIC_InitStructure);  //初始化NVIC寄存器
	
	TIM_Cmd(TIM3, ENABLE);  //使能TIMx		
}
unsigned int  VCC;
unsigned int  ms;
unsigned int  RX_speed;
unsigned int  TX_speed;

int roll;
int pitch;
int yaw;

int X_g;
int Y_g;
int Z_g;

int X_w;
int Y_w;
int Z_w;

void RX_TX_Times(void)
{
	ms++;
	if(ms>=100)
	{
	RX_speed=RX_times;
	RX_times=0;
	TX_speed=TX_times;
	TX_times=0;
	ms=0;
	}
}
void TME_5ms(void)
{
	
}
void TME_10ms(void)
{
	ADC1_Value();
  RX_TX_Times();
	NRF_TxPacket_AP(TxBuf,10);
	Nrf_Check_Event();

	roll  = NRFRX[0]-30000;
	pitch = NRFRX[1]-30000;
	yaw   = NRFRX[2]-30000;	
	X_g   = NRFRX[3]-30000;
	Y_g   = NRFRX[4]-30000;
	Z_g   = NRFRX[5]-30000;	
  X_w   = NRFRX[6]-30000;
	Y_w   = NRFRX[7]-30000;
	Z_w   = NRFRX[8]-30000;
 
	NRFTX[0] = accelerator;
	NRFTX[1] = Pitch_ta    +30000;
	NRFTX[2] = Roll_ta     +30000;
	NRFTX[3] = Yaw_ta      +30000;
	NRFTX[4] = 1;

	UART1_ReportIMU(X_g,Y_g,Z_g,X_w,Y_w,Z_w,RX_speed,TX_speed,accelerator,roll,pitch,yaw/10);
}
void TIM3_IRQHandler(void)		
{	
	static u8 ms10 = 0;				//中断次数计数器
	if (TIM_GetITStatus(TIM3, TIM_IT_Update) != RESET)    //检查TIM3更新中断发生与否
	{
		TIM_ClearITPendingBit(TIM3, TIM_IT_Update  );       //清除TIMx更新中断标志 
		
		ms10++;
				
		TME_5ms();
		if(ms10==2) 
		{
		ms10=0;					
		TME_10ms();
		}
	}
}

