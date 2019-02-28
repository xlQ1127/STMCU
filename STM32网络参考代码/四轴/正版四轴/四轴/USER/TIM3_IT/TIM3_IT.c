#include  "TIM3_IT.h"
#include  "imu.h"
#include  "mpu6050.h"
#include  "control.h"
#include  "NRF24L01.h"
#include  "usart.h"
#include  "adc.h"

#include "IIC.h"

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

unsigned int  RX_speed;
unsigned int  TX_speed;
unsigned int  ms;

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
	GET_MPU_DATA();	
  IMUupdate(X_w*Gyro_Gr,Y_w*Gyro_Gr,Z_w*Gyro_Gr,X_g_av,Y_g_av,Z_g_av);
  CONTROL(roll,pitch,yaw,0,0,0,accelerator,X_w*Gyro_Gr,Y_w*Gyro_Gr,Z_w);
}
void TME_10ms(void)
{
	RX_TX_Times();
	Nrf_Check_Event();
  if(NRFRX[0]>4000)
	accelerator  = accelerator;
  else	
	accelerator  = NRFRX[0]/4;
		
	Pitch_ta     = NRFRX[1]-30000;
	Roll_ta      = NRFRX[2]-30000;	
	Yaw_ta       = NRFRX[3]-30000;
	POWER        = 1;//NRFRX[4];

	NRFTX[0] = roll  * 100+30000;
	NRFTX[1] = pitch * 100+30000;
	NRFTX[2] = yaw   * 100+30000;
	NRFTX[3] = X_g_av  + 30000;
	NRFTX[4] = Y_g_av  + 30000;
	NRFTX[5] = Z_g_av  + 30000;
	NRFTX[6] = X_w   + 30000;
	NRFTX[7] = Y_w   + 30000;
	NRFTX[8] = Z_w   + 30000;
	NRFTX[9] = ADC_ConvertedValue;

	NRF_TxPacket(TxBuf,24);		
}
void TIM3_IRQHandler(void)		
{	
	static u8 ms10 = 0;				
	if (TIM_GetITStatus(TIM3, TIM_IT_Update) != RESET)    //检查TIM3更新中断发生与否
	{
		TIM_ClearITPendingBit(TIM3, TIM_IT_Update);         //清除TIMx更新中断标志 
		
		ms10++;
		
		TME_5ms();		
		if(ms10==2) 
		{
		ms10=0;					
		TME_10ms();
		}	
	}
}

//  UART1_ReportIMU(X_g_av,Y_g_av,Z_g_av,X_w,Y_w,Z_w,RX_speed,TX_speed,VCC*10,roll*100,pitch*100,yaw*10);	


