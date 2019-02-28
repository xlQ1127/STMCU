#include "stm32f10x.h"
#include "stdlib.h"
#include "math.h"
#include "delay.h" 
#include "NRF24L01.H"
#include "MPU6050.H"
#include "IIC.H"
#include "spi.H"
#include "ALL_config.h"
#include "var_global.h"
#include "Cal.h"
#include "Mot_crtl.h"
#include "HMC5883.H"
#include "flash.H"
#include "Bit_io.h"

#define PB13 PBin(13)
#define PB14 PBin(14)
#define PB15 PBin(15)

extern union 
{
	
u32 Data[8];
float F_Data[8];

}ftoc;

//-----------------------------------------------------------------//
u16 dir_time=0;

u16 Pulse_1=0,Pulse_2=0,Pulse_3=0;
//---------------------------------------------------------------------//

void IIC_Reboot() //重启iic总线
{ 
	GPIO_InitTypeDef GPIO_InitStructure; 
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6|GPIO_Pin_7;//  PB0-LED
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; 
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 
  GPIO_Init(GPIOB, &GPIO_InitStructure); 
 
	GPIO_ResetBits(GPIOB,GPIO_Pin_6);
  GPIO_ResetBits(GPIOB,GPIO_Pin_7);

  GPIO_SetBits(GPIOB,GPIO_Pin_6);
  GPIO_SetBits(GPIOB,GPIO_Pin_7);	


	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6|GPIO_Pin_7;//  PB0-LED
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; 
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_OD; 
  GPIO_Init(GPIOB, &GPIO_InitStructure);	

}


u32 *p=(u32 *)0x08008000;
int main()


{

RCC_Configuration();
GPIO_Configuration();	


TIM3_INT();	
TIM4_INT();
Exitinit1();
Exitinit2();
Exitinit3();	
while(1)
{
     

	
	
  GPIO_ResetBits(GPIOB,GPIO_Pin_0);
	Delay_s(10);
	GPIO_SetBits(GPIOB,GPIO_Pin_0);	
  Delay_s(10);


}

}
 

void TIM3_IRQHandler()//1ms
{TIM_ClearITPendingBit(TIM3,TIM_FLAG_Update); 
	

}


void EXTI15_10_IRQHandler()
{ 
//------------------------------------------------------//		
	if(EXTI_GetITStatus(EXTI_Line13)!= RESET) //PB13中断
	{	 EXTI_ClearITPendingBit(EXTI_Line13); 	
		if(PB13) TIM1->CNT=0;
		if(!PB13)Pulse_1=TIM1->CNT; 
  }
//------------------------------------------------------//	
	if(EXTI_GetITStatus(EXTI_Line14)!= RESET) //PB14中断	
	{	 EXTI_ClearITPendingBit(EXTI_Line14); 
		
		if(PB14)  TIM1->CNT=0;
		if(!PB14) Pulse_2=TIM1->CNT; 
  }
//------------------------------------------------------//	
	if(EXTI_GetITStatus(EXTI_Line15)!= RESET) //PB15中断
	{	EXTI_ClearITPendingBit(EXTI_Line15); 	
		if(PB15)TIM1->CNT=0;
		if(!PB15)Pulse_3=TIM1->CNT; 
  }	
}
