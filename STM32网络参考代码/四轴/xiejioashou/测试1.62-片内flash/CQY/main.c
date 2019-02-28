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

extern union 
{
	
u32 Data[8];
float F_Data[8];

}ftoc;

//-----------------------------------------------------------------//
u16 dir_time=0;
//---------------------------------------------------------------------//

void IIC_Reboot() //ÖØÆôiic×ÜÏß
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

ftoc.F_Data[0]=0.1010101;
//F_Data[1]=0.202020;
//F_Data[2]=0.202020;
// F_Data[4]=0.202020;
// F_Data[5]=0.101010;
// F_Data[6]=0.202020;
// F_Data[7]=0.101010;
//Writeflash();
delay_ms(10);
while(1)
{
     ftoc.Data[0]=*p;
 //  F_Data[2]=*(p+4);
	
// F_Data[3]=0.231695;
// F_Data[4]=0.202020;
// F_Data[5]=0.101010;
// F_Data[6]=0.202020;
// F_Data[7]=0.101010;
	
	
  GPIO_ResetBits(GPIOB,GPIO_Pin_0);
	Delay_s(10);
	GPIO_SetBits(GPIOB,GPIO_Pin_0);	
  Delay_s(10);


}

}
 

void TIM3_IRQHandler()//1ms
{TIM_ClearITPendingBit(TIM3,TIM_FLAG_Update); 
	

}


