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

//-----------------------------------------------------------------//
u8 FLy_count=0;
//---------------------------------------------------------------------//
int main()


{


RCC_Configuration();
I2C_Configuration();
GPIO_Configuration();	
Tim2_init();

SPIx_Init(); 
Init_MPU6050() ;	

	//NRFSetTxMode(TxDate);		



TIM3_INT();	
TIM4_INT();	

PID_INIT();	

NRF24L01Int();		
NRFSetRXMode();	
while(1)
{
	
	if(FLY_Enable)	
	GPIO_ResetBits(GPIOB,GPIO_Pin_0);	
	else
	GPIO_SetBits(GPIOB,GPIO_Pin_0);		
	

}

}
 

void TIM3_IRQHandler()//1ms
{TIM_ClearITPendingBit(TIM3,TIM_FLAG_Update); 
	
	READ_MPU6050();	
	Cal_TsData();	
	IMUupdate(GRY_F.X,GRY_F.Y,GRY_F.Z,ACC_AVG.X*5,ACC_AVG.Y*5,ACC_AVG.Z*5);
	PID_CAL();

	Get_RFdata();

//RF_send(1,(Q_ANGLE.Pitch)*12+2048);
//RF_send(0,(Q_ANGLE.Rool)*12+2048);	
//RF_send(2,(Q_ANGLE.Yaw )*12+2048);
//RF_send(2,((ACC_RealData.X)/(-8))+2048);	
//RF_send(3,((ACC_RealData.Y)/8)+2048);
//RF_send(3,Data_time/4);
	FLy_count++;
  if(FLy_count==20)FLy_count=0;	
}


