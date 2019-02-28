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


//-----------------------------------------------------------------//

//---------------------------------------------------------------------//
int main()


{


RCC_Configuration();
I2C_Configuration();
GPIO_Configuration();	
Tim2_init();

SPIx_Init(); 
Init_MPU6050() ;	
NRFSetTxMode(TxDate);	

GPIO_SetBits(GPIOB,GPIO_Pin_0);
TIM3_INT();	
	
while(1)
{
	


	
}

}
 

void TIM3_IRQHandler()//1ms
{GPIO_SetBits(GPIOB,GPIO_Pin_0); 
	

	READ_MPU6050();
  
	Cal_TsData();
	
	IMUupdate(GRY_F.X,GRY_F.Y,GRY_F.Z,ACC_F.X*5,ACC_F.Y*5,ACC_F.Z*5);
	
	RF_send(1,(Q_ANGLE.Pitch)*15+2048);

	RF_send(0,(Q_ANGLE.Rool)*15+2048);
	
	RF_send(2,((ACC_RealData.X)/16)+2048);
	
	RF_send(3,((ACC_RealData.Y)/16)+2048);

GPIO_ResetBits(GPIOB,GPIO_Pin_0);	
TIM_ClearITPendingBit(TIM3,TIM_FLAG_Update); 
}


