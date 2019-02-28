#include "stm32f4xx.h"

#include "delay.h"
#include "led.h"
#include "timer.h"
#include "pwm_output.h"
#include "usart.h"
#include "pwm_in.h"
#include "MPU6050.h"
#include "IIC.h"
#include "imu.h"
#include "control.h"

/*------------------------------------------------------------------------------------------*/
/*               						global varibles                    				    */
/*------------------------------------------------------------------------------------------*/
unsigned char tim2flag=0;				   //定时期2中断任务执行标志
unsigned char readyflag=0;				   //任务执行标志		0:初始化kp,kd;1:电机执行许可；2:执行电机执行许可和发送角度信息
/*------------------------------------------------------------------------------------------*/
/*                                     function code	                                    */
/*------------------------------------------------------------------------------------------*/


void TIM2_IRQHandler(void);
void NVIC_Configuration(void);
void USART3_IRQHandler(void);



int main(void)
{
		float Roll=0;
		float Pitch=0;
		int i=0;

		LED_Init();

		LED1_OFF;
		LED2_OFF;
		LED3_OFF;
		LED4_OFF;

		delay_init(168);		//延时初始化  并启动开机时间。

		NVIC_Configuration();
		usart_x_init(115200);
		Timer2_Init(40,8399);
		pwm_in_init();
		
		TIM5_PWM_Init();		//PWM输出初始化250hz
		TIM4_PWM_Init();		//50hz
		delay_ms(100);
		
		for(i=0;i<10000;i++)								//用以初始化电调航程的时间
		{
		 	TIM5_PWM_OUTPUT(pwmout2,pwmout2,pwmout2,pwmout2);
			delay_ms(1);
		}


		delay_ms(100);
		I2C_GPIO_Config(); 
		delay_ms(100);
		Init_MPU6050();
		
		Acc_Correct();
	    Gyro_Correct();

		PID_controllerInit();
		controlmiddleinit(pwmout1,pwmout2,pwmout3,pwmout4);	 //	  pwmout1:横滚	  pwmout2:油门
															 //	  pwmout3:俯仰	  pwmout4:航向	
																	
//		printf("\n\rUSARTx test:\n\r");

		delay_ms(500);
		delay_ms(500);

		LED1_OFF;
		LED2_ON;
		LED3_ON;
		LED4_OFF;
		
		while(1)
		{	
		     if(tim2flag!=0)
				{
							 tim2flag=0;			 

						//	 READ_MPU6050();  	
						//	 MPU6050_TEST();

							 IMUdataprepare();
							 IMUupdate(GyroFinal.X,GyroFinal.Y,GyroFinal.Z,AccFinal.X,AccFinal.Y,AccFinal.Z);
					
							 Roll=(float)atan2(AccFinal.Y,AccFinal.Z)*57.295779513;    //X轴角度值 
							 Pitch=-(float)atan2(AccFinal.X,AccFinal.Z)*57.295779513;  //Y轴角度值

					
						//	 SendData(Q_ANGLE.Pitch*10,Pitch*10,Q_ANGLE.Roll*10,Roll*10);
							 SendData(Q_ANGLE.Yaw*10,Q_ANGLE.Pitch*10,Q_ANGLE.Roll*10,0);	 //配合diIMU上位机，单位度*10  红 蓝 青 黄
						//	 SendData(0,Pitch*10,0,Roll*10);		
						//	 SendData(0,GyroFinal.X*10,0,GyroFinal.Y*10);		

							 Getdesireddata(pwmout1,pwmout2,pwmout3,pwmout4);

						//	 Q_ANGLE.Roll=-Q_ANGLE.Roll;
							 PID_CAL();
							 TIM5_PWM_OUTPUT(MOTOR1,MOTOR2,MOTOR3,MOTOR4);		//MOTOR1    REAR_R  后右电机
																				//MOTOR2	FRONT_R 前右电机
																				//MOTOR3	REAR_L  后左电机
																				//MOTOR4	FRONT_L 前左电机	
						//	 TIM5_PWM_OUTPUT(pwmout2,pwmout2,pwmout2,pwmout2);
							 platform_control(Q_ANGLE.Roll,Q_ANGLE.Pitch,0,0);
							 TIM4_PWM_OUTPUT(servo_Roll,servo_Pitch);
						//	 SendData(Q_ANGLE.Roll*10,(&pidRoll)->outP,(&pidRoll)->outD,0);
						//	 SendData(MOTOR1*10,MOTOR2*10,MOTOR3*10,MOTOR4*10);
							 LED1_Tog;
				}
		}	
}

void NVIC_Configuration(void)
{ 
  	NVIC_InitTypeDef NVIC_InitStructure;  

	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);					//中断分组0		先占优先级4位。从优先级0位
//Usart1 NVIC 配置

    NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=3 ;
//	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;		//
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQ通道使能
	NVIC_Init(&NVIC_InitStructure);	//根据NVIC_InitStruct中指定的参数初始化外设NVIC寄存器USART1

//TIM2定时中断设置

	NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;  //TIM3中断
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;  //先占优先级0级
//	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;  //从优先级3级
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; //IRQ通道被使能
	NVIC_Init(&NVIC_InitStructure);  //根据NVIC_InitStruct中指定的参数初始化外设NVIC寄存器
		
				
//中断分组初始化
	NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn;  //TIM5中断
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;  //先占优先级2级
//	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;  //从优先级0级
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; //IRQ通道被使能
	NVIC_Init(&NVIC_InitStructure);  //根据NVIC_InitStruct中指定的参数初始化外设NVIC寄存器	
		
}


void TIM2_IRQHandler(void)   //TIM2中断
{	

	if (TIM_GetITStatus(TIM2, TIM_IT_Update) != RESET) //检查指定的TIM中断发生与否:TIM 中断源 
		{
		TIM_ClearITPendingBit(TIM2, TIM_IT_Update  );  //清除TIMx的中断待处理位:TIM 中断源 
		}
//	LED4_Tog;
	tim2flag++;
	
}

void USART3_IRQHandler(void)                	//串口1中断服务程序
{
		char ch;

		LED1_OFF;
		LED3_ON;
		
		if(USART_GetITStatus(USART3, USART_IT_RXNE) != RESET)  //接收中断
		{
			ch =USART_ReceiveData(USART3);//(USART3->DR);	//读取接收到的数据			
			USART3_SendData(ch);
				 
     	} 	 		
				
}

