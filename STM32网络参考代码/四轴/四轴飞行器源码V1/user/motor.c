/***********************************************

标题: motor.c
作者: 秋阳电子
网址：http://qiuyangdz.taobao.com
日期: 2014/05/18
版本：v1.0
功能: 电机通道初始化及限幅输出
说明：
*************************************************/
#include "stm32f10x.h"
#include "motor.h" 
#include "usart.h"

u8 i;
/*************************************************

名称：motor_init(void)
功能：相关管脚及timer外设初始化（中断 定时时间）
输入参数：无
输出参数：无
返回值：  无
**************************************************/
void motor_init(void)
{
  TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
  TIM_OCInitTypeDef  TIM_OCInitStructure;
  GPIO_InitTypeDef GPIO_InitStructure;

  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
  													
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_9 | GPIO_Pin_14 | GPIO_Pin_15; 
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  
  GPIO_Init(GPIOB, &GPIO_InitStructure);

  /* TIM4 */
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);

  /* Time base configuration */
  TIM_TimeBaseStructure.TIM_Period = 2500 - 1;     
  TIM_TimeBaseStructure.TIM_Prescaler = 20 - 1;  
  TIM_TimeBaseStructure.TIM_ClockDivision = 0;
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  

  TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure);

  TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
  TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
  TIM_OCInitStructure.TIM_Pulse = 0;

  TIM_OC3Init(TIM4, &TIM_OCInitStructure);  
  TIM_OC4Init(TIM4, &TIM_OCInitStructure);  

  TIM_OC3PreloadConfig(TIM4, TIM_OCPreload_Enable);
  TIM_OC4PreloadConfig(TIM4, TIM_OCPreload_Enable);           

  /* TIM4 enable counter */
  TIM_Cmd(TIM4, ENABLE);

  /* TIM1 */
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);

  /* Time base configuration */
  TIM_TimeBaseStructure.TIM_Period = 2500 - 1;    
  TIM_TimeBaseStructure.TIM_Prescaler = 20 - 1;  
  TIM_TimeBaseStructure.TIM_ClockDivision = 0;
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  

  TIM_TimeBaseInit(TIM1, &TIM_TimeBaseStructure);

  TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Disable; 
  TIM_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Enable;
  TIM_OCInitStructure.TIM_OCNPolarity = TIM_OCNPolarity_High;
  TIM_OCInitStructure.TIM_Pulse = 0;

  TIM_OC2Init(TIM1, &TIM_OCInitStructure);  
  TIM_OC3Init(TIM1, &TIM_OCInitStructure);  

  TIM_OC2PreloadConfig(TIM1, TIM_OCPreload_Enable);
  TIM_OC3PreloadConfig(TIM1, TIM_OCPreload_Enable);
 
  /* TIM1 enable counter */
  TIM_Cmd(TIM1, ENABLE);
  
  TIM_CtrlPWMOutputs(TIM1, ENABLE);    
}
/*************************************************

名称：motor_control(s16 motor1, s16 motor2, s16 motor3, s16 motor4)
功能：电机输出PWM
输入参数：
   s16 motor1  1号电机输入
	 s16 motor2	 2号电机输入
	 s16 motor3  3号电机输入
	 s16 motor4  4号电机输入 
输出参数：限幅的电机输出值
返回值：  无
**************************************************/
void motor_control(s16 motor1, s16 motor2, s16 motor3, s16 motor4)
{
  if(motor1 > PWM_MOTOR_MAX)      
  {
    motor1 = PWM_MOTOR_MAX;
  }
  else if(motor1 < PWM_MOTOR_MIN)   
  {
    motor1 = PWM_MOTOR_MIN;
  }

  if(motor2 > PWM_MOTOR_MAX)      
  {
    motor2 = PWM_MOTOR_MAX;
  }
  else if(motor2 < PWM_MOTOR_MIN)   
  {
    motor2 = PWM_MOTOR_MIN;
  }

  if(motor3 > PWM_MOTOR_MAX)      
  {
    motor3 = PWM_MOTOR_MAX;
  }
  else if(motor3 < PWM_MOTOR_MIN)   
  {
    motor3 = PWM_MOTOR_MIN;
  }

  if(motor4 > PWM_MOTOR_MAX)      
  {
    motor4 = PWM_MOTOR_MAX;
  }
  else if(motor4 < PWM_MOTOR_MIN)   
  {
    motor4 = PWM_MOTOR_MIN;
  }

  PWM_MOTOR1 = motor1;
  PWM_MOTOR2 = motor2;
  PWM_MOTOR3 = motor3;
  PWM_MOTOR4 = motor4;
}
/***************************END OF FILE**********************************************************************/
