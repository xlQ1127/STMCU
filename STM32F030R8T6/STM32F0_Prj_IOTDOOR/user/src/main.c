/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2018 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
    
/* Includes ------------------------------------------------------------------*/
#include "common.h"
#include "GPIO_Init.h"
#include "wifi.h"
#include "FaceReco.h"
#include "systemstatus.h"
#include "motor.h"
#include "wtn6040.h"
/* Includes ------------------------------------------------------------------*/

void Board_init(void)
{
  BoardGPIO_init();
  Delay_Init();
  adc_init();
  uart1_init();//串口
  uart2_init();
  TIM1_init(500);//定时器
  rtc_init();//时钟
	//打开外设开关 BR    关闭  BS
	GPIOB->BSRR = GPIO_BSRR_BR_8;   
}
    
    
 int main(void)
{
	DisableIRQ;

        /* Initialize all paraments */
	SystemData.FaceCheckOpen = 0;
	SystemData.VisitorNum = -1;
	SystemData.DoorCmd = DOORWAIT;
        SystemData.DoorStatus = DOORCLOSE;
        SystemData.BatteryStatus = high;
        /* Initialize all paraments */
        
	/* Initialize all configured peripherals */
	Board_init();
        VDD_ON;
        
        SendMessage(0);
        delay_ms(100);
        
        
	rtc_init();
        
	LED0_ON;
 LED1_ON;
	delay_ms(500);
	LED0_OFF;
        
	/* Initialize all configured peripherals */
	FaceReco_Init();
	wifi_protocol_init();    
	
	IRLED0_ON;//红外LED
	IRLED1_ON;
	
        
	NVIC_EnableIRQ(TIM1_BRK_UP_TRG_COM_IRQn);
	NVIC_EnableIRQ(USART1_IRQn);
	NVIC_EnableIRQ(USART2_IRQn);
	NVIC_EnableIRQ(EXTI0_1_IRQn);
	NVIC_EnableIRQ(EXTI4_15_IRQn);
	EnableIRQ;

        SendMessage(4);
        mcu_reset_wifi();
        mcu_start_wifitest();
     
	for (;;)
	{
		SystemData.DataIR[0] = IRLED0_IN;
		SystemData.DataIR[1] = IRLED1_IN;
		if (SystemData.DoorCmd == FACECHECK && SystemData.FaceCheckOpen == 0) ///人脸检测
		{
			FaceCMDSemd();
   SendMessage(15);
			SystemData.FaceCheckOpen = 1;
			SystemData.DoorCmd = DOORWAIT;
		}
		else if (SystemData.DoorCmd == DOOROPEN)   //////开门
		{
                        LED0_ON;
                        SendMessage(16);
			DoorOpen(&SystemData);
			SystemData.DoorCmd = DOORWAIT;
                        LED0_OFF;
			SendMessage(7);
                        SystemData.DoorStatus = DOOROPEN;
		}
		else if (SystemData.DoorCmd == DOORCLOSE)  //////关门
		{
                        LED0_ON;
			DoorClose(&SystemData);
			SystemData.DoorCmd = DOORWAIT;
                        LED0_OFF;
			SendMessage(8);
                        SystemData.DoorStatus = DOORCLOSE;
		}
                else if(SystemData.DoorCmd == BATWARN)   ///发出低电警告
                {
                        LED0_ON;
                        LED1_OFF;
                        SendMessage(1);
                        SystemData.DoorCmd = DOORWAIT;
                }
		else if (SystemData.DoorCmd == DOORSLEEP) ////睡眠模式
		{
			VDD_OFF;
                        LED1_OFF;
                        LED0_OFF;
                        SystemData.DoorCmd = DOORWAIT;
                        NVIC_DisableIRQ(TIM1_BRK_UP_TRG_COM_IRQn);
			SleepMode();
                        SystemInit();
                        VDD_ON;
                        Board_init();
                        LED1_ON;
                        LED0_OFF;
                        NVIC_EnableIRQ(TIM1_BRK_UP_TRG_COM_IRQn);
		}
                else if(SystemData.DoorCmd == NONET)  ////无网络
                {
                        SendMessage(13);
                        SystemData.DoorCmd = DOORWAIT;
                }
                else if(SystemData.DoorCmd == GETNET) ///有网络
                {
                        SendMessage(12);
                        SystemData.DoorCmd = DOORWAIT;
                }
                else
                {
                  SystemData.DoorCmd = DOORWAIT;
                }
		wifi_uart_service();
	}


	return 1;
}

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
