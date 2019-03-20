#include "common.h"
#include "isr.h"
#include "wifi.h"
#include "FaceReco.h"
#include "systemstatus.h"



void USART2_IRQHandler(void)
{
	unsigned char temp=0;
        
        SystemData.LPWCount = 0;
        
        while(USART2->ISR & (1 << 5))
	{
		temp = USART2->RDR & 0xFF;
		uart_receive_input(temp);  /////wifiÄ£¿é´¦Àí
	}
}


void USART1_IRQHandler(void)
{
	unsigned char temp=0;
        
        SystemData.LPWCount = 0;
        
        while(USART1->ISR & (1 << 5))
	{
		temp = USART1->RDR & 0xFF;
		RecvFaceRecoData(temp);
	}
        if(SystemData.FaceCheckOpen == 1)
        {
          SystemData.VisitorNum = FaceDataCheck();
          if(SystemData.VisitorNum >= -1)
          {
            SystemData.FaceCheckOpen = 0;
            if(SystemData.VisitorNum >= 0)
              SystemData.DoorCmd = DOOROPEN;
          }     
        }
        else
        {
          FaceReco_Init();
        }
}

void TIM1_BRK_UP_TRG_COM_IRQHandler(void)
{
	if(TIM1_INT)
	{
		TIM1_CLEAR;
#if (LPWOPEN == 1)
                SystemData.LPWCount++;
                if(SystemData.LPWCount > 120)
                {
                    SystemData.LPWCount = 0;
                    SystemData.DoorCmd = DOORSLEEP;
                }
#endif
		              SystemData.ADOut[1] = SystemData.ADBuff[1] * 2475 / 80;
                SystemData.ADOut[2] = SystemData.ADBuff[2] * 2475 / 80;
                
                if(SystemData.ADOut[2] < 6000)
                {
                  SystemData.BATWarnCount++;
                  if(SystemData.BATWarnCount > 100)
                  {
                    SystemData.DoorCmd = BATWARN;
                    if(SystemData.ADOut[2] > 5500)
                      SystemData.BatteryStatus = medium;
                    else if(SystemData.ADOut[2] > 5000)
                      SystemData.BatteryStatus = low;
                    else
                      SystemData.BatteryStatus = poweroff;
                    SystemData.BATWarnCount = 0;
                  }
                   
                }
                else
                {
                  SystemData.BATWarnCount = 0;
                  SystemData.BatteryStatus = high;
                }
                
                RTC_GetDateTime(&SystemData.SystemTime);
	}
	
}

void EXTI0_1_IRQHandler(void)
{
  EXTI_ClearITPendingBit(EXTI_Line0);
  SystemData.LPWCount = 0;
}

void EXTI4_15_IRQHandler(void)
{
  if(EXTI->PR & (1 << 15))
  {
    NVIC_SystemReset();
  }
  if(EXTI->PR & (1 << 13) && SystemData.DoorStatus == DOOROPEN)
  {
    SystemData.DoorCmd = DOORCLOSE;
  }
  else if(EXTI->PR & (1 << 14) && SystemData.DoorStatus == DOORCLOSE)
  {
    SystemData.DoorCmd = DOOROPEN;
  }
  EXTI_ClearITPendingBit(EXTI_Line8);
  EXTI_ClearITPendingBit(EXTI_Line13);
  EXTI_ClearITPendingBit(EXTI_Line14);
  EXTI_ClearITPendingBit(EXTI_Line15); 
  SystemData.LPWCount = 0;
}