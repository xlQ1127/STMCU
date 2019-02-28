#include "SysTick.h"

uint32_t TimingDelay;

void SysTick_Init(void)   
{   
      /* SystemFrequency / 1000    1ms  
       * SystemFrequency / 100000  10us   
       * SystemFrequency / 1000000 1us   
       */  
      if (SysTick_Config(SystemCoreClock / 1000000))   
      {    
          while (1);   
      }    
      SysTick->CTRL &= ~ SysTick_CTRL_ENABLE_Msk;   
} 
void TimingDelay_Decrement(void)   
{   
   if (TimingDelay != 0x00)   
    {    
      TimingDelay--;   
    }   
}
void Delay_us(uint32_t nTime)   
{    
    TimingDelay = nTime;       
    SysTick->CTRL |=  SysTick_CTRL_ENABLE_Msk;   
    while(TimingDelay != 0);   
} 
void SysTick_Handler(void)
{
	TimingDelay_Decrement();
}

