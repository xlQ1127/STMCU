#include "BSP_Button.h"



uint8_t Scan_Button(void)
{
uint8_t Button_Num=0;
	
  if(HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_10)==GPIO_PIN_RESET) 
	{ 
		 vTaskDelay(5);
		if(HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_10)==GPIO_PIN_RESET) 
		{
     Button_Num=1 ; 
		}
	}	
	
  if(HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_11)==GPIO_PIN_RESET) 
	{ 
		     vTaskDelay(5);
		if(HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_11)==GPIO_PIN_RESET) 
		{
		Button_Num=2 ; 
		}
	}

  if(HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_12)==GPIO_PIN_RESET) 
	{ 
		 vTaskDelay(5);
		if(HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_12)==GPIO_PIN_RESET) 
		{
			Button_Num=3 ; 
		}
	}
	
  if(HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_13)==GPIO_PIN_RESET) 
	{ 
		vTaskDelay(5);
		if(HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_13)==GPIO_PIN_RESET) 
		{
		Button_Num=4 ;
		}
	}
	
  if(HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_14)==GPIO_PIN_RESET) 
	{ 
		vTaskDelay(5);
		if(HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_14)==GPIO_PIN_RESET) 
		Button_Num=5 ; 
	}
	
  if(HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_15)==GPIO_PIN_RESET) 
	{ 
		vTaskDelay(5);
			if(HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_15)==GPIO_PIN_RESET) 
			{
				Button_Num=6 ; 
			}
		
	}

	return  Button_Num;
}

void CD(uint8_t index)
{
		if(index!=0)
		{
				 switch (index)
				 {
					 case 1:
						 GUI_SendKeyMsg(GUI_KEY_BACKTAB,1);
					   
						 break;
					 
					 case 2:
						 GUI_SendKeyMsg(GUI_KEY_RIGHT,1);
					  
						 break;
					 
					 case 3:
						 GUI_SendKeyMsg(GUI_KEY_PGDOWN,1);
					  
						 break;
					 
					 case 4:
						 GUI_SendKeyMsg(GUI_KEY_ENTER,1);
					  
					   break;
					 
					 case 5:
						 GUI_SendKeyMsg(GUI_KEY_LEFT,1);	
            					 
             break;					 
					 
					 case 6:
						 GUI_SendKeyMsg(GUI_KEY_TAB,1);				 			 
             break;			
				 }
			 
		}

}
