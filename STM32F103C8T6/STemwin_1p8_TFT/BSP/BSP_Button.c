#include "BSP_Button.h"
#include "GUI.h"

uint8_t Scan_Button(void)
{
uint8_t Button_Num=0;
	
  if(HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_4)==GPIO_PIN_RESET) 
	{ 
		HAL_Delay(8);
		if(HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_4)==GPIO_PIN_RESET) 
		{
     Button_Num=1 ; 
		}
	}	
	
  if(HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_5)==GPIO_PIN_RESET) 
	{ 
		HAL_Delay(8);
		if(HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_5)==GPIO_PIN_RESET) 
		{
		Button_Num=2 ; 
		}
	}

  if(HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_6)==GPIO_PIN_RESET) 
	{ 
		HAL_Delay(8);
		if(HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_6)==GPIO_PIN_RESET) 
		{
			Button_Num=3 ; 
		}
	}
	
  if(HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_7)==GPIO_PIN_RESET) 
	{ 
		HAL_Delay(8);
		if(HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_7)==GPIO_PIN_RESET) 
		{
		Button_Num=4 ;
		}
	}
	
  if(HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_8)==GPIO_PIN_RESET) 
	{ 
		HAL_Delay(8);
		if(HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_8)==GPIO_PIN_RESET) 
		Button_Num=5 ; 
	}
	
  if(HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_9)==GPIO_PIN_RESET) 
	{ 
		HAL_Delay(8);
			if(HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_9)==GPIO_PIN_RESET) 
			{
				Button_Num=6 ; 
			}
		
	}

	return  Button_Num;
}

void CD(void)
{
  uint8_t index;


  index=Scan_Button();
		
		
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
