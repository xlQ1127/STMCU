//欢迎光临我们的公司的主页：www.hjrkj.com  深圳市好钜润科技有限公司
//欢迎光临我们的官方淘宝店：http://tiky.taobao.com
/* Includes ------------------------------------------------------------------*/
#include "stm32f10x.h"
#include "gui.h"
#include "Tiky_LCD.h" 	  
#include  "touch.h"
#include "MM1_240.h"
//#include "MM3_240.h"
/*******************************************************************************
* Function Name  : main.
* Description    : Main routine.
* Input          : None.
* Output         : None.
* Return         : None.
*******************************************************************************/
extern void MainTask(void);
#define  SystemFrequency  72000000
int main(void)
{	
  SystemInit();//初始化STM32
	Lcd_Initialize();
	SysTick_Config(SystemFrequency / 100);//10mS定时，1000为1ms
	Touch_GPIO_Config();
	GUI_Init();	//初始化GUI
	
	GUI_SetFont(&GUI_Font24_ASCII);//设置24号ASCII字体
	GUI_SetColor(GUI_WHITE);//设置前景色为白色
	GUI_SetBkColor(GUI_RED);//设置背景色为红色
  GUI_Clear();//清屏
	GUI_DispStringAt("Wellcome to Tiky LCD world", 5, 15);//在(200, 15)的座标上写上一行字
	GUI_Delay(200);
		LCD_Fill_Pic(0,0,320,240, gImage_MM1_240);GUI_Delay(200);
//	LCD_Fill_Pic(0,0,240,320, gImage_MM3_240);GUI_Delay(200);
	while(1)MainTask();
	
}



/******************* (C) COPYRIGHT 2007 STMicroelectronics *****END OF FILE****/
