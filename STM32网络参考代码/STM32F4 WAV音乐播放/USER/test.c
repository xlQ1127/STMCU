#include <stm32f10x_lib.h>
#include "sys.h"
#include "usart.h"		
#include "delay.h"	
#include "led.h" 
#include "key.h"
#include "lcd.h"	   
#include "dma.h"
#include "mmc_sd.h"
#include "text.h"
#include "fat.h"
#include "fontupd.h"
#include "sysfile.h"
#include "spi.h"
#include "jpegbmp.h"
#include "dac.h"
#include "wavplay.h"
//Mini STM32开发板范例代码25
//图片显示 实验
//正点原子@ALIENTEK
//2010.6.18
u32 sd_Capacity;
u8 volume;			 					
int main(void)
{	 		 
	u8 key;
	SD_Error i;		  
	FileInfoStruct *FileInfo;	   		 
	u16 pic_cnt=0;//当前目录下图片文件的个数
	u16 index=0;  //当前选择的文件编号	   
	u16 time=0;    	     	  					   

	Stm32_Clock_Init(9);//系统时钟设置
	delay_init(72);		//延时初始化
	uart_init(72,9600); //串口1初始化  	  
	LCD_Init();			//初始化液晶		  
	KEY_Init();			//按键初始化
	LED_Init();         //LED初始化
	//SPI_FLASH_Init(); //SPI FLASH使能 	  
	POINT_COLOR=RED;		     
	LCD_ShowString(60,50,"Mini STM32");	 
	MyDAC_Init();
	POINT_COLOR=RED;
	LCD_ShowString(60,50,"Mini STM32");
	i=SD_InitAndConfig();
	while(FAT_Init())//FAT 错误
	{
		LCD_ShowString(60,130,"fat wrong");  
		
		if(i!=SD_OK)LCD_ShowString(60,150,"SD wrong!");//SD卡初始化失败 
			  
		delay_ms(500);
		LCD_ShowNum(120,170,FAT_Init(),2,16);
		delay_ms(500);
		LED1=!LED1;	   
	}	   				 
	while(SysInfoGet(1))//得到音乐文件夹  
	{
		LCD_ShowString(60,130,"can't find file");  
		delay_ms(500);  
		FAT_Init();
		SD_Init();
		LED1=!LED1;
		LCD_Fill(60,130,240,170,WHITE);//清除显示			  
		delay_ms(500);		
	}
	LCD_ShowString(60,130,"start to play"); 
	delay_ms(1000);
	Cur_Dir_Cluster=PICCLUSTER;
	volume=35;//预置音量
	while(1)
	{	    			 
		pic_cnt=0;	 
		Get_File_Info(Cur_Dir_Cluster,FileInfo,T_WAV,&pic_cnt);//获取当前文件夹下面的目标文件个数 		    
		if(pic_cnt==0)//没有WAV文件
		{
			LCD_Clear(WHITE);//清屏	   
			while(1)
			{	  
				if(time%2==0)LCD_ShowString(32,150,"No WAV file");		 
				else LCD_Clear(WHITE);
				time++;
				delay_ms(300);
			}
		}								   
		FileInfo=&F_Info[0];//开辟暂存空间.
		index=1;
		while(1)
		{
			Get_File_Info(Cur_Dir_Cluster,FileInfo,T_WAV,&index);//得到文件信息	 
			LCD_Clear(WHITE);//清屏,加载下一幅图片的时候,一定清屏
			POINT_COLOR=RED; 				
			LCD_ShowString(60,10,FileInfo->F_Name);//显示文件名字
			Playwav(FileInfo);//开始播放			     			
			key=KEY_Scan();
			if(key==1)break;//下一首
			else if(key==2)//上一首
			{
				if(index>1)index-=2;
				else index=pic_cnt-1;
				break;
			}
			delay_ms(500);
			index++;
			if(index>pic_cnt)index=1;//播放第一首	  	 		 
		}
	}			   		 			  
}		 





