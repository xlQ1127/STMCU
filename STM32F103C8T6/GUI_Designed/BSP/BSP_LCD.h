#ifndef __BSP_LCD_H__
#define __BSP_LCD_H__		
	 
#include "stdlib.h"


#include "main.h"
#include "stm32f1xx_hal.h"

//SPI显示屏接口
//LCD_RST

                         
#define SPILCD_RST_SET   HAL_GPIO_WritePin(GPIOA, TFT_RST_Pin,GPIO_PIN_SET) 
#define SPILCD_RST_RESET HAL_GPIO_WritePin(GPIOA, TFT_RST_Pin,GPIO_PIN_RESET)  
//LCD_RS//dc  
#define SPILCD_RS_SET    HAL_GPIO_WritePin(GPIOB, TFT_AO_Pin,GPIO_PIN_SET) 
#define SPILCD_RS_RESET  HAL_GPIO_WritePin(GPIOB, TFT_AO_Pin,GPIO_PIN_RESET)  
//LCD_CS  
#define SPILCD_CS_SET    HAL_GPIO_WritePin(GPIOA, TFT_CS_Pin,GPIO_PIN_SET) 
#define SPILCD_CS_RESET  HAL_GPIO_WritePin(GPIOA, TFT_CS_Pin,GPIO_PIN_RESET) 

  
//LCD重要参数集
typedef struct  
{ 					    
	uint16_t width;			//LCD 宽度
	uint16_t height;			//LCD 高度
	uint16_t id;				//LCD ID
	uint8_t	wramcmd;		//开始写gram指令
	uint8_t  setxcmd;		//设置x坐标指令
	uint8_t  setycmd;		//设置y坐标指令	 
}_lcd_dev; 	  

//LCD参数
extern _lcd_dev lcddev;	//管理LCD重要参数
//LCD的画笔颜色和背景色	   
extern uint16_t  POINT_COLOR;//默认红色    
extern uint16_t  BACK_COLOR; //背景颜色.默认为白色


//////////////////////////////////////////////////////////////////////////////////	 
//-----------------LCD端口定义---------------- 
#define	LCD_REST PBout(1) //LCD REST    		 PB1 	    
//LCD地址结构体
typedef struct
{
	uint16_t LCD_REG;
	uint16_t LCD_RAM;
} LCD_TypeDef;
//使用NOR/SRAM的 Bank1.sector4,地址位HADDR[27,26]=11 A10作为数据命令区分线 
//注意设置时STM32内部会右移一位对其! 111110=0X3E			    
#define LCD_BASE        ((uint32_t)(0x60000000 | 0x0007FFFE))
#define LCD             ((LCD_TypeDef *) LCD_BASE)
//////////////////////////////////////////////////////////////////////////////////

//画笔颜色
#define WHITE         	 0xFFFF
#define BLACK         	 0x0000	  
#define BLUE         	 0x001F  
#define BRED             0XF81F
#define GRED 			 0XFFE0
#define GBLUE			 0X07FF
#define RED           	 0xF800
#define MAGENTA       	 0xF81F
#define GREEN         	 0x07E0
#define CYAN          	 0x7FFF
#define YELLOW        	 0xFFE0
#define BROWN 			 0XBC40 //棕色
#define BRRED 			 0XFC07 //棕红色
#define GRAY  			 0X8430 //灰色
//GUI颜色

#define DARKBLUE      	 0X01CF	//深蓝色
#define LIGHTBLUE      	 0X7D7C	//浅蓝色  
#define GRAYBLUE       	 0X5458 //灰蓝色
//以上三色为PANEL的颜色 
 
#define LIGHTGREEN     	 0X841F //浅绿色
//#define LIGHTGRAY        0XEF5B //浅灰色(PANNEL)
#define LGRAY 			 0XC618 //浅灰色(PANNEL),窗体背景色

#define LGRAYBLUE        0XA651 //浅灰蓝色(中间层颜色)
#define LBBLUE           0X2B12 //浅棕蓝色(选择条目的反色)


void SPI_WriteByte(uint8_t Data);
void LCD_WR_REG(uint8_t regval);
void LCD_WR_DATA(uint16_t data);
void LCD_WR_DATA8(uint8_t da); //写8位数据
void LCD_DrawPoint(uint16_t x,uint16_t y);
void LCD_Init(void);
void LCD_Clear(uint16_t color);

#endif 


	 
	 



