/** ###################################################################
 *     COPYRIGHT NOTICE
 *     Copyright (c) 2015,乐山师范物电学院创新实践中心
 *     All rights reserved.
 *     技术讨论：智能车论坛  乐师逐日飞翔  http://www.znczz.com/forum-81-1.html
 *
 *     除注明出处外，以下所有内容版权均属乐山师范物电学院创新实践中心所有，未经允许，不得用于商业用途，
 *     修改内容时必须保留乐师创新实践中心的版权声明。
 *
 * @author     乐师创新实践中心
 * @version    v1.0
 * @date       2015-05-25
 ** ###################################################################*/

#ifndef _OLED12864_H
#define _OLED12864_H
#include "headfile.h"



/*****以下管脚用户结合硬件自行更改*****/
#define LCD_SCL PORTB_PB1       //D0
#define LCD_SDA	PORTB_PB2       //D1
#define LCD_RST PORTB_PB3       //RST
#define LCD_DC  PORTB_PB4       //DC
/**************************************/

#define XLevelL		0x00
#define XLevelH		0x10
#define XLevel		((XLevelH&0x0F)*16+XLevelL)
#define Max_Column	128
#define Max_Row		  64
#define	Brightness	0xCF 


#define X_WIDTH 128
#define Y_WIDTH 64



 extern byte beyond96x64[512];
 extern byte beyond64x64[512];
 
 void LCD_Init(void);
 void LCD_CLS(void);
 void LCD_P6x8Str(byte x,byte y,byte ch[]);
 void oledt(byte x,byte y,int t);
 void oledt1(byte x,byte y,int t);
 void oledt2(byte x,byte y,int t);
 void LCD_P8x16Str(byte x,byte y,byte ch[]);
 void LCD_P14x16Str(byte x,byte y,byte ch[]);
 void LCD_Print(byte x, byte y, byte ch[]);
 void LCD_PutPixel(byte x,byte y);
 void LCD_Rectangle(byte x1,byte y1,byte x2,byte y2,byte gif);
 void Draw_BMP(byte x0,byte y0,byte x1,byte y1,byte bmp[]); 
 void LCD_Fill(byte dat);
#endif

