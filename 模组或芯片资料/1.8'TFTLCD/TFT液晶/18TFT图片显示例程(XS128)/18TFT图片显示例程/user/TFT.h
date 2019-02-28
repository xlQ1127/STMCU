#ifndef _TFT_h
#define _TFT_h

#include "headfile.h"

/******常用颜色*****/
#define RED     0XF800    //红色
#define GREEN   0X07E0    //绿色
#define BLUE    0X001F    //蓝色
#define BRED    0XF81F
#define GRED    0XFFE0    //灰色
#define GBLUE   0X07FF    //
#define BLACK   0X0000    //黑色
#define WHITE   0XFFFF    //白色
#define YELLOW  0xFFE0    //黄色


//定义写字笔的颜色
#define PENCOLOR RED

//定义背景颜色
#define BGCOLOR	 WHITE

extern const unsigned char gImage_qq[];

void lcd_initial(void);
void dsp_single_colour(int color);
void showimage(const unsigned char *p); //显示40*40 QQ图片
void lcd_showchar(word x,word y,byte dat);
void lcd_showint8(word x,word y,char dat);
void lcd_showint16(word x,word y,int dat);
void lcd_showstr(word x,word y,byte dat[]);           

#endif