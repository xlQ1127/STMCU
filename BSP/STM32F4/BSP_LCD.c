#include "BSP_LCD.h"

static void LCD_Rst(void)
{
	HAL_Delay(100);
	Lcd_RST_Set;
	HAL_Delay(100);
	Lcd_RST_Reset;
	HAL_Delay(100);
	Lcd_RST_Set;
	HAL_Delay(100);
}

void WriteComm(uint16_t CMD)
{
	*(__IO uint16_t *)(Bank1_LCD_C) = CMD;
}

void WriteData(uint16_t tem_data)
{
	*(__IO uint16_t *)(Bank1_LCD_D) = tem_data;
}

/**********************************************
Lcd初始化函数
***********************************************/
void Lcd_Initialize(void)
{

	LCD_Rst();

	LCD_WR_REG(0x11, 0x2004); //Power control 2
	LCD_WR_REG(0x13, 0xCC00); //Power control 4
	LCD_WR_REG(0x15, 0x2600); //Power control 6
	LCD_WR_REG(0x14, 0x252A); //Power control 5
	//LCD_WR_REG(0x14,0x002A);
	LCD_WR_REG(0x12, 0x0033); //Power control 3
	LCD_WR_REG(0x13, 0xCC04); //Power control 4
	LCD_WR_REG(0x13, 0xCC06); //Power control 4
	LCD_WR_REG(0x13, 0xCC4F); //Power control 4
	LCD_WR_REG(0x13, 0x674F); //Power control 4
	LCD_WR_REG(0x11, 0x2003); //Power control 2
	HAL_Delay(1);
	LCD_WR_REG(0x30, 0x2609); //Gamma control 1
	LCD_WR_REG(0x31, 0x242C); //Gamma control 2
	LCD_WR_REG(0x32, 0x1F23); //Gamma control 3
	LCD_WR_REG(0x33, 0x2425); //Gamma control 4
	LCD_WR_REG(0x34, 0x2226); //Gamma control 5
	LCD_WR_REG(0x35, 0x2523); //Gamma control 6
	LCD_WR_REG(0x36, 0x1C1A); //Gamma control 7
	LCD_WR_REG(0x37, 0x131D); //Gamma control 8
	LCD_WR_REG(0x38, 0x0B11); //Gamma control 9
	LCD_WR_REG(0x39, 0x1210); //Gamma control 10
	LCD_WR_REG(0x3A, 0x1315); //Gamma control 11
	LCD_WR_REG(0x3B, 0x3619); //Gamma control 12
	LCD_WR_REG(0x3C, 0x0D00); //Gamma control 13
	LCD_WR_REG(0x3D, 0x000D); //Gamma control 14
	HAL_Delay(1);
	LCD_WR_REG(0x16, 0x0007); //Power control 7
	LCD_WR_REG(0x02, 0x0013); //LCD-Driving-waveform control
	LCD_WR_REG(0x03, 0x0009); //Entry mode   0x000A
	LCD_WR_REG(0x01, 0x1027); //Driver output control
	HAL_Delay(1);
	LCD_WR_REG(0x08, 0x0303); //Blank period control 1
	LCD_WR_REG(0x0A, 0x000B); //Frame cycle control 1
	LCD_WR_REG(0x0B, 0x0003); //Frame cycle control
	LCD_WR_REG(0x0C, 0x0000); //External interface control  // 18-bit RGB interface (one transfer/pixel)
	LCD_WR_REG(0x41, 0x0000); //Vertical scroll control
	LCD_WR_REG(0x50, 0x0000); //MDDI wake up control
	LCD_WR_REG(0x60, 0x0005); //MTP INIT
	LCD_WR_REG(0x70, 0x000B); //Timing of signal from GOE
	LCD_WR_REG(0x71, 0x0000); //Gate start pulse delay timing
	LCD_WR_REG(0x78, 0x0000); //Vcom Output Control
	LCD_WR_REG(0x7A, 0x0000); //Panel signal control 2
	LCD_WR_REG(0x79, 0x0007); //Panel signal control 1
	LCD_WR_REG(0x07, 0x0051); //Display control
	HAL_Delay(1);
	LCD_WR_REG(0x07, 0x0053); //Display control
	LCD_WR_REG(0x79, 0x0000); //Panel signal control 1
	WriteComm(0x22);
}

/******************************************
函数名：Lcd写命令函数
功能：向Lcd指定位置写入应有命令或数据
入口参数：Index 要寻址的寄存器地址
          ConfigTemp 写入的数据或命令值
******************************************/
void LCD_WR_REG(uint16_t Index, uint16_t CongfigTemp)
{
	*(__IO uint16_t *)(Bank1_LCD_C) = Index;
	*(__IO uint16_t *)(Bank1_LCD_D) = CongfigTemp;
}
/************************************************
函数名：Lcd写开始函数
功能：控制Lcd控制引脚 执行写操作
************************************************/
void Lcd_WR_Start(void)
{
	*(__IO uint16_t *)(Bank1_LCD_C) = 0x22;
}
/*************************************************
函数名：Lcd光标起点定位函数
功能：指定320240液晶上的一点作为写数据的起始点
入口参数：x 坐标 0~239
          y 坐标 0~319
返回值：无
*************************************************/
void Lcd_SetCursor(uint16_t x, uint16_t y)
{
	//坐标调转
	LCD_WR_REG(0x20, y); //水平坐标
	LCD_WR_REG(0x21, x); //垂直坐标
}

/**********************************************
函数名：Lcd块选函数
功能：选定Lcd上指定的矩形区域

注意：xStart、yStart、Xend、Yend随着屏幕的旋转而改变，位置是矩形框的四个角

入口参数：xStart x方向的起始点
          ySrart y方向的起始点
          Xend   y方向的终止点
          Yend   y方向的终止点
返回值：无
***********************************************/
void BlockWrite(unsigned int Xstart, unsigned int Xend, unsigned int Ystart, unsigned int Yend)
{
	//x与y交换
	LCD_WR_REG(0x0046, (Yend << 8) | Ystart); //水平GRAM终止，起始位置
	LCD_WR_REG(0x0047, Xend);				  //垂直GRAM终止位置
	LCD_WR_REG(0x0048, Xstart);				  //垂直 GRAM起始位置

	Lcd_SetCursor(Xstart, Ystart);

	WriteComm(0x022);
}
uint16_t GetPoint(uint16_t x, uint16_t y)
{
	Lcd_SetCursor(x, y);
	WriteComm(0x022);
	x = *(__IO uint16_t *)(Bank1_LCD_D);
	return *(__IO uint16_t *)(Bank1_LCD_D);
}
/**********************************************
函数名：Lcd块选函数
功能：选定Lcd上指定的矩形区域

注意：xStart和 yStart随着屏幕的旋转而改变，位置是矩形框的四个角

入口参数：xStart x方向的起始点
          ySrart y方向的终止点
          xLong 要选定矩形的x方向长度
          yLong  要选定矩形的y方向长度
返回值：无
***********************************************/
void Lcd_ColorBox(uint16_t xStart, uint16_t yStart, uint16_t xLong, uint16_t yLong, uint16_t Color)
{
	uint32_t temp;

	BlockWrite(xStart, xStart + xLong - 1, yStart, yStart + yLong - 1);
	for (temp = 0; temp < xLong * yLong; temp++)
	{
		*(__IO uint16_t *)(Bank1_LCD_D) = Color;
	}
}

/******************************************
函数名：Lcd图像填充100*100
功能：向Lcd指定位置填充图像
入口参数：Index 要寻址的寄存器地址
          ConfigTemp 写入的数据或命令值
******************************************/
void LCD_Fill_Pic(uint16_t x, uint16_t y, uint16_t pic_H, uint16_t pic_V, const unsigned char *pic)
{
	unsigned long i;
	unsigned int tmp;

	//Set_address_mode
	LCD_WR_REG(0x01, 0x0127);
	LCD_WR_REG(0x03, 0x000A); //横屏，从左下角开始，从左到右，从下到上
	BlockWrite(x, x + pic_H - 1, y, y + pic_V - 1);
	for (i = 0; i < pic_H * pic_V * 2; i += 2)
	{
		tmp = pic[i];
		tmp = tmp << 8;
		tmp += pic[i + 1];
		*(__IO uint16_t *)(Bank1_LCD_D) = tmp;
	}
	//Set_address_mode
	LCD_WR_REG(0x01, 0x1027);
	LCD_WR_REG(0x03, 0x010A); //横屏，从右上角开始，从左到右，从上到下
}

//在指定座标上打一个点
void DrawPixel(uint16_t x, uint16_t y, uint16_t Color)
{
	BlockWrite(x, x, y, y);
	*(__IO uint16_t *)(Bank1_LCD_D) = Color;
}

uint16_t LCD_DecToRGB(uint8_t R, uint8_t G, uint8_t B)
{
	uint16_t color;
	color = R;
	color <<= 5;
	color = color + G;
	color <<= 6;
	color = color + B;
	return color;
}
