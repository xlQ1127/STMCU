#include "bmp180.h"
#include "IIC.h"
#include "SysTick.h"
#include "math.h"

 
struct  BMP180_INFO g_bmp180_data;


//读取未校正的温度值
static int bmp180_read_temperature(void)
{
	Single_WriteI2C(0xEE,0xF4,0x2E);
	Delay_us(5000);
	return Double_ReadI2C(0xEE,0xf6);
}
//读取未校正的气压值
static int bmp180_read_pressure(void)
{  
	int pressure = 0;
	Single_WriteI2C(0xEE,0xF4,(0x34+(BMP180_OSS<<6)));
	Delay_us(25000);
	pressure = Double_ReadI2C(0xEE,0xf6) ;
	pressure = ((pressure <<8) + Single_ReadI2C(0xEE,0xf8)) >>(8-BMP180_OSS) ;
	return pressure;
}
//读取校正参数
void bmp180_read_calibrate_param(void)
{
    g_bmp180_data.cal_param.ac1 = Double_ReadI2C(0xEE,0xAA);Delay_us(1000);
    g_bmp180_data.cal_param.ac2 = Double_ReadI2C(0xEE,0xAC);Delay_us(1000);
    g_bmp180_data.cal_param.ac3 = Double_ReadI2C(0xEE,0xAE);Delay_us(1000);
    g_bmp180_data.cal_param.ac4 = Double_ReadI2C(0xEE,0xB0);Delay_us(1000);
    g_bmp180_data.cal_param.ac5 = Double_ReadI2C(0xEE,0xB2);Delay_us(1000);
    g_bmp180_data.cal_param.ac6 = Double_ReadI2C(0xEE,0xB4);Delay_us(1000);
    g_bmp180_data.cal_param.b1  = Double_ReadI2C(0xEE,0xB6);Delay_us(1000);
    g_bmp180_data.cal_param.b2  = Double_ReadI2C(0xEE,0xB8);Delay_us(1000);
    g_bmp180_data.cal_param.mb  = Double_ReadI2C(0xEE,0xBA);Delay_us(1000);
    g_bmp180_data.cal_param.mc  = Double_ReadI2C(0xEE,0xBC);Delay_us(1000);
    g_bmp180_data.cal_param.md  = Double_ReadI2C(0xEE,0xBE);Delay_us(1000);
}
//初始化
void bmp180_init(void)
{
	g_bmp180_data.exist_flag = Single_ReadI2C(0xEE,0xD0);
  bmp180_read_calibrate_param();
}
//校正
void bmp180_convert(void)
{     
	int x1, x2, x3, b3, b5, b6, p, b7;
	unsigned int b4 ;  
	g_bmp180_data.unset_temperature = bmp180_read_temperature();  
	g_bmp180_data.unset_gas_press = bmp180_read_pressure();  

	x1 = ((g_bmp180_data.unset_temperature)-g_bmp180_data.cal_param.ac6)*(g_bmp180_data.cal_param.ac5)>>15;  
	x2 = ((g_bmp180_data.cal_param.mc)<<11)/(x1+g_bmp180_data.cal_param.md);  
	b5 = x1+x2;  
	g_bmp180_data.temperature = (b5+8)>>4;  
  
	b6 = b5-4000;  
	x1 = ((int)(g_bmp180_data.cal_param.b2)*(b6*b6>>12))>>11;  
	x2 = ((int)(g_bmp180_data.cal_param.ac2)*b6)>>11;  
	x3 = x1+x2;  
	b3 = ((((int)(g_bmp180_data.cal_param.ac1)*4+x3)<<BMP180_OSS)+2)/4;  
	x1 = ((int)g_bmp180_data.cal_param.ac3)*b6 >>13;  
	x2 = ((int)(g_bmp180_data.cal_param.b1)*(b6*b6>>12))>>16;  
	x3 = ((x1+x2)+2)>>2;  
	b4 = ((int)(g_bmp180_data.cal_param.ac4)*(unsigned int)(x3+32768))>>15;  
	b7 = ((unsigned int)(g_bmp180_data.unset_gas_press)-b3)*(50000>>BMP180_OSS);  
	if (b7<0x80000000) {p = (b7*2)/b4;} else {p = (b7/b4)*2;}  
	x1 = (p>>8)*(p>>8);
	x1 = ((signed int)x1*3038)>>16;
	x2 = (-7357*p)>>16;
	g_bmp180_data.gas_press = p+((x1+x2+3791)>>4);

	g_bmp180_data.altitude = (44330.0*(1.0-pow((float)(g_bmp180_data.gas_press)/101325.0, 1.0/5.255)));
}  
//************************************************************************************
