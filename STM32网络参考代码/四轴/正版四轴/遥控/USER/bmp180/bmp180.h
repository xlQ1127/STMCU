#ifndef _BMP180_H_
#define _BMP180_H_

//BMP180校正
struct BMP180_CAL_PARAM
{
	short int ac1;
	short int ac2;
	short int ac3;
	unsigned int ac4;
	unsigned int ac5;
	unsigned int ac6;
	short int b1;
	short int b2;
	short int mb;
	short int mc;
	short int md;
};

struct BMP180_INFO
{
	unsigned char exist_flag;          //存在标志
	unsigned char version;             //版本
	struct BMP180_CAL_PARAM cal_param; //修正系数
	int unset_temperature;     //未校正的温度
	int unset_gas_press;       //未校正的气压
	int temperature;           //校正后的温度
	int gas_press;             //校正后的气压
	float altitude;                    //海拔
};
extern struct BMP180_INFO g_bmp180_data;

#define BMP180_OSS 3  //范围(0~3)

extern void bmp180_init(void);
extern void bmp180_convert(void);

#endif
