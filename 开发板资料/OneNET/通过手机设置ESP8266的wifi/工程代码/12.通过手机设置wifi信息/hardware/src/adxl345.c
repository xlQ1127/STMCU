#include "stm32f10x.h"

#include "adxl345.h"
#include "i2c.h"
#include "delay.h"
#include "led.h"




ADXL345_INFO adxlInfo;






void ADXL345_Init(void)
{
	
    unsigned char devid = 0;
	
	unsigned char val = 0;

    DelayUs(300);

    I2C_ReadByte(I2C2, ADXL345_ADDRESS, 0x00, &devid); //读ID	且每次读写之前都需要读ID
	DelayUs(300);

	val = 0x2B;
    I2C_WriteByte(I2C2, ADXL345_ADDRESS, DATA_FORMAT_REG, &val); //低电平中断输出,13位全分辨率,输出数据右对齐,16g量程
	DelayUs(10);
    
	val = 0x0A;
    I2C_WriteByte(I2C2, ADXL345_ADDRESS, BW_RATE, &val); //数据输出速度为100Hz
	DelayUs(10);
    
	val = 0x28;
    I2C_WriteByte(I2C2, ADXL345_ADDRESS, POWER_CTL, &val);    //链接使能,测量模式
	DelayUs(10);
    
	val = 0;
    I2C_WriteByte(I2C2, ADXL345_ADDRESS, INT_ENABLE, &val);     //不使用中断
	DelayUs(10);
    
    I2C_WriteByte(I2C2, ADXL345_ADDRESS, OFSX, &val);
	DelayUs(10);
    
    I2C_WriteByte(I2C2, ADXL345_ADDRESS, OFSY, &val);
	DelayUs(10);
    
    I2C_WriteByte(I2C2, ADXL345_ADDRESS, OFSZ, &val);
	
    DelayUs(500);

}

void ADXL345_GetValue(void)
{

	unsigned char devid = 0;
	
	unsigned char dataTemp[6];

    DelayUs(200);
    I2C_ReadByte(I2C2, ADXL345_ADDRESS, 0x00, &devid); //读ID	且每次读写之前都需要读ID
	DelayUs(200);
	
	I2C_ReadBytes(I2C2, ADXL345_ADDRESS, 0x32, dataTemp, 6);
	
	adxlInfo.incidence_X = (short)(dataTemp[0] + ((unsigned short)dataTemp[1] << 8));
    adxlInfo.incidence_Y = (short)(dataTemp[2] + ((unsigned short)dataTemp[3] << 8));
    adxlInfo.incidence_Z = (short)(dataTemp[4] + ((unsigned short)dataTemp[5] << 8));
	
	adxlInfo.incidence_Xf = (float)adxlInfo.incidence_X * 0.004;
	adxlInfo.incidence_Yf = (float)adxlInfo.incidence_Y * 0.004;
	adxlInfo.incidence_Zf = (float)adxlInfo.incidence_Z * 0.004;

}
