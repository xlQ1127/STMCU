#ifndef _SHT20_H_
#define _SHT20_H_

#include "i2c.h"

extern I2C_HandleTypeDef hi2c2;
#define hi2cx  hi2c2


/*SHT20 设备操作相关宏定义，详见手册*/
#define SHT20_ADDRESS            0X40
#define SHT20_Measurement_RH_HM  0XE5
#define SHT20_Measurement_T_HM   0XE3
#define SHT20_Measurement_RH_NHM 0XF5
#define SHT20_Measurement_T_NHM  0XF3
#define SHT20_READ_REG           0XE7
#define SHT20_WRITE_REG          0XE6
#define SHT20_SOFT_RESET         0XFE


typedef struct
{

	float tempreture;
	float humidity;

} SHT20_INFO;

extern SHT20_INFO sht20_info;

void I2C_Write_Data(I2C_HandleTypeDef *hi2c,uint8_t data);
void I2C_Read_Data(I2C_HandleTypeDef *hi2c,uint8_t *Buf,uint8_t len);
char SHT2x_CheckCrc(char data[], char nbrOfBytes, char checksum);
float SHT2x_CalcTemperatureC(unsigned short u16sT);
float SHT2x_CalcRH(unsigned short u16sRH);
float Get_SHT20_TEMP(void);
float Get_SHT20_HUM(void);


#endif
