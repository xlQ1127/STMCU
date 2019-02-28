#include "stm32f10x.h"

#include "sht20.h"
#include "i2c.h"
#include "delay.h"
#include "global.h"




const int16_t POLYNOMIAL = 0x131;

SHT20_INFO sht20Info;

void SHT20_reset(void)
{
	
    //printf("%s %d\n",__func__,__LINE__);
    I2C_WriteByte(I2C2, SHT20_ADDRESS, SHT20_SOFT_RESET, (void *)0);
	
}

unsigned char  SHT20_read_user_reg(void)
{
	
    unsigned char val = 0;
	
    I2C_ReadByte(I2C2, SHT20_ADDRESS, SHT20_READ_REG, &val);
	
    return val;
	
}

char SHT2x_CheckCrc(char data[], char nbrOfBytes, char checksum)
{
	
    char crc = 0;
    char bit = 0;
    char byteCtr = 0;
	
    //calculates 8-Bit checksum with given polynomial
    for(byteCtr = 0; byteCtr < nbrOfBytes; ++byteCtr)
    {
        crc ^= (data[byteCtr]);
        for ( bit = 8; bit > 0; --bit)
        {
            if (crc & 0x80) crc = (crc << 1) ^ POLYNOMIAL;
            else crc = (crc << 1);
        }
    }
	
    if(crc != checksum)
		return 1;
    else
		return 0;
	
}

float SHT2x_CalcTemperatureC(short u16sT)
{
	
    float temperatureC = 0;            // variable for result

    u16sT &= ~0x0003;           // clear bits [1..0] (status bits)
    //-- calculate temperature [癈] --
    temperatureC = -46.85 + 175.72 / 65536 * (float)u16sT; //T= -46.85 + 175.72 * ST/2^16
	
    return temperatureC;
	
}

float SHT2x_CalcRH(unsigned short u16sRH)
{
	
    float humidityRH = 0;              // variable for result
	
    u16sRH &= ~0x0003;          // clear bits [1..0] (status bits)
    //-- calculate relative humidity [%RH] --
    //humidityRH = -6.0 + 125.0/65536 * (float)u16sRH; // RH= -6 + 125 * SRH/2^16
    humidityRH = ((float)u16sRH * 0.00190735) - 6;
	
    return humidityRH;
	
}

float SHT2x_MeasureHM(unsigned char cmd, unsigned short *pMeasurand)
{
	
    char  checksum = 0, addr = 0;  //checksum
    char  data[2];    //data array for checksum verification
    unsigned short tmp = 0;
    float t = 0;
	
    //start
    addr = SHT20_ADDRESS << 1;
    I2C_AcknowledgeConfig(I2C2, ENABLE);

    I2C_GenerateSTART(I2C2, ENABLE);      /**********start**********/
    while(!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_MODE_SELECT));
    I2C_Send7bitAddress(I2C2, addr, I2C_Direction_Transmitter); /*******device addr********/
    while (!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED));
    //send cmd
    I2C_SendData(I2C2, cmd);    /*****reg addr**********/
    while (!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_BYTE_TRANSMITTED));

    //start again
    DelayUs(15);
    I2C_GenerateSTART(I2C2, ENABLE);      /**********start**********/
    while(!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_MODE_SELECT));
    //send read
    DelayUs(15);
    I2C_Send7bitAddress(I2C2, addr, I2C_Direction_Receiver); /*******device addr********/
    while (!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED));

    DelayUs(30);//wait enough，下次改成SCL超时查询的方式
    while (!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_BYTE_RECEIVED));
    data[0] = I2C_ReceiveData(I2C2);
    while (!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_BYTE_RECEIVED));
    data[1] = I2C_ReceiveData(I2C2);

    while (!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_BYTE_RECEIVED));
    checksum = I2C_ReceiveData(I2C2);

    //I2C_AcknowledgeConfig(I2C2, DISABLE);//如果调用了，那么crc无效
    I2C_GenerateSTOP(I2C2, ENABLE);   /********stop******/
    SHT2x_CheckCrc(data, 2, checksum);
    tmp = (data[0] << 8) + data[1];
    if(cmd == SHT20_Measurement_T_HM)
    {
        t = SHT2x_CalcTemperatureC(tmp);
        //printf("%s data[0]=%d,data[1]=%d,checksum=%d,t=%f\n", __func__, data[0], data[1], SHT2x_CheckCrc(data, 2, checksum), t);
    }
    else
    {
        t = SHT2x_CalcRH(tmp);
        //printf("%s data[0]=%d,data[1]=%d,checksum=%d,rh=%f%\n", __func__, data[0], data[1], SHT2x_CheckCrc(data, 2, checksum), t);
    }
	
    if(pMeasurand)
    {
        *pMeasurand = (unsigned short)t;
    }
    //printf("%s %d\n", __func__, *pMeasurand);
	
    return t;
	
}

void SHT20_GetValue(void)
{
	
	unsigned char val = 0;

	//Event_Cmd_Off();
	
	SHT20_read_user_reg();
	DelayUs(100);
	
	sht20Info.tempreture = SHT2x_MeasureHM(SHT20_Measurement_T_HM, (void *)0);
	DelayUs(100);
	
	sht20Info.humidity = SHT2x_MeasureHM(SHT20_Measurement_RH_HM, (void *)0);
	DelayUs(100);
	
	SHT20_read_user_reg();
	DelayUs(100);
	
	I2C_WriteByte(I2C2, SHT20_ADDRESS, SHT20_WRITE_REG, &val);
	DelayUs(100);
	
	SHT20_read_user_reg();
	DelayUs(100);
	
	SHT20_reset();
	DelayUs(100);
	
	//Event_Cmd_On();

}
