#include "stm32f10x.h"

#include "gy30.h"
#include "i2c.h"
#include "delay.h"

#include "led.h"







GY30_INFO gy30Info;

void GY30_Init(void)
{

	DelayUs(5);
	
	I2C_WriteByte(I2C2, BH1750FVI_ADDR, BH1750_ON, (void *)0); //power on
    DelayUs(40);
	
	I2C_WriteByte(I2C2, BH1750FVI_ADDR, BH1750_RSET, (void *)0); //clear
    DelayUs(40);
	
	I2C_WriteByte(I2C2, BH1750FVI_ADDR, BH1750_Con_High_RM, (void *)0); //连续H分辨率模式，至少120ms，之后自动断电模式
    DelayUs(40);

}

void GY30_GetValue(void)
{
	
    unsigned char addr, num = 2;
    unsigned char data[2], *buf = data;
    unsigned short result = 0;
    float result_lx = 0;
	
    I2C_AcknowledgeConfig(I2C2, ENABLE);
    /* Send START condition */
    I2C_GenerateSTART(I2C2, ENABLE);/***start *******/
    addr = BH1750FVI_ADDR << 1;
    while(!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_MODE_SELECT));/* Test on EV5 and clear it */
	
    I2C_Send7bitAddress(I2C2, addr, I2C_Direction_Receiver); /***device addr**/
    while(!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED)); /* Test on EV6 and clear it */

    while(num)
    {
        if(num == 1)
        {
            /*the last one*/
            I2C_AcknowledgeConfig(I2C2, DISABLE);/* Disable Acknowledgement */
            I2C_GenerateSTOP(I2C2, ENABLE);/* Send STOP Condition */
        }

        /* Test on EV7 and clear it */
        if(I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_BYTE_RECEIVED))
        {
            /* Read a byte from the EEPROM */
            *buf = I2C_ReceiveData(I2C2);
            buf++; /* Point to the next location where the byte read will be saved */
            num--; /* Decrement the read bytes counter */
        }
    }
    /* Enable Acknowledgement to be ready for another reception */
    I2C_AcknowledgeConfig(I2C2, ENABLE);
	
    result = (unsigned short)((data[0] << 8) + data[1]); //合成数据，即光照数据
    result_lx = (float)result / 1.2;
	
    gy30Info.lightVal = (unsigned short)result_lx;
	
}
