#include "stm32f10x.h"
#include "IIC.h"
#include "var_global.h"
#include "delay.h"
#include "math.h"
#include "MPU6050.h"

int16_XYZ MAG_XYZ;
float MAG_angle;

float MAG_Offset;

u8 MAG_Ofs_OK;


void read_hmc5883l()
{
	    u8 BUF[6];
	
      I2C_WriteByte(0x3c,0x00,0x78);
      I2C_WriteByte(0x3c,0x01,0x60);	//00 20 60 三个等级
      I2C_WriteByte(0x3c,0x02,0x00);
      delay_ms(10);	

	    MPU_GetData(0x3d,0x03,6,BUF);
	

      MAG_XYZ.X=(  ((int16_t)BUF[0]) << 8) | BUF[1]; 
      MAG_XYZ.Z=(  ((int16_t)BUF[2]) << 8) | BUF[3]; 
	    MAG_XYZ.Y=(  ((int16_t)BUF[4]) << 8) | BUF[5]; 
       


	     
       MAG_XYZ.X=MAG_XYZ.X-147;
       MAG_XYZ.Y=MAG_XYZ.Y+85;	//椭圆磁场偏移量，最大+最小除以2 
	   
   		 MAG_angle= atan2(MAG_XYZ.Y,MAG_XYZ.X) * (180 / 3.14159265);
      
    if(MAG_Ofs_OK)MAG_angle-=MAG_Offset;
}






