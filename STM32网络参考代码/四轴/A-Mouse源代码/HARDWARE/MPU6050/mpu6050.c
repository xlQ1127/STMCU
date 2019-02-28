#include "mpu6050.h"
#include "sys.h"
#include "delay.h"
#include "usart.h"   
//////////////////////////////////////////////////////////////////////////////////	 
//±¾³ÌĞòÖ»¹©Ñ§Ï°Ê¹ÓÃ£¬Î´¾­×÷ÕßĞí¿É£¬²»µÃÓÃÓÚÆäËüÈÎºÎÓÃÍ¾
//ALIENTEK MiniSTM32F103¿ª·¢°å 
//MPU6050 Çı¶¯´úÂë	   
//ÕıµãÔ­×Ó@ALIENTEK
//¼¼ÊõÂÛÌ³:www.openedv.com
//´´½¨ÈÕÆÚ:2015/4/18
//°æ±¾£ºV1.0
//°æÈ¨ËùÓĞ£¬µÁ°æ±Ø¾¿¡£
//Copyright(C) ¹ãÖİÊĞĞÇÒíµç×Ó¿Æ¼¼ÓĞÏŞ¹«Ë¾ 2009-2019
//All rights reserved									  
////////////////////////////////////////////////////////////////////////////////// 
  
 void usart_send_char(u8 c)
{
	while((USART1->SR&0X40)==0);//?????????   
	USART1->DR=c;   	
} 


//³õÊ¼»¯MPU6050
//·µ»ØÖµ:0,³É¹¦
//    ÆäËû,´íÎó´úÂë
u8 MPU_Init(void)
{ 
	u8 res; 
	MPU_IIC_Init();//³õÊ¼»¯IIC×ÜÏß
	delay_ms(200);
	MPU_Write_Byte(MPU_PWR_MGMT1_REG,0X80);	//¸´Î»MPU6050
	
	//usart_send_char(0xFF);
	
    delay_ms(100);
	MPU_Write_Byte(MPU_PWR_MGMT1_REG,0X00);	//»½ĞÑMPU6050 
	MPU_Set_Gyro_Fsr(3);					//ÍÓÂİÒÇ´«¸ĞÆ÷,¡À2000dps
	MPU_Set_Accel_Fsr(0);					//¼ÓËÙ¶È´«¸ĞÆ÷,¡À2g
	
	
	
	MPU_Set_LPF(25);             //Êı×ÖµÍÍ¨ÂË²¨Æ÷ÉèÖÃÎª6Hz
	MPU_Set_Rate(50);						//ÉèÖÃ²ÉÑùÂÊ50Hz
	MPU_Write_Byte(MPU_INT_EN_REG,0X00);	//¹Ø±ÕËùÓĞÖĞ¶Ï
	MPU_Write_Byte(MPU_USER_CTRL_REG,0X00);	//I2CÖ÷Ä£Ê½¹Ø±Õ
	MPU_Write_Byte(MPU_FIFO_EN_REG,0X00);	//¹Ø±ÕFIFO
	MPU_Write_Byte(MPU_INTBP_CFG_REG,0X80);	//INTÒı½ÅµÍµçÆ½ÓĞĞ§
	
	//usart_send_char(res);
	//usart_send_char(0xDD);
	res=MPU_Read_Byte(MPU_DEVICE_ID_REG);
	//usart_send_char(0xEE);
	//usart_send_char(res);	
	
	if(res==MPU_ADDR)//Æ÷¼şIDÕıÈ·
	{
		//usart_send_char(0xCC);
		MPU_Write_Byte(MPU_PWR_MGMT1_REG,0X01);	//ÉèÖÃCLKSEL,PLL XÖáÎª²Î¿¼
		MPU_Write_Byte(MPU_PWR_MGMT2_REG,0X00);	//¼ÓËÙ¶ÈÓëÍÓÂİÒÇ¶¼¹¤×÷
		MPU_Set_Rate(50);						//ÉèÖÃ²ÉÑùÂÊÎª50Hz
		//usart_send_char(0xBB);
 	}else return 1;
	return 0;
}
//ÉèÖÃMPU6050ÍÓÂİÒÇ´«¸ĞÆ÷ÂúÁ¿³Ì·¶Î§
//fsr:0,¡À250dps;1,¡À500dps;2,¡À1000dps;3,¡À2000dps
//·µ»ØÖµ:0,ÉèÖÃ³É¹¦
//    ÆäËû,ÉèÖÃÊ§°Ü 
u8 MPU_Set_Gyro_Fsr(u8 fsr)
{
	return MPU_Write_Byte(MPU_GYRO_CFG_REG,fsr<<3);//ÉèÖÃÍÓÂİÒÇÂúÁ¿³Ì·¶Î§  
}
//ÉèÖÃMPU6050¼ÓËÙ¶È´«¸ĞÆ÷ÂúÁ¿³Ì·¶Î§
//fsr:0,¡À2g;1,¡À4g;2,¡À8g;3,¡À16g
//·µ»ØÖµ:0,ÉèÖÃ³É¹¦
//    ÆäËû,ÉèÖÃÊ§°Ü 
u8 MPU_Set_Accel_Fsr(u8 fsr)
{
	return MPU_Write_Byte(MPU_ACCEL_CFG_REG,fsr<<3);//ÉèÖÃ¼ÓËÙ¶È´«¸ĞÆ÷ÂúÁ¿³Ì·¶Î§  
}
//ÉèÖÃMPU6050µÄÊı×ÖµÍÍ¨ÂË²¨Æ÷
//lpf:Êı×ÖµÍÍ¨ÂË²¨ÆµÂÊ(Hz)
//·µ»ØÖµ:0,ÉèÖÃ³É¹¦
//    ÆäËû,ÉèÖÃÊ§°Ü 
u8 MPU_Set_LPF(u16 lpf)
{
	u8 data=0;
	if(lpf>=188)data=1;
	else if(lpf>=98)data=2;
	else if(lpf>=42)data=3;
	else if(lpf>=20)data=4;
	else if(lpf>=10)data=5;
	else data=6; 
	return MPU_Write_Byte(MPU_CFG_REG,data);//ÉèÖÃÊı×ÖµÍÍ¨ÂË²¨Æ÷  
}
//ÉèÖÃMPU6050µÄ²ÉÑùÂÊ(¼Ù¶¨Fs=1KHz)
//rate:4~1000(Hz)
//·µ»ØÖµ:0,ÉèÖÃ³É¹¦
//    ÆäËû,ÉèÖÃÊ§°Ü 
u8 MPU_Set_Rate(u16 rate)
{
	u8 data;
	if(rate>1000)rate=1000;
	if(rate<4)rate=4;
	data=1000/rate-1;
	data=MPU_Write_Byte(MPU_SAMPLE_RATE_REG,data);	//ÉèÖÃÊı×ÖµÍÍ¨ÂË²¨Æ÷
 	return MPU_Set_LPF(rate/2);	//×Ô¶¯ÉèÖÃLPFÎª²ÉÑùÂÊµÄÒ»°ë
}

//µÃµ½ÎÂ¶ÈÖµ
//·µ»ØÖµ:ÎÂ¶ÈÖµ(À©´óÁË100±¶)
short MPU_Get_Temperature(void)
{
    u8 buf[2]; 
    short raw;
	float temp;
	MPU_Read_Len(MPU_ADDR,MPU_TEMP_OUTH_REG,2,buf); 
    raw=((u16)buf[0]<<8)|buf[1];  
    temp=36.53+((double)raw)/340;  
    return temp*100;;
}
//µÃµ½ÍÓÂİÒÇÖµ(Ô­Ê¼Öµ)
//gx,gy,gz:ÍÓÂİÒÇx,y,zÖáµÄÔ­Ê¼¶ÁÊı(´ø·ûºÅ)
//·µ»ØÖµ:0,³É¹¦
//    ÆäËû,´íÎó´úÂë
u8 MPU_Get_Gyroscope(short *gx,short *gy,short *gz)
{
    u8 buf[6],res;  
	res=MPU_Read_Len(MPU_ADDR,MPU_GYRO_XOUTH_REG,6,buf);
	if(res==0)
	{
		*gx=((u16)buf[0]<<8)|buf[1];  
		*gy=((u16)buf[2]<<8)|buf[3];  
		*gz=((u16)buf[4]<<8)|buf[5];
	} 	
    return res;;
}
//µÃµ½¼ÓËÙ¶ÈÖµ(Ô­Ê¼Öµ)
//gx,gy,gz:ÍÓÂİÒÇx,y,zÖáµÄÔ­Ê¼¶ÁÊı(´ø·ûºÅ)
//·µ»ØÖµ:0,³É¹¦
//    ÆäËû,´íÎó´úÂë
u8 MPU_Get_Accelerometer(short *ax,short *ay,short *az)
{
    u8 buf[6],res;  
	res=MPU_Read_Len(MPU_ADDR,MPU_ACCEL_XOUTH_REG,6,buf);
	if(res==0)
	{
		*ax=((u16)buf[0]<<8)|buf[1];  
		*ay=((u16)buf[2]<<8)|buf[3];  
		*az=((u16)buf[4]<<8)|buf[5];
	} 	
    return res;
}
//IICÁ¬ĞøĞ´
//addr:Æ÷¼şµØÖ· 
//reg:¼Ä´æÆ÷µØÖ·
//len:Ğ´Èë³¤¶È
//buf:Êı¾İÇø
//·µ»ØÖµ:0,Õı³£
//    ÆäËû,´íÎó´úÂë
u8 MPU_Write_Len(u8 addr,u8 reg,u8 len,u8 *buf)
{
	u8 i; 
    MPU_IIC_Start(); 
	MPU_IIC_Send_Byte((addr<<1)|0);//·¢ËÍÆ÷¼şµØÖ·+Ğ´ÃüÁî	
	if(MPU_IIC_Wait_Ack())	//µÈ´ıÓ¦´ğ
	{
		MPU_IIC_Stop();		 
		return 1;		
	}
    MPU_IIC_Send_Byte(reg);	//Ğ´¼Ä´æÆ÷µØÖ·
    MPU_IIC_Wait_Ack();		//µÈ´ıÓ¦´ğ
	for(i=0;i<len;i++)
	{
		MPU_IIC_Send_Byte(buf[i]);	//·¢ËÍÊı¾İ
		if(MPU_IIC_Wait_Ack())		//µÈ´ıACK
		{
			MPU_IIC_Stop();	 
			return 1;		 
		}		
	}    
    MPU_IIC_Stop();	 
	return 0;	
} 
//IICÁ¬Ğø¶Á
//addr:Æ÷¼şµØÖ·
//reg:Òª¶ÁÈ¡µÄ¼Ä´æÆ÷µØÖ·
//len:Òª¶ÁÈ¡µÄ³¤¶È
//buf:¶ÁÈ¡µ½µÄÊı¾İ´æ´¢Çø
//·µ»ØÖµ:0,Õı³£
//    ÆäËû,´íÎó´úÂë
u8 MPU_Read_Len(u8 addr,u8 reg,u8 len,u8 *buf)
{ 
 	MPU_IIC_Start(); 
	MPU_IIC_Send_Byte((addr<<1)|0);//·¢ËÍÆ÷¼şµØÖ·+Ğ´ÃüÁî	
	if(MPU_IIC_Wait_Ack())	//µÈ´ıÓ¦´ğ
	{
		MPU_IIC_Stop();		 
		return 1;		
	}
    MPU_IIC_Send_Byte(reg);	//Ğ´¼Ä´æÆ÷µØÖ·
    MPU_IIC_Wait_Ack();		//µÈ´ıÓ¦´ğ
    MPU_IIC_Start();
	MPU_IIC_Send_Byte((addr<<1)|1);//·¢ËÍÆ÷¼şµØÖ·+¶ÁÃüÁî	
    MPU_IIC_Wait_Ack();		//µÈ´ıÓ¦´ğ 
	while(len)
	{
		if(len==1)*buf=MPU_IIC_Read_Byte(0);//¶ÁÊı¾İ,·¢ËÍnACK 
		else *buf=MPU_IIC_Read_Byte(1);		//¶ÁÊı¾İ,·¢ËÍACK  
		len--;
		buf++; 
	}    
    MPU_IIC_Stop();	//²úÉúÒ»¸öÍ£Ö¹Ìõ¼ş 
	return 0;	
}
//IICĞ´Ò»¸ö×Ö½Ú 
//reg:¼Ä´æÆ÷µØÖ·
//data:Êı¾İ
//·µ»ØÖµ:0,Õı³£
//    ÆäËû,´íÎó´úÂë
u8 MPU_Write_Byte(u8 reg,u8 data) 				 
{ 
    MPU_IIC_Start(); 
	MPU_IIC_Send_Byte((MPU_ADDR<<1)|0);//·¢ËÍÆ÷¼şµØÖ·+Ğ´ÃüÁî	
	if(MPU_IIC_Wait_Ack())	//µÈ´ıÓ¦´ğ
	{
		MPU_IIC_Stop();		 
		return 1;		
	}
    MPU_IIC_Send_Byte(reg);	//Ğ´¼Ä´æÆ÷µØÖ·
    MPU_IIC_Wait_Ack();		//µÈ´ıÓ¦´ğ 
	MPU_IIC_Send_Byte(data);//·¢ËÍÊı¾İ
	if(MPU_IIC_Wait_Ack())	//µÈ´ıACK
	{
		MPU_IIC_Stop();	 
		return 1;		 
	}		 
    MPU_IIC_Stop();	 
	return 0;
}
//IIC¶ÁÒ»¸ö×Ö½Ú 
//reg:¼Ä´æÆ÷µØÖ· 
//·µ»ØÖµ:¶Áµ½µÄÊı¾İ
u8 MPU_Read_Byte(u8 reg)
{
	u8 res;
    MPU_IIC_Start(); 
	MPU_IIC_Send_Byte((MPU_ADDR<<1)|0);//·¢ËÍÆ÷¼şµØÖ·+Ğ´ÃüÁ
		
	MPU_IIC_Wait_Ack();		//µÈ´ıÓ¦´ğ 
	
    MPU_IIC_Send_Byte(reg);	//Ğ´¼Ä´æÆ÷µØÖ·
	
    MPU_IIC_Wait_Ack();		//µÈ´ıÓ¦´ğ
	
    MPU_IIC_Start();
	MPU_IIC_Send_Byte((MPU_ADDR<<1)|1);//·¢ËÍÆ÷¼şµØÖ·+¶ÁÃüÁî	
	
    MPU_IIC_Wait_Ack();		//µÈ´ıÓ¦´ğ 
	
	res=MPU_IIC_Read_Byte(0);//¶ÁÈ¡Êı¾İ,·¢ËÍnACK 
    MPU_IIC_Stop();			//²úÉúÒ»¸öÍ£Ö¹Ìõ¼ş 
	return res;		
}


