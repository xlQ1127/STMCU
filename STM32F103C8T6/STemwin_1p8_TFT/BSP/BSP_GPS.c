#include "BSP_GPS.h"

#include "string.h"



extern UART_HandleTypeDef huart2;
extern uint8_t UART_GPS_RX[512];
uint8_t UTC_TO_BJT[24]={
8,9,10,11,12,13,14,15,16,17,18,19,20,21,22,23,24
};
	


Guassian_Projection_Typedef Projection_Cordinate;
float32_t X,Y,Latitude,Longitude,
           Longitude_Of_Meridian;//当前所处6°带的子午线经度
	

float32_t iPI = PI/180.0f; //3.1415926535898/180.0

uint8_t ZoneNumber=0;// 6度带标号

float32_t WSG84_a= 6378137.0,        //长轴长
          WSG84_f =1.0/298.257223563, 
	        WSG84_e1=0.0066943799013,  //第一曲率
	        WSG84_e2=0.00673949674227;//第二曲率 

float32_t yita_yita,t,m,N,alpha,l,COS_Latitude;
float32_t m0,m2,m4,m6,m8,
	  a0,a2,a4,a6,a8;


GPS_GPRMC_INFO Get_GPS_GPRMC(void)
{
	
 
  char * p;
		
 p=strstr( (const char*)UART_GPS_RX,"RMC");  
      if(p)
			{
						p = strchr(p, ',');
						
						p++;
						GPRMC.GPS_UTC_Hours =   StrToIntFix(p, 2);
				    GPRMC.GPS_UTC_Hours=UTC_TO_BJT[GPRMC.GPS_UTC_Hours];
						p += 2;
						GPRMC.GPS_UTC_Minutes = StrToIntFix(p, 2);
						p += 2;
						GPRMC.GPS_UTC_Seconds = StrToIntFix(p, 2);
						p += 3;

							/* 字段2 状态，A=定位，V=未定位 */
						p = strchr(p, ',');
						p++;
						
						
						
						if (*p != 'A')
						{
						  GPRMC.GPS_Statu = 1;
						}
						
            else
						{
						
						
						
						  GPRMC.GPS_Statu = 2;
							
											/* 字段3 纬度ddmm.mmmmm，度分格式（前导位数不足则补0） */
									p = strchr(p, ',');

									p++;
									
							    GPRMC.GPS_Latitude_DD=StrToIntFix(p, 2);
							    p += 2;
							
									GPRMC.GPS_Latitude_MM = StrToIntFix(p, 2)*100000;
									p += 3;
									
									GPRMC.GPS_Latitude_MM += StrToIntFix(p, 5);
									p += 5;
							

									/* 字段4 纬度N（北纬）或S（南纬）*/
									p = strchr(p, ',');
									if (p == 0)
									{
										GPRMC.GPS_NS = 2;
									}
									p++;
									
									if (*p == 'S')
									{
										GPRMC.GPS_NS = 2;
									}
									
									else if (*p == 'N')
									{
										GPRMC.GPS_NS = 1;
									}


									/* 字段5 经度dddmm.mmmmm，度分格式（前导位数不足则补0） */
									p = strchr(p, ',');
									

									p++;

							    GPRMC.GPS_Longitude_DD=StrToIntFix(p, 3);
							    p += 3;
							
									GPRMC.GPS_Longitude_MM = StrToIntFix(p, 2)*100000;
									p += 3;
									
									GPRMC.GPS_Longitude_MM += StrToIntFix(p, 5);
									p += 5;

									/* 字段6：经度E（东经）或W（西经） */
									p = strchr(p, ',');
									if (p == 0)
									{
										GPRMC.GPS_WE=2;
									}
									p++;
									
									if (*p == 'E')
									{
										GPRMC.GPS_WE = 2;
									}
									
									else if (*p == 'W')
									{
										GPRMC.GPS_WE = 1;
									}
							
						}

						
						
  
			
			}
	
	return GPRMC;
 
}




int32_t StrToInt(char *_pStr)
{
	uint8_t flag;
	char *p;
	uint32_t ulInt;
	uint8_t i;
	uint8_t ucTemp;

	p = _pStr;
	if (*p == '-')
	{
		flag = 1;	/* 负数 */
		p++;
	}
	else
	{
		flag = 0;
	}

	ulInt = 0;
	for (i = 0; i < 15; i++)
	{
		ucTemp = *p;
		if (ucTemp == '.')	/* 遇到小数点，自动跳过1个字节 */
		{
			p++;
			ucTemp = *p;
		}
		if ((ucTemp >= '0') && (ucTemp <= '9'))
		{
			ulInt = ulInt * 10 + (ucTemp - '0');
			p++;
		}
		else
		{
			break;
		}
	}

	if (flag == 1)
	{
		return -ulInt;
	}
	return ulInt;
}










int32_t StrToIntFix(char *_pStr, uint8_t _ucLen)
{
	uint8_t flag;
	char *p;
	uint32_t ulInt;
	uint8_t i;
	uint8_t ucTemp;

	p = _pStr;
	if (*p == '-')
	{
		flag = 1;	/* 负数 */
		p++;
		_ucLen--;
	}
	else
	{
		flag = 0;
	}

	ulInt = 0;
	for (i = 0; i < _ucLen; i++)
	{
		ucTemp = *p;
		if (ucTemp == '.')	/* 遇到小数点，自动跳过1个字节 */
		{
			p++;
			ucTemp = *p;
		}
		if ((ucTemp >= '0') && (ucTemp <= '9'))
		{
			ulInt = ulInt * 10 + (ucTemp - '0');
			p++;
		}
		else
		{
			break;
		}
	}

	if (flag == 1)
	{
		return -ulInt;
	}
	return ulInt;
}

Guassian_Projection_Typedef Guassian_Projection(uint32_t Latitude_DDD,uint32_t Latitude_MMMMMM,uint32_t Longitude_DDD,uint32_t Longitude_MMMMMM)
{
Guassian_Projection_Typedef Guassian_Projection_Cordinate;
     Latitude=(float32_t)  ( (Latitude_MMMMMM/6) + Latitude_DDD*1000000 )/1000000*iPI;
     Longitude=(float32_t) ( (Longitude_MMMMMM/6) + Longitude_DDD*1000000 )/1000000;

     
     ZoneNumber=(uint8_t)Longitude/3;
 
     Longitude_Of_Meridian=(ZoneNumber*3)*iPI;    
     Longitude=Longitude*iPI;
     
     l=Longitude-Longitude_Of_Meridian;
     COS_Latitude=arm_cos_f32(Latitude);
     m=COS_Latitude*l;
     
     N=arm_sin_f32(Latitude*WSG84_f)*arm_sin_f32(Latitude*WSG84_f);
     N=1-WSG84_e1*N;
     arm_sqrt_f32(N,&N);
     N=WSG84_a/N;
     

    t=COS_Latitude/arm_sin_f32(Latitude);
  
    yita_yita=WSG84_e2*WSG84_e2+COS_Latitude*COS_Latitude;
  
    m0=WSG84_a*(1-WSG84_e1*WSG84_e1);
    m2=3.0*m0*WSG84_e1*WSG84_e1/2.0;
    m4=5.0*m2*WSG84_e1*WSG84_e1/4.0;
    m6=7.0*m4*WSG84_e1*WSG84_e1/6.0;
    m8=9.0*WSG84_e1*WSG84_e1*m4/8.0;
    
    a0=m0+m2/2.0+3.0*m4/8.0+5.0*m6/16.0+35*m8/128;
    a2=m2/2.0+m4/2.0+m6*15.0/32.0+7*m8/16;
    a4=m4/8.0+m6*3.0/16.0+7*m8/32;
    a6=m6/32.0+m8/16;
    a8=m8/128;
    
    alpha= a0*Latitude-a2*arm_sin_f32(Latitude*2.0)+a4*arm_sin_f32(Latitude*4.0)+a6*arm_sin_f32(Latitude*6.0);
    
    Guassian_Projection_Cordinate.X=( m*m/2.0+m*m*m*m*(5.0-t*t+yita_yita*9.0+4.0*yita_yita*yita_yita)/24.0  + m*m*m*m*m*m*(61-58*t*t+t*t*t*t)/720.0 )*N*t +alpha;     //unit m
    Guassian_Projection_Cordinate.Y=N*(m+m*m*m*(1.0-t*t+yita_yita)/6.0+m*m*m*m*m*(5.0-18.0*t*t + t*t*t*t + 14*yita_yita -58*yita_yita*t*t )/120.0);//unit m to0.1m

  return Guassian_Projection_Cordinate;
}
