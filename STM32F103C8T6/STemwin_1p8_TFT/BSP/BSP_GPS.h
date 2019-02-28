#ifndef __BSP_GPS_H__
#define __BSP_GPS_H__

#include "string.h"

#include "main.h"
#include "stm32f1xx_hal.h"

#include "arm_math.h"

typedef struct 
	{
 uint8_t	GPS_UTC_Minutes  ;
 uint8_t  GPS_UTC_Hours    ;
 uint8_t	GPS_UTC_Seconds  ;
 
 uint32_t	GPS_Longitude_DD;
 uint32_t	GPS_Longitude_MM;		
		
 uint32_t	GPS_Latitude_DD;
 uint32_t	GPS_Latitude_MM;
	
	uint8_t	GPS_NS;//N:1 S:0  NOT GET:3
  uint8_t	GPS_WE;//W:1 W:0  NOT GET:3

 uint8_t GPS_Statu;	//0:NOT Ready 1: NOT OK 2:OK
	
} GPS_GPRMC_INFO;
	

typedef struct{
float32_t X;
float32_t Y;
}Guassian_Projection_Typedef;



extern GPS_GPRMC_INFO GPRMC;

	
GPS_GPRMC_INFO Get_GPS_GPRMC(void);
Guassian_Projection_Typedef Guassian_Projection(uint32_t Latitude_DDD,uint32_t Latitude_MMMMMM,uint32_t Longitude_DDD,uint32_t Longitude_MMMMMM);
	
int32_t StrToIntFix(char *_pStr, uint8_t _ucLen);
int32_t StrToInt(char *_pStr);	



#endif

