#ifndef __BSP_GPS_H__
#define __BSP_GPS_H__

#include "string.h"

#include "main.h"
#include "stm32f1xx_hal.h"


typedef struct 
	{
 uint8_t	GPS_UTC_Minutes  ;
 uint8_t  GPS_UTC_Hours    ;
 uint8_t	GPS_UTC_Seconds  ;
 
 uint32_t	GPS_Longititude_DD;
 uint32_t	GPS_Longititude_MM;		
		
 uint32_t	GPS_Lagitude_DD;
 uint32_t	GPS_Lagitude_MM;
	
	uint8_t	GPS_NS;//N:1 S:0  NOT GET:3
  uint8_t	GPS_WE;//W:1 W:0  NOT GET:3

 uint8_t GPS_Statu;	//0:NOT Ready 1: NOT OK 2:OK
	
} GPS_GPRMC_INFO;
	





extern GPS_GPRMC_INFO GPRMC;

	
GPS_GPRMC_INFO Get_GPS_GPRMC(void);

	
int32_t StrToIntFix(char *_pStr, uint8_t _ucLen);
int32_t StrToInt(char *_pStr);	



#endif

