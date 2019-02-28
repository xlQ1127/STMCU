#include "BSP_GPS.h"

#include "string.h"

#include "main.h"
#include "stm32f1xx_hal.h"


extern UART_HandleTypeDef huart2;
extern uint8_t UART_GPS_RX[512];
uint8_t UTC_TO_BJT[24]={
8,9,10,11,12,13,14,15,16,17,18,19,20,21,22,23,24
};
	
	


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
									
							    GPRMC.GPS_Lagitude_DD=StrToIntFix(p, 2);
							    p += 2;
							
									GPRMC.GPS_Lagitude_MM = StrToIntFix(p, 2)*100000;
									p += 3;
									
									GPRMC.GPS_Lagitude_MM += StrToIntFix(p, 5);
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

							    GPRMC.GPS_Longititude_DD=StrToIntFix(p, 3);
							    p += 3;
							
									GPRMC.GPS_Longititude_MM = StrToIntFix(p, 2)*100000;
									p += 3;
									
									GPRMC.GPS_Longititude_MM += StrToIntFix(p, 5);
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

