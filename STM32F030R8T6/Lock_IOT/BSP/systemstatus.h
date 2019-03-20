#ifndef SYSTEM_H
#define SYSTEM_H

#include "stm32f0xx_hal.h"

typedef enum
{
  DOOR_WAIT,
  DOOR_SLEEP,
  DOOR_OPEN,
  DOOR_CLOSE,
  BAT_WARN,
  FACE_CHECKED,
  NET_CONC,
  NET_DISCONC,
}DOORSTATUS_TypeEnum;

typedef enum
{
  high,
  medium,
  low,
  poweroff,
}BATTERY_TypeEnum;

typedef struct{
		BATTERY_TypeEnum  Status;
		float             AD;
		uint16_t          WarnCount;		
}Battery_Typedef;


typedef struct{
			int VisitorNum;
			uint16_t LPWCount;
}SystemStatus_Typedef;

extern SystemStatus_Typedef SystemStatus;
extern Battery_Typedef  Battery;
#endif
