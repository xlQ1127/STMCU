#ifndef SYSTEM_H
#define SYSTEM_H

#include "common.h"

typedef enum
{
  DOORWAIT,
  DOORSLEEP,
  DOOROPEN,
  DOORCLOSE,
  BATWARN,
  FACECHECK,
  NONET,
  GETNET,
}DOORSTATUS_e;

typedef enum
{
  high,
  medium,
  low,
  poweroff,
}Battery_e;

typedef struct{
	uint16 ADBuff[3];
	uint16 ADOut[3];
        RTC_DateTimeTypeDef SystemTime;
        DOORSTATUS_e DoorCmd;
        DOORSTATUS_e DoorStatus;
        int VisitorNum;
        Battery_e BatteryStatus;
        uint8 FaceCheckOpen;
        uint8 DataIR[2];
        uint8 KeyData[3];
        uint16 LPWCount;
        uint16 BATWarnCount;
}SystemDatatypedef;

extern SystemDatatypedef SystemData;

#endif