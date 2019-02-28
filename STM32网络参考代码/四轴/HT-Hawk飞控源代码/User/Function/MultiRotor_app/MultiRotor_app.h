#ifndef _MultiRotor_app_H_
#define _MultiRotor_app_H_
/* Includes ------------------------------------------------------------------*/
#include "include.h"

//各种标志的结构体封装
typedef struct {
	      u8 MpuExist;      // MPU存在
	      u8 MagExist;      // MAG存在
	      u8 NrfExist;      // NRF存在
	      u8 MagIssue;      // MAG有问题
          u8 ARMED;         // 电机解锁
	      u8 LockYaw;       // 航向锁定       
          u8 calibratingA;  // 加速度采集
	      u8 calibratingM;  // 磁力计采集
	      u8 calibratingM_pre; //磁力计预采集
	      
	      u8 ParamSave;     // 参数保存标志
	
	      u8 Loop_250Hz;
	      u8 Loop_100Hz;
	      u8 Loop_10Hz;
         }Flag_t;

extern Flag_t flag;

void loop(void);
void Bootloader_Set(void);
void InitBoard(void);
void Sensor_Init(void);
void Screen_Update(void);
void Time_slice(void);
#endif /* __MultiRotor_app_H */



