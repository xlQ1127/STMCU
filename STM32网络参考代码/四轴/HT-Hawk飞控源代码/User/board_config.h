#ifndef __BOARD_CONFIG_H
#define	__BOARD_CONFIG_H

#include "include.h"


#define QUADROTOR 
//#define HEXRCOPTER 

#ifdef QUADROTOR 
		#define MOTOR_NUM 4
#elif defined HEXRCOPTER
		#define MOTOR_NUM 6
#endif 



/*-------------向量表偏移量----------------------*/
/*-------------重要  不要动----------------------*/
#define FLASH_EXCURSION  0x20000
#define pro_FALG_ADD 0x0801FFF0

/*----------------电机怠速----------------------*/
#define IDLING 210

/*--------------遥控控制方式选择----------------*/
#define RC_CONTROL_USE_NRF24l01

/*---------------陀螺仪采集---------------------*/
#define GYRO_GATHER   100 

/*----------------油门检查----------------------*/
#define RC_MINCHECK   1200      //【油门最小值】
#define RC_MAXCHECK   1800

/*-------------自动解除武装时间-----------------*/
#define AUTODISARMDE_TIME 2500  


//用 typedef简化函数指针的定义
typedef void (*rcReadRawData)(void);      

#endif /* __BOARD_CONFIG_H */
