#ifndef _MultiRotor_rc_H_
#define _MultiRotor_rc_H_
#include "stm32f10x.h"

typedef struct {                     //用于存放提炼的遥控数据
	            int16_t rc_data[4];
				int16_t ROLL;
				int16_t PITCH;
				int16_t THROTTLE;
				int16_t YAW;
				int16_t SENSITIVITY; //【SENSITIVITY】敏感度参数
                }RC_GETDATA;   

/*********** RC alias *****************/
                
enum {                            //上锁解锁定义的枚举类型
    ROLL = 0,
    PITCH,     //1
    YAW,       //2
    THROTTLE,  //3
};

#define BYTE0(dwTemp)       (*(char *)(&dwTemp))
#define BYTE1(dwTemp)       (*((char *)(&dwTemp) + 1))
#define BYTE2(dwTemp)       (*((char *)(&dwTemp) + 2))
#define BYTE3(dwTemp)       (*((char *)(&dwTemp) + 3))

//枚举变量可以用字面值直接引用，当常量值用
//上锁解锁程序使用
#define ROL_LO (1 << (2 * ROLL))     //<<0   //0000 0001      0X01
#define ROL_CE (3 << (2 * ROLL))             //0000 0011      0X03
#define ROL_HI (2 << (2 * ROLL))             //0000 0010      0X02

#define PIT_LO (1 << (2 * PITCH))    //<<2   //0000 0100      0X04
#define PIT_CE (3 << (2 * PITCH))            //0000 1100      0X0C
#define PIT_HI (2 << (2 * PITCH))            //0000 1000      0X08

#define YAW_LO (1 << (2 * YAW))      //<<4   //0001 0000      0X10
#define YAW_CE (3 << (2 * YAW))              //0011 0000      0X30
#define YAW_HI (2 << (2 * YAW))              //0010 0000      0X20

#define THR_LO (1 << (2 * THROTTLE)) //<<6   //0100 0000      0X40
#define THR_CE (3 << (2 * THROTTLE))         //1100 0000      0XC0
#define THR_HI (2 << (2 * THROTTLE))         //1000 0000      0X80
				
				
extern  RC_GETDATA RC_Data;
extern  u8 accCorrect_flag;
extern  u8 turn_flag; 

void RC_Analy(void);
void RC_directive(void);
void RDAU(void);
void RC_Data_Refine(void);
#endif
