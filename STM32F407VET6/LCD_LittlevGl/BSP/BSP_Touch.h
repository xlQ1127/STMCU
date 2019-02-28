#ifndef  __BSP_TOUCH_H__
#define  __BSP_TOUCH_H__

#include "main.h"
#include "stm32f4xx_hal.h"

//#include "GUI.h"

// A/D 通道选择命令字和工作寄存器
// #define	CHX 	0xd0 	//通道Y+的选择控制字	
// #define	CHY 	0x90	//通道X+的选择控制字
#define	CHX 	0x90 	//通道Y+的选择控制字	
#define	CHY 	0xd0	//通道X+的选择控制字


#define TP_CS_Slecte       HAL_GPIO_WritePin(Touch_CS_GPIO_Port, Touch_CS_Pin, GPIO_PIN_RESET)
#define TP_CS_Unselected   HAL_GPIO_WritePin(Touch_CS_GPIO_Port, Touch_CS_Pin, GPIO_PIN_SET)

unsigned int TOUCH_X(void); 
unsigned int TOUCH_Y(void); 
/*
int  GUI_TOUCH_X_MeasureX(void) ;
int  GUI_TOUCH_X_MeasureY(void) ;
*/
void TP_GetAdXY(unsigned int *x,unsigned int *y);

char IsPressed(uint16_t x, uint16_t y,uint16_t x0, uint16_t y0, uint16_t lenth, uint16_t width,char* pressed);
char IsPressed_V2(uint16_t x, uint16_t y,uint16_t x0, uint16_t y0, uint16_t lenth, uint16_t width,char* pressed);
char IsPressed_V3(uint16_t x, uint16_t y,uint16_t x0, uint16_t y0, uint16_t lenth, uint16_t width,char* pressed);


#endif                                     
