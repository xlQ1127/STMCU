#ifndef __BSP_FDC2214_H
#define __BSP_FDC2214_H

#include "stm32f1xx_hal.h"
#include "i2c.h"
/*
#define hFDC2214 hi2c1
*/
/*FDC2214    iic从地址
 *ADDR = L , I2C Address = 0x2A
 *ADDR = H , I2C Address = 0x2B*/
#define FDC2214_ADDR 0x2A

#define FDC2214_DATA_CH0 0x00 //数据寄存器
#define FDC2214_DATA_LSB_CH0 0x01
#define FDC2214_DATA_CH1 0x02
#define FDC2214_DATA_LSB_CH1 0x03
#define FDC2214_DATA_CH2 0x04
#define FDC2214_DATA_LSB_CH2 0x05
#define FDC2214_DATA_CH3 0x06
#define FDC2214_DATA_LSB_CH3 0x07

#define FDC2214_RCOUNT_CH0 0x08 //
#define FDC2214_RCOUNT_CH1 0x09
#define FDC2214_RCOUNT_CH2 0x0A
#define FDC2214_RCOUNT_CH3 0x0B
//#define OFFSET_CH0 0x0C  //FDC2114
//#define OFFSET_CH1 0x0D
//#define OFFSET_CH2 0x0E
//#define OFFSET_CH3 0x0F
#define FDC2214_SETTLECOUNT_CH0 0x10
#define FDC2214_SETTLECOUNT_CH1 0x11
#define FDC2214_SETTLECOUNT_CH2 0x12
#define FDC2214_SETTLECOUNT_CH3 0x13

#define FDC2214_CLOCK_DIVIDERS_C_CH0 0x14 //时钟分频
#define FDC2214_CLOCK_DIVIDERS_C_CH1 0x15
#define FDC2214_CLOCK_DIVIDERS_C_CH2 0x16
#define FDC2214_CLOCK_DIVIDERS_C_CH3 0x17

#define FDC2214_STATUS 0x18       //状态寄存器
#define FDC2214_ERROR_CONFIG 0x19 //错误报告设置
#define FDC2214_CONFIG 0x1A
#define FDC2214_MUX_CONFIG 0x1B
#define FDC2214_RESET_DEV 0x1C

#define FDC2214_DRIVE_CURRENT_CH0 0x1E //电流驱动
#define FDC2214_DRIVE_CURRENT_CH1 0x1F
#define FDC2214_DRIVE_CURRENT_CH2 0x20
#define FDC2214_DRIVE_CURRENT_CH3 0x21

#define FDC2214_MANUFACTURER_ID 0x7E //0x5449
#define FDC2214_DEVICE_ID 0x7F       //0x3055

//extern uint16_t Data_FDC;

//相关函数申明
void FDC_I2C2_ErrHandel(void);
uint8_t FDC2214_WriteReg(uint8_t Reg, uint8_t MSB, uint8_t LSB);
uint16_t FDC_ReadReg(uint8_t Reg);
void FDC2214_Init(void);
uint32_t FCD2214_GetCap_Data(uint8_t CH);
double FDC2214_Calculate_Cap(uint32_t Data_FDC);

#endif
