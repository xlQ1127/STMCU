#ifndef __OV7670_H
#define __OV7670_H


#include "main.h"
#include "stm32f4xx_hal.h"

#define SCCB_SCL_H HAL_GPIO_WritePin(GPIOB, SCCB_CLK_Pin, GPIO_PIN_SET)
#define SCCB_SCL_L HAL_GPIO_WritePin(GPIOB, SCCB_CLK_Pin, GPIO_PIN_RESET)

#define SCCB_SDA_H HAL_GPIO_WritePin(GPIOB, SCCB_SDA_Pin, GPIO_PIN_SET)
#define SCCB_SDA_L HAL_GPIO_WritePin(GPIOB, SCCB_SDA_Pin, GPIO_PIN_RESET)
		
#define OV7670_RST_H HAL_GPIO_WritePin(OV7670_RST_GPIO_Port, OV7670_RST_Pin, GPIO_PIN_SET)
#define OV7670_RST_L HAL_GPIO_WritePin(OV7670_RST_GPIO_Port, OV7670_RST_Pin, GPIO_PIN_RESET)

#define OV7670_PDWN_H HAL_GPIO_WritePin(OV7670_PDWN_GPIO_Port, OV7670_PDWN_Pin, GPIO_PIN_SET)
#define OV7670_PDWN_L HAL_GPIO_WritePin(OV7670_PDWN_GPIO_Port, OV7670_PDWN_Pin, GPIO_PIN_RESET)

#define SCCB_READ_SDA HAL_GPIO_ReadPin(GPIOB, SCCB_SDA_Pin)

#define OV7670_Adress		0x42 

uint8_t OV7670_Config(void);

void OV7670_Set_window(uint16_t startx,uint16_t starty,uint16_t width, uint16_t height);

uint8_t OV7670_Init(void);

#endif 
