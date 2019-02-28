/*
WS28 - Library for the WS281X series of RGB LED device IC.

Copyright 2018 Waiman

Supported devices:
WS2811 WS2812
*/

#ifndef __WS2812_H__
#define __WS2812_H__

#include "main.h"

#include <stdio.h>
#include <string.h>

#include "spi.h"

#define PIXELS_LEN (71) //
#define BUFF_LEN (PIXELS_LEN * 12)
#define LED_ARR (BUFF_LEN + 1)

#define BIT00 (0x88)
#define BIT01 (0x8E)
#define BIT10 (0xE8)
#define BIT11 (0xEE)

#define BLACK ((uint32_t)(0x00000000U))
#define RED ((uint32_t)(0x0000FF00U))
#define GREEN ((uint32_t)(0x00FF0000U))
#define BLUE ((uint32_t)(0x000000FFU))
#define YELLOW ((uint32_t)(0x00FFFF00U))
#define TURQUOISE ((uint32_t)(0x00FF00FFU))
#define PURPLE ((uint32_t)(0x0000FFffU))
#define WHITE ((uint32_t)(0x00FFFFFFU))

typedef enum
{
    BUSY = 0x00U,
    READY = 0x01U
} BSP_UpdateEnumDef;

typedef enum
{
    CW,
    CCW
} DIRECTION_HandleTypeDef;

typedef struct
{
    uint8_t *BuffPoint;
    uint8_t *SendPoint;
    uint8_t Brightness;
    uint16_t PixelLen;
    uint16_t BuffLen;
    __IO BSP_UpdateEnumDef Status;
} WS2812_HandleTypeDef;

extern WS2812_HandleTypeDef hWS2812;

//**************************************************************************************
extern void WS2812_Init(WS2812_HandleTypeDef *swObj, uint8_t Len);
extern void WS2812_SetPixelsColor(WS2812_HandleTypeDef *swObj, uint8_t pix, uint32_t color);
extern void WS2812_AllBlock(WS2812_HandleTypeDef *swObj);

extern void WS2812_SetAllColor(WS2812_HandleTypeDef *swObj, uint32_t color);
extern void WS2812_Show(WS2812_HandleTypeDef *swObj);
extern uint32_t WS2812_Color(uint8_t r, uint8_t g, uint8_t b);
extern uint32_t WS2812_GetPixelColor(WS2812_HandleTypeDef *swObj, uint16_t pix);
extern uint32_t WS2812_ScaleColor(uint8_t scale, uint32_t color);
extern void WS2812_SetBrightness(WS2812_HandleTypeDef *swObj, uint8_t b);
extern void WS2812_Color2RGB(uint32_t color, uint8_t *r, uint8_t *g, uint8_t *b);

extern uint32_t Wheel(uint8_t WheelPos);

/*&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&*/
//&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
//					Ч������
//&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
/*&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&*/

extern void rainbow(WS2812_HandleTypeDef *swObj, uint16_t wait);
extern void rainbowCycle(WS2812_HandleTypeDef *swObj, uint16_t wait);
extern void theaterChase(WS2812_HandleTypeDef *swObj, uint32_t c, uint16_t wait);
extern void theaterChaseRainbow(WS2812_HandleTypeDef *swObj, uint16_t wait);
extern void colorWipe(WS2812_HandleTypeDef *swObj, uint32_t c, uint16_t wait);

extern void blink_all(WS2812_HandleTypeDef *swObj, uint32_t color, uint8_t times, uint16_t delay_time);
#endif
