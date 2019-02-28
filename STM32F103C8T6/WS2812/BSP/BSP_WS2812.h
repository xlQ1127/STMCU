#ifndef __WS2812_H__
#define __WS2812_H__

#include "main.h"

#include <stdio.h>
#include <string.h>

#include "spi.h"

#define USE_OSDELAY 1
#if USE_OSDELAY

#include "cmsis_os.h"

#endif

#define PIXELS_LEN (71) //
#define BUFF_LEN (PIXELS_LEN * 12)
#define LED_ARR (BUFF_LEN + 1)

#define BIT00 (0x88)
#define BIT01 (0x8E)
#define BIT10 (0xE8)
#define BIT11 (0xEE)

#define WS_BLACK ((uint32_t)(0x00000000U))
#define WS_RED ((uint32_t)(0x0000FF00U))
#define WS_GREEN ((uint32_t)(0x00FF0000U))
#define WS_BLUE ((uint32_t)(0x000000FFU))
#define WS_YELLOW ((uint32_t)(0x00FFFF00U))
#define WS_TURQUOISE ((uint32_t)(0x00FF00FFU))
#define WS_PURPLE ((uint32_t)(0x0000FFffU))
#define WS_WHITE ((uint32_t)(0x00FFFFFFU))

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



extern void ws2812_rainbow(WS2812_HandleTypeDef *swObj, uint16_t wait);
extern void ws2812_rainbowCycle(WS2812_HandleTypeDef *swObj, uint16_t wait);
extern void ws2812_theaterChase(WS2812_HandleTypeDef *swObj, uint32_t c, uint16_t wait);
extern void ws2812_theaterChaseRainbow(WS2812_HandleTypeDef *swObj, uint16_t wait);
extern void ws2812_colorWipe(WS2812_HandleTypeDef *swObj, uint32_t c, uint16_t wait);
extern void WS2812_ShowChar(WS2812_HandleTypeDef *swObj, uint8_t ch[16],uint32_t ch_color,uint32_t Bk_Color, uint16_t wait);
extern void ws2812_blink_all(WS2812_HandleTypeDef *swObj, uint32_t color, uint16_t delay_time);
#endif
