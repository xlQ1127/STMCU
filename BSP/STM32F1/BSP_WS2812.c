#include "WS2812.h"
#include <math.h>

// 定义一条WS2812灯串
WS2812_HandleTypeDef hWS2812;

// 用于显示缓存
static uint8_t buff_a[LED_ARR] = {BIT00};
// 用于发送缓存
static uint8_t buff_b[LED_ARR] = {BIT00};

/****************************************************************************
* 名    称： WS2812_Init(WS2812_HandleTypeDef *swObj, uint8_t pixelLen)
* 功    能：
* 入口参数：
* 出口参数：
* 说    明：
****************************************************************************/
void WS2812_Init(WS2812_HandleTypeDef *wsObj, uint8_t pixelLen)
{
    wsObj->PixelLen = pixelLen;
    wsObj->BuffLen = pixelLen * 12 + 1;
    wsObj->BuffPoint = buff_a;
    wsObj->SendPoint = buff_b;
    wsObj->Brightness = 0;
    *(wsObj->BuffPoint + wsObj->BuffLen - 1) = 0xFE; //

    wsObj->Status = READY;
    WS2812_AllBlock(wsObj);
    WS2812_Show(wsObj);
}

/****************************************************************************
* 名    称：void WS2812_AllBlock(void)
* 功    能：关闭颜色
* 入口参数：
* 出口参数：
* 说    明：
****************************************************************************/
void WS2812_AllBlock(WS2812_HandleTypeDef *swObj)
{
    memset(swObj->BuffPoint, BIT00, (swObj->BuffLen - 1));
    swObj->Status = READY;
}

/****************************************************************************
* 名    称：WS2812_SetPixelsColor(WS2812_HandleTypeDef *swObj, uint8_t pix, uint32_t color)
* 功    能：设置指定LED的颜色
* 入口参数：
* 出口参数：
* 说    明：
****************************************************************************/
void WS2812_SetPixelsColor(WS2812_HandleTypeDef *swObj, uint8_t pix, uint32_t color)
{
    uint8_t i;
    uint8_t tmp;
    uint8_t *pb = swObj->BuffPoint + pix * 12;
    for (i = 0; i < 12; i++)
    {
        tmp = (uint8_t)(color & 0x00000003);
        switch (tmp)
        {
        case 0x00:
            *pb = BIT00;
            break;
        case 0x01:
            *pb = BIT01;
            break;
        case 0x02:
            *pb = BIT10;
            break;
        case 0x03:
            *pb = BIT11;
            break;
        }
        color = color >> 2;
        pb++;
    }
}

/****************************************************************************
* 名    称：WS2812_SetAllColor(WS2812_HandleTypeDef *swObj, uint32_t color)
* 功    能：设置所以LED使用同一颜色
* 入口参数：
* 出口参数：
* 说    明：
****************************************************************************/
void WS2812_SetAllColor(WS2812_HandleTypeDef *swObj, uint32_t color)
{
    uint8_t i;
    uint8_t *dest = swObj->BuffPoint + 12;
    WS2812_SetPixelsColor(swObj, 0, color);
    for (i = 1; i < swObj->PixelLen; i++)
    {
        memcpy(dest, swObj->BuffPoint, (12 * sizeof(uint8_t)));
        dest += 12;
    }
    swObj->Status = READY;
}

/****************************************************************************
* 名    称：uint32_t WS2812_Color(uint8_t r, uint8_t g, uint8_t b)
* 功    能：
* 入口参数：
* 出口参数：
* 说    明：
****************************************************************************/
uint32_t WS2812_Color(uint8_t r, uint8_t g, uint8_t b)
{
    return (((uint32_t)g << 16) | ((uint32_t)r << 8) | b);
}

/****************************************************************************
* 名    称：void WS2812_Color2RGB(uint32_t color, uint8_t* r, uint8_t* g, uint8_t* b)
* 功    能：
* 入口参数：
* 出口参数：
* 说    明：
****************************************************************************/
void WS2812_Color2RGB(uint32_t color, uint8_t *r, uint8_t *g, uint8_t *b)
{
    *b = (uint8_t)color;
    *r = (uint8_t)(color >> 8);
    *g = (uint8_t)(color >> 16);
}

/****************************************************************************
* 名    称：uint32_t WS2812_GetPixelColor(WS2812_HandleTypeDef *swObj, uint16_t pix)
* 功    能：
* 入口参数：
* 出口参数：
* 说    明：
****************************************************************************/
uint32_t WS2812_GetPixelColor(WS2812_HandleTypeDef *swObj, uint16_t pix)
{
    uint8_t i;
    uint8_t tmp;
    uint32_t color = 0;
    uint8_t *pb = swObj->BuffPoint + pix * 12;
    for (i = 0; i < 12; i++)
    {
        color = color << 2;
        switch (*pb)
        {
        case BIT00:
            tmp = 0x0;
            break;
        case BIT01:
            tmp = 0x1;
            break;
        case BIT10:
            tmp = 0x2;
            break;
        case BIT11:
            tmp = 0x3;
            break;
        }
        color |= tmp;
        pb++;
    }
    return color;
}

/****************************************************************************
* 名    称：WS2812_Show(void)
* 功    能：
* 入口参数：
* 出口参数：
* 说    明：
****************************************************************************/
void WS2812_Show(WS2812_HandleTypeDef *swObj)
{
    // SPI 空闲
    if (HAL_SPI_GetState(&hspi1) == HAL_SPI_STATE_READY)
    {
        // 有缓存需要更新，更新显示缓存
        if (swObj->Status == READY)
        {
            memcpy(swObj->SendPoint, swObj->BuffPoint, swObj->BuffLen);
            swObj->Status = BUSY;
        }
        // 发送显示缓存
        if (HAL_SPI_Transmit(&hspi1, swObj->SendPoint, swObj->BuffLen, 0xFF) != HAL_OK)
        {
            Error_Handler();
        }
    }
}

/****************************************************************************
* 名    称：
* 功    能：
* 入口参数：
* 出口参数：
* 说    明：
****************************************************************************/
uint32_t WS2812_ScaleColor(uint8_t scale, uint32_t color)
{
    uint8_t r, g, b;

    b = (uint8_t)color;
    r = (uint8_t)(color >> 8);
    g = (uint8_t)(color >> 16);

    r = (r * scale) >> 8;
    g = (g * scale) >> 8;
    b = (b * scale) >> 8;

    return (((uint32_t)g << 16) | ((uint32_t)r << 8) | b);
}

/****************************************************************************
* 名    称：WS2812_Show(void)
* 功    能：
* 入口参数：
* 出口参数：
* 说    明：
****************************************************************************/
void WS2812_SetBrightness(WS2812_HandleTypeDef *swObj, uint8_t b)
{
    uint8_t newBrightness = b + 1;
    uint8_t oldBrightness = swObj->Brightness - 1; // De-wrap old brightness value
    uint32_t c;
    uint16_t scale, i;

    if (newBrightness != swObj->Brightness)
    { // Compare against prior value
        // Brightness has changed -- re-scale existing data in RAM

        if (oldBrightness == 0)
            scale = 0; // Avoid /0
        else if (b == 255)
            scale = 65535 / oldBrightness;
        else
            scale = (((uint16_t)newBrightness << 8) - 1) / oldBrightness;

        for (i = 0; i < swObj->PixelLen; i++)
        {
            c = WS2812_GetPixelColor(swObj, i); //获取16bit颜色
            c = WS2812_ScaleColor(scale, c);
            WS2812_SetPixelsColor(swObj, i, c); //设置颜色
        }
        swObj->Brightness = newBrightness;
    }
}

/****************************************************************************
* 名    称：
* 功    能：输入值0到255以获取颜色值。返回一个过渡颜色 r-g-b。
* 入口参数：
* 出口参数：
* 说    明：
****************************************************************************/
uint32_t Wheel(uint8_t WheelPos)
{
    WheelPos = 255 - WheelPos;
    if (WheelPos < 85)
    {
        return WS2812_Color(255 - WheelPos * 3, 0, WheelPos * 3);
    }
    if (WheelPos < 170)
    {
        WheelPos -= 85;
        return WS2812_Color(0, WheelPos * 3, 255 - WheelPos * 3);
    }
    WheelPos -= 170;
    return WS2812_Color(WheelPos * 3, 255 - WheelPos * 3, 0);
}

/*&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&*/
//&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
//					效果函数
//&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
/*&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&*/

/****************************************************************************
* 名    称：
* 功    能：彩虹
* 入口参数：
* 出口参数：
* 说    明：
****************************************************************************/
void rainbow(WS2812_HandleTypeDef *swObj, uint16_t wait)
{
    uint16_t i, j;
    for (j = 0; j < 256; j++)
    {
        for (i = 0; i < swObj->PixelLen; i++)
        {
            WS2812_SetPixelsColor(swObj, i, Wheel((i + j) & 255));
        }
        swObj->Status = READY;
        WS2812_Show(swObj);
        HAL_Delay(wait);
    }
}

/****************************************************************************
* 名    称：
* 功    能：略有不同, 这使得彩虹均匀分布在整个
* 入口参数：
* 出口参数：
* 说    明：
****************************************************************************/
void rainbowCycle(WS2812_HandleTypeDef *swObj, uint16_t wait)
{
    uint16_t i, j;
    for (j = 0; j < 256 * 5; j++)
    { // 5 cycles of all colors on wheel
        for (i = 0; i < swObj->PixelLen; i++)
        {
            WS2812_SetPixelsColor(swObj, i, Wheel(((i * 256 / swObj->PixelLen) + j) & 255));
        }
        swObj->Status = READY;
        WS2812_Show(swObj);
        HAL_Delay(wait);
    }
}

/****************************************************************************
* 名    称：
* 功    能：剧场式的爬行灯。
* 入口参数：
* 出口参数：
* 说    明：
****************************************************************************/
void theaterChase(WS2812_HandleTypeDef *swObj, uint32_t c, uint16_t wait)
{
    for (int j = 0; j < 10; j++)
    { //do 10 cycles of chasing
        for (int q = 0; q < 3; q++)
        {
            for (uint16_t i = 0; i < swObj->PixelLen; i = i + 3)
            {
                WS2812_SetPixelsColor(swObj, i + q, c); //turn every third pixel on
            }
            swObj->Status = READY;
            WS2812_Show(swObj);
            HAL_Delay(wait);

            for (uint16_t i = 0; i < swObj->PixelLen; i = i + 3)
            {
                WS2812_SetPixelsColor(swObj, i + q, 0); //turn every third pixel off
            }
            swObj->Status = READY;
            WS2812_Show(swObj);
            HAL_Delay(2);
        }
    }
}

/****************************************************************************
* 名    称：
* 功    能：具有彩虹效果的剧场式爬行灯
* 入口参数：
* 出口参数：
* 说    明：
****************************************************************************/
void theaterChaseRainbow(WS2812_HandleTypeDef *swObj, uint16_t wait)
{
    uint16_t j, q, i;
    for (j = 0; j < 256; j++)
    { // cycle all 256 colors in the wheel
        for (q = 0; q < 3; q++)
        {
            for (i = 0; i < swObj->PixelLen; i = i + 3)
            {
                WS2812_SetPixelsColor(swObj, i + q, Wheel((i + j) % 255)); //turn every third pixel on
            }
            swObj->Status = READY;
            WS2812_Show(swObj);
            HAL_Delay(wait);

            for (i = 0; i < swObj->PixelLen; i = i + 3)
            {
                WS2812_SetPixelsColor(swObj, i + q, 0); //turn every third pixel off
            }
            swObj->Status = READY;
            WS2812_Show(swObj);
            HAL_Delay(wait);
        }
    }
}

/****************************************************************************
* 名    称：
* 功    能：颜色填充一个后的点
* 入口参数：
* 出口参数：
* 说    明：
****************************************************************************/
void colorWipe(WS2812_HandleTypeDef *swObj, uint32_t c, uint16_t wait)
{
    uint16_t i = 0;
    for (i = 0; i < swObj->PixelLen; i++)
    {
        WS2812_SetPixelsColor(swObj, i, c);
        swObj->Status = READY;
        WS2812_Show(swObj);
        HAL_Delay(wait);
    }
}

/****************************************************************************
* 名    称：
* 功    能：指定颜色闪动
* 入口参数：
* 出口参数：
* 说    明：
****************************************************************************/
void blink_all(WS2812_HandleTypeDef *swObj, uint32_t color, uint8_t times, uint16_t delay_time)
{
    uint8_t i;
    for (i = 0; i < times; i++)
    {
        WS2812_SetAllColor(swObj, color);
        swObj->Status = READY;
        WS2812_Show(swObj);
        HAL_Delay(delay_time);

        WS2812_AllBlock(swObj); // off
        swObj->Status = READY;
        WS2812_Show(swObj);
        HAL_Delay(delay_time);
    }
}
