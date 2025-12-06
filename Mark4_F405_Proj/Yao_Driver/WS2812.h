#ifndef __WS2812_H
#define __WS2812_H

#include "main.h"
#include "stm32f4xx_hal.h"

// WS2812时序参数（基于168MHz系统时钟）
#define WS2812_RESET_PULSE  50    // 50us复位脉冲
#define WS2812_T0H         20    // 0码高电平时间（约0.4us）
#define WS2812_T1H         40    // 1码高电平时间（约0.8us）
#define WS2812_T0L         45    // 0码低电平时间
#define WS2812_T1L         25    // 1码低电平时间

// LED数量定义
#define LED_NUM            1     // 使用1个LED进行测试

// 颜色结构体
typedef struct {
    uint8_t green;
    uint8_t red;
    uint8_t blue;
} RGB_Color;

// 函数声明
void WS2812_Init(void);
void WS2812_SetColor(uint8_t led_num, RGB_Color color);
void WS2812_SetRGB(uint8_t led_num, uint8_t red, uint8_t green, uint8_t blue);
void WS2812_Update(void);
void WS2812_ClearAll(void);
void WS2812_TestPattern(void);

// 常用颜色定义
#define COLOR_BLACK    {0, 0, 0}
#define COLOR_RED      {0, 255, 0}
#define COLOR_GREEN    {255, 0, 0}
#define COLOR_BLUE     {0, 0, 255}
#define COLOR_WHITE    {255, 255, 255}
#define COLOR_YELLOW   {255, 255, 0}
#define COLOR_PURPLE   {255, 0, 255}
#define COLOR_CYAN     {0, 255, 255}

#endif