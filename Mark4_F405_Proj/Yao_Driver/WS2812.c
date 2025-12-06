#include "ws2812.h"

// WS2812数据缓冲区（每个LED需要24位，GRB顺序）
static uint8_t ws2812_buffer[LED_NUM * 24];
static GPIO_TypeDef* WS2812_PORT = GPIOA;
static uint16_t WS2812_PIN = GPIO_PIN_0;

// 精确延时函数（基于NOP指令）
static void delay_ns(uint32_t ns) {
    uint32_t cycles = (ns * 168) / 1000;  // 168MHz下每个周期约5.95ns
    if (cycles < 1) cycles = 1;
    
    for (uint32_t i = 0; i < cycles; i++) {
        __NOP();
    }
}

// 发送一位数据
static void WS2812_SendBit(uint8_t bit) {
    if (bit) {
        // 发送'1'
        HAL_GPIO_WritePin(WS2812_PORT, WS2812_PIN, GPIO_PIN_SET);
        delay_ns(WS2812_T1H * 100);  // 转换为纳秒
        HAL_GPIO_WritePin(WS2812_PORT, WS2812_PIN, GPIO_PIN_RESET);
        delay_ns(WS2812_T1L * 100);
    } else {
        // 发送'0'
        HAL_GPIO_WritePin(WS2812_PORT, WS2812_PIN, GPIO_PIN_SET);
        delay_ns(WS2812_T0H * 100);
        HAL_GPIO_WritePin(WS2812_PORT, WS2812_PIN, GPIO_PIN_RESET);
        delay_ns(WS2812_T0L * 100);
    }
}

// 发送一个字节数据（WS2812使用GRB顺序）
static void WS2812_SendByte(uint8_t data) {
    for (int8_t i = 7; i >= 0; i--) {
        WS2812_SendBit((data >> i) & 0x01);
    }
}

// 初始化函数
void WS2812_Init(void) {
    // 确保引脚初始为低电平
    HAL_GPIO_WritePin(WS2812_PORT, WS2812_PIN, GPIO_PIN_RESET);
    
    // 清空缓冲区
    WS2812_ClearAll();
    
    // 发送复位信号
    HAL_Delay(1);
}

// 设置LED颜色（不立即更新）
void WS2812_SetColor(uint8_t led_num, RGB_Color color) {
    if (led_num >= LED_NUM) return;
    
    uint8_t *p = &ws2812_buffer[led_num * 24];
    
    // WS2812使用GRB顺序
    // 发送绿色分量
    for (int8_t i = 7; i >= 0; i--) {
        p[i] = (color.green >> (7 - i)) & 0x01;
    }
    
    // 发送红色分量
    for (int8_t i = 7; i >= 0; i--) {
        p[8 + i] = (color.red >> (7 - i)) & 0x01;
    }
    
    // 发送蓝色分量
    for (int8_t i = 7; i >= 0; i--) {
        p[16 + i] = (color.blue >> (7 - i)) & 0x01;
    }
}

// 设置RGB颜色
void WS2812_SetRGB(uint8_t led_num, uint8_t red, uint8_t green, uint8_t blue) {
    RGB_Color color = {green, red, blue};  // GRB顺序
    WS2812_SetColor(led_num, color);
}

// 更新所有LED（发送数据到WS2812）
void WS2812_Update(void) {
    // 禁用中断以确保时序精确
    __disable_irq();
    
    // 发送每个LED的数据
    for (uint16_t i = 0; i < LED_NUM * 24; i++) {
        WS2812_SendBit(ws2812_buffer[i]);
    }
    
    // 重新启用中断
    __enable_irq();
    
    // 复位信号
    HAL_GPIO_WritePin(WS2812_PORT, WS2812_PIN, GPIO_PIN_RESET);
    delay_ns(WS2812_RESET_PULSE * 1000);  // 50us复位
}

// 清空所有LED
void WS2812_ClearAll(void) {
    for (uint16_t i = 0; i < LED_NUM * 24; i++) {
        ws2812_buffer[i] = 0;
    }
    WS2812_Update();
}

// 测试模式
void WS2812_TestPattern(void) {
    RGB_Color test_colors[] = {
        COLOR_RED,
        COLOR_GREEN, 
        COLOR_BLUE,
        COLOR_WHITE,
        COLOR_YELLOW,
        COLOR_PURPLE,
        COLOR_CYAN
    };
    
    for (uint8_t i = 0; i < sizeof(test_colors)/sizeof(test_colors[0]); i++) {
        WS2812_SetColor(0, test_colors[i]);
        WS2812_Update();
        HAL_Delay(500);
    }
}