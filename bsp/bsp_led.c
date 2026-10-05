/**
 * [H7-006] 迁移: WS2812B aRGB灯带驱动
 *   - F4原版用 TIM5 3通道PWM驱动普通RGB LED
 *   - H7改用 SPI6 (PA7 MOSI, 6.0Mbps) 驱动WS2812B
 *   - 位编码: 0xE0=0码(高500ns), 0xF8=1码(高833ns)
 *   - WS2812B数据顺序: GRB, MSB first
 */
#include "bsp_led.h"
#include "main.h"
#include "spi.h"

extern SPI_HandleTypeDef hspi6;

#define WS2812_BIT_0  0xE0
#define WS2812_BIT_1  0xF8

void aRGB_led_show(uint32_t aRGB)
{
    uint8_t alpha = (aRGB >> 24) & 0xFF;
    uint8_t green = ((uint16_t)((aRGB >> 8) & 0xFF)) * alpha / 255;
    uint8_t red   = ((uint16_t)((aRGB >> 16) & 0xFF)) * alpha / 255;
    uint8_t blue  = ((uint16_t)(aRGB & 0xFF)) * alpha / 255;

    uint8_t buf[24];
    uint8_t bit;
    uint8_t *p = buf;

    for (bit = 0; bit < 8; bit++) *p++ = (green & (0x80 >> bit)) ? WS2812_BIT_1 : WS2812_BIT_0;
    for (bit = 0; bit < 8; bit++) *p++ = (red   & (0x80 >> bit)) ? WS2812_BIT_1 : WS2812_BIT_0;
    for (bit = 0; bit < 8; bit++) *p++ = (blue  & (0x80 >> bit)) ? WS2812_BIT_1 : WS2812_BIT_0;

    HAL_SPI_Transmit(&hspi6, buf, 24, 2);
}
