#ifndef __OLED_KK_SIMPLE_H__
#define __OLED_KK_SIMPLE_H__

#include "font_kk_simple.h"
#include "main.h"
#include "string.h"

typedef enum {
  OLED_COLOR_NORMAL = 0, // 正常模式 黑底白字
  OLED_COLOR_REVERSED    // 反色模式 白底黑字
} OLED_ColorMode;

void OLED_kk_Init(void);
void OLED_DisPlay_On(void);
void OLED_DisPlay_Off(void);

void OLED_NewFrame(void);
void OLED_ShowFrame(void);
void OLED_SetPixel(uint8_t x, uint8_t y, OLED_ColorMode color);

// 绘制一个ASCII字符
void OLED_PrintASCIIChar(uint8_t x, uint8_t y, char ch, const ASCIIFont *font, OLED_ColorMode color);
// 绘制一个ASCII字符串
void OLED_PrintASCIIString(uint8_t x, uint8_t y, char *str, const ASCIIFont *font, OLED_ColorMode color);
// 绘制一个ASCII字符串，自动换行
void OLED_PrintASCIIStringAutoEnter(uint8_t x, uint8_t y, char *str, const ASCIIFont *font, OLED_ColorMode color);
// 绘制一个ASCII字符串，自动换行，每个单词完整显示
void OLED_PrintASCIIStringAutoEnterFullWords(uint8_t x, uint8_t y, char *str, const ASCIIFont *font, OLED_ColorMode color);
// 绘制一个uint8_t数字
void OLED_Print_u8_Num(uint8_t x, uint8_t y, uint8_t *num, const ASCIIFont *font, OLED_ColorMode color);
// 绘制一个uint16_t数字
void OLED_Print_u16_Num(uint8_t x, uint8_t y, uint16_t *num, const ASCIIFont *font, OLED_ColorMode color);
// 绘制一个浮点数
void OLED_PrintFloat(uint8_t x, uint8_t y, double num, uint8_t precision, const ASCIIFont *font, OLED_ColorMode color);

#endif // __OLED_KK_SIMPLE_H__
