#ifndef __OLED_KK_H__
#define __OLED_KK_H__

#include "font_kk.h"
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

// 画线
void OLED_DrawLine(uint8_t x1, uint8_t y1, uint8_t x2, uint8_t y2, OLED_ColorMode color);
// 画矩形
void OLED_DrawRectangle(uint8_t x, uint8_t y, uint8_t w, uint8_t h, OLED_ColorMode color);
// 画填充矩形
void OLED_DrawFilledRectangle(uint8_t x, uint8_t y, uint8_t w, uint8_t h, OLED_ColorMode color);
// 画三角形
void OLED_DrawTriangle(uint8_t x1, uint8_t y1, uint8_t x2, uint8_t y2, uint8_t x3, uint8_t y3, OLED_ColorMode color);
// 画填充三角形
void OLED_DrawFilledTriangle(uint8_t x1, uint8_t y1, uint8_t x2, uint8_t y2, uint8_t x3, uint8_t y3, OLED_ColorMode color);
// 画圆
void OLED_DrawCircle(uint8_t x, uint8_t y, uint8_t r, OLED_ColorMode color);
// 画填充圆
void OLED_DrawFilledCircle(uint8_t x, uint8_t y, uint8_t r, OLED_ColorMode color);
// 画椭圆
void OLED_DrawEllipse(uint8_t x, uint8_t y, uint8_t a, uint8_t b, OLED_ColorMode color);
// 画填充椭圆
void OLED_DrawFilledEllipse(uint8_t x, uint8_t y, uint8_t a, uint8_t b, OLED_ColorMode color);
// 画图片
void OLED_DrawImage(uint8_t x, uint8_t y, const Image *img, OLED_ColorMode color);

// 绘制一个ASCII字符
void OLED_PrintASCIIChar(uint8_t x, uint8_t y, char ch, const ASCIIFont *font, OLED_ColorMode color);
// 绘制一个ASCII字符串
void OLED_PrintASCIIString(uint8_t x, uint8_t y, char *str, const ASCIIFont *font, OLED_ColorMode color);
// 绘制一个ASCII字符串，自动换行
void OLED_PrintASCIIStringAutoEnter(uint8_t x, uint8_t y, char *str, const ASCIIFont *font, OLED_ColorMode color);
// 绘制一个ASCII字符串，自动换行，每个单词完整显示
void OLED_PrintASCIIStringAutoEnterFullWords(uint8_t x, uint8_t y, char *str, const ASCIIFont *font, OLED_ColorMode color);
// 绘制一个uint8_t数字
void OLED_PrintNum(uint8_t x, uint8_t y, uint8_t *num, const ASCIIFont *font, OLED_ColorMode color);
// 绘制一个浮点数
void OLED_PrintFloat(uint8_t x, uint8_t y, double num, uint8_t precision, const ASCIIFont *font, OLED_ColorMode color);
// 绘制字符串
void OLED_PrintString(uint8_t x, uint8_t y, char *str, const Font *font, OLED_ColorMode color);

#endif // __OLED_H__
