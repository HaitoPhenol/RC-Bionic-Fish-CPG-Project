/**
 * @file oled.c
 * @brief 波特律动OLED驱动(SSD1306)
 * @anchor 波特律动(keysking 博哥在学习)
 * @version 1.0
 * @date 2023-08-19
 * @license MIT License
 *
 * @attention
 * 本驱动库针对波特律动·keysking的STM32教程学习套件进行开发
 * 在其他平台或驱动芯片上使用可能需要进行移植
 *
 * @note
 * 使用流程:
 * 1. STM32初始化IIC完成后调用OLED_Init()初始化OLED. 注意STM32启动比OLED上电快, 可等待20ms再初始化OLED
 * 2. 调用OLED_NewFrame()开始绘制新的一帧
 * 3. 调用OLED_DrawXXX()系列函数绘制图形到显存 调用OLED_Printxxx()系列函数绘制文本到显存
 * 4. 调用OLED_ShowFrame()将显存内容显示到OLED
 *
 * @note
 * 为保证中文显示正常 请将编译器的字符集设置为UTF-8
 *
 */
#include "oled_kk_simple.h"
#include "i2c.h"
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>

#include "ctype.h"

// OLED器件地址
#define OLED_ADDRESS 0x78

// OLED参数
#define OLED_PAGE 8            // OLED页数
#define OLED_ROW 8 * OLED_PAGE // OLED行数
#define OLED_COLUMN 128        // OLED列数

// 显存
uint8_t OLED_GRAM[OLED_PAGE][OLED_COLUMN];

// ========================== 底层通信函数 ==========================

/**
 * @brief 向OLED发送数据的函数
 * @param data 要发送的数据
 * @param len 要发送的数据长度
 * @return None
 * @note 此函数是移植本驱动时的重要函数 将本驱动库移植到其他平台时应根据实际情况修改此函数
 */
void OLED_Send(uint8_t *data, uint8_t len)
{
  HAL_I2C_Master_Transmit(&hi2c1, OLED_ADDRESS, data, len, HAL_MAX_DELAY);
}

/**
 * @brief 向OLED发送指令
 */
void OLED_SendCmd(uint8_t cmd)
{
  static uint8_t sendBuffer[2] = {0};
  sendBuffer[1] = cmd;
  OLED_Send(sendBuffer, 2);
}

// ========================== OLED驱动函数 ==========================

/**
 * @brief 初始化OLED (SSD1306)
 * @note 此函数是移植本驱动时的重要函数 将本驱动库移植到其他驱动芯片时应根据实际情况修改此函数
 */
void OLED_kk_Init(void)
{
  OLED_SendCmd(0xAE); /*关闭显示 display off*/

  OLED_SendCmd(0x20);
  OLED_SendCmd(0x10);

  OLED_SendCmd(0xB0);

  OLED_SendCmd(0xC8);

  OLED_SendCmd(0x00);
  OLED_SendCmd(0x10);

  OLED_SendCmd(0x40);

  OLED_SendCmd(0x81);

  OLED_SendCmd(0xDF);
  OLED_SendCmd(0xA1);

  OLED_SendCmd(0xA6);
  OLED_SendCmd(0xA8);

  OLED_SendCmd(0x3F);

  OLED_SendCmd(0xA4);

  OLED_SendCmd(0xD3);
  OLED_SendCmd(0x00);

  OLED_SendCmd(0xD5);
  OLED_SendCmd(0xF0);

  OLED_SendCmd(0xD9);
  OLED_SendCmd(0x22);

  OLED_SendCmd(0xDA);
  OLED_SendCmd(0x12);

  OLED_SendCmd(0xDB);
  OLED_SendCmd(0x20);

  OLED_SendCmd(0x8D);
  OLED_SendCmd(0x14);

  OLED_NewFrame();
  OLED_ShowFrame();

  OLED_SendCmd(0xAF); /*开启显示 display ON*/
}

/**
 * @brief 开启OLED显示
 */
void OLED_DisPlay_On(void)
{
  OLED_SendCmd(0x8D); // 电荷泵使能
  OLED_SendCmd(0x14); // 开启电荷泵
  OLED_SendCmd(0xAF); // 点亮屏幕
}

/**
 * @brief 关闭OLED显示
 */
void OLED_DisPlay_Off(void)
{
  OLED_SendCmd(0x8D); // 电荷泵使能
  OLED_SendCmd(0x10); // 关闭电荷泵
  OLED_SendCmd(0xAE); // 关闭屏幕
}

/**
 * @brief 设置颜色模式 黑底白字或白底黑字
 * @param ColorMode 颜色模式COLOR_NORMAL/COLOR_REVERSED
 * @note 此函数直接设置屏幕的颜色模式
 */
void OLED_SetColorMode(OLED_ColorMode mode)
{
  if (mode == OLED_COLOR_NORMAL)
  {
    OLED_SendCmd(0xA6); // 正常显示
  }
  if (mode == OLED_COLOR_REVERSED)
  {
    OLED_SendCmd(0xA7); // 反色显示
  }
}

// ========================== 显存操作函数 ==========================

/**
 * @brief 清空显存 绘制新的一帧
 */
void OLED_NewFrame(void)
{
  memset(OLED_GRAM, 0, sizeof(OLED_GRAM));
}

/**
 * @brief 将当前显存显示到屏幕上
 * @note 此函数是移植本驱动时的重要函数 将本驱动库移植到其他驱动芯片时应根据实际情况修改此函数
 */
void OLED_ShowFrame(void)
{
  static uint8_t sendBuffer[OLED_COLUMN + 1];
  sendBuffer[0] = 0x40;
  for (uint8_t i = 0; i < OLED_PAGE; i++)
  {
    OLED_SendCmd(0xB0 + i); // 设置页地址
    OLED_SendCmd(0x00);     // 设置列地址低4位
    OLED_SendCmd(0x10);     // 设置列地址高4位
    memcpy(sendBuffer + 1, OLED_GRAM[i], OLED_COLUMN);
    OLED_Send(sendBuffer, OLED_COLUMN + 1);
  }
  // OLED_NewFrame();
}

/**
 * @brief 设置一个像素点
 * @param x 横坐标
 * @param y 纵坐标
 * @param color 颜色
 */
void OLED_SetPixel(uint8_t x, uint8_t y, OLED_ColorMode color)
{
  if (x >= OLED_COLUMN || y >= OLED_ROW)
    return;
  if (!color)
  {
    OLED_GRAM[y / 8][x] |= 1 << (y % 8);
  }
  else
  {
    OLED_GRAM[y / 8][x] &= ~(1 << (y % 8));
  }
}

/**
 * @brief 设置显存中一字节数据的某几位
 * @param page 页地址
 * @param column 列地址
 * @param data 数据
 * @param start 起始位
 * @param end 结束位
 * @param color 颜色
 * @note 此函数将显存中的某一字节的第start位到第end位设置为与data相同
 * @note start和end的范围为0-7, start必须小于等于end
 * @note 此函数与OLED_SetByte_Fine的区别在于此函数只能设置显存中的某一真实字节
 */
void OLED_SetByte_Fine(uint8_t page, uint8_t column, uint8_t data, uint8_t start, uint8_t end, OLED_ColorMode color)
{
  static uint8_t temp;
  if (page >= OLED_PAGE || column >= OLED_COLUMN)
    return;
  if (color)
    data = ~data;

  temp = data | (0xff << (end + 1)) | (0xff >> (8 - start));
  OLED_GRAM[page][column] &= temp;
  temp = data & ~(0xff << (end + 1)) & ~(0xff >> (8 - start));
  OLED_GRAM[page][column] |= temp;
  // 使用OLED_SetPixel实现
  // for (uint8_t i = start; i <= end; i++) {
  //   OLED_SetPixel(column, page * 8 + i, !((data >> i) & 0x01));
  // }
}

/**
 * @brief 设置显存中的一字节数据
 * @param page 页地址
 * @param column 列地址
 * @param data 数据
 * @param color 颜色
 * @note 此函数将显存中的某一字节设置为data的值
 */
void OLED_SetByte(uint8_t page, uint8_t column, uint8_t data, OLED_ColorMode color)
{
  if (page >= OLED_PAGE || column >= OLED_COLUMN)
    return;
  if (color)
    data = ~data;
  OLED_GRAM[page][column] = data;
}

/**
 * @brief 设置显存中的一字节数据的某几位
 * @param x 横坐标
 * @param y 纵坐标
 * @param data 数据
 * @param len 位数
 * @param color 颜色
 * @note 此函数将显存中从(x,y)开始向下数len位设置为与data相同
 * @note len的范围为1-8
 * @note 此函数与OLED_SetByte_Fine的区别在于此函数的横坐标和纵坐标是以像素为单位的, 可能出现跨两个真实字节的情况(跨页)
 */
void OLED_SetBits_Fine(uint8_t x, uint8_t y, uint8_t data, uint8_t len, OLED_ColorMode color)
{
  uint8_t page = y / 8;
  uint8_t bit = y % 8;
  if (bit + len > 8)
  {
    OLED_SetByte_Fine(page, x, data << bit, bit, 7, color);
    OLED_SetByte_Fine(page + 1, x, data >> (8 - bit), 0, len + bit - 1 - 8, color);
  }
  else
  {
    OLED_SetByte_Fine(page, x, data << bit, bit, bit + len - 1, color);
  }
  // 使用OLED_SetPixel实现
  // for (uint8_t i = 0; i < len; i++) {
  //   OLED_SetPixel(x, y + i, !((data >> i) & 0x01));
  // }
}

/**
 * @brief 设置显存中一字节长度的数据
 * @param x 横坐标
 * @param y 纵坐标
 * @param data 数据
 * @param color 颜色
 * @note 此函数将显存中从(x,y)开始向下数8位设置为与data相同
 * @note 此函数与OLED_SetByte的区别在于此函数的横坐标和纵坐标是以像素为单位的, 可能出现跨两个真实字节的情况(跨页)
 */
void OLED_SetBits(uint8_t x, uint8_t y, uint8_t data, OLED_ColorMode color)
{
  uint8_t page = y / 8;
  uint8_t bit = y % 8;
  OLED_SetByte_Fine(page, x, data << bit, bit, 7, color);
  if (bit)
  {
    OLED_SetByte_Fine(page + 1, x, data >> (8 - bit), 0, bit - 1, color);
  }
}

/**
 * @brief 设置一块显存区域
 * @param x 起始横坐标
 * @param y 起始纵坐标
 * @param data 数据的起始地址
 * @param w 宽度
 * @param h 高度
 * @param color 颜色
 * @note 此函数将显存中从(x,y)开始的w*h个像素设置为data中的数据
 * @note data的数据应该采用列行式排列
 */
void OLED_SetBlock(uint8_t x, uint8_t y, const uint8_t *data, uint8_t w, uint8_t h, OLED_ColorMode color)
{
  uint8_t fullRow = h / 8; // 完整的行数
  uint8_t partBit = h % 8; // 不完整的字节中的有效位数
  for (uint8_t i = 0; i < w; i++)
  {
    for (uint8_t j = 0; j < fullRow; j++)
    {
      OLED_SetBits(x + i, y + j * 8, data[i + j * w], color);
    }
  }
  if (partBit)
  {
    uint16_t fullNum = w * fullRow; // 完整的字节数
    for (uint8_t i = 0; i < w; i++)
    {
      OLED_SetBits_Fine(x + i, y + (fullRow * 8), data[fullNum + i], partBit, color);
    }
  }
}

// ================================ 文字绘制 ================================

// 计算字符的网格横坐标（每个字符占用6个像素）
uint8_t x_line(uint8_t x)
{
  return (x - 1) * 6 + 1;
}

// 计算字符的网格纵坐标（每个字符占用8个像素）
uint8_t y_line(uint8_t y)
{
  return (y - 1) * 8 + 1;
}

/**
 * @brief 绘制一个ASCII字符
 * @param x 起始点横坐标
 * @param y 起始点纵坐标
 * @param ch 字符
 * @param font 字体
 * @param color 颜色
 */
void OLED_PrintASCIIChar(uint8_t x, uint8_t y, char ch, const ASCIIFont *font, OLED_ColorMode color)
{
  // 将char类型转换为unsigned char，避免有符号类型作为数组索引导致的警告
  OLED_SetBlock(x, y, font->chars + ((unsigned char)ch - ' ') * (((font->h + 7) / 8) * font->w), font->w, font->h, color);
}

/**
 * @brief 绘制一个ASCII字符串
 * @param x 起始点横坐标
 * @param y 起始点纵坐标
 * @param str 字符串
 * @param font 字体
 * @param color 颜色
 */
void OLED_PrintASCIIString(uint8_t x, uint8_t y, char *str, const ASCIIFont *font, OLED_ColorMode color)
{
  uint8_t x0 = x_line(x);
  while (*str)
  {
    OLED_PrintASCIIChar(x0, y_line(y), *str, font, color);
    x0 += font->w;
    str++;
  }
}

/**
 * @brief 绘制一个ASCII字符串，并自动换行
 * @param x 起始点横坐标
 * @param y 起始点纵坐标
 * @param str 字符串
 * @param font 字体
 * @param color 颜色
 */
void OLED_PrintASCIIStringAutoEnter(uint8_t x, uint8_t y, char *str, const ASCIIFont *font, OLED_ColorMode color)
{
  uint8_t x0 = x_line(x);
  uint8_t y0 = y_line(y);
  uint8_t max_x = OLED_COLUMN - font->w; // 计算最大x坐标
  
  while (*str) {
    // 处理换行符
    if (*str == '\n') {
      x0 = x;
      y0 += font->h;
      str++;
      continue;
    }
    
    // 自动换行检查
    if (x0 > max_x) {
      x0 = x;
      y0 += font->h;
    }
    
    // 绘制字符
    OLED_PrintASCIIChar(x0, y0, *str, font, color);
    x0 += font->w;
    str++;
  }
}

/**
 * @brief 绘制一个ASCII字符串，并自动换行，且单词不会被换行分割
 * @param x 起始点横坐标
 * @param y 起始点纵坐标
 * @param str 字符串
 * @param font 字体
 * @param color 颜色
 */
void OLED_PrintASCIIStringAutoEnterFullWords(uint8_t x, uint8_t y, char *str, const ASCIIFont *font, OLED_ColorMode color)
{
  uint8_t x0 = x_line(x);
  uint8_t y0 = y_line(y);
  uint8_t max_x = OLED_COLUMN - font->w; // 计算最大x坐标
  char *word_start = str; // 当前单词起始位置
  uint8_t word_width = 0; // 当前单词宽度
  
  while (*str) {
    // 处理换行符
    if (*str == '\n') {
      // 绘制当前行已积累的单词
      while (word_start < str) {
        OLED_PrintASCIIChar(x0, y0, *word_start, font, color);
        x0 += font->w;
        word_start++;
      }
      x0 = x;
      y0 += font->h;
      str++;
      word_start = str;
      word_width = 0;
      continue;
    }
    
    // 检测单词边界（空格或标点） 需要包含ctype.h头文件以使用ispunct()
    // 将char转换为unsigned char后再传递给ispunct()，避免有符号字符导致的警告
    if (*str == ' ' || *str == '\t' || ispunct((unsigned char)*str)) {
      // 检查单词是否超出边界
      if (x0 + word_width > max_x) {
        x0 = x;
        y0 += font->h;
      }
      // 绘制整个单词
      while (word_start <= str) {
        OLED_PrintASCIIChar(x0, y0, *word_start, font, color);
        x0 += font->w;
        word_start++;
      }
      word_width = 0;
    } else {
      word_width += font->w;
    }
    
    // 检查当前行剩余空间是否足够
    if (x0 + word_width > max_x) {
      x0 = x;
      y0 += font->h;
    }
    
    str++;
  }
  
  // 绘制最后一个单词
  while (word_start < str) {
    OLED_PrintASCIIChar(x0, y0, *word_start, font, color);
    x0 += font->w;
    word_start++;
  }
}

/**
 * @brief 绘制一个uint8_t数字，输入数字自动转字符串
 * @param x 起始点横坐标
 * @param y 起始点纵坐标
 * @param num 要显示的数字指针
 * @param font 字体
 * @param color 颜色
 */
void OLED_Print_u8_Num(uint8_t x, uint8_t y, uint8_t *num, const ASCIIFont *font, OLED_ColorMode color)
{
  char str[4];
  sprintf(str, "%d", (int)*num);
  
  uint8_t x0 = x_line(x);
  char *p = str; // 使用临时指针指向字符数组
  while (*p) {  // 遍历临时指针
    OLED_PrintASCIIChar(x0, y_line(y), *p, font, color);
    x0 += font->w;
    p++;         // 自增临时指针，而不是数组名
  }
}

/**
 * @brief 绘制一个uint8_t数字，输入数字自动转字符串
 * @param x 起始点横坐标
 * @param y 起始点纵坐标
 * @param num 要显示的数字指针
 * @param font 字体
 * @param color 颜色
 */
void OLED_Print_u16_Num(uint8_t x, uint8_t y, uint16_t *num, const ASCIIFont *font, OLED_ColorMode color)
{
  char str[6];
  sprintf(str, "%d", (int)*num);
  
  uint8_t x0 = x_line(x);
  char *p = str; // 使用临时指针指向字符数组
  while (*p) {  // 遍历临时指针
    OLED_PrintASCIIChar(x0, y_line(y), *p, font, color);
    x0 += font->w;
    p++;         // 自增临时指针，而不是数组名
  }
}

/**
 * @brief 绘制一个浮点数
 * @param x 起始点横坐标
 * @param y 起始点纵坐标
 * @param num 要显示的浮点数
 * @param precision 小数点后保留位数
 * @param font 字体
 * @param color 颜色
 */
void OLED_PrintFloat(uint8_t x, uint8_t y, double num, uint8_t precision, const ASCIIFont *font, OLED_ColorMode color)
{
    char str[32];
    int integer_part;
    int fractional_part;
    int index = 0;
    
    // 处理负数
    if (num < 0) {
        str[index++] = '-';
        num = -num;
    }
    
    // 提取整数部分
    integer_part = (int)num;
    
    // 提取小数部分（四舍五入）
    double fractional = num - integer_part;
    double multiplier = 1.0;
    for (uint8_t i = 0; i < precision; i++) {
        multiplier *= 10.0;
    }
    fractional_part = (int)(fractional * multiplier + 0.5);
    
    // 处理整数部分
    if (integer_part == 0) {
        str[index++] = '0';
    } else {
        // 处理整数部分的各位数字
        char temp[10]; // 临时存储整数部分的数字
        int temp_index = 0;
        
        // 将整数转换为字符串（逆序）
        while (integer_part > 0) {
            temp[temp_index++] = (integer_part % 10) + '0';
            integer_part /= 10;
        }
        
        // 反转字符串得到正确的顺序
        for (int i = temp_index - 1; i >= 0; i--) {
            str[index++] = temp[i];
        }
    }
    
    // 处理小数部分
    if (precision > 0) {
        str[index++] = '.';
        
        // 补零确保小数部分有足够的位数
        int temp = fractional_part;
        int actual_digits = 0;
        
        // 计算实际位数
        int temp2 = temp;
        while (temp2 > 0) {
            actual_digits++;
            temp2 /= 10;
        }
        
        // 补前导零
        for (uint8_t i = 0; i < precision - actual_digits; i++) {
            str[index++] = '0';
        }
        
        // 处理小数部分的各位数字
        char temp_frac[10];
        int temp_index = 0;
        
        // 将小数转换为字符串（逆序）
        while (temp > 0 && temp_index < precision) {
            temp_frac[temp_index++] = (temp % 10) + '0';
            temp /= 10;
        }
        
        // 反转字符串得到正确的顺序
        for (int i = temp_index - 1; i >= 0; i--) {
            str[index++] = temp_frac[i];
        }
    }
    
    str[index] = '\0'; // 结束符
    
    // 显示字符串
    uint8_t x0 = x_line(x);
    char *p = str;
    while (*p) {
        OLED_PrintASCIIChar(x0, y_line(y), *p, font, color);
        x0 += font->w;
        p++;
    }
}
