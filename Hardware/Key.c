#include "stm32f10x.h"                  // Device header
#include "Delay.h"

/**
  * 函    数：按键初始化
  * 参    数：无
  * 返 回 值：无
  */
void Key_Init(void)
{
	/*开启时钟*/
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);		//开启GPIOB的时钟
	
	/*GPIO初始化*/
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_10 | GPIO_Pin_11 | GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14; // 增加PB13、PB14
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);						//初始化所有按键引脚
}

/**
  * 函    数：按键获取键码
  * 参    数：无
  * 返 回 值：按下按键的键码值，范围：0~2，返回0代表没有按键按下，对应按键如下：
  *           1 - 按键1（PB1）	右转动作
  *           2 - 按键2（PB10）	前进动作
  * 		  3 - 按键3（PB11）	左转动作
  * 		  4 - 按键4（PB0）	后退动作
  * 		  5 - 按键5（PB12）	停止并回到初始位置
  * 		  6 - 按键6（PB13）	加速
  * 		  7 - 按键7（PB14）	减速
  * 注意事项：此函数是非阻塞式操作，当按键按下时，函数会输出一次返回值，然后停止输出，直到下一次按下。
  */
uint8_t Key_GetNum(void)
{
	static uint8_t key_up = 1; // 按键松开标志
	uint8_t KeyNum = 0;

	if (key_up && (
		GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_0) == 0 ||
		GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_1) == 0 ||
		GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_10) == 0 ||
		GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_11) == 0 ||
		GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_12) == 0 ||
		GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_13) == 0 ||
		GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_14) == 0
	)) {
		Delay_ms(20); // 消抖
		key_up = 0;
		if (GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_1) == 0) {
			KeyNum = 1;
		} else if (GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_10) == 0) {
			KeyNum = 2;
		} else if (GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_11) == 0) {
			KeyNum = 3;
		} else if (GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_0) == 0) {
			KeyNum = 4;
		} else if (GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_12) == 0) {
			KeyNum = 5;
		} else if (GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_13) == 0) {
			KeyNum = 6; // 加速
		} else if (GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_14) == 0) {
			KeyNum = 7; // 减速
		}
	} else if (
		GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_0) == 1 &&
		GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_1) == 1 &&
		GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_10) == 1 &&
		GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_11) == 1 &&
		GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_12) == 1 &&
		GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_13) == 1 &&
		GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_14) == 1
	) {
		key_up = 1; // 所有按键都松开
	}
	return KeyNum;
}
