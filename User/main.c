#include "stm32f10x.h"
#include "Delay.h"
#include "LED.h"
#include "Key.h"
#include "Servo.h"
#include <math.h>

#define PI 3.1415926f

int main(void)
{
	LED_Init();     // 初始化LED
	Key_Init();     // 初始化按键
	Servo_Init();   // 初始化舵机

	while (1)
	{
		if (Key_GetNum() == 1)    // 检测到按键按下
		{
			LED_ON();             // 点亮LED，表示动作开始
			for (int swing = 0; swing < 3; swing++)   // 摆动3次
			{
				// 从80度平滑摆动到120度（正弦插值，仿生鱼尾动作）
				for (int t = 0; t <= 100; t++)
				{
					// theta为当前角度，t从0~100映射到0~PI，实现加减速效果
					float theta = 80.0f + 40 * 0.5f * (1 - cosf(t * PI / 100.0f));
					Servo_SetAngle(theta);   // 设置舵机角度
					Delay_ms(12);           // 控制单步速度
				}
				Delay_ms(10);  // 等待舵机到位

				// 从120度平滑摆回80度
				for (int t = 0; t <= 100; t++)
				{
					float theta = 120.0f - 40 * 0.5f * (1 - cosf(t * PI / 100.0f));
					Servo_SetAngle(theta);
					Delay_ms(12);
				}
				Delay_ms(10);  // 等待舵机到位
			}
			LED_OFF();    // 熄灭LED，表示动作结束
		}
	}
}
