#include "stm32f10x.h"
#include "Delay.h"
#include "LED.h"
#include "Key.h"
#include "Servo.h"
#include <math.h>

#define PI 3.1415926f

int main(void)
{
	LED_Init();
	Key_Init();
	Servo_Init();

	while (1)
	{
		if (Key_GetNum() == 1)
		{
			LED_ON();
			for (int swing = 0; swing < 3; swing++)
			{
				// 幅度范围为40~160度
				for (int t = 0; t <= 100; t++)
				{
					float theta = 40 + 120 * 0.5f * (1 - cosf(t * PI / 100.0f));
					Servo_SetAngle(theta);
					Delay_ms(12);  //  不能再小了，否则会转不到位
				}
				Delay_ms(20);  //  等待执行到位
				for (int t = 0; t <= 100; t++)
				{
					float theta = 160 - 120 * 0.5f * (1 - cosf(t * PI / 100.0f));
					Servo_SetAngle(theta);
					Delay_ms(12);
				}
			}
			LED_OFF();
		}
	}
}
