#include "stm32f10x.h"
#include "Delay.h"
#include "LED.h"
#include "Key.h"
#include "Servo.h"

int main(void)
{
	LED_Init();
	Key_Init();
	Servo_Init();

	float angle = 0; // 当前角度

	while (1)
	{
		if (Key_GetNum() == 1)
		{
			LED_ON();
			angle += 30;
			if (angle > 180) angle = 0; // 超过180度回到0
			Servo_SetAngle(angle);
			Delay_ms(500);
			LED_OFF();
		}
	}
}
