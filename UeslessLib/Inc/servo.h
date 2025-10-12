#ifndef __SERVO_H__
#define __SERVO_H__

#include "main.h"

// 舵机ID枚举，直接映射到TIM_CHANNEL
typedef enum {
    SERVO_1 = TIM_CHANNEL_1,
    SERVO_2 = TIM_CHANNEL_2,
    SERVO_3 = TIM_CHANNEL_3,
    SERVO_4 = TIM_CHANNEL_4
} SERVO_ID;

void Servo_Init(void);
void Servo_SetAngle(uint8_t servo_id, float Angle);

#endif
