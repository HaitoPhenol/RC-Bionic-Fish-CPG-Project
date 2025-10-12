#ifndef __SERVO_H__
#define __SERVO_H__

#include "main.h"

// 舵机ID枚举，直接映射到TIM_CHANNEL
typedef enum {
    SERVO_1 = TIM_CHANNEL_1, // PA6
    SERVO_2 = TIM_CHANNEL_2, // PA7
    SERVO_3 = TIM_CHANNEL_3, // PB0
    SERVO_4 = TIM_CHANNEL_4  // PB1
} SERVO_ID;

void Servo_Init(void);
void Servo_SetAngle(SERVO_ID servo_id, float Angle);

#endif
