#include "servo.h"
#include "tim.h"

void Servo_Init(void){
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1); // PA6
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2); // PA7
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3); // PB0
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4); // PB1
}

void Servo_SetAngle(SERVO_ID servo_id, float Angle)
{
    // 确保角度在有效范围内
    if (Angle < 0.0f) Angle = 0.0f;
    if (Angle > 180.0f) Angle = 180.0f;
    
    // 正确的舵机PWM脉宽计算：0.5ms~2.5ms对应0°~180°
    // 由于TIM3周期为2000，0.5ms对应50，2.5ms对应250
    uint16_t compare = (uint16_t)(Angle / 180.0f * 200 + 50);
    __HAL_TIM_SetCompare(&htim3, servo_id, compare);
}
// __HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_1, i);
// 设置第一个舵机（连接到TIM3通道1）到90度
// Servo_SetAngle(SERVO_1, 90.0f);
