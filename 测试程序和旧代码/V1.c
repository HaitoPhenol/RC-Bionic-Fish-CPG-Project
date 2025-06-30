#include "stm32f10x.h"
#include "cpg_control.h"
#include "Delay.h"
#include "LED.h"
#include "OLED.h"
#include "Key.h"
#include "Servo.h"
#include "PWM.h"
#include <math.h>

static const double PI = 3.14159265358979323846;

void simple_forward(void); // 添加函数声明

CPG_State cpg_state;

int main(void) {
    PWM_Init();
    OLED_Init();
    LED_Init();
    Key_Init();
    Servo_Init();
    
    //HAL_Init();
    //SystemClock_Config();
    
    CPG_Init(&cpg_state);
    //TIM_HandleTypeDef htim;
    //MX_TIM_Init(&htim); 
    //HAL_TIM_Base_Start(&htim);
    
    while(1) {
        // 测试按键
        uint8_t keynum = Key_GetNum();
        OLED_ShowNum(4, 16, keynum, 1);
        // OLED_Clear();
        if 		 (keynum == 1){
			OLED_ShowString(1, 1, "                ");
            OLED_ShowString(1, 3, "Right");
        } else if(keynum == 2) {
			OLED_ShowString(1, 1, "                ");
            OLED_ShowString(1, 3, "Forward");
            while (1){
                simple_forward();  // 执行简单前进动作/* code */
                if(Key_GetNum() == 4){break;}
            }
        } else if(keynum == 3) {
			OLED_ShowString(1, 1, "                ");
            OLED_ShowString(1, 3, "Left");
        } else if(keynum == 4){
			OLED_ShowString(1, 1, "                ");
            LED_OFF();
            OLED_ShowString(1, 3, "Stop");
        }


        //uint32_t start_tick = HAL_GetTick();
        
        CPG_Update(&cpg_state, 0.01);
        
        double left_fin, right_fin, tail;
        CPG_GetOutput(&cpg_state, &left_fin, &right_fin, &tail);
        
        uint16_t left_pwm = (uint16_t)(1000 + left_fin * 100); 
        uint16_t right_pwm = (uint16_t)(1000 + right_fin * 100);
        uint16_t tail_pwm = (uint16_t)(1000 + tail * 100);

        //set_pwm_output(LEFT_FIN_CHANNEL, left_pwm);
        //set_pwm_output(RIGHT_FIN_CHANNEL, right_pwm);
        //set_pwm_output(TAIL_CHANNEL, tail_pwm);
        
        //while((HAL_GetTick() - start_tick) < 10) {}
    }
}

//void MX_TIM_Init(TIM_HandleTypeDef *htim) {}

void set_pwm_output(uint8_t channel, uint16_t value) {
    // 假设LEFT_FIN_CHANNEL=1, RIGHT_FIN_CHANNEL=2, TAIL_CHANNEL=3
    switch(channel) {
        case 1: // LEFT_FIN_CHANNEL
            TIM_SetCompare1(TIM2, value);
            break;
        case 2: // RIGHT_FIN_CHANNEL
            TIM_SetCompare2(TIM2, value);
            break;
        case 3: // TAIL_CHANNEL
            TIM_SetCompare3(TIM2, value);
            break;
        default:
            break;
    }
}

void simple_forward(void) {
    LED_ON();   // 点亮LED，表示动作开始
    // 从80度平滑摆动到120度（正弦插值，仿生鱼尾动作）
    for (int t = 0; t <= 100; t++)
    {
        // theta为当前角度，t从0~100映射到0~PI，实现加减速效果
        float theta = 80.0f + 40 * 0.5f * (1 - cosf(t * PI / 100.0f));
        Servo_SetAngle(theta);   // 设置舵机角度
        OLED_ShowNum(4, 3, (int)theta, 3); // 显示当前角度
        Delay_ms(12);           // 控制单步速度
    }
    Delay_ms(10);  // 等待舵机到位

    // 从120度平滑摆回80度
    for (int t = 0; t <= 100; t++)
    {
        float theta = 120.0f - 40 * 0.5f * (1 - cosf(t * PI / 100.0f));
        Servo_SetAngle(theta);
        OLED_ShowNum(4, 3, (int)theta, 3); // 显示当前角度
        Delay_ms(12);
    }
    Delay_ms(10);  // 等待舵机到位
    }
