#include "stm32f10x.h"     // STM32F10x 标准外设库头文件，包含芯片寄存器定义等
#include "cpg_control.h"   // CPG（中枢模式发生器）控制相关函数和结构体
#include "Delay.h"         // 延时函数
#include "LED.h"           // LED 控制相关函数
#include "OLED.h"          // OLED 显示屏控制相关函数
#include "Key.h"           // 按键控制相关函数
#include "Servo.h"         // 舵机控制相关函数
#include "PWM.h"           // PWM（脉宽调制）控制相关函数
#include <math.h>          // 数学库函数，如 sin、cos、pow 等
#include <stdio.h>         // 标准输入输出库，主要用于 sprintf 等格式化输出
// #include <stdlib.h>     // 标准库函数，如 malloc、free、rand 等（当前未使用）

uint8_t KeyNum = 0;          // 定义用于接收按键码的变量
double cpg_freq = 2.5;       // 当前CPG频率（Hz），初始值2.5

int main() {
    // 初始化硬件
    PWM_Init();
    OLED_Init();
    Key_Init();
    Servo_Init();

	CPGState state;
    initCPGState(&state);
    double dt = 0.01; // 时间步长
    // 初始状态g_motion_mode=5，鱼尾伸直
    while (1) {
        KeyNum = Key_GetNum(); // 获取按键键码
        // 处理加速/减速键
        if (KeyNum == 6) { // B13加速
            cpg_freq += 0.1;
            if (cpg_freq > 5.0) cpg_freq = 5.0;
        } else if (KeyNum == 7) { // B14减速
            cpg_freq -= 0.1;
            if (cpg_freq < 1.0) cpg_freq = 1.0;
        }
        set_cpg_frequency(&state, cpg_freq); // 每次循环都设置频率

        // 按钮控制运动状态
        // 1 - 按键1（PB1）     右转动作
        // 2 - 按键2（PB10）    前进动作
        // 3 - 按键3（PB11）    左转动作
        // 4 - 按键4（PB0）     后退动作
        // 5 - 按键5（PB12）    停止并回到初始位置
        if (KeyNum == 1) {
			OLED_ShowString(1, 1, "Right  "); // 1
        } else if (KeyNum == 2) {
            OLED_ShowString(1, 1, "Forward"); // 2
        } else if (KeyNum == 3) {
            OLED_ShowString(1, 1, "Left   "); // 3
        } else if (KeyNum == 4) {
            OLED_ShowString(1, 1, "Back   "); // 4 倒游
        } else if (KeyNum == 5) {
            OLED_ShowString(1, 1, "Reset  "); // 5 停止并回到初始位置
        }

        // 更新舵机角度数据
        cpg_update(&state, dt);
        double servo_angles[3];
        for (int i = 0; i < 3; i++) {
            servo_angles[i] = map_to_servo(state.y[i]);
        }

        // 显示当前速度（频率），在第一行右侧
        char freq_str[10];
        sprintf(freq_str, "V:%.1f", cpg_freq);
        OLED_ShowString(1, 12, freq_str);
        // 显示舵机角度（取整显示），并设置舵机角度
        double bias[3];
        get_bias_values(KeyNum, bias, &state); // bias由g_motion_mode决定
        for (int i = 2; i < 5; i++) {
            char servo_label[5];
            sprintf(servo_label, "S%d:", i - 1);
            OLED_ShowString(i, 1, servo_label);
            OLED_ShowNum   (i, 4, (int)servo_angles[i - 2], 3);
            OLED_ShowString(i, 8, "b:");
            OLED_ShowNum   (i, 10, (int)bias[i - 2], 1);
            Servo_SetAngle(i - 1, servo_angles[i - 2]); //分别设置3个舵机
        }
    }
}
