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

// 主程序
/**
 * @brief 主程序入口
 * @return int 通常返回0（但在嵌入式系统中不会返回）。
 * @details
 * 1. 初始化所有硬件外设（PWM、OLED、LED、按键、舵机等）。
 * 2. 初始化CPG状态结构体。
 * 3. 进入主循环：
 *    - 按固定步长dt调用cpg_update推进CPG状态。
 *    - 读取CPG输出，映射为舵机角度，控制3个舵机。
 *    - 根据当前时间段显示不同的运动模式（如直游、转弯、倒游等）及当前时间。
 *    - 实时显示每个舵机的角度和偏置值。
 *    - 持续运行，不会退出。
 */
int main() {
    // 初始化硬件
    PWM_Init();
    OLED_Init();
    LED_Init();
    Key_Init();
    Servo_Init();

	CPGState state;
    initCPGState(&state);
    double dt = 0.01; // 时间步长

    while (1) {
        cpg_update(&state, dt);
        double servo_angles[3];
        for (int i = 0; i < 3; i++) {
            servo_angles[i] = map_to_servo(state.y[i]);
        }

        if (state.t <= 10) {
            OLED_ShowString(1, 1, "Forward");
        } else if (state.t <= 20) {
            OLED_ShowString(1, 1, "Turn   ");
        } else if (state.t <= 30) {
            OLED_ShowString(1, 1, "Forward");
        } else if (state.t  <= 40) {
            OLED_ShowString(1, 1, "Back   "); // 倒游
        } else {
            OLED_ShowString(1, 1, "Reset  "); // 复位
        }
        OLED_ShowString(1, 9, "t:");
        OLED_ShowNum(1, 11, (int)state.t, 3);
        

        // 显示舵机角度（取整显示）
        double bias[3];
        get_bias_values(state.t, bias, &state);

        for (int i = 2; i < 5; i++) {
            char servo_label[5];
            sprintf(servo_label, "S%d:", i - 1);
            OLED_ShowString(i, 1, servo_label);
            OLED_ShowNum   (i, 4, (int)servo_angles[i - 2], 3);
            OLED_ShowString(i, 8, "b:");
            OLED_ShowNum   (i, 10, (int)bias[i - 2], 1);
            Servo_SetAngle(i - 1, servo_angles[i - 2]); // 新增：分别设置3个舵机
        }
    }
}
