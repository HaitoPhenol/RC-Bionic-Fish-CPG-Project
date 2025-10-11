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
#include "SBUS.h"

extern uint8_t uart_sbus_ok;//SBUS数据就绪(单次接收完毕) 全局变量

int main() {
    uint16_t chbuf[6]={0};//6个通道数据存储数组

    // 初始化硬件
    PWM_Init();
    OLED_Init();
    Servo_Init();
    uart_init(100000);

	CPGState state;
    initCPGState(&state);
    double dt = 0.01; // 时间步长
    while (1) {
        if(uart_sbus_ok){       // 数据就绪可以读取
			uart_sbus_ok=0;
			Save_6CH(chbuf);    // 使用Save_6CH函数向chbuf写入通道数据
			rxbuf_clear();      // 清空串口缓冲区

		
            // 更新CPG状态
            set_cpg_frequency(&state, chbuf[2]);    // 每次循环都设置频率
            set_cpg_bias(&state, chbuf[3]);
            // CPG更新舵机角度数据
            cpg_update(&state, dt);

            // 获取舵机角度数据
            double servo_angles[3];
            for (int i = 0; i < 3; i++) {
                servo_angles[i] = map_to_servo(state.y[i]);
            }

            // 显示舵机角度（取整显示），并设置舵机角度
            for (int i = 2; i < 5; i++) {
                // 舵机角度数据
                char servo_label[5];
                sprintf(servo_label, "S%d:", i - 1);
                OLED_ShowString(i, 1, servo_label);
                OLED_ShowNum   (i, 4, (int)servo_angles[i - 2], 3);
                Servo_SetAngle(i - 1, servo_angles[i - 2]);             //分别设置3个舵机
                Servo_SetAngle(i, map_to_range(55, 145, chbuf[1]));     // 单独设置侧鳍舵机
            }
                // 速度、偏置、侧鳍 数据
                OLED_ShowString(2, 8, "a:");
                OLED_ShowFloat(2, 10, state.omega[0], 3, 1);            // 显示3位整数1位小数
                OLED_ShowString(3, 8, "b:");
                OLED_ShowFloat(3, 10, state.bias[0], 3, 1);             // 显示3位整数1位小数
                OLED_ShowString(4, 8, "c:");
                OLED_ShowFloat(4, 10, map_to_range(55, 145, chbuf[1]), 3, 1);           // 转换为浮点数显示
        }
    }
}
