#include "stm32f10x.h"
#include "cpg_control.h"
#include "Delay.h"
#include "LED.h"
#include "OLED.h"
#include "Key.h"
#include "Servo.h"
#include "PWM.h"
#include <math.h>
#include <stdio.h>
#include <math.h>
#include <stdlib.h>

static const double M_PI = 3.14159265358979323846;

// 定义CPG状态结构体
typedef struct {
    double t;               // 时间
    double x[3];            // 状态变量x1-x3
    double y[3];            // 状态变量y1-y3（输出相关）
    double omega[5];        // 频率
    double amplitude[3];    // 幅值
    double bias[3];         // 偏置量
    double k;               // 收敛因子
    double coupling[3][2];  // 耦合权重
    double phase[2];        // 相位差
} CPGState;

// 初始化CPG状态
void initCPGState(CPGState *state) {
    state->t = 0.0;
    for (int i = 0; i < 3; i++) {
        state->x[i] = 0.1;
        state->y[i] = 0.1;
        state->omega[i] = 1.5 * M_PI;
        state->amplitude[i] = (i == 0) ? 3.0 : (i == 1) ? 3.5 : 4.0;
        state->bias[i] = 0.0;
    }
    state->k = 0.8;
    state->coupling[0][0] = 0.0; state->coupling[0][1] = 0.2;
    state->coupling[1][0] = 1.0; state->coupling[1][1] = 0.2;
    state->coupling[2][0] = 1.0; state->coupling[2][1] = 0.0;
    state->phase[0] = 0.25 * M_PI;
    state->phase[1] = 0.25 * M_PI;
}

// 获取不同时间段的偏置量
void get_bias_values(double t, double *bias) {
    if (t <= 10) {
        bias[0] = 0.0; bias[1] = 0.0; bias[2] = 0.0; // 直游（无偏置）
    } else if (t <= 20) {
        bias[0] = 3.0; bias[1] = 3.0; bias[2] = 3.0; // 转弯（加3单位偏置）
    } else if (t <= 30) {
        bias[0] = 0.0; bias[1] = 0.0; bias[2] = 0.0; // 直游
    } else if (t <= 40) {
        bias[0] = 0.0; bias[1] = 0.0; bias[2] = 0.0; // 倒游
    } else {
        bias[0] = 0.0; bias[1] = 0.0; bias[2] = 0.0; // 复位
    }
}

// 计算CPG微分方程
void cpg_ode(CPGState *state, double *dx, double *dy) {
    double bias[3];
    get_bias_values(state->t, bias);

    double phase[2];
    if (state->t <= 15) {
        phase[0] = state->phase[0];
        phase[1] = state->phase[1];
    } else {
        phase[0] = -state->phase[0];
        phase[1] = -state->phase[1];
    }

    for (int i = 0; i < 3; i++) {
        double x = state->x[i];
        double y = state->y[i];
        double omega = state->omega[i];
        double A = state->amplitude[i];
        double b = bias[i];
        double k = state->k;

        double nonlinear = A * A - x * x - (y - b) * (y - b);

        double p = 0.0;
        double q = 0.0;

        if (i == 0) {
            if (i + 1 < 3) {
                double h = state->coupling[i][1];
                double phi = phase[0];
                double x_next = state->x[i + 1];
                double y_next = state->y[i + 1];
                double b_next = bias[i + 1];
                q = h * (x_next * sin(phi) + (y_next - b_next) * cos(phi));
            }
        } else if (i == 2) {
            if (i - 1 >= 0) {
                double h = state->coupling[i][0];
                double phi = phase[1];
                double x_prev = state->x[i - 1];
                double y_prev = state->y[i - 1];
                double b_prev = bias[i - 1];
                p = h * (x_prev * cos(phi) + (y_prev - b_prev) * sin(phi));
            }
        } else {
            double h_prev = state->coupling[i][0];
            double phi_prev = phase[0];
            double x_prev = state->x[i - 1];
            double y_prev = state->y[i - 1];
            double b_prev = bias[i - 1];
            p = h_prev * (x_prev * cos(phi_prev) + (y_prev - b_prev) * sin(phi_prev));

            double h_next = state->coupling[i][1];
            double phi_next = phase[1];
            double x_next = state->x[i + 1];
            double y_next = state->y[i + 1];
            double b_next = bias[i + 1];
            q = h_next * (x_next * sin(phi_next) + (y_next - b_next) * cos(phi_next));
        }

        dx[i] = -omega * (y - b) + k * x * nonlinear + p;
        dy[i] = omega * x + k * (y - b) * nonlinear + q;
    }
}

// 欧拉法更新CPG状态
void cpg_update(CPGState *state, double dt) {
    double dx[3];
    double dy[3];
    cpg_ode(state, dx, dy);

    for (int i = 0; i < 3; i++) {
        state->x[i] += dx[i] * dt;
        state->y[i] += dy[i] * dt;
    }
    state->t += dt;
}

// 将CPG输出映射为舵机角度（80~120范围）
double map_to_servo(double y_value) {
    // 1. 限制CPG输出范围（论文式3-89：y∈[-9,9]）
    double y_clamped = (y_value < -9.0)? -9.0 : (y_value > 9.0)? 9.0 : y_value;
    // 2. 放大5倍（论文λ=10）→ [-90,90]
    double angle_raw = y_clamped * 5;
    // 3. 偏移到80~120（中间值100）
    return 100 + angle_raw;
}

// 主程序
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
        get_bias_values(state.t, bias);

        OLED_ShowString(2, 1, "S1:");
        OLED_ShowNum   (2, 4, (int)servo_angles[0], 3);
        OLED_ShowString(2, 8, "b:");
        OLED_ShowNum   (2, 10, (int)bias[0], 1);

        OLED_ShowString(3, 1, "S2:");
        OLED_ShowNum   (3, 4, (int)servo_angles[1], 3);
        OLED_ShowString(3, 8, "b:");
        OLED_ShowNum   (3, 10, (int)bias[1], 1);

        OLED_ShowString(4, 1, "S3:");
        OLED_ShowNum   (4, 4, (int)servo_angles[2], 3);
        OLED_ShowString(4, 8, "b:");
        OLED_ShowNum   (4, 10, (int)bias[2], 1);

        Servo_SetAngle(1, servo_angles[0]); // 新增：分别设置3个舵机
        Servo_SetAngle(2, servo_angles[1]);
        Servo_SetAngle(3, servo_angles[2]);
        //Delay_ms(10); // 增加延时，避免舵机卡顿，10ms~20ms均可
    }
}
