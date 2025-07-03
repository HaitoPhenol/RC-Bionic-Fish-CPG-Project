#include "cpg_control.h"
#include <stdlib.h>

/*
x和y代表振荡器的状态变量; ω代表振荡器的频率; r、A代表振荡器的幅值; 
k为Hopf振荡器的收敛因子; 耦合项p(t)与q(t); 
*/

//static：限定变量只在本文件（.c 文件）内可见，外部无法访问，防止命名冲突。
//const：常量，初始化后不能修改。
static const double M_PI = 3.14159265358979323846;

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
    // 耦合权重初始化
    state->coupling[0][0] = 0.0; state->coupling[0][1] = 0.2;
    state->coupling[1][0] = 1.0; state->coupling[1][1] = 0.2;
    state->coupling[2][0] = 1.0; state->coupling[2][1] = 0.0;
    // 相位差初始化
    state->phase[0] = 0.25 * M_PI;
    state->phase[1] = 0.25 * M_PI;
}

// 设置相位方向（正向或反向）
void set_phase_direction(CPGState *state, int reverse) {
    if (reverse) {
        state->phase[0] = -0.25 * M_PI;
        state->phase[1] = -0.25 * M_PI;
    } else {
        state->phase[0] = 0.25 * M_PI;
        state->phase[1] = 0.25 * M_PI;
    }
}

// 获取不同时间段的偏置量，并切换相位方向
void get_bias_values(double t, double *bias, CPGState *state) {
    if (t <= 10) {
        bias[0] = 0.0; bias[1] = 0.0; bias[2] = 0.0; // 直游（无偏置）
        set_phase_direction(state, 0); // 正向
    } else if (t <= 20) {
        bias[0] = 3.0; bias[1] = 3.0; bias[2] = 3.0; // 转弯（加3单位偏置）
        set_phase_direction(state, 0); // 正向
    } else if (t <= 30) {
        bias[0] = 0.0; bias[1] = 0.0; bias[2] = 0.0; // 直游
        set_phase_direction(state, 0); // 正向
    } else if (t <= 40) {
        bias[0] = 0.0; bias[1] = 0.0; bias[2] = 0.0; // 倒游
        set_phase_direction(state, 1); // 反向
    } else {
        bias[0] = 0.0; bias[1] = 0.0; bias[2] = 0.0; // 复位
        set_phase_direction(state, 0); // 正向
    }
}

// 计算CPG微分方程
void cpg_ode(CPGState *state, double *dx, double *dy) {
    double bias[3];
    get_bias_values(state->t, bias, state);

    double phase[2];
    phase[0] = state->phase[0];
    phase[1] = state->phase[1];

    for (int i = 0; i < 3; i++) {
        double x = state->x[i];
        double y = state->y[i];
        double omega = state->omega[i];
        double A = state->amplitude[i];
        double b = bias[i];
        double k = state->k;

        // Hopf振荡器的非线性项
        double nonlinear = A * A - x * x - (y - b) * (y - b);

        double p = 0.0;
        double q = 0.0;

        // 计算耦合项
        if (i == 0) {
            // 第一个振荡器只与下一个耦合
            if (i + 1 < 3) {
                double h = state->coupling[i][1];
                double phi = phase[0];
                double x_next = state->x[i + 1];
                double y_next = state->y[i + 1];
                double b_next = bias[i + 1];
                q = h * (x_next * sin(phi) + (y_next - b_next) * cos(phi));
            }
        } else if (i == 2) {
            // 最后一个振荡器只与前一个耦合
            if (i - 1 >= 0) {
                double h = state->coupling[i][0];
                double phi = phase[1];
                double x_prev = state->x[i - 1];
                double y_prev = state->y[i - 1];
                double b_prev = bias[i - 1];
                p = h * (x_prev * cos(phi) + (y_prev - b_prev) * sin(phi));
            }
        } else {
            // 中间振荡器与前后都耦合
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

        // Hopf振荡器微分方程
        dx[i] = -omega * (y - b) + k * x * nonlinear + p;
        dy[i] = omega * x + k * (y - b) * nonlinear + q;
    }
}

// 欧拉法更新CPG状态
void cpg_update(CPGState *state, double dt) {
    double dx[3];
    double dy[3];
    cpg_ode(state, dx, dy);

    // 状态变量按欧拉法积分
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
