#include "cpg_control.h"

/*
x和y代表振荡器的状态变量; ω代表振荡器的频率; r、A代表振荡器的幅值; 
k为Hopf振荡器的收敛因子; 耦合项p(t)与q(t); 
*/

//static：限定变量只在本文件（.c 文件）内可见，外部无法访问，防止命名冲突。
//const：常量，初始化后不能修改。
// double M_PI = 3.14159265358979323846; <math.h>头文件提供了标准的M_PI宏定义,无需将M_PI定义为一个变量

/**
 * @brief 初始化CPG状态结构体
 * @param state [输入/输出] 指向CPGState结构体的指针，函数将对其成员进行初始化。
 * @details 设置CPG的初始时间、状态变量、频率、幅值、偏置、收敛因子、耦合权重和相位差等参数。
 *          必须在CPG算法使用前调用一次。
 */
void initCPGState(CPGState *state) {
    state->t = 0.0;
    for (int i = 0; i < 3; i++) {
        state->x[i] = 0.1;
        state->y[i] = 0.1;
        state->omega[i] = 2.5 * M_PI; // 增大频率，摆动更快
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

/**
 * @brief 将SBUS通道值映射到目标范围，支持负值传递
 * @param sbus_value SBUS原始值（范围353-1695）
 * @param target_min 目标范围最小值
 * @param target_max 目标范围最大值
 * @return double 映射后的目标值
 * @details 中间值1024将映射到目标范围的中点
 */
double map_to_range(double target_min, double target_max, double sbus_value) {
    // SBUS原始范围参数
    const int16_t SBUS_MIN = 353;
    const int16_t SBUS_MAX = 1695;
    // const int16_t SBUS_MID = 1024;
    
    // 限制输入值在有效范围内
    if (sbus_value < SBUS_MIN) sbus_value = SBUS_MIN;
    if (sbus_value > SBUS_MAX) sbus_value = SBUS_MAX;
    
    return (sbus_value - SBUS_MIN) * (target_max - target_min) / (SBUS_MAX - SBUS_MIN) + target_min;
    // // 计算归一化比例（0-1）
    // double normalized = (double)(sbus_value - SBUS_MIN) / (SBUS_MAX - SBUS_MIN);
    // // 映射到目标范围（支持正负范围）
    // return target_min + normalized * (target_max - target_min);
}

/**
 * @brief 设置CPG相位方向（正向或反向）
 * @param state [输入/输出] 指向CPGState结构体的指针，函数将修改其phase成员。
 * @param reverse [输入] 整型，0表示正向（phase为正），1表示反向（phase为负）。
 * @details 用于根据游动方向（正游或倒游）动态切换CPG的相位差，影响振荡器输出的时序关系。
 */
void set_phase_direction(CPGState *state, int reverse) {
    if (reverse) {
        state->phase[0] = -0.25 * M_PI;
        state->phase[1] = -0.25 * M_PI;
    } else {
        state->phase[0] = 0.25 * M_PI;
        state->phase[1] = 0.25 * M_PI;
    }
}

/**
 * @brief 动态设置CPG振荡器频率
 * @param state [输入/输出] 指向CPGState结构体的指针
 * @param freq [输入] 新的频率（单位：Hz）
 *              使用SBUS通道2的数据，映射范围 (-5.5, 5.5)
 * @details 所有振荡器频率统一设置为 freq
 */
void set_cpg_frequency(CPGState *state, double freq) {
    for (int i = 0; i < 3; i++) {
        state->omega[i] = M_PI * fabs(map_to_range(-5.5, 5.5, freq)); // freq为Hz
    }
    // req为负时调转相位为后退状态
    if (freq < 0) {
        set_phase_direction(state, 1); // 假设存在后退相位常量
    } else {
        set_phase_direction(state, 0);  // 假设存在前进相位常量
    }
}

/**
 * @brief 获取不同时间段的偏置量，并切换相位方向
 * @param state [输入/输出] 指向CPGState结构体的指针，函数可能会调用set_phase_direction修改其相位。
 * @param bias [输出] 指向长度为3的double数组，函数将填充各振荡器的偏置量。
 *              使用SBUS通道3的数据，映射范围 (-3, 3)
 * @details 根据g_motion_mode切换不同的运动模式（如直游、转弯、倒游等），并设置对应的偏置和相位方向。
 */
void set_cpg_bias(CPGState *state, double bias) {
    for (int i = 0; i < 3; i++) {
    state->bias[i] = map_to_range(-3.0, 3.0, bias);
    }
}

/**
 * @brief 计算CPG微分方程（Hopf振荡器）
 * @param state [输入] 指向CPGState结构体的指针，提供当前的状态变量和参数。
 * @param dx [输出] 指向长度为3的double数组，函数将填充x变量的微分（dx/dt）。
 * @param dy [输出] 指向长度为3的double数组，函数将填充y变量的微分（dy/dt）。
 * @details 根据Hopf振荡器模型和耦合关系，计算每个振荡器的状态变量微分，用于后续数值积分。
 *          通常不直接调用，由cpg_update间接调用。
 */
void cpg_ode(CPGState *state, double *dx, double *dy) {
    // 准备一些必要的局部变量
    double bias[3];
    bias[0] = state->bias[0];
    bias[1] = state->bias[1];
    bias[2] = state->bias[2];
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

/**
 * @brief 欧拉法更新CPG状态
 * @param state [输入/输出] 指向CPGState结构体的指针，函数将根据微分方程更新其状态变量和时间。
 * @param dt [输入] 时间步长（单位：秒），用于积分计算。
 * @details 使用欧拉法对CPG状态进行一步积分，推进系统演化。每个主循环周期应调用一次。
 */
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

/**
 * @brief 将CPG输出y值映射为舵机角度
 * @param y_value [输入] CPG输出的y变量值（通常范围-9~9）。
 * @return double 映射后的舵机角度（范围约80~120，中心为100）。
 * @details 先将y_value限制在[-9,9]，再线性放大并偏移到舵机角度区间，便于直接用于舵机控制。
 */
double map_to_servo(double y_value) {
    // 1. 限制CPG输出范围（论文式3-89：y∈[-9,9]）
    double y_clamped = (y_value < -9.0)? -9.0 : (y_value > 9.0)? 9.0 : y_value;
    // 2. 放大5倍（论文λ=10）→ [-90,90]
    double angle_raw = y_clamped * 5;
    // 3. 偏移到80~120（中间值100）
    return 100 + angle_raw;
}


