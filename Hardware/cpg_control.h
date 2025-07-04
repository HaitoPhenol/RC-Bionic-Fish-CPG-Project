#ifndef CPG_CONTROL_H
#define CPG_CONTROL_H

#include <stdint.h>
#include <math.h>

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

/**
 * @brief 初始化CPG状态结构体
 * @param state [输入/输出] 指向CPGState结构体的指针，函数将对其成员进行初始化。
 * @details 设置CPG的初始时间、状态变量、频率、幅值、偏置、收敛因子、耦合权重和相位差等参数。
 *          必须在CPG算法使用前调用一次。
 */
void initCPGState(CPGState *state);

/**
 * @brief 设置CPG相位方向（正向或反向）
 * @param state [输入/输出] 指向CPGState结构体的指针，函数将修改其phase成员。
 * @param reverse [输入] 整型，0表示正向（phase为正），1表示反向（phase为负）。
 * @details 用于根据游动方向（正游或倒游）动态切换CPG的相位差，影响振荡器输出的时序关系。
 */
void set_phase_direction(CPGState *state, int reverse);

/**
 * @brief 获取不同按钮对应的偏置量，并切换相位方向
 * @param KeyNum [输入] 当前按键值，决定当前运动阶段。
 * @param bias [输出] 指向长度为3的double数组，函数将填充各振荡器的偏置量。
 * @param state [输入/输出] 指向CPGState结构体的指针，函数可能会调用set_phase_direction修改其相位。
 * @details 根据按键KeyNum自动切换不同的运动模式（如直游、转弯、倒游等），并设置对应的偏置和相位方向。
 *          该函数通常在每次CPG状态更新时调用。
 */
void get_bias_values(uint8_t KeyNum, double *bias, CPGState *state);

/**
 * @brief 计算CPG微分方程（Hopf振荡器）
 * @param state [输入] 指向CPGState结构体的指针，提供当前的状态变量和参数。
 * @param dx [输出] 指向长度为3的double数组，函数将填充x变量的微分（dx/dt）。
 * @param dy [输出] 指向长度为3的double数组，函数将填充y变量的微分（dy/dt）。
 * @details 根据Hopf振荡器模型和耦合关系，计算每个振荡器的状态变量微分，用于后续数值积分。
 *          通常不直接调用，由cpg_update间接调用。
 */
void cpg_ode(CPGState *state, double *dx, double *dy);

/**
 * @brief 欧拉法更新CPG状态
 * @param state [输入/输出] 指向CPGState结构体的指针，函数将根据微分方程更新其状态变量和时间。
 * @param dt [输入] 时间步长（单位：秒），用于积分计算。
 * @details 使用欧拉法对CPG状态进行一步积分，推进系统演化。每个主循环周期应调用一次。
 */
void cpg_update(CPGState *state, double dt);

/**
 * @brief 将CPG输出y值映射为舵机角度
 * @param y_value [输入] CPG输出的y变量值（通常范围-9~9）。
 * @return double 映射后的舵机角度（范围约80~120，中心为100）。
 * @details 先将y_value限制在[-9,9]，再线性放大并偏移到舵机角度区间，便于直接用于舵机控制。
 */
double map_to_servo(double y_value);

// 新增：全局动作模式变量声明
extern uint8_t g_motion_mode;

#endif
