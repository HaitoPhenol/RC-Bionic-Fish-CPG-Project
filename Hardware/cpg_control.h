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

void initCPGState(CPGState *state);
void set_phase_direction(CPGState *state, int reverse);
void get_bias_values(double t, double *bias);
void cpg_ode(CPGState *state, double *dx, double *dy);
void cpg_update(CPGState *state, double dt);
double map_to_servo(double y_value);

#endif
