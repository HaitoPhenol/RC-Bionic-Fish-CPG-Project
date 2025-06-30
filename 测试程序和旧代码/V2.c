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

/*
x和y代表振荡器的状态变量; ω代表振荡器的频率; r、A代表振荡器的幅值; 
k为Hopf振荡器的收敛因子; 耦合项p(t)与q(t); 
*/

// 尾部三舵机对应3个CPG振荡器（CPG5、CPG6、CPG7简化）
typedef struct {
    double t;  // 时间
    double x[3];  // 状态变量x1-x3（对应三个尾部关节）
    double y[3];  // 状态变量y1-y3（输出相关）
    // 核心参数（基于论文表2和C代码调整）
    double omega[3];  // 频率（直游）
    double amplitude[3];  // 幅值
    double bias[3];  // 偏置量
    double k;  // 收敛因子（论文推荐值）
    double coupling[2];  // 耦合权重（前向、后向）
    double phase[2];  // 相位差（关节间）
} CPGState;

static const double M_PI = 3.14159265358979323846;

// 获取不同时间段的偏置量（模拟动作切换）
void get_bias_values(double t, double *bias) {
    if (t <= 5) {
        bias[0] = 0.0;
        bias[1] = 0.0;
        bias[2] = 0.0;  // 直游
    } else if (t <= 10) {
        bias[0] = 2.0;
        bias[1] = 2.0;
        bias[2] = 2.0;  // 转弯（加偏置）
    } else if (t <= 15) {
        bias[0] = 0.0;
        bias[1] = 0.0;
        bias[2] = 0.0;  // 直游
    } else if (t <= 20) {
        bias[0] = 0.0;
        bias[1] = 0.0;
        bias[2] = 0.0;  // 倒游（通过相位差实现）
    } else {
        bias[0] = 0.0;
        bias[1] = 0.0;
        bias[2] = 0.0;  // 复位
    }
}

// 计算CPG微分方程（核心算法）
void cpg_ode(CPGState *state, double *dx, double *dy) {
    double bias[3];
    get_bias_values(state->t, bias);
    
    // 倒游时反转相位差
    double phase[2];
    if (state->t <= 15) {
        phase[0] = state->phase[0];
        phase[1] = state->phase[1];
    } else {
        phase[0] = -state->phase[0];
        phase[1] = -state->phase[1];
    }
    
    // 第一个尾部关节（CPG1）
    double r1 = hypot(state->x[0], state->y[0] - bias[0]);
    double coupling_x = state->coupling[1] * (state->x[1] * cos(phase[0]) + 
                                              (state->y[1] - bias[1]) * sin(phase[0]));
    dx[0] = state->k * (state->amplitude[0] * state->amplitude[0] - r1 * r1) * state->x[0] - state->omega[0] * (state->y[0] - bias[0]) + coupling_x;
    dy[0] = state->k * (state->amplitude[0] * state->amplitude[0] - r1 * r1) * (state->y[0] - bias[0]) + state->omega[0] * state->x[0];
    
    // 第二个尾部关节（CPG2）
    double r2 = hypot(state->x[1], state->y[1] - bias[1]);
    double coupling_x_prev = state->coupling[0] * (state->x[0] * cos(-phase[0]) + 
                                                   (state->y[0] - bias[0]) * sin(-phase[0]));
    double coupling_x_next = state->coupling[1] * (state->x[2] * cos(phase[1]) + 
                                                   (state->y[2] - bias[2]) * sin(phase[1]));
    dx[1] = state->k * (state->amplitude[1] * state->amplitude[1] - r2 * r2) * state->x[1] - state->omega[1] * (state->y[1] - bias[1]) + coupling_x_prev + coupling_x_next;
    dy[1] = state->k * (state->amplitude[1] * state->amplitude[1] - r2 * r2) * (state->y[1] - bias[1]) + state->omega[1] * state->x[1];
    
    // 第三个尾部关节（CPG3）
    double r3 = hypot(state->x[2], state->y[2] - bias[2]);
    double coupling_x_prev2 = state->coupling[0] * (state->x[1] * cos(-phase[1]) + 
                                                    (state->y[1] - bias[1]) * sin(-phase[1]));
    dx[2] = state->k * (state->amplitude[2] * state->amplitude[2] - r3 * r3) * state->x[2] - state->omega[2] * (state->y[2] - bias[2]) + coupling_x_prev2;
    dy[2] = state->k * (state->amplitude[2] * state->amplitude[2] - r3 * r3) * (state->y[2] - bias[2]) + state->omega[2] * state->x[2];
}

// 欧拉法更新CPG状态
void cpg_update(CPGState *state, double dt) {
    double dx[3], dy[3];
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
    // 2. 放大10倍（论文λ=10）→ [-90,90]
    double angle_raw = y_clamped * 10;
    // 3. 偏移到80~120（中间值100）
    return 100 + angle_raw;
}

// 主程序
int main(void) {
    // 初始化硬件
    PWM_Init();
    OLED_Init();
    LED_Init();
    Key_Init();
    Servo_Init();

    CPGState state;
    state.t = 0.0;
    for (int i = 0; i < 3; i++) {
        state.x[i] = 0.1;
        state.y[i] = 0.1;
        state.omega[i] = 1.5 * M_PI;
        state.amplitude[i] = i == 0? 3.0 : (i == 1? 3.5 : 4.0);
        state.bias[i] = 0.0;
    }
    state.k = 0.8;
    state.coupling[0] = 1.0;
    state.coupling[1] = 0.2;
    state.phase[0] = 0.25 * M_PI;
    state.phase[1] = 0.25 * M_PI;

    double dt = 0.01;  // 时间步长
   
    while(1) {
        // 更新CPG状态
        cpg_update(&state, dt);
        OLED_Clear();
        // 计算舵机角度
        double servo_angles[3];
        for (int i = 0; i < 3; i++) {
            servo_angles[i] = map_to_servo(state.y[i]);
        }
        
        // 显示舵机角度（取整显示）
        OLED_ShowString(2, 1, "S1:");
        OLED_ShowNum(2, 4, (int)servo_angles[0], 3);
        OLED_ShowString(3, 1, "S2:");
        OLED_ShowNum(3, 4, (int)servo_angles[1], 3);
        OLED_ShowString(4, 1, "S3:");
        OLED_ShowNum(4, 4, (int)servo_angles[2], 3);

        
    }
}
