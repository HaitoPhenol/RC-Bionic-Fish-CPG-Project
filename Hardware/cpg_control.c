#include "cpg_control.h"
#include <stdlib.h>

/*
x和y代表振荡器的状态变量; ω代表振荡器的频率; r、A代表振荡器的幅值; 
k为Hopf振荡器的收敛因子; 耦合项p(t)与q(t); 
*/

//时间值
double t1 = 5.0, t2 = 10.0, t3 = 15.0, t4 = 20.0, t5 = 25.0, t6 = 30.0, t7 = 35.0;

//static：限定变量只在本文件（.c 文件）内可见，外部无法访问，防止命名冲突。
//const：常量，初始化后不能修改。

static const double M_PI = 3.14159265358979323846;
static const double k = 15.0;   // 在Hopf振荡器中加入收敛因子k，k值越大，振荡器的收敛速度越快，是过渡较为剧烈，k值越小，振荡器的收敛速度越慢，但是过渡过程越平滑．
static const double s = 0.5;    // 占空比
static const double j1 = 0.5;   // 左侧鳍
static const double j2 = 0.5;   // 右侧鳍 
static const double j3 = 0.0;   // 尾鳍
static const double u = 100.0;  
static const double a = 100.0;  // 较大的常数
static const double w_sw = 3.14159265358979323846;  // 摆动相频率

// 初始状态,取较小的随机数 
void CPG_Init(CPG_State *state) {
    state->t = 0.0;
    for(int i = 0; i < 6; i++) {
        state->x[i] = 0.5 * (rand() / (double)RAND_MAX);
    }
}

// 定义游动步态的分段函数
static void get_b_values(double t, double *b1, double *b2) {
    if(t <= t1) {
        *b1 = 0; *b2 = 0;    // 直游
    } else if(t > t1 && t <= t2) {
        *b1 = 5; *b2 = 0;    // 下潜
    } else if(t > t2 && t <= t3) {
        *b1 = 0; *b2 = 0;    // 直游
    } else if(t > t3 && t <= t4) {
        *b1 = 0; *b2 = 5;    // 转向
    } else if(t > t4 && t <= t5) {
        *b1 = 0; *b2 = 0;    // 直游
    } else if(t > t5 && t <= t6) {
        *b1 = -5; *b2 = 0;   // 上浮
    } else if(t > t6 && t <= t7) {
        *b1 = 0; *b2 = 0;    // 直游
    } else {
        *b1 = 0; *b2 = 0;
    }
}

// 爬行CPG（2-12）和游动CPG（14-16）单元的频率
static double calculate_w(double x) {
    double w_st = w_sw * (1 - s) / s;   // 支撑相频率
    return w_st / (1 + exp(-a * x)) + w_sw / (1 + exp(a * x));
}

// ODE45解微分方程
static void cpg_ode(double t, const double x[6], double dx[6]) {
    double b1, b2;
    get_b_values(t, &b1, &b2);
    double w[3] = {
        calculate_w(x[1]),
        calculate_w(x[3]),
        calculate_w(x[5])
    };
    
    // r 代表振荡器的幅值，采用这种非线性函数的振荡器称为Hopf振荡器
    // r 是所有CPG单元之间的相位差矩阵，3*3矩阵，r(1,2)：表示CPG1与CPG2之间的相位差，r(1,2)=j1-j2
    double r[3][3] = {
        {2*M_PI*(j1-j1), 2*M_PI*(j1-j2), 2*M_PI*(j1-j3)},
        {2*M_PI*(j2-j1), 2*M_PI*(j2-j2), 2*M_PI*(j2-j3)},
        {2*M_PI*(j3-j1), 2*M_PI*(j3-j2), 2*M_PI*(j3-j3)}
    };
    
    // 计算耦合项p(t)与q(t)
    double p1 = cos(r[0][1])*x[2] - sin(r[0][1])*(x[3]-b1) + cos(r[0][2])*x[4] - sin(r[0][2])*(x[5]-b2);
    double q1 = sin(r[0][1])*x[2] + cos(r[0][1])*(x[3]-b1) + sin(r[0][2])*x[4] + cos(r[0][2])*(x[5]-b2);
    
    double p2 = cos(r[1][0])*x[0] - sin(r[1][0])*(x[1]-b1) + cos(r[1][2])*x[4] - sin(r[1][2])*(x[5]-b2);
    double q2 = sin(r[1][0])*x[0] + cos(r[1][0])*(x[1]-b1) + sin(r[1][2])*x[4] + cos(r[1][2])*(x[5]-b2);
    
    double p3 = cos(r[2][0])*x[0] - sin(r[2][0])*(x[1]-b1) + cos(r[2][1])*x[2] - sin(r[2][1])*(x[3]-b1);
    double q3 = sin(r[2][0])*x[0] + cos(r[2][0])*(x[1]-b1) + sin(r[2][1])*x[2] + cos(r[2][1])*(x[3]-b1);
    
    // 爬行CPG1
    dx[0] = k*(u - (sqrt(x[0]*x[0] + (x[1]-b1)*(x[1]-b1)))*(sqrt(x[0]*x[0] + (x[1]-b1)*(x[1]-b1))))*x[0] - w[0]*(x[1]-b1) + p1;
    dx[1] = k*(u - (sqrt(x[0]*x[0] + (x[1]-b1)*(x[1]-b1)))*(sqrt(x[0]*x[0] + (x[1]-b1)*(x[1]-b1))))*(x[1]-b1) + w[0]*x[0] + q1;
    
    // 爬行CPG2
    dx[2] = k*(u - (sqrt(x[2]*x[2] + (x[3]-b1)*(x[3]-b1)))*(sqrt(x[2]*x[2] + (x[3]-b1)*(x[3]-b1))))*x[2] - w[1]*(x[3]-b1) + p2;
    dx[3] = k*(u - (sqrt(x[2]*x[2] + (x[3]-b1)*(x[3]-b1)))*(sqrt(x[2]*x[2] + (x[3]-b1)*(x[3]-b1))))*(x[3]-b1) + w[1]*x[2] + q2;
    
    // 爬行CPG3
    dx[4] = k*(u - (sqrt(x[4]*x[4] + (x[5]-b2)*(x[5]-b2)))*(sqrt(x[4]*x[4] + (x[5]-b2)*(x[5]-b2))))*x[4] - w[2]*(x[5]-b2) + p3;
    dx[5] = k*(u - (sqrt(x[4]*x[4] + (x[5]-b2)*(x[5]-b2)))*(sqrt(x[4]*x[4] + (x[5]-b2)*(x[5]-b2))))*(x[5]-b2) + w[2]*x[4] + q3;
}

/*当你有一个结构体指针（比如 CPG_State *state），想访问它的成员（比如 t 或 x），就用 ->。
比如：state->t  等价于：(*state).t
意思是“取 state 指针指向的结构体里的 t 成员”。
如果你有结构体变量（不是指针），则用点号 .，比如 state.t。*/

// CPG状态更新，使用欧拉法积分
void CPG_Update(CPG_State *state, double dt) {
    double dx[6]; // 存储微分方程的结果
    cpg_ode(state->t, state->x, dx); // 计算当前状态的导数
    
    for(int i = 0; i < 6; i++) {
        state->x[i] += dx[i] * dt; // 用欧拉法更新状态变量
    }
    state->t += dt; // 更新时间
}

// CPG输出接口，将状态变量映射为左鳍、右鳍、尾鳍的输出
void CPG_GetOutput(const CPG_State *state, double *left_fin, double *right_fin, double *tail) {
    *left_fin = state->x[1];   // 左鳍输出
    *right_fin = state->x[3];  // 右鳍输出
    *tail = state->x[5];       // 尾鳍输出
}
