import math
import time
import os

# 尾部三舵机对应3个CPG振荡器（CPG1、CPG2、CPG3，对应原5、6、7关节）
class CPGState:
    def __init__(self):
        self.t = 0.0  # 时间
        self.x = [0.1, 0.1, 0.1]  # 状态变量x1-x3
        self.y = [0.1, 0.1, 0.1]  # 状态变量y1-y3（输出相关）
        # 核心参数（直游模式参考文档表2，倒游参考表2倒游相位差）
        self.omega = [1.5 * math.pi, 1.5 * math.pi, 1.5 * math.pi]  # 频率
        self.amplitude = [3.0, 3.5, 4.0]  # 幅值（文档直游模式参数）
        self.bias = [0.0, 0.0, 0.0]  # 偏置量（动态更新）
        self.k = 0.8  # 收敛因子（文档推荐值）
        # 耦合权重：[[h(i,i-1), h(i,i+1)], ...]（参考文档双向邻近耦合）
        self.coupling = [
            [0.0, 0.2],  # CPG1：无前置关节(h=0)，后向耦合h=0.2
            [1.0, 0.2],  # CPG2：前向耦合h=1，后向耦合h=0.2
            [1.0, 0.0]   # CPG3：前向耦合h=1，无后置关节(h=0)
        ]
        self.phase = [0.25 * math.pi, 0.25 * math.pi]  # 相位差（直游模式）


# 获取不同时间段的偏置量（模拟动作切换，参考文档转弯时偏置设置）
def get_bias_values(t):
    if t <= 5:
        return [0.0, 0.0, 0.0]  # 直游（无偏置）
    elif t <= 10:
        return [3.0, 3.0, 3.0]  # 转弯（加3单位偏置，参考文档表2转弯参数）
    elif t <= 15:
        return [0.0, 0.0, 0.0]  # 直游
    elif t <= 20:
        return [0.0, 0.0, 0.0]  # 倒游（通过相位差反转实现，参考表2倒游相位）
    else:
        return [0.0, 0.0, 0.0]  # 复位


# 计算CPG微分方程（核心算法，严格遵循文档公式）
def cpg_ode(state):
    dx = [0.0, 0.0, 0.0]
    dy = [0.0, 0.0, 0.0]
    bias = get_bias_values(state.t)  # 当前偏置量
    
    # 倒游时反转相位差（参考文档表2倒游相位差φ=-0.25π）
    phase = state.phase if state.t <= 15 else [-p for p in state.phase]
    
    # 遍历3个关节计算dx、dy（严格对应文档公式11）
    for i in range(3):
        x = state.x[i]
        y = state.y[i]
        omega = state.omega[i]
        A = state.amplitude[i]
        b = bias[i]
        k = state.k
        
        # 非线性项：A² - x² - (y - b)²（文档公式核心项）
        nonlinear = A**2 - x**2 - (y - b)** 2
        
        # 1. 计算耦合项p(t)和q(t)（参考文档公式10的双向邻近耦合）
        p = 0.0  # 前向耦合（来自i-1关节）
        q = 0.0  # 后向耦合（来自i+1关节）
        
        if i == 0:  # CPG1（无前置关节）
            # 后向耦合：仅受i+1=1关节影响
            if i + 1 < 3:
                h = state.coupling[i][1]  # h1,2=0.2
                phi = phase[0]  # φ1,2=0.25π
                x_next = state.x[i+1]
                y_next = state.y[i+1]
                b_next = bias[i+1]
                q = h * (x_next * math.sin(phi) + (y_next - b_next) * math.cos(phi))
        
        elif i == 2:  # CPG3（无后置关节）
            # 前向耦合：仅受i-1=1关节影响
            if i - 1 >= 0:
                h = state.coupling[i][0]  # h3,2=1.0
                phi = phase[1]  # φ3,2=0.25π（与φ2,3相等）
                x_prev = state.x[i-1]
                y_prev = state.y[i-1]
                b_prev = bias[i-1]
                p = h * (x_prev * math.cos(phi) + (y_prev - b_prev) * math.sin(phi))
        
        else:  # CPG2（有前置i-1=0和后置i+1=2关节）
            # 前向耦合：受i-1=0关节影响
            h_prev = state.coupling[i][0]  # h2,1=1.0
            phi_prev = phase[0]  # φ2,1=0.25π（与φ1,2相等）
            x_prev = state.x[i-1]
            y_prev = state.y[i-1]
            b_prev = bias[i-1]
            p = h_prev * (x_prev * math.cos(phi_prev) + (y_prev - b_prev) * math.sin(phi_prev))
            
            # 后向耦合：受i+1=2关节影响
            h_next = state.coupling[i][1]  # h2,3=0.2
            phi_next = phase[1]  # φ2,3=0.25π
            x_next = state.x[i+1]
            y_next = state.y[i+1]
            b_next = bias[i+1]
            q = h_next * (x_next * math.sin(phi_next) + (y_next - b_next) * math.cos(phi_next))
        
        # 2. 计算dx和dy（严格遵循文档公式11）
        dx[i] = -omega * (y - b) + k * x * nonlinear + p  # dx最后一项为p(t)
        dy[i] = omega * x + k * (y - b) * nonlinear + q    # dy最后一项为q(t)
    
    return dx, dy


# 欧拉法更新CPG状态（保持不变）
def cpg_update(state, dt):
    dx, dy = cpg_ode(state)
    for i in range(3):
        state.x[i] += dx[i] * dt
        state.y[i] += dy[i] * dt
    state.t += dt


# 将CPG输出映射为舵机角度（参考文档公式87-89的映射规则）
def map_to_servo(y_value):
    # 1. 限制CPG输出范围（文档式3-89：y∈[-9,9]）
    y_clamped = max(-9.0, min(9.0, y_value))
    # 2. 放大5倍（论文λ=10）→ [-90,90]
    angle_raw = y_clamped * 5
    # 3. 偏移到80~120（中间值100）
    return 100 + angle_raw

# 主程序（保持不变）
def main():
    state = CPGState()
    dt = 0.01  # 时间步长
    link_length = 8
    
    try:
        while True:
            cpg_update(state, dt)
            servo_angles = [map_to_servo(state.y[i]) for i in range(3)]
            
            os.system('cls' if os.name == 'nt' else 'clear')
            print("=== 尾部三舵机CPG数据模拟 ===")
            print(f"时间: {state.t:.2f}s")
            print("-------------------------")
            print(f"CPG1 x: {state.x[0]:.4f}, y: {state.y[0]:.4f}")
            print(f"CPG2 x: {state.x[1]:.4f}, y: {state.y[1]:.4f}")
            print(f"CPG3 x: {state.x[2]:.4f}, y: {state.y[2]:.4f}")
            print("-------------------------")
            print(f"频率: {[f'{w:.2f}' for w in state.omega]}")
            print(f"幅值: {state.amplitude}")
            print(f"偏置: {get_bias_values(state.t)}")
            print("-------------------------")
            print("动作阶段: 直游" if state.t <=5 else 
                  "动作阶段: 转弯" if state.t <=10 else 
                  "动作阶段: 直游" if state.t <=15 else 
                  "动作阶段: 倒游")
            
            time.sleep(0.05)
    except KeyboardInterrupt:
        print("\n程序已停止")


if __name__ == "__main__":
    main()