import math
import time
import os

# 尾部三舵机对应3个CPG振荡器（CPG5、CPG6、CPG7简化）
class CPGState:
    def __init__(self):
        self.t = 0.0  # 时间
        self.x = [0.1, 0.1, 0.1]  # 状态变量x1-x3（对应三个尾部关节）
        self.y = [0.1, 0.1, 0.1]  # 状态变量y1-y3（输出相关）
        # 核心参数（基于论文表2和C代码调整）
        self.omega = [1.5 * math.pi, 1.5 * math.pi, 1.5 * math.pi]  # 频率（直游）
        self.amplitude = [3.0, 3.5, 4.0]  # 幅值
        self.bias = [0.0, 0.0, 0.0]  # 偏置量
        self.k = 0.8  # 收敛因子（论文推荐值）
        self.coupling = [1.0, 0.2]  # 耦合权重（前向、后向）
        self.phase = [0.25 * math.pi, 0.25 * math.pi]  # 相位差（关节间）

# 获取不同时间段的偏置量（模拟动作切换）
def get_bias_values(t):
    if t <= 5:
        return [0.0, 0.0, 0.0]  # 直游
    elif t <= 10:
        return [2.0, 2.0, 2.0]  # 转弯（加偏置）
    elif t <= 15:
        return [0.0, 0.0, 0.0]  # 直游
    elif t <= 20:
        return [0.0, 0.0, 0.0]  # 倒游（通过相位差实现）
    else:
        return [0.0, 0.0, 0.0]  # 复位

# 计算CPG微分方程（核心算法）
def cpg_ode(state):
    dx = [0.0, 0.0, 0.0]
    dy = [0.0, 0.0, 0.0]
    bias = get_bias_values(state.t)
    
    # 倒游时反转相位差
    phase = state.phase if state.t <= 15 else [-p for p in state.phase]
    
    # 第一个尾部关节（CPG1）
    r1 = math.hypot(state.x[0], state.y[0] - bias[0])
    coupling_x = state.coupling[1] * (state.x[1] * math.cos(phase[0]) + 
                                     (state.y[1] - bias[1]) * math.sin(phase[0]))
    dx[0] = state.k * (state.amplitude[0]**2 - r1**2) * state.x[0] - state.omega[0] * (state.y[0] - bias[0]) + coupling_x
    dy[0] = state.k * (state.amplitude[0]**2 - r1**2) * (state.y[0] - bias[0]) + state.omega[0] * state.x[0]
    
    # 第二个尾部关节（CPG2）
    r2 = math.hypot(state.x[1], state.y[1] - bias[1])
    coupling_x_prev = state.coupling[0] * (state.x[0] * math.cos(-phase[0]) + 
                                          (state.y[0] - bias[0]) * math.sin(-phase[0]))
    coupling_x_next = state.coupling[1] * (state.x[2] * math.cos(phase[1]) + 
                                          (state.y[2] - bias[2]) * math.sin(phase[1]))
    dx[1] = state.k * (state.amplitude[1]**2 - r2**2) * state.x[1] - state.omega[1] * (state.y[1] - bias[1]) + coupling_x_prev + coupling_x_next
    dy[1] = state.k * (state.amplitude[1]**2 - r2**2) * (state.y[1] - bias[1]) + state.omega[1] * state.x[1]
    
    # 第三个尾部关节（CPG3）
    r3 = math.hypot(state.x[2], state.y[2] - bias[2])
    coupling_x_prev = state.coupling[0] * (state.x[1] * math.cos(-phase[1]) + 
                                          (state.y[1] - bias[1]) * math.sin(-phase[1]))
    dx[2] = state.k * (state.amplitude[2]**2 - r3**2) * state.x[2] - state.omega[2] * (state.y[2] - bias[2]) + coupling_x_prev
    dy[2] = state.k * (state.amplitude[2]**2 - r3**2) * (state.y[2] - bias[2]) + state.omega[2] * state.x[2]
    
    return dx, dy

# 欧拉法更新CPG状态
def cpg_update(state, dt):
    dx, dy = cpg_ode(state)
    for i in range(3):
        state.x[i] += dx[i] * dt
        state.y[i] += dy[i] * dt
    state.t += dt

# 将CPG输出映射为舵机角度（80~120范围）
def map_to_servo(y_value):
    # 1. 限制CPG输出范围（论文式3-89：y∈[-9,9]）
    y_clamped = max(-9.0, min(9.0, y_value))
    # 2. 放大10倍（论文λ=10）→ [-90,90]
    angle_raw = y_clamped * 10
    # 3. 偏移到80~120（中间值100）
    return 100 + angle_raw

def servo_bar(angle):
    # 角度范围80~120，映射到20字符宽度
    min_angle, max_angle = 80, 120
    width = 20
    pos = int((angle - min_angle) / (max_angle - min_angle) * (width - 1))
    bar = ' ' * pos + '|'
    return bar

# 主程序
def main():
    state = CPGState()
    dt = 0.01  # 时间步长
    try:
        while True:
            # 更新CPG状态
            cpg_update(state, dt)
            
            # 计算舵机角度
            servo_angles = [map_to_servo(state.y[i]) for i in range(3)]
            
            # 清屏并打印数据
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
            print(f"舵机1角度: {servo_angles[0]:.1f} {servo_bar(servo_angles[0])}")
            print(f"舵机2角度: {servo_angles[1]:.1f} {servo_bar(servo_angles[1])}")
            print(f"舵机3角度: {servo_angles[2]:.1f} {servo_bar(servo_angles[2])}")
            print("-------------------------")
            print("动作阶段: 直游" if state.t <=5 else 
                  "动作阶段: 转弯" if state.t <=10 else 
                  "动作阶段: 直游" if state.t <=15 else 
                  "动作阶段: 倒游")
            
            time.sleep(0.05)  # 控制显示帧率
    except KeyboardInterrupt:
        print("\n程序已停止")

if __name__ == "__main__":
    main()