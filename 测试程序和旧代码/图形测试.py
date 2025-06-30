import numpy as np
import matplotlib.pyplot as plt

# 假设初始状态
x = np.zeros(6)
t = 0.0
dt = 0.01
T = 2.0
left_fin_list = []
right_fin_list = []
tail_list = []

def get_b_values(t):
    # 只模拟直游
    return 0, 0

def calculate_w(x):
    w_sw = np.pi
    s = 0.5
    a = 100.0
    w_st = w_sw * (1 - s) / s
    return w_st / (1 + np.exp(-a * x)) + w_sw / (1 + np.exp(a * x))

def cpg_ode(t, x):
    b1, b2 = get_b_values(t)
    w = [calculate_w(x[1]), calculate_w(x[3]), calculate_w(x[5])]
    M_PI = np.pi
    j1, j2, j3 = 0.5, 0.5, 0.0
    k = 15.0
    u = 100.0
    r = [
        [2*M_PI*(j1-j1), 2*M_PI*(j1-j2), 2*M_PI*(j1-j3)],
        [2*M_PI*(j2-j1), 2*M_PI*(j2-j2), 2*M_PI*(j2-j3)],
        [2*M_PI*(j3-j1), 2*M_PI*(j3-j2), 2*M_PI*(j3-j3)]
    ]
    p1 = np.cos(r[0][1])*x[2] - np.sin(r[0][1])*(x[3]-b1) + np.cos(r[0][2])*x[4] - np.sin(r[0][2])*(x[5]-b2)
    q1 = np.sin(r[0][1])*x[2] + np.cos(r[0][1])*(x[3]-b1) + np.sin(r[0][2])*x[4] + np.cos(r[0][2])*(x[5]-b2)
    p2 = np.cos(r[1][0])*x[0] - np.sin(r[1][0])*(x[1]-b1) + np.cos(r[1][2])*x[4] - np.sin(r[1][2])*(x[5]-b2)
    q2 = np.sin(r[1][0])*x[0] + np.cos(r[1][0])*(x[1]-b1) + np.sin(r[1][2])*x[4] + np.cos(r[1][2])*(x[5]-b2)
    p3 = np.cos(r[2][0])*x[0] - np.sin(r[2][0])*(x[1]-b1) + np.cos(r[2][1])*x[2] - np.sin(r[2][1])*(x[3]-b1)
    q3 = np.sin(r[2][0])*x[0] + np.cos(r[2][0])*(x[1]-b1) + np.sin(r[2][1])*x[2] + np.cos(r[2][1])*(x[3]-b1)
    dx = np.zeros(6)
    dx[0] = k*(u - (np.sqrt(x[0]**2 + (x[1]-b1)**2))**2)*x[0] - w[0]*(x[1]-b1) + p1
    dx[1] = k*(u - (np.sqrt(x[0]**2 + (x[1]-b1)**2))**2)*(x[1]-b1) + w[0]*x[0] + q1
    dx[2] = k*(u - (np.sqrt(x[2]**2 + (x[3]-b1)**2))**2)*x[2] - w[1]*(x[3]-b1) + p2
    dx[3] = k*(u - (np.sqrt(x[2]**2 + (x[3]-b1)**2))**2)*(x[3]-b1) + w[1]*x[2] + q2
    dx[4] = k*(u - (np.sqrt(x[4]**2 + (x[5]-b2)**2))**2)*x[4] - w[2]*(x[5]-b2) + p3
    dx[5] = k*(u - (np.sqrt(x[4]**2 + (x[5]-b2)**2))**2)*(x[5]-b2) + w[2]*x[4] + q3
    return dx

while t < T:
    dx = cpg_ode(t, x)
    x += dx * dt
    t += dt
    left_fin_list.append(x[1])
    right_fin_list.append(x[3])
    tail_list.append(x[5])

plt.plot(left_fin_list, label='left_fin')
plt.plot(right_fin_list, label='right_fin')
plt.plot(tail_list, label='tail')
plt.legend()
plt.title('CPG输出信号仿真')
plt.xlabel('步数')
plt.ylabel('输出值')
plt.show()