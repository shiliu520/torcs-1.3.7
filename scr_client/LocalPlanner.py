import numpy as np
import matplotlib.pyplot as plt
import matplotlib.font_manager as fm

def generate_trajectory(e, psi, L, order=3, k0=0, kL=0):
    """
    生成三次或五次多项式轨迹系数
    :param e:   横向偏差(到道路中心的距离,左正右负)
    :param psi: 车辆航向角与道路的夹角(弧度，朝左为正，朝右为负)
    :param L:   预瞄距离(纵向规划长度)
    :param order: 多项式阶数(3或5)
    :param k0:  起点曲率(二阶导数,仅order=5时生效)
    :param kL:  终点曲率(二阶导数,仅order=5时生效)
    :return: 多项式系数列表(按高阶到低阶排列)
    """
    if order == 3:
        # 三次多项式 y(x) = a x^3 + b x^2 + c x + d
        # 约束条件：起点位置、方向；终点位置、方向
        A = np.array([
            [L**3, L**2, L, 1],        # 终点位置 y(L) = 0
            [3*L**2, 2*L, 1, 0],       # 终点一阶导数 dy/dx(L) = 0
            [0, 0, 0, 1],              # 起点位置 y(0) = e
            [0, 0, 1, 0]               # 起点一阶导数 dy/dx(0) = tan(psi)
        ])
        b = np.array([0, 0, e, np.tan(psi)])
    elif order == 5:
        # 五次多项式 y(x) = a x^5 + b x^4 + c x^3 + d x^2 + e x + f
        # 约束条件：起点位置、方向、曲率；终点位置、方向、曲率
        A = np.array([
            [L**5, L**4, L**3, L**2, L, 1],    # 终点位置 y(L) = 0
            [5*L**4, 4*L**3, 3*L**2, 2*L, 1, 0],  # 终点一阶导数 dy/dx(L) = 0
            [20*L**3, 12*L**2, 6*L, 2, 0, 0],  # 终点二阶导数 d²y/dx²(L) = kL
            [0, 0, 0, 0, 0, 1],                # 起点位置 y(0) = e
            [0, 0, 0, 0, 1, 0],                # 起点一阶导数 dy/dx(0) = tan(psi)
            [0, 0, 0, 2, 0, 0]                 # 起点二阶导数 d²y/dx²(0) = k0
        ])
        b = np.array([0, 0, kL, e, np.tan(psi), k0])
    else:
        raise ValueError("Order must be 3 or 5.")

    coefficients = np.linalg.solve(A, b)
    return coefficients

def generate_trajectory_test():
    # 参数定义
    e = 0.5      # 横向偏差(米)
    psi = np.radians(5)  # 航向角偏差(弧度)，这里假设车辆朝右偏5度
    L = 10.0     # 预瞄距离(米)
    d_left = 3.0  # 左边界距离
    d_right = 3.0  # 右边界距离

    # 生成三次多项式轨迹
    coeff_3rd = generate_trajectory(e, -psi, L, order=3)
    x = np.linspace(0, L, 100)
    y_3rd = coeff_3rd[0]*x**3 + coeff_3rd[1]*x**2 + coeff_3rd[2]*x + coeff_3rd[3]

    # 生成五次多项式轨迹(假设起点和终点曲率为0)
    coeff_5th = generate_trajectory(e, -psi, L, order=5, k0=0, kL=0)
    y_5th = coeff_5th[0]*x**5 + coeff_5th[1]*x**4 + coeff_5th[2]*x**3 + \
            coeff_5th[3]*x**2 + coeff_5th[4]*x + coeff_5th[5]

    # 检查安全性
    safe_3rd = all(-d_right <= y <= d_left for y in y_3rd)
    safe_5th = all(-d_right <= y <= d_left for y in y_5th)
    print(f"三次轨迹安全性: {safe_3rd}, 五次轨迹安全性: {safe_5th}")

    # 可视化轨迹
    plt.figure(figsize=(10, 6))
    plt.plot(x, y_3rd, label='Cubic Polynomial Trajectory', color='blue')
    plt.plot(x, y_5th, label='Quintic Polynomial Trajectory', color='red')
    plt.axhline(y=d_left, color='green', linestyle='--', label='Left Boundary')
    plt.axhline(y=-d_right, color='orange', linestyle='--', label='Right Boundary')
    plt.xlabel('Longitudinal Distance (m)')
    plt.ylabel('Lateral Deviation (m)')
    plt.title('Visualization of Cubic and Quintic Polynomial Trajectories')
    plt.legend()
    plt.grid(True)
    plt.show()

def calculate_preview_distance(v, min_L=30.0, time_horizon=2.5):
    """动态计算预瞄距离: L = max(v * time_horizon, min_L)"""
    return max(v * time_horizon, min_L)

def generate_velocity_aware_trajectory(v, e_current, psi, target_lane = 0, lane_width=3.5, a_lat_max=2.0):
    """
    考虑车速的五次多项式换道轨迹
    :param v:          当前车速（m/s）
    :param e_current:  当前横向偏差
    :param psi:        当前航向角偏差（弧度）
    :param lane_width: 目标车道宽度（米）
    :param a_lat_max: 最大横向加速度（m/s²）
    :return: 多项式系数、预瞄距离L、时间序列t
    """
    # 动态预瞄距离
    L = calculate_preview_distance(v)

    # 曲率约束（基于车速和横向加速度）
    kappa_max = a_lat_max / (v**2 + 1e-3)  # 避免除零

    # 五次多项式约束（目标横向位置为车道中心）
    # target_lane: 当前车道为0，左侧车道为1，右侧车道为-1
    d_lane = lane_width * target_lane
    A = np.array([
        [L**5, L**4, L**3, L**2, L, 1],          # y(L) = d_lane
        [5*L**4, 4*L**3, 3*L**2, 2*L, 1, 0],     # dy/dx(L) = 0
        [20*L**3, 12*L**2, 6*L, 2, 0, 0],        # d²y/dx²(L) = 0 (目标车道为直线)
        [0, 0, 0, 0, 0, 1],                      # y(0) = e_current
        [0, 0, 0, 0, 1, 0],                      # dy/dx(0) = tan(psi)
        [0, 0, 0, 2, 0, 0]                       # d²y/dx²(0) = k0（当前曲率）
    ])
    b = np.array([d_lane, 0, 0, e_current, np.tan(psi), 0])  # 假设k0=0

    # 解五次多项式方程
    coeff = np.linalg.solve(A, b)

    # 时间参数化（假设匀速）
    t_total = L / v
    t_points = np.linspace(0, t_total, 100)

    return coeff, L, t_points

def validate_trajectory(coeff, L, v):
    x = np.linspace(0, L, 100)
    y = coeff[0]*x**5 + coeff[1]*x**4 + coeff[2]*x**3 + coeff[3]*x**2 + coeff[4]*x + coeff[5]

    # 计算一阶导数 dy/dx
    dy_dx = 5*coeff[0]*x**4 + 4*coeff[1]*x**3 + 3*coeff[2]*x**2 + 2*coeff[3]*x + coeff[4]

    # 计算二阶导数 d²y/dx²
    d2y_dx2 = 20*coeff[0]*x**3 + 12*coeff[1]*x**2 + 6*coeff[2]*x + 2*coeff[3]

    # 计算曲率 kappa 和横向加速度 a_lat
    kappa = d2y_dx2 / (1 + dy_dx**2)**1.5
    a_lat = v**2 * kappa

    # 检查约束
    max_kappa = np.max(np.abs(kappa))
    max_a_lat = np.max(np.abs(a_lat))
    print(f"最大曲率: {max_kappa:.3f} 1/m, 最大横向加速度: {max_a_lat:.3f} m/s²")

    return max_kappa, max_a_lat, y, dy_dx

def plot_trajectory(x, y, t_points, coeff, v, dy_dx):
    import matplotlib.pyplot as plt
    plt.figure(figsize=(12, 4))

    # 路径形状
    plt.subplot(1, 2, 1)
    plt.plot(x, y, color = 'k')
    plt.xlabel("Longitudinal Distance (m)")
    plt.ylabel("Lateral Distance (m)")
    plt.title("Trajectory Shape")

    # 横向加速度随时间变化
    plt.subplot(1, 2, 2)
    a_lat = v**2 * (20*coeff[0]*x**3 + 12*coeff[1]*x**2 + 6*coeff[2]*x + 2*coeff[3]) / (1 + dy_dx**2)**1.5
    plt.plot(t_points, a_lat, color = 'k')
    plt.xlabel("Time (s)")
    plt.ylabel("Lateral Acceleration (m/s²)")
    plt.title("Lateral Acceleration Profile")

    plt.tight_layout()
    plt.show()

# 示例使用
if __name__ == "__main__":
    generate_trajectory_test()

    v = 10  # 10 m/s = 36 km/h
    coeff, L, t = generate_velocity_aware_trajectory(v, e_current=0.5, psi=0, target_lane=0)
    max_kappa, max_a_lat, y, dy_dx = validate_trajectory(coeff, L, v)
    plot_trajectory(np.linspace(0, L, 100), y, t, coeff, v, dy_dx)
    a_lat_max = 2.0
    if max_a_lat > a_lat_max:
        # 方法1：降低车速重新规划
        v_new = np.sqrt(a_lat_max / max_kappa)
        print(f"需降速至 {v_new:.1f} m/s")

        # 方法2：在MPC中增加曲率惩罚项
        # 目标函数: min ∫(kappa^2) dx + 权重 * (a_lat - a_lat_max)^2