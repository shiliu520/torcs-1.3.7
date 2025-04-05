import re
import numpy as np
import matplotlib.pyplot as plt
from scipy.interpolate import UnivariateSpline, CubicSpline, BSpline, splrep
from sklearn.linear_model import RANSACRegressor
from sklearn.preprocessing import PolynomialFeatures
from sklearn.pipeline import make_pipeline
from scipy.special import comb

# 设置 matplotlib 的字体
plt.rcParams['font.sans-serif'] = ['STSong']  # 使用黑体字体
plt.rcParams['axes.unicode_minus'] = False  # 解决负号显示问题

def fit_polynomial(x, y, x_fit_dense, degree=3):
    coeffs = np.polyfit(x, y, degree)

    poly = np.poly1d(coeffs)
    return poly(x_fit_dense)

def fit_spline(x, y, x_fit_dense, s=0.5):
    spline = CubicSpline(x, y)
    return spline(x_fit_dense)

def fit_ransac(x, y, x_fit_dense, degree=3):
    x_reshape = x.reshape(-1, 1)
    model = make_pipeline(
        PolynomialFeatures(degree),
        RANSACRegressor(random_state=42)
    )
    model.fit(x_reshape, y)
    return model.predict(x_fit_dense.reshape(-1, 1))

def fit_bezier(x, y, num_points=100):
    n = len(x)
    t = np.linspace(0, 1, num_points)
    bezier = np.zeros_like(t)
    for i in range(n):
        bezier += comb(n-1, i) * (1 - t)**(n-1 - i) * t**i * y[i]
    return x.min() + t*(x.max() - x.min()), bezier

def fit_bspline(x, y, x_fit_dense, k=3, s=0.1):
    tck = splrep(x, y, k=k, s=s)
    return BSpline(*tck)(x_fit_dense)


# 道路边界数据
# road_info = "angle: 0.000210, trackPos: -0.333363, track_0: 7.333500, track_1: 7.591290, track_2: 8.462330, track_3: 10.347500, track_4: 14.556600, track_5: 20.982300, track_6: 27.063200, track_7: 38.155800, track_8: 65.507896, track_9: 200.000000, track_10: 57.890999, track_11: 22.322001, track_12: 14.437300, track_13: 10.821200, track_14: 7.357770, track_15: 5.190360, track_16: 4.234850, track_17: 3.795900, track_18: 3.666500"
road_info = "angle: 0.523599, trackPos: 0.200000, track_0: 4.618802, track_1: 5.656854, track_2: 8.000000, track_3: 15.454813, track_4: 40.000000, track_5: 34.552623, track_6: 23.182220, track_7: 17.542826, track_8: 14.197209, track_9: 12.000000, track_10: 10.460681, track_11: 9.334343, track_12: 8.485281, track_13: 7.832444, track_14: 6.928203, track_15: 6.211657, track_16: 6.000000, track_17: 6.211657, track_18: 6.928203"
# 解析 angle 和 trackPos
angle_match = re.search(r'angle: (-?\d+\.\d+)', road_info)
trackPos_match = re.search(r'trackPos: (-?\d+\.\d+)', road_info)

angle = float(angle_match.group(1)) if angle_match else None
trackPos = float(trackPos_match.group(1)) if trackPos_match else None

# 解析 track_0 到 track_18 的值
track_matches = re.findall(r'track_\d+: (-?\d+\.\d+)', road_info)
track_data = [float(match) for match in track_matches]
# track_data = [3.463680, 3.577070, 3.975250, 4.838260, 6.745560, 9.608670, 12.314000, 17.096300, 26.956900, 49.470001, 86.315903, 127.277000, 36.190399, 24.608101, 15.789900, 10.858600, 8.773960, 7.825430, 7.536650]

# 角度范围，从正左（90度）到正右（-90度），递减10度
install_angle = np.array([-90,-75,-60,-45,-30,-20,-15,-10,-5,0,5,10,15,20,30,45,60,75,90])
angles =  install_angle[::-1] * np.pi / 180  # 将角度转换为弧度

# 转换为车辆坐标系下的x和y坐标
x_coords = np.array([d * np.cos(theta) for d, theta in zip(track_data, angles)])
y_coords = np.array([d * np.sin(theta) for d, theta in zip(track_data, angles)])

# 划分左右部分的索引
num_points = len(track_data)
# 找到 track_data 最大值对应的索引
max_index = np.argmax(track_data)

# 划分左右部分的索引
left_indices = np.arange(0, max_index)
right_indices = np.arange(max_index+1, len(track_data))

# 绘制左边部分
plt.scatter(x_coords[left_indices], y_coords[left_indices], color='black', marker = 'o', linestyle='solid', label='left_bound_det')
x_bound_left = x_coords[left_indices]
y_bound_left = y_coords[left_indices]
sort_idx = np.argsort(x_bound_left)
x_bound_left = x_bound_left[sort_idx]
y_bound_left = y_bound_left[sort_idx]
x_fit_dense_left = np.linspace(x_bound_left.min(), x_bound_left.max(), 100)

# 绘制右边部分
plt.scatter(x_coords[right_indices], y_coords[right_indices], color='black', marker = 'o', linestyle='solid', label='right_bound_det')
x_bound_right = x_coords[right_indices]
y_bound_right = y_coords[right_indices]
sort_idx = np.argsort(x_bound_right)
x_bound_right = x_bound_right[sort_idx]
y_bound_right = y_bound_right[sort_idx]
x_fit_dense_right = np.linspace(x_bound_right.min(), x_bound_right.max(), 100)

# 绘制中间部分（虚线）
x_middle = [0, 20]
y_middle = [0, 0]
plt.plot(x_middle, y_middle, color='black', linestyle='dashed', label='vehicle x_axes')

theta = angle
rotation_matrix1 = np.array([
    [np.cos(theta), -np.sin(theta)],
    [np.sin(theta), np.cos(theta)]
])
beta = theta + np.pi / 2
rotation_matrix2 = np.array([
    [np.cos(beta), -np.sin(beta)],
    [np.sin(beta), np.cos(beta)]
])
base_vector_x = np.array([[1, 0]])
base_vector_y = np.array([[0, 1]])
track_vector = rotation_matrix1 @ base_vector_x.T
tau_vector = rotation_matrix2 @ base_vector_x.T
road_width = (np.abs(y_coords[left_indices[0]]) + np.abs(y_coords[right_indices[-1]])) * np.cos(theta)
print("road_width: ", road_width)
toMiddle = road_width / 2 * trackPos

# 提取旋转后的 x 和 y 坐标
track_vector_len = max(track_data)
front_rear_ratio = 0.1
proj_point = -toMiddle * tau_vector
proj_point = proj_point.T
start_point = proj_point - track_vector.T * track_vector_len * front_rear_ratio
end_point = proj_point + track_vector.T * track_vector_len * (1 - front_rear_ratio)
# 提取端点的 x 和 y 坐标
start_x, start_y = start_point[0]
end_x, end_y = end_point[0]
# 绘制线段
plt.plot([start_x, end_x], [start_y, end_y], color='r', linestyle='dashed', label='road center')

x_fit_dense = np.arange(0, 35 + 0.1, 0.1)
y_fit_left = fit_polynomial(x_bound_left, y_bound_left, x_fit_dense)
y_fit_right = fit_polynomial(x_bound_right, y_bound_right, x_fit_dense)
y_fit_center = (y_fit_left + y_fit_right) / 2
plt.plot(x_fit_dense, y_fit_center, color='k', linestyle='dashed', label='road center fit')

left_proj_point = proj_point.T + road_width / 2 * tau_vector
right_proj_point = proj_point.T - road_width / 2 * tau_vector
start_point_left = left_proj_point - track_vector * track_vector_len * front_rear_ratio
end_point_left = left_proj_point + track_vector * track_vector_len * (1 - front_rear_ratio)
start_point_right = right_proj_point - track_vector * track_vector_len * front_rear_ratio
end_point_right = right_proj_point + track_vector * track_vector_len * (1 - front_rear_ratio)
# plt.scatter(start_point_left[0][0], start_point_left[1][0], color='r', marker='o', label='left_proj_point')
start_x, start_y = start_point_left[0, 0], start_point_left[1, 0]
end_x, end_y = end_point_left[0, 0], end_point_left[1, 0]
plt.plot([start_x, end_x], [start_y, end_y], color='r', linestyle='dashed', label='left bound')
start_x, start_y = start_point_right[0, 0], start_point_right[1, 0]
end_x, end_y = end_point_right[0, 0], end_point_right[1, 0]
plt.plot([start_x, end_x], [start_y, end_y], color='r', linestyle='dashed', label='right bound')


# 添加标签和标题
plt.title("road boundary")
plt.xlabel("x/m")
plt.ylabel("y/m")
plt.legend()

# 显示图形
plt.show()

y_poly_left = fit_polynomial(x_bound_left, y_bound_left, x_fit_dense_left)
y_spline_left = fit_spline(x_bound_left, y_bound_left, x_fit_dense_left)
y_ransac_left = fit_ransac(x_bound_left, y_bound_left, x_fit_dense_left)
x_bezier_left, y_bezier_left = fit_bezier(x_bound_left, y_bound_left)
y_bspline_left = fit_bspline(x_bound_left, y_bound_left, x_fit_dense_left)
y_poly_right = fit_polynomial(x_bound_right, y_bound_right, x_fit_dense_right)
y_spline_right = fit_spline(x_bound_right, y_bound_right, x_fit_dense_right)
y_ransac_right = fit_ransac(x_bound_right, y_bound_right, x_fit_dense_right)
x_bezier_right, y_bezier_right = fit_bezier(x_bound_right, y_bound_right)
y_bspline_right = fit_bspline(x_bound_right, y_bound_right, x_fit_dense_right)

plt.figure(figsize=(12, 8))

# 绘制各拟合曲线
plt.scatter(x_bound_left, y_bound_left, c='black', label="Boundary Points", zorder=5)
plt.plot(x_fit_dense_left, y_poly_left, label="Polynomial (3rd Degree)", linestyle='--')
plt.plot(x_fit_dense_left, y_spline_left, label="Cubic Spline", linestyle='-.')
plt.plot(x_fit_dense_left, y_ransac_left, label="RANSAC (3rd Degree)", linestyle=':')
plt.plot(x_bezier_left, y_bezier_left, label="Bézier Curve", linewidth=2)
plt.plot(x_fit_dense_left, y_bspline_left, label="B-Spline (k=3)", linewidth=2)

plt.scatter(x_bound_right, y_bound_right, c='black', label="Boundary Points", zorder=5)
# 绘制各拟合曲线
plt.plot(x_fit_dense_right, y_poly_right, label="Polynomial (3rd Degree)", linestyle='--')
plt.plot(x_fit_dense_right, y_spline_right, label="Cubic Spline", linestyle='-.')
plt.plot(x_fit_dense_right, y_ransac_right, label="RANSAC (3rd Degree)", linestyle=':')
plt.plot(x_bezier_right, y_bezier_right, label="Bézier Curve", linewidth=2)
plt.plot(x_fit_dense_right, y_bspline_right, label="B-Spline (k=3)", linewidth=2)

plt.xlabel('X')
plt.ylabel('Y')
plt.title('Road Boundary Fitting Comparison')
plt.legend()
plt.grid(True)
plt.show()
