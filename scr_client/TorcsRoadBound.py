import re
import numpy as np
import matplotlib.pyplot as plt

# 设置 matplotlib 的字体
plt.rcParams['font.sans-serif'] = ['STSong']  # 使用黑体字体
plt.rcParams['axes.unicode_minus'] = False  # 解决负号显示问题


# 道路边界数据
# road_info = "2025-03-19 17:42:20.825 INFO <SimpleDriver.cpp:168> angle: -0.101198, trackPos: -0.277055, track_0: 7.059810, track_1: 7.114990, track_2: 7.696840, track_3: 9.048410, track_4: 11.944600, track_5: 15.937600, track_6: 19.355000, track_7: 24.681101, track_8: 34.212502, track_9: 56.436901, track_10: 164.641998, track_11: 84.404503, track_12: 27.462500, track_13: 16.685200, track_14: 9.774880, track_15: 6.304710, track_16: 4.905990, track_17: 4.254090, track_18: 3.996700"
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

# 绘制右边部分
plt.scatter(x_coords[right_indices], y_coords[right_indices], color='black', marker = 'o', linestyle='solid', label='right_bound_det')

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