import numpy as np
import matplotlib.pyplot as plt

def get_road_bound(road_width, self_position_y, self_angle, install_angle, max_distance):
    # 道路参数
    # road_width = 10
    # self_position_y = 1
    # self_angle = -30

    # 传感器参数
    # install_angle = np.array([-90, -75, -60, -45, -30, -20, -15, -10, -5, 0, 5, 10, 15, 20, 30, 45, 60, 75, 90])
    # max_distance = 40

    # 计算道路边界
    bottom_boundary = -road_width / 2
    top_boundary = road_width / 2


    # 计算交点距离的函数
    def calculate_intersection_distance(angle):
        relative_angle = np.radians(angle + self_angle)
        if np.abs(np.abs(relative_angle) - np.pi / 2) < 1e-6:
            if relative_angle > 0:
                return (top_boundary - self_position_y) / np.sin(relative_angle)
            else:
                return (bottom_boundary - self_position_y) / np.sin(relative_angle)
        elif np.abs(relative_angle) < 1e-6:
            return max_distance
        else:
            distance_bottom = (bottom_boundary - self_position_y) / np.sin(relative_angle)
            distance_top = (top_boundary - self_position_y) / np.sin(relative_angle)
            valid_distances = [d for d in [distance_bottom, distance_top] if d > 0]
            return min(valid_distances) if valid_distances else max_distance


    # 计算距离
    distances = [calculate_intersection_distance(angle) for angle in install_angle[::-1]]

    angle = -self_angle / 180 * np.pi
    trackPos = self_position_y / road_width * 2
    output_str = f"angle: {angle:.6f}, trackPos: {trackPos:.6f}, "
    output_str += ", ".join([f"track_{i}: {dist:.6f}" for i, dist in enumerate(distances)])

    return distances, bottom_boundary, top_boundary, output_str


if __name__ == '__main__':
    # 道路参数
    road_width = 10
    self_position_y = 1
    self_angle = -30

    # 传感器参数
    install_angle = np.array([-90, -75, -60, -45, -30, -20, -15, -10, -5, 0, 5, 10, 15, 20, 30, 45, 60, 75, 90])
    max_distance = 40

    distances, bottom_boundary, top_boundary, output_str = get_road_bound(road_width, self_position_y, self_angle, install_angle, max_distance)
    print()
    print(output_str)

    # 绘制简单示意图
    plt.figure(figsize=(8, 4))
    plt.axhline(y=bottom_boundary, color='k', linestyle='--')
    plt.axhline(y=top_boundary, color='k', linestyle='--')
    plt.scatter(0, self_position_y, color='r', marker='o')
    install_angle = install_angle[::-1]
    for i, angle in enumerate(install_angle):
        distance = distances[i]
        relative_angle = np.radians(angle + self_angle)
        x = distance * np.cos(relative_angle)
        y = self_position_y + distance * np.sin(relative_angle)
        plt.plot([0, x], [self_position_y, y], 'b-', alpha=0.5)
    plt.xlabel('X (m)')
    plt.ylabel('Y (m)')
    plt.title('Road and Sensor Intersection')
    plt.grid(True)
    plt.axis('equal')
    plt.show()