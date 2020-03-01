#只有弹球模型
import numpy as np

def get_initial_coordinates(box_width):
    x_coord = np.random.random() * (box_width - 0.1)
    y_coord = np.random.random() * (box_width - 0.1)
    return x_coord, y_coord

def get_initial_velocity():
    vel_initial = np.random.random() * Uav.max_vel
    x_vel = (np.random.random() - 0.5) * 2 * vel_initial
    y_vel = np.sign(np.random.random() - 0.5) * np.power(np.power(vel_initial,2) - np.power(x_vel,2), 0.5)
    return x_vel, y_vel

#无人机之间的加速度势函数
# def acceleration_potential_func(distance):
#     acceleration = 0
#     if distance <= Uav.recognition_radius:
#         acceleration = ((Uav.recognition_radius - distance) / (distance + 1/np.power(10, 5)))
#     return acceleration

def acceleration_potential_func(distance):
    acceleration = 0
    if distance <= Uav.recognition_radius:
        acceleration = ((Uav.recognition_radius - distance) / (distance + 1/np.power(10, 5))) * 10
    return acceleration

def get_min_and_sign(x, y):
    m = min(abs(x), abs(y))
    if abs(x) <= abs(y):
        s = np.sign(x)
    else:
        s = np.sign(y)
    return m, s

class Uav:
    recognition_radius = 5
    max_vel = 15
    def __init__(self, box_width, dt):
        self.x_coord, self.y_coord = get_initial_coordinates(box_width)
        self.x_vel, self.y_vel = get_initial_velocity()
        self.x_acceleration, self.y_acceleration = [(np.random.random() - 0.5) * 2 * 10, (np.random.random() - 0.5) * 2 * 10]
        #self.max_vel_x, self.max_vel_y = [3 * box_width / 10, 3 * box_width / 10]
        #定义无人机的“感知半径”
        #self.recognition_radius = 5
        #一个存放出本身之外其他无人机的列表
        self.other_uavs = []
        #邻居
        self.neighbors = []
        #边界
        self.boarder = box_width
        #时间间隔
        self.time_interval = dt
        return

    def calculate_location_for_next_step(self):
        #找到邻居
        neighbors = self.neighbors_searching(self.other_uavs)
        #计算速度
        self.calculate_and_set_velocity_for_next_step(neighbors)
        #计算新的位置坐标
        self.x_coord = self.x_coord + self.x_vel * self.time_interval
        #如果越界了怎么办，首先纠正位置信息，然后使该方向上的速度变为反方向。
        # if self.x_coord > self.boarder:
        #     self.x_coord = self.boarder
        #     self.x_vel = -1 * self.x_vel
        # #self.y_coord = self.y_coord + self.y_vel
        # if self.x_coord < 0:
        #     self.x_coord = 0
        #     self.x_vel = -1 * self.x_vel
        self.y_coord = self.y_coord + self.y_vel * self.time_interval
        # if self.y_coord > self.boarder:
        #     self.y_coord = self.boarder
        #     self.y_vel = -1 * self.y_vel
        # if self.y_coord < 0:
        #     self.y_coord = 0
        #     self.y_vel = -1 * self.y_vel

    def calculate_distance_to_me(self, other_uav):
        distance = np.power(
            np.power((other_uav.x_coord-self.x_coord), 2) + np.power((other_uav.y_coord-self.y_coord), 2), 1/2)
        return distance

    # 寻找邻居的功能，将在自身感知半径内的无人机全部找出来
    # --input:
    #    other_uavs: 包含除自身外的其他无人机的一个列表
    # --output:
    #    neighbors: 包含此瞬间自身所有邻居的一个列表
    def neighbors_searching(self, other_uavs ):
        # 对other_uavs中的所有entry计算其与自身之间的距离，距离在self.recognition_radius之内的作为令居，加入返回值列表中
        # 首先定义空列表（返回值）
        neighbors = []
        for other_uav in other_uavs:
            distance = self.calculate_distance_to_me(other_uav)
            if distance <= Uav.recognition_radius:
                neighbors.append(other_uav)

        self.neighbors = neighbors
        #print(self.neighbors)
        return neighbors

    #计算下一步的加速度
    def calculate_acceleration_for_next_step(self, neighbors):
        acceleration_x = self.x_acceleration
        acceleration_y = self.y_acceleration
        for neighbor in neighbors:
            acceleration = acceleration_potential_func(self.calculate_distance_to_me(neighbor))
            acceleration_x_for_this_neighbor = acceleration * np.cos(
                np.arctan(abs((neighbor.y_coord-self.y_coord)/(neighbor.x_coord-self.x_coord + 1/np.power(10,5))))) * np.sign(
                (self.x_coord-neighbor.x_coord))
            acceleration_y_for_this_neighbor = acceleration * np.sin(
                np.arctan(abs((neighbor.y_coord - self.y_coord) / (neighbor.x_coord - self.x_coord + 1/np.power(10,5))))) * np.sign(
                (self.y_coord - neighbor.y_coord))
            acceleration_x += acceleration_x_for_this_neighbor
            acceleration_y += acceleration_y_for_this_neighbor
        # 最后要防止无人机在边界停留
        acceleration_x += self.boundary_acceleration_x()
        acceleration_y += self.boundary_acceleration_y()
        return acceleration_x, acceleration_y

    def boundary_acceleration_x(self):
        m, s = get_min_and_sign(self.x_coord-0, self.x_coord-self.boarder)
        boundary_acceleration_x = self.calculate_boundary_acceleration(m, s)
        return boundary_acceleration_x

    def boundary_acceleration_y(self):
        m, s = get_min_and_sign(self.y_coord - 0, self.y_coord - self.boarder)
        boundary_acceleration_y = self.calculate_boundary_acceleration(m, s)
        return boundary_acceleration_y

    def calculate_boundary_acceleration(self, m, s):
        if m <= 5:
            boundary_acceleration = 100 / (np.power(m, 2) + 1/np.power(10,5)) * s
        else:
            boundary_acceleration = 0
        return boundary_acceleration


    #计算并设置下一步的速度
    def calculate_and_set_velocity_for_next_step(self, neighbors):
        #第一步获取到邻居列表
        #neighbors = self.neighbors_searching(other_uavs)
        # 计算并设置下一步的加速度
        self.x_acceleration, self.y_acceleration = self.calculate_acceleration_for_next_step(neighbors)
        # 计算x方向速度，由方向符号
        x_vel = self.x_vel + self.x_acceleration * self.time_interval
        # 计算y方向速度，有方向符号
        y_vel = self.y_vel + self.y_acceleration * self.time_interval
        # 计算总速度
        vel = np.power((np.power(x_vel, 2) + np.power(y_vel, 2)), 1 / 2)
        # 判断总速度是否大于最大速度，如果大于最大速度，则将总速度设置为最大速度
        if vel > Uav.max_vel:
            # 计算速度夹角（正角）
            alpha = np.arctan(abs(y_vel / x_vel))
            x_vel = Uav.max_vel * np.cos(alpha) * np.sign(x_vel)
            y_vel = Uav.max_vel * np.sin(alpha) * np.sign(y_vel)
        # 设置下一步的速度
        self.x_vel, self.y_vel = [x_vel, y_vel]
        return x_vel, y_vel




