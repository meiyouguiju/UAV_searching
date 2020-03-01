#最大速度：15m/s
#初始速度的设置变化
#只有菲克定律的情况
#有一个障碍物的情况
# ----------------------------------
# |                                |
# |        7___8                   |
# |     6 __| |__(50,70) 1         |
# |     5 |_   _|2                 |
# |       4 |_|3                   |
# |                                |
# |   (10m every edge)             |
# |                                |
# |                                |
# |                                |
# |                                |
# |                                |
# ----------------------------------

import numpy as np

#保证初始化的时候不在障碍物里面
def get_initial_coordinates(box_width):
    x_coord, y_coord = [35, 65]
    while(in_obstacle(x_coord, y_coord)):
        x_coord = np.random.random()*(box_width-0.1)
        y_coord = np.random.random()*(box_width-0.1)
    return x_coord, y_coord

def in_obstacle(x, y):
    if ((x<=(40+0.1) and x>=(30-0.1)) and (y<=(80+0.1) and y>=(50-0.1)) or \
            (x<=(50+0.1) and x>=(20-0.1)) and (y<=(70+0.1) and y>=(60-0.1))) :
        return True
    return False

# def get_initial_velocity(box_width):
#     x_vel = (np.random.random() - 0.5) * 2 * box_width
#     y_vel = (np.random.random() - 0.5) * 2 * box_width
#     return x_vel, y_vel

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
    dense_recognition_radius = 10
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
        #浓度邻居
        self.dense_neighbors = []
        #边界
        self.boarder = box_width
        #时间间隔
        self.time_interval = dt
        return

    def calculate_location_for_next_step(self):
        #找到邻居
        self.neighbors = self.neighbors_searching(self.other_uavs, self.recognition_radius)
        #找到浓度邻居
        self.dense_neighbors = self.neighbors_searching(self.other_uavs, self.dense_recognition_radius)
        #计算速度
        self.calculate_and_set_velocity_for_next_step(self.neighbors, self.dense_neighbors)
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
    #以下这个函数会再将来被写入上面的函数中======
    def calculate_point_distance_tome(self, x, y):
        distance = np.power(
            np.power((self.x_coord - x), 2) + np.power((self.y_coord - y), 2), 1 / 2)
        return distance

    # 寻找邻居的功能，将在自身感知半径内的无人机全部找出来
    # --input:
    #    other_uavs: 包含除自身外的其他无人机的一个列表
    # --output:
    #    neighbors: 包含此瞬间自身所有邻居的一个列表
    def neighbors_searching(self, other_uavs, distance_lim ):
        # 对other_uavs中的所有entry计算其与自身之间的距离，距离在self.recognition_radius之内的作为令居，加入返回值列表中
        # 首先定义空列表（返回值）
        neighbors = []
        for other_uav in other_uavs:
            distance = self.calculate_distance_to_me(other_uav)
            if distance <= distance_lim:
                neighbors.append(other_uav)

        #self.neighbors = neighbors
        #print(self.neighbors)
        return neighbors

    #计算下一步的加速度，总共两种加速度，其他无人机给的和边界给的
    def calculate_acceleration_for_next_step(self, neighbors, dense_neighbors):
        acceleration_x = self.x_acceleration
        acceleration_y = self.y_acceleration
        #首先是由于邻居对其的排斥造成的加速度
        # for neighbor in neighbors:
        #     acceleration = acceleration_potential_func(self.calculate_distance_to_me(neighbor))
        #     acceleration_x_for_this_neighbor = acceleration * np.cos(
        #         np.arctan(abs((neighbor.y_coord-self.y_coord)/(neighbor.x_coord-self.x_coord + 1/np.power(10,5))))) * np.sign(
        #         (self.x_coord-neighbor.x_coord))
        #     acceleration_y_for_this_neighbor = acceleration * np.sin(
        #         np.arctan(abs((neighbor.y_coord - self.y_coord) / (neighbor.x_coord - self.x_coord + 1/np.power(10,5))))) * np.sign(
        #         (self.y_coord - neighbor.y_coord))
        #     acceleration_x += acceleration_x_for_this_neighbor
        #     acceleration_y += acceleration_y_for_this_neighbor
        #然后是根据菲克定律，由无人机的浓度差造成的加速度
        acceleration_x += self.dense_acceleration_x(dense_neighbors)
        acceleration_y += self.dense_acceleration_y(dense_neighbors)
        #最后要防止无人机在边界停留
        acceleration_x += self.boundary_acceleration_x()
        acceleration_y += self.boundary_acceleration_y()
        #障碍物对无人机造成的加速度
        acceleration_x += self.obstacle_acceleration_x()
        acceleration_y += self.obstacle_acceleration_y()
        return acceleration_x, acceleration_y

    def obstacle_acceleration_x(self):
        obstacle_acceleration_x = (self.calculate_obstacle_acceleration(self.x_coord - 20) + self.calculate_obstacle_acceleration(self.x_coord - 50)) \
                                  * (self.y_coord <= 70 and self.y_coord >= 60) + \
                                  (self.calculate_obstacle_acceleration(self.x_coord - 30) + self.calculate_obstacle_acceleration(self.x_coord - 40)) \
                                  * (self.y_coord <= 80 and self.y_coord >= 70 or self.y_coord <= 60 and self.y_coord >= 50) + \
                                  self.eight_points_accel_x()
        return obstacle_acceleration_x

    def obstacle_acceleration_y(self):
        obstacle_acceleration_y = (self.calculate_obstacle_acceleration(self.y_coord - 50) + self.calculate_obstacle_acceleration(self.y_coord - 80))\
                                  * (self.x_coord<=40 and self.x_coord>=30) + \
                                  (self.calculate_obstacle_acceleration(self.y_coord - 60) + self.calculate_obstacle_acceleration(self.y_coord - 70))\
                                  * (self.x_coord<=30 and self.x_coord>=20 or self.x_coord<=50 and self.x_coord>=40) + \
                                  self.eight_points_accel_y()
        return obstacle_acceleration_y

    #八个特殊点的加速度计算
    def eight_points_accel_x(self):
        eight_points_accel_x = self.cal_eightp_accel_x(50, 70) * (self.x_coord>=50 and self.y_coord>=70)+\
                self.cal_eightp_accel_x(50, 60) * (self.x_coord>=50 and self.y_coord<=60) +\
                self.cal_eightp_accel_x(40, 50) * (self.x_coord>=40 and self.y_coord<=50) +\
                self.cal_eightp_accel_x(30, 50) * (self.x_coord<=30 and self.y_coord<=50) +\
                self.cal_eightp_accel_x(20, 60) * (self.x_coord<=20 and self.y_coord<=60) +\
                self.cal_eightp_accel_x(20, 70) * (self.x_coord<=20 and self.y_coord>=70) +\
                self.cal_eightp_accel_x(30, 80) * (self.x_coord<=30 and self.y_coord>=80) +\
                self.cal_eightp_accel_x(40, 80) * (self.x_coord>=40 and self.y_coord>=80)
        return eight_points_accel_x

    def eight_points_accel_y(self):
        eight_points_accel_y = self.cal_eightp_accel_y(50, 70) * (self.x_coord >= 50 and self.y_coord >= 70) +\
                self.cal_eightp_accel_y(50, 60) * (self.x_coord >= 50 and self.y_coord <= 60) +\
                self.cal_eightp_accel_y(40, 50) * (self.x_coord >= 40 and self.y_coord <= 50) +\
                self.cal_eightp_accel_y(30, 50) * (self.x_coord <= 30 and self.y_coord <= 50) +\
                self.cal_eightp_accel_y(20, 60) * (self.x_coord <= 20 and self.y_coord <= 60) +\
                self.cal_eightp_accel_y(20, 70) * (self.x_coord <= 20 and self.y_coord >= 70) +\
                self.cal_eightp_accel_y(30, 80) * (self.x_coord <= 30 and self.y_coord >= 80) +\
                self.cal_eightp_accel_y(40, 80) * (self.x_coord >= 40 and self.y_coord >= 80)
        return eight_points_accel_y

    def cal_eightp_accel_x(self, x, y):
        #首先计算当前无人机到所指定的八个点其中之一的距离
        distance = self.calculate_point_distance_tome(x, y)
        #先计算大小
        accel = 100 / (np.power(distance, 2) + 1 / np.power(10, 5)) * (distance <= 5)
        #再计算x方向的大小，正负号别忘记
        accel_x = accel * np.cos(abs(np.arctan((self.y_coord-y)/(self.x_coord-x)))) * \
                  np.sign(self.x_coord-x)
        return accel_x

    def cal_eightp_accel_y(self, x, y):
        #首先计算当前无人机到所指定的八个点其中之一的距离
        distance = self.calculate_point_distance_tome(x, y)
        #先计算大小
        accel = 100 / (np.power(distance, 2) + 1 / np.power(10, 5)) * (distance <= 5)
        #再计算x方向的大小，正负号别忘记
        accel_y = accel * np.sin(abs(np.arctan((self.y_coord-y)/(self.x_coord-x)))) * \
                  np.sign(self.y_coord-y)
        return accel_y

    def calculate_obstacle_acceleration(self, signed_d):
        s = np.sign(signed_d)
        d = np.abs(signed_d)
        obstacle_acceleration = self.calculate_boundary_acceleration(d, s)
        return obstacle_acceleration

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

    def dense_acceleration_x(self, dense_neighbors):
        #首先需要得到左右两侧无人机的数量
        num_uavs_left, num_uavs_right = self.calculate_num_uavs_left_right(dense_neighbors)
        #然后根据两侧的数量差（无人机的“浓度差”）计算出x方向的加速度dense_acceleration_x
        dense_acceleration_x = abs(num_uavs_left-num_uavs_right)/(num_uavs_left+num_uavs_right + 1/np.power(10, 5)) * 10 * np.sign(
            num_uavs_left-num_uavs_right)
        return dense_acceleration_x

    def dense_acceleration_y(self, dense_neighbors):
        # 首先需要得到上下两侧无人机的数量
        num_uavs_down, num_uavs_up = self.calculate_num_uavs_down_up(dense_neighbors)
        # 然后根据上下的数量差（无人机的“浓度差”）计算出y方向的加速度dense_acceleration_y
        dense_acceleration_y = abs(num_uavs_down - num_uavs_up) / (num_uavs_down + num_uavs_up+ 1/np.power(10, 5)) * 10 * np.sign(
            num_uavs_down - num_uavs_up)
        return dense_acceleration_y

    def calculate_num_uavs_left_right(self, dense_neighbors):
        num_uavs_left = 0
        num_uavs_right = 0
        for uav in dense_neighbors:
            if uav.x_coord <= self.x_coord:
                num_uavs_left += 1
            else:
                num_uavs_right += 1
        return num_uavs_left, num_uavs_right


    def calculate_num_uavs_down_up(self, dense_neighbors):
        num_uav_down = 0
        num_uav_up = 0
        for uav in dense_neighbors:
            if uav.y_coord <= self.y_coord:
                num_uav_down += 1
            else:
                num_uav_up += 1
        return num_uav_down, num_uav_up


    #计算并设置下一步的速度
    def calculate_and_set_velocity_for_next_step(self, neighbors, dense_neighbors):
        #第一步获取到邻居列表
        #neighbors = self.neighbors_searching(other_uavs) #在calculate_location_for_next_step函数中已经有过寻找邻居的过程
        #计算并设置下一步的加速度
        self.x_acceleration, self.y_acceleration = self.calculate_acceleration_for_next_step(neighbors, dense_neighbors)
        #print(self.x_acceleration)
        #计算x方向速度，由方向符号
        x_vel = self.x_vel + self.x_acceleration * self.time_interval
        #计算y方向速度，有方向符号
        y_vel = self.y_vel + self.y_acceleration * self.time_interval
        #计算总速度
        vel = np.power((np.power(x_vel,2)+np.power(y_vel,2)), 1/2)
        #判断总速度是否大于最大速度，如果大于最大速度，则将总速度设置为最大速度
        if vel > Uav.max_vel:
            #计算速度夹角（正角）
            alpha = np.arctan(abs(y_vel/x_vel))
            x_vel = Uav.max_vel * np.cos(alpha) * np.sign(x_vel)
            y_vel = Uav.max_vel * np.sin(alpha) * np.sign(y_vel)
        #设置下一步的速度
        self.x_vel, self.y_vel = [x_vel, y_vel]
        return x_vel, y_vel




