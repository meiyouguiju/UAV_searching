#最大速度：15m/s
#初始速度的设置变化
#只有菲克定律的情况
#每一步都释放信息素，信息素的蒸发速度的单位为单位浓度/s
import numpy as np
import time
import matplotlib.pyplot as plt
from matplotlib.path import Path
import matplotlib.patches as patches
import matplotlib.animation as animation
from matplotlib import cm
from matplotlib.colors import ListedColormap
import json
from matplotlib.animation import FFMpegWriter


# class patch:
#     """patch类，将搜索区域划分为若干小patch，主要用于后续覆盖面积的计算等。"""
#     def __init__(self,
#                  patch_id = None,
#                  size = 1,
#                  x_coord = None,
#                  y_coord = None,
#                  been_searched = False,     #是否被搜索过
#                  is_obstacle = False,       #是否是障碍物
#                  pheromone_density = None,  #此patch上的pheromone浓度是多少
#                  ):
#         self.patch_id = patch_id
#         self.size = size
#         self.x_coord = x_coord
#         self.y_coord = y_coord
#         self.been_searched = been_searched
#         self.is_obstacle = is_obstacle
#         self.pheromone_density = pheromone_density



def fick_accel_cal(n):
    return (abs(n[2] - n[3]) / (n[2] + n[3] + 1 / np.power(10, 5)) * np.sign(n[2] - n[3]) * np.array([1, 0]) + \
            abs(n[1] - n[0]) / (n[1] + n[0] + 1 / np.power(10, 5)) * np.sign(n[1] - n[0]) * np.array([0, 1])) * 10

figsize = 10
fig, ax = plt.subplots(figsize=(figsize, figsize))

np.random.seed(83987)

class uav_coverage:
    """这个类代表整个覆盖
    最大速度默认为15m/s
    各种搜索模式：0bounce 1mixed 2fick 3stochastic"""
    def __init__(self,
                 number_of_uavs = None,
                 ini_position_of_uavs = None,
                 ini_velocity_of_uavs = None,
                 sense_range = 5, #bounce range
                 dense_sense_range = 10, #fick model range
                 obstacle_type = 'no_obstacle',
                 coverage_strategy = 2,
                 time_step = 0.01,
                 area_width = 100,
                 max_vel = 15, #最大速度，m/s
                 vel_of_phe_evap = None, #信息素的蒸发速度
                 ini_phe = None,
                 min_phe_detect_range = 1,
                 max_phe_detect_range = None,
                 phe_release_range = None,
                 #phe_step = None,
                 end_time = 20,
                 pheromone = False,
                 ):
        self.area_width = area_width
        self.max_vel = max_vel
        self.end_time = end_time
        self.strategy = {
            0 : self.bounce,
            1 : self.mixed,
            2 : self.fick,
            3 : self.stochastic,
        }
        self.number_of_uavs = number_of_uavs
        if (isinstance(ini_position_of_uavs, type(np.ones(1)))):
            self.posotion_of_uavs = ini_position_of_uavs
        elif (ini_position_of_uavs == None):
            self.posotion_of_uavs = self.ini_pos_of_uavs(obstacle_type)#初始化无人机的位置，根据area_width######################
        if (isinstance(ini_velocity_of_uavs, type(np.ones(1)))):
            self.velocity_of_uavs = ini_velocity_of_uavs
        elif (ini_velocity_of_uavs == None):
            self.velocity_of_uavs = self.ini_vel_of_uavs()#初始化无人机的速度##############################
        self.sense_range = sense_range
        self.dense_sense_range = dense_sense_range
        #根据初始函数的参数obstacle_type来初始化障碍物的位置和障碍物区域的面积
        self.position_of_obs = self.cal_pos_of_obs(obstacle_type)
        self.obs_area = self.cal_obs_area(obstacle_type)
        self.obs_type = obstacle_type
        self.coverage_strategy = coverage_strategy
        self.time_step = time_step
        #初始化信息素浓度的地图
        self.phe_map = np.zeros((self.area_width, self.area_width))
        #初始化障碍物#####################
        #初始化自身的alpha值
        if pheromone == True:
            self.alpha_time = self.find_alpha_time()
            #self.alpha_time = 0
        else:
            self.alpha_time = 10000
        #self.alpha_time = 0.031
        self.ini_phe = ini_phe
        #self.phe_step = phe_step
        self.vel_of_phe_evap= vel_of_phe_evap
        self.coverage_map = np.zeros((self.area_width, self.area_width))
        self.min_phe_detect_range = min_phe_detect_range
        if max_phe_detect_range == None:
            self.max_phe_detect_range = sense_range
        else:
            self.max_phe_detect_range = max_phe_detect_range
        if phe_release_range == None:
            self.phe_release_range = sense_range
        else:
            self.phe_release_range = phe_release_range


    def coverage(self):
        #首先初始化储存无人机位置、速度、信息素浓度、仿真时间的list
        coverage_time = 0
        time_now = time.time()
        end_time = self.end_time
        self.program_run_time = []
        self.position_of_flock_time = []
        self.velocity_of_flock_time = []
        self.density_of_phe_time = []
        self.coverage_per_time = []
        self.coverage_map_time = []
        self.accel_time = []
        self.time_slot = []

        #计算下一步的加速度（过了时间转折点之后需要考虑信息素的作用）
        while coverage_time < end_time:
            #首先计算普通的加速度，需要根据strategy的类型+
            #边界加速度+ 信息素加速度(分时段)+障碍物施加的加速度
            self.acceleration = self.strategy[self.coverage_strategy]() + \
                self.accel_boundary_cal() + \
                self.accel_phe(coverage_time) +\
                self.accel_obs()
            self.accel_time.append(self.acceleration.copy())
            #喷洒信息素
            #首先更新信息素地图(包含信息素的蒸发)，再使用append添加
            self.density_of_phe_time.append(self.update_phe_map(coverage_time).copy())
            #更新自身的位置和速度,并且使用append添加
            self.posotion_of_uavs = self.posotion_of_uavs +\
                self.velocity_of_uavs * self.time_step
            self.position_of_flock_time.append(self.posotion_of_uavs.copy())
            #print(self.velocity_of_uavs)
            # print(self.velocity_of_uavs +\
            #     self.acceleration * self.time_step)
            self.velocity_of_uavs = self.check_vel_of_uavs(self.velocity_of_uavs +\
                self.acceleration * self.time_step)
            #print(self.velocity_of_uavs)
            self.velocity_of_flock_time.append(self.velocity_of_uavs.copy())
            #计算覆盖率
            self.time_slot.append(coverage_time)
            cov_per = self.cal_coverage_per()
            self.coverage_per_time.append(cov_per)
            self.coverage_map_time.append(self.coverage_map.copy())
            #更新时间
            coverage_time += self.time_step
            self.program_run_time.append(time.time()-time_now)
            time_now = time.time()
            #打印信息
            print('time:{:.2}, coverage percentage:{}'.format(
                coverage_time-self.time_step, cov_per
            ))

    def plot(self,frame_slot,
             arrow_width=.25):
        #首先清空ax
        fig.clear()
        ax = fig.add_subplot(111)
        ax.clear()
        cmap1 = ListedColormap(["white", "lawngreen"])
        cmap2 = cm.get_cmap('Oranges', 256)
        #画无人机位置
        ax.plot(self.position_of_flock_time[frame_slot][:,0],
                self.position_of_flock_time[frame_slot][:,1],
                'or', alpha= 0.5)
        #画速度箭头
        unit = np.zeros(self.velocity_of_flock_time[frame_slot].shape)
        # print(self.velocity_of_flock_time[frame_slot].shape)
        # print(type(self.velocity_of_flock_time))
        # print(len(self.velocity_of_flock_time))
        norms = np.zeros(self.number_of_uavs)
        # Velocity vector normalization
        for i in range(0, self.number_of_uavs):
            norms[i] = np.linalg.norm(self.velocity_of_flock_time[frame_slot][i])
            if norms[i] != 0:
                unit[i] = self.velocity_of_flock_time[frame_slot][i] / norms[i]
            else:
                unit[i] = np.zeros(2)
        rel = np.zeros(self.velocity_of_flock_time[frame_slot].shape)
        # Velocity vector arrow normalization to show the length of arrow
        for i in range(0, self.number_of_uavs):
            rel[i] = unit[i] * (np.linalg.norm(self.velocity_of_flock_time[frame_slot][i]) / max(norms))

        for i in range(0, self.number_of_uavs):
            plt.arrow(self.position_of_flock_time[frame_slot][i, 0], self.position_of_flock_time[frame_slot][i, 1],
                      rel[i, 0], rel[i, 1],
                      width=arrow_width,
                      edgecolor='green',
                      facecolor='green')
        #画加速度箭头
        unit = np.zeros(self.accel_time[frame_slot].shape)
        norms = np.zeros(self.number_of_uavs)
        # Velocity vector normalization
        for i in range(0, self.number_of_uavs):
            norms[i] = np.linalg.norm(self.accel_time[frame_slot][i])
            if norms[i] != 0:
                unit[i] = self.accel_time[frame_slot][i] / norms[i]
            else:
                unit[i] = np.zeros(2)
        rel = np.zeros(self.accel_time[frame_slot].shape)
        # Velocity vector arrow normalization to show the length of arrow
        for i in range(0, self.number_of_uavs):
            rel[i] = unit[i] * (np.linalg.norm(self.accel_time[frame_slot][i]) / max(norms))

        for i in range(0, self.number_of_uavs):
            plt.arrow(self.position_of_flock_time[frame_slot][i, 0], self.position_of_flock_time[frame_slot][i, 1],
                      rel[i, 0], rel[i, 1],
                      width=arrow_width,
                      edgecolor='blue',
                      facecolor='blue')
        #画障碍物
        param = self.get_param_obs_concave()
        x = param[0]
        y = param[1]
        h = param[2]
        w = param[3]
        t = param[4]
        verts = [
            (x, y),
            (x, y+h),
            (x+t, y+h),
            (x+t, y+t),
            (x+w-t, y+t),
            (x+w-t, y+h),
            (x+w, y+h),
            (x+w, y),
            (x, y)
        ]

        codes = [
            Path.MOVETO,
            Path.LINETO,
            Path.LINETO,
            Path.LINETO,
            Path.LINETO,
            Path.LINETO,
            Path.LINETO,
            Path.LINETO,
            Path.CLOSEPOLY,
        ]

        path = Path(verts, codes)
        patch = patches.PathPatch(path, facecolor='cyan', lw=2)
        ax.add_patch(patch)
        #画覆盖区域
        pcm1 = plt.pcolormesh(self.coverage_map_time[frame_slot],
                            cmap= cmap1, rasterized= True, vmin= 0, vmax= 1,
                            alpha= 0.5)
        # 画信息素浓度
        pcm2 = plt.pcolormesh(self.density_of_phe_time[frame_slot],
                             cmap= cmap2, vmin= 0, vmax= 10,
                             alpha= 0.5)
        fig.colorbar(pcm2, ax= ax, extend= 'max')
        plt.autoscale(enable= True)
        ax.set_xlim(0, 100)
        ax.set_ylim(0, 100)
        ax.set_title('t = {:.2}'.format(frame_slot/100))

    def cal_pos_of_obs(self, obstacle_type):
        """此函数用于根据__init__()函数中的obstacle_type来初始化搜索区域中障碍物的位置
            input: obstacle_type
            output:a list containing ndarray representing positions of the 'obstacle_type'
                   and a number representing the area of the obstacle"""
        #如果obstacle_type是no_obstacle的话，就返回None
        if obstacle_type == 'no_obstacle':
            return None
        elif obstacle_type == 'concave':
            """    ___2      ___6  
                   | |       | |    (x,y) is the coordinates of the low-left point of the obstacle
                  1| |3  4  5| |h   h is the total height of the obstacle
                   |t|_______| |7   w is the total width of the obstacle
            (x, y) |___________|    t is the thickness of the obstacle
                       8 w
            """
            #首先得到表示障碍物的一组点的坐标
            #获取concave障碍物的相关参数
            param = self.get_param_obs_concave()
            x = param[0]
            y = param[1]
            h = param[2]
            w = param[3]
            t = param[4]
            coor_obs = self.cal_coor_obs_concave(x, y, h, w, t)
            return coor_obs
        #后续可以加上其他的情况

    def cal_coor_obs_concave(self, x, y, h, w, t):
        """calculate the position of the concave obstacle
            input: x, y, h, w, t
            output: an ndarray reps the position of the concave obstacle"""
        pos1 = np.zeros((h, 2))
        pos2 = np.zeros((t, 2))
        pos3 = np.zeros((h-t, 2))
        pos4 = np.zeros((w-2*t, 2))
        pos5 = np.zeros((h-t, 2))
        pos6 = np.zeros((t, 2))
        pos7 = np.zeros((h, 2))
        pos8 = np.zeros((w, 2))
        pos1[:,0] = x
        pos1[:,1] = np.linspace(y, y+h, h, endpoint=False)
        pos2[:,1] = y+h
        pos2[:,0] = np.linspace(x, x+t,t, endpoint=False)
        pos3[:,0] = x+t
        pos3[:,1] = np.linspace(y+h, y+t, h-t, endpoint=False)
        pos4[:,1] = y+t
        pos4[:,0] = np.linspace(x+t, x+w-t, w-2*t, endpoint=False)
        pos5[:,0] = x+w-t
        pos5[:,1] = np.linspace(y+t, y+h, h-t, endpoint=False)
        pos6[:,1] = y+h
        pos6[:,0] = np.linspace(x+w-t, x+w, t, endpoint=False)
        pos7[:,0] = x+w
        pos7[:,1] = np.linspace(y+h, y, h, endpoint=False)
        pos8[:,1] = y
        pos8[:,0] = np.linspace(x+w, x, w,endpoint=False)
        pos = np.append(pos1, pos2)
        pos = np.append(pos, pos3)
        pos = np.append(pos, pos4)
        pos = np.append(pos, pos5)
        pos = np.append(pos, pos6)
        pos = np.append(pos, pos7)
        pos = np.append(pos, pos8)
        pos = pos.reshape((len(pos1)+len(pos2)+len(pos3)+len(pos4)+len(pos5)+len(pos6)+len(pos7)+len(pos8), 2))
        return pos

    def cal_obs_area(self, obs_type):
        """计算障碍物的占地面积"""
        if obs_type == 'no_obstacle':
            return 0
        elif obs_type == 'concave':
            param = self.get_param_obs_concave()
            # x = param[0]
            # y = param[1]
            h = param[2]
            w = param[3]
            t = param[4]
            return -2*t*t+t*(2*h+w)


    def get_param_obs_concave(self):
        #return[        x,                         y,            h, w, t]
        return [self.area_width/2-30/2, self.area_width/2-20/2, 20, 30, 5]


    def ini_pos_of_uavs(self, obstacle_type):
        pos_of_uavs = np.random.random((self.number_of_uavs, 2)) * (self.area_width - 0.1)
        #对于pos_of_uavs中每一个无人机的初始位置都要检查，如果再obs的范围内的话，需要
        #重新生成这个无人机的坐标，直到不在障碍物的区域内为止
        for i in range(self.number_of_uavs):
            if self.in_obs(pos_of_uavs[i], obstacle_type):
                pos_of_uavs[i] = self.step_out_of_obs(obstacle_type)
        return pos_of_uavs

    def step_out_of_obs(self, obstype):
        """对于在障碍物里面的无人机的坐标，重新计算随机坐标值，直到得到的新坐标不在障碍物里面"""
        if obstype == 'concave':
            new_pos = np.zeros((2,))
            while True:
                new_pos = np.random.random((2,)) * (self.area_width -0.1)
                if not self.in_obs(new_pos, obstype):
                    break
            return new_pos
        #还可以进行拓展

    def in_obs(self, p, obstype):
        if obstype == 'no_obstacle':
            return False
        elif obstype == 'concave':
            param = self.get_param_obs_concave()
            x = param[0]
            y = param[1]
            h = param[2]
            w = param[3]
            t = param[4]
            #开始判断
            if (x <= p[0] <= x+t and y <= p[1] <= y+h )\
                    or (x+t <= p[0] <= x+w-t and y <= p[1] <= y+t)\
                    or(x+w-t <= p[0] <= x+w and y <= p[1] <= y+h):
                return True
            else:
                return  False
        #还可以加上其他情况

    def ini_vel_of_uavs(self):
        vel_norm = np.random.random((self.number_of_uavs, 1)) * self.max_vel
        vel_dir_unit = np.exp(np.random.random((self.number_of_uavs, 1)) * np.pi*2 * 1j)
        vel_exp = vel_dir_unit * vel_norm
        vel_of_uavs = np.zeros((self.number_of_uavs, 2))
        for i in range(self.number_of_uavs):
            vel_of_uavs[i] = np.array([vel_exp[i][0].real, vel_exp[i][0].imag])
            #print(np.linalg.norm(vel_of_uavs[i]))
        return vel_of_uavs

    def check_vel_of_uavs(self, v):
        v = v
        for i in range(self.number_of_uavs):
            v[i] = (v[i]/np.linalg.norm(v[i]) * min(self.max_vel, np.linalg.norm(v[i])))
        return v

    def cal_coverage_per(self):
        for u in range(self.number_of_uavs):
            for i in range(self.area_width):
                for j in range(self.area_width):
                    x = j + 0.5
                    y = i + 0.5
                    if np.linalg.norm(np.array([x, y]) - self.posotion_of_uavs[u]) <= self.sense_range \
                            and self.coverage_map[i][j] != 1\
                            and not self.in_obs(np.array([x, y]), self.obs_type):
                        self.coverage_map[i][j] = 1
        n = 0
        for i in range(self.area_width):
            for j in range(self.area_width):
                if self.coverage_map[i][j] == 1:
                    n+=1
        coverage_per = n / (np.power(self.area_width, 2) - self.obs_area)
        return coverage_per

    def update_phe_map(self, t):
        if t <= self.alpha_time:
            return self.phe_map
        else:
            return self.release_and_evaporate_phe()

    def release_and_evaporate_phe(self):
        #如果到了该喷洒的时间，在self.phe_release_range范围内喷洒信息素
        #if self.time_to_release(t):
        for u in range(self.number_of_uavs):
            for i in range(self.area_width):
                for j in range(self.area_width):
                    #首先进行坐标的转化
                    x = j+0.5
                    y = i+0.5
                    if np.linalg.norm(np.array([x,y])-self.posotion_of_uavs[u])<=\
                            self.phe_release_range:
                        self.phe_map[i][j] += self.ini_phe
        #每一步都需要蒸发，并且将负值置0
        self.phe_map -= self.vel_of_phe_evap
        self.phe_map[self.phe_map<0] = 0
        return self.phe_map

    # def time_to_release(self, t):
    #     if (self.step(t) - self.step(self.alpha_time)) % (self.step(self.phe_step)) == 0:
    #         return True
    #     return  False

    def step(self, t):
        #print(type(t))
        return int(t/self.time_step)

    def accel_obs(self):
        """这个函数用来计算障碍物对无人机的加速度的作用"""
        accel_obs = np.zeros((self.number_of_uavs, 2))
        if self.obs_type == 'no_obstacle':
            return accel_obs
        elif self.obs_type == 'concave':
            #如果是concave情况的话，需要逐个计算每一个无人机受到的加速度
            for i in range(self.number_of_uavs):
                neighbors = self.find_my_obs_neighbors(i, self.sense_range)
                accel_obs[i] = self.obs_bounce_accel_cal(i, neighbors)
            return accel_obs

    def accel_phe(self, t):
        accel = np.zeros((self.number_of_uavs, 2))
        if t <= self.alpha_time:
            return accel
        else:
            accel = self.cal_accel_phe()
            #####################################################################
            return accel

    # def cal_accel_phe(self):
    #     accel = np.zeros((self.number_of_uavs, 2))
    #     for u in range(self.number_of_uavs):
    #         min = np.power(10, 5)
    #         to_x = 0
    #         to_y = 0
    #         for i in range(self.area_width):
    #             for j in range(self.area_width):
    #                 x = j+0.5
    #                 y = i+0.5
    #                 if self.min_phe_detect_range<=\
    #                         np.linalg.norm(np.array([x,y])-self.posotion_of_uavs[u])\
    #                         <=self.max_phe_detect_range \
    #                         and self.phe_map[i][j]<min :
    #                     min = self.phe_map[i][j]
    #                     to_x = x
    #                     to_y = y
    #         accel[u] = self.phe_attr(to_x, to_y, u) * \
    #             (np.array([to_x, to_y]) - self.posotion_of_uavs[u])/np.linalg.norm(
    #             np.array([to_x, to_y]) - self.posotion_of_uavs[u])
    #     return accel

    def cal_accel_phe(self):
        accel = np.zeros((self.number_of_uavs, 2))
        #对于集群中的每一个无人机
        for u in range(self.number_of_uavs):
            #根据平均信息素浓度最低的扇形大区来确定无人机的加速度方向和大小；
            accel[u] = self.phe_attr(self.cal_avg_dense_of_six(self.posotion_of_uavs[u]), self.posotion_of_uavs[u])
        return  accel


    def cal_avg_dense_of_six(self, p):
        dense_clasify = [[]] * 6 #记录每个大区信息素的浓度，每个大区的浓度为一个list
        min_dense_clasify = [100000] * 6 #记录每个大区信息素的最低浓度
        min_dense_coord_clasify = [[]] * 6 #记录每个大区信息素浓度最低值所对应的坐标
        coord_list = np.array([]) #所有满足距离条件的patch的坐标的数组（xy坐标）
        dense_list = np.array([]) #所有满足距离条件的patch的坐标的数组（xy坐标）
        dense_clasify_dict = {
            0:dense_clasify[0],
            1:dense_clasify[1],
            2:dense_clasify[2],
            -3:dense_clasify[3],
            -2:dense_clasify[4],
            -1:dense_clasify[5],
        }
        min_dense_clasify_dict = {
            0: min_dense_clasify[0],
            1: min_dense_clasify[1],
            2: min_dense_clasify[2],
            -3: min_dense_clasify[3],
            -2: min_dense_clasify[4],
            -1: min_dense_clasify[5],
        }
        min_dense_coord_clasify_dict = {
            0: min_dense_coord_clasify[0],
            1: min_dense_coord_clasify[1],
            2: min_dense_coord_clasify[2],
            -3: min_dense_coord_clasify[3],
            -2: min_dense_coord_clasify[4],
            -1: min_dense_coord_clasify[5],
        }
        #n = 0
        for i in range(self.area_width):
            for j in range(self.area_width):
                x = j+ 0.5
                y = i+ 0.5
                if self.min_phe_detect_range <= \
                        np.linalg.norm(np.array([x,y])-p) <=self.max_phe_detect_range:
                    coord_list = np.append(coord_list, np.array([x, y]))
                    dense_list = np.append(dense_list, self.phe_map[i, j])
                    #n += 1
        # print(n)
        coord_list = coord_list.reshape((int(coord_list.size/2), 2))
        dense_list = dense_list.reshape((dense_list.size,))
        theta_array = np.rad2deg(np.arctan2(coord_list[:,1], coord_list[:,0]))
        for i in range(theta_array.shape[0]):
            a = theta_array[i]//60
            dense_clasify_dict[a].append(dense_list[i])
            #接下来处理最小值
            if dense_list[i] < min_dense_clasify_dict[a]:
                min_dense_clasify_dict[a] = dense_list[i]
                min_dense_coord_clasify_dict[a][:] = []
                min_dense_coord_clasify_dict[a].append(coord_list[i])
            elif dense_list[i] == min_dense_clasify_dict[a]:
                min_dense_coord_clasify_dict[a].append(coord_list[i])
        avg_list = list(map(np.mean, dense_clasify))
        #得到平均浓度最低的大区的序号
        area_num = avg_list.index(min(avg_list))
        #计算这个大区中的信息素浓度最低的patches的平均坐标,得到一个数组
        to_coord = np.array([np.mean(np.array(min_dense_coord_clasify[area_num])[:, 0]), \
                    np.mean(np.array(min_dense_coord_clasify[area_num])[:, 1])])
        return to_coord

    def phe_attr(self,to_coord, position_of_current_uav): ###可以修改
        accel = 10 * \
                (to_coord - position_of_current_uav)/np.linalg.norm(to_coord - position_of_current_uav)
        return accel

    def accel_boundary_cal(self):
        accel = np.zeros((self.number_of_uavs, 2))
        for i in range(self.number_of_uavs):
            accel[i] = self.boundary_function(i)
        return accel

    def boundary_function(self, i):
        accel = np.array([0, 0])
        pairs1 = [(self.posotion_of_uavs[i][0], 1),
                  (self.area_width-self.posotion_of_uavs[i][0], -1)]
        pairs2 = [(self.posotion_of_uavs[i][1], 1),
                  (self.area_width-self.posotion_of_uavs[i][1], -1)]
        pairs1.sort(key=lambda pair: pair[0])
        pairs2.sort(key=lambda pair: pair[0])
        pairh = pairs1[0]
        pairv = pairs2[0]
        accel = np.array([1,0]) * pairh[1] * self.boundary_potential_function(pairh[0]) +\
            np.array([0,1]) * pairv[1] * self.boundary_potential_function(pairv[0])
        return accel

    def boundary_potential_function(self, d):
        accel = 0
        if d <= self.sense_range:
            accel = 100 / (np.power(d, 2) + 1/np.power(10,5))
        return  accel

    def bounce(self):
        accel = np.zeros((self.number_of_uavs, 2))
        for i in range(self.number_of_uavs):
            neighbors = self.find_my_neighbors(i, self.sense_range)
            accel[i] = self.bounce_accel_cal(i, neighbors)
        return accel

    def bounce_accel_cal(self, i, neighbors):
        accel = np.array([0., 0.])
        for j in neighbors:
            accel += (self.posotion_of_uavs[i]-self.posotion_of_uavs[j])/np.linalg.norm(
                self.posotion_of_uavs[i]-self.posotion_of_uavs[j]) * \
                self.acceleration_potential_func(np.linalg.norm(
                    self.posotion_of_uavs[i] - self.posotion_of_uavs[j]))
        return accel

    def obs_bounce_accel_cal(self, i, neighbors):
        accel = np.array([0., 0.])
        for j in neighbors:
            accel += (self.posotion_of_uavs[i] - self.position_of_obs[j]) / np.linalg.norm(
                self.posotion_of_uavs[i] - self.position_of_obs[j]) * \
                     self.acceleration_potential_func(np.linalg.norm(
                         self.posotion_of_uavs[i] - self.position_of_obs[j]))
        return accel


    def acceleration_potential_func(self, distance):
        acceleration = 0
        if distance <= self.sense_range:
            acceleration = ((self.sense_range - distance) / (distance + 1 / np.power(10, 5))) * 10
        return acceleration

    def fick(self):
        #print('fick yes')
        #最后需要返回的量：N_uavs*2 accel
        accel = np.zeros((self.number_of_uavs, 2))
        for i in range(self.number_of_uavs):
            neighbors = self.find_my_neighbors(
                i, self.dense_sense_range)#首先是找邻居，返回一个包含所有邻居id的list
            #print('{} neighbors:'.format(i), neighbors)
            num_up_down_left_right = self.find_concentration_diff(
                i, neighbors)#找到上下左右的无人机数量
            #print('{} num_udlr:'.format(i), num_up_down_left_right)
            accel[i] = fick_accel_cal(num_up_down_left_right)
        return accel


    def mixed(self):
        accel = self.bounce() + self.fick()
        return accel

    def stochastic(self):
        accel = np.zeros((self.number_of_uavs, 2))
        return accel

    def find_my_neighbors(self, i, r):
        neighbors = [j for j in range(self.number_of_uavs)\
                     if j!=i and \
                     np.linalg.norm(self.posotion_of_uavs[i]-self.posotion_of_uavs[j]) <= r ]
        return neighbors

    def find_my_obs_neighbors(self, i, r):
        neighbors = [j for j in range(len(self.position_of_obs)) \
                     if np.linalg.norm(self.posotion_of_uavs[i] - self.position_of_obs[j]) <= r]
        return neighbors

    def find_concentration_diff(self, i, neighbors):
        num_neighbors = len(neighbors)
        uav_id_left = [j for j in neighbors if \
                       self.posotion_of_uavs[j][0] <= self.posotion_of_uavs[i][0]]
        uav_id_down = [j for j in neighbors if \
                       self.posotion_of_uavs[j][1] <= self.posotion_of_uavs[i][1]]

        num_down = len(uav_id_down)
        num_up = num_neighbors - num_down
        num_left = len(uav_id_left)
        num_right = num_neighbors - num_left
        num_up_down_left_right = [num_up, num_down, num_left, num_right]
        return num_up_down_left_right

    def find_alpha_time(self,):
        alpha = 0.7
        alpha_time = np.power(self.area_width, 2) / (self.number_of_uavs *
                                                     np.pi * np.power(self.sense_range, 2)) * alpha
        return alpha_time
##############################################################################################
##############################################################################################

def excecute(i, strategy, sd, phr,er, phe, coverage_of_scene_method_of_time):
    scenary_name = 'v2_concave_' + sd[i] + '_' + str(phr[i]) + '_' + str(er[i])
    # j_file_name = 'improved_no_obstacle_' + scenary_name + '.json'
    mp4_file_name = scenary_name + '.mp4'
    print('The scenary of {} will be running......'.format(scenary_name))
    #确定两个colormaps:
    #无人机数量
    N = 20
    end_time = 30
    coverage = uav_coverage(number_of_uavs=N, end_time=end_time,
                            ini_phe= 2, vel_of_phe_evap= er[i],
                            max_phe_detect_range= phr[i],
                            min_phe_detect_range=1, pheromone= phe[i],
                            obstacle_type='concave',coverage_strategy= strategy[i])
    frames = end_time / coverage.time_step
    coverage.coverage()
    # 开始写入文件
    coverage_of_scene_method_of_time.append(coverage.coverage_per_time)
    print('The whole coverage percentage of time of scenary {} is here:'.format(scenary_name))  ##################
    print(coverage.coverage_per_time)
    print('The scenary of {} has done.'.format(scenary_name))
    if i == 1:
        with open('coverage_data/v2_concave_stochastic_phe_and_nophe.json', 'w') as f:#############
            json.dump(coverage_of_scene_method_of_time, f)
            print('coverage percentage of all the scenary is loaded completed......')##############

        with open('coverage_data/time.json', 'w') as f:
            json.dump(coverage.time_slot, f)
            print('timeline loading completed......')

    ani = animation.FuncAnimation(
        fig, coverage.plot, int(frames), interval= coverage.time_step*1000,
        save_count= 3000
    )

    plt.rcParams['animation.ffmpeg_path'] = 'D:\Program Files\\ffmpeg\\bin\\ffmpeg.exe'
    writer = FFMpegWriter(fps=15, metadata=dict(artist='Me'), bitrate=1800)
    ani.save("outcome/"+mp4_file_name, writer=writer)################
    print('The mp4 of scenary {} is saved.'.format(scenary_name),'\n')
    #
    # plt.show()

def main():
    coverage_of_scene_method_of_time = []
    strategy_dict = {
        0: 'stochastic_phe_concave',
        1: 'stochastic_nophe_concave',
    }
    strategy = {
        0 : 3,
        1 : 3,
    }
    phe_max_range = {
        0: None,
        1: None,
    }
    evap_rate = {
        0: 2/10,
        1: 2/10,
    }
    pheromone = {
        0 : True,
        1 : False,
    }
    for i in range(2):
        excecute(i, strategy, strategy_dict, phe_max_range,evap_rate,pheromone,coverage_of_scene_method_of_time)
    # excecute(0, strategy, strategy_dict, phe_max_range, evap_rate, coverage_of_scene_method_of_time)
    print('All is done.')


if __name__ == "__main__":
    main()
