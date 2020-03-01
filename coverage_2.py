#-*- coding:utf-8 -*-

import UAV_class_5 as uav
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import matplotlib as mpl
import CalCoverageArea as cca
from matplotlib.animation import FFMpegWriter

def search():
    #x_coord, y_coord = get_initial_coordinates()
    #x_vel, y_vel = get_initial_velocity()
    print(len(t))
    for j in range(1, len(t)):
        for i in range(n_particles):
            uav_all[i].calculate_location_for_next_step()
            x_coord[i] = uav_all[i].x_coord
            y_coord[i] = uav_all[i].y_coord
            #每一轮迭代中，每一架无人机都要讲自己的位置信息发送给建立的map来完善此轮迭代中的覆盖面积计算
            #map.cal_coverage_area(x_coord[i], y_coord[i], box_width)
        #cov_percen = map.searched_percen
        #color_map = map.map_color_list
        time = j * 0.01
        print('time:{}, searched area:{},   coverage percentage:{}'.\
                format(time, map.searched_area, map.searched_percen))
        yield x_coord, y_coord, time

def animate(data):
    #x_coord, y_coord, cov_percen, time = data
    x_coord, y_coord, time = data
    ax.set_xlim(0, 100)
    ax.set_ylim(0, 100)
    #不清除
    #然后画这一次的覆盖区域
    draw_coverage_now(ax, x_coord, y_coord)
    #然后画这一次的无人机位置
    draw_uavs_position(ax, x_coord, y_coord)
    #最后画障碍物的位置
    draw_obstacles(ax, 'city')
    ax.set_title('t = {:.2}'.format(time))
    return  ax

def init():
    # ax.set_xlim(0, 10)
    # ax.set_ylim(0, 10)
    #ax.scatter(x_coord, y_coord)
    draw_obstacles(ax, scene = 'city')
    return ax

def draw_obstacles(ax, scene):
    if scene == 'city':
        x = [30, 40, 40, 50, 50, 40, 40, 30, 30, 20, 20, 30]
        y = [80, 80, 70, 70, 60, 60, 50, 50, 60, 60, 70, 70]
        ax.fill(x, y, facecolor = 'grey')
        return  ax
    #for more classes of obstacles


def draw_coverage_now(ax, x_coord, y_coord):
    for i in range(0, len(x_coord)):
        theta = np.arange(0, 2*np.pi, 2*np.pi/100)
        complex_vector = np.exp(theta*1j)
        x = (complex_vector * uav.Uav.recognition_radius).real + x_coord[i]
        y = (complex_vector * uav.Uav.recognition_radius).imag + y_coord[i]
        ax.fill(x, y, facecolor = 'green')
    return ax

def draw_uavs_position(ax, x_coord, y_coord):
    for i in range(0, len(x_coord)):
        theta = np.arange(0, 2*np.pi, 2*np.pi/10)
        complex_vector = np.exp(theta*1j)
        x = (complex_vector * 0.5).real + x_coord[i]
        y = (complex_vector * 0.5).imag + y_coord[i]
        ax.fill(x, y, facecolor = 'red')
    return ax

######################################################################################
#整体的设置
n_particles = 20
box_width = 100
n_steps = 5000
dt = 0.01
t = np.arange(0, 50, dt)

#无人机搜索动画的设置
# fig = plt.figure()
# ax = fig.add_subplot(111, autoscale_on=False, xlim=(0, box_width), ylim=(0, box_width))
# ax.set_aspect('equal')
# ax.grid()

#搜索区域百分比动画的设置
fig = plt.figure()
ax = fig.add_subplot(111, autoscale_on=True)
ax.grid()

x_coord_last = []
y_coord_last = []

#创建出一定数量的无人机对象
#存放所有无人机的列表uav_all
uav_all = []
for i in range(n_particles):
    uav_new = uav.Uav(box_width, dt)
    uav_all.append(uav_new)

#为每一个无人机设置其other_uavs列表
for i in range(n_particles):
    uav_all[i].other_uavs = uav_all[:i] + uav_all[i+1:]

#得到x_coord和y_coord
x_coord = []
y_coord = []
for i in range(n_particles):
    x_coord.append(uav_all[i].x_coord)
    y_coord.append(uav_all[i].y_coord)

#创建新的Map对象
map = cca.Map(box_width)
#初始化覆盖百分比的列表
cov_percen = 0
#总时间
time = 0


ani = animation.FuncAnimation(fig, animate, search, save_count=1500, interval=dt*1000, blit=False, init_func=init, repeat=False)

plt.rcParams['animation.ffmpeg_path'] = 'C:\Program Files\\ffmpeg\\bin\\ffmpeg.exe'
writer = FFMpegWriter(fps=15, metadata=dict(artist='Me'), bitrate=1800)
ani.save("movie_revised.mp4", writer=writer)

# plt.show()