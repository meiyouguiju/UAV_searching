import UAV_class_4 as uav
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import CalCoverageArea as cca
from matplotlib.animation import FFMpegWriter

def search():
    #x_coord, y_coord = get_initial_coordinates()
    #x_vel, y_vel = get_initial_velocity()
    for j in range(1, len(t)):
        for i in range(n_particles):
            uav_all[i].calculate_location_for_next_step()
            x_coord[i] = uav_all[i].x_coord
            y_coord[i] = uav_all[i].y_coord
            #每一轮迭代中，每一架无人机都要讲自己的位置信息发送给建立的map来完善此轮迭代中的覆盖面积计算
            map.cal_coverage_area(x_coord[i], y_coord[i], box_width)
        cov_percen = map.searched_percen
        time = j * 0.01
        print('time:{}, searched area:{},   coverage percentage:{}'.format(time, map.searched_area, cov_percen))
        # print(map.searched_area)
        # print(cov_percen)
        #yield x_coord, y_coord, cov_percen, time
        yield cov_percen, time

def animate(data):
    #x_coord, y_coord, cov_percen, time = data
    cov_percen, time = data
    # ax.cla()
    # ax.set_xlim(0, box_width)
    # ax.set_ylim(0, box_width)
    # xdata.append(x_coord)
    # ydata.append(y_coord)
    # ax.scatter(x_coord, y_coord)

    ax2.plot(time, cov_percen, **{'marker':'o'})
    return  ax2

def init():
    # ax.set_xlim(0, 10)
    # ax.set_ylim(0, 10)
    #ax.scatter(x_coord, y_coord)
    ax2.plot(0, 0)
    return ax2

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
fig2 = plt.figure()
ax2 = fig2.add_subplot(111, autoscale_on=True)
ax2.grid()

xdata, ydata = [], []

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


ani = animation.FuncAnimation(fig2, animate, search, interval=dt*1000, blit=False, init_func=init, repeat=False)

# plt.rcParams['animation.ffmpeg_path'] = 'C:\Program Files\\ffmpeg\\bin\\ffmpeg.exe'
# writer = FFMpegWriter(fps=15, metadata=dict(artist='Me'), bitrate=1800)
# ani.save("movie.mp4", writer=writer)

plt.show()