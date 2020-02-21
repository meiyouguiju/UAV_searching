import UAV_class as uav
import numpy as np


def distance(patch, uav_x, uav_y):
    distance = np.power(
        np.power((patch.x_coord - uav_x), 2) + np.power((patch.y_coord - uav_y), 2), 1 / 2)
    return distance

#借用netlogo的思维，将地图分为许多小方格，每一个小方格为一个patch
class Patch:
    area = 1
    def __init__(self, x, y):
        self.x_coord = x
        self.y_coord = y
        self.is_searched = False
        self.patch_id = 0

#建立Map类，Map控制着总共搜索的面积
class Map:
    def __init__(self, box_width):
        #获得包含地图中所有patch的列表patch_list,此列表为一个一维列表
        self.patch_list = self.get_patch_list(box_width)
        self.map_color_list = self.get_map_color_list(box_width)
        self.searched_area = 0
        self.total_area = box_width * box_width * Patch.area
        self.searched_percen = 0

    def get_map_color_list(self, box_width):
        map_color_list = np.zeros((box_width, box_width))
        return map_color_list

    #首先定义获取patch_list的函数
    def get_patch_list(self, box_width):
        patch_list = []
        for i in range(box_width): #i与patch的横坐标相关
            for j in range(box_width): #j与patch的纵坐标相关
                patch = Patch(i+0.5, j+0.5) #新建立patch的横纵坐标
                patch.patch_id = i * box_width + j +1 #获得每一个patch的id
                patch_list.append(patch)
        return patch_list

    #定义计算已搜寻的场地面积的函数，接受无人机的位置信息作为计算的依据。
    def cal_coverage_area(self, uav_x, uav_y, box_width):
        #根据接受的无人机的坐标信息，将距离此坐标的距离不大于recognition_radius的patch标记为已搜索过的，如果已经标记过则不重复标记。
        for patch in self.patch_list:
            if distance(patch, uav_x, uav_y) <= uav.Uav.recognition_radius:
                #如果此patch没有被搜索过则将其标记为已经搜索过,并更新Map对象的seatched_area和searched_percen属性
                if patch.is_searched == False:
                    patch.is_searched = True
                    self.map_color_list[np.int(box_width-patch.y_coord-0.4)][np.int(patch.x_coord-0.4)] = 1
                    self.searched_area = self.searched_area + Patch.area
                    self.searched_percen = self.searched_area / self.total_area


