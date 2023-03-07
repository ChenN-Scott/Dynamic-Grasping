import cv2
import math
import time
import pybullet as p
import numpy as np
import scipy.io as scio
import os
from datasets import Datasets

# 图像尺寸
IMAGEWIDTH = 1280
IMAGEHEIGHT = 960
nearPlane = 0.01
farPlane = 10
aspect = IMAGEWIDTH / IMAGEHEIGHT

def radians_TO_angle(radians):
    """
    弧度转角度
    """
    return 180 * radians / math.pi

def angle_TO_radians(angle):
    """
    角度转弧度
    """
    return math.pi * angle / 180

class Camera:
    def __init__(self, bullet_client, kinect_path, pose_point, target_point, head_point):
        """
        初始化相机参数，计算相机内参
        """
        self.kinect_path = kinect_path
        self.p = bullet_client
        # 计算相机内参
        self.fov = 60   # 垂直视场
        A = IMAGEHEIGHT / (2 * math.tan(math.pi * self.fov/2 / 180))
        self.InMatrix = np.array([[A, 0, IMAGEWIDTH/2 - 0.5], [0, A, IMAGEHEIGHT/2 - 0.5], [0, 0, 1]], dtype=float)

        # 计算相机外参
        self.pose_point = pose_point
        self.target_point = target_point
        self.head_point = head_point
        self.ExMatrix = self.Cal_OutMatrix(self.pose_point, self.target_point, self.head_point)

        # 计算相机参数
        self.viewMatrix = self.p.computeViewMatrix(self.pose_point, self.target_point, self.head_point)
        self.projectionMatrix = self.p.computeProjectionMatrixFOV(self.fov, aspect, nearPlane, farPlane)

        self.datasets = Datasets(self.kinect_path)
        self.img_list = []
        self.pos_list = []

    def Cal_OutMatrix(self, pose, point, head):
        point = [point[i] - pose[i] for i in range(len(point))]
        head = [head[i] - pose[i] for i in range(len(head))]
        temp1 = self.Determinant_2(head[2],head[1],point[2],point[1])
        temp2 = self.Determinant_2(head[0],head[2],point[0],point[2])
        temp3 = self.Determinant_2(head[1],head[0],point[1],point[0])

        judge = math.sqrt(math.pow(temp1,2)+math.pow(temp2,2)+math.pow(temp3,2))
        [a1, a2, a3] = [1/judge * i for i in [temp1, temp2, temp3]]

        judge = math.sqrt(head[0]*head[0]+head[1]*head[1]+head[2]*head[2])
        [b1, b2, b3] = [1/judge * i for i in [-head[0], -head[1], -head[2]]]

        judge = math.sqrt(point[0]*point[0]+point[1]*point[1]+point[2]*point[2])
        [c1, c2, c3] = [1/judge * i for i in [point[0], point[1], point[2]]]
        return np.array(([a1,a2,a3,pose[0]],[b1,b2,b3,pose[1]],[c1,c2,c3,pose[2]],[0,0,0,1]),dtype=float)

    def Determinant_2(self,a,b,c,d):
        return a*d-b*c

    def img2camera(self, pt, dep):
        """
        获取像素点pt在相机坐标系中的坐标
        pt: [x, y]
        dep: 深度值

        return: [x, y, z]
        """
        pt_in_img = np.array([[pt[0]], [pt[1]], [1]], dtype=np.float)
        ret = np.matmul(np.linalg.inv(self.InMatrix), pt_in_img) * dep
        return list(ret.reshape((3,)))
        # print('坐标 = ', ret)
    
    def camera2img(self, coord):
        """
        将相机坐标系中的点转换至图像
        coord: [x, y, z]

        return: [row, col]
        """
        z = coord[2]
        coord = np.array(coord).reshape((3, 1))
        rc = (np.matmul(self.InMatrix, coord) / z).reshape((3,))

        return list(rc)[:-1]

    def length_TO_pixels(self, l, dep):
        """
        与相机距离为dep的平面上 有条线，长l，获取这条线在图像中的像素长度
        l: m
        dep: m
        """
        return l * self.A / dep
    
    def camera2world(self, coord):
        """
        获取相机坐标系中的点在世界坐标系中的坐标
        corrd: [x, y, z]

        return: [x, y, z]
        """
        coord.append(1.)
        coord = np.array(coord).reshape((4, 1))
        # coord_new = np.matmul(self.transMat, coord).reshape((4,))
        coord_new = np.matmul(self.OutMatrix, coord).reshape((4,1))
        return list(coord_new)[:-1]
    
    def world2camera(self, coord):
        """
        获取世界坐标系中的点在相机坐标系中的坐标
        corrd: [x, y, z]

        return: [x, y, z]
        """
        coord.append(1.)
        coord = np.array(coord).reshape((4, 1))
        coord_new = np.matmul(np.linalg.inv(self.transMat), coord).reshape((4,))
        return list(coord_new)[:-1]

    def world2img(self, coord):
        """
        获取世界坐标系中的点在图像中的坐标
        corrd: [x, y, z]

        return: [row, col]
        """
        # 转到相机坐标系
        coord = self.world2camera(coord)
        # 转到图像
        pt = self.camera2img(coord) # [y, x]
        return [int(pt[1]), int(pt[0])]

    def img2world(self, pt, dep):
        """
        获取像素点的世界坐标
        pt: [x, y]
        dep: 深度值 m
        """
        coordInCamera = self.img2camera(pt, dep)
        return self.camera2world(coordInCamera)
    
    def reset(self, pose, target, head):
        # 重新计算相机外参
        self.pose_point = pose
        self.target_point = target
        self.head_point = head
        self.ExMatrix = self.getTransMat(self.pose_point, self.target_point, self.head_point)
        self.viewMatrix = self.p.computeViewMatrix(self.pose_point, self.target_point, self.head_point)

    def save_Extrinsics(self):
        """
        保存相机外参
        """
        np.save(os.path.join(self.kinect_path,'Extrinsics.npy'),np.array(self.ExMatrix))

    def save_Intrinsics(self):
        """ 
        保存相机内参
        """
        np.save(os.path.join(self.kinect_path,'Intrinsics.npy'),np.array(self.InMatrix))

    def render_Image(self, py, target_id):
        img = py.getCameraImage(IMAGEWIDTH, IMAGEHEIGHT, self.viewMatrix, self.projectionMatrix, renderer=p.ER_BULLET_HARDWARE_OPENGL)
        target_pose, target_orn = p.getBasePositionAndOrientation(target_id)
        self.img_list.append(img)
        self.pos_list.append([target_pose, target_orn])

    def save_Image(self, object_name):
        id = 0
        for img in self.img_list:
            w = img[0]      # width of the image, in pixels
            h = img[1]      # height of the image, in pixels
            rgba = img[2]    # color data RGB
            dep = img[3]    # depth data

            # 获取彩色图像
            im_rgb = np.reshape(rgba, (h, w, 4))[:, :, [2, 1, 0]]
            im_rgb = im_rgb.astype(np.uint8)

            # 获取深度图像
            depth = np.reshape(dep, (h, w))  # [40:440, 120:520]
            A = np.ones((IMAGEHEIGHT, IMAGEWIDTH), dtype=np.float64) * farPlane * nearPlane
            B = np.ones((IMAGEHEIGHT, IMAGEWIDTH), dtype=np.float64) * farPlane
            C = np.ones((IMAGEHEIGHT, IMAGEWIDTH), dtype=np.float64) * (farPlane - nearPlane)
            im_depthCamera = np.divide(A, (np.subtract(B, np.multiply(C, depth))))  # 单位 m

            # 保存图像
            self.datasets.save_rgb(id, im_rgb)
            self.datasets.save_depth(id, im_depthCamera)

        id = 0
        for pos in self.pos_list:
            self.datasets.save_annotation(id, object_name, pos[0], pos[1])